//! Helpers shared by the integration tests.
//!
//! Everything here goes through the public register interface of [`Soc`], the
//! same path firmware uses, so the tests double as executable documentation of
//! the register sequences the HAL performs.

#![allow(dead_code)]

use std::cell::RefCell;
use std::rc::Rc;

use gsim_core::{AccessWidth, Event, MemoryBus};
use gsim_stm32f1::{map, Soc};

pub fn w8(soc: &mut Soc, addr: u32, value: u8) {
    soc.write(addr, AccessWidth::Byte, u32::from(value))
        .unwrap_or_else(|error| panic!("write8 0x{addr:08X}: {error}"));
}

pub fn w16(soc: &mut Soc, addr: u32, value: u16) {
    soc.write(addr, AccessWidth::Half, u32::from(value))
        .unwrap_or_else(|error| panic!("write16 0x{addr:08X}: {error}"));
}

pub fn w32(soc: &mut Soc, addr: u32, value: u32) {
    soc.write(addr, AccessWidth::Word, value)
        .unwrap_or_else(|error| panic!("write32 0x{addr:08X}: {error}"));
}

pub fn r8(soc: &mut Soc, addr: u32) -> u8 {
    soc.read(addr, AccessWidth::Byte).unwrap() as u8
}

pub fn r16(soc: &mut Soc, addr: u32) -> u16 {
    soc.read(addr, AccessWidth::Half).unwrap() as u16
}

pub fn r32(soc: &mut Soc, addr: u32) -> u32 {
    soc.read(addr, AccessWidth::Word).unwrap()
}

/// Polls `register & bit` until it reaches `expected`, stepping the simulation.
pub fn wait_for_bit(soc: &mut Soc, addr: u32, bit: u32, expected: bool, timeout_ns: u64) {
    let mut waited = 0;
    while waited < timeout_ns {
        let set = r32(soc, addr) & bit != 0;
        if set == expected {
            return;
        }
        soc.advance_ns(1_000);
        waited += 1_000;
    }
    panic!(
        "bit 0x{bit:08X} of 0x{addr:08X} did not become {expected} within {timeout_ns} ns (value 0x{:08X})",
        r32(soc, addr)
    );
}

/// Sets one pin's `CRL`/`CRH` nibble (`[CNF(2) MODE(2)]`).
pub fn set_pin_nibble(soc: &mut Soc, port: usize, pin: u8, nibble: u8) {
    let base = map::gpio::PORTS[port];
    let register = if pin < 8 {
        map::gpio::CRL
    } else {
        map::gpio::CRH
    };
    let shift = u32::from(pin % 8) * 4;
    let mut value = r32(soc, base + register);
    value = (value & !(0xF << shift)) | (u32::from(nibble) << shift);
    w32(soc, base + register, value);
}

/// Push-pull output at 2 MHz (`CNF = 00`, `MODE = 10`).
pub const PIN_OUTPUT_PP_2MHZ: u8 = 0b0010;
/// Alternate function push-pull output at 50 MHz (`CNF = 10`, `MODE = 11`).
pub const PIN_AF_PP_50MHZ: u8 = 0b1011;
/// Floating input (`CNF = 01`, `MODE = 00`).
pub const PIN_INPUT_FLOATING: u8 = 0b0100;
/// Input with pull-up/pull-down (`CNF = 10`, `MODE = 00`).
pub const PIN_INPUT_PULL: u8 = 0b1000;

/// Enables APB2 clocks (`RCC_APB2ENR`).
pub fn enable_apb2(soc: &mut Soc, mask: u32) {
    let current = r32(soc, map::rcc::BASE + map::rcc::APB2ENR);
    w32(soc, map::rcc::BASE + map::rcc::APB2ENR, current | mask);
}

/// Enables APB1 clocks (`RCC_APB1ENR`).
pub fn enable_apb1(soc: &mut Soc, mask: u32) {
    let current = r32(soc, map::rcc::BASE + map::rcc::APB1ENR);
    w32(soc, map::rcc::BASE + map::rcc::APB1ENR, current | mask);
}

/// Enables AHB clocks (`RCC_AHBENR`).
pub fn enable_ahb(soc: &mut Soc, mask: u32) {
    let current = r32(soc, map::rcc::BASE + map::rcc::AHBENR);
    w32(soc, map::rcc::BASE + map::rcc::AHBENR, current | mask);
}

/// Reproduces what `SystemClock_Config()` does for a 72 MHz HSE/PLL setup:
/// HSE on, PLL ×9, AHB /1, APB1 /2, APB2 /1, ADC /6.
pub fn configure_72mhz(soc: &mut Soc) {
    let rcc = map::rcc::BASE;
    w32(soc, rcc + map::rcc::CR, (1 << 16) | 1); // HSEON | HSION
    wait_for_bit(soc, rcc + map::rcc::CR, 1 << 17, true, 20_000_000);

    let cfgr = (1 << 16)          // PLLSRC = HSE
        | (0b0111 << 18)          // PLLMUL = ×9
        | (0b100 << 8)            // PPRE1 = /2
        | (0b10 << 14); // ADCPRE = /6
    w32(soc, rcc + map::rcc::CFGR, cfgr);

    w32(soc, rcc + map::rcc::CR, (1 << 16) | (1 << 24) | 1); // HSEON | PLLON | HSION
    wait_for_bit(soc, rcc + map::rcc::CR, 1 << 25, true, 2_000_000);

    w32(soc, rcc + map::rcc::CFGR, cfgr | 0b10); // SW = PLL
    wait_for_bit(soc, rcc + map::rcc::CFGR, 0b11 << 2, true, 1_000_000);
    assert_eq!((r32(soc, rcc + map::rcc::CFGR) >> 2) & 0b11, 0b10);
}

/// Collects peripheral events from a [`Soc`].
#[derive(Clone, Default)]
pub struct EventLog(Rc<RefCell<Vec<Event>>>);

impl EventLog {
    pub fn new() -> Self {
        EventLog(Rc::new(RefCell::new(Vec::new())))
    }

    pub fn attach(&self, soc: &mut Soc) {
        let log = Rc::clone(&self.0);
        soc.add_event_sink(Box::new(move |event: &Event| {
            log.borrow_mut().push(event.clone())
        }));
    }

    pub fn events(&self) -> Vec<Event> {
        self.0.borrow().clone()
    }

    pub fn clear(&self) {
        self.0.borrow_mut().clear();
    }

    /// Bytes shifted out by one USART.
    pub fn uart_bytes(&self, unit: u8) -> Vec<u8> {
        self.0
            .borrow()
            .iter()
            .filter_map(|event| match event {
                Event::UartTx { unit: u, byte } if *u == unit => Some(*byte),
                _ => None,
            })
            .collect()
    }

    /// Frames transferred by one SPI.
    pub fn spi_frames(&self, unit: u8) -> Vec<(u16, u16)> {
        self.0
            .borrow()
            .iter()
            .filter_map(|event| match event {
                Event::SpiTransfer { unit: u, tx, rx } if *u == unit => Some((*tx, *rx)),
                _ => None,
            })
            .collect()
    }
}
