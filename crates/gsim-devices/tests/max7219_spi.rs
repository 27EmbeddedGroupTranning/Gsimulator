//! End to end test of the MAX7219 model: an SPI1 master configures the LED
//! matrix exactly the way the CubeMX firmware does (`PA5` SCK, `PA7` MOSI,
//! `PA4` LOAD) and the module shows the picture in its register file.

use std::cell::RefCell;
use std::rc::Rc;

use gsim_core::{AccessWidth, Event, MemoryBus};
use gsim_devices::Max7219;
use gsim_stm32f1::{map, Soc};

const RCC: u32 = map::rcc::BASE;
const SPI1: u32 = map::spi::BASE_1;
const GPIOA: u32 = map::gpio::PORTS[0];

const APB2ENR_GPIOA: u32 = 1 << 2;
const APB2ENR_SPI1: u32 = 1 << 12;

fn write(soc: &mut Soc, address: u32, value: u32) {
    soc.write(address, AccessWidth::Word, value).unwrap();
}

fn read(soc: &mut Soc, address: u32) -> u32 {
    soc.read(address, AccessWidth::Word).unwrap()
}

/// Configures `PA5`/`PA7` as SPI1 alternate function, `PA4` as a push-pull
/// chip select and the SPI as an 8 bit master at `fPCLK/8`.
fn configured(soc: &mut Soc) {
    write(soc, RCC + map::rcc::APB2ENR, APB2ENR_GPIOA | APB2ENR_SPI1);
    // PA4: output push-pull 2 MHz, PA5/PA7: AF push-pull 50 MHz.
    let crl = read(soc, GPIOA + map::gpio::CRL);
    let crl = (crl & !0x000F_0000) | (0b0010 << 16);
    let crl = (crl & !((0xF << 20) | (0xF << 28))) | (0b1011 << 20) | (0b1011 << 28);
    write(soc, GPIOA + map::gpio::CRL, crl);
    // MSTR, BR = /8, SPE.
    write(soc, SPI1 + map::spi::CR1, (1 << 2) | (0b010 << 3) | (1 << 6));
    // `LOAD` idles high, exactly like `MAX7219_Init()` leaves it.
    write(soc, GPIOA + map::gpio::BSRR, 1 << 4);
}

/// Shifts one byte and lets the frame complete.
fn shift_byte(soc: &mut Soc, matrix: &Rc<RefCell<Max7219>>, byte: u8) {
    let miso = matrix.borrow().miso();
    soc.spi_set_miso(0, miso);
    write(soc, SPI1 + map::spi::DR, u32::from(byte));
    for _ in 0..24 {
        soc.advance_ns(500);
    }
}

fn write_register(soc: &mut Soc, matrix: &Rc<RefCell<Max7219>>, register: u8, value: u8) {
    // `LOAD` low starts the transaction, high latches the word.
    write(soc, GPIOA + map::gpio::BSRR, 1 << (4 + 16));
    shift_byte(soc, matrix, register);
    shift_byte(soc, matrix, value);
    write(soc, GPIOA + map::gpio::BSRR, 1 << 4);
}

fn wired_soc() -> (Soc, Rc<RefCell<Max7219>>) {
    let mut soc = Soc::new();
    configured(&mut soc);
    let matrix = Rc::new(RefCell::new(Max7219::firmware_default()));
    let sink = Rc::clone(&matrix);
    soc.add_event_sink(Box::new(move |event: &Event| {
        sink.borrow_mut().on_event(event);
    }));
    (soc, matrix)
}

#[test]
fn the_matrix_shows_the_word_the_spi_master_wrote() {
    let (mut soc, matrix) = wired_soc();

    // The firmware's initialisation sequence.
    for (register, value) in [
        (0x0Fu8, 0x00u8),
        (0x09, 0x00),
        (0x0B, 0x07),
        (0x0A, 0x07),
        (0x0C, 0x01),
    ] {
        write_register(&mut soc, &matrix, register, value);
    }
    // The check mark the sample firmware draws.
    for (row, byte) in [0x00u8, 0x01, 0x02, 0x04, 0x48, 0x30, 0x20, 0x00]
        .into_iter()
        .enumerate()
    {
        write_register(&mut soc, &matrix, 0x01 + row as u8, byte);
    }

    let device = matrix.borrow();
    let chip = &device.chips()[0];
    assert!(chip.enabled(), "the display was never switched on");
    assert_eq!(chip.intensity(), 7);
    assert_eq!(chip.scanned_rows(), 8);
    assert_eq!(device.words(), 13);

    let pixels = chip.pixels();
    assert!(pixels[4][1], "row 5 bit 1 of the check mark");
    assert!(pixels[4][4]);
    assert!(!pixels[0][0]);
    drop(device);
    assert_eq!(
        soc.read(SPI1 + map::spi::SR, AccessWidth::Word).unwrap() & 0x80,
        0,
        "the SPI is idle again"
    );
}

#[test]
fn the_load_line_delimits_transactions() {
    let (mut soc, matrix) = wired_soc();

    // Bytes shifted while LOAD stays high are ignored by the part.
    shift_byte(&mut soc, &matrix, 0x0C);
    shift_byte(&mut soc, &matrix, 0x01);
    assert_eq!(matrix.borrow().words(), 0);
    assert!(!matrix.borrow().chips()[0].enabled());
}