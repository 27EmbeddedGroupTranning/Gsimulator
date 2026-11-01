//! `gsimulator demo` – a scripted firmware style tour of every phase 1 block.
//!
//! The demo drives the device exactly like firmware does, through register
//! writes and reads, and prints a transcript of what the modelled hardware
//! does in response. It needs no ELF file and no cross compiler.

use std::cell::{Cell, RefCell};
use std::io::Write;
use std::rc::Rc;

use gsim_core::irq::{exception, lines};
use gsim_core::{AccessWidth, CanFrame, Event, EventSink, MemoryBus};
use gsim_devices::{CanBus, CanTransceiver, Max7219};
use gsim_stm32f1::{map, Soc};

/// Runs the demonstration and prints its transcript.
pub fn run() -> Result<(), String> {
    println!("Gsimulator - STM32F103 peripheral demo (on-chip blocks + off-chip parts)");
    println!("----------------------------------------------------------------------");

    let soc = Rc::new(RefCell::new(Soc::new()));
    let sink = ConsoleSink;
    soc.borrow_mut().add_event_sink(Box::new(sink));

    clock_tree(&soc);
    gpio_and_exti(&soc)?;
    usart(&soc)?;
    spi(&soc)?;
    timers(&soc)?;
    adc_with_dma(&soc)?;
    i2c(&soc)?;
    systick(&soc);
    watchdog(&soc)?;
    can_and_led_matrix(&soc)?;

    println!();
    println!(
        "demo finished: {} bus accesses, {} watch/reset events, {:.3} ms simulated",
        soc.borrow().stats().reads + soc.borrow().stats().writes,
        soc.borrow().reset_count(),
        soc.borrow().now_ns() as f64 / 1_000_000.0
    );
    Ok(())
}

/// Prints USART bytes and noteworthy events as they happen.
struct ConsoleSink;

impl EventSink for ConsoleSink {
    fn on_event(&mut self, event: &Event) {
        match event {
            Event::UartTx { unit, byte } if *unit == 0 => {
                let mut stdout = std::io::stdout();
                let _ = stdout.write_all(&[*byte]);
                let _ = stdout.flush();
            }
            Event::Note {
                source,
                message,
                detail,
            } => {
                println!("  [{source}] {message} (0x{detail:X})");
            }
            _ => {}
        }
    }
}

fn write32(soc: &Rc<RefCell<Soc>>, address: u32, value: u32) {
    soc.borrow_mut()
        .write(address, AccessWidth::Word, value)
        .unwrap_or_else(|error| panic!("write 0x{address:08X}: {error}"));
}

fn read32(soc: &Rc<RefCell<Soc>>, address: u32) -> u32 {
    soc.borrow_mut().read(address, AccessWidth::Word).unwrap()
}

fn clock_tree(soc: &Rc<RefCell<Soc>>) {
    let clocks = soc.borrow().clocks();
    println!();
    println!("[RCC] power-on clock tree");
    println!(
        "  SYSCLK {:>9} Hz  HCLK {:>9} Hz  PCLK1 {:>9} Hz  PCLK2 {:>9} Hz",
        clocks.sysclk_hz, clocks.hclk_hz, clocks.pclk1_hz, clocks.pclk2_hz
    );

    let rcc = map::rcc::BASE;
    // HSE on, then wait for the ready flag like the HAL does.
    write32(soc, rcc + map::rcc::CR, (1 << 16) | 1);
    let mut waited = 0u64;
    while read32(soc, rcc + map::rcc::CR) & (1 << 17) == 0 {
        soc.borrow_mut().advance_ns(1_000);
        waited += 1_000;
        assert!(waited < 20_000_000, "HSE never became ready");
    }
    println!("  HSE ready after {:.2} ms", waited as f64 / 1_000_000.0);

    // PLL x9 from HSE, AHB /1, APB1 /2, APB2 /1, ADC /6.
    let cfgr = (1 << 16) | (0b0111 << 18) | (0b100 << 8) | (0b10 << 14);
    write32(soc, rcc + map::rcc::CFGR, cfgr);
    write32(soc, rcc + map::rcc::CR, (1 << 16) | (1 << 24) | 1);
    while read32(soc, rcc + map::rcc::CR) & (1 << 25) == 0 {
        soc.borrow_mut().advance_ns(1_000);
    }
    write32(soc, rcc + map::rcc::CFGR, cfgr | 0b10);
    while (read32(soc, rcc + map::rcc::CFGR) >> 2) & 0b11 != 0b10 {
        soc.borrow_mut().advance_ns(1_000);
    }
    let clocks = soc.borrow().clocks();
    println!("[RCC] PLL locked, running from HSE x9");
    println!(
        "  SYSCLK {:>9} Hz  HCLK {:>9} Hz  PCLK1 {:>9} Hz  PCLK2 {:>9} Hz",
        clocks.sysclk_hz, clocks.hclk_hz, clocks.pclk1_hz, clocks.pclk2_hz
    );
    println!(
        "  TIM1/TIM2 clock {:>9} Hz, ADCCLK {:>9} Hz",
        clocks.tim1_hz, clocks.adc_hz
    );

    // Clocks for everything used below.
    write32(
        soc,
        rcc + map::rcc::APB2ENR,
        (1 << 0) | (1 << 2) | (1 << 9) | (1 << 12) | (1 << 14),
    );
    write32(
        soc,
        rcc + map::rcc::APB1ENR,
        (1 << 1) | (1 << 11) | (1 << 21) | (1 << 17),
    );
    write32(soc, rcc + map::rcc::AHBENR, 1);
    println!("[RCC] enabled AFIO, GPIOA, ADC1, SPI1, USART1, TIM3, WWDG, I2C1, USART2, DMA1");
}

fn gpio_and_exti(soc: &Rc<RefCell<Soc>>) -> Result<(), String> {
    println!();
    println!("[GPIO] PA5 as a push-pull output, PA0 as a pull-down EXTI input");

    // PA5 output 2 MHz: CRL nibble 5 = 0b0010.
    let crl = read32(soc, map::gpio::BASE_A + map::gpio::CRL);
    let crl = (crl & !(0xF << 20)) | (0b0010 << 20);
    // PA0 input with pull-down: nibble 0 = 0b1000.
    let crl = (crl & !0xF) | 0b1000;
    write32(soc, map::gpio::BASE_A + map::gpio::CRL, crl);

    write32(soc, map::gpio::BASE_A + map::gpio::BSRR, 1 << 5);
    let idr = read32(soc, map::gpio::BASE_A + map::gpio::IDR);
    println!("  PA5 high -> IDR bit 5 = {}", (idr >> 5) & 1);
    write32(soc, map::gpio::BASE_A + map::gpio::BRR, 1 << 5);
    println!(
        "  PA5 low  -> IDR bit 5 = {}",
        (read32(soc, map::gpio::BASE_A + map::gpio::IDR) >> 5) & 1
    );

    // EXTI0 <- port A, rising edge, unmasked, NVIC enabled at priority 0x20.
    write32(soc, map::afio::BASE + map::afio::EXTICR1, 0);
    write32(soc, map::exti::BASE + map::exti::RTSR, 1);
    write32(soc, map::exti::BASE + map::exti::IMR, 1);
    {
        let mut borrow = soc.borrow_mut();
        borrow.nvic.enable(lines::EXTI0, true);
        borrow.nvic.set_priority(lines::EXTI0, 0x20);
    }

    // Drive the pin high: the SoC samples pins into EXTI on every step.
    soc.borrow_mut().gpio_set_input(0, 0, true);
    soc.borrow_mut().advance_ns(1_000);
    let pending = soc.borrow().pending_exception();
    println!(
        "[EXTI] rising edge on PA0 -> PR = 0x{:04X}, pending exception = {:?}",
        read32(soc, map::exti::BASE + map::exti::PR),
        pending
    );

    if let Some(exception_number) = pending {
        soc.borrow_mut().enter_exception(exception_number);
        println!(
            "  handler entered, nesting depth = {}",
            soc.borrow().nvic.nesting_depth()
        );
        // The ISR acknowledges the line.
        write32(soc, map::exti::BASE + map::exti::PR, 1);
        soc.borrow_mut().exit_exception(exception_number);
    }
    Ok(())
}

fn usart(soc: &Rc<RefCell<Soc>>) -> Result<(), String> {
    println!();
    println!("[USART1] 115200 8N1 on PCLK2 = 72 MHz, polling TXE");

    // 72 MHz / 115200 = 625.
    write32(soc, map::usart::BASE_1 + map::usart::BRR, 625);
    write32(
        soc,
        map::usart::BASE_1 + map::usart::CR1,
        (1 << 3) | (1 << 13),
    );
    println!(
        "  BRR = 625 -> {} baud",
        soc.borrow().usart[0].baud(72_000_000)
    );

    print!("  wire: ");
    for byte in b"Hello, Gsimulator!\r\n" {
        write32(soc, map::usart::BASE_1 + map::usart::DR, u32::from(*byte));
        // One 8N1 frame at 115200 baud is ~87 us; give each byte its time.
        soc.borrow_mut().advance_ns(90_000);
    }
    println!();
    Ok(())
}

fn spi(soc: &Rc<RefCell<Soc>>) -> Result<(), String> {
    println!();
    println!("[SPI1] master, fPCLK2/8, writing a MAX7219 initialisation sequence");

    // PA5/PA7 as alternate function push-pull 50 MHz.
    let mut crl = read32(soc, map::gpio::BASE_A + map::gpio::CRL);
    crl = (crl & !(0xF << 20)) | (0b1011 << 20);
    crl = (crl & !(0xF << 28)) | (0b1011 << 28);
    write32(soc, map::gpio::BASE_A + map::gpio::CRL, crl);

    // MSTR, BR = /8, SPE, software slave management.
    write32(
        soc,
        map::spi::BASE_1 + map::spi::CR1,
        (1 << 2) | (0b010 << 3) | (1 << 6) | (1 << 8) | (1 << 9),
    );
    println!("  SCK = {} Hz", soc.borrow().spi[0].sck_hz(72_000_000));

    for (register, value) in [(0x09u8, 0xFFu8), (0x0B, 0x07), (0x0C, 0x01)] {
        for byte in [register, value] {
            write32(soc, map::spi::BASE_1 + map::spi::DR, u32::from(byte));
            soc.borrow_mut().advance_ns(1_000);
        }
        println!("  frame 0x{register:02X}{value:02X} shifted out");
    }
    Ok(())
}

fn timers(soc: &Rc<RefCell<Soc>>) -> Result<(), String> {
    println!();
    println!("[TIM3] 1 kHz PWM on channel 1 (APB1 timer clock = 72 MHz)");

    write32(soc, map::tim::BASE_3 + map::tim::PSC, 71); // 1 MHz counter clock
    write32(soc, map::tim::BASE_3 + map::tim::ARR, 999); // 1 kHz period
    write32(soc, map::tim::BASE_3 + map::tim::CCMR1, 0b110 << 4); // PWM mode 1
    write32(soc, map::tim::BASE_3 + map::tim::CCER, 1);
    write32(soc, map::tim::BASE_3 + map::tim::CCR1, 250);
    write32(soc, map::tim::BASE_3 + map::tim::CR1, 1);

    // Sample the compare output at four points of the period.
    write32(soc, map::tim::BASE_3 + map::tim::SR, 0);
    let mut elapsed = 0u64;
    for microseconds in [100u64, 200, 300, 400] {
        soc.borrow_mut().run_us(microseconds);
        elapsed += microseconds;
        let counter = soc.borrow().tim[2].counter();
        let level = soc.borrow().tim[2].output_level(0);
        println!(
            "  t = {elapsed:>4} us, CNT = {counter:>4} -> OC1 = {:?}",
            level.unwrap_or(false)
        );
    }
    println!(
        "  CCMR1 = 0x{:04X}, CCR1 = {} (25 % of 1000)",
        read32(soc, map::tim::BASE_3 + map::tim::CCMR1),
        read32(soc, map::tim::BASE_3 + map::tim::CCR1)
    );

    println!("[TIM3] counter overflow sets UIF and raises the TIM3 interrupt");
    write32(soc, map::tim::BASE_3 + map::tim::SR, 0);
    write32(soc, map::tim::BASE_3 + map::tim::DIER, 1);
    soc.borrow_mut().nvic.enable(lines::TIM3, true);
    soc.borrow_mut().run_us(1_100);
    let status = read32(soc, map::tim::BASE_3 + map::tim::SR);
    println!(
        "  SR = 0x{status:04X} (UIF = {}), pending exception = {:?}",
        status & 1,
        soc.borrow().pending_exception()
    );
    Ok(())
}

fn adc_with_dma(soc: &Rc<RefCell<Soc>>) -> Result<(), String> {
    println!();
    println!("[ADC1 + DMA1 ch1] 1650 mV on channel 1 converted into a circular SRAM buffer");

    let buffer = map::SRAM_BASE;
    for index in 0..8 {
        soc.borrow_mut()
            .write(buffer + index * 2, AccessWidth::Half, 0)
            .map_err(|error| error.to_string())?;
    }
    soc.borrow_mut().adc_set_channel_mv(0, 1, 1650);

    // ADC1: channel 1, 239.5 sample cycles, single conversion, DMA on EOC.
    write32(soc, map::adc::BASE_1 + map::adc::SMPR2, 7);
    write32(soc, map::adc::BASE_1 + map::adc::SQR1, 0);
    write32(soc, map::adc::BASE_1 + map::adc::SQR3, 1);
    write32(
        soc,
        map::adc::BASE_1 + map::adc::CR2,
        (1 << 0) | (1 << 1) | (1 << 8) | (1 << 22),
    ); // ADON | CONT | DMA | SWSTART

    // DMA1 channel 1: peripheral -> memory, 16 bit, circular, memory increment.
    let channel = map::dma1::BASE + map::dma1::channel(0);
    write32(
        soc,
        channel + map::dma1::CPAR,
        map::adc::BASE_1 + map::adc::DR,
    );
    write32(soc, channel + map::dma1::CMAR, buffer);
    write32(soc, channel + map::dma1::CNDTR, 8);
    write32(
        soc,
        channel + map::dma1::CCR,
        (1 << 0) | (0b01 << 8) | (1 << 5) | (1 << 7),
    ); // EN | 16 bit | CIRC | MINC

    soc.borrow_mut().run_us(120);

    let mut codes = Vec::new();
    for index in 0..8 {
        codes.push(
            soc.borrow_mut()
                .read(buffer + index * 2, AccessWidth::Half)
                .map_err(|error| error.to_string())?,
        );
    }
    println!("  buffer = {codes:?}");
    println!(
        "  DMA ISR = 0x{:08X} (TC flags), {} transfers",
        read32(soc, map::dma1::BASE + map::dma1::ISR),
        soc.borrow().stats().dma_transfers
    );
    Ok(())
}

fn i2c(soc: &Rc<RefCell<Soc>>) -> Result<(), String> {
    println!();
    println!("[I2C1] master start + address 0xA0, with no device attached");

    write32(soc, map::i2c::BASE_1 + map::i2c::CR2, 36); // 36 MHz peripheral clock
    write32(soc, map::i2c::BASE_1 + map::i2c::CCR, 180); // 100 kHz
    write32(soc, map::i2c::BASE_1 + map::i2c::TRISE, 37);
    write32(soc, map::i2c::BASE_1 + map::i2c::CR1, 1); // PE
    write32(soc, map::i2c::BASE_1 + map::i2c::CR1, 1 | (1 << 8)); // PE | START
    write32(soc, map::i2c::BASE_1 + map::i2c::DR, 0xA0);
    let sr1 = read32(soc, map::i2c::BASE_1 + map::i2c::SR1);
    println!(
        "  SR1 = 0x{sr1:04X} -> ACK failure = {}",
        sr1 & (1 << 10) != 0
    );
    println!(
        "  SR2 = 0x{:04X} (bus busy, master)",
        read32(soc, map::i2c::BASE_1 + map::i2c::SR2)
    );
    write32(soc, map::i2c::BASE_1 + map::i2c::CR1, 1 | (1 << 9)); // STOP
    println!(
        "  after STOP, busy = {}",
        read32(soc, map::i2c::BASE_1 + map::i2c::SR2) & 0b10 != 0
    );
    Ok(())
}

fn systick(soc: &Rc<RefCell<Soc>>) {
    println!();
    println!("[SysTick] 1 ms period at 72 MHz, count three ticks");

    write32(soc, 0xE000_E014, 71_999);
    write32(soc, 0xE000_E010, 0b111);

    let mut ticks = 0;
    while ticks < 3 {
        soc.borrow_mut().advance_ns(100_000);
        let pending = soc.borrow().nvic.is_system_pending(exception::SYSTICK);
        if pending {
            soc.borrow_mut()
                .nvic
                .set_system_pending(exception::SYSTICK, false);
            ticks += 1;
            println!(
                "  tick {ticks} at {:.3} ms",
                soc.borrow().now_ns() as f64 / 1_000_000.0
            );
        }
    }
}

fn watchdog(soc: &Rc<RefCell<Soc>>) -> Result<(), String> {
    println!();
    println!("[IWDG] start with a 10 count reload, then let it expire");

    write32(soc, map::iwdg::BASE + map::iwdg::KR, 0x5555);
    write32(soc, map::iwdg::BASE + map::iwdg::PR, 0);
    write32(soc, map::iwdg::BASE + map::iwdg::RLR, 10);
    write32(soc, map::iwdg::BASE + map::iwdg::KR, 0xCCCC);
    println!("  timeout = {} us", soc.borrow().iwdg.timeout_ns() / 1_000);
    let before = soc.borrow().reset_count();
    soc.borrow_mut().advance_ns(2_000_000);
    let after = soc.borrow().reset_count();
    println!(
        "  resets: {before} -> {after}, cause flags in RCC_CSR = 0x{:08X}",
        read32(soc, map::rcc::BASE + map::rcc::CSR)
    );
    Ok(())
}

/// Forwards peripheral events to the two off-chip parts of the demo.
struct DeviceSink {
    matrix: Rc<RefCell<Max7219>>,
    transceiver: Rc<RefCell<CanTransceiver>>,
    clock_ns: Rc<Cell<u64>>,
}

impl EventSink for DeviceSink {
    fn on_event(&mut self, event: &Event) {
        let now = self.clock_ns.get();
        self.matrix.borrow_mut().on_event(event);
        self.transceiver.borrow_mut().on_event(event, now);
    }
}

/// Drives CAN1 in loopback mode plus the two device models of `gsim-devices`.
fn can_and_led_matrix(soc: &Rc<RefCell<Soc>>) -> Result<(), String> {
    let matrix = Rc::new(RefCell::new(Max7219::firmware_default()));
    let bus = Rc::new(RefCell::new(CanBus::new()));
    let transceiver = Rc::new(RefCell::new(CanTransceiver::attach(
        Rc::clone(&bus),
        "demo 节点",
        0,
    )));
    let clock = Rc::new(Cell::new(soc.borrow().now_ns()));
    soc.borrow_mut().add_event_sink(Box::new(DeviceSink {
        matrix: Rc::clone(&matrix),
        transceiver: Rc::clone(&transceiver),
        clock_ns: Rc::clone(&clock),
    }));

    println!();
    println!("== CAN1 (bxCAN) 回环 + 收发器 + 虚拟总线 ==");
    let can = map::can::BASE_1;
    let apb1enr = read32(soc, map::rcc::BASE + map::rcc::APB1ENR);
    write32(soc, map::rcc::BASE + map::rcc::APB1ENR, apb1enr | (1 << 25));

    // 200 kbit/s: PCLK1 36 MHz / 36 = 1 us per tq, 5 tq per bit.
    write32(soc, can + map::can::MCR, 1);
    write32(
        soc,
        can + map::can::BTR,
        (2 - 1) << 16 | (2 - 1) << 20 | 35 | (1 << 30),
    );
    // One 32 bit mask filter for the standard identifier 0x123, into FIFO0.
    write32(soc, can + map::can::FMR, 1);
    write32(soc, can + map::can::FS1R, 1);
    write32(soc, can + map::can::FILTER_BASE, 0x123 << 21);
    write32(soc, can + map::can::FILTER_BASE + 4, 0x7FF << 21);
    write32(soc, can + map::can::FA1R, 1);
    write32(soc, can + map::can::FMR, 0);
    write32(soc, can + map::can::MCR, 0);
    write32(soc, can + map::can::IER, 1 << 1);

    let payload = b"Hello";
    let mut low = 0u32;
    let mut high = 0u32;
    for index in 0..4 {
        low |= u32::from(*payload.get(index).unwrap_or(&0)) << (index * 8);
        high |= u32::from(*payload.get(index + 4).unwrap_or(&0)) << (index * 8);
    }
    write32(soc, can + map::can::TX_BASE, 0x123 << 21);
    write32(
        soc,
        can + map::can::TX_BASE + map::can::TDTR,
        payload.len() as u32,
    );
    write32(soc, can + map::can::TX_BASE + map::can::TDHR, high);
    write32(soc, can + map::can::TX_BASE + map::can::TDLR, low);
    write32(soc, can + map::can::TX_BASE, (0x123 << 21) | 1);
    println!("  邮箱已请求发送 0x123 [48 65 6C 6C 6F]（回环模式）");

    // The watchdog demo reset the device, so the bus runs on the 8 MHz HSI:
    // one bit is 22.5 us and an 87 bit frame needs about 2 ms.
    advance(soc, &clock, 2_500_000);
    let received = soc.borrow().can.rx_frames(0);
    println!(
        "  帧时间到 → FIFO0 收到 {} 帧：{}",
        received.len(),
        received
            .iter()
            .map(|frame| format!("{} [{}]", frame.id_label(), frame.data_label()))
            .collect::<Vec<_>>()
            .join(", ")
    );
    println!(
        "  控制器状态：{}，TEC/REC = {:?}，累计发送 {} 帧",
        soc.borrow().can.mode().label(),
        soc.borrow().can.error_counters(),
        soc.borrow().can.stats().tx_frames
    );

    // A second ECU puts a frame on the wire; the transceiver hands it to CAN1.
    let now = soc.borrow().now_ns();
    let injected = CanFrame::standard(0x123, b"bus");
    bus.borrow_mut().inject(injected, now);
    loop {
        let frame = transceiver.borrow_mut().poll();
        let Some(frame) = frame else { break };
        soc.borrow_mut().can_receive(0, frame);
    }
    println!(
        "  另一节点注入 0x123 [62 75 73] → FIFO0 {} 帧，总线记录 {} 条",
        soc.borrow().can.rx_level(0),
        bus.borrow().records().len()
    );

    println!();
    println!("== CAN 物理层：真实线与仲裁 + 终端电阻 ==");
    can_wire_physics(soc, can);

    println!();
    println!("== MAX7219 8x8 点阵（SPI1: PA5/PA7，PA4 = LOAD）==");
    let gpioa = map::gpio::PORTS[0];
    let apb2enr = read32(soc, map::rcc::BASE + map::rcc::APB2ENR);
    write32(
        soc,
        map::rcc::BASE + map::rcc::APB2ENR,
        apb2enr | (1 << 2) | (1 << 12),
    );
    let crl = read32(soc, gpioa + map::gpio::CRL);
    let crl = (crl & !0x000F_0000) | (0b0010 << 16);
    let crl = (crl & !((0xF << 20) | (0xF << 28))) | (0b1011 << 20) | (0b1011 << 28);
    write32(soc, gpioa + map::gpio::CRL, crl);
    write32(soc, gpioa + map::gpio::BSRR, 1 << 4);
    write32(
        soc,
        map::spi::BASE_1 + map::spi::CR1,
        (1 << 2) | (0b010 << 3) | (1 << 6),
    );

    let icon: [u8; 8] = [0x3C, 0x42, 0xA5, 0x81, 0xA5, 0x99, 0x42, 0x3C];
    for (register, value) in [
        (0x0Fu8, 0x00u8),
        (0x09, 0x00),
        (0x0B, 0x07),
        (0x0A, 0x0F),
        (0x0C, 0x01),
    ] {
        max7219_write(soc, &matrix, &clock, register, value);
    }
    for (row, byte) in icon.iter().enumerate() {
        max7219_write(soc, &matrix, &clock, 0x01 + row as u8, *byte);
    }

    let device = matrix.borrow();
    let pixels = device.chips()[0].pixels();
    for row in pixels {
        let line: String = row
            .iter()
            .map(|pixel| if *pixel { '#' } else { '.' })
            .collect();
        println!("  {line}");
    }
    println!(
        "  亮度 {}/15，已锁存 {} 帧，DOUT 回显 0x{:04X}",
        device.chips()[0].intensity(),
        device.words(),
        device.miso()
    );
    Ok(())
}

/// Runs the wire bit by bit for `bits` bit times, as the GUI bridge does.
fn run_wire(soc: &Rc<RefCell<Soc>>, bus: &Rc<RefCell<CanBus>>, bits: u32) {
    for _ in 0..bits {
        let bit_ns = soc.borrow().can_bit_time_ns(0).max(1);
        let now = soc.borrow().now_ns();
        let controller = soc.borrow().can_driven_level(0);
        let level = bus.borrow_mut().step(controller, now);
        soc.borrow_mut().can_sample_bit(0, level);
        soc.borrow_mut().advance_ns(bit_ns);
    }
}

/// Arms transmit mailbox 0 with a standard data frame.
fn request_standard(soc: &Rc<RefCell<Soc>>, can: u32, id: u32, payload: &[u8]) {
    let mut low = 0u32;
    let mut high = 0u32;
    for index in 0..4 {
        low |= u32::from(*payload.get(index).unwrap_or(&0)) << (index * 8);
        high |= u32::from(*payload.get(index + 4).unwrap_or(&0)) << (index * 8);
    }
    write32(soc, can + map::can::TX_BASE, id << 21);
    write32(
        soc,
        can + map::can::TX_BASE + map::can::TDTR,
        payload.len() as u32,
    );
    write32(soc, can + map::can::TX_BASE + map::can::TDHR, high);
    write32(soc, can + map::can::TX_BASE + map::can::TDLR, low);
    write32(soc, can + map::can::TX_BASE, (id << 21) | 1);
}

/// Walks the bit level wire: a rival node, real arbitration and what a missing
/// termination resistor does to the frames.
fn can_wire_physics(soc: &Rc<RefCell<Soc>>, can: u32) {
    let wire = Rc::new(RefCell::new(CanBus::new()));
    // Leaves loopback mode: from now on the controller follows the wire.
    write32(soc, can + map::can::MCR, 1);
    write32(soc, can + map::can::BTR, (2 - 1) << 16 | (2 - 1) << 20 | 35);
    write32(soc, can + map::can::MCR, 0);
    soc.borrow_mut().can_set_on_bus(0, true);
    wire.borrow_mut()
        .set_bit_time(soc.borrow().can_bit_time_ns(0));
    // Two 120 Ω parts, one at each end of the pair.
    wire.borrow_mut().set_termination(Some(60.0));
    let rival = wire.borrow_mut().add_peer("对手节点");

    // Both nodes ask for the wire on the same bit; the identifier decides.
    wire.borrow_mut()
        .queue_frame(rival, CanFrame::standard(0x100, b"win"));
    request_standard(soc, can, 0x321, b"mine");
    run_wire(soc, &wire, 240);

    let stats = soc.borrow().can.stats();
    println!(
        "  0x321 与 0x100 同时起跑：仲裁失败 {} 次，位错误 {} 次，最终发出 {} 帧",
        stats.arbitration_lost, stats.tx_errors, stats.tx_frames
    );

    println!(
        "  收发记录：{}",
        wire.borrow()
            .records()
            .iter()
            .map(|record| format!(
                "{} {} {}",
                record.node_name,
                if record.direction == gsim_core::CanDirection::Tx {
                    "发送"
                } else {
                    "接收"
                },
                record.frame.id_label()
            ))
            .collect::<Vec<_>>()
            .join("，")
    );

    for (label, ohms) in [
        ("两个 120 Ω（并联 60 Ω，教科书接法）", Some(60.0)),
        ("只留一个 120 Ω", Some(120.0)),
        ("两只都改成 30 Ω（并联 15 Ω，负载过重）", Some(15.0)),
        ("完全没有终端电阻", None),
    ] {
        wire.borrow_mut().set_termination(ohms);
        let before = soc.borrow().can.stats().rx_frames;
        let errors = wire.borrow().stats().error_frames;
        for _ in 0..6 {
            let now = soc.borrow().now_ns();
            wire.borrow_mut()
                .inject_bit_frame(CanFrame::standard(0x123, b"GSIM"), now);
            run_wire(soc, &wire, 150);
        }
        let bus = wire.borrow();
        println!(
            "  {label}：质量 {:.0}%，位错误率 {:.3}%/位，位 {}，尖峰 {}，错误帧 {}，控制器收到 {} 帧",
            bus.quality() * 100.0,
            bus.error_rate() * 100.0,
            bus.stats().bits,
            bus.glitches(),
            bus.stats().error_frames - errors,
            soc.borrow().can.stats().rx_frames - before
        );
    }
    soc.borrow_mut().can_set_on_bus(0, false);
}

/// Shifts one 16 bit command into the matrix (two SPI bytes, LOAD around them).
fn max7219_write(
    soc: &Rc<RefCell<Soc>>,
    matrix: &Rc<RefCell<Max7219>>,
    clock: &Rc<Cell<u64>>,
    register: u8,
    value: u8,
) {
    write32(soc, map::gpio::PORTS[0] + map::gpio::BSRR, 1 << (4 + 16));
    for byte in [register, value] {
        let miso = matrix.borrow().miso();
        soc.borrow_mut().spi_set_miso(0, miso);
        write32(soc, map::spi::BASE_1 + map::spi::DR, u32::from(byte));
        advance(soc, clock, 12_000);
    }
    write32(soc, map::gpio::PORTS[0] + map::gpio::BSRR, 1 << 4);
}

/// Steps the simulation, keeping the device sink's clock up to date.
fn advance(soc: &Rc<RefCell<Soc>>, clock: &Rc<Cell<u64>>, nanoseconds: u64) {
    let mut remaining = nanoseconds;
    while remaining > 0 {
        let chunk = remaining.min(1_000);
        clock.set(soc.borrow().now_ns());
        soc.borrow_mut().advance_ns(chunk);
        remaining -= chunk;
    }
    clock.set(soc.borrow().now_ns());
}
