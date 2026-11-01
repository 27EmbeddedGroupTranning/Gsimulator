//! Runs the peripheral self test firmware that ships in `firmware/CAN`.
//!
//! The image walks every block the simulator models (RCC, SysTick, GPIO, the
//! bit-band aliases, AFIO/EXTI, USART1..3, SPI1, I2C1, TIM1..4, ADC1, DMA1,
//! FLASH, CRC, PWR/BKP, RTC, IWDG, WWDG and bxCAN) with plain register access
//! and prints one line per block on USART1. The tests below assert the whole
//! report byte for byte, the fallback to the internal oscillator when no HSE
//! crystal is fitted, the USART1 receive/echo path, and that the main loop
//! keeps the watchdogs alive instead of letting the part reset.
//!
//! The image is a build artefact: when it is missing the tests only report the
//! skip, so a fresh checkout still passes.

mod support;

use std::cell::RefCell;
use std::path::PathBuf;
use std::rc::Rc;

use gsim_cpu::{Cpu, ElfImage};
use gsim_stm32f1::Soc;
use support::EventLog;

/// Report of a run with the 8 MHz crystal fitted, 1650 mV on ADC channel 1 and
/// no part wired to the board.
const REPORT_WITH_HSE: &str = "\
GSIM-SELFTEST 1
CLK 72000000 HSE
SYSTICK OK
GPIO OK
BITBAND OK
EXTI OK
USART1 OK
USART2 OK
USART3 OK
SPI1 OK
I2C1 OK
TIM1 OK
TIM2 OK
TIM3 OK
TIM4 OK
ADC OK 2047
DMA OK
FLASH OK
CRC OK 0xC704DD7B
RTC OK
PWR OK
BKP OK
CAN OK
IWDG OK
WWDG OK
ENV HSE=READY LSE=WAIT RTC=LSI RESET=POR
ONCHIP 23/23
DEV MATRIX=NA ECHO=0x00000000
LOOP
";

fn firmware() -> Option<PathBuf> {
    let path =
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../firmware/CAN/build/Debug/CAN.elf");
    if path.exists() {
        Some(path)
    } else {
        eprintln!(
            "skipping: {} is missing (build firmware/CAN with CMake first)",
            path.display()
        );
        None
    }
}

struct Session {
    log: EventLog,
    soc: Rc<RefCell<Soc>>,
    cpu: Cpu,
}

impl Session {
    /// Boots the firmware with or without the HSE crystal on the board.
    fn start(with_hse: bool) -> Option<Self> {
        let path = firmware()?;
        let image = ElfImage::open(&path).expect("CAN.elf");
        let soc = Rc::new(RefCell::new(Soc::new()));
        {
            let mut borrow = soc.borrow_mut();
            borrow.set_hse_present(with_hse);
            borrow.set_lse_present(false);
            // Channel 1 sees 1650 mV, which is almost exactly half scale.
            borrow.adc_set_channel_mv(0, 1, 1650);
        }
        let log = EventLog::new();
        log.attach(&mut soc.borrow_mut());
        let mut cpu = Cpu::new(Rc::clone(&soc)).expect("cpu");
        cpu.load(&image).expect("load");
        Some(Session { log, soc, cpu })
    }

    /// Runs the firmware until `needle` shows up on USART1 (or the budget runs
    /// out), so the tests never depend on a hard-coded instruction count.
    fn run_until(&mut self, needle: &str, budget: u64) -> bool {
        while self.cpu.instructions() < budget {
            self.cpu.run(200_000).expect("firmware execution failed");
            if self.log.uart_text(0).contains(needle) {
                return true;
            }
        }
        false
    }

    fn text(&self) -> String {
        self.log.uart_text(0)
    }
}

/// The part of the transcript printed before the main loop starts.
fn report_of(text: &str) -> &str {
    match text.find("LOOP\n") {
        Some(index) => &text[..index + "LOOP\n".len()],
        None => text,
    }
}

#[test]
fn the_firmware_reports_every_modelled_peripheral_and_keeps_running() {
    let Some(mut session) = Session::start(true) else {
        return;
    };

    assert!(
        session.run_until("EVT READY", 40_000_000),
        "the self test never finished, got {:?}",
        session.text()
    );

    let text = session.text();
    let report = report_of(&text);
    assert_eq!(report, REPORT_WITH_HSE);
    assert!(
        !text.contains("FAIL"),
        "a block reported a failure: {report:?}"
    );
    assert!(
        session.soc.borrow().hse_ready(),
        "the crystal never started"
    );

    // The terminal plays the peer: hand one byte to the receiver, the main
    // loop must echo it back and count it.
    session.soc.borrow_mut().usart_receive(0, b'k');
    assert!(
        session.run_until("EVT UART1 RX 1\n", 5_000_000),
        "the receiver interrupt never fired, got {:?}",
        session.text()
    );

    // Well past the WWDG timeout in simulated time. A firmware that stopped
    // refreshing the watchdog would reset, print a second banner and lose the
    // loop heartbeat.
    session
        .cpu
        .run(1_500_000)
        .expect("the firmware never faults");
    let text = session.text();
    assert_eq!(
        text.matches("GSIM-SELFTEST 1").count(),
        1,
        "the device was reset while the loop was running"
    );
}

#[test]
fn the_firmware_falls_back_to_the_internal_oscillator_without_a_crystal() {
    let Some(mut session) = Session::start(false) else {
        return;
    };

    assert!(
        session.run_until("EVT READY", 80_000_000),
        "the self test never finished, got {:?}",
        session.text()
    );

    let text = session.text();
    let report = report_of(&text);
    assert!(
        report.starts_with("GSIM-SELFTEST 1\nCLK 8000000 HSI\n"),
        "expected the HSI clock, got {report:?}"
    );
    assert!(report.contains("\nENV HSE=ABSENT "), "{report:?}");
    assert!(report.contains("\nONCHIP 23/23\n"), "{report:?}");
}
