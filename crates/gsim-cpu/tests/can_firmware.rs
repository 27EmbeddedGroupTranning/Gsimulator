//! Runs the CubeMX firmware that ships in `firmware/CAN` far enough to prove
//! the CAN controller and the SPI LED matrix are wired into the device.
//!
//! The image is a build artefact: when it is missing the test only reports the
//! skip, so a fresh checkout still passes.

mod support;

use std::cell::RefCell;
use std::path::PathBuf;
use std::rc::Rc;

use gsim_cpu::{Cpu, ElfImage};
use gsim_stm32f1::{CanMode, Soc};

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

#[test]
fn the_can_firmware_initialises_the_controller_and_the_led_matrix() {
    let Some(path) = firmware() else {
        return;
    };
    let image = ElfImage::open(&path).expect("CAN.elf");
    let soc = Rc::new(RefCell::new(Soc::with_hse(8_000_000)));
    let log = support::EventLog::new();
    log.attach(&mut soc.borrow_mut());
    let mut cpu = Cpu::new(Rc::clone(&soc)).expect("cpu");
    cpu.load(&image).expect("load");

    // HAL_Init, SystemClock_Config, MX_GPIO_Init, MX_CAN_Init, the acceptance
    // filter, HAL_CAN_Start, the notification setup and MAX7219_Init all run
    // within this budget; a bus error anywhere would have surfaced as `Err`.
    cpu.run(1_500_000).expect("the firmware never faults");

    let can = soc.borrow();
    assert_eq!(can.can.mode(), CanMode::Loopback, "CAN_MODE_LOOPBACK");
    assert_eq!(can.can.bitrate(can.clocks().pclk1_hz), 200_000);
    assert_eq!(can.can.active_filters(), 1);
    assert_eq!(can.can.stats().aborted, 0);
    drop(can);

    // 5 configuration registers and a full clear: 26 bytes over SPI1.
    let spi_frames = log
        .events()
        .iter()
        .filter(|event| matches!(event, gsim_core::Event::SpiTransfer { unit: 0, .. }))
        .count();
    assert!(spi_frames >= 26, "the LED matrix was never initialised");
}

#[test]
fn the_can_firmware_sends_its_loopback_frame_after_the_delay() {
    let Some(path) = firmware() else {
        return;
    };
    let image = ElfImage::open(&path).expect("CAN.elf");
    let soc = Rc::new(RefCell::new(Soc::with_hse(8_000_000)));
    let log = support::EventLog::new();
    log.attach(&mut soc.borrow_mut());
    let mut cpu = Cpu::new(Rc::clone(&soc)).expect("cpu");
    cpu.load(&image).expect("load");

    // The main loop waits 500 ms in `HAL_Delay` and then sends one frame in
    // loopback mode, so one second of instructions has to produce one.
    cpu.run(80_000_000).expect("the firmware never faults");


    let frames: Vec<_> = log
        .events()
        .iter()
        .filter_map(|event| match event {
            gsim_core::Event::CanFrame { frame, .. } => Some(*frame),
            _ => None,
        })
        .collect();
    assert!(
        !frames.is_empty(),
        "no loopback frame after {:.1} ms",
        soc.borrow().now_ns() as f64 / 1e6
    );
    assert_eq!(frames[0].id, 0x123);
    assert_eq!(frames[0].bytes(), b"Hello");
    // The firmware's RX interrupt already emptied FIFO0, so the counters are
    // what proves the frame was looped back and accepted by the filter.
    assert!(
        soc.borrow().can.stats().loopback_frames > 0,
        "the controller never looped the frame back"
    );
}