//! Runs a real firmware image compiled with `arm-none-eabi-gcc`.
//!
//! The firmware in `tests/firmware/main.c` drives USART1 (polled), SysTick and
//! EXTI0 (both with interrupts), TIM3 and ADC1 and prints an ASCII report. If
//! the simulator gets any of those peripherals or the exception handling
//! wrong, the report changes or the run never finishes.

mod support;

use std::cell::RefCell;
use std::rc::Rc;

use gsim_cpu::Cpu;
use gsim_stm32f1::Soc;
use support::{build_firmware, EventLog};

fn run_smoke_firmware() -> Option<(EventLog, Rc<RefCell<Soc>>, u64)> {
    let elf = build_firmware("main")?;
    let soc = Rc::new(RefCell::new(Soc::new()));
    let log = EventLog::new();
    log.attach(&mut soc.borrow_mut());
    // Channel 1 sees 1650 mV, which is almost exactly half scale.
    soc.borrow_mut().adc_set_channel_mv(0, 1, 1650);

    let mut cpu = Cpu::new(Rc::clone(&soc)).unwrap();
    cpu.load_file(&elf).unwrap();

    // Run in chunks so the test stops as soon as the report is complete
    // instead of spinning in the firmware's endless loop.
    for _ in 0..40 {
        cpu.run(50_000).expect("firmware execution failed");
        if log.uart_text(0).contains("OK\n") || cpu.halted() {
            break;
        }
    }

    let instructions = cpu.instructions();
    Some((log, soc, instructions))
}

#[test]
fn firmware_reaches_main_and_prints_the_banner() {
    let Some((log, _soc, instructions)) = run_smoke_firmware() else {
        eprintln!("arm-none-eabi-gcc not found, skipping firmware test");
        return;
    };
    let text = log.uart_text(0);
    assert!(
        text.starts_with("GSIM\n"),
        "expected the banner first, got {text:?} after {instructions} instructions"
    );
}

#[test]
fn firmware_reports_the_expected_peripheral_state() {
    let Some((log, _soc, instructions)) = run_smoke_firmware() else {
        eprintln!("arm-none-eabi-gcc not found, skipping firmware test");
        return;
    };
    let text = log.uart_text(0);
    let expected = "GSIM\nT3\nE1\nM250\nA2047\nB13\nOK\n";
    assert_eq!(
        text, expected,
        "unexpected report after {instructions} instructions"
    );
}

#[test]
fn firmware_pwm_reaches_its_alternate_function_pin() {
    let Some((_log, soc, instructions)) = run_smoke_firmware() else {
        eprintln!("arm-none-eabi-gcc not found, skipping firmware test");
        return;
    };
    let mut soc = soc.borrow_mut();
    assert_eq!(
        soc.tim_channel_pin(2, 0),
        Some((0, 6)),
        "TIM3_CH1 belongs on PA6"
    );
    // A few more periods of the 1 kHz waveform the firmware configured.
    soc.advance_ns(10_000_000);
    let duty = soc.pin_duty(0, 6).unwrap_or_else(|| {
        panic!("the firmware left PA6 as a plain input after {instructions} instructions")
    });
    assert!(
        (duty - 0.25).abs() < 0.05,
        "the firmware asked for 25 % and the pin measures {duty}"
    );
}

#[test]
fn firmware_keeps_running_after_the_report() {
    let Some((log, soc, _instructions)) = run_smoke_firmware() else {
        eprintln!("arm-none-eabi-gcc not found, skipping firmware test");
        return;
    };
    let before = log.uart_text(0);
    assert!(before.ends_with("OK\n"));

    // The firmware parks in an endless loop, so more execution must not fail.
    let mut cpu = Cpu::new(Rc::clone(&soc)).unwrap();
    cpu.run(10_000).expect("second run failed");
    assert_eq!(log.uart_text(0), before, "no extra output expected");
}
