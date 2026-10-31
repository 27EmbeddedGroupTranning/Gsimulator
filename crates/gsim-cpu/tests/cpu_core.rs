//! Core level tests that do not need any cross compiler: a hand assembled
//! Thumb program is loaded straight into flash and executed.

use std::cell::RefCell;
use std::rc::Rc;

use gsim_cpu::{Cpu, StopReason};
use gsim_stm32f1::{map, Soc};

/// Address of the tiny program used by these tests.
const PROGRAM: u32 = map::FLASH_BASE + 0x100;
/// Address of the `b .` handler every exception vector points at.
const HANDLER: u32 = map::FLASH_BASE + 0x200;

fn soc_with_program() -> Rc<RefCell<Soc>> {
    let soc = Rc::new(RefCell::new(Soc::new()));

    // Vector table: SP, reset vector, then every other exception pointing at a
    // harmless `b .` handler so that taken interrupts never run off the map.
    let mut table = Vec::new();
    table.extend_from_slice(&(map::SRAM_BASE + map::SRAM_SIZE).to_le_bytes());
    table.extend_from_slice(&(PROGRAM + 1).to_le_bytes());
    for _ in 2..56 {
        table.extend_from_slice(&(HANDLER + 1).to_le_bytes());
    }

    // 0x100: ldr r0, [pc, #8]   ; GPIOA_ODR   (literal at 0x10C)
    // 0x102: ldr r1, [pc, #12]  ; 0x0000BEEF  (literal at 0x110)
    // 0x104: str r1, [r0]
    // 0x106: b .
    // 0x108: padding
    // 0x10C: .word 0x40010C00
    // 0x110: .word 0x0000BEEF
    let mut code = Vec::new();
    for halfword in [0x4802u16, 0x4903, 0x6001, 0xE7FE, 0xBF00, 0xBF00] {
        code.extend_from_slice(&halfword.to_le_bytes());
    }
    code.extend_from_slice(&(map::gpio::BASE_A + map::gpio::ODR).to_le_bytes());
    code.extend_from_slice(&0x0000_BEEFu32.to_le_bytes());

    let handler = 0xE7FEu16.to_le_bytes();

    soc.borrow_mut()
        .load_image(map::FLASH_BASE, &table)
        .expect("vector table");
    soc.borrow_mut()
        .load_image(PROGRAM, &code)
        .expect("program");
    soc.borrow_mut()
        .load_image(HANDLER, &handler)
        .expect("handler");
    soc
}

fn booted_cpu() -> (Cpu, Rc<RefCell<Soc>>) {
    let soc = soc_with_program();
    let mut cpu = Cpu::new(Rc::clone(&soc)).unwrap();
    cpu.reset().unwrap();
    (cpu, soc)
}

#[test]
fn hand_assembled_program_writes_a_peripheral_register() {
    let (mut cpu, soc) = booted_cpu();
    assert_eq!(cpu.pc(), PROGRAM);
    assert_eq!(cpu.sp(), map::SRAM_BASE + map::SRAM_SIZE);

    let reason = cpu.run(64).unwrap();
    assert_eq!(reason, StopReason::InstructionLimit);
    assert_eq!(
        soc.borrow().gpio[0].odr(),
        0xBEEF,
        "the STR went through the SoC bus into the GPIO output register"
    );
    assert_eq!(cpu.instructions(), 64);
}

#[test]
fn executed_instructions_advance_simulated_time() {
    let (mut cpu, soc) = booted_cpu();
    cpu.run(1_000).unwrap();
    // 1000 cycles at 8 MHz (the reset clock) is 125 us.
    assert_eq!(soc.borrow().now_ns(), 125_000);
}

#[test]
fn breakpoints_stop_the_core_and_can_be_stepped_over() {
    let (mut cpu, soc) = booted_cpu();
    cpu.add_breakpoint(PROGRAM + 4);

    let reason = cpu.run(1_000).unwrap();
    assert!(cpu.halted());
    assert_eq!(reason, StopReason::Breakpoint(PROGRAM + 4));
    assert_eq!(soc.borrow().gpio[0].odr(), 0, "the STR has not run yet");

    cpu.resume();
    cpu.run(1_000).unwrap();
    assert_eq!(soc.borrow().gpio[0].odr(), 0xBEEF);

    cpu.remove_breakpoint(PROGRAM + 4);
    assert!(!cpu.halted());
}

#[test]
fn peripheral_irqs_reach_the_core_when_the_nvic_allows_it() {
    let (mut cpu, soc) = booted_cpu();

    // Without an enabled line nothing may be requested.
    cpu.pend_interrupt(gsim_core::irq::lines::TIM3);
    cpu.run(10).unwrap();
    assert_eq!(cpu.interrupts(), 0);

    // Enable IRQ 29 (TIM3) in the core's NVIC and let the arbitration pick it up.
    cpu.write_core_register(0xE000_E100, 1 << 29).unwrap();
    cpu.pend_interrupt(gsim_core::irq::lines::TIM3);
    cpu.run(64).unwrap();

    assert_eq!(cpu.interrupts(), 1);
    assert_eq!(cpu.pc(), HANDLER, "the core vectored to the TIM3 entry");
    assert_eq!(
        soc.borrow().gpio[0].odr(),
        0xBEEF,
        "the interrupted program had already completed its store"
    );
}

#[test]
fn a_debugger_can_read_and_write_the_register_file() {
    let (mut cpu, _soc) = booted_cpu();
    assert!(cpu.set_register(0, 0x1234_5678));
    assert_eq!(cpu.register(0), Some(0x1234_5678));

    assert!(cpu.set_register(15, PROGRAM + 6));
    assert_eq!(cpu.pc(), PROGRAM + 6);

    cpu.set_xpsr(0x0100_0000);
    assert_eq!(cpu.xpsr(), 0x0100_0000);

    // r13 writes land in the stack pointer the core currently uses.
    let stack = map::SRAM_BASE + 0x800;
    assert!(cpu.set_register(13, stack));
    assert_eq!(cpu.sp(), stack);

    assert!(!cpu.set_register(16, 0), "xPSR is not in the main file");
}

#[test]
fn a_halt_request_stops_the_core_and_a_step_runs_one_instruction() {
    let (mut cpu, _soc) = booted_cpu();
    cpu.halt();
    assert!(cpu.halted());
    assert_eq!(cpu.stop_reason(), Some(StopReason::Debug));
    assert_eq!(cpu.run(100).unwrap(), StopReason::Debug);
    assert_eq!(cpu.instructions(), 0);

    cpu.resume();
    cpu.step_instruction().unwrap();
    assert_eq!(cpu.instructions(), 1);
    assert_eq!(cpu.pc(), PROGRAM + 2);
    assert!(!cpu.halted());

    // A breakpoint sitting on the current pc must not block the step itself,
    // which is exactly what a debugger asks for.
    cpu.add_breakpoint(PROGRAM + 2);
    cpu.resume();
    cpu.step_instruction().unwrap();
    assert_eq!(cpu.pc(), PROGRAM + 4);
}

#[test]
fn every_line_enabled_by_the_firmware_stays_enabled() {
    // A firmware that enables several interrupt lines with one store each, the
    // way `HAL_NVIC_EnableIRQ()` does. The core model keeps only the last value
    // written to `NVIC_ISER`, so the front end has to rebuild the sticky set.
    let soc = Rc::new(RefCell::new(Soc::new()));
    let mut table = Vec::new();
    table.extend_from_slice(&(map::SRAM_BASE + map::SRAM_SIZE).to_le_bytes());
    table.extend_from_slice(&(PROGRAM + 1).to_le_bytes());
    for _ in 2..56 {
        table.extend_from_slice(&(HANDLER + 1).to_le_bytes());
    }

    // ldr r0, [pc, #128]   ; NVIC_ISER0
    // ldr r1, [pc, #132]   ; 1 << 29
    // str r1, [r0]         ; enable IRQ 29
    // 60 x nop
    // ldr r1, [pc, #8]     ; 1 << 30
    // str r1, [r0]         ; enable IRQ 30
    // b .
    const NOPS: usize = 60;
    let mut code = Vec::new();
    for halfword in [0x4820u16, 0x4921, 0x6001] {
        code.extend_from_slice(&halfword.to_le_bytes());
    }
    for _ in 0..NOPS {
        code.extend_from_slice(&0xBF00u16.to_le_bytes());
    }
    for halfword in [0x4902u16, 0x6001, 0xE7FE] {
        code.extend_from_slice(&halfword.to_le_bytes());
    }
    code.extend_from_slice(&0xE000_E100u32.to_le_bytes());
    code.extend_from_slice(&(1u32 << 29).to_le_bytes());
    code.extend_from_slice(&(1u32 << 30).to_le_bytes());

    soc.borrow_mut()
        .load_image(map::FLASH_BASE, &table)
        .expect("vector table");
    soc.borrow_mut()
        .load_image(PROGRAM, &code)
        .expect("program");
    soc.borrow_mut()
        .load_image(HANDLER, &0xE7FEu16.to_le_bytes())
        .expect("handler");

    let mut cpu = Cpu::new(Rc::clone(&soc)).unwrap();
    cpu.reset().unwrap();
    cpu.run(1_000).unwrap();

    // Line 29 was written first and is the one a "last value wins" register
    // would have lost.
    cpu.pend_interrupt(gsim_core::irq::lines::TIM3);
    cpu.run(64).unwrap();
    assert_eq!(cpu.interrupts(), 1);
    assert_eq!(cpu.pc(), HANDLER);
}
