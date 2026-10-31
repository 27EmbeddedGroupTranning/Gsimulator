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
