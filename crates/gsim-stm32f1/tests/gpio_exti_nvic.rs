//! GPIO, AFIO and EXTI wired together, ending in an NVIC pending exception.

mod common;

use common::*;
use gsim_core::irq::{exception, lines};
use gsim_stm32f1::{map, Soc};

fn setup_exti0() -> Soc {
    let mut soc = Soc::new();
    enable_apb2(&mut soc, (1 << 0) | (1 << 2) | (1 << 3)); // AFIO, GPIOA, GPIOB

    // PA0 is a pull-down input, so it starts low.
    set_pin_nibble(&mut soc, 0, 0, PIN_INPUT_PULL);
    w32(&mut soc, map::gpio::BASE_A + map::gpio::BSRR, 1 << 16);
    // PB1 is a pull-up input, so it starts high.
    set_pin_nibble(&mut soc, 1, 1, PIN_INPUT_PULL);
    w32(&mut soc, map::gpio::BASE_B + map::gpio::BSRR, 1 << 1);

    // EXTI0 -> port A, EXTI1 -> port B.
    w32(&mut soc, map::afio::BASE + map::afio::EXTICR1, 0b0001 << 4);

    // Both lines trigger on rising edge and are unmasked.
    w32(&mut soc, map::exti::BASE + map::exti::RTSR, 0b11);
    w32(&mut soc, map::exti::BASE + map::exti::IMR, 0b11);

    // Enable them in the NVIC with distinct priorities.
    soc.nvic.enable(lines::EXTI0, true);
    soc.nvic.enable(lines::EXTI1, true);
    soc.nvic.set_priority(lines::EXTI0, 0x20);
    soc.nvic.set_priority(lines::EXTI1, 0x40);
    soc
}

/// Creates a genuine rising edge on a pin that is otherwise held high.
fn pulse_high_pin(soc: &mut Soc, port: usize, pin: u8) {
    soc.gpio_set_input(port, pin, false);
    soc.advance_ns(2_000);
    soc.gpio_set_input(port, pin, true);
    soc.advance_ns(2_000);
}

#[test]
fn rising_edge_on_a_pin_pends_the_exti_interrupt() {
    let mut soc = setup_exti0();

    soc.gpio_set_input(0, 0, true);
    soc.advance_ns(2_000);

    assert_ne!(r32(&mut soc, map::exti::BASE + map::exti::PR) & 1, 0);
    assert!(soc.nvic.is_pending(lines::EXTI0));
    assert_eq!(soc.pending_exception(), Some(lines::EXTI0.exception()));
}

#[test]
fn handler_entry_and_exit_manage_the_active_stack() {
    let mut soc = setup_exti0();
    soc.gpio_set_input(0, 0, true);
    soc.advance_ns(2_000);

    let exc = lines::EXTI0.exception();
    assert_eq!(soc.pending_exception(), Some(exc));
    soc.enter_exception(exc);
    assert_eq!(soc.nvic.nesting_depth(), 1);
    assert!(soc.nvic.is_active(lines::EXTI0));
    assert_eq!(soc.pending_exception(), None, "handler is running");

    // The ISR acknowledges the line.
    w32(&mut soc, map::exti::BASE + map::exti::PR, 1);
    soc.exit_exception(exc);
    assert_eq!(soc.nvic.nesting_depth(), 0);
    assert!(!soc.nvic.is_pending(lines::EXTI0));
}

#[test]
fn nested_interrupt_preempts_a_lower_priority_handler() {
    let mut soc = setup_exti0();
    // Both lines fire: EXTI0 (priority 0x20) is higher than EXTI1 (0x40).
    soc.gpio_set_input(0, 0, true);
    soc.advance_ns(2_000);
    pulse_high_pin(&mut soc, 1, 1);

    let low = lines::EXTI1.exception();
    let high = lines::EXTI0.exception();
    soc.enter_exception(low);
    // While servicing EXTI1 the higher priority EXTI0 must still be taken.
    assert_eq!(soc.pending_exception(), Some(high));
    soc.enter_exception(high);
    assert_eq!(soc.nvic.nesting_depth(), 2);
    soc.exit_exception(high);
    soc.exit_exception(low);
    assert_eq!(soc.nvic.nesting_depth(), 0);
}

#[test]
fn output_pins_report_their_level_changes() {
    let mut soc = Soc::new();
    enable_apb2(&mut soc, 1 << 2);
    let log = EventLog::new();
    log.attach(&mut soc);

    set_pin_nibble(&mut soc, 0, 5, PIN_OUTPUT_PP_2MHZ);
    w32(&mut soc, map::gpio::BASE_A + map::gpio::BSRR, 1 << 5);
    assert_ne!(
        r32(&mut soc, map::gpio::BASE_A + map::gpio::IDR) & (1 << 5),
        0
    );

    w32(&mut soc, map::gpio::BASE_A + map::gpio::BRR, 1 << 5);
    assert_eq!(
        r32(&mut soc, map::gpio::BASE_A + map::gpio::IDR) & (1 << 5),
        0
    );

    let pins: Vec<_> = log
        .events()
        .into_iter()
        .filter(|event| matches!(event, gsim_core::Event::PinOutput { .. }))
        .collect();
    assert_eq!(pins.len(), 2, "one event per level change: {pins:?}");
}

#[test]
fn software_interrupt_request_pends_without_any_pin_activity() {
    let mut soc = setup_exti0();
    w32(&mut soc, map::exti::BASE + map::exti::SWIER, 1 << 1);
    assert_eq!(soc.pending_exception(), Some(lines::EXTI1.exception()));
}

#[test]
fn systick_and_exti_share_the_same_arbitration() {
    let mut soc = setup_exti0();
    w32(&mut soc, 0xE000_E014, 999);
    w32(&mut soc, 0xE000_E010, 0b111);
    soc.nvic.set_system_priority(exception::SYSTICK, 0x10);
    soc.gpio_set_input(0, 0, true);

    soc.advance_ns(200_000);
    assert_eq!(
        soc.pending_exception(),
        Some(exception::SYSTICK),
        "SysTick has the higher priority of the two"
    );
}
