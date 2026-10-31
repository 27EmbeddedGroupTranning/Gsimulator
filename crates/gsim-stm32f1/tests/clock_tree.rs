//! Clock tree behaviour observed through `RCC` registers, the way the HAL
//! `SystemClock_Config()` drives it.

mod common;

use common::*;
use gsim_core::{AccessWidth, MemoryBus};
use gsim_stm32f1::{map, Soc};

#[test]
fn power_on_runs_from_the_internal_oscillator() {
    let soc = Soc::new();
    let clocks = soc.clocks();
    assert_eq!(clocks.sysclk_hz, 8_000_000);
    assert_eq!(clocks.hclk_hz, 8_000_000);
    assert_eq!(clocks.pclk1_hz, 8_000_000);
    assert_eq!(clocks.pclk2_hz, 8_000_000);
    assert_eq!(clocks.adc_hz, 4_000_000);
    assert_eq!(clocks.rtc_hz, 0, "no RTC clock source selected yet");
}

#[test]
fn hse_and_pll_reach_72mhz_with_realistic_start_up_delays() {
    let mut soc = Soc::new();
    let rcc = map::rcc::BASE;

    // Before HSE has started the ready flag must stay clear.
    w32(&mut soc, rcc + map::rcc::CR, (1 << 16) | 1);
    assert_eq!(r32(&mut soc, rcc + map::rcc::CR) & (1 << 17), 0);
    soc.advance_ns(1_000_000);
    assert_eq!(r32(&mut soc, rcc + map::rcc::CR) & (1 << 17), 0);
    soc.advance_ns(1_500_000);
    assert_ne!(r32(&mut soc, rcc + map::rcc::CR) & (1 << 17), 0);

    configure_72mhz(&mut soc);
    let clocks = soc.clocks();
    assert_eq!(clocks.sysclk_hz, 72_000_000);
    assert_eq!(clocks.hclk_hz, 72_000_000);
    assert_eq!(clocks.pclk1_hz, 36_000_000);
    assert_eq!(clocks.pclk2_hz, 72_000_000);
    assert_eq!(clocks.tim1_hz, 72_000_000);
    assert_eq!(clocks.tim2_hz, 72_000_000, "APB1 timer clock is doubled");
    assert_eq!(clocks.adc_hz, 12_000_000);
}

#[test]
fn peripheral_gating_controls_the_clock_output() {
    let mut soc = Soc::new();
    assert_eq!(soc.peripheral_clock(map::usart::BASE_1), 0);

    enable_apb2(&mut soc, 1 << 14);
    assert_eq!(soc.peripheral_clock(map::usart::BASE_1), 8_000_000);

    enable_apb2(&mut soc, 1 << 2 | 1 << 0);
    assert_eq!(soc.peripheral_clock(map::gpio::BASE_A), 8_000_000);
    assert_eq!(soc.peripheral_clock(map::afio::BASE), 8_000_000);
}

#[test]
fn strict_gating_rejects_access_to_unclocked_peripherals() {
    let mut soc = Soc::new();
    soc.set_strict_clock_gating(true);
    let status = map::usart::BASE_1 + map::usart::SR;
    assert!(soc.read(status, AccessWidth::Word).is_err());

    enable_apb2(&mut soc, 1 << 14);
    assert!(soc.read(status, AccessWidth::Word).is_ok());
}

#[test]
fn apb_reset_registers_reset_the_target_peripheral() {
    let mut soc = Soc::new();
    enable_apb2(&mut soc, 1 << 14);
    // Program a baud rate and enable the transmitter.
    w32(&mut soc, map::usart::BASE_1 + map::usart::BRR, 625);
    w32(
        &mut soc,
        map::usart::BASE_1 + map::usart::CR1,
        (1 << 3) | (1 << 13),
    );
    assert_eq!(r32(&mut soc, map::usart::BASE_1 + map::usart::BRR), 625);

    w32(&mut soc, map::rcc::BASE + map::rcc::APB2RSTR, 1 << 14);
    assert_eq!(
        r32(&mut soc, map::usart::BASE_1 + map::usart::BRR),
        0,
        "USART1 was reset"
    );
    assert_eq!(r32(&mut soc, map::usart::BASE_1 + map::usart::CR1), 0);
}

#[test]
fn systick_uses_the_core_clock() {
    let mut soc = Soc::new();
    // 1 ms tick from the 8 MHz HCLK.
    w32(&mut soc, 0xE000_E014, 7_999);
    w32(&mut soc, 0xE000_E010, 0b111);
    soc.advance_ns(1_000_000);
    assert!(
        soc.nvic
            .is_system_pending(gsim_core::irq::exception::SYSTICK),
        "SysTick exception should be pending after 1 ms"
    );
    assert_eq!(
        soc.pending_exception(),
        Some(gsim_core::irq::exception::SYSTICK)
    );
}
