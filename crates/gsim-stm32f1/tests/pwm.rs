//! Compare outputs reach the GPIO pins their channels are mapped to, which is
//! what makes a PWM channel visible on the schematic.

mod common;

use common::*;
use gsim_core::Event;
use gsim_stm32f1::{map, Soc};

/// Configures TIM3 channel 1 as a 1 kHz PWM at `percent` duty on PA6.
fn tim3_pwm_on_pa6(soc: &mut Soc, percent: u32, nibble: u8) {
    enable_apb1(soc, 1 << 1); // TIM3EN
    enable_apb2(soc, 1 << 2); // GPIOAEN
    set_pin_nibble(soc, 0, 6, nibble);
    let base = map::tim::BASE_3;
    w16(soc, base + map::tim::PSC, 7); // 8 MHz / 8 = 1 MHz counter clock
    w16(soc, base + map::tim::ARR, 999); // 1 kHz period
    w16(soc, base + map::tim::CCR1, (1000 * percent / 100) as u16);
    w16(soc, base + map::tim::CCMR1, 0b110 << 4); // PWM mode 1
    w16(soc, base + map::tim::CCER, 1); // CC1E
    w16(soc, base + map::tim::CR1, 1); // CEN
}

#[test]
fn a_pwm_channel_reaches_its_alternate_function_pin() {
    let mut soc = Soc::new();
    tim3_pwm_on_pa6(&mut soc, 25, PIN_AF_PP_50MHZ);
    assert_eq!(soc.tim_channel_pin(2, 0), Some((0, 6)));

    let log = EventLog::new();
    log.attach(&mut soc);

    soc.advance_ns(2_000_000);
    let duty = soc.pin_duty(0, 6).expect("PA6 carries TIM3_CH1");
    assert!((duty - 0.25).abs() < 0.02, "measured duty was {duty}");

    // The pin really carries the square wave: five periods, two edges each.
    let mut edges = 0;
    let mut last = soc.gpio[0].input_level(6);
    for _ in 0..500 {
        soc.advance_ns(10_000);
        let level = soc.gpio[0].input_level(6);
        if level != last {
            edges += 1;
            last = level;
        }
    }
    assert!(
        edges >= 8,
        "expected a 1 kHz square wave, saw {edges} edges"
    );

    let reported = log
        .events()
        .iter()
        .filter(|event| {
            matches!(
                event,
                Event::PinOutput {
                    port: 0,
                    pin: 6,
                    ..
                }
            )
        })
        .count();
    assert!(
        reported >= 8,
        "the front end needs the pin edges, saw {reported}"
    );
}

#[test]
fn changing_the_compare_value_dims_the_pin() {
    let mut soc = Soc::new();
    tim3_pwm_on_pa6(&mut soc, 25, PIN_AF_PP_50MHZ);
    soc.advance_ns(5_000_000);
    assert!(soc.pin_duty(0, 6).is_some_and(|duty| duty < 0.35));

    w16(&mut soc, map::tim::BASE_3 + map::tim::CCR1, 750);
    soc.advance_ns(20_000_000);
    let duty = soc.pin_duty(0, 6).expect("PA6 carries TIM3_CH1");
    assert!((duty - 0.75).abs() < 0.03, "measured duty was {duty}");
}

#[test]
fn a_plain_output_pin_keeps_its_latch() {
    let mut soc = Soc::new();
    tim3_pwm_on_pa6(&mut soc, 50, PIN_OUTPUT_PP_2MHZ);
    soc.advance_ns(5_000_000);
    assert_eq!(soc.pin_duty(0, 6), None, "the pin is not an AF output");
    assert!(!soc.gpio[0].input_level(6), "ODR stays the only driver");
}

#[test]
fn the_afio_remap_moves_the_channel_to_another_port() {
    let mut soc = Soc::new();
    enable_apb2(&mut soc, 1 << 0); // AFIOEN
    assert_eq!(soc.tim_channel_pin(2, 0), Some((0, 6)));
    // TIM3 full remap: CH1..CH4 move to PC6..PC9.
    w32(&mut soc, map::afio::BASE + map::afio::MAPR, 0b11 << 10);
    assert_eq!(soc.tim_channel_pin(2, 0), Some((2, 6)));
    assert_eq!(soc.tim_channel_pin(2, 3), Some((2, 9)));
    // TIM4 remap is a single bit: CH1..CH4 move to PD12..PD15.
    assert_eq!(soc.tim_channel_pin(3, 0), Some((1, 6)));
    w32(
        &mut soc,
        map::afio::BASE + map::afio::MAPR,
        (0b11 << 10) | (1 << 12),
    );
    assert_eq!(soc.tim_channel_pin(3, 0), Some((3, 12)));
}

#[test]
fn the_advanced_timer_needs_its_main_output_enabled() {
    let mut soc = Soc::new();
    enable_apb2(&mut soc, (1 << 11) | (1 << 2)); // TIM1EN | GPIOAEN
    set_pin_nibble(&mut soc, 0, 8, PIN_AF_PP_50MHZ);
    let base = map::tim::BASE_1;
    w16(&mut soc, base + map::tim::PSC, 7);
    w16(&mut soc, base + map::tim::ARR, 999);
    w16(&mut soc, base + map::tim::CCR1, 500);
    w16(&mut soc, base + map::tim::CCMR1, 0b110 << 4);
    w16(&mut soc, base + map::tim::CCER, 1);
    w16(&mut soc, base + map::tim::CR1, 1);
    assert_eq!(soc.tim_channel_pin(0, 0), Some((0, 8)));

    soc.advance_ns(2_000_000);
    assert_eq!(soc.pin_duty(0, 8), None, "MOE is still cleared");

    // BDTR.MOE, the way `HAL_TIM_PWM_Start()` enables an advanced timer.
    w16(&mut soc, base + map::tim::BDTR, 1 << 15);
    soc.advance_ns(20_000_000);
    let duty = soc.pin_duty(0, 8).expect("PA8 carries TIM1_CH1");
    assert!((duty - 0.5).abs() < 0.03, "measured duty was {duty}");
}
