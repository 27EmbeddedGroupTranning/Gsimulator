//! End-to-end tests for the configurable built-in stimulus: pin choice,
//! square-wave timing, constant levels and custom expressions.

use gsimulator_gui::sim::{Simulation, Source, StimulusConfig, StimulusWaveform};

/// Advances simulated time by `ns` (the SoC runs at 8 MHz after power-on).
fn advance_ns(sim: &mut Simulation, ns: u64) {
    let sysclk = u64::from(sim.clocks().sysclk_hz.max(1));
    let budget = (ns.saturating_mul(sysclk) / 1_000_000_000).max(1);
    sim.advance_frame(budget as u32);
}

fn square(port: u8, pin: u8, half_period_ns: u64, duty: f32) -> StimulusConfig {
    StimulusConfig {
        port,
        pin,
        waveform: StimulusWaveform::Square {
            half_period_ns,
            duty,
        },
    }
}

#[test]
fn the_stimulus_drives_the_configured_pin() {
    let mut sim = Simulation::new();
    sim.set_stimulus_config(square(1, 13, 10_000_000, 0.5))
        .expect("config must be valid");
    sim.start_stimulus();

    assert!(
        sim.pin_levels().is_output(1, 13),
        "PB13 must be driven as an output"
    );
    assert!(
        !sim.pin_levels().is_output(0, 5),
        "the previously used PA5 must be released"
    );

    let mut saw_high = false;
    let mut saw_low = false;
    for _ in 0..10 {
        advance_ns(&mut sim, 2_000_000);
        let level = sim.pin_levels().level_of(1, 13);
        saw_high |= level;
        saw_low |= !level;
    }
    assert!(saw_high, "PB13 never went high");
    assert!(saw_low, "PB13 never went low");
    assert!(sim.snapshot().source_label.contains("PB13"));
}

#[test]
fn the_duty_cycle_controls_the_high_time() {
    let mut sim = Simulation::new();
    // 20 ms period, 25 % duty -> 5 ms high, 15 ms low.
    sim.set_stimulus_config(square(0, 5, 10_000_000, 0.25))
        .expect("config must be valid");
    sim.start_stimulus();

    advance_ns(&mut sim, 3_000_000);
    assert!(sim.pin_levels().level_of(0, 5), "must be high at 3 ms");
    advance_ns(&mut sim, 4_000_000);
    assert!(!sim.pin_levels().level_of(0, 5), "must be low at 7 ms");
    advance_ns(&mut sim, 15_000_000);
    assert!(
        sim.pin_levels().level_of(0, 5),
        "must be high again at 22 ms"
    );
    advance_ns(&mut sim, 4_000_000);
    assert!(!sim.pin_levels().level_of(0, 5), "must be low at 26 ms");
}

#[test]
fn constant_levels_never_toggle() {
    let mut sim = Simulation::new();
    sim.set_stimulus_config(StimulusConfig {
        port: 0,
        pin: 6,
        waveform: StimulusWaveform::Level(true),
    })
    .expect("config must be valid");
    sim.start_stimulus();
    for _ in 0..5 {
        advance_ns(&mut sim, 10_000_000);
        assert!(sim.pin_levels().level_of(0, 6), "the level must stay high");
    }
    assert!(sim
        .snapshot()
        .stimulus
        .as_ref()
        .expect("stimulus is running")
        .next_toggle_ns
        .is_none());

    sim.set_stimulus_config(StimulusConfig {
        port: 0,
        pin: 6,
        waveform: StimulusWaveform::Level(false),
    })
    .expect("config must be valid");
    sim.start_stimulus();
    for _ in 0..5 {
        advance_ns(&mut sim, 10_000_000);
        assert!(!sim.pin_levels().level_of(0, 6), "the level must stay low");
    }
}

#[test]
fn a_custom_function_drives_the_pin() {
    let mut sim = Simulation::new();
    sim.set_stimulus_config(StimulusConfig {
        port: 2,
        pin: 13,
        waveform: StimulusWaveform::Function {
            expression: "t >= 0.1 && t < 0.15".to_string(),
        },
    })
    .expect("config must be valid");
    sim.start_stimulus();

    advance_ns(&mut sim, 50_000_000);
    assert!(!sim.pin_levels().level_of(2, 13), "low before 100 ms");
    advance_ns(&mut sim, 60_000_000);
    assert!(sim.pin_levels().level_of(2, 13), "high inside the window");
    advance_ns(&mut sim, 60_000_000);
    assert!(!sim.pin_levels().level_of(2, 13), "low after 150 ms");
}

#[test]
fn invalid_settings_are_rejected_and_keep_the_running_waveform() {
    let mut sim = Simulation::new();
    sim.start_stimulus();
    let before = sim.stimulus().clone();

    assert!(sim
        .set_stimulus_config(StimulusConfig {
            port: 9,
            pin: 0,
            ..StimulusConfig::default()
        })
        .is_err());
    assert!(sim
        .set_stimulus_config(StimulusConfig {
            port: 2,
            pin: 5,
            ..StimulusConfig::default()
        })
        .is_err());
    assert!(sim
        .set_stimulus_config(StimulusConfig {
            waveform: StimulusWaveform::Function {
                expression: "1 +".to_string(),
            },
            ..StimulusConfig::default()
        })
        .is_err());

    assert_eq!(sim.stimulus(), &before);
    assert!(sim.is_running());
    assert!(matches!(sim.source(), Source::Stimulus));
}

#[test]
fn stopping_releases_the_configured_pin_but_keeps_the_clock() {
    let mut sim = Simulation::new();
    sim.set_stimulus_config(square(1, 13, 5_000_000, 0.5))
        .expect("config must be valid");
    sim.start_stimulus();
    sim.stop_stimulus();

    assert!(matches!(sim.source(), Source::Idle));
    assert!(
        sim.is_running(),
        "the clock is independent of the stimulus and keeps running"
    );
    assert!(!sim.pin_levels().is_output(1, 13));
    assert!(!sim.pin_levels().level_of(1, 13));

    let before = sim.now_ns();
    advance_ns(&mut sim, 10_000_000);
    assert!(
        sim.now_ns() > before,
        "an idle simulation must still advance its clock"
    );
}

#[test]
fn an_idle_simulation_can_be_started_and_paused() {
    let mut sim = Simulation::new();
    sim.stop_stimulus();
    sim.set_running(false);

    let frozen = sim.now_ns();
    sim.advance_frame(400_000);
    assert_eq!(sim.now_ns(), frozen, "paused means paused");

    sim.set_running(true);
    let before = sim.now_ns();
    sim.advance_frame(400_000);
    assert!(
        sim.now_ns() > before,
        "an idle simulation must run when started"
    );
    assert!(matches!(sim.source(), Source::Idle));
}

#[test]
fn reconfiguring_while_running_keeps_running() {
    let mut sim = Simulation::new();
    sim.set_stimulus_config(square(1, 13, 10_000_000, 0.5))
        .expect("config must be valid");
    sim.start_stimulus();
    assert!(sim.is_running());
    assert_eq!(sim.stimulus().pin, 13);
    assert!(sim.pin_levels().is_output(1, 13));

    sim.set_running(false);
    sim.set_stimulus_config(square(1, 14, 10_000_000, 0.5))
        .expect("config must be valid");
    assert!(!sim.is_running(), "a paused stimulus must stay paused");
    assert_eq!(sim.stimulus().pin, 14);
}

#[test]
fn a_square_stimulus_reports_its_duty_so_an_led_dims() {
    let mut sim = Simulation::new();
    // 20 ms period at 25 %: the pin is high a quarter of the time.
    sim.set_stimulus_config(square(0, 5, 10_000_000, 0.25))
        .expect("config must be valid");
    sim.start_stimulus();

    advance_ns(&mut sim, 7_000_000);
    let levels = sim.pin_levels();
    assert!(!levels.level_of(0, 5), "the sampled instant is low");
    assert!(
        (levels.duty_of(0, 5) - 0.25).abs() < 1e-6,
        "the waveform duty must be visible, got {}",
        levels.duty_of(0, 5)
    );
    assert_eq!(levels.duty_of(0, 6), 0.0, "idle pins hold no duty");

    sim.stop_stimulus();
    assert_eq!(
        sim.pin_levels().duty_of(0, 5),
        0.0,
        "a released pin stops modulating"
    );
}
