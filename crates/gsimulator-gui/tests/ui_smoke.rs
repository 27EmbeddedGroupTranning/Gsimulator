//! Headless end-to-end test: the built-in stimulus drives PA5, the schematic
//! wiring made in the EDA-style canvas decides whether the LED is lit, and the
//! whole window renders without a display.

use eframe::egui;
use gsimulator_gui::app::App;
use gsimulator_gui::board::{Board, DeviceKind, PinLevels, PinRef, POWER_PIN};
use gsimulator_gui::canvas::Schematic;
use gsimulator_gui::sim::{Simulation, Source, StimulusConfig, StimulusWaveform};

fn raw_input() -> egui::RawInput {
    egui::RawInput {
        screen_rect: Some(egui::Rect::from_min_size(
            egui::Pos2::ZERO,
            egui::Vec2::new(1440.0, 900.0),
        )),
        ..Default::default()
    }
}

#[test]
fn the_stimulus_dims_the_demo_led_by_its_duty_cycle() {
    let mut sim = Simulation::new();
    sim.start_stimulus();
    let board = Board::demo();
    let led = board.first_of(DeviceKind::Led).expect("demo LED").id;

    let mut saw_high = false;
    let mut saw_low = false;
    for _ in 0..40 {
        sim.advance_frame(400_000);
        if sim.pin_levels().level_of(0, 5) {
            saw_high = true;
        } else {
            saw_low = true;
        }
    }
    assert!(saw_high, "the stimulus never drove PA5 high");
    assert!(saw_low, "the stimulus never drove PA5 low");

    // The default stimulus is a 50 % square wave, so the LED sits at half
    // brightness instead of flickering with the sampled instant.
    let brightness = board.evaluate(&sim.pin_levels()).led_brightness(led);
    assert!((brightness - 0.5).abs() < 1e-6, "brightness {brightness}");

    // A constant level is a 0 %/100 % duty: fully dark, fully on.
    for (waveform, expected) in [
        (StimulusWaveform::Level(false), 0.0),
        (StimulusWaveform::Level(true), 1.0),
    ] {
        sim.set_stimulus_config(StimulusConfig {
            port: 0,
            pin: 5,
            waveform,
        })
        .expect("config must be valid");
        sim.start_stimulus();
        sim.advance_frame(400_000);
        let brightness = board.evaluate(&sim.pin_levels()).led_brightness(led);
        assert!(
            (brightness - expected).abs() < 1e-6,
            "expected {expected}, got {brightness}"
        );
    }
}

#[test]
fn stopping_the_stimulus_releases_the_pin_but_keeps_the_clock() {
    let mut sim = Simulation::new();
    sim.start_stimulus();
    sim.advance_frame(400_000);
    assert!(sim.now_ns() > 0, "the stimulus must drive the clock");

    sim.stop_stimulus();
    assert!(matches!(sim.source(), Source::Idle));
    assert!(
        sim.is_running(),
        "stopping the driver must not pause the clock"
    );
    assert!(!sim.pin_levels().is_output(0, 5), "PA5 must be released");
    assert!(!sim.pin_levels().level_of(0, 5), "PA5 must be driven low");

    // The clock is independent of the stimulus and keeps advancing.
    let before = sim.now_ns();
    sim.advance_frame(400_000);
    assert!(
        sim.now_ns() > before,
        "an idle simulation must still advance its clock"
    );

    // Pausing still freezes it.
    sim.set_running(false);
    let frozen = sim.now_ns();
    sim.advance_frame(400_000);
    assert_eq!(sim.now_ns(), frozen);

    // A reset with no driver must not switch the stimulus back on.
    sim.reset();
    assert!(matches!(sim.source(), Source::Idle));
    assert!(!sim.is_running(), "a paused clock stays paused after reset");

    // It can be turned back on, and the clock follows the run state again.
    sim.start_stimulus();
    assert!(matches!(sim.source(), Source::Stimulus));
    assert!(sim.is_running());
    let before = sim.now_ns();
    sim.advance_frame(400_000);
    assert!(sim.now_ns() > before);
}

#[test]
fn clicking_two_pins_creates_a_wire_like_an_eda_tool() {
    let mut board = Board::new();
    let led = board.add_led(egui::Pos2::new(600.0, 120.0));
    let mut schematic = Schematic::default();

    // First click only arms the wire.
    schematic.click_pin(&mut board, PinRef::Mcu { port: 0, pin: 5 });
    assert_eq!(board.wires().len(), 0);
    assert_eq!(
        schematic.wire_source(),
        Some(PinRef::Mcu { port: 0, pin: 5 })
    );

    // Second click completes it.
    schematic.click_pin(
        &mut board,
        PinRef::Device {
            device: led,
            pin: 0,
        },
    );
    assert_eq!(board.wires().len(), 1);
    assert!(board.is_connected(
        PinRef::Mcu { port: 0, pin: 5 },
        PinRef::Device {
            device: led,
            pin: 0
        }
    ));
    assert!(schematic.wire_source().is_none());

    // A duplicate connection is refused instead of duplicated.
    schematic.click_pin(&mut board, PinRef::Mcu { port: 0, pin: 5 });
    schematic.click_pin(
        &mut board,
        PinRef::Device {
            device: led,
            pin: 0,
        },
    );
    assert_eq!(board.wires().len(), 1);

    // Cancel leaves the board untouched.
    schematic.click_pin(
        &mut board,
        PinRef::Device {
            device: led,
            pin: 1,
        },
    );
    schematic.cancel_wire();
    assert_eq!(board.wires().len(), 1);
}

#[test]
fn schematic_renders_every_element_without_a_display() {
    let ctx = egui::Context::default();
    let mut board = Board::demo();
    let mut schematic = Schematic::default();
    // A short between VCC and GND plus an extra LED cover the short-warning and
    // partially wired paint paths as well.
    let vcc = board.first_of(DeviceKind::Vcc).expect("demo VCC").id;
    let gnd = board.first_of(DeviceKind::Gnd).expect("demo GND").id;
    board.connect(
        PinRef::Device {
            device: vcc,
            pin: POWER_PIN,
        },
        PinRef::Device {
            device: gnd,
            pin: POWER_PIN,
        },
    );
    // The new parts are painted too.
    board.add_can(egui::Pos2::new(200.0, 620.0));
    board.add_matrix(egui::Pos2::new(360.0, 620.0));
    let extra = board.add_led(egui::Pos2::new(600.0, 420.0));
    board.connect(
        PinRef::Mcu { port: 2, pin: 13 },
        PinRef::Device {
            device: extra,
            pin: 0,
        },
    );
    let levels = PinLevels::default();
    let state = board.evaluate(&levels);

    let output = ctx.run(raw_input(), |ctx| {
        egui::CentralPanel::default().show(ctx, |ui| {
            schematic.show(ui, &mut board, &levels, &state, 0);
        });
    });
    assert!(!output.shapes.is_empty(), "nothing was painted");
}

#[test]
fn the_device_panel_shows_the_resistor_value_and_its_effect() {
    use gsimulator_gui::board::{
        CAN_CANH, CAN_CANL, RESISTOR_A, RESISTOR_B, SCOPE_CANH, SCOPE_CANL,
    };

    let ctx = egui::Context::default();
    let mut app = App::with_context(&ctx, None);
    let (resistor, transceiver, scope) = {
        let board = app.board_mut();
        let transceiver = board.add_can(egui::Pos2::new(480.0, 200.0));
        let resistor = board.add_resistor(egui::Pos2::new(300.0, 320.0));
        let scope = board.add_scope(egui::Pos2::new(700.0, 320.0));
        let pin = |device: u32, pin: u8| PinRef::Device { device, pin };
        // The resistor bridges CANH and CANL, the probe taps the same pair.
        board.connect(pin(transceiver, CAN_CANH), pin(resistor, RESISTOR_A));
        board.connect(pin(resistor, RESISTOR_B), pin(transceiver, CAN_CANL));
        board.connect(pin(transceiver, CAN_CANH), pin(scope, SCOPE_CANH));
        board.connect(pin(transceiver, CAN_CANL), pin(scope, SCOPE_CANL));
        (resistor, transceiver, scope)
    };
    let _ = (transceiver, scope);
    app.refresh();

    // One 120 Ω part across the pair: the panel reports the load it creates.
    let network = app.board().can_network();
    assert_eq!(network.terminator_ids, vec![resistor]);
    assert!(network.quality() > 0.4 && network.quality() < 1.0);

    // The whole window renders with the resistor row and its value controls.
    let output = ctx.run(raw_input(), |ctx| app.draw(ctx));
    assert!(!output.shapes.is_empty(), "the app painted nothing");
}

#[test]
fn the_device_panel_turns_the_can_motor_from_a_frame_on_the_bus() {
    use gsimulator_gui::board::{
        CAN_CANH, CAN_CANL, MOTOR_CANH, MOTOR_CANL, MOTOR_GND, MOTOR_VCC, POWER_PIN,
    };

    let ctx = egui::Context::default();
    let mut app = App::with_context(&ctx, None);
    let motor = {
        let board = app.board_mut();
        let transceiver = board.add_can(egui::Pos2::new(480.0, 200.0));
        let motor = board.add_motor(egui::Pos2::new(760.0, 200.0));
        let vcc = board.add_vcc(egui::Pos2::new(60.0, 700.0));
        let gnd = board.add_gnd(egui::Pos2::new(200.0, 700.0));
        let pin = |device: u32, pin: u8| PinRef::Device { device, pin };
        board.connect(pin(transceiver, CAN_CANH), pin(motor, MOTOR_CANH));
        board.connect(pin(transceiver, CAN_CANL), pin(motor, MOTOR_CANL));
        board.connect(pin(vcc, POWER_PIN), pin(motor, MOTOR_VCC));
        board.connect(pin(gnd, POWER_PIN), pin(motor, MOTOR_GND));
        motor
    };
    app.refresh();
    assert_eq!(
        app.board().can_network().motors.len(),
        1,
        "the motor sits on the pair"
    );

    // The panel is drawn with the motor row, and a command put on the wire by
    // the external node reaches the part while the app runs its frames.
    let command = gsim_core::CanFrame::standard(0x200, &1200i16.to_be_bytes());
    app.board_mut()
        .can_bus()
        .borrow_mut()
        .inject_bit_frame(command, app.sim_time_ns());
    for _ in 0..8 {
        let output = ctx.run(raw_input(), |ctx| app.draw(ctx));
        assert!(!output.shapes.is_empty(), "the app painted nothing");
    }

    let handle = app
        .board()
        .device(motor)
        .and_then(|device| device.hardware.motor().cloned())
        .expect("motor model");
    let handle = handle.borrow();
    assert_eq!(handle.commands(), 1, "the panel's frame reached the motor");
    assert!(handle.rpm() > 0.0, "the shaft turns: {} rpm", handle.rpm());
}

#[test]
fn the_motor_panel_opens_the_protocol_document() {
    use gsimulator_gui::board::{
        CAN_CANH, CAN_CANL, MOTOR_CANH, MOTOR_CANL, MOTOR_GND, MOTOR_VCC, POWER_PIN,
    };

    let ctx = egui::Context::default();
    let mut app = App::with_context(&ctx, None);
    let motor = {
        let board = app.board_mut();
        let transceiver = board.add_can(egui::Pos2::new(480.0, 200.0));
        let motor = board.add_motor(egui::Pos2::new(760.0, 200.0));
        let vcc = board.add_vcc(egui::Pos2::new(60.0, 700.0));
        let gnd = board.add_gnd(egui::Pos2::new(200.0, 700.0));
        let pin = |device: u32, pin: u8| PinRef::Device { device, pin };
        board.connect(pin(transceiver, CAN_CANH), pin(motor, MOTOR_CANH));
        board.connect(pin(transceiver, CAN_CANL), pin(motor, MOTOR_CANL));
        board.connect(pin(vcc, POWER_PIN), pin(motor, MOTOR_VCC));
        board.connect(pin(gnd, POWER_PIN), pin(motor, MOTOR_GND));
        motor
    };
    app.refresh();

    // What the panel's "通讯协议" button does: attach the document to this part.
    app.motor_protocol_mut().open_for(motor);
    assert!(app.motor_protocol_mut().is_open());
    assert_eq!(app.motor_protocol_mut().device(), Some(motor));

    // The window is drawn together with the rest of the app, live state and all.
    for _ in 0..3 {
        let output = ctx.run(raw_input(), |ctx| app.draw(ctx));
        assert!(!output.shapes.is_empty(), "the app painted nothing");
    }

    // Deleting the part must not take the window (or the frame) down.
    app.board_mut().remove_device(motor);
    let output = ctx.run(raw_input(), |ctx| app.draw(ctx));
    assert!(!output.shapes.is_empty(), "the app painted nothing");
}

#[test]
fn app_draws_frames_headlessly() {
    let ctx = egui::Context::default();
    let mut app = App::with_context(&ctx, None);

    for _ in 0..4 {
        let output = ctx.run(raw_input(), |ctx| app.draw(ctx));
        assert!(!output.shapes.is_empty(), "nothing was painted");
    }

    // The built-in stimulus must have advanced the simulated time.
    assert!(app.sim_time_ns() > 0);
}

#[test]
fn clicking_the_canvas_drops_the_part_armed_by_the_library() {
    let ctx = egui::Context::default();
    let mut board = Board::new();
    let mut schematic = Schematic::default();
    let levels = PinLevels::default();
    let state = board.evaluate(&levels);
    schematic.place_pending(DeviceKind::Uart);

    let click = egui::Pos2::new(520.0, 420.0);
    // Hover first so the canvas knows where the pointer is, then click it.
    for events in [
        vec![egui::Event::PointerMoved(click)],
        vec![
            egui::Event::PointerButton {
                pos: click,
                button: egui::PointerButton::Primary,
                pressed: true,
                modifiers: egui::Modifiers::default(),
            },
            egui::Event::PointerButton {
                pos: click,
                button: egui::PointerButton::Primary,
                pressed: false,
                modifiers: egui::Modifiers::default(),
            },
        ],
    ] {
        let mut input = raw_input();
        input.events = events;
        let _ = ctx.run(input, |ctx| {
            egui::CentralPanel::default().show(ctx, |ui| {
                schematic.show(ui, &mut board, &levels, &state, 0);
            });
        });
    }

    assert_eq!(board.devices().len(), 1, "the click must place one part");
    assert_eq!(board.devices()[0].kind, DeviceKind::Uart);
    assert_eq!(schematic.pending_device(), None);
}

#[test]
fn the_device_library_window_renders_headlessly() {
    let ctx = egui::Context::default();
    let mut app = App::with_context(&ctx, None);
    app.library_mut().open = true;

    for _ in 0..3 {
        let output = ctx.run(raw_input(), |ctx| app.draw(ctx));
        assert!(!output.shapes.is_empty(), "nothing was painted");
    }

    assert!(app.library_mut().open, "the library window closed itself");
}

#[test]
fn missing_elf_is_reported_instead_of_panicking() {
    let mut sim = Simulation::new();
    let error = sim
        .load_elf(std::path::Path::new("definitely-not-here.elf"))
        .expect_err("loading a missing file must fail");
    assert!(error.contains("ELF"), "unexpected message: {error}");
}

#[test]
fn the_app_starts_with_the_hse_crystal_and_a_live_clock() {
    let ctx = egui::Context::default();
    let mut app = App::with_context(&ctx, None);
    for _ in 0..2 {
        let _ = ctx.run(raw_input(), |ctx| app.draw(ctx));
    }

    let board = app.board();
    assert!(
        board.first_of(DeviceKind::HseCrystal).is_some(),
        "a new project must already carry the HSE crystal"
    );
    assert_eq!(
        board.crystals().hse_hz,
        Some(8_000_000),
        "the crystal must be wired across OSC_IN/OSC_OUT"
    );
    assert_eq!(
        board.devices().len(),
        1,
        "no other part is placed by default"
    );

    let levels = app.levels();
    assert!(
        !levels.is_output(0, 5),
        "no stimulus is configured at start-up"
    );
    assert!(
        !levels.level_of(0, 0) && !levels.level_of(0, 1),
        "there is no encoder pulling contacts high"
    );
    assert!(
        app.sim_time_ns() > 0,
        "the simulation clock itself keeps running"
    );
}
