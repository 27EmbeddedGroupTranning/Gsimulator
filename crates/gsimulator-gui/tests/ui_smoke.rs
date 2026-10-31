//! Headless end-to-end test: the built-in stimulus drives PA5, the schematic
//! wiring made in the EDA-style canvas decides whether the LED is lit, and the
//! whole window renders without a display.

use eframe::egui;
use gsimulator_gui::app::App;
use gsimulator_gui::board::{Board, DeviceKind, PinLevels, PinRef, Rail};
use gsimulator_gui::canvas::Schematic;
use gsimulator_gui::sim::{Simulation, Source};

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
fn stimulus_toggles_pa5_and_lights_the_demo_led() {
    let mut sim = Simulation::new();
    let board = Board::demo();
    let led = board.devices()[0].id;

    let mut saw_high = false;
    let mut saw_low = false;
    let mut saw_lit = false;
    let mut saw_dark = false;

    for _ in 0..40 {
        sim.advance_frame(400_000);
        let levels = sim.pin_levels();
        let state = board.evaluate(&levels);
        if levels.level_of(0, 5) {
            saw_high = true;
            saw_lit |= state.led_lit(led);
        } else {
            saw_low = true;
            saw_dark |= !state.led_lit(led);
        }
    }

    assert!(saw_high, "the stimulus never drove PA5 high");
    assert!(saw_low, "the stimulus never drove PA5 low");
    assert!(saw_lit, "the LED never lit up");
    assert!(saw_dark, "the LED never went dark again");
}

#[test]
fn stopping_the_stimulus_leaves_the_simulation_idle() {
    let mut sim = Simulation::new();
    sim.advance_frame(400_000);
    assert!(sim.now_ns() > 0, "the stimulus must start automatically");

    sim.stop_stimulus();
    assert!(!sim.is_running());
    assert!(matches!(sim.source(), Source::Idle));
    assert!(!sim.pin_levels().is_output(0, 5), "PA5 must be released");
    assert!(!sim.pin_levels().level_of(0, 5), "PA5 must be driven low");

    // Idle: frames neither advance time nor re-arm the square wave.
    let frozen = sim.now_ns();
    sim.advance_frame(400_000);
    assert_eq!(sim.now_ns(), frozen);

    // A reset with no driver must not switch the stimulus back on.
    sim.reset();
    assert!(matches!(sim.source(), Source::Idle));
    assert!(!sim.is_running());
    sim.advance_frame(400_000);
    assert_eq!(sim.now_ns(), frozen);

    // It can be turned back on.
    sim.start_stimulus();
    assert!(matches!(sim.source(), Source::Stimulus));
    sim.advance_frame(400_000);
    assert!(sim.now_ns() > frozen);
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
    schematic.click_pin(&mut board, PinRef::Rail(Rail::Gnd));
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
    board.connect(PinRef::Rail(Rail::Vcc), PinRef::Rail(Rail::Gnd));
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
fn the_demo_board_drives_the_wired_encoder_contacts() {
    let ctx = egui::Context::default();
    let mut app = App::with_context(&ctx, None);
    for _ in 0..2 {
        ctx.run(raw_input(), |ctx| app.draw(ctx));
    }
    let levels = app.levels();
    assert!(
        levels.level_of(0, 0) && levels.level_of(0, 1),
        "the encoder contacts must be pulled high by the harness"
    );
    assert!(
        levels.is_output(0, 5),
        "the stimulus drives PA5 as an output"
    );
}
