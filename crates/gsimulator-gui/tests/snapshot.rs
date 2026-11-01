//! Renders the window off-screen with the wgpu test renderer.
//!
//! This is a smoke test (every paint path runs, nothing panics) and the source
//! of the screenshot shown in the README: run with `GSIM_SNAPSHOT=1` to refresh
//! `target/gui-snapshot.png`.

use eframe::egui;
use gsimulator_gui::app::App;
use gsimulator_gui::board::{Board, DeviceKind, PinLevels, PinRef, POWER_PIN};
use gsimulator_gui::canvas::Schematic;

#[test]
fn window_renders_offscreen() {
    let mut app: Option<App> = None;
    let mut harness = egui_kittest::Harness::builder()
        .with_size(egui::Vec2::new(1440.0, 900.0))
        .with_pixels_per_point(1.0)
        .wgpu()
        .build(move |ctx| {
            let app = app.get_or_insert_with(|| App::with_context(ctx, None));
            app.draw(ctx);
        });

    harness.run_steps(3);
    // Machines without a GPU adapter simply skip the image checks.
    let Ok(image) = harness.render() else {
        eprintln!("no offscreen renderer available, skipping the snapshot check");
        return;
    };

    if std::env::var_os("GSIM_SNAPSHOT").is_some() {
        image
            .save(snapshot_path("gui-snapshot.png"))
            .expect("saving snapshot");
    }

    assert_eq!(image.width(), 1440);
    assert_eq!(image.height(), 900);
    // The canvas background must have been painted (dark blue-grey); sample a
    // corner so the fitted MCU symbol can never cover it.
    let pixel = image.get_pixel(20, 840).0;
    assert!(
        pixel[0] < 70 && pixel[1] < 75 && pixel[2] < 85,
        "canvas background looks wrong: {pixel:?}"
    );
}

#[test]
fn the_stimulus_toggle_switches_the_square_wave_off() {
    use egui::accesskit::Role;
    use egui_kittest::kittest::{By, Queryable};
    use std::cell::RefCell;
    use std::rc::Rc;

    let app: Rc<RefCell<Option<App>>> = Rc::new(RefCell::new(None));
    let slot = Rc::clone(&app);
    let mut harness = egui_kittest::Harness::builder()
        .with_size(egui::Vec2::new(1440.0, 900.0))
        .with_pixels_per_point(1.0)
        .build(move |ctx| {
            let mut slot = slot.borrow_mut();
            let app = slot.get_or_insert_with(|| App::with_context(ctx, None));
            app.draw(ctx);
        });

    harness.run_steps(2);
    // Nothing drives the pins yet: only the toolbar switch exists, and no
    // "idle source" line is shown in the status panel.
    assert_eq!(harness.query_all_by_label("内置激励").count(), 1);
    assert!(harness.query_by_label_contains("没有引脚驱动源").is_none());

    harness
        .get(By::new().label("内置激励").role(Role::Button))
        .click();
    harness.run_steps(2);

    assert_eq!(
        harness.query_all_by_label("内置激励").count(),
        2,
        "the status section appears while the stimulus runs"
    );

    harness
        .get(By::new().label("内置激励").role(Role::Button))
        .click();
    harness.run_steps(2);

    assert_eq!(
        harness.query_all_by_label("内置激励").count(),
        1,
        "the status section must disappear, only the toggle button stays"
    );
    assert!(harness.query_by_label_contains("内置激励：PA5").is_none());

    // The simulation clock is not tied to the stimulus: it keeps running.
    let stop = app.borrow().as_ref().expect("app").sim_time_ns();
    harness.run_steps(3);
    let later = app.borrow().as_ref().expect("app").sim_time_ns();
    assert!(
        later > stop,
        "the clock must keep running after the stimulus is switched off"
    );
}

#[test]
fn the_stimulus_settings_window_opens() {
    use egui_kittest::kittest::Queryable;

    let mut app: Option<App> = None;
    let mut harness = egui_kittest::Harness::builder()
        .with_size(egui::Vec2::new(1440.0, 900.0))
        .with_pixels_per_point(1.0)
        .build(move |ctx| {
            let app = app.get_or_insert_with(|| App::with_context(ctx, None));
            app.draw(ctx);
        });

    harness.run_steps(2);
    harness.get_by_label("⚙ 激励").click();
    harness.run_steps(2);

    assert!(harness.query_by_label("激励设置").is_some());
    assert!(harness.query_by_label("应用并运行").is_some());
}

#[test]
fn schematic_renders_wires_and_devices_offscreen() {
    let mut board = Board::demo();
    let led = board.first_of(DeviceKind::Led).expect("demo LED").id;
    let vcc = board.first_of(DeviceKind::Vcc).expect("demo VCC").id;
    board.add_can(egui::Pos2::new(320.0, 640.0));
    board.add_matrix(egui::Pos2::new(480.0, 640.0));
    board.add_motor(egui::Pos2::new(700.0, 640.0));
    board.add_led(egui::Pos2::new(640.0, 430.0));
    board.connect(
        PinRef::Device {
            device: vcc,
            pin: POWER_PIN,
        },
        PinRef::Device {
            device: led,
            pin: 0,
        },
    );
    let levels = PinLevels {
        level: {
            let mut level = [[false; 16]; 5];
            level[0][5] = true;
            level
        },
        output: [[true; 16]; 5],
        pwm: [[None; 16]; 5],
    };
    let state = board.evaluate(&levels);
    let mut schematic = Schematic::default();

    let mut harness = egui_kittest::Harness::builder()
        .with_size(egui::Vec2::new(1000.0, 760.0))
        .with_pixels_per_point(1.0)
        .wgpu()
        .build_ui(move |ui| {
            schematic.show(ui, &mut board, &levels, &state, 0);
        });
    harness.run_steps(2);
    let Ok(image) = harness.render() else {
        eprintln!("no offscreen renderer available, skipping the snapshot check");
        return;
    };

    if std::env::var_os("GSIM_SNAPSHOT").is_some() {
        image
            .save(snapshot_path("gui-snapshot-canvas.png"))
            .expect("saving the canvas snapshot");
    }
    assert!(image.width() > 0);
}

/// The scope window paints the waveform plus the bit ruler of a captured frame.
#[test]
fn the_scope_window_renders_the_decoded_bits() {
    use gsim_core::can::{CanFrame, WireFrame, DOMINANT};
    use gsim_devices::{ScopeSample, WireLevels};
    use gsimulator_gui::board::{CAN_CANH, CAN_CANL, SCOPE_CANH, SCOPE_CANL};
    use gsimulator_gui::scope::ScopeWindow;
    use gsimulator_gui::sim::Simulation;
    use std::cell::RefCell;

    let sim = Simulation::new();
    let mut snapshot = sim.snapshot();
    let mut board = Board::new();
    let transceiver = board.add_can(egui::Pos2::new(400.0, 200.0));
    let probe = board.add_scope(egui::Pos2::new(760.0, 200.0));
    for (from, to) in [(CAN_CANH, SCOPE_CANH), (CAN_CANL, SCOPE_CANL)] {
        board.connect(
            PinRef::Device {
                device: transceiver,
                pin: from,
            },
            PinRef::Device {
                device: probe,
                pin: to,
            },
        );
    }
    {
        let scope = board.scopes()[0].clone();
        let mut scope = scope.borrow_mut();
        scope.set_window_ns(10_000_000);
        let wire = WireFrame::encode(&CanFrame::standard(0x321, b"GSIM"));
        for (index, bit) in wire.bits().iter().enumerate() {
            let level = if index == wire.ack_slot() {
                DOMINANT
            } else {
                *bit
            };
            let levels = WireLevels::of(level);
            scope.record(ScopeSample {
                time_ns: index as u64 * 4_000,
                can_h: levels.can_h,
                can_l: levels.can_l,
                dominant: level == DOMINANT,
                glitch: false,
            });
        }
    }
    board.fill_snapshot(&mut snapshot, &PinLevels::default());

    let window = RefCell::new({
        let mut window = ScopeWindow::default();
        window.open = true;
        window
    });
    let mut harness = egui_kittest::Harness::builder()
        .with_size(egui::Vec2::new(1240.0, 720.0))
        .with_pixels_per_point(1.0)
        .wgpu()
        .build(move |ctx| {
            window.borrow_mut().show(ctx, &snapshot);
        });
    harness.run_steps(3);
    let Ok(image) = harness.render() else {
        eprintln!("no offscreen renderer available, skipping the snapshot check");
        return;
    };
    if std::env::var_os("GSIM_SNAPSHOT").is_some() {
        image
            .save(snapshot_path("gui-snapshot-scope.png"))
            .expect("saving the scope snapshot");
    }
    assert_eq!(image.width(), 1240);
    assert_eq!(image.height(), 720);
}

/// The device panel carries the motor row, and its speed shortcut puts a real
/// command frame on the bus.
#[test]
fn the_device_panel_drives_the_can_motor() {
    use egui_kittest::kittest::Queryable;
    use gsimulator_gui::board::{CAN_CANH, CAN_CANL, MOTOR_CANH, MOTOR_CANL, MOTOR_GND, MOTOR_VCC};
    use std::cell::RefCell;
    use std::rc::Rc;

    let app: Rc<RefCell<Option<App>>> = Rc::new(RefCell::new(None));
    let slot = Rc::clone(&app);
    let mut harness = egui_kittest::Harness::builder()
        .with_size(egui::Vec2::new(1440.0, 900.0))
        .with_pixels_per_point(1.0)
        .build(move |ctx| {
            let mut slot = slot.borrow_mut();
            let app = slot.get_or_insert_with(|| {
                let mut app = App::with_context(ctx, None);
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
                app
            });
            app.draw(ctx);
        });
    harness.run_steps(2);
    assert!(
        harness.query_by_label("正转 ▶").is_some(),
        "the panel has no motor speed shortcut"
    );
    harness.get_by_label("正转 ▶").click();
    for _ in 0..6 {
        harness.run_steps(1);
    }

    let app = app.borrow();
    let app = app.as_ref().expect("app");
    let motor = app
        .board()
        .first_of(DeviceKind::Motor)
        .expect("the motor is placed");
    let motor = motor.hardware.motor().expect("motor model").borrow();
    assert_eq!(motor.commands(), 1, "the shortcut put a frame on the bus");
    assert!(motor.rpm() > 0.0, "the shaft turns: {} rpm", motor.rpm());
}

/// The protocol document attached to the motor paint path renders off-screen.
#[test]
fn the_motor_protocol_document_renders() {
    use gsimulator_gui::board::{CAN_CANH, CAN_CANL, MOTOR_CANH, MOTOR_CANL, MOTOR_GND, MOTOR_VCC};
    use std::cell::RefCell;
    use std::rc::Rc;

    let app: Rc<RefCell<Option<App>>> = Rc::new(RefCell::new(None));
    let slot = Rc::clone(&app);
    let mut harness = egui_kittest::Harness::builder()
        .with_size(egui::Vec2::new(1100.0, 800.0))
        .with_pixels_per_point(1.0)
        .wgpu()
        .build(move |ctx| {
            let mut slot = slot.borrow_mut();
            let app = slot.get_or_insert_with(|| {
                let mut app = App::with_context(ctx, None);
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
                app.motor_protocol_mut().open_for(motor);
                app
            });
            app.draw(ctx);
        });
    harness.run_steps(3);

    let Ok(image) = harness.render() else {
        eprintln!("no offscreen renderer available, skipping the snapshot check");
        return;
    };
    if std::env::var_os("GSIM_SNAPSHOT").is_some() {
        image
            .save(snapshot_path("gui-snapshot-motor-protocol.png"))
            .expect("saving the protocol snapshot");
    }
    assert_eq!(image.width(), 1100);
    assert_eq!(image.height(), 800);
}

/// The button in the motor row is what brings the document up.
#[test]
fn the_motor_row_opens_the_protocol_document() {
    use egui_kittest::kittest::Queryable;
    use std::cell::RefCell;
    use std::rc::Rc;

    let app: Rc<RefCell<Option<App>>> = Rc::new(RefCell::new(None));
    let slot = Rc::clone(&app);
    let mut harness = egui_kittest::Harness::builder()
        .with_size(egui::Vec2::new(1440.0, 900.0))
        .with_pixels_per_point(1.0)
        .build(move |ctx| {
            let mut slot = slot.borrow_mut();
            let app = slot.get_or_insert_with(|| {
                let mut app = App::with_context(ctx, None);
                app.board_mut().add_motor(egui::Pos2::new(760.0, 200.0));
                app
            });
            app.draw(ctx);
        });
    harness.run_steps(2);
    assert!(
        harness.query_by_label_contains("1 · 帧格式").is_none(),
        "the document starts closed"
    );

    harness.get_by_label("📖 通讯协议").click();
    harness.run_steps(2);

    assert!(
        harness.query_by_label_contains("1 · 帧格式").is_some(),
        "the button must open the protocol document"
    );
    assert!(
        harness.query_by_label_contains("转速高字节").is_some(),
        "the frame table is part of the document"
    );
}

/// Snapshots go to the workspace `target/` directory, creating it if needed.
fn snapshot_path(name: &str) -> std::path::PathBuf {
    let directory = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../target");
    std::fs::create_dir_all(&directory).expect("creating target directory");
    directory.join(name)
}
