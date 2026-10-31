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
    // The canvas background must have been painted (dark blue-grey).
    let pixel = image.get_pixel(600, 600).0;
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
    // Once in the toolbar, once as the status section heading.
    assert_eq!(harness.query_all_by_label("内置激励").count(), 2);

    harness
        .get(By::new().label("内置激励").role(Role::Button))
        .click();
    harness.run_steps(2);

    assert_eq!(
        harness.query_all_by_label("内置激励").count(),
        1,
        "the status section must disappear, only the toggle button stays"
    );
    assert!(harness
        .query_by_label_contains("没有引脚驱动源")
        .is_some());
    assert!(harness
        .query_by_label_contains("内置激励：PA5")
        .is_none());

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

/// Snapshots go to the workspace `target/` directory, creating it if needed.
fn snapshot_path(name: &str) -> std::path::PathBuf {
    let directory = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../target");
    std::fs::create_dir_all(&directory).expect("creating target directory");
    directory.join(name)
}
