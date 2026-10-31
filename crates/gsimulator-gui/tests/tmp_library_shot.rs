//! Temporary manual screenshot helper (removed once the layout is verified).

use eframe::egui;
use gsimulator_gui::app::App;
use gsimulator_gui::board::{Board, DeviceKind, PinLevels};
use gsimulator_gui::canvas::Schematic;

#[test]
fn library_snapshot() {
    let mut app: Option<App> = None;
    let mut harness = egui_kittest::Harness::builder()
        .with_size(egui::Vec2::new(1280.0, 820.0))
        .with_pixels_per_point(1.0)
        .wgpu()
        .build(move |ctx| {
            let app = app.get_or_insert_with(|| {
                let mut app = App::with_context(ctx, None);
                app.library_mut().open = true;
                app
            });
            app.draw(ctx);
        });
    harness.run_steps(4);
    let image = harness.render().expect("offscreen render");
    let path =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../target/library-shot.png");
    image.save(&path).expect("saving the library screenshot");
    eprintln!("saved {}", path.display());
}

#[test]
fn ghost_snapshot() {
    let mut board = Board::demo();
    let levels = PinLevels::default();
    let state = board.evaluate(&levels);
    let mut schematic = Schematic::default();
    schematic.place_pending(DeviceKind::Encoder);
    let mut harness = egui_kittest::Harness::builder()
        .with_size(egui::Vec2::new(900.0, 700.0))
        .with_pixels_per_point(1.0)
        .wgpu()
        .build_ui(move |ui| {
            schematic.show(ui, &mut board, &levels, &state, 0);
        });
    harness.run_steps(2);
    for _ in 0..3 {
        harness
            .input_mut()
            .events
            .push(egui::Event::PointerMoved(egui::Pos2::new(330.0, 600.0)));
        harness.run_steps(1);
    }
    let image = harness.render().expect("offscreen render");
    let path =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../target/ghost-shot.png");
    image.save(&path).expect("saving the ghost screenshot");
    eprintln!("saved {}", path.display());
}
