//! The schematic survives a round trip through the project file.

use gsimulator_gui::app::App;
use gsimulator_gui::board::Board;
use gsimulator_gui::project;

/// Unique scratch file, so parallel tests do not share one path.
fn temp_path(name: &str) -> std::path::PathBuf {
    let stamp = std::time::SystemTime::now()
        .duration_since(std::time::SystemTime::UNIX_EPOCH)
        .map(|elapsed| elapsed.as_nanos())
        .unwrap_or(0);
    std::env::temp_dir().join(format!(
        "gsimulator-{name}-{}-{stamp}.{}",
        std::process::id(),
        project::EXTENSION
    ))
}

#[test]
fn an_app_opens_a_saved_schematic_over_the_empty_canvas() {
    let ctx = egui::Context::default();
    let mut app = App::with_context(&ctx, None);
    // A new project only carries the seed parts; the demo adds the rest.
    let seed_devices = app.board().devices().len();

    let saved = Board::demo();
    assert!(
        saved.devices().len() > seed_devices,
        "the demo circuit must add parts on top of a new project"
    );
    let path = temp_path("open");
    project::save(&path, &saved).expect("the demo board is writable");
    app.open_project_from(&path).expect("the file reads back");
    let _ = std::fs::remove_file(&path);

    assert_eq!(app.board().devices().len(), saved.devices().len());
    assert_eq!(app.board().wires().len(), saved.wires().len());
    assert_eq!(
        app.board().crystals(),
        saved.crystals(),
        "the crystals of the loaded board reach the RCC"
    );
}

#[test]
fn saving_and_opening_the_app_keeps_the_drawing() {
    let ctx = egui::Context::default();
    let seed = temp_path("seed");
    project::save(&seed, &Board::demo()).expect("the demo board is writable");
    let mut app = App::with_context(&ctx, None);
    app.open_project_from(&seed).expect("the seed reads back");
    let _ = std::fs::remove_file(&seed);

    let path = temp_path("save");
    app.save_project_to(&path).expect("the canvas is writable");

    let mut reopened = App::with_context(&ctx, None);
    reopened
        .open_project_from(&path)
        .expect("the saved file reads back");
    let _ = std::fs::remove_file(&path);

    assert_eq!(
        reopened.board().devices().len(),
        app.board().devices().len()
    );
    assert_eq!(reopened.board().wires().len(), app.board().wires().len());
}

#[test]
fn a_broken_file_leaves_the_schematic_alone() {
    let ctx = egui::Context::default();
    let mut app = App::with_context(&ctx, None);
    let before = app.board().devices().len();
    let path = temp_path("broken");
    std::fs::write(&path, "这不是工程文件\n").expect("scratch file");
    let error = app
        .open_project_from(&path)
        .expect_err("a broken file must be reported");
    let _ = std::fs::remove_file(&path);
    assert!(
        error.contains("第 1 行"),
        "the message must point at the line: {error}"
    );
    assert_eq!(
        app.board().devices().len(),
        before,
        "nothing is loaded from a broken file"
    );
}
