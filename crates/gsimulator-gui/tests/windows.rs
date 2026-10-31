//! Headless tests of the two status windows: the peripheral browser and the
//! dedicated CAN bus window.

use eframe::egui;
use gsimulator_gui::board::Board;
use gsimulator_gui::canbus::CanBusWindow;
use gsimulator_gui::peripherals::PeripheralBrowser;
use gsimulator_gui::sim::Simulation;

fn raw_input() -> egui::RawInput {
    egui::RawInput {
        screen_rect: Some(egui::Rect::from_min_size(
            egui::Pos2::ZERO,
            egui::Vec2::new(1440.0, 900.0),
        )),
        ..Default::default()
    }
}

/// Snapshot of the simulation plus the parts placed on the drawing.
fn frame(sim: &Simulation, board: &Board) -> gsimulator_gui::sim::Snapshot {
    let mut snapshot = sim.snapshot();
    board.fill_snapshot(&mut snapshot);
    snapshot
}

#[test]
fn the_peripheral_browser_lists_every_block_and_the_placed_parts() {
    let sim = Simulation::new();
    let mut board = Board::demo();
    board.add_matrix(egui::Pos2::new(320.0, 640.0));
    board.add_can(egui::Pos2::new(480.0, 640.0));
    let snapshot = frame(&sim, &board);
    for expected in [
        "RCC", "GPIOA", "AFIO", "EXTI", "USART1", "SPI1", "I2C1", "CAN1", "TIM1", "ADC1",
        "DMA1", "FLASH", "PWR", "RTC", "IWDG", "WWDG", "CRC", "NVIC", "SysTick", "MX1",
        "CAN1",
    ] {
        assert!(
            snapshot
                .peripherals
                .iter()
                .any(|status| status.name == expected),
            "{expected} is missing from the peripheral browser"
        );
    }

    let mut browser = PeripheralBrowser::new();
    browser.open = true;
    let ctx = egui::Context::default();
    let output = ctx.run(raw_input(), |ctx| browser.show(ctx, &snapshot));
    assert!(!output.shapes.is_empty(), "the browser painted nothing");
    assert_eq!(browser.selected(), Some("RCC"));
}

#[test]
fn the_can_window_shows_injected_frames() {
    let mut board = Board::demo();
    board.add_can(egui::Pos2::new(480.0, 640.0));
    let sim = Simulation::new();
    let now = sim.now_ns();
    board
        .can_bus()
        .borrow_mut()
        .inject(gsim_core::CanFrame::standard(0x321, &[0x11, 0x22]), now);
    let snapshot = frame(&sim, &board);
    assert!(!snapshot.can.records.is_empty());
    assert_eq!(snapshot.can.records.last().map(|r| r.frame.id), Some(0x321));

    let mut window = CanBusWindow::default();
    window.open = true;
    assert_eq!(
        window.visible_frames(&snapshot),
        1,
        "an external ECU only logs its own transmission"
    );

    let ctx = egui::Context::default();
    let output = ctx.run(raw_input(), |ctx| {
        window.show(ctx, &snapshot);
    });
    assert!(!output.shapes.is_empty(), "the CAN window painted nothing");
}
