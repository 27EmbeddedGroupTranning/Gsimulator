//! Headless tests of the status windows: the peripheral browser, the dedicated
//! CAN bus window and the oscilloscope.

use eframe::egui;
use gsim_devices::CanBus;
use gsimulator_gui::board::{Board, PinLevels};
use gsimulator_gui::canbus::CanBusWindow;
use gsimulator_gui::peripherals::PeripheralBrowser;
use gsimulator_gui::scope::ScopeWindow;
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

#[test]
fn the_peripheral_browser_lists_every_block_and_the_placed_devices() {
    let sim = Simulation::new();
    let mut snapshot = sim.snapshot();
    for expected in [
        "RCC", "GPIOA", "AFIO", "EXTI", "USART1", "SPI1", "I2C1", "CAN1", "TIM1", "ADC1", "DMA1",
        "FLASH", "PWR", "RTC", "IWDG", "WWDG", "CRC", "NVIC", "SysTick",
    ] {
        assert!(
            snapshot
                .peripherals
                .iter()
                .any(|status| status.name == expected),
            "{expected} is missing from the peripheral browser"
        );
    }

    // The off-chip parts only exist once they are placed on the schematic.
    let mut board = Board::new();
    board.add_matrix(egui::Pos2::new(600.0, 100.0));
    board.add_can(egui::Pos2::new(600.0, 300.0));
    board.fill_snapshot(&mut snapshot, &PinLevels::default());
    assert!(
        snapshot
            .peripherals
            .iter()
            .any(|status| status.kind.contains("MAX7219")),
        "the LED matrix has no status entry"
    );
    assert!(
        snapshot
            .peripherals
            .iter()
            .any(|status| status.kind.contains("CAN 收发器")),
        "the CAN transceiver has no status entry"
    );

    let mut browser = PeripheralBrowser::new();
    browser.open = true;
    let ctx = egui::Context::default();
    let output = ctx.run(raw_input(), |ctx| browser.show(ctx, &snapshot));
    assert!(!output.shapes.is_empty(), "the browser painted nothing");
    assert_eq!(browser.selected(), Some("RCC"));
}

#[test]
fn the_can_window_shows_the_frames_on_the_bus() {
    let sim = Simulation::new();
    let mut snapshot = sim.snapshot();
    let mut bus = CanBus::new();
    bus.inject(gsim_core::CanFrame::standard(0x321, &[0x11, 0x22]), 0);
    snapshot.can.observe(&bus, None);

    assert_eq!(snapshot.can.records.len(), 1);
    assert_eq!(snapshot.can.records[0].frame.id, 0x321);

    let mut window = CanBusWindow::default();
    window.open = true;
    assert_eq!(
        window.visible_frames(&snapshot),
        1,
        "the bus has no node attached, so only the external transmission is logged"
    );

    let ctx = egui::Context::default();
    let output = ctx.run(raw_input(), |ctx| {
        window.show(ctx, &snapshot);
    });
    assert!(!output.shapes.is_empty(), "the CAN window painted nothing");
}
#[test]
fn the_scope_window_draws_the_trace_of_a_wired_probe() {
    use gsim_devices::{ScopeSample, WireLevels};
    use gsimulator_gui::board::{CAN_CANH, CAN_CANL, SCOPE_CANH, SCOPE_CANL};

    let sim = Simulation::new();
    let mut snapshot = sim.snapshot();

    // A probe has to be wired across the pair to see anything.
    let mut board = Board::new();
    let transceiver = board.add_can(egui::Pos2::new(500.0, 200.0));
    let scope = board.add_scope(egui::Pos2::new(700.0, 200.0));
    board.connect(
        gsimulator_gui::board::PinRef::Device {
            device: transceiver,
            pin: CAN_CANH,
        },
        gsimulator_gui::board::PinRef::Device {
            device: scope,
            pin: SCOPE_CANH,
        },
    );
    board.connect(
        gsimulator_gui::board::PinRef::Device {
            device: transceiver,
            pin: CAN_CANL,
        },
        gsimulator_gui::board::PinRef::Device {
            device: scope,
            pin: SCOPE_CANL,
        },
    );
    assert_eq!(board.can_network().scopes.len(), 1);

    let handle = board.scopes()[0].clone();
    {
        let mut probe = handle.borrow_mut();
        for step in 0..64u64 {
            let dominant = step % 4 < 2;
            let levels = WireLevels::of(dominant);
            probe.record(ScopeSample {
                time_ns: step * 5_000,
                can_h: levels.can_h,
                can_l: levels.can_l,
                dominant,
                glitch: step == 37,
            });
        }
    }
    board.fill_snapshot(&mut snapshot, &PinLevels::default());
    assert_eq!(snapshot.scopes.len(), 1);
    assert_eq!(snapshot.scopes[0].samples.len(), 64);
    assert!(snapshot.scopes[0].peak > 1.9);

    let mut window = ScopeWindow::default();
    window.open = true;
    let ctx = egui::Context::default();
    let output = ctx.run(raw_input(), |ctx| {
        window.show(ctx, &snapshot);
    });
    assert!(!output.shapes.is_empty(), "the scope painted nothing");

    // Without a probe the window explains what is missing instead of panicking.
    let mut window = ScopeWindow::default();
    window.open = true;
    let empty = sim.snapshot();
    let ctx = egui::Context::default();
    let output = ctx.run(raw_input(), |ctx| {
        window.show(ctx, &empty);
    });
    assert!(
        !output.shapes.is_empty(),
        "the empty window painted nothing"
    );
}
