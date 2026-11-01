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
    board.add_motor(egui::Pos2::new(800.0, 300.0));
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
    assert!(
        snapshot
            .peripherals
            .iter()
            .any(|status| status.kind.contains("CAN 电机")),
        "the CAN motor has no status entry"
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

/// Text the UI painted, so a headless test can assert on the labels.
fn painted_text(output: &egui::FullOutput) -> Vec<String> {
    fn collect(shape: &egui::epaint::Shape, texts: &mut Vec<String>) {
        match shape {
            egui::epaint::Shape::Text(text) => texts.push(text.galley.text().to_string()),
            egui::epaint::Shape::Vec(shapes) => {
                for shape in shapes {
                    collect(shape, texts);
                }
            }
            _ => {}
        }
    }
    let mut texts = Vec::new();
    for clipped in &output.shapes {
        collect(&clipped.shape, &mut texts);
    }
    texts
}

#[test]
fn the_scope_window_captions_every_bit_of_a_captured_frame() {
    use gsim_core::can::{CanFrame, WireFrame, DOMINANT};
    use gsim_devices::{ScopeSample, WireLevels};
    use gsimulator_gui::board::{CAN_CANH, CAN_CANL, SCOPE_CANH, SCOPE_CANL};

    let sim = Simulation::new();
    let mut snapshot = sim.snapshot();
    let mut board = Board::new();
    let transceiver = board.add_can(egui::Pos2::new(400.0, 200.0));
    let probe = board.add_scope(egui::Pos2::new(760.0, 200.0));
    for (from, to) in [(CAN_CANH, SCOPE_CANH), (CAN_CANL, SCOPE_CANL)] {
        board.connect(
            gsimulator_gui::board::PinRef::Device {
                device: transceiver,
                pin: from,
            },
            gsimulator_gui::board::PinRef::Device {
                device: probe,
                pin: to,
            },
        );
    }
    let handle = board.scopes()[0].clone();
    {
        let mut scope = handle.borrow_mut();
        scope.set_window_ns(10_000_000);
        let frame = CanFrame::standard(0x321, b"GSIM");
        let wire = WireFrame::encode(&frame);
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
    assert_eq!(
        snapshot.scopes[0].frames.len(),
        1,
        "the probe decoded a frame"
    );

    let mut window = ScopeWindow::default();
    window.open = true;
    let ctx = egui::Context::default();
    // A window fades in, so the very first pass paints placeholders only.
    let _ = ctx.run(raw_input(), |ctx| {
        window.show(ctx, &snapshot);
    });
    let output = ctx.run(raw_input(), |ctx| {
        window.show(ctx, &snapshot);
    });
    let texts = painted_text(&output);
    assert!(
        texts.iter().any(|text| text == "CAN 帧解码"),
        "the bit decode area is missing: {texts:?}"
    );
    for caption in ["0·SOF", "12·RTR", "19·数据0", "EOF0"] {
        assert!(
            texts.iter().any(|text| text.contains(caption)),
            "{caption} is not captioned: {texts:?}"
        );
    }
    assert!(
        texts.iter().any(|text| text.contains("ID=0x321")),
        "the decoded identifier is missing: {texts:?}"
    );
}
