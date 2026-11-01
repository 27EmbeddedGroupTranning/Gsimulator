//! Wiring bench end-to-end test for the self test firmware in `firmware/CAN`.
//!
//! The schematic is the one the README documents (`docs/selftest.gsim`:
//! terminal, LED, encoder, LED matrix, CAN transceiver with a terminator and a
//! probe, both crystals) and the real ELF is loaded into it. Everything the
//! firmware reports on USART1 therefore had to travel through the same
//! register model, wiring harness and device models the GUI uses.
//!
//! The image is a build artefact: when it is missing the tests only report the
//! skip, so a fresh checkout still passes.

use std::path::PathBuf;

use gsim_core::CanFrame;
use gsim_devices::Direction;
use gsimulator_gui::board::{Board, DeviceId, DeviceKind};
use gsimulator_gui::bridge;
use gsimulator_gui::sim::Simulation;

const CAN_ID: u16 = 0x123;

fn firmware() -> Option<PathBuf> {
    let path =
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../firmware/CAN/build/Debug/CAN.elf");
    if path.exists() {
        Some(path)
    } else {
        eprintln!(
            "skipping: {} is missing (build firmware/CAN with CMake first)",
            path.display()
        );
        None
    }
}

/// The drawing the README tells the reader to open, with the parts the tests
/// need looked up by kind.
fn bench() -> (Board, DeviceId, DeviceId, DeviceId) {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../docs/selftest.gsim");
    let board = gsimulator_gui::project::open(&path).expect("docs/selftest.gsim must parse");
    let find = |kind: DeviceKind| {
        board
            .first_of(kind)
            .unwrap_or_else(|| panic!("the drawing needs a {kind:?}"))
            .id
    };
    let uart = find(DeviceKind::Uart);
    let encoder = find(DeviceKind::Encoder);
    let transceiver = find(DeviceKind::Can);
    (board, uart, encoder, transceiver)
}

fn terminal_text(board: &Board, uart: DeviceId) -> String {
    board
        .device(uart)
        .expect("terminal")
        .hardware
        .uart()
        .expect("uart model")
        .rx_text()
        .to_string()
}

fn encoder_mut(board: &mut Board, id: DeviceId) -> &mut gsim_devices::Encoder {
    board
        .device_mut(id)
        .expect("encoder")
        .hardware
        .encoder_mut()
        .expect("encoder model")
}

/// Runs up to `frames` slices of `budget` instructions (1 ms of simulated time
/// each), stopping as soon as `done` says so.
fn run_until(
    board: &mut Board,
    sim: &mut Simulation,
    frames: u32,
    budget: u32,
    done: impl Fn(&Board, &Simulation) -> bool,
) -> bool {
    for _ in 0..frames {
        bridge::run_frame(board, sim, budget, 1_000_000);
        if done(board, sim) {
            return true;
        }
    }
    false
}

#[test]
fn the_report_reaches_the_terminal_and_the_led_matrix_shows_the_picture() {
    let Some(path) = firmware() else {
        return;
    };
    let (mut board, uart, _, _) = bench();
    let matrix = board.first_of(DeviceKind::Matrix).expect("matrix part").id;
    let led = board.first_of(DeviceKind::Led).expect("LED part").id;

    let mut sim = Simulation::new();
    sim.set_crystals(board.crystals());
    assert_eq!(
        sim.with_soc(|soc| (soc.hse_present(), soc.lse_present())),
        (true, true),
        "both crystals of the drawing must reach the RCC"
    );
    sim.load_elf(&path).expect("load the firmware");

    // Sample the LED during the first milliseconds of the run: the GPIO test
    // drives PB12 high for a millisecond.
    let mut saw_lit = false;
    let mut saw_dark = false;
    for _ in 0..600 {
        bridge::run_frame(&mut board, &mut sim, 1_500, 1_000_000);
        let lit = board.evaluate(&sim.pin_levels()).led_lit(led);
        saw_lit |= lit;
        saw_dark |= !lit;
    }
    assert!(saw_lit, "PB12 never lit the LED");
    assert!(saw_dark, "the LED stayed lit");

    assert!(
        run_until(&mut board, &mut sim, 2_000, 200_000, |board, _| {
            terminal_text(board, uart).contains("EVT READY")
        }),
        "the firmware never reached its main loop, terminal shows {:?}",
        terminal_text(&board, uart)
    );

    let text = terminal_text(&board, uart);
    assert!(text.starts_with("GSIM-SELFTEST 1\n"), "{text:?}");
    assert!(text.contains("\nCLK 72000000 HSE\n"), "{text:?}");
    assert!(
        !text.contains("FAIL"),
        "a block reported a failure: {text:?}"
    );
    assert!(text.contains("\nONCHIP 23/23\n"), "{text:?}");
    assert!(
        text.contains("\nDEV MATRIX=OK ECHO="),
        "the matrix never answered on DOUT: {text:?}"
    );
    assert!(sim.hse_ready(), "the HSE crystal never started");

    let model = board
        .device(matrix)
        .expect("matrix")
        .hardware
        .matrix()
        .cloned()
        .expect("matrix model");
    let model = model.borrow();
    assert!(model.words() > 0, "the chain never latched a word");
    assert!(model.chips()[0].enabled(), "the matrix is still shut down");
    assert_eq!(model.chips()[0].intensity(), 7);
    assert!(
        model.chips()[0]
            .pixels()
            .iter()
            .flatten()
            .any(|pixel| *pixel),
        "the matrix is blank"
    );
}

#[test]
fn the_encoder_and_the_can_peer_reach_the_running_firmware() {
    let Some(path) = firmware() else {
        return;
    };
    let (mut board, uart, encoder, transceiver) = bench();

    let mut sim = Simulation::new();
    sim.set_crystals(board.crystals());
    sim.load_elf(&path).expect("load the firmware");

    assert!(
        run_until(&mut board, &mut sim, 2_000, 200_000, |board, _| {
            terminal_text(board, uart).contains("EVT READY")
        }),
        "the firmware never reached its main loop, terminal shows {:?}",
        terminal_text(&board, uart)
    );

    // A second node on the pair acknowledges the frames the firmware sends and
    // puts one frame of its own on the wire. It joins only now, so its frame
    // cannot be lost while the controller is still being initialised.
    let peer = board.can_bus().borrow_mut().add_peer("ECU");
    board
        .can_bus()
        .borrow_mut()
        .queue_frame(peer, CanFrame::standard(CAN_ID, b"PING"));

    // One clockwise detent is four quarter steps; the EXTI handlers count them.
    let now = sim.now_ns();
    encoder_mut(&mut board, encoder).rotate(Direction::Clockwise, 1, now);
    encoder_mut(&mut board, encoder).press(true);

    assert!(
        run_until(&mut board, &mut sim, 400, 200_000, |board, _| {
            terminal_text(board, uart).contains("\nEVT ENC 4 1\n")
        }),
        "the encoder never reached the firmware, terminal shows {:?}",
        terminal_text(&board, uart)
    );

    // The frame the peer queued has to arrive in FIFO0 and be reported.
    assert!(
        run_until(&mut board, &mut sim, 400, 200_000, |board, _| {
            terminal_text(board, uart).contains("EVT CAN RX")
        }),
        "the peer frame never reached the controller, terminal shows {:?}",
        terminal_text(&board, uart)
    );
    assert!(
        terminal_text(&board, uart).contains("EVT CAN RX 0x00000123 4"),
        "unexpected receive report: {:?}",
        terminal_text(&board, uart)
    );

    // And the frame the firmware transmitted itself was acknowledged on the
    // wire, so the transceiver, the terminator and the peer all took part.
    let records = board.can_bus().borrow().records().len();
    let sent = board
        .can_bus()
        .borrow()
        .records()
        .iter()
        .filter(|record| record.frame.id == u32::from(CAN_ID) && record.acknowledged)
        .count();
    assert!(
        sent > 0,
        "no acknowledged frame on the bus ({records} records)"
    );
    assert!(
        board.device(transceiver).is_some(),
        "the transceiver is part of the bench"
    );
}
