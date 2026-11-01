//! Wiring bench end-to-end test for the self test firmware in `firmware/CAN`.
//!
//! The schematic is built the way the README documents it (terminal, LED,
//! encoder, LED matrix, CAN transceiver with a terminator, both crystals) and
//! the real ELF is loaded into it. Everything the firmware reports on USART1
//! therefore had to travel through the same register model, wiring harness and
//! device models the GUI uses.
//!
//! The image is a build artefact: when it is missing the tests only report the
//! skip, so a fresh checkout still passes.

use std::path::PathBuf;

use gsim_core::CanFrame;
use gsim_devices::Direction;
use gsimulator_gui::board::{
    Board, DeviceId, PinRef, CAN_CANH, CAN_CANL, CAN_RXD, CAN_TXD, CRYSTAL_X1, CRYSTAL_X2,
    ENCODER_A, ENCODER_B, ENCODER_SW, MATRIX_CLK, MATRIX_CS, MATRIX_DIN, MATRIX_DOUT, POWER_PIN,
    RESISTOR_A, RESISTOR_B, UART_RX, UART_TX,
};
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

/// Feeds a placed part from a fresh `VCC`/`GND` pair, which is what makes it
/// take part in the simulation at all.
fn power(board: &mut Board, device: DeviceId) {
    let (vcc_pin, gnd_pin) = board
        .device(device)
        .expect("placed device")
        .kind
        .supply_pins()
        .expect("part with a supply");
    let vcc = board.add_vcc(egui::Pos2::new(0.0, 700.0));
    let gnd = board.add_gnd(egui::Pos2::new(140.0, 700.0));
    board.connect(
        PinRef::Device {
            device: vcc,
            pin: POWER_PIN,
        },
        PinRef::Device {
            device,
            pin: vcc_pin,
        },
    );
    board.connect(
        PinRef::Device {
            device: gnd,
            pin: POWER_PIN,
        },
        PinRef::Device {
            device,
            pin: gnd_pin,
        },
    );
}

fn wire(board: &mut Board, device: DeviceId, pin: u8, port: u8, mcu: u8) {
    board.connect(
        PinRef::Device { device, pin },
        PinRef::Mcu { port, pin: mcu },
    );
}

struct Bench {
    board: Board,
    uart: DeviceId,
    matrix: DeviceId,
    encoder: DeviceId,
    led: DeviceId,
    transceiver: DeviceId,
}

/// Builds the schematic the README documents for the self test firmware.
fn bench() -> Bench {
    let mut board = Board::new();

    let uart = board.add_uart(egui::Pos2::new(200.0, 120.0));
    wire(&mut board, uart, UART_RX, 0, 9); /* terminal RX  <- USART1_TX */
    wire(&mut board, uart, UART_TX, 0, 10); /* terminal TX  -> USART1_RX */
    power(&mut board, uart);

    let matrix = board.add_matrix(egui::Pos2::new(520.0, 120.0));
    wire(&mut board, matrix, MATRIX_DIN, 0, 7);
    wire(&mut board, matrix, MATRIX_CLK, 0, 5);
    wire(&mut board, matrix, MATRIX_CS, 0, 4);
    wire(&mut board, matrix, MATRIX_DOUT, 0, 6);
    power(&mut board, matrix);

    let encoder = board.add_encoder(egui::Pos2::new(200.0, 320.0));
    wire(&mut board, encoder, ENCODER_A, 1, 0);
    wire(&mut board, encoder, ENCODER_B, 1, 1);
    wire(&mut board, encoder, ENCODER_SW, 1, 2);
    power(&mut board, encoder);

    let led = board.add_led(egui::Pos2::new(420.0, 320.0));
    wire(&mut board, led, 0, 1, 12); /* anode <- PB12 */
    let gnd = board.add_gnd(egui::Pos2::new(560.0, 320.0));
    board.connect(
        PinRef::Device {
            device: led,
            pin: 1,
        },
        PinRef::Device {
            device: gnd,
            pin: POWER_PIN,
        },
    );

    let transceiver = board.add_can(egui::Pos2::new(520.0, 480.0));
    wire(&mut board, transceiver, CAN_TXD, 0, 12);
    wire(&mut board, transceiver, CAN_RXD, 0, 11);
    power(&mut board, transceiver);
    let resistor = board.add_resistor(egui::Pos2::new(700.0, 480.0));
    board.connect(
        PinRef::Device {
            device: transceiver,
            pin: CAN_CANH,
        },
        PinRef::Device {
            device: resistor,
            pin: RESISTOR_A,
        },
    );
    board.connect(
        PinRef::Device {
            device: transceiver,
            pin: CAN_CANL,
        },
        PinRef::Device {
            device: resistor,
            pin: RESISTOR_B,
        },
    );

    let hse = board.add_hse_crystal(egui::Pos2::new(200.0, 480.0));
    wire(&mut board, hse, CRYSTAL_X1, 3, 0);
    wire(&mut board, hse, CRYSTAL_X2, 3, 1);
    let lse = board.add_lse_crystal(egui::Pos2::new(360.0, 620.0));
    wire(&mut board, lse, CRYSTAL_X1, 2, 14);
    wire(&mut board, lse, CRYSTAL_X2, 2, 15);

    Bench {
        board,
        uart,
        matrix,
        encoder,
        led,
        transceiver,
    }
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

fn matrix_model(board: &Board, id: DeviceId) -> std::rc::Rc<std::cell::RefCell<gsim_devices::Max7219>> {
    board
        .device(id)
        .expect("matrix")
        .hardware
        .matrix()
        .cloned()
        .expect("matrix model")
}

fn encoder_mut(board: &mut Board, id: DeviceId) -> &mut gsim_devices::Encoder {
    board
        .device_mut(id)
        .expect("encoder")
        .hardware
        .encoder_mut()
        .expect("encoder model")
}

/// Runs `frames` slices of `budget` instructions, with 1 ms of simulated time
/// per slice, stopping early when `done` says so.
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
    let Bench {
        mut board,
        uart,
        matrix,
        led,
        ..
    } = bench();

    let mut sim = Simulation::new();
    sim.set_crystals(board.crystals());
    sim.load_elf(&path).expect("load the firmware");

    // Sample the LED during the first milliseconds of the run: the GPIO test
    // drives PB12 high for a millisecond before it continues.
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

    let model = matrix_model(&board, matrix);
    let model = model.borrow();
    assert!(model.words() > 0, "the chain never latched a word");
    assert!(model.chips()[0].enabled(), "the matrix is still shut down");
    assert_eq!(model.chips()[0].intensity(), 7);
    assert!(
        model.chips()[0].pixels().iter().flatten().any(|pixel| *pixel),
        "the matrix is blank"
    );
}

#[test]
fn the_encoder_and_the_can_peer_reach_the_running_firmware() {
    let Some(path) = firmware() else {
        return;
    };
    let Bench {
        mut board,
        uart,
        encoder,
        transceiver,
        ..
    } = bench();

    let mut sim = Simulation::new();
    sim.set_crystals(board.crystals());
    sim.load_elf(&path).expect("load the firmware");

    // A second node on the pair acknowledges the frames the firmware sends and
    // puts one frame of its own on the wire.
    let peer = board.can_bus().borrow_mut().add_peer("ECU");
    board.can_bus().borrow_mut().queue_frame(
        peer,
        CanFrame::standard(CAN_ID, b"PING"),
    );

    assert!(
        run_until(&mut board, &mut sim, 2_000, 200_000, |board, _| {
            terminal_text(board, uart).contains("EVT READY")
        }),
        "the firmware never reached its main loop, terminal shows {:?}",
        terminal_text(&board, uart)
    );

    // One clockwise detent is four quarter steps; the EXTI handlers count them.
    let now = sim.now_ns();
    encoder_mut(&mut board, encoder).rotate(Direction::Clockwise, 1, now);
    encoder_mut(&mut board, encoder).press(true);

    assert!(
        run_until(&mut board, &mut sim, 400, 200_000, |board, _| {
            let text = terminal_text(board, uart);
            text.contains("\nEVT ENC 4 1\n")
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
    let sent = board
        .can_bus()
        .borrow()
        .records()
        .iter()
        .filter(|record| record.frame.id == u32::from(CAN_ID) && record.acknowledged)
        .count();
    assert!(
        sent > 0,
        "no acknowledged frame on the bus (records: {})",
        board.can_bus().borrow().records().len()
    );
    assert!(
        board.device(transceiver).is_some(),
        "the transceiver is part of the bench"
    );
}
