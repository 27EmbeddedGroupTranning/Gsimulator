//! End-to-end tests for the wiring harness: a knob turned on the schematic
//! changes MCU pin levels inside the running simulation, and a terminal both
//! hears the controller and answers it.

use gsim_core::{AccessWidth, MemoryBus};
use gsim_devices::Direction;
use gsim_stm32f1::map;
use gsimulator_gui::board::{
    Board, DeviceKind, PinRef, ENCODER_A, ENCODER_B, ENCODER_SW, UART_RX, UART_TX,
};
use gsimulator_gui::bridge::{self, USART_RX_PINS, USART_TX_PINS};
use gsimulator_gui::canvas::Schematic;
use gsimulator_gui::sim::Simulation;

fn encoder_board(port: u8, a: u8, b: u8, sw: u8) -> (Board, u32) {
    let mut board = Board::new();
    let encoder = board.add_encoder(egui::Pos2::new(500.0, 200.0));
    for (pin, mcu) in [(ENCODER_A, a), (ENCODER_B, b), (ENCODER_SW, sw)] {
        board.connect(
            PinRef::Device {
                device: encoder,
                pin,
            },
            PinRef::Mcu { port, pin: mcu },
        );
    }
    (board, encoder)
}

fn encoder_mut(board: &mut Board, id: u32) -> &mut gsim_devices::Encoder {
    board
        .device_mut(id)
        .unwrap()
        .hardware
        .encoder_mut()
        .unwrap()
}

/// Enables USART1 for transmission and reception, as `MX_USART1_UART_Init()` does.
fn open_usart1(sim: &mut Simulation) {
    let base = map::usart::BASE_1;
    sim.with_soc(|soc| {
        soc.write(
            map::rcc::BASE + map::rcc::APB2ENR,
            AccessWidth::Word,
            1 << 14,
        )
        .unwrap();
        soc.write(base + map::usart::BRR, AccessWidth::Word, 69)
            .unwrap();
        // UE | TE | RE
        soc.write(
            base + map::usart::CR1,
            AccessWidth::Word,
            (1 << 13) | (1 << 3) | (1 << 2),
        )
        .unwrap();
    });
}

/// Makes the controller shift one byte out of USART1.
fn transmit(sim: &mut Simulation, byte: u8) {
    sim.with_soc(|soc| {
        soc.write(
            map::usart::BASE_1 + map::usart::DR,
            AccessWidth::Word,
            u32::from(byte),
        )
        .unwrap();
    });
}

fn uart_terminal(board: &Board, id: u32) -> &gsim_devices::Uart {
    board.device(id).unwrap().hardware.uart().unwrap()
}

#[test]
fn turning_the_knob_drives_the_wired_pins_of_the_running_simulation() {
    let (mut board, encoder) = encoder_board(0, 0, 1, 2);
    let mut sim = Simulation::new();

    // A wired encoder rests with both contacts released (pulled up).
    bridge::service(&mut board, &mut sim);
    let resting = sim.pin_levels();
    assert!(
        resting.level_of(0, 0) && resting.level_of(0, 1),
        "the idle contacts must be pulled high"
    );

    // One clockwise detent: four quarter steps of 2 ms each.
    let now = sim.now_ns();
    encoder_mut(&mut board, encoder).rotate(Direction::Clockwise, 1, now);

    let mut seen = Vec::new();
    let mut last = (true, true);
    for _ in 0..80 {
        // 200 µs slices, so every quarter step is visible on its own.
        bridge::run_frame(&mut board, &mut sim, 2_000, 200_000);
        let levels = sim.pin_levels();
        let current = (levels.level_of(0, 0), levels.level_of(0, 1));
        if current != last {
            seen.push(current);
            last = current;
        }
    }

    assert_eq!(
        seen,
        vec![(false, true), (false, false), (true, false), (true, true)],
        "the contacts must walk the quadrature code once"
    );
    assert_eq!(encoder_mut(&mut board, encoder).detents(), 1);
    assert_eq!(encoder_mut(&mut board, encoder).pending_edges(), 0);
    // Both contacts rest high again once the detent is finished.
    let levels = sim.pin_levels();
    assert!(levels.level_of(0, 0) && levels.level_of(0, 1));
}

#[test]
fn an_unwired_encoder_does_not_touch_any_pin() {
    let mut board = Board::new();
    let encoder = board.add_encoder(egui::Pos2::new(0.0, 0.0));
    let mut sim = Simulation::new();

    let now = sim.now_ns();
    encoder_mut(&mut board, encoder).rotate(Direction::CounterClockwise, 2, now);
    bridge::run_frame(&mut board, &mut sim, 40_000, 1_000_000);

    let levels = sim.pin_levels();
    assert!(!levels.level_of(0, 0) && !levels.level_of(0, 1));
}

#[test]
fn the_button_pulls_the_wired_pin_low_while_pressed() {
    let (mut board, encoder) = encoder_board(0, 0, 1, 2);
    let mut sim = Simulation::new();

    // Released: the module's pull-up keeps `SW` high.
    bridge::service(&mut board, &mut sim);
    assert!(
        sim.pin_levels().level_of(0, 2),
        "an unpressed button must read high"
    );

    encoder_mut(&mut board, encoder).press(true);
    bridge::service(&mut board, &mut sim);
    assert!(
        !sim.pin_levels().level_of(0, 2),
        "pressing the shaft must pull SW low"
    );

    encoder_mut(&mut board, encoder).press(false);
    bridge::service(&mut board, &mut sim);
    assert!(sim.pin_levels().level_of(0, 2));
}

#[test]
fn a_terminal_wired_to_the_port_hears_what_the_controller_sends() {
    let mut board = Board::new();
    let uart = board.add_uart(egui::Pos2::new(500.0, 100.0));
    let (tx_port, tx_pin) = USART_TX_PINS[0];
    board.connect(
        PinRef::Mcu {
            port: tx_port,
            pin: tx_pin,
        },
        PinRef::Device {
            device: uart,
            pin: UART_RX,
        },
    );
    let mut sim = Simulation::new();
    open_usart1(&mut sim);

    transmit(&mut sim, b'O');
    bridge::run_frame(&mut board, &mut sim, 40_000, 1_000_000);
    transmit(&mut sim, b'K');
    bridge::run_frame(&mut board, &mut sim, 40_000, 1_000_000);

    let terminal = uart_terminal(&board, uart);
    assert_eq!(terminal.rx_text(), "OK");
    assert_eq!(terminal.rx_bytes(), 2);
}

#[test]
fn an_unwired_terminal_hears_nothing() {
    let mut board = Board::new();
    let uart = board.add_uart(egui::Pos2::new(500.0, 100.0));
    let mut sim = Simulation::new();
    open_usart1(&mut sim);

    transmit(&mut sim, b'X');
    bridge::run_frame(&mut board, &mut sim, 40_000, 1_000_000);

    assert_eq!(uart_terminal(&board, uart).rx_bytes(), 0);
}

#[test]
fn bytes_typed_into_a_wired_terminal_reach_the_controller_receiver() {
    let mut board = Board::new();
    let uart = board.add_uart(egui::Pos2::new(500.0, 100.0));
    let (rx_port, rx_pin) = USART_RX_PINS[0];
    board.connect(
        PinRef::Device {
            device: uart,
            pin: UART_TX,
        },
        PinRef::Mcu {
            port: rx_port,
            pin: rx_pin,
        },
    );
    let mut sim = Simulation::new();
    open_usart1(&mut sim);

    board
        .device_mut(uart)
        .unwrap()
        .hardware
        .uart_mut()
        .unwrap()
        .send("ok");

    // One byte per slice, and it really landed in the USART data register.
    bridge::service(&mut board, &mut sim);
    assert_eq!(sim.usart_read(0), Some(b'o'));
    assert_eq!(uart_terminal(&board, uart).pending_tx(), 1);

    // Once the firmware read the data register the next byte follows.
    bridge::service(&mut board, &mut sim);
    assert_eq!(sim.usart_read(0), Some(b'k'));
    assert_eq!(uart_terminal(&board, uart).pending_tx(), 0);
}

#[test]
fn a_terminal_keeps_its_bytes_while_the_receiver_is_closed() {
    let mut board = Board::new();
    let uart = board.add_uart(egui::Pos2::new(500.0, 100.0));
    let (rx_port, rx_pin) = USART_RX_PINS[0];
    board.connect(
        PinRef::Device {
            device: uart,
            pin: UART_TX,
        },
        PinRef::Mcu {
            port: rx_port,
            pin: rx_pin,
        },
    );
    let mut sim = Simulation::new();
    // No `open_usart1()`: UE/RE are still clear.

    board
        .device_mut(uart)
        .unwrap()
        .hardware
        .uart_mut()
        .unwrap()
        .send("z");
    let report = bridge::service(&mut board, &mut sim);

    assert_eq!(report.bytes_sent, 0);
    assert_eq!(uart_terminal(&board, uart).pending_tx(), 1);
}

#[test]
fn the_demo_board_has_all_three_parts_on_it() {
    let board = Board::demo();
    let kinds: Vec<DeviceKind> = board.devices().iter().map(|device| device.kind).collect();
    assert_eq!(
        kinds,
        vec![DeviceKind::Led, DeviceKind::Uart, DeviceKind::Encoder]
    );
}

#[test]
fn every_library_entry_is_searchable_by_name_and_alias() {
    for kind in DeviceKind::ALL {
        assert!(!kind.label().is_empty(), "a part without a label");
        assert!(!kind.category().is_empty(), "a part without a category");
        assert!(!kind.summary().is_empty(), "a part without a summary");
        assert!(
            !kind.pin_summary().is_empty(),
            "a part without pins to show"
        );
        assert!(
            kind.matches(&kind.label().to_lowercase()),
            "{} cannot be found by its own label",
            kind.label()
        );
    }
    // The aliases are what a Chinese search box is likely to receive.
    assert!(DeviceKind::Led.matches("二极管"));
    assert!(DeviceKind::Uart.matches("串口"));
    assert!(DeviceKind::Encoder.matches("旋钮"));
    assert!(!DeviceKind::Encoder.matches("二极管"));
}

#[test]
fn a_part_picked_in_the_library_lands_under_the_click() {
    let mut board = Board::new();
    let mut schematic = Schematic::default();
    schematic.place_pending(DeviceKind::Encoder);
    assert_eq!(schematic.pending_device(), Some(DeviceKind::Encoder));

    let click = egui::Pos2::new(500.0, 400.0);
    let id = schematic
        .place_pending_at(&mut board, click)
        .expect("the armed part must be placed");
    let device = board.device(id).expect("the placed device exists");
    assert_eq!(device.kind, DeviceKind::Encoder);
    assert_eq!(device.rect().center(), click);

    // One click places one part; the canvas is back to wiring afterwards.
    assert_eq!(schematic.pending_device(), None);
    assert!(schematic
        .place_pending_at(&mut board, click)
        .is_none());
    assert_eq!(board.devices().len(), 1);

    // `Esc` (and the API behind it) throws an armed part away.
    schematic.place_pending(DeviceKind::Led);
    schematic.cancel_pending();
    assert_eq!(schematic.pending_device(), None);
}

/// Builds the shared test firmware; `None` when no cross compiler is installed.
fn build_firmware() -> Option<std::path::PathBuf> {
    let toolchain = "arm-none-eabi-gcc";
    if std::process::Command::new(toolchain)
        .arg("--version")
        .output()
        .is_err()
    {
        return None;
    }
    let source =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../gsim-cpu/tests/firmware");
    let output = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join("gui-firmware.elf");
    let status = std::process::Command::new(toolchain)
        .current_dir(&source)
        .args([
            "-mcpu=cortex-m3",
            "-mthumb",
            "-O1",
            "-ffreestanding",
            "-fno-builtin",
            "-nostdlib",
            "-nostartfiles",
            "-T",
            "link.ld",
            "-Wl,--gc-sections",
            "-o",
        ])
        .arg(&output)
        .args(["startup.c", "main.c"])
        .status()
        .expect("cannot run the ARM cross compiler");
    assert!(status.success(), "the shared test firmware failed to build");
    Some(output)
}

#[test]
fn a_real_firmware_shows_up_in_the_terminal() {
    let Some(elf) = build_firmware() else {
        eprintln!("arm-none-eabi-gcc not found, skipping the firmware terminal test");
        return;
    };

    // Same wiring as the demo board: USART1 TX (PA9) -> terminal RX.
    let mut board = Board::new();
    let uart = board.add_uart(egui::Pos2::new(500.0, 100.0));
    let (tx_port, tx_pin) = USART_TX_PINS[0];
    board.connect(
        PinRef::Mcu {
            port: tx_port,
            pin: tx_pin,
        },
        PinRef::Device {
            device: uart,
            pin: UART_RX,
        },
    );

    let mut sim = Simulation::new();
    sim.load_elf(&elf).expect("loading the firmware");

    for _ in 0..40 {
        bridge::run_frame(&mut board, &mut sim, 400_000, 1_000_000);
        if uart_terminal(&board, uart).rx_text().contains("OK") {
            break;
        }
    }

    let text = uart_terminal(&board, uart).rx_text().to_string();
    assert!(
        text.starts_with("GSIM"),
        "the terminal should show the firmware banner, got {text:?}"
    );
    assert!(text.contains('T') && text.contains("OK"), "got {text:?}");
}
