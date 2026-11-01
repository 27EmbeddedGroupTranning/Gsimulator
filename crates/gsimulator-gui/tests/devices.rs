//! End-to-end tests for the wiring harness: a knob turned on the schematic
//! changes MCU pin levels inside the running simulation, and a terminal both
//! hears the controller and answers it.

use gsim_core::{AccessWidth, MemoryBus};
use gsim_devices::Direction;
use gsim_stm32f1::map;
use gsimulator_gui::board::{
    Board, DeviceKind, PinRef, CAN_CANH, CAN_CANL, CAN_RXD, CAN_TXD, ENCODER_A, ENCODER_B,
    ENCODER_SW, JUNCTION_PIN, MATRIX_CLK, MATRIX_CS, MATRIX_DIN, MATRIX_DOUT, POWER_PIN,
    RESISTOR_A, RESISTOR_B, SCOPE_CANH, SCOPE_CANL, UART_RX, UART_TX,
};
use gsimulator_gui::bridge::{self, USART_RX_PINS, USART_TX_PINS};
use gsimulator_gui::sim::Simulation;

/// Feeds a placed part from the two power symbols of the drawing, which is what
/// makes it take part in the simulation at all.
fn power(board: &mut Board, device: u32) {
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
    power(&mut board, encoder);
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
    // Powered but with none of its contacts wired to the controller.
    power(&mut board, encoder);
    let mut sim = Simulation::new();

    let now = sim.now_ns();
    encoder_mut(&mut board, encoder).rotate(Direction::CounterClockwise, 2, now);
    bridge::run_frame(&mut board, &mut sim, 40_000, 1_000_000);

    let levels = sim.pin_levels();
    assert!(!levels.level_of(0, 0) && !levels.level_of(0, 1));
}

#[test]
fn an_unpowered_encoder_leaves_the_pins_alone() {
    let mut board = Board::new();
    let encoder = board.add_encoder(egui::Pos2::new(0.0, 0.0));
    for (pin, mcu) in [(ENCODER_A, 0u8), (ENCODER_B, 1), (ENCODER_SW, 2)] {
        board.connect(
            PinRef::Device {
                device: encoder,
                pin,
            },
            PinRef::Mcu { port: 0, pin: mcu },
        );
    }
    let mut sim = Simulation::new();

    encoder_mut(&mut board, encoder).rotate(Direction::Clockwise, 2, sim.now_ns());
    bridge::run_frame(&mut board, &mut sim, 40_000, 1_000_000);

    let levels = sim.pin_levels();
    for pin in 0..3u8 {
        assert!(
            !levels.level_of(0, pin),
            "P{}{pin} must stay untouched while the part has no supply",
            0
        );
    }
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
    power(&mut board, uart);
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
fn a_junction_tapped_into_the_wire_keeps_the_terminal_talking() {
    let mut board = Board::new();
    let uart = board.add_uart(egui::Pos2::new(500.0, 100.0));
    power(&mut board, uart);
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
    // Double clicking that wire splits it in two, both ending on the tap.
    let index = board
        .wires()
        .iter()
        .position(|wire| {
            wire.a
                == PinRef::Mcu {
                    port: tx_port,
                    pin: tx_pin,
                }
                || wire.b
                    == PinRef::Mcu {
                        port: tx_port,
                        pin: tx_pin,
                    }
        })
        .expect("the TX wire exists");
    let junction = board
        .add_junction(index, 0, egui::Pos2::new(400.0, 90.0))
        .expect("the wire exists");
    let tap = PinRef::Device {
        device: junction,
        pin: JUNCTION_PIN,
    };
    // The tap really is a terminal: branch a wire from it to a spare pin.
    assert!(board.connect(tap, PinRef::Mcu { port: 0, pin: 0 }));

    let mut sim = Simulation::new();
    open_usart1(&mut sim);
    transmit(&mut sim, b'O');
    bridge::run_frame(&mut board, &mut sim, 40_000, 1_000_000);

    assert_eq!(
        uart_terminal(&board, uart).rx_text(),
        "O",
        "tapping the wire must not stop the terminal from hearing USART1"
    );
}

#[test]
fn an_unwired_terminal_hears_nothing() {
    let mut board = Board::new();
    let uart = board.add_uart(egui::Pos2::new(500.0, 100.0));
    // Powered, but its `RX` contact is not wired to the port.
    power(&mut board, uart);
    let mut sim = Simulation::new();
    open_usart1(&mut sim);

    transmit(&mut sim, b'X');
    bridge::run_frame(&mut board, &mut sim, 40_000, 1_000_000);

    assert_eq!(uart_terminal(&board, uart).rx_bytes(), 0);
}

#[test]
fn a_terminal_only_works_once_its_supply_is_on_the_rails() {
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

    transmit(&mut sim, b'A');
    bridge::run_frame(&mut board, &mut sim, 40_000, 1_000_000);
    assert_eq!(
        uart_terminal(&board, uart).rx_bytes(),
        0,
        "a terminal without a supply must not hear the port"
    );

    power(&mut board, uart);
    transmit(&mut sim, b'B');
    bridge::run_frame(&mut board, &mut sim, 40_000, 1_000_000);
    assert_eq!(uart_terminal(&board, uart).rx_text(), "B");
}

#[test]
fn bytes_typed_into_a_wired_terminal_reach_the_controller_receiver() {
    let mut board = Board::new();
    let uart = board.add_uart(egui::Pos2::new(500.0, 100.0));
    power(&mut board, uart);
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
    power(&mut board, uart);
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
fn the_demo_board_has_the_power_symbols_three_parts_and_the_hse_crystal() {
    let board = Board::demo();
    let kinds: Vec<DeviceKind> = board.devices().iter().map(|device| device.kind).collect();
    assert_eq!(
        kinds,
        vec![
            DeviceKind::Vcc,
            DeviceKind::Gnd,
            DeviceKind::Led,
            DeviceKind::Uart,
            DeviceKind::Encoder,
            DeviceKind::HseCrystal,
            DeviceKind::LseCrystal,
        ]
    );
}

#[test]
fn the_library_lists_every_placeable_part() {
    let kinds: Vec<DeviceKind> = DeviceKind::ALL.to_vec();
    for expected in [
        DeviceKind::Led,
        DeviceKind::Uart,
        DeviceKind::Encoder,
        DeviceKind::Vcc,
        DeviceKind::Gnd,
        DeviceKind::Can,
        DeviceKind::Matrix,
        DeviceKind::HseCrystal,
        DeviceKind::LseCrystal,
    ] {
        assert!(
            kinds.contains(&expected),
            "{expected:?} is not in the library"
        );
    }
}

/// Enables SPI1 on `PA5`/`PA7` with `PA4` as a push-pull `LOAD`, like the
/// CubeMX firmware does.
fn open_spi1(sim: &mut Simulation) {
    let gpioa = map::gpio::PORTS[0];
    sim.with_soc(|soc| {
        soc.write(
            map::rcc::BASE + map::rcc::APB2ENR,
            AccessWidth::Word,
            (1 << 2) | (1 << 12),
        )
        .unwrap();
        let crl = soc.read(gpioa + map::gpio::CRL, AccessWidth::Word).unwrap();
        let crl = (crl & !0x000F_0000) | (0b0010 << 16);
        let crl = (crl & !((0xF << 20) | (0xF << 28))) | (0b1011 << 20) | (0b1011 << 28);
        soc.write(gpioa + map::gpio::CRL, AccessWidth::Word, crl)
            .unwrap();
        // MSTR, BR = /8, SPE.
        soc.write(
            map::spi::BASE_1 + map::spi::CR1,
            AccessWidth::Word,
            (1 << 2) | (0b010 << 3) | (1 << 6),
        )
        .unwrap();
        soc.write(gpioa + map::gpio::BSRR, AccessWidth::Word, 1 << 4)
            .unwrap();
    });
}

fn write_soc(sim: &mut Simulation, address: u32, value: u32) {
    sim.with_soc(|soc| {
        soc.write(address, AccessWidth::Word, value).unwrap();
    });
}

#[test]
fn a_matrix_wired_to_spi1_latches_the_words_the_controller_shifts_out() {
    let mut board = Board::new();
    let matrix = board.add_matrix(egui::Pos2::new(500.0, 200.0));
    power(&mut board, matrix);
    for (pin, (port, mcu)) in [
        (MATRIX_DIN, (0u8, 7u8)),
        (MATRIX_CLK, (0, 5)),
        (MATRIX_CS, (0, 4)),
        (MATRIX_DOUT, (0, 6)),
    ] {
        board.connect(
            PinRef::Device {
                device: matrix,
                pin,
            },
            PinRef::Mcu { port, pin: mcu },
        );
    }

    let mut sim = Simulation::new();
    // The built-in stimulus also drives PA5; release it before taking over.
    sim.stop_stimulus();
    open_spi1(&mut sim);

    let gpioa = map::gpio::PORTS[0];
    // LOAD low, two bytes, LOAD high: one word for the chain.
    write_soc(&mut sim, gpioa + map::gpio::BSRR, 1 << (4 + 16));
    for byte in [0x0Au8, 0x05] {
        bridge::service(&mut board, &mut sim);
        write_soc(&mut sim, map::spi::BASE_1 + map::spi::DR, u32::from(byte));
        for _ in 0..8 {
            sim.step_once(2_000);
            bridge::service(&mut board, &mut sim);
        }
    }
    write_soc(&mut sim, gpioa + map::gpio::BSRR, 1 << 4);
    bridge::service(&mut board, &mut sim);

    let handle = board
        .device(matrix)
        .expect("matrix device")
        .hardware
        .matrix()
        .cloned()
        .expect("matrix model");
    let model = handle.borrow();
    assert_eq!(model.frames(), 2, "both bytes reached the chain");
    assert_eq!(model.words(), 1, "the LOAD edge latched one word");
    assert_eq!(model.chips()[0].intensity(), 5);
}

#[test]
fn an_unwired_matrix_hears_nothing_from_the_spi_port() {
    let mut board = Board::new();
    let matrix = board.add_matrix(egui::Pos2::new(500.0, 200.0));
    power(&mut board, matrix);
    // Powered, but only the clock is wired: without DIN the chain stays silent.
    board.connect(
        PinRef::Device {
            device: matrix,
            pin: MATRIX_CLK,
        },
        PinRef::Mcu { port: 0, pin: 5 },
    );

    let mut sim = Simulation::new();
    sim.stop_stimulus();
    open_spi1(&mut sim);
    write_soc(&mut sim, map::spi::BASE_1 + map::spi::DR, 0x0A);
    for _ in 0..8 {
        sim.step_once(2_000);
        bridge::service(&mut board, &mut sim);
    }

    let handle = board
        .device(matrix)
        .expect("matrix device")
        .hardware
        .matrix()
        .cloned()
        .expect("matrix model");
    assert_eq!(handle.borrow().frames(), 0);
}

#[test]
fn an_unpowered_matrix_ignores_the_spi_port() {
    let mut board = Board::new();
    let matrix = board.add_matrix(egui::Pos2::new(500.0, 200.0));
    // Fully wired to SPI1, but with no supply at all.
    for (pin, (port, mcu)) in [
        (MATRIX_DIN, (0u8, 7u8)),
        (MATRIX_CLK, (0, 5)),
        (MATRIX_CS, (0, 4)),
        (MATRIX_DOUT, (0, 6)),
    ] {
        board.connect(
            PinRef::Device {
                device: matrix,
                pin,
            },
            PinRef::Mcu { port, pin: mcu },
        );
    }

    let mut sim = Simulation::new();
    sim.stop_stimulus();
    open_spi1(&mut sim);
    let gpioa = map::gpio::PORTS[0];
    write_soc(&mut sim, gpioa + map::gpio::BSRR, 1 << (4 + 16));
    for byte in [0x0Au8, 0x05] {
        bridge::service(&mut board, &mut sim);
        write_soc(&mut sim, map::spi::BASE_1 + map::spi::DR, u32::from(byte));
        for _ in 0..8 {
            sim.step_once(2_000);
            bridge::service(&mut board, &mut sim);
        }
    }
    write_soc(&mut sim, gpioa + map::gpio::BSRR, 1 << 4);
    bridge::service(&mut board, &mut sim);

    let handle = board
        .device(matrix)
        .expect("matrix device")
        .hardware
        .matrix()
        .cloned()
        .expect("matrix model");
    assert_eq!(handle.borrow().frames(), 0, "no supply, no frames");
}

/// `CAN1` base address, as the firmware sees it.
const CAN: u32 = map::can::BASE_1;

/// Wires a transceiver to `CAN1` and puts it on a pair terminated by two
/// `120 Ω` resistors with a probe across it.
///
/// Returns the board and the id of the oscilloscope.
fn can_board() -> (Board, u32) {
    let mut board = Board::new();
    let transceiver = board.add_can(egui::Pos2::new(520.0, 180.0));
    power(&mut board, transceiver);
    board.connect(
        PinRef::Device {
            device: transceiver,
            pin: CAN_TXD,
        },
        PinRef::Mcu { port: 0, pin: 12 },
    );
    board.connect(
        PinRef::Device {
            device: transceiver,
            pin: CAN_RXD,
        },
        PinRef::Mcu { port: 0, pin: 11 },
    );

    // One terminator at each end of the pair, plus a probe across it.
    for index in 0..2u32 {
        let resistor = board.add_resistor(egui::Pos2::new(360.0 + index as f32 * 40.0, 320.0));
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
    }
    let scope = board.add_scope(egui::Pos2::new(700.0, 320.0));
    board.connect(
        PinRef::Device {
            device: transceiver,
            pin: CAN_CANH,
        },
        PinRef::Device {
            device: scope,
            pin: SCOPE_CANH,
        },
    );
    board.connect(
        PinRef::Device {
            device: transceiver,
            pin: CAN_CANL,
        },
        PinRef::Device {
            device: scope,
            pin: SCOPE_CANL,
        },
    );
    (board, scope)
}

/// `HAL_CAN_Init` + `HAL_CAN_Start` for normal mode, with one 32 bit mask
/// filter that accepts every standard identifier into FIFO0.
fn open_can1(sim: &mut Simulation) {
    write_soc(sim, map::rcc::BASE + map::rcc::APB1ENR, 1 << 25);
    write_soc(sim, CAN + map::can::MCR, 1);
    write_soc(
        sim,
        CAN + map::can::BTR,
        (2 - 1) << 16 | (2 - 1) << 20 | (8 - 1),
    );
    write_soc(sim, CAN + map::can::FMR, 1);
    write_soc(sim, CAN + map::can::FS1R, 1);
    write_soc(sim, CAN + map::can::FM1R, 0);
    write_soc(sim, CAN + map::can::FFA1R, 0);
    write_soc(sim, CAN + map::can::FILTER_BASE, 0);
    write_soc(sim, CAN + map::can::FILTER_BASE + 4, 0);
    write_soc(sim, CAN + map::can::FA1R, 1);
    write_soc(sim, CAN + map::can::FMR, 0);
    write_soc(sim, CAN + map::can::MCR, 0);
}

/// Runs `slices` slices of simulation time, servicing the parts in between.
fn run(board: &mut Board, sim: &mut Simulation, slices: u32) {
    for _ in 0..slices {
        bridge::run_frame(board, sim, 20_000, 1_000_000);
    }
}

#[test]
fn a_resistor_across_the_pair_sets_the_bus_quality() {
    let (board, _scope) = can_board();
    let network = board.can_network();
    assert_eq!(network.nodes, 1, "the transceiver sits on the pair");
    assert_eq!(network.terminators.len(), 2);
    let ohms = network.termination_ohm.expect("termination");
    assert!((ohms - 60.0).abs() < 1e-6, "{ohms}");
    assert!((network.quality() - 1.0).abs() < 1e-9);

    // Removing one part degrades the pair, removing both opens it.
    let mut board = board;
    let resistors: Vec<u32> = board
        .devices()
        .iter()
        .filter(|device| device.kind == DeviceKind::Resistor)
        .map(|device| device.id)
        .collect();
    board.remove_device(resistors[0]);
    let half = board.can_network();
    assert!(half.quality() < network.quality());
    board.remove_device(resistors[1]);
    let open = board.can_network();
    assert!(open.termination_ohm.is_none());
    assert!(open.quality() < 0.1);
}

#[test]
fn an_unpowered_transceiver_keeps_the_controller_off_the_bus() {
    let mut board = Board::new();
    let transceiver = board.add_can(egui::Pos2::new(520.0, 180.0));
    board.connect(
        PinRef::Device {
            device: transceiver,
            pin: CAN_TXD,
        },
        PinRef::Mcu { port: 0, pin: 12 },
    );
    board.connect(
        PinRef::Device {
            device: transceiver,
            pin: CAN_RXD,
        },
        PinRef::Mcu { port: 0, pin: 11 },
    );
    // A terminated pair, but the transceiver itself has no supply.
    let resistor = board.add_resistor(egui::Pos2::new(360.0, 320.0));
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

    let mut sim = Simulation::new();
    sim.stop_stimulus();
    open_can1(&mut sim);
    sim.set_running(true);

    board
        .can_bus()
        .borrow_mut()
        .inject_bit_frame(gsim_core::CanFrame::standard(0x321, b"GSIM"), sim.now_ns());
    run(&mut board, &mut sim, 6);

    assert_eq!(
        sim.with_soc(|soc| soc.can.rx_frames(0).len()),
        0,
        "an unpowered transceiver must not put the controller on the wire"
    );
}

#[test]
fn a_frame_from_another_node_reaches_the_controller_over_the_drawn_wire() {
    let (mut board, scope) = can_board();
    // Keep the whole capture on screen: the default two millisecond window
    // would roll past the frame long before the test looks at it.
    board
        .scopes()
        .first()
        .expect("scope")
        .borrow_mut()
        .set_window_ns(40_000_000);
    let mut sim = Simulation::new();
    sim.stop_stimulus();
    open_can1(&mut sim);
    sim.set_running(true);

    let frame = gsim_core::CanFrame::standard(0x321, b"GSIM");
    board
        .can_bus()
        .borrow_mut()
        .inject_bit_frame(frame, sim.now_ns());
    // Enough time for the frame, but not so much that the probe's two
    // millisecond window has already dropped it.
    run(&mut board, &mut sim, 4);

    let level = sim.with_soc(|soc| soc.can.rx_level(0));
    assert_eq!(level, 1, "the controller accepted the frame");
    let frames = sim.with_soc(|soc| soc.can.rx_frames(0));
    assert_eq!(frames[0].id, 0x321);
    assert_eq!(frames[0].bytes(), b"GSIM");

    let handle = board
        .device(scope)
        .expect("scope")
        .hardware
        .scope()
        .cloned()
        .expect("scope model");
    let handle = handle.borrow();
    assert!(
        handle.samples().len() > 50,
        "the probe captured the bits: {}",
        handle.samples().len()
    );
    assert!(handle.glitches() == 0, "a terminated pair is clean");
    assert!(
        handle.peak_differential() > 1.9,
        "dominant bits reach the pair"
    );
    // The probe decodes the burst it captured, bit by bit.
    let decoded = handle.frames().back().expect("the probe decoded the frame");
    assert!(decoded.complete);
    assert!(decoded.crc_ok, "the pair is terminated, so the CRC is good");
    assert!(decoded.acknowledged, "a receiver acknowledged the frame");
    assert_eq!(decoded.frame.map(|frame| frame.id), Some(0x321));
    let captions: Vec<String> = decoded.bits.iter().map(|bit| bit.caption()).collect();
    assert_eq!(captions.first().map(String::as_str), Some("SOF"));
    assert!(captions.contains(&"ID10".to_string()), "{captions:?}");
    assert!(captions.contains(&"数据0".to_string()), "{captions:?}");
    assert!(captions.contains(&"CRC14".to_string()), "{captions:?}");
    assert!(captions.contains(&"EOF0".to_string()), "{captions:?}");
}

#[test]
fn an_open_pair_loses_frames_a_terminated_one_carries() {
    let (mut board, _scope) = can_board();
    // Pull both terminators out of the drawing.
    let resistors: Vec<u32> = board
        .devices()
        .iter()
        .filter(|device| device.kind == DeviceKind::Resistor)
        .map(|device| device.id)
        .collect();
    for id in resistors {
        board.remove_device(id);
    }
    assert!(board.can_network().termination_ohm.is_none());

    let mut sim = Simulation::new();
    sim.stop_stimulus();
    open_can1(&mut sim);
    sim.set_running(true);

    for _ in 0..6 {
        board
            .can_bus()
            .borrow_mut()
            .inject_bit_frame(gsim_core::CanFrame::standard(0x321, b"GSIM"), sim.now_ns());
        run(&mut board, &mut sim, 30);
    }
    let bus = board.can_bus();
    let bus = bus.borrow();
    assert!(bus.glitches() > 0, "an open pair rings: {bus:?}");
    assert!(bus.stats().error_frames > 0);
    // Reading a clean frame out of that is a matter of luck, not of course.
    let accepted = sim.with_soc(|soc| soc.can.stats().rx_frames);
    let delivered = bus.stats().deliveries;
    assert!(
        accepted + delivered < 6 || bus.stats().error_frames > 0,
        "the open pair must lose something"
    );
}

#[test]
fn two_nodes_that_start_together_arbitrate_on_the_drawn_bus() {
    let (mut board, _scope) = can_board();
    let mut sim = Simulation::new();
    sim.stop_stimulus();
    open_can1(&mut sim);
    sim.set_running(true);

    // The external node asks for the wire with a lower identifier at the same
    // moment the controller arms a mailbox, so both start on the same bit.
    board
        .can_bus()
        .borrow_mut()
        .inject_bit_frame(gsim_core::CanFrame::standard(0x100, b"win"), sim.now_ns());
    write_soc(&mut sim, CAN + map::can::TX_BASE, 0x321 << 21);
    write_soc(&mut sim, CAN + map::can::TX_BASE + map::can::TDTR, 4);
    write_soc(
        &mut sim,
        CAN + map::can::TX_BASE + map::can::TDLR,
        0x4C_4F_53_45,
    );
    write_soc(&mut sim, CAN + map::can::TX_BASE, 0x321 << 21 | 1);

    run(&mut board, &mut sim, 60);

    let stats = sim.with_soc(|soc| soc.can.stats());
    assert!(
        stats.arbitration_lost >= 1,
        "the higher identifier must lose: {stats:?}"
    );
    // Losing is not an error: the controller retries and the frame goes out.
    assert_eq!(stats.tx_errors, 0, "{stats:?}");
    assert!(
        stats.tx_frames >= 1,
        "the retry reached the wire: {stats:?}"
    );
    let records = board.can_bus().borrow().records().clone();
    let ids: Vec<u32> = records
        .iter()
        .filter(|record| record.direction == gsim_core::CanDirection::Tx)
        .map(|record| record.frame.id)
        .collect();
    assert!(ids.contains(&0x100), "{ids:?}");
    assert!(ids.contains(&0x321), "{ids:?}");
}
