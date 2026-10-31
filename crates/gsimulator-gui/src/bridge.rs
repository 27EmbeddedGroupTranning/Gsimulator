//! Wiring harness between the simulated MCU and the parts on the schematic.
//!
//! Every slice of simulated time the front end first asks the bridge to service
//! the devices, then advances the simulation. Servicing means:
//!
//! 1. hand the peripheral events produced by the SoC since the last slice to the
//!    device they are wired to (a UART terminal only hears its own port when its
//!    `RX` contact is actually wired to that USART's `TX` pin, the CAN
//!    transceiver only sees frames when `TXD` reaches `CAN1_TX`, ...),
//! 2. release the quadrature edges an encoder has due and drive the MCU pins it
//!    is wired to,
//! 3. inject the bytes queued in a terminal into the MCU receiver and the frames
//!    the bus delivered into the CAN controller,
//! 4. tell the simulation which LED matrix (if any) primes the SPI `MISO` line.
//!
//! Keeping this out of the devices makes them testable on their own, and keeping
//! it out of `App` makes the timing loop easy to reason about.

use crate::board::{
    Board, DeviceId, DeviceKind, Hardware, PinRef, CAN_RXD, CAN_TXD, ENCODER_A, ENCODER_B,
    ENCODER_SW, MATRIX_CLK, MATRIX_CS, MATRIX_DIN, MATRIX_DOUT, UART_RX, UART_TX,
};
use crate::sim::Simulation;

/// `TX` pin of USART1/2/3 on the F103 (default mapping, no remap).
pub const USART_TX_PINS: [(u8, u8); 3] = [(0, 9), (0, 2), (1, 10)];
/// `RX` pin of USART1/2/3.
pub const USART_RX_PINS: [(u8, u8); 3] = [(0, 10), (0, 3), (1, 11)];
/// `CAN1_TX` of the bxCAN controller.
pub const CAN1_TX: (u8, u8) = (0, 12);
/// `CAN1_RX` of the bxCAN controller.
pub const CAN1_RX: (u8, u8) = (0, 11);
/// `SCK` pin of SPI1/2/3 (default mapping, no remap).
pub const SPI_SCK_PINS: [(u8, u8); 3] = [(0, 5), (1, 13), (2, 10)];
/// `MISO` pin of SPI1/2/3.
pub const SPI_MISO_PINS: [(u8, u8); 3] = [(0, 6), (1, 14), (2, 11)];
/// `MOSI` pin of SPI1/2/3.
pub const SPI_MOSI_PINS: [(u8, u8); 3] = [(0, 7), (1, 15), (2, 12)];

/// Bytes one terminal may push into the MCU per slice.
const MAX_BYTES_PER_SLICE: usize = 1;
/// Frames one transceiver may hand to the controller per slice.
const MAX_CAN_FRAMES_PER_SLICE: usize = 4;

/// What the bridge did, mostly for tests and the status line.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ServiceReport {
    /// Quadrature edges applied this slice.
    pub encoder_edges: u32,
    /// Bytes injected into a controller receiver.
    pub bytes_sent: usize,
    /// Frames handed to the CAN controller.
    pub can_frames: usize,
    /// Events handed to a device.
    pub events_delivered: usize,
}

/// What a placed part is wired to, resolved once per slice.
enum Attach {
    Uart {
        listening: bool,
        answering: bool,
        unit: u8,
    },
    Encoder {
        phases: [(u8, u8); 2],
        button: Option<(u8, u8)>,
    },
    Can {
        /// `true` when `TXD` reaches `CAN1_TX`.
        transmitting: bool,
        /// `true` when `RXD` drives `CAN1_RX`.
        receiving: bool,
    },
    Matrix {
        /// SPI port the chain is wired to, if both `DIN` and `CLK` land on it.
        unit: Option<u8>,
        /// MCU pin driving `CS`/`LOAD`.
        cs: Option<(u8, u8)>,
        /// `DOUT` reaches the `MISO` pin of `unit`.
        miso: bool,
    },
}

struct Wiring {
    id: DeviceId,
    attach: Attach,
}

impl Wiring {
    fn collect(board: &Board) -> Vec<Wiring> {
        board
            .devices()
            .iter()
            .filter_map(|device| {
                let pin = |index: u8| {
                    board.mcu_pin_for(PinRef::Device {
                        device: device.id,
                        pin: index,
                    })
                };
                let attach = match (&device.kind, &device.hardware) {
                    (DeviceKind::Uart, Hardware::Uart(uart)) => {
                        let unit = uart.unit().min(2);
                        let (tx_port, tx_pin) = USART_TX_PINS[usize::from(unit)];
                        let (rx_port, rx_pin) = USART_RX_PINS[usize::from(unit)];
                        Attach::Uart {
                            listening: board.is_connected(
                                PinRef::Mcu {
                                    port: tx_port,
                                    pin: tx_pin,
                                },
                                PinRef::Device {
                                    device: device.id,
                                    pin: UART_RX,
                                },
                            ),
                            answering: board.is_connected(
                                PinRef::Device {
                                    device: device.id,
                                    pin: UART_TX,
                                },
                                PinRef::Mcu {
                                    port: rx_port,
                                    pin: rx_pin,
                                },
                            ),
                            unit,
                        }
                    }
                    (DeviceKind::Encoder, Hardware::Encoder(_)) => Attach::Encoder {
                        phases: [
                            pin(ENCODER_A).unwrap_or((0xFF, 0xFF)),
                            pin(ENCODER_B).unwrap_or((0xFF, 0xFF)),
                        ],
                        button: pin(ENCODER_SW),
                    },
                    (DeviceKind::Can, Hardware::Can(_)) => Attach::Can {
                        transmitting: pin(CAN_TXD) == Some(CAN1_TX),
                        receiving: pin(CAN_RXD) == Some(CAN1_RX),
                    },
                    (DeviceKind::Matrix, Hardware::Matrix(_)) => {
                        let clk = pin(MATRIX_CLK);
                        let din = pin(MATRIX_DIN);
                        let unit = clk
                            .and_then(|clk| SPI_SCK_PINS.iter().position(|pin| *pin == clk))
                            .filter(|unit| din == Some(SPI_MOSI_PINS[*unit]))
                            .map(|unit| unit as u8);
                        let miso = unit.is_some_and(|unit| {
                            pin(MATRIX_DOUT) == Some(SPI_MISO_PINS[usize::from(unit)])
                        });
                        Attach::Matrix {
                            unit,
                            cs: pin(MATRIX_CS),
                            miso,
                        }
                    }
                    _ => return None,
                };
                Some(Wiring {
                    id: device.id,
                    attach,
                })
            })
            .collect()
    }
}

/// Services every device once, before the next slice of simulation runs.
pub fn service(board: &mut Board, sim: &mut Simulation) -> ServiceReport {
    let events = sim.take_events();
    let now_ns = sim.now_ns();
    let wiring = Wiring::collect(board);
    let mut report = ServiceReport::default();

    for entry in &wiring {
        let Some(device) = board
            .devices_mut()
            .iter_mut()
            .find(|device| device.id == entry.id)
        else {
            continue;
        };
        match (&mut device.hardware, &entry.attach) {
            (
                Hardware::Uart(uart),
                Attach::Uart {
                    listening,
                    answering,
                    unit,
                },
            ) => {
                if *listening {
                    for event in &events {
                        uart.on_event(event);
                        report.events_delivered += 1;
                    }
                }
                if *answering {
                    for _ in 0..MAX_BYTES_PER_SLICE {
                        let Some(byte) = uart.next_outgoing() else {
                            break;
                        };
                        if sim.send_uart(usize::from(*unit), byte) {
                            report.bytes_sent += 1;
                        } else {
                            // Receiver busy: keep the byte for the next slice.
                            uart.requeue(byte);
                            break;
                        }
                    }
                }
            }
            (
                Hardware::Encoder(encoder),
                Attach::Encoder {
                    phases,
                    button,
                },
            ) => {
                report.encoder_edges += encoder.poll(now_ns);
                // The contacts hold their level: drive them even between
                // edges, so the MCU reads the encoder state right away.
                let phase = encoder.phase();
                if phases[0].0 != 0xFF {
                    sim.drive_pin(phases[0], phase.a);
                }
                if phases[1].0 != 0xFF {
                    sim.drive_pin(phases[1], phase.b);
                }
                if let Some(pin) = button {
                    // The module pulls the contact up; pressing it closes the
                    // switch to ground, so the pin goes low.
                    sim.drive_pin(*pin, !encoder.button());
                }
            }
            (
                Hardware::Can(transceiver),
                Attach::Can {
                    transmitting,
                    receiving,
                },
            ) => {
                let mut transceiver = transceiver.borrow_mut();
                // A part that is not wired to the controller sees nothing.
                if *transmitting {
                    for event in &events {
                        transceiver.on_event(event, now_ns);
                        report.events_delivered += 1;
                    }
                }
                if *receiving {
                    for _ in 0..MAX_CAN_FRAMES_PER_SLICE {
                        let Some(frame) = transceiver.poll() else {
                            break;
                        };
                        sim.receive_can(0, frame);
                        report.can_frames += 1;
                    }
                }
            }
            (
                Hardware::Matrix(matrix),
                Attach::Matrix {
                    unit,
                    cs,
                    miso: _,
                },
            ) => {
                let (Some(unit), Some(cs)) = (*unit, *cs) else {
                    continue;
                };
                let mut matrix = matrix.borrow_mut();
                matrix.set_wiring(unit, cs.0, cs.1);
                for event in &events {
                    matrix.on_event(event);
                    report.events_delivered += 1;
                }
            }
            _ => {}
        }
    }

    // Exactly one matrix may prime the `MISO` line of the port it is wired to;
    // the simulation samples it between instruction chunks.
    let attached = wiring.iter().find_map(|entry| match entry.attach {
        Attach::Matrix {
            unit: Some(_),
            cs: Some(_),
            miso: true,
        } => board
            .device(entry.id)
            .and_then(|device| device.hardware.matrix().cloned()),
        _ => None,
    });
    sim.attach_matrix(attached);

    report
}

/// Splits one frame of `budget` instructions into slices of about `slice_ns`
/// simulated time, servicing the devices in between.
pub fn run_frame(
    board: &mut Board,
    sim: &mut Simulation,
    budget: u32,
    slice_ns: u64,
) -> ServiceReport {
    let hclk = u64::from(sim.clocks().hclk_hz.max(1));
    let slice_instructions = ((slice_ns.saturating_mul(hclk)) / 1_000_000_000).max(1);
    let slices = ((u64::from(budget) / slice_instructions) as u32).clamp(1, 128);
    let per_slice = (budget / slices).max(1);

    let mut report = ServiceReport::default();
    for _ in 0..slices {
        let partial = service(board, sim);
        report.encoder_edges += partial.encoder_edges;
        report.bytes_sent += partial.bytes_sent;
        report.can_frames += partial.can_frames;
        report.events_delivered += partial.events_delivered;
        sim.advance_frame(per_slice);
    }
    report
}
