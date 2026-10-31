//! Wiring harness between the simulated MCU and the parts on the schematic.
//!
//! Every slice of simulated time the front end first asks the bridge to service
//! the devices, then advances the simulation. Servicing means:
//!
//! 1. hand the peripheral events produced by the SoC since the last slice to the
//!    device they are wired to (a UART terminal only hears its own port when its
//!    `RX` contact is actually wired to that USART's `TX` pin),
//! 2. release the quadrature edges an encoder has due and drive the MCU pins it
//!    is wired to,
//! 3. inject the bytes queued in a terminal into the MCU receiver.
//!
//! Keeping this out of the devices makes them testable on their own, and keeping
//! it out of `App` makes the timing loop easy to reason about.

use crate::board::{
    Board, DeviceId, DeviceKind, Hardware, PinRef, ENCODER_A, ENCODER_B, ENCODER_SW, UART_RX,
    UART_TX,
};
use crate::sim::Simulation;

/// `TX` pin of USART1/2/3 on the F103 (default mapping, no remap).
pub const USART_TX_PINS: [(u8, u8); 3] = [(0, 9), (0, 2), (1, 10)];
/// `RX` pin of USART1/2/3.
pub const USART_RX_PINS: [(u8, u8); 3] = [(0, 10), (0, 3), (1, 11)];

/// Bytes one terminal may push into the MCU per slice.
const MAX_BYTES_PER_SLICE: usize = 1;

/// What the bridge did, mostly for tests and the status line.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ServiceReport {
    /// Quadrature edges applied this slice.
    pub encoder_edges: u32,
    /// Bytes injected into a controller receiver.
    pub bytes_sent: usize,
    /// Events handed to a device.
    pub events_delivered: usize,
}

/// Read-only view of what a device is wired to.
struct Wiring {
    id: DeviceId,
    /// USART the terminal listens to and may answer on.
    uart_listening: bool,
    uart_answering: bool,
    unit: u8,
    encoder_pins: [(u8, u8); 2],
    button_pin: Option<(u8, u8)>,
}

impl Wiring {
    fn collect(board: &Board) -> Vec<Wiring> {
        board
            .devices()
            .iter()
            .filter_map(|device| match (&device.kind, &device.hardware) {
                (DeviceKind::Uart, Hardware::Uart(uart)) => {
                    let unit = uart.unit().min(2);
                    let (tx_port, tx_pin) = USART_TX_PINS[usize::from(unit)];
                    let (rx_port, rx_pin) = USART_RX_PINS[usize::from(unit)];
                    Some(Wiring {
                        id: device.id,
                        uart_listening: board.is_connected(
                            PinRef::Mcu {
                                port: tx_port,
                                pin: tx_pin,
                            },
                            PinRef::Device {
                                device: device.id,
                                pin: UART_RX,
                            },
                        ),
                        uart_answering: board.is_connected(
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
                        encoder_pins: [(0, 0); 2],
                        button_pin: None,
                    })
                }
                (DeviceKind::Encoder, Hardware::Encoder(_)) => {
                    let a = board
                        .mcu_pin_for(PinRef::Device {
                            device: device.id,
                            pin: ENCODER_A,
                        })
                        .unwrap_or((0xFF, 0xFF));
                    let b = board
                        .mcu_pin_for(PinRef::Device {
                            device: device.id,
                            pin: ENCODER_B,
                        })
                        .unwrap_or((0xFF, 0xFF));
                    Some(Wiring {
                        id: device.id,
                        uart_listening: false,
                        uart_answering: false,
                        unit: 0,
                        encoder_pins: [a, b],
                        button_pin: board.mcu_pin_for(PinRef::Device {
                            device: device.id,
                            pin: ENCODER_SW,
                        }),
                    })
                }
                _ => None,
            })
            .collect()
    }

    fn drives(&self, pin: (u8, u8)) -> bool {
        pin.0 != 0xFF
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
        match &mut device.hardware {
            Hardware::Uart(uart) => {
                if entry.uart_listening {
                    for event in &events {
                        uart.on_event(event);
                        report.events_delivered += 1;
                    }
                }
                if entry.uart_answering {
                    for _ in 0..MAX_BYTES_PER_SLICE {
                        let Some(byte) = uart.next_outgoing() else {
                            break;
                        };
                        if sim.send_uart(usize::from(entry.unit), byte) {
                            report.bytes_sent += 1;
                        } else {
                            // Receiver busy: keep the byte for the next slice.
                            uart.requeue(byte);
                            break;
                        }
                    }
                }
            }
            Hardware::Encoder(encoder) => {
                report.encoder_edges += encoder.poll(now_ns);
                // The contacts hold their level: drive them even between
                // edges, so the MCU reads the encoder state right away.
                let phase = encoder.phase();
                if entry.drives(entry.encoder_pins[0]) {
                    sim.drive_pin(entry.encoder_pins[0], phase.a);
                }
                if entry.drives(entry.encoder_pins[1]) {
                    sim.drive_pin(entry.encoder_pins[1], phase.b);
                }
                if let Some(pin) = entry.button_pin {
                    // The module pulls the contact up; pressing it closes the
                    // switch to ground, so the pin goes low.
                    sim.drive_pin(pin, !encoder.button());
                }
            }
            Hardware::None => {}
        }
    }

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
        report.events_delivered += partial.events_delivered;
        sim.advance_frame(per_slice);
    }
    report
}
