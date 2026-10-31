//! Off-chip device models for the Gsimulator board.
//!
//! The crate holds the concrete external peripherals the simulated STM32F103
//! talks to. They live outside `gsim-stm32f1` on purpose: like real parts they
//! only see the pins and the frames the controller moves, never its registers.
//!
//! * [`max7219`] – 8x8 LED matrix controller driven through an SPI port and a
//!   chip-select line (the CubeMX firmware in `firmware/CAN` drives one on
//!   SPI1 with `PA4` as LOAD/CS).
//! * [`uart`] – a serial terminal: it shows what the controller transmits and
//!   types bytes back into the receiver.
//! * [`encoder`] – a rotary encoder with quadrature `A`/`B` contacts and a
//!   push button, scheduled edge by edge so polling firmware sees every step.
//! * [`can_bus`] – a virtual CAN bus plus the CAN transceiver that connects one
//!   controller to it (TXD on `PA12`, RXD on `PA11` on the F103).
//!
//! They are ordinary state machines: feed them the [`gsim_core::Event`]s the
//! SoC produced since the last step, then read their state for the UI or for
//! the next register access.

pub mod can_bus;
pub mod encoder;
pub mod max7219;
pub mod uart;

pub use can_bus::{
    BusRecord, CanBus, CanBusStats, CanTransceiver, NodeId, TransceiverMode, TransceiverStats,
};
pub use encoder::{Direction, Encoder, Phase as EncoderPhase, EDGES_PER_DETENT};
pub use max7219::{ChipState, Max7219, MATRIX_SIZE};
pub use uart::Uart;
