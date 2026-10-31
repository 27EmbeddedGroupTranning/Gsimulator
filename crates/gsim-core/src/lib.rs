//! Core infrastructure shared by the Gsimulator device models.
//!
//! The crate provides everything that is independent of a concrete MCU:
//!
//! * [`bus`] – width aware register access and [`bus::BusError`]
//! * [`can`] – classic CAN frames shared by controllers and device models
//! * [`ctx`] – the per-operation context, clock snapshot and [`ctx::Event`] bus
//! * [`peripheral`] – the [`peripheral::Peripheral`] trait implemented by blocks
//! * [`irq`] – interrupt line numbering
//! * [`nvic`], [`scb`], [`systick`] – Cortex-M3 core peripherals
//! * [`time`] – drift-free nanosecond to clock cycle conversion
//!
//! Device models live in `gsim-stm32f1` and only use these abstractions, so a
//! new MCU family or a new block can be added without touching the core.

pub mod bus;
pub mod can;
pub mod ctx;
pub mod irq;
pub mod nvic;
pub mod peripheral;
pub mod scb;
pub mod systick;
pub mod time;

pub use bus::{AccessWidth, BusError, MemoryBus};
pub use can::{CanDirection, CanFrame};
pub use ctx::{Clocks, Ctx, Event, EventSink};
pub use irq::Irq;
pub use nvic::Nvic;
pub use peripheral::Peripheral;
pub use scb::Scb;
pub use systick::SysTick;
