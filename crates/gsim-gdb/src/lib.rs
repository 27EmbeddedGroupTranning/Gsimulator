//! Virtual ST-Link: a GDB remote protocol server for Gsimulator.
//!
//! The server makes the simulated STM32F103 look like a chip behind a debug
//! probe. GDB connects over TCP the same way it connects to OpenOCD
//! (`target extended-remote :3333`), can program the flash, place breakpoints,
//! step, halt with control-C and inspect peripheral registers, because every
//! request is served by the real device model in `gsim-stm32f1`.
//!
//! ```text
//! arm-none-eabi-gdb firmware.elf -ex 'target extended-remote :3333'
//! (gdb) load
//! (gdb) monitor reset
//! (gdb) break main
//! (gdb) continue
//! ```
//!
//! The transport is deliberately plain: one thread owns the simulated device
//! and resumes the core in slices, polling the socket in between, so a
//! control-C stops the firmware without any locking.

pub mod connection;
pub mod description;
pub mod rsp;
pub mod server;
pub mod target;

pub use connection::{Connection, Poll, Waiting};
pub use server::{GdbServer, Reply, ServerOptions};
pub use target::{DebugTarget, FlashReport, PointKind, Signal, Stop, Target};
