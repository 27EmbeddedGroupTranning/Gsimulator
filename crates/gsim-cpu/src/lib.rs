//! ARM Cortex-M3 execution front end for Gsimulator.
//!
//! This crate turns the register level device model of `gsim-stm32f1` into a
//! runnable system: it loads real ELF firmware images, executes them on a
//! Cortex-M3 core and connects every memory access back to the peripheral
//! models.
//!
//! The instruction set comes from [`armagnac`], a pure Rust Thumb / ARMv7-M
//! emulator (MIT OR Apache-2.0), and the ELF parsing from [`object`], the
//! `gimli` project's object file reader.

pub mod cpu;
pub mod elf;
pub mod interface;

pub use cpu::{Cpu, CpuError, StopReason, EXTERNAL_IRQ_COUNT};
pub use elf::{ElfError, ElfImage, Segment, Symbol, SymbolKind};
pub use interface::SocInterface;

/// Re-exported so callers can inspect emulation failures in detail.
pub use armagnac::core::RunError;
