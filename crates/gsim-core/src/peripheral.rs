//! The peripheral interface shared by every modelled block.
//!
//! `gsim-stm32f1` implements this trait for all of its blocks. The trait is
//! also the attachment point for blocks that are not part of the SoC's static
//! decode table: [`crate::Peripheral`] values can be registered with a
//! half-open address range at run time, which is how a device controller that
//! is not yet wired into the core decode can be added without touching it.

use crate::bus::{AccessWidth, BusError};
use crate::ctx::Ctx;

/// A memory mapped block.
pub trait Peripheral {
    /// Human readable name used by diagnostics and the memory map dump.
    fn name(&self) -> &'static str {
        "peripheral"
    }

    /// Reads a register at `offset` (already relative to the block base).
    fn read(&mut self, cx: &Ctx, offset: u32, width: AccessWidth) -> Result<u32, BusError>;

    /// Writes a register at `offset` (already relative to the block base).
    fn write(
        &mut self,
        cx: &Ctx,
        offset: u32,
        width: AccessWidth,
        value: u32,
    ) -> Result<(), BusError>;

    /// Advances the internal state by `elapsed_ns` nanoseconds of simulation.
    fn advance(&mut self, cx: &Ctx, elapsed_ns: u64);

    /// Returns the block to its power-on state.
    fn reset(&mut self, cx: &Ctx);
}
