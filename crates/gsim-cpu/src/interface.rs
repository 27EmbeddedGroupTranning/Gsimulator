//! Bridge between the ARM core model and the simulated device bus.
//!
//! Every memory region the core can reach is mapped to a [`SocInterface`],
//! which forwards the access to [`gsim_stm32f1::Soc`]. Flash, SRAM and the
//! peripheral window therefore share one single source of truth: DMA
//! transfers, the debugger view and the CPU all observe the same bytes.

use std::cell::RefCell;
use std::rc::Rc;

use armagnac::memory::{
    Env, MemoryAccessError, MemoryInterface, MemoryReadResult, MemoryWriteResult,
};
use gsim_core::{AccessWidth, BusError, MemoryBus};
use gsim_stm32f1::Soc;

/// A window of the [`Soc`] address space exposed to the core.
pub struct SocInterface {
    soc: Rc<RefCell<Soc>>,
    base: u32,
    size: u32,
}

impl SocInterface {
    /// Creates an interface of `size` bytes starting at `base` of the SoC space.
    pub fn new(soc: Rc<RefCell<Soc>>, base: u32, size: u32) -> Self {
        SocInterface { soc, base, size }
    }

    fn read(&self, offset: u32, width: AccessWidth) -> MemoryReadResult<u32> {
        let address = self.base.wrapping_add(offset);
        self.soc
            .borrow_mut()
            .read(address, width)
            .map_err(map_error)
    }

    fn write(&self, offset: u32, width: AccessWidth, value: u32) -> MemoryWriteResult {
        let address = self.base.wrapping_add(offset);
        self.soc
            .borrow_mut()
            .write(address, width, value)
            .map_err(map_error)
    }
}

fn map_error(error: BusError) -> MemoryAccessError {
    match error {
        BusError::Unmapped { .. } => MemoryAccessError::InvalidAddress,
        BusError::Misaligned { .. } => MemoryAccessError::InvalidAlignment,
        BusError::NotWritable { .. } => MemoryAccessError::ReadOnly,
        BusError::NotReadable { .. } | BusError::Peripheral { .. } => MemoryAccessError::Illegal,
    }
}
impl MemoryInterface for SocInterface {
    fn read_u8(&mut self, address: u32, _env: &mut Env) -> MemoryReadResult<u8> {
        self.read(address, AccessWidth::Byte)
            .map(|value| value as u8)
    }

    fn write_u8(&mut self, address: u32, value: u8, _env: &mut Env) -> MemoryWriteResult {
        self.write(address, AccessWidth::Byte, u32::from(value))
    }

    fn read_u16le(&mut self, address: u32, _env: &mut Env) -> MemoryReadResult<u16> {
        self.read(address, AccessWidth::Half)
            .map(|value| value as u16)
    }

    fn write_u16le(&mut self, address: u32, value: u16, _env: &mut Env) -> MemoryWriteResult {
        self.write(address, AccessWidth::Half, u32::from(value))
    }

    fn read_u32le(&mut self, address: u32, _env: &mut Env) -> MemoryReadResult<u32> {
        self.read(address, AccessWidth::Word)
    }

    fn write_u32le(&mut self, address: u32, value: u32, _env: &mut Env) -> MemoryWriteResult {
        self.write(address, AccessWidth::Word, value)
    }

    fn size(&self) -> u32 {
        self.size
    }
}
