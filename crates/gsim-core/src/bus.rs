//! Register level bus access primitives.
//!
//! The simulator exposes every peripheral through the same address space the
//! real device uses, so a firmware image (or, in later phases, an instruction
//! set simulator) drives peripherals exclusively through `read`/`write` calls.

use std::fmt;

/// Width of a single bus access.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum AccessWidth {
    Byte,
    Half,
    Word,
}

impl AccessWidth {
    /// Number of bytes covered by the access.
    pub const fn bytes(self) -> u32 {
        match self {
            AccessWidth::Byte => 1,
            AccessWidth::Half => 2,
            AccessWidth::Word => 4,
        }
    }

    /// Bit mask covering the access.
    pub const fn mask(self) -> u32 {
        match self {
            AccessWidth::Byte => 0x0000_00FF,
            AccessWidth::Half => 0x0000_FFFF,
            AccessWidth::Word => 0xFFFF_FFFF,
        }
    }

    /// Width described by a byte count, if it is a supported size.
    pub const fn from_bytes(bytes: u32) -> Option<Self> {
        match bytes {
            1 => Some(AccessWidth::Byte),
            2 => Some(AccessWidth::Half),
            4 => Some(AccessWidth::Word),
            _ => None,
        }
    }
}

/// Failure modes of a bus access.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum BusError {
    /// No peripheral answers at this address.
    Unmapped {
        addr: u32,
        width: AccessWidth,
        write: bool,
    },
    /// Address is not naturally aligned for the access width.
    Misaligned { addr: u32, width: AccessWidth },
    /// The register exists but cannot be read.
    NotReadable { addr: u32 },
    /// The register exists but cannot be written.
    NotWritable { addr: u32 },
    /// The peripheral refused the access for a model specific reason.
    Peripheral { addr: u32, reason: &'static str },
}

impl fmt::Display for BusError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            BusError::Unmapped { addr, width, write } => write!(
                f,
                "unmapped {} access at 0x{addr:08X} ({} byte)",
                if *write { "write" } else { "read" },
                width.bytes()
            ),
            BusError::Misaligned { addr, width } => write!(
                f,
                "misaligned {} byte access at 0x{addr:08X}",
                width.bytes()
            ),
            BusError::NotReadable { addr } => write!(f, "register at 0x{addr:08X} is write only"),
            BusError::NotWritable { addr } => write!(f, "register at 0x{addr:08X} is read only"),
            BusError::Peripheral { addr, reason } => {
                write!(f, "peripheral error at 0x{addr:08X}: {reason}")
            }
        }
    }
}

impl std::error::Error for BusError {}

/// Addressable device driven by the simulator.
pub trait MemoryBus {
    fn read(&mut self, addr: u32, width: AccessWidth) -> Result<u32, BusError>;
    fn write(&mut self, addr: u32, width: AccessWidth, value: u32) -> Result<(), BusError>;

    fn read_u8(&mut self, addr: u32) -> Result<u8, BusError> {
        self.read(addr, AccessWidth::Byte).map(|v| v as u8)
    }

    fn read_u16(&mut self, addr: u32) -> Result<u16, BusError> {
        self.read(addr, AccessWidth::Half).map(|v| v as u16)
    }

    fn read_u32(&mut self, addr: u32) -> Result<u32, BusError> {
        self.read(addr, AccessWidth::Word)
    }

    fn write_u8(&mut self, addr: u32, value: u8) -> Result<(), BusError> {
        self.write(addr, AccessWidth::Byte, u32::from(value))
    }

    fn write_u16(&mut self, addr: u32, value: u16) -> Result<(), BusError> {
        self.write(addr, AccessWidth::Half, u32::from(value))
    }

    fn write_u32(&mut self, addr: u32, value: u32) -> Result<(), BusError> {
        self.write(addr, AccessWidth::Word, value)
    }
}

/// Extracts `width` bytes starting at `offset` from a 32 bit register.
///
/// Peripheral registers are modelled as `u32` values; narrower accesses read
/// the corresponding lane the way an APB/AHB slave does.
pub fn extract(reg: u32, offset: u32, width: AccessWidth) -> u32 {
    let shift = (offset & 3) * 8;
    (reg >> shift) & width.mask()
}

/// Writes `width` bytes starting at `offset` into a 32 bit register.
pub fn merge(reg: u32, offset: u32, width: AccessWidth, value: u32) -> u32 {
    let shift = (offset & 3) * 8;
    let mask = width.mask() << shift;
    (reg & !mask) | ((value & width.mask()) << shift)
}

/// Checks that `addr` is naturally aligned for `width`.
pub fn check_alignment(addr: u32, width: AccessWidth) -> Result<(), BusError> {
    if addr % width.bytes() == 0 {
        Ok(())
    } else {
        Err(BusError::Misaligned { addr, width })
    }
}

/// Reads a register-field expressed as `(value, offset, width)`.
pub const fn field(value: u32, offset: u32, width: u32) -> u32 {
    (value >> offset) & ((1u32 << width) - 1)
}

/// Replaces a register-field, keeping all other bits untouched.
pub const fn with_field(value: u32, offset: u32, width: u32, field_value: u32) -> u32 {
    let mask = ((1u32 << width) - 1) << offset;
    (value & !mask) | ((field_value << offset) & mask)
}

/// Sets or clears a single register bit.
pub const fn with_bit(value: u32, bit: u32, set: bool) -> u32 {
    if set {
        value | (1u32 << bit)
    } else {
        value & !(1u32 << bit)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn extract_and_merge_lanes() {
        let reg = 0x1122_3344;
        assert_eq!(extract(reg, 0, AccessWidth::Byte), 0x44);
        assert_eq!(extract(reg, 1, AccessWidth::Byte), 0x33);
        assert_eq!(extract(reg, 2, AccessWidth::Half), 0x1122);
        assert_eq!(merge(reg, 1, AccessWidth::Byte, 0xAB), 0x1122_AB44);
        assert_eq!(merge(reg, 2, AccessWidth::Half, 0x99AA), 0x99AA_3344);
    }

    #[test]
    fn field_helpers() {
        assert_eq!(field(0b1011_0000, 4, 3), 0b011);
        assert_eq!(with_field(0, 4, 3, 0b101), 0b101 << 4);
        assert_eq!(with_bit(0, 3, true), 0b1000);
        assert_eq!(with_bit(0xFF, 3, false), 0xF7);
    }

    #[test]
    fn alignment_is_checked() {
        assert!(check_alignment(0x1000, AccessWidth::Word).is_ok());
        assert!(check_alignment(0x1001, AccessWidth::Word).is_err());
        assert!(check_alignment(0x1001, AccessWidth::Byte).is_ok());
    }
}
