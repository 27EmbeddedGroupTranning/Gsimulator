//! Minimal ELF32 loader for Cortex-M firmware.
//!
//! The `object` crate (from the Rust ecosystem) does the heavy lifting; this
//! module only decides what a loaded image looks like for the simulator and
//! copies the segments into the device memory.

use std::path::{Path, PathBuf};

use gsim_stm32f1::{map, Soc};
use object::{Object, ObjectSegment, ObjectSymbol, SymbolKind as ObjSymbolKind};

/// Errors reported while reading or loading an ELF file.
#[derive(Debug)]
pub enum ElfError {
    Io(std::io::Error),
    Parse(object::Error),
    /// No loadable segment was found.
    NoSegments,
    /// The reset vector is outside of the modelled memories.
    BadVectorTable {
        reason: String,
    },
    /// Writing a segment into the device failed.
    Load {
        address: u32,
        error: gsim_core::BusError,
    },
}

impl std::fmt::Display for ElfError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ElfError::Io(error) => write!(f, "cannot read ELF file: {error}"),
            ElfError::Parse(error) => write!(f, "cannot parse ELF file: {error}"),
            ElfError::NoSegments => write!(f, "ELF file has no loadable segment"),
            ElfError::BadVectorTable { reason } => write!(f, "invalid vector table: {reason}"),
            ElfError::Load { address, error } => {
                write!(f, "cannot load segment at 0x{address:08X}: {error}")
            }
        }
    }
}

impl std::error::Error for ElfError {}

impl From<std::io::Error> for ElfError {
    fn from(error: std::io::Error) -> Self {
        ElfError::Io(error)
    }
}

impl From<object::Error> for ElfError {
    fn from(error: object::Error) -> Self {
        ElfError::Parse(error)
    }
}

/// One `PT_LOAD` segment of the image.
#[derive(Debug, Clone)]
pub struct Segment {
    /// Runtime (virtual) address of the segment.
    pub address: u32,
    /// Address the bytes must be programmed at, `p_paddr` in the ELF header.
    ///
    /// Initialised data lives in SRAM at `address` but is stored in flash at
    /// `load_address`; the reset handler copies it across.
    pub load_address: u32,
    pub bytes: Vec<u8>,
}

impl Segment {
    pub fn size(&self) -> u32 {
        self.bytes.len() as u32
    }

    /// True when the segment is programmed into flash.
    pub fn is_flash(&self) -> bool {
        (map::FLASH_BASE..map::FLASH_BASE + map::FLASH_SIZE).contains(&self.load_address)
    }

    /// True when the segment is destined for SRAM.
    pub fn is_sram(&self) -> bool {
        (map::SRAM_BASE..map::SRAM_BASE + map::SRAM_SIZE).contains(&self.address)
    }
}

/// Kind of an image symbol, as far as the simulator cares.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SymbolKind {
    Function,
    Data,
    Other,
}

/// A symbol table entry.
#[derive(Debug, Clone)]
pub struct Symbol {
    pub name: String,
    pub address: u32,
    pub size: u32,
    pub kind: SymbolKind,
}

/// A parsed firmware image ready to be loaded.
#[derive(Debug, Clone)]
pub struct ElfImage {
    path: PathBuf,
    entry: u32,
    segments: Vec<Segment>,
    symbols: Vec<Symbol>,
}

impl ElfImage {
    /// Parses `path` without touching the device.
    pub fn open(path: impl AsRef<Path>) -> Result<Self, ElfError> {
        let path = path.as_ref().to_path_buf();
        let data = std::fs::read(&path)?;
        let file = object::File::parse(&*data)?;

        let paddrs = load_addresses(&file);
        let mut segments = Vec::new();
        for (index, segment) in file.segments().enumerate() {
            let address = segment.address() as u32;
            let bytes = segment.data()?.to_vec();
            if bytes.is_empty() {
                continue;
            }
            let load_address = paddrs
                .get(index)
                .copied()
                .filter(|address| *address != 0)
                .unwrap_or(address);
            segments.push(Segment {
                address,
                load_address,
                bytes,
            });
        }
        if segments.is_empty() {
            return Err(ElfError::NoSegments);
        }
        segments.sort_by_key(|segment| segment.address);

        let mut symbols = Vec::new();
        for symbol in file.symbols() {
            let Ok(name) = symbol.name() else { continue };
            if name.is_empty() {
                continue;
            }
            let kind = match symbol.kind() {
                ObjSymbolKind::Text => SymbolKind::Function,
                ObjSymbolKind::Data => SymbolKind::Data,
                _ => SymbolKind::Other,
            };
            symbols.push(Symbol {
                name: name.to_string(),
                address: symbol.address() as u32,
                size: symbol.size() as u32,
                kind,
            });
        }

        Ok(ElfImage {
            path,
            entry: file.entry() as u32,
            segments,
            symbols,
        })
    }

    pub fn path(&self) -> &Path {
        &self.path
    }

    /// Address of the ELF entry point (`e_entry`).
    pub fn entry(&self) -> u32 {
        self.entry
    }

    pub fn segments(&self) -> &[Segment] {
        &self.segments
    }

    pub fn symbols(&self) -> &[Symbol] {
        &self.symbols
    }

    /// Looks a symbol up by exact name.
    pub fn symbol(&self, name: &str) -> Option<&Symbol> {
        self.symbols.iter().find(|symbol| symbol.name == name)
    }

    /// Finds the function symbol that contains `address`.
    ///
    /// Thumb function symbols carry bit 0 set in `st_value`, so the comparison
    /// ignores that bit; without it the byte just before a function would be
    /// attributed to the previous function.
    pub fn function_at(&self, address: u32) -> Option<&Symbol> {
        let address = address & !1;
        self.symbols
            .iter()
            .filter(|symbol| symbol.kind == SymbolKind::Function)
            .filter(|symbol| {
                let start = symbol.address & !1;
                let size = symbol.size.max(1);
                (start..start.saturating_add(size)).contains(&address)
            })
            .max_by_key(|symbol| symbol.address & !1)
    }

    /// Total number of bytes the image occupies.
    pub fn total_size(&self) -> u32 {
        self.segments.iter().map(Segment::size).sum()
    }

    /// Initial stack pointer and reset vector read from the vector table.
    pub fn vector_table(&self) -> Result<(u32, u32), ElfError> {
        let mut table = [0u8; 8];
        for segment in &self.segments {
            if !segment.is_flash() {
                continue;
            }
            let base = segment.address.max(map::FLASH_BASE);
            let offset = (base - segment.address) as usize;
            if segment.bytes.len() >= offset + table.len() && base == map::FLASH_BASE {
                let length = table.len();
                table.copy_from_slice(&segment.bytes[offset..offset + length]);
                let _ = length;
                break;
            }
        }
        let sp = u32::from_le_bytes([table[0], table[1], table[2], table[3]]);
        let reset = u32::from_le_bytes([table[4], table[5], table[6], table[7]]);
        if reset == 0 || reset == 0xFFFF_FFFF {
            return Err(ElfError::BadVectorTable {
                reason: format!("reset vector is 0x{reset:08X}"),
            });
        }
        Ok((sp, reset))
    }

    /// Copies every segment into the device memory.
    ///
    /// Bytes are programmed at their load address, which is what a debugger
    /// flashing the part does: `.text` and the initialised-data image land in
    /// flash, and the reset handler copies the initialised data into SRAM. The
    /// SRAM image is written as well so that a firmware without a startup copy
    /// loop still finds its variables initialised.
    pub fn load_into(&self, soc: &mut Soc) -> Result<(), ElfError> {
        for segment in &self.segments {
            if segment.is_flash() {
                soc.load_image(segment.load_address, &segment.bytes)
                    .map_err(|error| ElfError::Load {
                        address: segment.load_address,
                        error,
                    })?;
            }
            if segment.is_sram() && segment.load_address != segment.address {
                soc.load_image(segment.address, &segment.bytes)
                    .map_err(|error| ElfError::Load {
                        address: segment.address,
                        error,
                    })?;
            }
        }
        Ok(())
    }
}

/// Load addresses (`p_paddr`) of the `PT_LOAD` segments, in segment order.
fn load_addresses(file: &object::File<'_>) -> Vec<u32> {
    use object::read::elf::ProgramHeader as _;
    match file {
        object::File::Elf32(elf) => elf
            .raw_segments()
            .iter()
            .filter(|header| header.p_type(elf.endian()) == object::elf::PT_LOAD)
            .map(|header| header.p_paddr(elf.endian()))
            .collect(),
        object::File::Elf64(elf) => elf
            .raw_segments()
            .iter()
            .filter(|header| header.p_type(elf.endian()) == object::elf::PT_LOAD)
            .map(|header| header.p_paddr(elf.endian()) as u32)
            .collect(),
        _ => Vec::new(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn missing_file_is_reported() {
        let error = ElfImage::open("does-not-exist.elf").unwrap_err();
        assert!(matches!(error, ElfError::Io(_)));
    }

    #[test]
    fn truncated_file_is_rejected() {
        let path = std::env::temp_dir().join("gsim-bad.elf");
        std::fs::write(&path, [0x7F, b'E', b'L', b'F', 1, 2, 3]).unwrap();
        let error = ElfImage::open(&path).unwrap_err();
        assert!(matches!(error, ElfError::Parse(_)));
        let _ = std::fs::remove_file(path);
    }
}
