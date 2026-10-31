//! Embedded flash and SRAM storage plus the flash interface (`FLASH`).

use super::map::{flash as reg, FLASH_BASE, FLASH_SIZE, SRAM_BASE, SRAM_SIZE};
use gsim_core::bus::{extract, field, merge, AccessWidth, BusError};
use gsim_core::ctx::Ctx;
use gsim_core::irq::lines;

/// Key sequence that unlocks the flash controller.
pub const FLASH_KEY1: u32 = 0x4567_0123;
/// Second key of the unlock sequence.
pub const FLASH_KEY2: u32 = 0xCDEF_89AB;

/// Flash/SRAM backing store of the simulated device.
#[derive(Debug, Clone)]
pub struct Memory {
    flash: Vec<u8>,
    sram: Vec<u8>,
}

impl Default for Memory {
    fn default() -> Self {
        Memory::new()
    }
}

impl Memory {
    pub fn new() -> Self {
        Memory {
            flash: vec![0xFF; FLASH_SIZE as usize],
            sram: vec![0x00; SRAM_SIZE as usize],
        }
    }

    /// True when `addr` falls inside the flash alias.
    pub fn is_flash(addr: u32) -> bool {
        (FLASH_BASE..FLASH_BASE + FLASH_SIZE).contains(&addr)
    }

    /// True when `addr` falls inside SRAM.
    pub fn is_sram(addr: u32) -> bool {
        (SRAM_BASE..SRAM_BASE + SRAM_SIZE).contains(&addr)
    }

    pub fn read(&self, addr: u32, width: AccessWidth) -> Result<u32, BusError> {
        let bytes = self.slice(addr, width)?;
        let mut value = 0u32;
        for (index, byte) in bytes.iter().enumerate() {
            value |= u32::from(*byte) << (index * 8);
        }
        Ok(value)
    }

    /// Unrestricted write, used by the loader and by the flash programmer.
    pub fn write(&mut self, addr: u32, width: AccessWidth, value: u32) -> Result<(), BusError> {
        let bytes = self.slice_mut(addr, width)?;
        for (index, byte) in bytes.iter_mut().enumerate() {
            *byte = ((value >> (index * 8)) & 0xFF) as u8;
        }
        Ok(())
    }

    /// Copies a firmware image into flash starting at `addr`.
    pub fn load(&mut self, addr: u32, data: &[u8]) -> Result<(), BusError> {
        for (index, byte) in data.iter().enumerate() {
            let target = addr + index as u32;
            if Self::is_flash(target) {
                self.flash[(target - FLASH_BASE) as usize] = *byte;
            } else if Self::is_sram(target) {
                self.sram[(target - SRAM_BASE) as usize] = *byte;
            } else {
                return Err(BusError::Unmapped {
                    addr: target,
                    width: AccessWidth::Byte,
                    write: true,
                });
            }
        }
        Ok(())
    }

    /// Programs flash the way the on-chip controller does: bits can only be
    /// cleared, and a word is programmed as two half-word writes.
    pub fn program_flash(&mut self, addr: u32, value: u16) {
        if !Self::is_flash(addr) {
            return;
        }
        let index = (addr - FLASH_BASE) as usize;
        let current = u16::from_le_bytes([self.flash[index], self.flash[index + 1]]);
        let programmed = current & value;
        self.flash[index..index + 2].copy_from_slice(&programmed.to_le_bytes());
    }

    /// Erases one flash page (1 KiB on medium density parts).
    pub fn erase_page(&mut self, addr: u32) {
        if !Self::is_flash(addr) {
            return;
        }
        let page = ((addr - FLASH_BASE) / 1024) as usize;
        let start = page * 1024;
        let end = (start + 1024).min(self.flash.len());
        for byte in &mut self.flash[start..end] {
            *byte = 0xFF;
        }
    }

    /// Erases the whole main flash bank.
    pub fn erase_all(&mut self) {
        for byte in &mut self.flash {
            *byte = 0xFF;
        }
    }

    pub fn flash(&self) -> &[u8] {
        &self.flash
    }

    pub fn sram(&self) -> &[u8] {
        &self.sram
    }

    fn range(addr: u32, width: AccessWidth) -> Option<(usize, usize)> {
        let size = width.bytes();
        let end = addr.checked_add(size)?;
        if Self::is_flash(addr) && Self::is_flash(end - 1) {
            return Some(((addr - FLASH_BASE) as usize, size as usize));
        }
        if Self::is_sram(addr) && Self::is_sram(end - 1) {
            return Some(((addr - SRAM_BASE) as usize, size as usize));
        }
        None
    }

    fn slice(&self, addr: u32, width: AccessWidth) -> Result<&[u8], BusError> {
        let Some((offset, size)) = Self::range(addr, width) else {
            return Err(BusError::Unmapped {
                addr,
                width,
                write: false,
            });
        };
        let buffer = if Self::is_flash(addr) {
            &self.flash
        } else {
            &self.sram
        };
        Ok(&buffer[offset..offset + size])
    }

    fn slice_mut(&mut self, addr: u32, width: AccessWidth) -> Result<&mut [u8], BusError> {
        let Some((offset, size)) = Self::range(addr, width) else {
            return Err(BusError::Unmapped {
                addr,
                width,
                write: false,
            });
        };
        let buffer = if Self::is_flash(addr) {
            &mut self.flash
        } else {
            &mut self.sram
        };
        Ok(&mut buffer[offset..offset + size])
    }
}

/// Operation requested through the flash controller.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Erase {
    /// Erase the page that contains the address.
    Page(u32),
    /// Erase the whole main bank.
    Mass,
}

/// Flash access control interface.
#[derive(Debug, Clone)]
pub struct FlashInterface {
    acr: u32,
    sr: u32,
    cr: u32,
    ar: u32,
    obr: u32,
    wrpr: u32,
    key_stage: u8,
    pending_erase: Option<Erase>,
}

impl Default for FlashInterface {
    fn default() -> Self {
        FlashInterface::new()
    }
}

impl FlashInterface {
    pub fn new() -> Self {
        FlashInterface {
            acr: 0,
            sr: 0,
            cr: 1 << 7, // LOCK
            ar: 0,
            obr: 0x03FF_F0FC,
            wrpr: 0xFFFF_FFFF,
            key_stage: 0,
            pending_erase: None,
        }
    }

    pub fn reset(&mut self) {
        *self = FlashInterface::new();
    }

    /// Number of wait states configured through `ACR`.
    pub fn latency(&self) -> u32 {
        field(self.acr, 0, 3)
    }

    /// True once the unlock sequence has been accepted.
    pub fn is_unlocked(&self) -> bool {
        self.cr & (1 << 7) == 0
    }

    /// True while a program operation is armed.
    pub fn programming_enabled(&self) -> bool {
        self.is_unlocked() && self.cr & 1 != 0
    }

    /// Reports a successful program/erase to `SR.EOP`.
    pub fn set_end_of_operation(&mut self) {
        self.sr |= 1 << 5;
    }

    /// Reports a write to a protected or locked flash region.
    pub fn set_write_error(&mut self) {
        self.sr |= 1 << 4;
    }

    /// Takes the erase request latched by a write to `CR.STRT`.
    pub fn take_erase(&mut self) -> Option<Erase> {
        std::mem::take(&mut self.pending_erase)
    }

    pub fn read_reg(
        &mut self,
        _cx: &Ctx,
        offset: u32,
        width: AccessWidth,
    ) -> Result<u32, BusError> {
        let value = match offset & !3 {
            reg::ACR => self.acr,
            reg::KEYR | reg::OPTKEYR => 0,
            reg::SR => self.sr,
            reg::CR => self.cr,
            reg::AR => self.ar,
            reg::OBR => self.obr,
            reg::WRPR => self.wrpr,
            _ => {
                return Err(BusError::Unmapped {
                    addr: offset,
                    width,
                    write: false,
                })
            }
        };
        Ok(extract(value, offset, width))
    }

    pub fn write_reg(
        &mut self,
        cx: &Ctx,
        offset: u32,
        width: AccessWidth,
        value: u32,
    ) -> Result<(), BusError> {
        match offset & !3 {
            reg::ACR => self.acr = merge(self.acr, offset, width, value) & 0x3F,
            reg::KEYR => {
                if value == FLASH_KEY1 {
                    self.key_stage = 1;
                } else if value == FLASH_KEY2 && self.key_stage == 1 {
                    self.cr &= !(1 << 7);
                    self.key_stage = 2;
                } else {
                    self.key_stage = 0;
                }
            }
            reg::OPTKEYR => {}
            reg::SR => self.sr &= !(value & ((1 << 5) | (1 << 4) | (1 << 1))),
            reg::CR => {
                let merged = merge(self.cr, offset, width, value);
                if merged & (1 << 7) != 0 {
                    self.cr |= 1 << 7;
                }
                if self.is_unlocked() {
                    self.cr = (self.cr & !0x00FF) | (merged & 0x00FF & !(1 << 6));
                    if merged & (1 << 6) != 0 {
                        let erase = if merged & (1 << 1) != 0 {
                            Erase::Page(self.ar)
                        } else {
                            // MER (mass erase) is the only other erase mode.
                            Erase::Mass
                        };
                        self.pending_erase = Some(erase);
                        cx.note("FLASH", "erase started", self.ar);
                    }
                }
            }
            reg::AR => self.ar = merge(self.ar, offset, width, value) & 0xFFFF_FFFC,
            reg::OBR | reg::WRPR => {}
            _ => {
                return Err(BusError::Unmapped {
                    addr: offset,
                    width,
                    write: true,
                })
            }
        }
        Ok(())
    }

    pub fn advance(&mut self, _cx: &Ctx, _elapsed_ns: u64) {}
}

crate::impl_peripheral!(FlashInterface, "FLASH");

/// Flash ready interrupt line, used when a program/erase completes.
pub const FLASH_IRQ: gsim_core::Irq = lines::FLASH;

impl FlashInterface {
    /// Raises the flash interrupt when `EOP` or an error flag is armed.
    pub fn refresh_irq(&mut self, cx: &Ctx) {
        let high = self.sr & ((1 << 5) | (1 << 4) | (1 << 1)) != 0;
        cx.irq(FLASH_IRQ, high);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use gsim_core::ctx::Clocks;

    fn ctx() -> Ctx {
        Ctx::new(0, Clocks::default())
    }

    #[test]
    fn sram_round_trips_all_widths() {
        let mut mem = Memory::new();
        mem.write(SRAM_BASE + 0x10, AccessWidth::Word, 0x1234_5678)
            .unwrap();
        assert_eq!(
            mem.read(SRAM_BASE + 0x10, AccessWidth::Word).unwrap(),
            0x1234_5678
        );
        assert_eq!(mem.read(SRAM_BASE + 0x11, AccessWidth::Byte).unwrap(), 0x56);
        mem.write(SRAM_BASE + 0x12, AccessWidth::Half, 0xABCD)
            .unwrap();
        assert_eq!(
            mem.read(SRAM_BASE + 0x10, AccessWidth::Word).unwrap(),
            0xABCD_5678
        );
    }

    #[test]
    fn flash_starts_erased_and_accepts_images() {
        let mut mem = Memory::new();
        assert_eq!(
            mem.read(FLASH_BASE, AccessWidth::Word).unwrap(),
            0xFFFF_FFFF
        );
        mem.load(FLASH_BASE, &[0x00, 0x10, 0x00, 0x20]).unwrap();
        assert_eq!(mem.read(FLASH_BASE, AccessWidth::Half).unwrap(), 0x1000);
    }

    #[test]
    fn programming_can_only_clear_bits() {
        let mut mem = Memory::new();
        mem.load(FLASH_BASE, &[0xF0, 0x0F]).unwrap();
        assert_eq!(mem.read(FLASH_BASE, AccessWidth::Half).unwrap(), 0x0FF0);
        mem.program_flash(FLASH_BASE, 0x00FF);
        assert_eq!(mem.read(FLASH_BASE, AccessWidth::Half).unwrap(), 0x00F0);
    }

    #[test]
    fn page_erase_restores_ff() {
        let mut mem = Memory::new();
        mem.load(FLASH_BASE, &[0x11, 0x22]).unwrap();
        mem.load(FLASH_BASE + 0x800, &[0x33, 0x44]).unwrap();
        // Pages are 1 KiB, so erasing the second page keeps the first intact.
        mem.erase_page(FLASH_BASE + 0x800);
        assert_eq!(mem.read(FLASH_BASE, AccessWidth::Half).unwrap(), 0x2211);
        assert_eq!(
            mem.read(FLASH_BASE + 0x800, AccessWidth::Half).unwrap(),
            0xFFFF
        );
        mem.erase_page(FLASH_BASE);
        assert_eq!(mem.read(FLASH_BASE, AccessWidth::Half).unwrap(), 0xFFFF);
    }

    #[test]
    fn unlock_sequence_and_program_bit() {
        let cx = ctx();
        let mut flash = FlashInterface::new();
        assert!(!flash.is_unlocked());
        flash
            .write_reg(&cx, reg::KEYR, AccessWidth::Word, FLASH_KEY1)
            .unwrap();
        flash
            .write_reg(&cx, reg::KEYR, AccessWidth::Word, FLASH_KEY2)
            .unwrap();
        assert!(flash.is_unlocked());
        flash.write_reg(&cx, reg::CR, AccessWidth::Word, 1).unwrap();
        assert!(flash.programming_enabled());
        flash
            .write_reg(&cx, reg::CR, AccessWidth::Word, 1 << 7)
            .unwrap();
        assert!(!flash.is_unlocked(), "LOCK locks the controller again");
    }

    #[test]
    fn latched_erase_request_is_taken_once() {
        let cx = ctx();
        let mut flash = FlashInterface::new();
        flash
            .write_reg(&cx, reg::KEYR, AccessWidth::Word, FLASH_KEY1)
            .unwrap();
        flash
            .write_reg(&cx, reg::KEYR, AccessWidth::Word, FLASH_KEY2)
            .unwrap();
        flash
            .write_reg(&cx, reg::AR, AccessWidth::Word, FLASH_BASE + 0x400)
            .unwrap();
        flash
            .write_reg(&cx, reg::CR, AccessWidth::Word, (1 << 1) | (1 << 6))
            .unwrap();
        assert_eq!(flash.take_erase(), Some(Erase::Page(FLASH_BASE + 0x400)));
        assert_eq!(flash.take_erase(), None);
    }
}

impl FlashInterface {
    /// Labelled state for the peripheral status browser.
    pub fn status(&self, _clocks: &gsim_core::Clocks) -> Vec<(&'static str, String)> {
        vec![
            (
                "等待周期",
                format!("{}", self.acr & 0b111),
            ),
            (
                "编程解锁",
                if self.key_stage >= 2 { "是" } else { "否" }.to_string(),
            ),
            ("状态 SR", crate::status::hex(self.sr)),
            ("控制 CR", crate::status::hex(self.cr)),
            ("目标地址 AR", crate::status::hex(self.ar)),
        ]
    }
}