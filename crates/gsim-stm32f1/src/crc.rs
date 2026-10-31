//! CRC calculation unit and the debug MCU block.

use super::map::{crc as creg, dbgmcu as dreg, DBGMCU_IDCODE};
use gsim_core::bus::{extract, merge, AccessWidth, BusError};
use gsim_core::ctx::Ctx;

/// Reset value of the CRC data register.
pub const CRC_RESET_VALUE: u32 = 0xFFFF_FFFF;
/// CRC-32 polynomial used by the STM32F1 calculation unit.
pub const CRC_POLYNOMIAL: u32 = 0x04C1_1DB7;

/// CRC calculation unit.
#[derive(Debug, Clone)]
pub struct Crc {
    dr: u32,
}

impl Default for Crc {
    fn default() -> Self {
        Crc::new()
    }
}

impl Crc {
    pub fn new() -> Self {
        Crc {
            dr: CRC_RESET_VALUE,
        }
    }

    pub fn reset(&mut self) {
        *self = Crc::new();
    }

    pub fn value(&self) -> u32 {
        self.dr
    }

    fn accumulate(&mut self, word: u32) {
        let mut crc = self.dr;
        for bit in (0..32).rev() {
            let incoming = (word >> bit) & 1;
            let top = (crc >> 31) & 1;
            crc = (crc << 1) | incoming;
            if top != 0 {
                crc ^= CRC_POLYNOMIAL;
            }
        }
        self.dr = crc;
    }

    pub fn read_reg(
        &mut self,
        _cx: &Ctx,
        offset: u32,
        width: AccessWidth,
    ) -> Result<u32, BusError> {
        let value = match offset & !3 {
            creg::DR => self.dr,
            creg::IDR => self.dr,
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
        _cx: &Ctx,
        offset: u32,
        width: AccessWidth,
        value: u32,
    ) -> Result<(), BusError> {
        match offset & !3 {
            creg::DR => self.accumulate(merge(self.dr, offset, width, value)),
            creg::IDR => {
                self.dr = merge(self.dr, offset, width, value);
            }
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

crate::impl_peripheral!(Crc, "CRC");

/// Debug MCU identification and freeze registers.
#[derive(Debug, Clone, Default)]
pub struct DbgMcu {
    cr: u32,
}

impl DbgMcu {
    pub fn new() -> Self {
        DbgMcu::default()
    }

    pub fn reset(&mut self) {
        *self = DbgMcu::new();
    }

    /// Device identification word (`DEV_ID` | `REV_ID`).
    pub fn idcode(&self) -> u32 {
        DBGMCU_IDCODE
    }

    pub fn read_reg(
        &mut self,
        _cx: &Ctx,
        offset: u32,
        width: AccessWidth,
    ) -> Result<u32, BusError> {
        let value = match offset & !3 {
            dreg::IDCODE => self.idcode(),
            dreg::CR => self.cr,
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
        _cx: &Ctx,
        offset: u32,
        width: AccessWidth,
        value: u32,
    ) -> Result<(), BusError> {
        match offset & !3 {
            dreg::CR => self.cr = merge(self.cr, offset, width, value) & 0x000F_FFFF,
            dreg::IDCODE => {}
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

crate::impl_peripheral!(DbgMcu, "DBGMCU");

impl Crc {
    /// Labelled state for the peripheral status browser.
    pub fn status(&self, _clocks: &gsim_core::Clocks) -> Vec<(&'static str, String)> {
        vec![
            ("数据寄存器", crate::status::hex(self.dr)),
            ("IDR", crate::status::hex(0x00A4_5C12)),
        ]
    }
}

impl DbgMcu {
    /// Labelled state for the peripheral status browser.
    pub fn status(&self, _clocks: &gsim_core::Clocks) -> Vec<(&'static str, String)> {
        vec![
            ("IDCODE", crate::status::hex(0x2003_6410)),
            ("CR", crate::status::hex(self.cr)),
        ]
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
    fn crc_starts_at_all_ones() {
        let cx = ctx();
        let mut crc = Crc::new();
        assert_eq!(
            crc.read_reg(&cx, creg::DR, AccessWidth::Word).unwrap(),
            CRC_RESET_VALUE
        );
    }

    #[test]
    fn known_crc_value_for_a_single_word() {
        let cx = ctx();
        let mut crc = Crc::new();
        crc.write_reg(&cx, creg::IDR, AccessWidth::Word, CRC_RESET_VALUE)
            .unwrap();
        // The reference implementation of the F1 unit: feeding a word with the
        // initial value yields this well known result for 0x00000000.
        crc.write_reg(&cx, creg::DR, AccessWidth::Word, 0).unwrap();
        assert_eq!(
            crc.read_reg(&cx, creg::DR, AccessWidth::Word).unwrap(),
            0xC704_DD7B
        );
    }

    #[test]
    fn idr_sets_the_checksum_directly() {
        let cx = ctx();
        let mut crc = Crc::new();
        crc.write_reg(&cx, creg::IDR, AccessWidth::Word, 0x1234_5678)
            .unwrap();
        assert_eq!(crc.value(), 0x1234_5678);
    }

    #[test]
    fn debug_idcode_matches_the_device() {
        let cx = ctx();
        let mut dbg = DbgMcu::new();
        assert_eq!(
            dbg.read_reg(&cx, dreg::IDCODE, AccessWidth::Word).unwrap(),
            0x2003_6410
        );
    }
}
