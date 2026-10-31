//! Cortex-M3 system control block (`0xE000_ED00`).
//!
//! Only the parts that a bare metal firmware can observe are modelled:
//! identification, `VTOR`, the priority grouping in `AIRCR`, `ICSR` runtime
//! bits, the system handler priorities and the fault status registers.

use crate::bus::{extract, field, with_field, AccessWidth, BusError};
use crate::irq::exception;
use crate::nvic::Nvic;

/// `CPUID` for the Cortex-M3 revision found on the STM32F103 (r1p1).
pub const CPUID_CORTEX_M3_R1P1: u32 = 0x411F_C231;

const VECTKEY: u32 = 0x05FA;
const VECTKEY_READ: u32 = 0xFA05;

mod reg {
    pub const CPUID: u32 = 0x00;
    pub const ICSR: u32 = 0x04;
    pub const VTOR: u32 = 0x08;
    pub const AIRCR: u32 = 0x0C;
    pub const SCR: u32 = 0x10;
    pub const CCR: u32 = 0x14;
    pub const SHPR1: u32 = 0x18;
    pub const SHPR2: u32 = 0x1C;
    pub const SHPR3: u32 = 0x20;
    pub const SHCSR: u32 = 0x24;
    pub const CFSR: u32 = 0x28;
    pub const HFSR: u32 = 0x2C;
    pub const DFSR: u32 = 0x30;
    pub const MMFAR: u32 = 0x34;
    pub const BFAR: u32 = 0x38;
    pub const AFSR: u32 = 0x3C;
}

/// System control block state.
#[derive(Debug, Clone)]
pub struct Scb {
    pub vtor: u32,
    aircr: u32,
    pub scr: u32,
    pub ccr: u32,
    pub shcsr: u32,
    pub cfsr: u32,
    pub hfsr: u32,
    pub dfsr: u32,
    pub mmfar: u32,
    pub bfar: u32,
    pub afsr: u32,
    pub actlr: u32,
    reset_requested: bool,
}

impl Default for Scb {
    fn default() -> Self {
        Scb::new()
    }
}

impl Scb {
    pub fn new() -> Self {
        Scb {
            vtor: 0,
            aircr: 0,
            scr: 0,
            // STKALIGN is set out of reset on Cortex-M3 r2 and later; the F103
            // core (r1p1) reports it as clear.
            ccr: 0,
            shcsr: 0,
            cfsr: 0,
            hfsr: 0,
            dfsr: 0,
            mmfar: 0,
            bfar: 0,
            afsr: 0,
            actlr: 0,
            reset_requested: false,
        }
    }

    pub fn reset(&mut self) {
        *self = Scb::new();
    }

    /// Priority grouping extracted from `AIRCR.PRIGROUP`.
    pub fn prigroup(&self) -> u8 {
        field(self.aircr, 8, 3) as u8
    }

    /// Set when firmware wrote `AIRCR.SYSRESETREQ`. Consumed by the SoC.
    pub fn take_reset_request(&mut self) -> bool {
        std::mem::take(&mut self.reset_requested)
    }

    /// Value of `CPUID`.
    pub fn cpuid(&self) -> u32 {
        CPUID_CORTEX_M3_R1P1
    }

    /// Reads a register; `offset` is relative to `0xE000_ED00`.
    pub fn read_reg(&self, offset: u32, width: AccessWidth, nvic: &Nvic) -> Result<u32, BusError> {
        let aligned = offset & !3;
        let value = match aligned {
            reg::CPUID => self.cpuid(),
            reg::ICSR => self.icsr(nvic),
            reg::VTOR => self.vtor,
            reg::AIRCR => (VECTKEY_READ << 16) | (self.aircr & 0x0000_FFFF),
            reg::SCR => self.scr,
            reg::CCR => self.ccr,
            reg::SHPR1 => Self::pack_priorities(nvic, 4),
            reg::SHPR2 => Self::pack_priorities(nvic, 8),
            reg::SHPR3 => Self::pack_priorities(nvic, 12),
            reg::SHCSR => self.shcsr_read(nvic),
            reg::CFSR => self.cfsr,
            reg::HFSR => self.hfsr,
            reg::DFSR => self.dfsr,
            reg::MMFAR => self.mmfar,
            reg::BFAR => self.bfar,
            reg::AFSR => self.afsr,
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

    /// Writes a register; `offset` is relative to `0xE000_ED00`.
    pub fn write_reg(
        &mut self,
        offset: u32,
        width: AccessWidth,
        value: u32,
        nvic: &mut Nvic,
    ) -> Result<(), BusError> {
        let aligned = offset & !3;
        let lane_shift = (offset & 3) * 8;
        let lane_mask = width.mask() << lane_shift;
        let incoming = (value & width.mask()) << lane_shift;
        match aligned {
            reg::ICSR => {
                if incoming & lane_mask & (1 << 31) != 0 {
                    nvic.set_system_pending(exception::NMI, true);
                }
                if incoming & (1 << 28) != 0 {
                    nvic.set_system_pending(exception::PEND_SV, true);
                }
                if incoming & (1 << 27) != 0 {
                    nvic.set_system_pending(exception::PEND_SV, false);
                }
                if incoming & (1 << 26) != 0 {
                    nvic.set_system_pending(exception::SYSTICK, true);
                }
                if incoming & (1 << 25) != 0 {
                    nvic.set_system_pending(exception::SYSTICK, false);
                }
            }
            reg::VTOR => {
                self.vtor = (self.vtor & !lane_mask) | (incoming & lane_mask & 0xFFFF_FF80);
            }
            reg::AIRCR => {
                if field(value, 16, 16) == VECTKEY {
                    let prigroup = field(value, 8, 3);
                    self.aircr = with_field(self.aircr, 8, 3, prigroup);
                    if value & (1 << 2) != 0 {
                        self.reset_requested = true;
                    }
                    if value & (1 << 1) != 0 {
                        // VECTCLRACTIVE: drop all active state.
                        while nvic.current_exception().is_some() {
                            let exc = nvic.current_exception().unwrap();
                            nvic.exit(exc);
                        }
                    }
                }
            }
            reg::SCR => self.scr = (self.scr & !lane_mask) | (incoming & lane_mask & 0x16),
            reg::CCR => self.ccr = (self.ccr & !lane_mask) | (incoming & lane_mask & 0x30B),
            reg::SHPR1 => Self::unpack_priorities(nvic, 4, offset, width, value),
            reg::SHPR2 => Self::unpack_priorities(nvic, 8, offset, width, value),
            reg::SHPR3 => Self::unpack_priorities(nvic, 12, offset, width, value),
            reg::SHCSR => {
                self.shcsr = (self.shcsr & !lane_mask) | (incoming & lane_mask & 0x0007_0000)
            }
            reg::CFSR => self.cfsr &= !incoming,
            reg::HFSR => self.hfsr &= !incoming,
            reg::DFSR => self.dfsr &= !incoming,
            reg::MMFAR => self.mmfar = (self.mmfar & !lane_mask) | incoming,
            reg::BFAR => self.bfar = (self.bfar & !lane_mask) | incoming,
            reg::AFSR => self.afsr &= !incoming,
            reg::CPUID => {}
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

    fn icsr(&self, nvic: &Nvic) -> u32 {
        let active = nvic.current_exception().unwrap_or(0);
        let pending = nvic.pending_exception(self.prigroup()).unwrap_or(0);
        let mut value = u32::from(active & 0x1FF) | (u32::from(pending & 0x1FF) << 12);
        if nvic.current_exception().is_none() {
            value |= 1 << 11; // RETTOBASE
        }
        if nvic.pending_count() > 0 {
            value |= 1 << 22; // ISRPENDING
        }
        if nvic.is_system_pending(exception::SYSTICK) {
            value |= 1 << 26;
        }
        if nvic.is_system_pending(exception::PEND_SV) {
            value |= 1 << 28;
        }
        if nvic.is_system_pending(exception::NMI) {
            value |= 1 << 31;
        }
        value
    }

    fn shcsr_read(&self, nvic: &Nvic) -> u32 {
        let mut value = self.shcsr & 0x0007_0000;
        if nvic.is_system_active(exception::SYSTICK) {
            value |= 1 << 11;
        }
        if nvic.is_system_active(exception::PEND_SV) {
            value |= 1 << 10;
        }
        if nvic.is_system_pending(exception::SVCALL) {
            value |= 1 << 15;
        }
        value
    }

    fn pack_priorities(nvic: &Nvic, first_exception: u16) -> u32 {
        let mut value = 0u32;
        for lane in 0..4 {
            let exc = first_exception + lane;
            value |= u32::from(nvic.system_priority(exc)) << (lane * 8);
        }
        value
    }

    fn unpack_priorities(
        nvic: &mut Nvic,
        first_exception: u16,
        offset: u32,
        width: AccessWidth,
        value: u32,
    ) {
        let shift = (offset & 3) * 8;
        for lane in 0..width.bytes() {
            let bit_offset = shift + lane * 8;
            let exc = first_exception + (bit_offset / 8) as u16;
            let byte = ((value >> (lane * 8)) & 0xFF) as u8;
            if exc < 16 {
                nvic.set_system_priority(exc, byte);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::irq::lines;

    #[test]
    fn prigroup_requires_the_vectkey() {
        let mut scb = Scb::new();
        let mut nvic = Nvic::new();
        scb.write_reg(0x0C, AccessWidth::Word, 0x0300, &mut nvic)
            .unwrap();
        assert_eq!(scb.prigroup(), 0, "write ignored without VECTKEY");
        scb.write_reg(0x0C, AccessWidth::Word, 0x05FA_0300, &mut nvic)
            .unwrap();
        assert_eq!(scb.prigroup(), 3);
    }

    #[test]
    fn vtor_is_word_aligned() {
        let mut scb = Scb::new();
        let mut nvic = Nvic::new();
        scb.write_reg(0x08, AccessWidth::Word, 0x0800_010F, &mut nvic)
            .unwrap();
        assert_eq!(scb.vtor, 0x0800_0100);
    }

    #[test]
    fn icsr_reports_pending_exception() {
        let scb = Scb::new();
        let mut nvic = Nvic::new();
        nvic.enable(lines::USART1, true);
        nvic.set_pending(lines::USART1, true);
        let icsr = scb.read_reg(0x04, AccessWidth::Word, &nvic).unwrap();
        assert_eq!(field(icsr, 12, 9), u32::from(lines::USART1.exception()));
        assert_ne!(icsr & (1 << 22), 0, "ISRPENDING set");
    }

    #[test]
    fn sysresetreq_is_latched_for_the_soc() {
        let mut scb = Scb::new();
        let mut nvic = Nvic::new();
        assert!(!scb.take_reset_request());
        scb.write_reg(0x0C, AccessWidth::Word, 0x05FA_0004, &mut nvic)
            .unwrap();
        assert!(scb.take_reset_request());
        assert!(!scb.take_reset_request(), "request is consumed once");
    }

    #[test]
    fn pending_set_and_clear_bits_drive_the_nvic() {
        let mut scb = Scb::new();
        let mut nvic = Nvic::new();
        scb.write_reg(0x04, AccessWidth::Word, 1 << 26, &mut nvic)
            .unwrap();
        assert!(nvic.is_system_pending(exception::SYSTICK));
        scb.write_reg(0x04, AccessWidth::Word, 1 << 25, &mut nvic)
            .unwrap();
        assert!(!nvic.is_system_pending(exception::SYSTICK));
    }

    #[test]
    fn shpr_reaches_nvic_system_priorities() {
        let mut scb = Scb::new();
        let mut nvic = Nvic::new();
        // SHPR3 byte 3 is SysTick.
        scb.write_reg(0x23, AccessWidth::Byte, 0xE0, &mut nvic)
            .unwrap();
        assert_eq!(nvic.system_priority(exception::SYSTICK), 0xE0);
    }
}

impl Scb {
    /// Labelled state for the peripheral status browser.
    pub fn status(&self, _clocks: &crate::Clocks) -> Vec<(&'static str, String)> {
        let group = match self.prigroup() {
            0 => "0 位（7 位抢占）",
            1 => "1 位",
            2 => "2 位",
            3 => "3 位",
            4 => "4 位",
            5 => "5 位",
            6 => "6 位",
            _ => "7 位（无抢占）",
        };
        vec![
            ("CPUID", format!("0x{:08X}", self.cpuid())),
            ("优先级分组", group.to_string()),
            ("AIRCR", format!("0x{:08X}", self.aircr)),
            ("SHCSR", format!("0x{:08X}", self.shcsr)),
            ("CFSR", format!("0x{:08X}", self.cfsr)),
            ("HFSR", format!("0x{:08X}", self.hfsr)),
            ("向量表", format!("0x{:08X}", self.vtor)),
        ]
    }
}