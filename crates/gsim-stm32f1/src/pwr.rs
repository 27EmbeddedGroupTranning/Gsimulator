//! Power control (`PWR`) and backup registers (`BKP`).

use super::map::{bkp as breg, pwr as reg};
use gsim_core::bus::{extract, merge, AccessWidth, BusError};
use gsim_core::ctx::Ctx;

/// `CR` bits.
pub mod cr {
    pub const LPDS: u32 = 1 << 0;
    pub const PDDS: u32 = 1 << 1;
    pub const CWUF: u32 = 1 << 2;
    pub const CSBF: u32 = 1 << 3;
    pub const PVDE: u32 = 1 << 4;
    pub const PLS: u32 = 0b111 << 5;
    pub const DBP: u32 = 1 << 8;
}

/// `CSR` bits.
pub mod csr {
    pub const WUF: u32 = 1 << 0;
    pub const SBF: u32 = 1 << 1;
    pub const EWUP: u32 = 1 << 8;
    pub const BRE: u32 = 1 << 9;
    pub const PVDO: u32 = 1 << 2;
}

/// Low power mode requested through `CR`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LowPowerMode {
    Run,
    Sleep,
    Stop,
    Standby,
}

/// Power controller.
#[derive(Debug, Clone, Default)]
pub struct Pwr {
    cr: u32,
    csr: u32,
}

impl Pwr {
    pub fn new() -> Self {
        Pwr::default()
    }

    pub fn reset(&mut self) {
        *self = Pwr::new();
    }

    /// True when the backup domain (RTC + BKP) accepts writes.
    pub fn backup_access_enabled(&self) -> bool {
        self.cr & cr::DBP != 0
    }

    /// Mode selected by `LPDS`/`PDDS`.
    pub fn requested_mode(&self) -> LowPowerMode {
        if self.cr & cr::PDDS != 0 {
            LowPowerMode::Standby
        } else if self.cr & cr::LPDS != 0 {
            LowPowerMode::Stop
        } else {
            LowPowerMode::Run
        }
    }

    /// True when the wake-up flag is set.
    pub fn wakeup_pending(&self) -> bool {
        self.csr & csr::WUF != 0
    }

    /// Simulates the wake-up pin, setting `WUF`.
    pub fn signal_wakeup(&mut self) {
        self.csr |= csr::WUF;
    }

    pub fn read_reg(
        &mut self,
        _cx: &Ctx,
        offset: u32,
        width: AccessWidth,
    ) -> Result<u32, BusError> {
        let value = match offset & !3 {
            reg::CR => self.cr,
            reg::CSR => self.csr,
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
            reg::CR => {
                let merged = merge(self.cr, offset, width, value);
                if merged & cr::CWUF != 0 {
                    self.csr &= !csr::WUF;
                }
                if merged & cr::CSBF != 0 {
                    self.csr &= !csr::SBF;
                }
                self.cr = merged & !(cr::CWUF | cr::CSBF) & 0x0001_01FF;
            }
            reg::CSR => {
                let merged = merge(self.csr, offset, width, value);
                self.csr = (self.csr & !0x0303) | (merged & 0x0303);
                if value & csr::WUF != 0 {
                    self.csr &= !csr::WUF;
                }
                if value & csr::SBF != 0 {
                    self.csr &= !csr::SBF;
                }
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

crate::impl_peripheral!(Pwr, "PWR");

/// Backup domain registers.
#[derive(Debug, Clone)]
pub struct Bkp {
    rtccr: u32,
    cr: u32,
    csr: u32,
    data: [u16; 10],
    write_enabled: bool,
}

impl Default for Bkp {
    fn default() -> Self {
        Bkp::new()
    }
}

impl Bkp {
    pub fn new() -> Self {
        Bkp {
            rtccr: 0,
            cr: 0,
            csr: 0,
            data: [0; 10],
            write_enabled: false,
        }
    }

    pub fn reset(&mut self) {
        *self = Bkp::new();
    }

    /// Mirrors `PWR_CR.DBP`, the SoC calls this before dispatching a write.
    pub fn set_write_enabled(&mut self, enabled: bool) {
        self.write_enabled = enabled;
    }

    /// Value of one of the ten backup data registers.
    pub fn data_register(&self, index: usize) -> u16 {
        self.data.get(index).copied().unwrap_or(0)
    }

    pub fn read_reg(
        &mut self,
        _cx: &Ctx,
        offset: u32,
        width: AccessWidth,
    ) -> Result<u32, BusError> {
        let value = match offset & !3 {
            breg::RTCCR => self.rtccr,
            breg::CR => self.cr,
            breg::CSR => self.csr,
            breg::DR1..=breg::DR_LAST => {
                let index = ((offset & !3) - breg::DR1) as usize / 4;
                u32::from(self.data[index])
            }
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
            breg::RTCCR => {
                if self.write_enabled {
                    self.rtccr = merge(self.rtccr, offset, width, value) & 0x03FF;
                }
            }
            breg::CR => {
                if self.write_enabled {
                    self.cr = merge(self.cr, offset, width, value) & 0x0003;
                }
            }
            breg::CSR => {
                let merged = merge(self.csr, offset, width, value);
                self.csr = (self.csr & !0x0303) | (merged & 0x0303);
            }
            breg::DR1..=breg::DR_LAST => {
                if self.write_enabled {
                    let index = ((offset & !3) - breg::DR1) as usize / 4;
                    self.data[index] =
                        merge(u32::from(self.data[index]), offset, width, value) as u16;
                }
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

crate::impl_peripheral!(Bkp, "BKP");

#[cfg(test)]
mod tests {
    use super::*;
    use gsim_core::ctx::Clocks;

    fn ctx() -> Ctx {
        Ctx::new(0, Clocks::default())
    }

    #[test]
    fn backup_writes_need_the_dbp_bit() {
        let cx = ctx();
        let mut pwr = Pwr::new();
        let mut bkp = Bkp::new();
        bkp.set_write_enabled(pwr.backup_access_enabled());
        bkp.write_reg(&cx, breg::DR1, AccessWidth::Word, 0x1234)
            .unwrap();
        assert_eq!(bkp.data_register(0), 0, "domain still protected");

        pwr.write_reg(&cx, reg::CR, AccessWidth::Word, cr::DBP)
            .unwrap();
        assert!(pwr.backup_access_enabled());
        bkp.set_write_enabled(pwr.backup_access_enabled());
        bkp.write_reg(&cx, breg::DR1, AccessWidth::Word, 0x1234)
            .unwrap();
        assert_eq!(bkp.data_register(0), 0x1234);
    }

    #[test]
    fn wakeup_flag_clears_on_cwuf() {
        let cx = ctx();
        let mut pwr = Pwr::new();
        pwr.signal_wakeup();
        assert!(pwr.wakeup_pending());
        pwr.write_reg(&cx, reg::CR, AccessWidth::Word, cr::CWUF)
            .unwrap();
        assert!(!pwr.wakeup_pending());
    }

    #[test]
    fn low_power_mode_decoding() {
        let cx = ctx();
        let mut pwr = Pwr::new();
        assert_eq!(pwr.requested_mode(), LowPowerMode::Run);
        pwr.write_reg(&cx, reg::CR, AccessWidth::Word, cr::LPDS)
            .unwrap();
        assert_eq!(pwr.requested_mode(), LowPowerMode::Stop);
        pwr.write_reg(&cx, reg::CR, AccessWidth::Word, cr::PDDS)
            .unwrap();
        assert_eq!(pwr.requested_mode(), LowPowerMode::Standby);
    }
}

impl Pwr {
    /// Labelled state for the peripheral status browser.
    pub fn status(&self, _clocks: &gsim_core::Clocks) -> Vec<(&'static str, String)> {
        vec![
            (
                "低功耗模式",
                match self.requested_mode() {
                    LowPowerMode::Run => "运行",
                    LowPowerMode::Sleep => "睡眠",
                    LowPowerMode::Stop => "停止",
                    LowPowerMode::Standby => "待机",
                }
                .to_string(),
            ),
            ("备份域访问", crate::status::on_off(self.backup_access_enabled()).to_string()),
            ("CR", crate::status::hex(self.cr)),
            ("CSR", crate::status::hex(self.csr)),
        ]
    }
}

impl Bkp {
    /// Labelled state for the peripheral status browser.
    pub fn status(&self, _clocks: &gsim_core::Clocks) -> Vec<(&'static str, String)> {
        vec![
            ("写使能", crate::status::on_off(self.write_enabled).to_string()),
            ("RTCCR", crate::status::hex(self.rtccr)),
            ("备份数据 DR1", format!("0x{:04X}", self.data[0])),
            ("备份数据 DR10", format!("0x{:04X}", self.data[9])),
        ]
    }
}