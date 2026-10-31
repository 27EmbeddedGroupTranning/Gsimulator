//! Independent and window watchdogs.

use super::map::{iwdg as ireg, wwdg as wreg};
use gsim_core::bus::{extract, field, merge, AccessWidth, BusError};
use gsim_core::ctx::Ctx;
use gsim_core::time::NanoAccumulator;

/// Key written to `IWDG_KR` to start the watchdog.
pub const IWDG_START: u32 = 0xCCCC;
/// Key written to `IWDG_KR` to reload the counter.
pub const IWDG_RELOAD: u32 = 0xAAAA;
/// Key written to `IWDG_KR` to unlock `PR`/`RLR`.
pub const IWDG_ACCESS: u32 = 0x5555;

/// Independent watchdog driven by the 40 kHz LSI.
#[derive(Debug, Clone)]
pub struct Iwdg {
    pr: u32,
    rlr: u32,
    sr: u32,
    running: bool,
    counter: u32,
    acc: NanoAccumulator,
    expired: bool,
}

impl Default for Iwdg {
    fn default() -> Self {
        Iwdg::new()
    }
}

impl Iwdg {
    pub fn new() -> Self {
        Iwdg {
            pr: 0,
            rlr: 0x0FFF,
            sr: 0x3,
            running: false,
            counter: 0x0FFF,
            acc: NanoAccumulator::new(),
            expired: false,
        }
    }

    pub fn reset(&mut self) {
        *self = Iwdg::new();
    }

    pub fn running(&self) -> bool {
        self.running
    }

    /// Timeout in LSI cycles: `4 * 2^PR * RLR`.
    pub fn reload_cycles(&self) -> u32 {
        4 * (1 << (self.pr & 0x7)) * (self.rlr & 0xFFF)
    }

    /// Timeout in nanoseconds.
    pub fn timeout_ns(&self) -> u64 {
        u64::from(self.reload_cycles()) * gsim_core::time::NANOS_PER_SEC
            / u64::from(super::rcc::LSI_HZ)
    }

    /// Consumes the "watchdog fired" condition, which the SoC turns into a reset.
    pub fn take_expired(&mut self) -> bool {
        std::mem::take(&mut self.expired)
    }

    pub fn read_reg(
        &mut self,
        _cx: &Ctx,
        offset: u32,
        width: AccessWidth,
    ) -> Result<u32, BusError> {
        let value = match offset & !3 {
            ireg::KR => 0,
            ireg::PR => self.pr & 0x7,
            ireg::RLR => self.rlr & 0xFFF,
            ireg::SR => self.sr & 0x3,
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
            ireg::KR => match value & 0xFFFF {
                IWDG_START => {
                    if !self.running {
                        self.running = true;
                        self.counter = self.reload_cycles();
                    }
                }
                IWDG_RELOAD => {
                    if self.running {
                        self.counter = self.reload_cycles();
                    }
                }
                IWDG_ACCESS => self.sr = 0,
                _ => {}
            },
            ireg::PR => {
                if self.sr & 0x1 == 0 {
                    self.pr = merge(self.pr, offset, width, value) & 0x7;
                    self.sr |= 0x1;
                }
            }
            ireg::RLR => {
                if self.sr & 0x2 == 0 {
                    self.rlr = merge(self.rlr, offset, width, value) & 0xFFF;
                    self.sr |= 0x2;
                }
            }
            ireg::SR => {}
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

    pub fn advance(&mut self, cx: &Ctx, elapsed_ns: u64) {
        if !self.running {
            self.acc.reset();
            return;
        }
        let cycles = self.acc.cycles(elapsed_ns, super::rcc::LSI_HZ);
        if cycles == 0 {
            return;
        }
        if cycles >= u64::from(self.counter) {
            self.expired = true;
            self.counter = self.reload_cycles();
            cx.note("IWDG", "watchdog expired", 0);
        } else {
            self.counter -= cycles as u32;
        }
    }
}

crate::impl_peripheral!(Iwdg, "IWDG");

/// Window watchdog clocked from `PCLK1 / 4096 / 2^WDGTB`.
#[derive(Debug, Clone)]
pub struct Wwdg {
    cr: u32,
    cfr: u32,
    sr: u32,
    counter: u32,
    acc: NanoAccumulator,
    expired: bool,
}

impl Default for Wwdg {
    fn default() -> Self {
        Wwdg::new()
    }
}

impl Wwdg {
    pub fn new() -> Self {
        Wwdg {
            cr: 0x7F,
            cfr: 0x7F,
            sr: 0,
            counter: 0x7F,
            acc: NanoAccumulator::new(),
            expired: false,
        }
    }

    pub fn reset(&mut self) {
        *self = Wwdg::new();
    }

    pub fn running(&self) -> bool {
        self.cr & (1 << 7) != 0
    }

    pub fn take_expired(&mut self) -> bool {
        std::mem::take(&mut self.expired)
    }

    pub fn read_reg(
        &mut self,
        _cx: &Ctx,
        offset: u32,
        width: AccessWidth,
    ) -> Result<u32, BusError> {
        let value = match offset & !3 {
            wreg::CR => self.cr,
            wreg::CFR => self.cfr,
            wreg::SR => self.sr & 0x1,
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
            wreg::CR => {
                let merged = merge(self.cr, offset, width, value);
                self.counter = 0x7F;
                self.cr = (self.cr & (1 << 7)) | (merged & 0x7F) | (merged & (1 << 7));
            }
            wreg::CFR => {
                if !self.running() {
                    self.cfr = merge(self.cfr, offset, width, value) & 0x02FF;
                }
            }
            wreg::SR => self.sr &= !(value & 1),
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

    pub fn advance(&mut self, cx: &Ctx, elapsed_ns: u64) {
        if !self.running() {
            self.acc.reset();
            return;
        }
        let pclk1 = cx.clocks().pclk1_hz;
        if pclk1 == 0 {
            return;
        }
        let prescaler = 4096u32 << field(self.cfr, 7, 2);
        let clock = (pclk1 / prescaler).max(1);
        let cycles = self.acc.cycles(elapsed_ns, clock);
        if cycles == 0 {
            return;
        }
        let counter = u64::from(self.counter);
        if cycles >= counter {
            self.expired = true;
            self.counter = 0x7F;
            cx.note("WWDG", "watchdog expired", 0);
            return;
        }
        self.counter -= cycles as u32;
        if self.counter < 0x40 && self.cfr & (1 << 9) != 0 {
            self.sr |= 1;
        }
    }
}

crate::impl_peripheral!(Wwdg, "WWDG");

impl Iwdg {
    /// Labelled state for the peripheral status browser.
    pub fn status(&self, _clocks: &gsim_core::Clocks) -> Vec<(&'static str, String)> {
        vec![
            ("运行", crate::status::on_off(self.running).to_string()),
            ("计数器", format!("{}", self.counter)),
            ("重装值", format!("{}", self.rlr & 0xFFF)),
            ("超时周期", format!("{} 个 LSI 周期", self.reload_cycles())),
        ]
    }
}

impl Wwdg {
    /// Labelled state for the peripheral status browser.
    pub fn status(&self, _clocks: &gsim_core::Clocks) -> Vec<(&'static str, String)> {
        vec![
            (
                "使能",
                crate::status::on_off(self.cr & (1 << 7) != 0).to_string(),
            ),
            ("计数器", format!("{}", self.counter)),
            ("CR", crate::status::hex(self.cr)),
            ("CFR", crate::status::hex(self.cfr)),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use gsim_core::ctx::Clocks;

    fn ctx() -> Ctx {
        Ctx::new(0, Clocks::HSI_8MHZ)
    }

    #[test]
    fn iwdg_counts_down_and_expires() {
        let cx = ctx();
        let mut iwdg = Iwdg::new();
        iwdg.write_reg(&cx, ireg::KR, AccessWidth::Word, IWDG_ACCESS)
            .unwrap();
        iwdg.write_reg(&cx, ireg::PR, AccessWidth::Word, 0).unwrap();
        iwdg.write_reg(&cx, ireg::RLR, AccessWidth::Word, 10)
            .unwrap();
        iwdg.write_reg(&cx, ireg::KR, AccessWidth::Word, IWDG_START)
            .unwrap();
        assert!(iwdg.running());
        assert_eq!(iwdg.reload_cycles(), 40);
        // 40 LSI cycles at 40 kHz = 1 ms.
        iwdg.advance(&cx, 1_100_000);
        assert!(iwdg.take_expired());
        assert!(!iwdg.take_expired());
    }

    #[test]
    fn reloading_prevents_expiry() {
        let cx = ctx();
        let mut iwdg = Iwdg::new();
        iwdg.write_reg(&cx, ireg::KR, AccessWidth::Word, IWDG_START)
            .unwrap();
        iwdg.advance(&cx, 400_000);
        iwdg.write_reg(&cx, ireg::KR, AccessWidth::Word, IWDG_RELOAD)
            .unwrap();
        iwdg.advance(&cx, 400_000);
        assert!(!iwdg.take_expired());
    }

    #[test]
    fn wwdg_expires_without_refresh() {
        let cx = ctx();
        let mut wwdg = Wwdg::new();
        wwdg.write_reg(&cx, wreg::CR, AccessWidth::Word, 0x7F)
            .unwrap();
        wwdg.write_reg(&cx, wreg::CR, AccessWidth::Word, 0xFF)
            .unwrap();
        assert!(wwdg.running());
        // 8 MHz / 4096 = 1953 Hz, one tick is ~512 us.
        wwdg.advance(&cx, 70_000_000);
        assert!(wwdg.take_expired());
    }

    #[test]
    fn refreshing_the_window_watchdog_keeps_it_alive() {
        let cx = ctx();
        let mut wwdg = Wwdg::new();
        wwdg.write_reg(&cx, wreg::CR, AccessWidth::Word, 0xFF)
            .unwrap();
        for _ in 0..10 {
            wwdg.advance(&cx, 10_000_000);
            wwdg.write_reg(&cx, wreg::CR, AccessWidth::Word, 0xFF)
                .unwrap();
        }
        assert!(!wwdg.take_expired());
    }
}
