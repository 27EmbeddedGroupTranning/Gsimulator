//! Real time clock (`RTC`).
//!
//! The prescaler divides the RTC clock selected in `RCC_BDCR.RTCSEL` down to a
//! 1 Hz tick that increments the 32 bit counter; the alarm register is compared
//! on every tick. Registers can only be written while `CRL.CNF` is set, as on
//! silicon.

use super::map::rtc as reg;
use gsim_core::bus::{extract, field, merge, AccessWidth, BusError};
use gsim_core::ctx::Ctx;
use gsim_core::irq::lines;
use gsim_core::time::NanoAccumulator;

/// `CRL` bits.
pub mod crl {
    pub const SECF: u32 = 1 << 0;
    pub const ALRF: u32 = 1 << 1;
    pub const OWF: u32 = 1 << 2;
    pub const RSF: u32 = 1 << 3;
    pub const CNF: u32 = 1 << 4;
    pub const RTOFF: u32 = 1 << 5;
}

/// `CRH` bits.
pub mod crh {
    pub const SECIE: u32 = 1 << 0;
    pub const ALRIE: u32 = 1 << 1;
    pub const OWIE: u32 = 1 << 2;
}

/// Default prescaler value for a 32.768 kHz crystal (32767).
pub const DEFAULT_PRESCALER: u32 = 0x7FFF;

/// Real time clock state.
#[derive(Debug, Clone)]
pub struct Rtc {
    crh: u32,
    crl: u32,
    prescaler: u32,
    divider: u32,
    counter: u32,
    alarm: u32,
    acc: NanoAccumulator,
    second_irq_high: bool,
    alarm_irq_high: bool,
}

impl Default for Rtc {
    fn default() -> Self {
        Rtc::new()
    }
}

impl Rtc {
    pub fn new() -> Self {
        Rtc {
            crh: 0,
            crl: crl::RTOFF | crl::RSF,
            prescaler: DEFAULT_PRESCALER,
            divider: 0,
            counter: 0,
            alarm: 0xFFFF_FFFF,
            acc: NanoAccumulator::new(),
            second_irq_high: false,
            alarm_irq_high: false,
        }
    }

    pub fn reset(&mut self) {
        *self = Rtc::new();
    }

    /// Current 32 bit second counter.
    pub fn counter(&self) -> u32 {
        self.counter
    }

    pub fn set_counter(&mut self, value: u32) {
        self.counter = value;
    }

    fn configuration_mode(&self) -> bool {
        self.crl & crl::CNF != 0
    }

    fn refresh_irq(&mut self, cx: &Ctx) {
        let second = self.crh & crh::SECIE != 0 && self.crl & crl::SECF != 0;
        if second != self.second_irq_high {
            self.second_irq_high = second;
            cx.irq(lines::RTC, second);
        }
        let alarm = self.crh & crh::ALRIE != 0 && self.crl & crl::ALRF != 0;
        if alarm != self.alarm_irq_high {
            self.alarm_irq_high = alarm;
            cx.irq(lines::RTC_ALARM, alarm);
        }
    }

    pub fn advance(&mut self, cx: &Ctx, elapsed_ns: u64) {
        let clock = cx.clocks().rtc_hz;
        if clock == 0 {
            self.acc.reset();
            return;
        }
        let cycles = self.acc.cycles(elapsed_ns, clock);
        if cycles == 0 {
            return;
        }
        let period = u64::from(self.prescaler) + 1;
        let total = u64::from(self.divider) + cycles;
        self.divider = (total % period) as u32;
        let seconds = total / period;
        if seconds > 0 {
            self.counter = self.counter.wrapping_add(seconds as u32);
            self.crl |= crl::SECF;
            if self.counter == self.alarm {
                self.crl |= crl::ALRF;
            }
        }
        self.refresh_irq(cx);
    }

    pub fn read_reg(&mut self, cx: &Ctx, offset: u32, width: AccessWidth) -> Result<u32, BusError> {
        let aligned = offset & !3;
        let value = match aligned {
            reg::CRH => self.crh,
            reg::CRL => self.crl,
            reg::PRLH => (self.prescaler >> 16) & 0xF,
            reg::PRLL => self.prescaler & 0xFFFF,
            reg::DIVH => (self.divider >> 16) & 0xF,
            reg::DIVL => self.divider & 0xFFFF,
            reg::CNTH => self.counter >> 16,
            reg::CNTL => self.counter & 0xFFFF,
            reg::ALRH => self.alarm >> 16,
            reg::ALRL => self.alarm & 0xFFFF,
            _ => {
                return Err(BusError::Unmapped {
                    addr: offset,
                    width,
                    write: false,
                })
            }
        };
        let _ = cx;
        Ok(extract(value, offset, width))
    }

    pub fn write_reg(
        &mut self,
        cx: &Ctx,
        offset: u32,
        width: AccessWidth,
        value: u32,
    ) -> Result<(), BusError> {
        let aligned = offset & !3;
        match aligned {
            reg::CRH => self.crh = merge(self.crh, offset, width, value) & 0x07,
            reg::CRL => {
                let merged = merge(self.crl, offset, width, value);
                if merged & crl::CNF != 0 {
                    self.crl |= crl::CNF;
                    self.crl &= !crl::RTOFF;
                } else if self.configuration_mode() {
                    self.crl &= !crl::CNF;
                    self.crl |= crl::RTOFF;
                }
                if value & crl::SECF != 0 {
                    self.crl &= !crl::SECF;
                }
                if value & crl::ALRF != 0 {
                    self.crl &= !crl::ALRF;
                }
                if value & crl::OWF != 0 {
                    self.crl &= !crl::OWF;
                }
            }
            reg::PRLH => {
                if self.configuration_mode() {
                    let high = field(value, 0, 4);
                    self.prescaler = (self.prescaler & 0xFFFF) | (high << 16);
                }
            }
            reg::PRLL => {
                if self.configuration_mode() {
                    self.prescaler = (self.prescaler & 0xF_0000) | (value & 0xFFFF);
                }
            }
            reg::DIVH | reg::DIVL => {
                if self.configuration_mode() {
                    self.divider = 0;
                }
            }
            reg::CNTH => {
                if self.configuration_mode() {
                    self.counter = (self.counter & 0xFFFF) | (field(value, 0, 16) << 16);
                }
            }
            reg::CNTL => {
                if self.configuration_mode() {
                    self.counter = (self.counter & 0xFFFF_0000) | (value & 0xFFFF);
                }
            }
            reg::ALRH => {
                if self.configuration_mode() {
                    self.alarm = (self.alarm & 0xFFFF) | (field(value, 0, 16) << 16);
                }
            }
            reg::ALRL => {
                if self.configuration_mode() {
                    self.alarm = (self.alarm & 0xFFFF_0000) | (value & 0xFFFF);
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
        self.refresh_irq(cx);
        Ok(())
    }
}

crate::impl_peripheral!(Rtc, "RTC");

#[cfg(test)]
mod tests {
    use super::*;
    use gsim_core::ctx::Clocks;
    use gsim_core::irq::lines;

    fn ctx(rtc_hz: u32) -> Ctx {
        Ctx::new(
            0,
            Clocks {
                rtc_hz,
                ..Clocks::HSI_8MHZ
            },
        )
    }

    #[test]
    fn prescaler_produces_one_second_ticks() {
        let cx = ctx(32_768);
        let mut rtc = Rtc::new();
        rtc.advance(&cx, 1_000_000_000);
        assert_eq!(rtc.counter(), 1);
        rtc.advance(&cx, 500_000_000);
        assert_eq!(rtc.counter(), 1);
        rtc.advance(&cx, 500_000_000);
        assert_eq!(rtc.counter(), 2);
    }

    #[test]
    fn second_interrupt_requires_the_enable() {
        let cx = ctx(32_768);
        let mut rtc = Rtc::new();
        rtc.write_reg(&cx, reg::CRH, AccessWidth::Word, crh::SECIE)
            .unwrap();
        cx.take_events();
        rtc.advance(&cx, 1_000_000_000);
        assert!(cx.take_events().iter().any(|e| matches!(
            e,
            gsim_core::Event::Irq { irq, high: true } if *irq == lines::RTC
        )));
    }

    #[test]
    fn alarm_matches_the_counter() {
        let cx = ctx(32_768);
        let mut rtc = Rtc::new();
        rtc.write_reg(&cx, reg::CRL, AccessWidth::Word, crl::CNF)
            .unwrap();
        rtc.write_reg(&cx, reg::ALRH, AccessWidth::Word, 0).unwrap();
        rtc.write_reg(&cx, reg::ALRL, AccessWidth::Word, 3).unwrap();
        rtc.write_reg(&cx, reg::CRL, AccessWidth::Word, 0).unwrap();
        rtc.advance(&cx, 3_000_000_000);
        assert_eq!(rtc.counter(), 3);
        assert_ne!(
            rtc.read_reg(&cx, reg::CRL, AccessWidth::Word).unwrap() & crl::ALRF,
            0
        );
    }

    #[test]
    fn writes_are_ignored_outside_configuration_mode() {
        let cx = ctx(32_768);
        let mut rtc = Rtc::new();
        rtc.write_reg(&cx, reg::ALRL, AccessWidth::Word, 42)
            .unwrap();
        assert_eq!(
            rtc.read_reg(&cx, reg::ALRL, AccessWidth::Word).unwrap(),
            0xFFFF
        );
    }

    #[test]
    fn no_clock_means_no_ticks() {
        let cx = ctx(0);
        let mut rtc = Rtc::new();
        rtc.advance(&cx, 2_000_000_000);
        assert_eq!(rtc.counter(), 0);
    }
}

impl Rtc {
    /// Labelled state for the peripheral status browser.
    pub fn status(&self, _clocks: &gsim_core::Clocks) -> Vec<(&'static str, String)> {
        vec![
            ("计数", format!("{} s", self.counter)),
            ("预分频", format!("{}", self.prescaler)),
            ("闹钟", format!("{}", self.alarm)),
            ("CRH", crate::status::hex(self.crh)),
            ("CRL", crate::status::hex(self.crl)),
        ]
    }
}