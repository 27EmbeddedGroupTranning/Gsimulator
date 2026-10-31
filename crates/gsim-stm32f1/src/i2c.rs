//! Inter-integrated circuit bus controller (I2C1/I2C2).
//!
//! The register level master state machine is modelled: `START`/`STOP`
//! generation, the address phase, `SB`/`ADDR`/`TXE`/`RXNE` flag sequencing and
//! the event/error interrupt lines. In this phase the bus is unpopulated, so
//! every address is answered with a NACK and the controller reports
//! `SR1.AF`, exactly as a real F103 does when nothing is on the bus. Attaching
//! targets is a later phase concern; the peripheral reports over the shared
//! event bus either way.

use super::map::i2c as reg;
use gsim_core::bus::{extract, merge, AccessWidth, BusError};
use gsim_core::ctx::Ctx;
use gsim_core::irq::Irq;

/// `CR1` bits.
pub mod cr1 {
    pub const PE: u32 = 1 << 0;
    pub const ENGC: u32 = 1 << 6;
    pub const START: u32 = 1 << 8;
    pub const STOP: u32 = 1 << 9;
    pub const ACK: u32 = 1 << 10;
    pub const POS: u32 = 1 << 11;
    pub const SWRST: u32 = 1 << 15;
}

/// `CR2` bits.
pub mod cr2 {
    pub const FREQ: u32 = 0x3F;
    pub const ITERREN: u32 = 1 << 8;
    pub const ITEVTEN: u32 = 1 << 9;
    pub const ITBUFEN: u32 = 1 << 10;
    pub const DMAEN: u32 = 1 << 11;
    pub const LAST: u32 = 1 << 12;
}

/// `SR1` bits.
pub mod sr1 {
    pub const SB: u32 = 1 << 0;
    pub const ADDR: u32 = 1 << 1;
    pub const BTF: u32 = 1 << 2;
    pub const STOPF: u32 = 1 << 4;
    pub const RXNE: u32 = 1 << 6;
    pub const TXE: u32 = 1 << 7;
    pub const BERR: u32 = 1 << 8;
    pub const ARLO: u32 = 1 << 9;
    pub const AF: u32 = 1 << 10;
    pub const OVR: u32 = 1 << 11;
}

/// `SR2` bits.
pub mod sr2 {
    pub const MSL: u32 = 1 << 0;
    pub const BUSY: u32 = 1 << 1;
    pub const TRA: u32 = 1 << 2;
}

/// One I2C instance.
#[derive(Debug, Clone)]
pub struct I2c {
    index: u8,
    event_irq: Irq,
    error_irq: Irq,
    cr1: u32,
    cr2: u32,
    oar1: u32,
    oar2: u32,
    dr: u32,
    sr1: u32,
    sr2: u32,
    ccr: u32,
    trise: u32,
    event_high: bool,
    error_high: bool,
}

impl I2c {
    pub fn new(index: u8, event_irq: Irq, error_irq: Irq) -> Self {
        I2c {
            index,
            event_irq,
            error_irq,
            cr1: 0,
            cr2: 0,
            oar1: 0,
            oar2: 0,
            dr: 0,
            sr1: 0,
            sr2: 0,
            ccr: 0,
            trise: 2,
            event_high: false,
            error_high: false,
        }
    }

    pub fn reset(&mut self) {
        let (index, event_irq, error_irq) = (self.index, self.event_irq, self.error_irq);
        *self = I2c::new(index, event_irq, error_irq);
    }

    /// Unit index (0 = I2C1).
    pub fn index(&self) -> u8 {
        self.index
    }

    /// True while the controller owns the bus.
    pub fn is_master(&self) -> bool {
        self.sr2 & sr2::MSL != 0
    }

    /// True while the bus is busy.
    pub fn bus_busy(&self) -> bool {
        self.sr2 & sr2::BUSY != 0
    }

    /// True when the last address or data phase was not acknowledged.
    pub fn nack(&self) -> bool {
        self.sr1 & sr1::AF != 0
    }

    /// `SR1.AF` is read-zero-to-clear: only a write that drives the bit low
    /// clears it.
    fn clear_af(&mut self, written: u32, lane_mask: u32) {
        if lane_mask & sr1::AF != 0 && written & sr1::AF == 0 {
            self.sr1 &= !sr1::AF;
        }
    }

    fn enabled(&self) -> bool {
        self.cr1 & cr1::PE != 0
    }

    fn status_flags(&self) -> u32 {
        let mut value = self.sr1 & !(sr1::TXE | sr1::RXNE);
        if self.enabled() && self.sr2 & sr2::MSL != 0 && self.sr2 & sr2::TRA != 0 {
            value |= sr1::TXE;
        }
        value
    }

    fn refresh_irq(&mut self, cx: &Ctx) {
        let flags = self.status_flags();
        let events = flags & (sr1::SB | sr1::ADDR | sr1::BTF | sr1::STOPF);
        let buffered = flags & (sr1::RXNE | sr1::TXE);
        let errors = flags & (sr1::AF | sr1::BERR | sr1::ARLO | sr1::OVR);

        let event_high = self.cr2 & cr2::ITEVTEN != 0
            && (events != 0 || (self.cr2 & cr2::ITBUFEN != 0 && buffered != 0));
        let error_high = self.cr2 & cr2::ITERREN != 0 && errors != 0;

        if event_high != self.event_high {
            self.event_high = event_high;
            cx.irq(self.event_irq, event_high);
        }
        if error_high != self.error_high {
            self.error_high = error_high;
            cx.irq(self.error_irq, error_high);
        }
    }

    fn start(&mut self, cx: &Ctx) {
        if !self.enabled() {
            return;
        }
        if self.sr2 & sr2::BUSY == 0 {
            self.sr2 |= sr2::BUSY | sr2::MSL;
            self.sr1 |= sr1::SB;
        }
        self.refresh_irq(cx);
    }

    fn stop(&mut self, cx: &Ctx) {
        self.sr1 &= !(sr1::SB | sr1::ADDR | sr1::BTF);
        self.sr2 &= !(sr2::BUSY | sr2::MSL | sr2::TRA);
        self.refresh_irq(cx);
    }

    /// Address or data phase: nothing answers on the unpopulated bus.
    fn transmit(&mut self, cx: &Ctx, value: u8) {
        self.dr = u32::from(value);
        if self.sr1 & sr1::SB != 0 {
            // Address byte: bit 0 selects transmit (0) or receive (1).
            self.sr1 &= !sr1::SB;
            if value & 1 == 0 {
                self.sr2 |= sr2::TRA;
            } else {
                self.sr2 &= !sr2::TRA;
            }
        }
        self.sr1 |= sr1::AF;
        self.refresh_irq(cx);
    }

    pub fn read_reg(&mut self, cx: &Ctx, offset: u32, width: AccessWidth) -> Result<u32, BusError> {
        let value = match offset & !3 {
            reg::CR1 => self.cr1 & !(cr1::START | cr1::STOP),
            reg::CR2 => self.cr2,
            reg::OAR1 => self.oar1,
            reg::OAR2 => self.oar2,
            reg::DR => {
                self.sr1 &= !sr1::RXNE;
                self.refresh_irq(cx);
                self.dr
            }
            reg::SR1 => self.status_flags(),
            reg::SR2 => self.sr2,
            reg::CCR => self.ccr,
            reg::TRISE => self.trise,
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
            reg::CR1 => {
                if value & cr1::SWRST != 0 {
                    let (index, event_irq, error_irq) =
                        (self.index, self.event_irq, self.error_irq);
                    *self = I2c::new(index, event_irq, error_irq);
                } else {
                    self.cr1 = merge(self.cr1, offset, width, value) & 0xBFFF;
                    self.cr1 &= !(cr1::START | cr1::STOP);
                    if value & cr1::START != 0 {
                        self.start(cx);
                    }
                    if value & cr1::STOP != 0 {
                        self.stop(cx);
                    }
                }
            }
            reg::CR2 => self.cr2 = merge(self.cr2, offset, width, value) & 0x1F3F,
            reg::OAR1 => self.oar1 = merge(self.oar1, offset, width, value) & 0x83FF,
            reg::OAR2 => self.oar2 = merge(self.oar2, offset, width, value) & 0x00FF,
            reg::DR => {
                let byte = (value & 0xFF) as u8;
                if self.enabled() && self.sr2 & sr2::MSL != 0 {
                    self.transmit(cx, byte);
                } else {
                    self.dr = u32::from(byte);
                }
            }
            reg::SR1 => {
                // Only AF is read-zero-to-clear; the other bits are read only.
                let written = merge(0, offset, width, value);
                let lane_mask = width.mask() << ((offset & 3) * 8);
                self.clear_af(written, lane_mask);
            }
            reg::SR2 => {}
            reg::CCR => self.ccr = merge(self.ccr, offset, width, value) & 0x8FFF,
            reg::TRISE => self.trise = merge(self.trise, offset, width, value) & 0x003F,
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

    pub fn advance(&mut self, _cx: &Ctx, _elapsed_ns: u64) {}
}

crate::impl_peripheral!(I2c, "I2C");

impl I2c {
    /// Labelled state for the peripheral status browser.
    pub fn status(&self, _clocks: &gsim_core::Clocks) -> Vec<(&'static str, String)> {
        vec![
            ("使能", crate::status::on_off(self.cr1 & 1 != 0).to_string()),
            (
                "速度",
                if self.ccr & (1 << 15) != 0 {
                    "快速模式"
                } else {
                    "标准模式"
                }
                .to_string(),
            ),
            ("SR1", crate::status::hex(self.sr1)),
            ("SR2", crate::status::hex(self.sr2)),
            ("CCR", crate::status::hex(self.ccr)),
            ("TRISE", crate::status::hex(self.trise)),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use gsim_core::ctx::Clocks;
    use gsim_core::irq::lines;

    fn ctx() -> Ctx {
        Ctx::new(0, Clocks::HSI_8MHZ)
    }

    fn enabled() -> (I2c, Ctx) {
        let cx = ctx();
        let mut i2c = I2c::new(0, lines::I2C1_EV, lines::I2C1_ER);
        i2c.write_reg(&cx, reg::CR1, AccessWidth::Word, cr1::PE)
            .unwrap();
        // 100 kHz at 8 MHz: CR2.FREQ = 8, CCR = 40, TRISE = 9.
        i2c.write_reg(&cx, reg::CR2, AccessWidth::Word, 8).unwrap();
        i2c.write_reg(&cx, reg::CCR, AccessWidth::Word, 40).unwrap();
        i2c.write_reg(&cx, reg::TRISE, AccessWidth::Word, 9)
            .unwrap();
        (i2c, cx)
    }

    #[test]
    fn start_sets_sb_and_bus_busy() {
        let (mut i2c, cx) = enabled();
        i2c.write_reg(&cx, reg::CR1, AccessWidth::Word, cr1::PE | cr1::START)
            .unwrap();
        assert_eq!(
            i2c.read_reg(&cx, reg::SR1, AccessWidth::Word).unwrap() & sr1::SB,
            sr1::SB
        );
        assert!(i2c.bus_busy());
        assert!(i2c.is_master());
    }

    #[test]
    fn address_phase_reports_nack_without_a_target() {
        let (mut i2c, cx) = enabled();
        i2c.write_reg(&cx, reg::CR1, AccessWidth::Word, cr1::PE | cr1::START)
            .unwrap();
        i2c.write_reg(&cx, reg::DR, AccessWidth::Word, 0xA0)
            .unwrap();
        let status = i2c.read_reg(&cx, reg::SR1, AccessWidth::Word).unwrap();
        assert_eq!(status & sr1::AF, sr1::AF, "no slave answers on the bus");
        assert_eq!(status & sr1::SB, 0);
        assert!(i2c.nack());

        // AF is cleared by writing 0 to it.
        i2c.write_reg(&cx, reg::SR1, AccessWidth::Word, 0).unwrap();
        assert!(!i2c.nack());
    }

    #[test]
    fn stop_releases_the_bus() {
        let (mut i2c, cx) = enabled();
        i2c.write_reg(&cx, reg::CR1, AccessWidth::Word, cr1::PE | cr1::START)
            .unwrap();
        i2c.write_reg(&cx, reg::CR1, AccessWidth::Word, cr1::PE | cr1::STOP)
            .unwrap();
        assert!(!i2c.bus_busy());
        assert!(!i2c.is_master());
    }

    #[test]
    fn error_interrupt_follows_ack_failure() {
        let (mut i2c, cx) = enabled();
        i2c.write_reg(
            &cx,
            reg::CR2,
            AccessWidth::Word,
            8 | cr2::ITERREN | cr2::ITEVTEN,
        )
        .unwrap();
        cx.take_events();
        i2c.write_reg(&cx, reg::CR1, AccessWidth::Word, cr1::PE | cr1::START)
            .unwrap();
        i2c.write_reg(&cx, reg::DR, AccessWidth::Word, 0xA0)
            .unwrap();
        let events = cx.take_events();
        assert!(events.iter().any(|e| matches!(
            e,
            gsim_core::Event::Irq { irq, high: true } if *irq == lines::I2C1_ER
        )));
    }

    #[test]
    fn software_reset_clears_configuration() {
        let (mut i2c, cx) = enabled();
        i2c.write_reg(&cx, reg::CR1, AccessWidth::Word, cr1::PE | cr1::SWRST)
            .unwrap();
        assert_eq!(i2c.read_reg(&cx, reg::CR1, AccessWidth::Word).unwrap(), 0);
        assert_eq!(i2c.read_reg(&cx, reg::CCR, AccessWidth::Word).unwrap(), 0);
    }
}
