//! Serial peripheral interface (SPI1/SPI2).
//!
//! The block is modelled as a full featured controller: baud rate prescaler,
//! 8/16 bit frames, `TXE`/`RXNE`/`BSY`/`OVR` semantics and interrupt
//! generation. The MISO line is fed by [`Spi::set_miso`], the attachment point
//! used by the test harness today and by external device models later.

use super::map::spi as reg;
use gsim_core::bus::{extract, merge, AccessWidth, BusError};
use gsim_core::ctx::{Ctx, Event};
use gsim_core::irq::Irq;

/// `CR1` bits.
pub mod cr1 {
    pub const CPHA: u32 = 1 << 0;
    pub const CPOL: u32 = 1 << 1;
    pub const MSTR: u32 = 1 << 2;
    pub const BR: u32 = 0b111 << 3;
    pub const SPE: u32 = 1 << 6;
    pub const LSBFIRST: u32 = 1 << 7;
    pub const SSI: u32 = 1 << 8;
    pub const SSM: u32 = 1 << 9;
    pub const RXONLY: u32 = 1 << 10;
    pub const DFF: u32 = 1 << 11;
    pub const BIDIOE: u32 = 1 << 14;
    pub const BIDIMODE: u32 = 1 << 15;
}

/// `CR2` bits.
pub mod cr2 {
    pub const RXDMAEN: u32 = 1 << 0;
    pub const TXDMAEN: u32 = 1 << 1;
    pub const SSOE: u32 = 1 << 2;
    pub const ERRIE: u32 = 1 << 5;
    pub const RXNEIE: u32 = 1 << 6;
    pub const TXEIE: u32 = 1 << 7;
}

/// `SR` bits.
pub mod sr {
    pub const RXNE: u32 = 1 << 0;
    pub const TXE: u32 = 1 << 1;
    pub const UDR: u32 = 1 << 3;
    pub const CRCERR: u32 = 1 << 4;
    pub const MODF: u32 = 1 << 5;
    pub const OVR: u32 = 1 << 6;
    pub const BSY: u32 = 1 << 7;
}

#[derive(Debug, Clone, Copy)]
struct Shift {
    tx: u16,
    remaining_ns: u64,
}

/// One SPI instance.
#[derive(Debug, Clone)]
pub struct Spi {
    index: u8,
    irq: Irq,
    cr1: u32,
    cr2: u32,
    crcpr: u32,
    i2scfgr: u32,
    i2spr: u32,
    tx_buffer: Option<u16>,
    shift: Option<Shift>,
    rx: Option<u16>,
    overrun: bool,
    miso: u16,
    irq_high: bool,
}

impl Spi {
    pub fn new(index: u8, irq: Irq) -> Self {
        Spi {
            index,
            irq,
            cr1: 0,
            cr2: 0,
            crcpr: 0x0007,
            i2scfgr: 0,
            i2spr: 0,
            tx_buffer: None,
            shift: None,
            rx: None,
            overrun: false,
            miso: 0,
            irq_high: false,
        }
    }

    pub fn reset(&mut self) {
        let (index, irq) = (self.index, self.irq);
        *self = Spi::new(index, irq);
    }

    /// Unit index (0 = SPI1).
    pub fn index(&self) -> u8 {
        self.index
    }

    /// Provides the value shifted in on MISO during the next frame.
    pub fn set_miso(&mut self, value: u16) {
        self.miso = value;
    }

    /// True while a frame is being shifted.
    pub fn busy(&self) -> bool {
        self.shift.is_some()
    }

    fn word_mask(&self) -> u32 {
        if self.cr1 & cr1::DFF != 0 {
            0xFFFF
        } else {
            0xFF
        }
    }

    fn bits_per_frame(&self) -> u64 {
        if self.cr1 & cr1::DFF != 0 {
            16
        } else {
            8
        }
    }

    /// SCK frequency for the current `BR` setting.
    pub fn sck_hz(&self, pclk_hz: u32) -> u32 {
        let br = (self.cr1 & cr1::BR) >> 3;
        pclk_hz / (1 << (br + 1))
    }

    fn clock_hz(&self, cx: &Ctx) -> u32 {
        if self.index == 0 {
            cx.clocks().pclk2_hz
        } else {
            cx.clocks().pclk1_hz
        }
    }

    fn frame_ns(&self, cx: &Ctx) -> u64 {
        let pclk = self.clock_hz(cx);
        let sck = self.sck_hz(pclk);
        if sck == 0 {
            return 0;
        }
        self.bits_per_frame() * gsim_core::time::NANOS_PER_SEC / u64::from(sck)
    }

    fn status_flags(&self) -> u32 {
        let mut value = 0;
        if self.rx.is_some() {
            value |= sr::RXNE;
        }
        if self.tx_buffer.is_none() {
            value |= sr::TXE;
        }
        if self.shift.is_some() {
            value |= sr::BSY;
        }
        if self.overrun {
            value |= sr::OVR;
        }
        value
    }

    fn should_irq(&self) -> bool {
        let flags = self.status_flags();
        (self.cr2 & cr2::RXNEIE != 0 && flags & sr::RXNE != 0)
            || (self.cr2 & cr2::TXEIE != 0 && flags & sr::TXE != 0)
            || (self.cr2 & cr2::ERRIE != 0 && self.overrun)
    }

    fn refresh_irq(&mut self, cx: &Ctx) {
        let high = self.should_irq();
        if high != self.irq_high {
            self.irq_high = high;
            cx.irq(self.irq, high);
        }
    }

    fn start_transfer(&mut self, cx: &Ctx) {
        if self.shift.is_some() || self.cr1 & cr1::SPE == 0 {
            return;
        }
        let Some(tx) = self.tx_buffer.take() else {
            return;
        };
        let duration = self.frame_ns(cx);
        if duration == 0 {
            self.finish_transfer(cx, tx);
        } else {
            self.shift = Some(Shift {
                tx,
                remaining_ns: duration,
            });
        }
    }

    fn finish_transfer(&mut self, cx: &Ctx, tx: u16) {
        let rx = self.miso & self.word_mask() as u16;
        if self.rx.is_some() {
            self.overrun = true;
        } else {
            self.rx = Some(rx);
        }
        cx.emit(Event::SpiTransfer {
            unit: self.index,
            tx,
            rx,
        });
        self.refresh_irq(cx);
    }

    pub fn advance(&mut self, cx: &Ctx, elapsed_ns: u64) {
        if let Some(mut shift) = self.shift {
            if elapsed_ns >= shift.remaining_ns {
                self.shift = None;
                self.finish_transfer(cx, shift.tx);
                self.start_transfer(cx);
            } else {
                shift.remaining_ns -= elapsed_ns;
                self.shift = Some(shift);
            }
        } else {
            self.start_transfer(cx);
        }
        self.refresh_irq(cx);
    }

    pub fn read_reg(&mut self, cx: &Ctx, offset: u32, width: AccessWidth) -> Result<u32, BusError> {
        let value = match offset & !3 {
            reg::CR1 => self.cr1,
            reg::CR2 => self.cr2,
            reg::SR => self.status_flags(),
            reg::DR => {
                let value = self.rx.take().unwrap_or(0);
                self.overrun = false;
                self.refresh_irq(cx);
                u32::from(value)
            }
            reg::CRCPR => self.crcpr,
            reg::RXCRCR | reg::TXCRCR => 0,
            reg::I2SCFGR => self.i2scfgr,
            reg::I2SPR => self.i2spr,
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
                self.cr1 = merge(self.cr1, offset, width, value) & 0xFF7F;
                if self.cr1 & cr1::SPE == 0 {
                    self.shift = None;
                    self.tx_buffer = None;
                }
            }
            reg::CR2 => self.cr2 = merge(self.cr2, offset, width, value) & 0x00E7,
            reg::SR => {}
            reg::DR => {
                let frame = (value & self.word_mask()) as u16;
                self.tx_buffer = Some(frame);
                self.start_transfer(cx);
            }
            reg::CRCPR => self.crcpr = merge(self.crcpr, offset, width, value) & 0xFFFF,
            reg::RXCRCR | reg::TXCRCR => {}
            reg::I2SCFGR => self.i2scfgr = merge(self.i2scfgr, offset, width, value) & 0x0FFF,
            reg::I2SPR => self.i2spr = merge(self.i2spr, offset, width, value) & 0x03FF,
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

    /// True when the DMA transmit request line is asserted.
    pub fn dma_tx_request(&self) -> bool {
        self.cr2 & cr2::TXDMAEN != 0 && self.tx_buffer.is_none() && self.cr1 & cr1::SPE != 0
    }

    /// True when the DMA receive request line is asserted.
    pub fn dma_rx_request(&self) -> bool {
        self.cr2 & cr2::RXDMAEN != 0 && self.rx.is_some()
    }
}

crate::impl_peripheral!(Spi, "SPI");

impl Spi {
    /// Labelled state for the peripheral status browser.
    pub fn status(&self, clocks: &gsim_core::Clocks) -> Vec<(&'static str, String)> {
        let pclk = if self.index == 0 {
            clocks.pclk2_hz
        } else {
            clocks.pclk1_hz
        };
        let mut flags = Vec::new();
        if self.tx_buffer.is_none() {
            flags.push("TXE");
        }
        if self.rx.is_some() {
            flags.push("RXNE");
        }
        if self.shift.is_some() {
            flags.push("BSY");
        }
        if self.overrun {
            flags.push("OVR");
        }
        vec![
            (
                "使能",
                crate::status::on_off(self.cr1 & cr1::SPE != 0).to_string(),
            ),
            (
                "模式",
                if self.cr1 & cr1::MSTR != 0 {
                    "主机"
                } else {
                    "从机"
                }
                .to_string(),
            ),
            ("SCK", crate::status::hz(self.sck_hz(pclk))),
            (
                "帧宽",
                if self.cr1 & cr1::DFF != 0 {
                    "16 位"
                } else {
                    "8 位"
                }
                .to_string(),
            ),
            (
                "标志",
                if flags.is_empty() {
                    "无".to_string()
                } else {
                    flags.join(" ")
                },
            ),
            ("CR1", crate::status::hex(self.cr1)),
            ("CR2", crate::status::hex(self.cr2)),
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

    fn configured() -> (Spi, Ctx) {
        let cx = ctx();
        let mut spi = Spi::new(0, lines::SPI1);
        // Master, fPCLK/8, SPE.
        spi.write_reg(
            &cx,
            reg::CR1,
            AccessWidth::Word,
            cr1::MSTR | (0b010 << 3) | cr1::SPE,
        )
        .unwrap();
        (spi, cx)
    }

    #[test]
    fn sck_frequency_follows_the_prescaler() {
        let (spi, cx) = configured();
        assert_eq!(spi.sck_hz(cx.clocks().pclk2_hz), 1_000_000);
    }

    #[test]
    fn frame_is_shifted_and_rx_value_is_captured() {
        let (mut spi, cx) = configured();
        spi.set_miso(0x00);
        cx.take_events();
        spi.write_reg(&cx, reg::DR, AccessWidth::Word, 0x09)
            .unwrap();
        assert!(spi.busy());
        spi.advance(&cx, 8_000); // 8 bits at 1 MHz
        assert!(!spi.busy());
        let status = spi.read_reg(&cx, reg::SR, AccessWidth::Word).unwrap();
        assert_eq!(status & sr::RXNE, sr::RXNE);
        assert_eq!(status & sr::TXE, sr::TXE);
        assert_eq!(spi.read_reg(&cx, reg::DR, AccessWidth::Word).unwrap(), 0);
        assert_eq!(
            cx.take_events(),
            vec![Event::SpiTransfer {
                unit: 0,
                tx: 0x09,
                rx: 0
            }]
        );
    }

    #[test]
    fn rx_not_read_causes_overrun_on_next_frame() {
        let (mut spi, cx) = configured();
        spi.write_reg(&cx, reg::DR, AccessWidth::Word, 0xAA)
            .unwrap();
        spi.advance(&cx, 8_000);
        spi.write_reg(&cx, reg::DR, AccessWidth::Word, 0xBB)
            .unwrap();
        spi.advance(&cx, 8_000);
        let status = spi.read_reg(&cx, reg::SR, AccessWidth::Word).unwrap();
        assert_eq!(status & sr::OVR, sr::OVR);
    }

    #[test]
    fn sixteen_bit_frames_take_twice_as_long() {
        let cx = ctx();
        let mut spi = Spi::new(0, lines::SPI1);
        spi.write_reg(
            &cx,
            reg::CR1,
            AccessWidth::Word,
            cr1::MSTR | (0b010 << 3) | cr1::SPE | cr1::DFF,
        )
        .unwrap();
        spi.set_miso(0x1234);
        spi.write_reg(&cx, reg::DR, AccessWidth::Word, 0xBEEF)
            .unwrap();
        spi.advance(&cx, 15_000);
        assert!(spi.busy());
        spi.advance(&cx, 1_000);
        assert_eq!(
            spi.read_reg(&cx, reg::DR, AccessWidth::Word).unwrap(),
            0x1234
        );
    }

    #[test]
    fn disabling_the_peripheral_aborts_the_frame() {
        let (mut spi, cx) = configured();
        spi.write_reg(&cx, reg::DR, AccessWidth::Word, 0x55)
            .unwrap();
        spi.write_reg(&cx, reg::CR1, AccessWidth::Word, 0).unwrap();
        spi.advance(&cx, 20_000);
        assert!(!spi.busy());
        assert_eq!(
            spi.read_reg(&cx, reg::SR, AccessWidth::Word).unwrap() & sr::RXNE,
            0
        );
    }
}
