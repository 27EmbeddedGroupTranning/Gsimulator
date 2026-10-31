//! Analog to digital converter (ADC1/ADC2).
//!
//! The model walks the regular conversion sequence at the real conversion
//! time (`sampling + 12.5` ADCCLK cycles), raises `EOC`/`EOS`, supports single,
//! continuous and scan modes, the calibration sequence and DMA requests.
//! Analog inputs are set with [`Adc::set_channel_mv`], which is how the test
//! harness (and later a board model) provides voltages.

use super::map::adc as reg;
use gsim_core::bus::{extract, field, merge, AccessWidth, BusError};
use gsim_core::ctx::Ctx;
use gsim_core::irq::Irq;
use gsim_core::time::NanoAccumulator;

/// `SR` bits.
pub mod sr {
    pub const AWD: u32 = 1 << 0;
    pub const EOC: u32 = 1 << 1;
    pub const JEOC: u32 = 1 << 2;
    pub const JSTRT: u32 = 1 << 3;
    pub const STRT: u32 = 1 << 4;
}

/// `CR1` bits.
pub mod cr1 {
    pub const EOCIE: u32 = 1 << 5;
    pub const SCAN: u32 = 1 << 8;
    pub const DISCEN: u32 = 1 << 11;
}

/// `CR2` bits.
pub mod cr2 {
    pub const ADON: u32 = 1 << 0;
    pub const CONT: u32 = 1 << 1;
    pub const CAL: u32 = 1 << 2;
    pub const RSTCAL: u32 = 1 << 3;
    pub const DMA: u32 = 1 << 8;
    pub const ALIGN: u32 = 1 << 11;
    pub const EXTTRIG: u32 = 1 << 20;
    pub const SWSTART: u32 = 1 << 22;
}

/// Sampling time in ADCCLK cycles for each `SMP` code.
const SAMPLING_CYCLES: [u32; 8] = [1, 7, 13, 28, 41, 55, 71, 239];
/// Calibration takes 83 ADCCLK cycles.
const CALIBRATION_CYCLES: u64 = 83;
/// Reference voltage of the converter in millivolts.
pub const VREF_MV: u32 = 3300;
/// Full scale code.
pub const FULL_SCALE: u32 = 4095;
/// Analog channel count (16 external + internal temperature/reference).
pub const CHANNEL_COUNT: usize = 18;

const MAX_CONVERSIONS_PER_STEP: u64 = 1024;

/// One ADC instance.
#[derive(Debug, Clone)]
pub struct Adc {
    index: u8,
    irq: Irq,
    sr: u32,
    cr1: u32,
    cr2: u32,
    smpr1: u32,
    smpr2: u32,
    sqr1: u32,
    sqr2: u32,
    sqr3: u32,
    jsqr: u32,
    htr: u32,
    ltr: u32,
    jofr: [u32; 4],
    jdr: [u32; 4],
    dr: u32,
    inputs_mv: [u32; CHANNEL_COUNT],
    position: usize,
    remaining_ns: u64,
    active: bool,
    calibrating_ns: u64,
    enabled: bool,
    acc: NanoAccumulator,
    irq_high: bool,
}

impl Adc {
    pub fn new(index: u8, irq: Irq) -> Self {
        Adc {
            index,
            irq,
            sr: 0,
            cr1: 0,
            cr2: 0,
            smpr1: 0,
            smpr2: 0,
            sqr1: 0,
            sqr2: 0,
            sqr3: 0,
            jsqr: 0,
            htr: 0x0FFF,
            ltr: 0,
            jofr: [0; 4],
            jdr: [0; 4],
            dr: 0,
            inputs_mv: [0; CHANNEL_COUNT],
            position: 0,
            remaining_ns: 0,
            active: false,
            calibrating_ns: 0,
            enabled: false,
            acc: NanoAccumulator::new(),
            irq_high: false,
        }
    }

    pub fn reset(&mut self) {
        let (index, irq) = (self.index, self.irq);
        *self = Adc::new(index, irq);
    }

    /// Unit index (0 = ADC1).
    pub fn index(&self) -> u8 {
        self.index
    }

    /// Provides the voltage present on an analog channel.
    pub fn set_channel_mv(&mut self, channel: usize, millivolts: u32) {
        if channel < CHANNEL_COUNT {
            self.inputs_mv[channel] = millivolts;
        }
    }

    /// True while a regular sequence is converting.
    pub fn converting(&self) -> bool {
        self.active
    }

    /// Number of regular conversions configured by `SQR1.L`.
    fn sequence_length(&self) -> usize {
        (field(self.sqr1, 20, 4) as usize + 1).min(16)
    }

    /// Regular conversion sequence, `SQ1` first.
    fn sequence(&self) -> Vec<u8> {
        let mut channels = Vec::with_capacity(16);
        for position in 0..16u32 {
            let (register, shift) = match position {
                0..=5 => (self.sqr3, position * 5),
                6..=11 => (self.sqr2, (position - 6) * 5),
                _ => (self.sqr1, (position - 12) * 5),
            };
            channels.push(field(register, shift, 5) as u8);
        }
        channels.truncate(self.sequence_length());
        channels
    }

    fn sampling_cycles(&self, channel: u8) -> u32 {
        let channel = u32::from(channel) & 0x1F;
        let (register, shift) = if channel < 10 {
            (self.smpr2, channel * 3)
        } else {
            (self.smpr1, (channel - 10) * 3)
        };
        SAMPLING_CYCLES[field(register, shift, 3) as usize]
    }

    fn conversion_ns(&self, cx: &Ctx, channel: u8) -> u64 {
        let adcclk = cx.clocks().adc_hz;
        if adcclk == 0 {
            return 0;
        }
        let cycles = u64::from(self.sampling_cycles(channel)) * 2 + 25;
        cycles * gsim_core::time::NANOS_PER_SEC / (2 * u64::from(adcclk))
    }

    fn digital_value(&self, channel: u8) -> u32 {
        let mv = self
            .inputs_mv
            .get(usize::from(channel))
            .copied()
            .unwrap_or(0);
        ((mv.min(VREF_MV) as u64 * u64::from(FULL_SCALE)) / u64::from(VREF_MV)) as u32
    }

    fn start_sequence(&mut self, cx: &Ctx) {
        if !self.enabled || self.active {
            return;
        }
        let sequence = self.sequence();
        let Some(&first) = sequence.first() else {
            return;
        };
        self.position = 0;
        self.active = true;
        self.remaining_ns = self.conversion_ns(cx, first);
        self.sr |= sr::STRT;
        self.refresh_irq(cx);
    }

    fn complete_conversion(&mut self, cx: &Ctx) {
        let sequence = self.sequence();
        let channel = sequence.get(self.position).copied().unwrap_or(0);
        let value = self.digital_value(channel);
        self.dr = if self.cr2 & cr2::ALIGN != 0 {
            value << 4
        } else {
            value
        };
        self.sr |= sr::EOC;
        self.sr &= !sr::STRT;

        let last = self.position + 1 >= sequence.len();
        let scan = self.cr1 & cr1::SCAN != 0;
        if last || !scan {
            if self.cr2 & cr2::CONT != 0 && !scan {
                self.position = 0;
                self.remaining_ns = self.conversion_ns(cx, channel);
            } else if last && self.cr2 & cr2::CONT != 0 {
                self.position = 0;
                self.remaining_ns = self.conversion_ns(cx, sequence[0]);
            } else {
                self.active = false;
            }
        } else {
            self.position += 1;
            self.remaining_ns = self.conversion_ns(cx, sequence[self.position]);
        }
        self.refresh_irq(cx);
    }

    fn refresh_irq(&mut self, cx: &Ctx) {
        let high = self.cr1 & cr1::EOCIE != 0 && self.sr & sr::EOC != 0;
        if high != self.irq_high {
            self.irq_high = high;
            cx.irq(self.irq, high);
        }
    }

    pub fn advance(&mut self, cx: &Ctx, elapsed_ns: u64) {
        if self.calibrating_ns > 0 {
            if elapsed_ns >= self.calibrating_ns {
                self.calibrating_ns = 0;
                self.cr2 &= !cr2::CAL;
                self.sr |= sr::EOC;
                self.refresh_irq(cx);
            } else {
                self.calibrating_ns -= elapsed_ns;
                return;
            }
        }
        if !self.active {
            self.acc.reset();
            return;
        }
        let mut remaining_elapsed = elapsed_ns;
        let mut guard = 0;
        while self.active
            && remaining_elapsed >= self.remaining_ns
            && guard < MAX_CONVERSIONS_PER_STEP
        {
            remaining_elapsed -= self.remaining_ns;
            self.complete_conversion(cx);
            guard += 1;
        }
        if self.active {
            self.remaining_ns = self.remaining_ns.saturating_sub(remaining_elapsed);
        }
        if guard >= MAX_CONVERSIONS_PER_STEP {
            cx.note(
                "ADC",
                "conversion catch-up truncated",
                u32::from(self.index),
            );
        }
    }

    pub fn read_reg(&mut self, cx: &Ctx, offset: u32, width: AccessWidth) -> Result<u32, BusError> {
        let value = match offset & !3 {
            reg::SR => self.sr,
            reg::CR1 => self.cr1,
            reg::CR2 => self.cr2,
            reg::SMPR1 => self.smpr1,
            reg::SMPR2 => self.smpr2,
            reg::JOFR1..=reg::JOFR_LAST => self.jofr[((offset & !3) - reg::JOFR1) as usize / 4],
            reg::HTR => self.htr,
            reg::LTR => self.ltr,
            reg::SQR1 => self.sqr1,
            reg::SQR2 => self.sqr2,
            reg::SQR3 => self.sqr3,
            reg::JSQR => self.jsqr,
            reg::JDR1..=reg::JDR_LAST => self.jdr[((offset & !3) - reg::JDR1) as usize / 4],
            reg::DR => {
                self.sr &= !sr::EOC;
                self.refresh_irq(cx);
                self.dr
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
        cx: &Ctx,
        offset: u32,
        width: AccessWidth,
        value: u32,
    ) -> Result<(), BusError> {
        match offset & !3 {
            reg::SR => self.sr &= !(merge(0, offset, width, value) & 0x1F),
            reg::CR1 => self.cr1 = merge(self.cr1, offset, width, value) & 0xFFFF,
            reg::CR2 => {
                let merged = merge(self.cr2, offset, width, value);
                if merged & cr2::ADON != 0 && !self.enabled {
                    self.enabled = true;
                }
                if merged & cr2::ADON == 0 {
                    self.enabled = false;
                    self.active = false;
                }
                if merged & cr2::CAL != 0 {
                    self.calibrating_ns = {
                        let adcclk = cx.clocks().adc_hz.max(1);
                        CALIBRATION_CYCLES * gsim_core::time::NANOS_PER_SEC / u64::from(adcclk)
                    };
                }
                if merged & cr2::RSTCAL != 0 {
                    self.cr2 &= !cr2::CAL;
                }
                self.cr2 = merged & !(cr2::SWSTART | cr2::RSTCAL);
                if value & cr2::SWSTART != 0 {
                    self.start_sequence(cx);
                }
            }
            reg::SMPR1 => self.smpr1 = merge(self.smpr1, offset, width, value) & 0x00FF_FFFF,
            reg::SMPR2 => self.smpr2 = merge(self.smpr2, offset, width, value) & 0x3FFF_FFFF,
            reg::JOFR1..=reg::JOFR_LAST => {
                let index = ((offset & !3) - reg::JOFR1) as usize / 4;
                self.jofr[index] = merge(self.jofr[index], offset, width, value) & 0x0FFF;
            }
            reg::HTR => self.htr = merge(self.htr, offset, width, value) & 0x0FFF,
            reg::LTR => self.ltr = merge(self.ltr, offset, width, value) & 0x0FFF,
            reg::SQR1 => self.sqr1 = merge(self.sqr1, offset, width, value) & 0x00FF_FFFF,
            reg::SQR2 => self.sqr2 = merge(self.sqr2, offset, width, value) & 0x3FFF_FFFF,
            reg::SQR3 => self.sqr3 = merge(self.sqr3, offset, width, value) & 0x3FFF_FFFF,
            reg::JSQR => self.jsqr = merge(self.jsqr, offset, width, value) & 0x003F_FFFF,
            reg::JDR1..=reg::JDR_LAST => {
                let index = ((offset & !3) - reg::JDR1) as usize / 4;
                self.jdr[index] = merge(self.jdr[index], offset, width, value) & 0xFFFF;
            }
            reg::DR => self.dr = merge(self.dr, offset, width, value) & 0xFFFF,
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

    /// True when the `EOC` DMA request line is asserted.
    pub fn dma_request(&self) -> bool {
        self.cr2 & cr2::DMA != 0 && self.sr & sr::EOC != 0
    }
}

crate::impl_peripheral!(Adc, "ADC");

impl Adc {
    /// Labelled state for the peripheral status browser.
    pub fn status(&self, _clocks: &gsim_core::Clocks) -> Vec<(&'static str, String)> {
        vec![
            ("使能", crate::status::on_off(self.enabled).to_string()),
            (
                "转换",
                if self.active { "进行中" } else { "空闲" }.to_string(),
            ),
            ("数据寄存器", format!("{}", self.dr)),
            ("状态 SR", crate::status::hex(self.sr)),
            ("控制 CR2", crate::status::hex(self.cr2)),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use gsim_core::ctx::Clocks;
    use gsim_core::irq::lines;

    fn ctx() -> Ctx {
        Ctx::new(
            0,
            Clocks {
                adc_hz: 12_000_000,
                ..Clocks::HSI_8MHZ
            },
        )
    }

    fn adc() -> (Adc, Ctx) {
        let cx = ctx();
        let mut adc = Adc::new(0, lines::ADC1_2);
        adc.write_reg(&cx, reg::SMPR2, AccessWidth::Word, 0)
            .unwrap();
        adc.write_reg(&cx, reg::SQR1, AccessWidth::Word, 0).unwrap();
        adc.write_reg(&cx, reg::SQR3, AccessWidth::Word, 5).unwrap();
        adc.write_reg(&cx, reg::CR2, AccessWidth::Word, cr2::ADON)
            .unwrap();
        (adc, cx)
    }

    #[test]
    fn single_conversion_produces_a_code() {
        let (mut adc, cx) = adc();
        adc.set_channel_mv(5, 1650);
        adc.write_reg(&cx, reg::CR2, AccessWidth::Word, cr2::ADON | cr2::SWSTART)
            .unwrap();
        // 1.5 + 12.5 = 14 ADCCLK cycles at 12 MHz is ~1.17 us.
        adc.advance(&cx, 1_200);
        let status = adc.read_reg(&cx, reg::SR, AccessWidth::Word).unwrap();
        assert_eq!(status & sr::EOC, sr::EOC);
        let code = adc.read_reg(&cx, reg::DR, AccessWidth::Word).unwrap();
        assert!(
            (code as i32 - 2047).abs() <= 1,
            "1650 mV should map to about half scale, got {code}"
        );
        assert_eq!(
            adc.read_reg(&cx, reg::SR, AccessWidth::Word).unwrap() & sr::EOC,
            0,
            "reading DR clears EOC"
        );
    }

    #[test]
    fn scan_mode_walks_the_whole_sequence() {
        let (mut adc, cx) = adc();
        adc.set_channel_mv(5, 3300);
        adc.set_channel_mv(6, 0);
        adc.write_reg(&cx, reg::SQR3, AccessWidth::Word, 5 | (6 << 5))
            .unwrap();
        adc.write_reg(&cx, reg::SQR1, AccessWidth::Word, 1 << 20)
            .unwrap();
        adc.write_reg(&cx, reg::CR1, AccessWidth::Word, cr1::SCAN)
            .unwrap();
        adc.write_reg(&cx, reg::CR2, AccessWidth::Word, cr2::ADON | cr2::SWSTART)
            .unwrap();

        adc.advance(&cx, 1_200);
        assert_eq!(
            adc.read_reg(&cx, reg::DR, AccessWidth::Word).unwrap(),
            FULL_SCALE
        );
        adc.advance(&cx, 1_200);
        assert_eq!(adc.read_reg(&cx, reg::DR, AccessWidth::Word).unwrap(), 0);
        assert!(!adc.converting(), "sequence finished");
    }

    #[test]
    fn continuous_mode_keeps_restarting() {
        let (mut adc, cx) = adc();
        adc.write_reg(
            &cx,
            reg::CR2,
            AccessWidth::Word,
            cr2::ADON | cr2::CONT | cr2::SWSTART,
        )
        .unwrap();
        adc.advance(&cx, 1_200);
        assert!(adc.converting());
        adc.advance(&cx, 1_200);
        assert!(adc.converting());
    }

    #[test]
    fn dma_request_tracks_eoc() {
        let (mut adc, cx) = adc();
        adc.write_reg(&cx, reg::CR2, AccessWidth::Word, cr2::ADON | cr2::DMA)
            .unwrap();
        adc.write_reg(
            &cx,
            reg::CR2,
            AccessWidth::Word,
            cr2::ADON | cr2::DMA | cr2::SWSTART,
        )
        .unwrap();
        adc.advance(&cx, 1_200);
        assert!(adc.dma_request());
        adc.read_reg(&cx, reg::DR, AccessWidth::Word).unwrap();
        assert!(!adc.dma_request());
    }

    #[test]
    fn calibration_completes_after_83_cycles() {
        let (mut adc, cx) = adc();
        adc.write_reg(&cx, reg::CR2, AccessWidth::Word, cr2::ADON | cr2::CAL)
            .unwrap();
        adc.advance(&cx, 1_000);
        assert_eq!(
            adc.read_reg(&cx, reg::SR, AccessWidth::Word).unwrap() & sr::EOC,
            0
        );
        adc.advance(&cx, 7_000);
        assert_ne!(
            adc.read_reg(&cx, reg::SR, AccessWidth::Word).unwrap() & sr::EOC,
            0
        );
    }
}
