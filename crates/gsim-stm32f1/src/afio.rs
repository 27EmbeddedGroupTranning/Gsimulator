//! Alternate function I/O (AFIO): remap control and EXTI line routing.

use super::map::afio as reg;
use gsim_core::bus::{extract, field, merge, AccessWidth, BusError};
use gsim_core::ctx::Ctx;

/// `MAPR` remap bit positions.
mod mapr {
    pub const SPI1_REMAP: u32 = 0;
    pub const I2C1_REMAP: u32 = 1;
    pub const USART1_REMAP: u32 = 2;
    pub const USART2_REMAP: u32 = 3;
    pub const USART3_REMAP: u32 = 4;
    pub const TIM1_REMAP: u32 = 6;
    pub const TIM2_REMAP: u32 = 8;
    pub const TIM3_REMAP: u32 = 10;
    pub const TIM4_REMAP: u32 = 12;
    pub const PD01_REMAP: u32 = 15;
    pub const SWJ_CFG: u32 = 24;
}

/// Alternate function remap and EXTI routing state.
#[derive(Debug, Clone, Default)]
pub struct Afio {
    evcr: u32,
    mapr: u32,
    swj_cfg: u32,
    exticr: [u32; 4],
}

impl Afio {
    pub fn new() -> Self {
        Afio::default()
    }

    pub fn reset(&mut self) {
        *self = Afio::new();
    }

    /// GPIO port index (`0` = A) that drives EXTI line `line`.
    pub fn exti_port(&self, line: u8) -> u8 {
        let index = usize::from(line / 4);
        let shift = u32::from(line % 4) * 4;
        field(self.exticr[index], shift, 4) as u8
    }

    pub fn exti_port_of(&self, port: u8, pin: u8) -> bool {
        self.exti_port(pin) == port
    }

    pub fn mapr(&self) -> u32 {
        self.mapr
    }

    pub fn spi1_remapped(&self) -> bool {
        self.mapr & (1 << mapr::SPI1_REMAP) != 0
    }

    pub fn i2c1_remapped(&self) -> bool {
        self.mapr & (1 << mapr::I2C1_REMAP) != 0
    }

    pub fn usart1_remapped(&self) -> bool {
        self.mapr & (1 << mapr::USART1_REMAP) != 0
    }

    pub fn usart2_remapped(&self) -> bool {
        self.mapr & (1 << mapr::USART2_REMAP) != 0
    }

    pub fn usart3_remap(&self) -> u32 {
        field(self.mapr, mapr::USART3_REMAP, 2)
    }

    pub fn tim1_remap(&self) -> u32 {
        field(self.mapr, mapr::TIM1_REMAP, 2)
    }

    pub fn tim2_remap(&self) -> u32 {
        field(self.mapr, mapr::TIM2_REMAP, 2)
    }

    pub fn tim3_remap(&self) -> u32 {
        field(self.mapr, mapr::TIM3_REMAP, 2)
    }

    pub fn tim4_remapped(&self) -> bool {
        self.mapr & (1 << mapr::TIM4_REMAP) != 0
    }

    pub fn pd01_remapped(&self) -> bool {
        self.mapr & (1 << mapr::PD01_REMAP) != 0
    }

    /// `SWJ_CFG`: 0 = full SWJ, 4 = JTAG-DP disabled, 7 = SWD disabled.
    ///
    /// The field is write only and reads back as zero, so it is kept in a
    /// separate latch.
    pub fn swj_cfg(&self) -> u32 {
        self.swj_cfg
    }

    pub fn read_reg(
        &mut self,
        _cx: &Ctx,
        offset: u32,
        width: AccessWidth,
    ) -> Result<u32, BusError> {
        let value = match offset & !3 {
            reg::EVCR => self.evcr,
            reg::MAPR => self.mapr,
            reg::EXTICR1 => self.exticr[0],
            reg::EXTICR2 => self.exticr[1],
            reg::EXTICR3 => self.exticr[2],
            reg::EXTICR4 => self.exticr[3],
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
            reg::EVCR => self.evcr = merge(self.evcr, offset, width, value) & 0x0000_00FF,
            reg::MAPR => {
                let merged = merge(self.mapr, offset, width, value);
                self.mapr = merged & 0x00FF_FFFF;
                self.swj_cfg = field(value, mapr::SWJ_CFG, 3);
            }
            reg::EXTICR1 => self.exticr[0] = merge(self.exticr[0], offset, width, value) & 0xFFFF,
            reg::EXTICR2 => self.exticr[1] = merge(self.exticr[1], offset, width, value) & 0xFFFF,
            reg::EXTICR3 => self.exticr[2] = merge(self.exticr[2], offset, width, value) & 0xFFFF,
            reg::EXTICR4 => self.exticr[3] = merge(self.exticr[3], offset, width, value) & 0xFFFF,
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

crate::impl_peripheral!(Afio, "AFIO");

#[cfg(test)]
mod tests {
    use super::*;
    use gsim_core::ctx::Clocks;

    fn ctx() -> Ctx {
        Ctx::new(0, Clocks::default())
    }

    #[test]
    fn exti_lines_are_routed_to_ports() {
        let cx = ctx();
        let mut afio = Afio::new();
        // EXTI0 -> port B (1), EXTI5 -> port C (2).
        afio.write_reg(&cx, reg::EXTICR1, AccessWidth::Word, 0b0001)
            .unwrap();
        afio.write_reg(&cx, reg::EXTICR2, AccessWidth::Word, 0b0010 << 4)
            .unwrap();
        assert_eq!(afio.exti_port(0), 1);
        assert_eq!(afio.exti_port(5), 2);
        assert_eq!(afio.exti_port(1), 0);
    }

    #[test]
    fn remap_bits_are_readable() {
        let cx = ctx();
        let mut afio = Afio::new();
        afio.write_reg(&cx, reg::MAPR, AccessWidth::Word, 1 << 2)
            .unwrap();
        assert!(afio.usart1_remapped());
        assert!(!afio.spi1_remapped());
        assert_eq!(afio.swj_cfg(), 0);
    }

    #[test]
    fn swj_configuration_is_not_retained_when_written_as_zero() {
        let cx = ctx();
        let mut afio = Afio::new();
        afio.write_reg(&cx, reg::MAPR, AccessWidth::Word, 4 << 24)
            .unwrap();
        assert_eq!(afio.swj_cfg(), 4);
        afio.write_reg(&cx, reg::MAPR, AccessWidth::Word, 0)
            .unwrap();
        assert_eq!(afio.swj_cfg(), 0);
    }
}

impl Afio {
    /// Labelled state for the peripheral status browser.
    pub fn status(&self, _clocks: &gsim_core::Clocks) -> Vec<(&'static str, String)> {
        vec![
            ("MAPR", crate::status::hex(self.mapr)),
            ("SPI1 重映射", if self.spi1_remapped() { "是" } else { "否" }.to_string()),
            ("I2C1 重映射", if self.i2c1_remapped() { "是" } else { "否" }.to_string()),
            ("USART1 重映射", if self.usart1_remapped() { "是" } else { "否" }.to_string()),
            ("SWJ_CFG", format!("0b{:03b}", self.swj_cfg)),
            ("EVCR", crate::status::hex(self.evcr)),
        ]
    }
}