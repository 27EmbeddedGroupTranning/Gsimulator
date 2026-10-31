//! Live peripheral state for the front end's status browser.
//!
//! Every block exposes a `status()` list of labelled values; this module glues
//! them together with the address map and the clock gating so a UI can show
//! "what is this peripheral doing right now" for the whole chip in one call.
//! The labels are user facing strings, the values are pre-formatted so the
//! front end does not need to know any register layout.

use crate::map;
use crate::soc::Soc;

/// Everything the browser shows for one peripheral.
#[derive(Clone, Debug, Default)]
pub struct PeripheralStatus {
    /// Instance name, for example `USART1`.
    pub name: String,
    /// Group shown next to the name, for example `串口`.
    pub kind: &'static str,
    pub base: u32,
    /// Clock feeding the block, `0` when the gate is closed.
    pub clock_hz: u32,
    pub rows: Vec<(&'static str, String)>,
}

/// `12.000 MHz` style clock text.
pub fn hz(value: u32) -> String {
    if value >= 1_000_000 {
        format!("{:.3} MHz", f64::from(value) / 1e6)
    } else if value >= 1_000 {
        format!("{:.1} kHz", f64::from(value) / 1e3)
    } else {
        format!("{value} Hz")
    }
}

/// `0x4001_3800` style register text.
pub fn hex(value: u32) -> String {
    format!("0x{value:08X}")
}

/// Chinese label of a boolean state.
pub fn on_off(value: bool) -> &'static str {
    if value {
        "开"
    } else {
        "关"
    }
}

impl Soc {
    /// Snapshot of every modelled peripheral, in address order.
    pub fn peripheral_status(&self) -> Vec<PeripheralStatus> {
        let clocks = self.clocks();
        let mut list = Vec::new();
        let mut push = |name: &str,
                        kind: &'static str,
                        base: u32,
                        clock_hz: u32,
                        rows: Vec<(&'static str, String)>| {
            list.push(PeripheralStatus {
                name: name.to_string(),
                kind,
                base,
                clock_hz,
                rows,
            });
        };

        push(
            "RCC",
            "时钟",
            map::rcc::BASE,
            clocks.hclk_hz,
            self.rcc.status(),
        );
        for (index, port) in self.gpio.iter().enumerate() {
            push(
                &format!("GPIO{}", crate::gpio::PORT_NAMES[index]),
                "通用输入输出",
                map::gpio::PORTS[index],
                self.peripheral_clock(map::gpio::PORTS[index]),
                port.status(&clocks),
            );
        }
        push(
            "AFIO",
            "复用功能",
            map::afio::BASE,
            self.peripheral_clock(map::afio::BASE),
            self.afio.status(&clocks),
        );
        push(
            "EXTI",
            "外部中断",
            map::exti::BASE,
            clocks.pclk2_hz,
            self.exti.status(&clocks),
        );
        for (index, usart) in self.usart.iter().enumerate() {
            push(
                &format!("USART{}", index + 1),
                "串口",
                map::usart::BASES[index],
                self.peripheral_clock(map::usart::BASES[index]),
                usart.status(&clocks),
            );
        }
        for (index, spi) in self.spi.iter().enumerate() {
            push(
                &format!("SPI{}", index + 1),
                "SPI",
                map::spi::BASES[index],
                self.peripheral_clock(map::spi::BASES[index]),
                spi.status(&clocks),
            );
        }
        for (index, i2c) in self.i2c.iter().enumerate() {
            push(
                &format!("I2C{}", index + 1),
                "I2C",
                map::i2c::BASES[index],
                self.peripheral_clock(map::i2c::BASES[index]),
                i2c.status(&clocks),
            );
        }
        push(
            "CAN1",
            "CAN",
            map::can::BASE_1,
            self.peripheral_clock(map::can::BASE_1),
            self.can.status(&clocks),
        );
        for (index, tim) in self.tim.iter().enumerate() {
            push(
                &format!("TIM{}", index + 1),
                "定时器",
                map::tim::BASES[index],
                self.peripheral_clock(map::tim::BASES[index]),
                tim.status(&clocks),
            );
        }
        for (index, adc) in self.adc.iter().enumerate() {
            push(
                &format!("ADC{}", index + 1),
                "模数转换",
                map::adc::BASES[index],
                self.peripheral_clock(map::adc::BASES[index]),
                adc.status(&clocks),
            );
        }
        for (index, base) in [map::dma1::BASE, map::dma2::BASE].iter().enumerate() {
            let dma = if index == 0 { &self.dma1 } else { &self.dma2 };
            push(
                &format!("DMA{}", index + 1),
                "直接内存访问",
                *base,
                self.peripheral_clock(*base),
                dma.status(&clocks),
            );
        }
        push(
            "FLASH",
            "闪存接口",
            map::flash::BASE,
            clocks.hclk_hz,
            self.flash_if.status(&clocks),
        );
        push(
            "PWR",
            "电源控制",
            map::pwr::BASE,
            self.peripheral_clock(map::pwr::BASE),
            self.pwr.status(&clocks),
        );
        push(
            "BKP",
            "备份寄存器",
            map::bkp::BASE,
            self.peripheral_clock(map::bkp::BASE),
            self.bkp.status(&clocks),
        );
        push(
            "RTC",
            "实时时钟",
            map::rtc::BASE,
            self.peripheral_clock(map::rtc::BASE),
            self.rtc.status(&clocks),
        );
        push(
            "IWDG",
            "独立看门狗",
            map::iwdg::BASE,
            self.peripheral_clock(map::iwdg::BASE),
            self.iwdg.status(&clocks),
        );
        push(
            "WWDG",
            "窗口看门狗",
            map::wwdg::BASE,
            self.peripheral_clock(map::wwdg::BASE),
            self.wwdg.status(&clocks),
        );
        push(
            "CRC",
            "校验单元",
            map::crc::BASE,
            self.peripheral_clock(map::crc::BASE),
            self.crc.status(&clocks),
        );
        push(
            "DBGMCU",
            "调试支持",
            map::dbgmcu::BASE,
            clocks.hclk_hz,
            self.dbgmcu.status(&clocks),
        );
        push(
            "NVIC",
            "中断控制器",
            0xE000_E100,
            clocks.hclk_hz,
            self.nvic.status(&clocks),
        );
        push(
            "SCB",
            "系统控制",
            0xE000_ED00,
            clocks.hclk_hz,
            self.scb.status(&clocks),
        );
        push(
            "SysTick",
            "系统节拍",
            0xE000_E010,
            clocks.hclk_hz,
            self.systick.status(&clocks),
        );
        list
    }
}
