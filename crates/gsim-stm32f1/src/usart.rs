//! Universal synchronous/asynchronous receiver transmitter (USART1..3).
//!
//! Only the asynchronous mode is modelled, but with real frame timing: a byte
//! occupies `frame_bits * BRR * 1/fCK` on the wire, `TC` is raised when the
//! shift register drains and `TXE` reflects the transmit buffer. Received
//! bytes are injected with [`Usart::receive_byte`], which is how the test
//! harness (and later an external device model) plays the role of the peer.

use super::map::usart as reg;
use gsim_core::bus::{extract, merge, AccessWidth, BusError};
use gsim_core::ctx::{Ctx, Event};
use gsim_core::irq::Irq;

/// `SR` bits.
pub mod sr {
    pub const PE: u32 = 1 << 0;
    pub const FE: u32 = 1 << 1;
    pub const NE: u32 = 1 << 2;
    pub const ORE: u32 = 1 << 3;
    pub const IDLE: u32 = 1 << 4;
    pub const RXNE: u32 = 1 << 5;
    pub const TC: u32 = 1 << 6;
    pub const TXE: u32 = 1 << 7;
}

/// `CR1` bits.
pub mod cr1 {
    pub const SBK: u32 = 1 << 0;
    pub const RWU: u32 = 1 << 1;
    pub const RE: u32 = 1 << 2;
    pub const TE: u32 = 1 << 3;
    pub const IDLEIE: u32 = 1 << 4;
    pub const RXNEIE: u32 = 1 << 5;
    pub const TCIE: u32 = 1 << 6;
    pub const TXEIE: u32 = 1 << 7;
    pub const PEIE: u32 = 1 << 8;
    pub const M: u32 = 1 << 12;
    pub const UE: u32 = 1 << 13;
}

/// `CR2` bits.
pub mod cr2 {
    pub const STOP: u32 = 0b11 << 12;
}

/// `CR3` bits.
pub mod cr3 {
    pub const DMA_RX: u32 = 1 << 6;
    pub const DMA_TX: u32 = 1 << 7;
}

/// What the shift register is currently doing.
#[derive(Debug, Clone, Copy)]
struct Shift {
    byte: u8,
    remaining_ns: u64,
}

/// One USART instance.
#[derive(Debug, Clone)]
pub struct Usart {
    index: u8,
    irq: Irq,
    brr: u32,
    cr1: u32,
    cr2: u32,
    cr3: u32,
    gtpr: u32,
    rx_data: u8,
    rxne: bool,
    overrun: bool,
    idle: bool,
    dr: u32,
    tx_buffer: Option<u8>,
    shift: Option<Shift>,
    tc: bool,
    sr_read_since_dr: bool,
    irq_high: bool,
}

impl Usart {
    pub fn new(index: u8, irq: Irq) -> Self {
        Usart {
            index,
            irq,
            brr: 0,
            cr1: 0,
            cr2: 0,
            cr3: 0,
            gtpr: 0,
            rx_data: 0,
            rxne: false,
            overrun: false,
            idle: false,
            dr: 0,
            tx_buffer: None,
            shift: None,
            tc: false,
            sr_read_since_dr: false,
            irq_high: false,
        }
    }

    pub fn reset(&mut self) {
        let (index, irq) = (self.index, self.irq);
        *self = Usart::new(index, irq);
    }

    /// Unit index (0 = USART1).
    pub fn index(&self) -> u8 {
        self.index
    }

    /// True when the transmitter is shifting a byte out.
    pub fn busy(&self) -> bool {
        self.shift.is_some()
    }

    fn word_bits(&self) -> u32 {
        if self.cr1 & cr1::M != 0 {
            9
        } else {
            8
        }
    }

    fn frame_bits(&self) -> u64 {
        let parity = if self.cr1 & (1 << 10) != 0 { 1 } else { 0 };
        let stop: u64 = if (self.cr2 >> 12) & 0b11 == 0b10 {
            2
        } else {
            1
        };
        u64::from(1 + self.word_bits() + parity) + stop
    }

    fn clock_hz(&self, cx: &Ctx) -> u32 {
        if self.index == 0 {
            cx.clocks().pclk2_hz
        } else {
            cx.clocks().pclk1_hz
        }
    }

    /// Nominal baud rate derived from `BRR` and the peripheral clock.
    pub fn baud(&self, pclk_hz: u32) -> u32 {
        if self.brr == 0 || pclk_hz == 0 {
            0
        } else {
            pclk_hz / self.brr
        }
    }

    fn frame_ns(&self, pclk_hz: u32) -> u64 {
        if pclk_hz == 0 || self.brr == 0 {
            return 0;
        }
        self.frame_bits() * u64::from(self.brr) * gsim_core::time::NANOS_PER_SEC
            / u64::from(pclk_hz)
    }

    /// Plays the peer: hands one received byte to the receiver.
    pub fn receive_byte(&mut self, cx: &Ctx, byte: u8) {
        if self.cr1 & cr1::RE == 0 || self.cr1 & cr1::UE == 0 {
            return;
        }
        if self.rxne {
            self.overrun = true;
        } else {
            self.rx_data = byte;
            self.dr = u32::from(byte);
            self.rxne = true;
        }
        self.refresh_irq(cx);
    }

    fn status_flags(&self) -> u32 {
        let mut value = 0;
        if self.overrun {
            value |= sr::ORE;
        }
        if self.idle {
            value |= sr::IDLE;
        }
        if self.rxne {
            value |= sr::RXNE;
        }
        if self.tc {
            value |= sr::TC;
        }
        if self.tx_buffer.is_none() {
            value |= sr::TXE;
        }
        value
    }

    fn should_irq(&self) -> bool {
        if self.cr1 & cr1::UE == 0 {
            return false;
        }
        let flags = self.status_flags();
        (self.cr1 & cr1::RXNEIE != 0 && flags & sr::RXNE != 0)
            || (self.cr1 & cr1::TXEIE != 0 && flags & sr::TXE != 0)
            || (self.cr1 & cr1::TCIE != 0 && flags & sr::TC != 0)
            || (self.cr1 & cr1::IDLEIE != 0 && flags & sr::IDLE != 0)
            || (self.cr1 & cr1::PEIE != 0 && flags & (sr::ORE | sr::NE | sr::FE | sr::PE) != 0)
    }

    fn refresh_irq(&mut self, cx: &Ctx) {
        let high = self.should_irq();
        if high != self.irq_high {
            self.irq_high = high;
            cx.irq(self.irq, high);
        }
    }

    fn start_next_frame(&mut self, cx: &Ctx) {
        if self.shift.is_some() {
            return;
        }
        if let Some(byte) = self.tx_buffer.take() {
            if self.cr1 & cr1::TE == 0 || self.cr1 & cr1::UE == 0 {
                return;
            }
            let duration = self.frame_ns(self.clock_hz(cx));
            self.tc = false;
            if duration == 0 {
                self.complete_frame(cx, byte);
            } else {
                self.shift = Some(Shift {
                    byte,
                    remaining_ns: duration,
                });
            }
        }
    }

    fn complete_frame(&mut self, cx: &Ctx, byte: u8) {
        cx.emit(Event::UartTx {
            unit: self.index,
            byte,
        });
        self.tc = self.tx_buffer.is_none();
        self.refresh_irq(cx);
    }

    pub fn advance(&mut self, cx: &Ctx, elapsed_ns: u64) {
        if let Some(mut shift) = self.shift {
            if elapsed_ns >= shift.remaining_ns {
                self.shift = None;
                self.complete_frame(cx, shift.byte);
                self.start_next_frame(cx);
            } else {
                shift.remaining_ns -= elapsed_ns;
                self.shift = Some(shift);
            }
        } else {
            self.start_next_frame(cx);
        }
        self.refresh_irq(cx);
    }

    pub fn read_reg(&mut self, cx: &Ctx, offset: u32, width: AccessWidth) -> Result<u32, BusError> {
        let value = match offset & !3 {
            reg::SR => {
                self.sr_read_since_dr = true;
                self.status_flags()
            }
            reg::DR => {
                let value = self.dr & 0x1FF;
                self.dr = 0;
                self.rxne = false;
                self.idle = false;
                if self.sr_read_since_dr {
                    self.overrun = false;
                }
                self.sr_read_since_dr = false;
                self.refresh_irq(cx);
                value
            }
            reg::BRR => self.brr,
            reg::CR1 => self.cr1,
            reg::CR2 => self.cr2,
            reg::CR3 => self.cr3,
            reg::GTPR => self.gtpr,
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
            reg::SR => {}
            reg::DR => {
                let byte = (value & 0xFF) as u8;
                self.dr = value & 0x1FF;
                self.tx_buffer = Some(byte);
                if self.sr_read_since_dr {
                    self.tc = false;
                }
                self.sr_read_since_dr = false;
                self.start_next_frame(cx);
            }
            reg::BRR => self.brr = merge(self.brr, offset, width, value) & 0xFFFF,
            reg::CR1 => self.cr1 = merge(self.cr1, offset, width, value) & 0x3FFF,
            reg::CR2 => self.cr2 = merge(self.cr2, offset, width, value) & 0x7F7F,
            reg::CR3 => self.cr3 = merge(self.cr3, offset, width, value) & 0x07FF,
            reg::GTPR => self.gtpr = merge(self.gtpr, offset, width, value) & 0xFFFF,
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

    /// True when the DMA receive request line is asserted.
    pub fn dma_rx_request(&self) -> bool {
        self.cr3 & cr3::DMA_RX != 0 && self.rxne
    }

    /// True when the DMA transmit request line is asserted.
    pub fn dma_tx_request(&self) -> bool {
        self.cr3 & cr3::DMA_TX != 0 && self.tx_buffer.is_none() && self.cr1 & cr1::TE != 0
    }
}

crate::impl_peripheral!(Usart, "USART");

impl Usart {
    /// Labelled state for the peripheral status browser.
    pub fn status(&self, clocks: &gsim_core::Clocks) -> Vec<(&'static str, String)> {
        let pclk = if self.index == 0 {
            clocks.pclk2_hz
        } else {
            clocks.pclk1_hz
        };
        let baud = if self.brr >= 16 { pclk / self.brr } else { 0 };
        let mut flags = Vec::new();
        if self.tx_buffer.is_some() || self.shift.is_some() {
            flags.push("发送中");
        }
        if self.tc {
            flags.push("发送完成");
        }
        if self.rxne {
            flags.push("收到字节");
        }
        if self.overrun {
            flags.push("溢出");
        }
        vec![
            (
                "使能",
                crate::status::on_off(self.cr1 & (1 << 13) != 0).to_string(),
            ),
            (
                "发送",
                crate::status::on_off(self.cr1 & (1 << 3) != 0).to_string(),
            ),
            (
                "接收",
                crate::status::on_off(self.cr1 & (1 << 2) != 0).to_string(),
            ),
            ("波特率", crate::status::hz(baud)),
            ("BRR", crate::status::hex(self.brr)),
            (
                "状态",
                if flags.is_empty() {
                    "空闲".to_string()
                } else {
                    flags.join("，")
                },
            ),
            ("最近接收", format!("0x{:02X}", self.rx_data)),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use gsim_core::ctx::Clocks;
    use gsim_core::irq::lines;

    fn ctx(pclk2: u32) -> Ctx {
        Ctx::new(
            0,
            Clocks {
                pclk2_hz: pclk2,
                pclk1_hz: pclk2,
                ..Clocks::HSI_8MHZ
            },
        )
    }

    fn configured(pclk: u32) -> (Usart, Ctx) {
        let cx = ctx(pclk);
        let mut uart = Usart::new(0, lines::USART1);
        // 8N1, TE | RE | UE, 115200 baud at 72 MHz -> BRR = 625.
        uart.write_reg(&cx, reg::BRR, AccessWidth::Word, 625)
            .unwrap();
        uart.write_reg(
            &cx,
            reg::CR1,
            AccessWidth::Word,
            cr1::TE | cr1::RE | cr1::UE,
        )
        .unwrap();
        (uart, cx)
    }

    #[test]
    fn baud_rate_matches_brr() {
        let (uart, cx) = configured(72_000_000);
        assert_eq!(uart.baud(cx.clocks().pclk2_hz), 115_200);
        assert_eq!(uart.frame_bits(), 10);
    }

    #[test]
    fn transmission_sets_txe_then_raises_tc() {
        let (mut uart, cx) = configured(72_000_000);
        assert_eq!(
            uart.read_reg(&cx, reg::SR, AccessWidth::Word).unwrap() & sr::TXE,
            sr::TXE
        );
        cx.take_events();
        uart.write_reg(&cx, reg::DR, AccessWidth::Word, u32::from(b'H'))
            .unwrap();
        assert!(uart.busy());
        // 10 bits at 115200 baud is ~86.8 us.
        uart.advance(&cx, 86_000);
        assert!(uart.busy());
        uart.advance(&cx, 1_000);
        assert!(!uart.busy());
        assert_eq!(
            cx.take_events(),
            vec![Event::UartTx {
                unit: 0,
                byte: b'H'
            }]
        );
        let status = uart.read_reg(&cx, reg::SR, AccessWidth::Word).unwrap();
        assert_eq!(status & sr::TC, sr::TC);
        assert_eq!(status & sr::TXE, sr::TXE);
    }

    #[test]
    fn received_byte_sets_rxne_and_raises_irq() {
        let (mut uart, cx) = configured(72_000_000);
        uart.write_reg(
            &cx,
            reg::CR1,
            AccessWidth::Word,
            cr1::TE | cr1::RE | cr1::UE | cr1::RXNEIE,
        )
        .unwrap();
        cx.take_events();
        uart.receive_byte(&cx, b'K');
        assert_eq!(
            cx.take_events(),
            vec![Event::Irq {
                irq: lines::USART1,
                high: true
            }]
        );
        let status = uart.read_reg(&cx, reg::SR, AccessWidth::Word).unwrap();
        assert_eq!(status & sr::RXNE, sr::RXNE);
        let data = uart.read_reg(&cx, reg::DR, AccessWidth::Word).unwrap();
        assert_eq!(data, u32::from(b'K'));
        let status = uart.read_reg(&cx, reg::SR, AccessWidth::Word).unwrap();
        assert_eq!(status & sr::RXNE, 0);
    }

    #[test]
    fn overrun_is_flagged_when_rxne_is_not_serviced() {
        let (mut uart, cx) = configured(72_000_000);
        uart.receive_byte(&cx, b'1');
        uart.receive_byte(&cx, b'2');
        let status = uart.read_reg(&cx, reg::SR, AccessWidth::Word).unwrap();
        assert_eq!(status & sr::ORE, sr::ORE);
        assert_eq!(
            uart.read_reg(&cx, reg::DR, AccessWidth::Word).unwrap(),
            u32::from(b'1')
        );
    }

    #[test]
    fn disabled_peripheral_ignores_injected_bytes() {
        let cx = ctx(72_000_000);
        let mut uart = Usart::new(0, lines::USART1);
        uart.receive_byte(&cx, b'x');
        assert_eq!(
            uart.read_reg(&cx, reg::SR, AccessWidth::Word).unwrap() & sr::RXNE,
            0
        );
    }
}
