//! Direct memory access controller (DMA1/DMA2).
//!
//! The controller is split into a transfer engine (`next_transfer` /
//! [`Dma::complete`]) and the bus accesses themselves, which the SoC performs.
//! That keeps the DMA model free of bus references while still producing
//! realistic behaviour: priorities, increment modes, circular buffers,
//! half/complete transfer flags and per channel interrupts.

use super::map::dma1 as reg;
use gsim_core::bus::{extract, field, merge, with_field, AccessWidth, BusError};
use gsim_core::ctx::Ctx;
use gsim_core::irq::{lines, Irq};

/// `CCR` bits.
pub mod ccr {
    pub const EN: u32 = 1 << 0;
    pub const TCIE: u32 = 1 << 1;
    pub const HTIE: u32 = 1 << 2;
    pub const TEIE: u32 = 1 << 3;
    pub const DIR: u32 = 1 << 4;
    pub const CIRC: u32 = 1 << 5;
    pub const PINC: u32 = 1 << 6;
    pub const MINC: u32 = 1 << 7;
    pub const PSIZE: u32 = 0b11 << 8;
    pub const MSIZE: u32 = 0b11 << 10;
    pub const PL: u32 = 0b11 << 12;
    pub const MEM2MEM: u32 = 1 << 14;
}

/// Interrupt flag bits inside `ISR`/`IFCR` for one channel.
const GIF: u32 = 1 << 0;
const TCIF: u32 = 1 << 1;
const HTIF: u32 = 1 << 2;

/// One DMA channel.
#[derive(Debug, Clone, Default)]
pub struct Channel {
    pub ccr: u32,
    pub cndtr: u32,
    pub cpar: u32,
    pub cmar: u32,
    /// Number of transfers already completed in the current window.
    done: u32,
    /// Latched interrupt flags: GIF | TCIF | HTIF.
    flags: u32,
}

/// A single transfer the SoC has to perform on the bus.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Transfer {
    pub channel: usize,
    pub source: u32,
    pub destination: u32,
    pub width: AccessWidth,
}

/// Which peripheral request lines reach a channel.
pub type RequestMap = [bool; 7];

/// DMA controller with `n` channels.
#[derive(Debug, Clone)]
pub struct Dma {
    channels: Vec<Channel>,
    base_irq: u16,
}

impl Dma {
    /// Creates a controller whose channel `n` interrupt is `base_irq + n`.
    pub fn new(channel_count: usize, base_irq: u16) -> Self {
        Dma {
            channels: vec![Channel::default(); channel_count],
            base_irq,
        }
    }

    pub fn reset(&mut self) {
        let (count, base) = (self.channels.len(), self.base_irq);
        *self = Dma::new(count, base);
    }

    pub fn channel_count(&self) -> usize {
        self.channels.len()
    }

    pub fn channel(&self, index: usize) -> Option<&Channel> {
        self.channels.get(index)
    }

    /// Number of transfers still to perform.
    pub fn remaining(&self, index: usize) -> u32 {
        self.channels.get(index).map_or(0, |c| c.cndtr)
    }

    /// True when at least one channel is enabled.
    pub fn any_enabled(&self) -> bool {
        self.channels.iter().any(|c| c.ccr & ccr::EN != 0)
    }

    /// Interrupt line of a channel.
    pub fn channel_irq(&self, index: usize) -> Irq {
        Irq::new(self.base_irq + index as u16)
    }

    /// Selects the next transfer to run, honouring software priority and the
    /// peripheral request lines.
    pub fn next_transfer(&self, requests: &RequestMap) -> Option<Transfer> {
        let mut best: Option<(u32, u32, usize)> = None;
        for (index, channel) in self.channels.iter().enumerate() {
            if channel.ccr & ccr::EN == 0 || channel.cndtr == 0 {
                continue;
            }
            let requested = requests.get(index).copied().unwrap_or(false);
            if !requested && channel.ccr & ccr::MEM2MEM == 0 {
                continue;
            }
            let priority = 3 - field(channel.ccr, 12, 2);
            let key = (priority, index as u32);
            if best.map_or(true, |(p, i, _)| key < (p, i)) {
                best = Some((key.0, key.1, index));
            }
        }
        best.map(|(_, _, index)| {
            let channel = &self.channels[index];
            let width = match field(channel.ccr, 8, 2) {
                0 => AccessWidth::Byte,
                1 => AccessWidth::Half,
                _ => AccessWidth::Word,
            };
            let memory_to_peripheral = channel.ccr & ccr::DIR != 0;
            if memory_to_peripheral {
                Transfer {
                    channel: index,
                    source: channel.cmar,
                    destination: channel.cpar,
                    width,
                }
            } else {
                Transfer {
                    channel: index,
                    source: channel.cpar,
                    destination: channel.cmar,
                    width,
                }
            }
        })
    }

    /// Books a completed transfer: pointers, counter and flags.
    pub fn complete(&mut self, cx: &Ctx, transfer: Transfer) {
        let irq = self.channel_irq(transfer.channel);
        let Some(channel) = self.channels.get_mut(transfer.channel) else {
            return;
        };
        let pinc = channel.ccr & ccr::PINC != 0;
        let minc = channel.ccr & ccr::MINC != 0;
        let memory_to_peripheral = channel.ccr & ccr::DIR != 0;
        let step = transfer.width.bytes();

        if memory_to_peripheral {
            if minc {
                channel.cmar = channel.cmar.wrapping_add(step);
            }
            if pinc {
                channel.cpar = channel.cpar.wrapping_add(step);
            }
        } else {
            if pinc {
                channel.cpar = channel.cpar.wrapping_add(step);
            }
            if minc {
                channel.cmar = channel.cmar.wrapping_add(step);
            }
        }

        channel.done += 1;
        channel.cndtr = channel.cndtr.saturating_sub(1);
        let total = channel.done + channel.cndtr;
        if total > 0 && channel.done == total / 2 {
            channel.flags |= HTIF;
            if channel.ccr & ccr::HTIE != 0 {
                cx.irq(irq, true);
            }
        }
        if channel.cndtr == 0 {
            channel.flags |= TCIF | GIF;
            if channel.ccr & ccr::CIRC != 0 {
                channel.cndtr = channel.done;
                channel.done = 0;
            } else {
                channel.ccr &= !ccr::EN;
            }
            if channel.ccr & ccr::TCIE != 0 || channel.ccr & ccr::CIRC != 0 {
                cx.irq(irq, true);
            }
        }
    }

    /// Clears the channel interrupt line once firmware serviced it.
    pub fn refresh_irq(&self, cx: &Ctx, index: usize) {
        let Some(channel) = self.channels.get(index) else {
            return;
        };
        let high =
            (channel.flags & (TCIF | HTIF) != 0) && (channel.ccr & (ccr::TCIE | ccr::HTIE) != 0);
        cx.irq(self.channel_irq(index), high);
    }

    fn isr(&self) -> u32 {
        let mut value = 0u32;
        for (index, channel) in self.channels.iter().enumerate() {
            let shift = (index as u32) * 4;
            value |= (channel.flags & 0b111) << shift;
        }
        value
    }

    pub fn read_reg(
        &mut self,
        _cx: &Ctx,
        offset: u32,
        width: AccessWidth,
    ) -> Result<u32, BusError> {
        let value = if (offset & !3) == reg::ISR {
            self.isr()
        } else if (offset & !3) == reg::IFCR {
            0
        } else if offset >= 0x08 {
            let index = ((offset - 0x08) / 0x14) as usize;
            let register = ((offset - 0x08) % 0x14) & !3;
            let Some(channel) = self.channels.get(index) else {
                return Err(BusError::Unmapped {
                    addr: offset,
                    width,
                    write: false,
                });
            };
            match register {
                reg::CCR => channel.ccr,
                reg::CNDTR => channel.cndtr,
                reg::CPAR => channel.cpar,
                reg::CMAR => channel.cmar,
                _ => {
                    return Err(BusError::Unmapped {
                        addr: offset,
                        width,
                        write: false,
                    })
                }
            }
        } else {
            return Err(BusError::Unmapped {
                addr: offset,
                width,
                write: false,
            });
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
        let aligned = offset & !3;
        if aligned == reg::IFCR {
            let clear = value & 0x0FFF_FFFF;
            for index in 0..self.channels.len() {
                let shift = (index as u32) * 4;
                let bits = (clear >> shift) & 0b111;
                self.channels[index].flags &= !bits;
                self.refresh_irq(cx, index);
            }
            return Ok(());
        }
        if aligned == reg::ISR {
            return Ok(());
        }
        if offset < 0x08 {
            return Err(BusError::Unmapped {
                addr: offset,
                width,
                write: true,
            });
        }
        let index = ((offset - 0x08) / 0x14) as usize;
        let register = ((offset - 0x08) % 0x14) & !3;
        let lane = offset & 3;
        let Some(channel) = self.channels.get_mut(index) else {
            return Err(BusError::Unmapped {
                addr: offset,
                width,
                write: true,
            });
        };
        match register {
            reg::CCR => {
                let merged = merge(channel.ccr, lane, width, value) & 0x7FFF;
                let was_enabled = channel.ccr & ccr::EN != 0;
                channel.ccr = merged;
                if merged & ccr::EN != 0 && !was_enabled {
                    channel.done = 0;
                }
            }
            reg::CNDTR => channel.cndtr = merge(channel.cndtr, lane, width, value) & 0xFFFF,
            reg::CPAR => channel.cpar = merge(channel.cpar, lane, width, value),
            reg::CMAR => channel.cmar = merge(channel.cmar, lane, width, value),
            _ => {
                return Err(BusError::Unmapped {
                    addr: offset,
                    width,
                    write: true,
                })
            }
        }
        self.refresh_irq(cx, index);
        Ok(())
    }

    pub fn advance(&mut self, _cx: &Ctx, _elapsed_ns: u64) {}

    /// Sets a channel's software priority, used by the demo to show ordering.
    pub fn set_priority(&mut self, index: usize, priority: u32) {
        if let Some(channel) = self.channels.get_mut(index) {
            channel.ccr = with_field(channel.ccr, 12, 2, priority & 0b11);
        }
    }
}

crate::impl_peripheral!(Dma, "DMA");

/// DMA request source, following RM0008 "DMA1 request mapping".
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Request {
    Adc1,
    Adc2,
    Spi1Rx,
    Spi1Tx,
    Spi2Rx,
    Spi2Tx,
    Usart1Rx,
    Usart1Tx,
    Usart2Rx,
    Usart2Tx,
    Usart3Rx,
    Usart3Tx,
    I2c1Rx,
    I2c1Tx,
    I2c2Rx,
    I2c2Tx,
    Tim1Ch1,
    Tim1Ch2,
    Tim1Ch3,
    Tim1Ch4,
    Tim1Up,
    Tim1Trig,
    Tim1Com,
    Tim2Ch1,
    Tim2Ch2,
    Tim2Ch3,
    Tim2Ch4,
    Tim2Up,
    Tim3Ch1,
    Tim3Ch3,
    Tim3Ch4,
    Tim3Up,
    Tim4Ch1,
    Tim4Ch2,
    Tim4Ch3,
    Tim4Up,
}

impl Request {
    /// Controller index (0 = DMA1) and channel index (0 based).
    pub fn route(self) -> (u8, u8) {
        use Request::*;
        match self {
            Adc1 | Tim2Ch3 | Tim4Ch1 => (0, 0),
            Spi1Rx | Usart3Tx | Tim1Ch1 | Tim2Up | Tim3Ch3 => (0, 1),
            Spi1Tx | Usart3Rx | Tim1Ch2 | Tim3Ch4 | Tim3Up => (0, 2),
            Spi2Rx | Usart1Tx | I2c2Tx | Tim1Ch4 | Tim1Trig | Tim1Com | Tim4Ch2 => (0, 3),
            Spi2Tx | Usart1Rx | I2c2Rx | Tim1Up | Tim2Ch1 | Tim4Ch3 => (0, 4),
            Usart2Rx | I2c1Tx | Tim1Ch3 | Tim3Ch1 => (0, 5),
            Usart2Tx | I2c1Rx | Tim2Ch2 | Tim2Ch4 | Tim4Up => (0, 6),
            // ADC2 shares the ADC1 channel; the second controller has no
            // phase 1 sources.
            Adc2 => (0, 0),
        }
    }

    /// All request sources, in a stable order.
    pub const ALL: &'static [Request] = &[
        Request::Adc1,
        Request::Adc2,
        Request::Spi1Rx,
        Request::Spi1Tx,
        Request::Spi2Rx,
        Request::Spi2Tx,
        Request::Usart1Rx,
        Request::Usart1Tx,
        Request::Usart2Rx,
        Request::Usart2Tx,
        Request::Usart3Rx,
        Request::Usart3Tx,
        Request::I2c1Rx,
        Request::I2c1Tx,
        Request::I2c2Rx,
        Request::I2c2Tx,
        Request::Tim1Ch1,
        Request::Tim1Ch2,
        Request::Tim1Ch3,
        Request::Tim1Ch4,
        Request::Tim1Up,
        Request::Tim1Trig,
        Request::Tim1Com,
        Request::Tim2Ch1,
        Request::Tim2Ch2,
        Request::Tim2Ch3,
        Request::Tim2Ch4,
        Request::Tim2Up,
        Request::Tim3Ch1,
        Request::Tim3Ch3,
        Request::Tim3Ch4,
        Request::Tim3Up,
        Request::Tim4Ch1,
        Request::Tim4Ch2,
        Request::Tim4Ch3,
        Request::Tim4Up,
    ];
}

/// Interrupt line of the first channel of each controller.
pub fn base_irq(controller: u8) -> u16 {
    match controller {
        0 => lines::DMA1_CHANNEL1.number(),
        _ => lines::DMA2_CHANNEL1.number(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use gsim_core::ctx::Clocks;

    fn ctx() -> Ctx {
        Ctx::new(0, Clocks::default())
    }

    fn engine() -> Dma {
        Dma::new(7, lines::DMA1_CHANNEL1.number())
    }

    #[test]
    fn only_enabled_channels_with_requests_are_served() {
        let cx = ctx();
        let mut dma = engine();
        dma.write_reg(&cx, 0x08, AccessWidth::Word, ccr::EN)
            .unwrap();
        dma.write_reg(&cx, 0x0C, AccessWidth::Word, 4).unwrap();
        dma.write_reg(&cx, 0x10, AccessWidth::Word, 0x4001_2404)
            .unwrap();
        dma.write_reg(&cx, 0x14, AccessWidth::Word, 0x2000_0000)
            .unwrap();

        assert!(dma.next_transfer(&[false; 7]).is_none());
        let transfer = dma.next_transfer(&[true, false, false, false, false, false, false]);
        assert_eq!(
            transfer,
            Some(Transfer {
                channel: 0,
                source: 0x4001_2404,
                destination: 0x2000_0000,
                width: AccessWidth::Byte,
            })
        );
    }

    #[test]
    fn circular_channel_reloads_and_raises_tc() {
        let cx = ctx();
        let mut dma = engine();
        dma.write_reg(
            &cx,
            0x08,
            AccessWidth::Word,
            ccr::EN | ccr::CIRC | ccr::MINC,
        )
        .unwrap();
        dma.write_reg(&cx, 0x0C, AccessWidth::Word, 2).unwrap();
        dma.write_reg(&cx, 0x10, AccessWidth::Word, 0x4001_244C)
            .unwrap();
        dma.write_reg(&cx, 0x14, AccessWidth::Word, 0x2000_0000)
            .unwrap();
        let requests = [true, false, false, false, false, false, false];

        let transfer = dma.next_transfer(&requests).unwrap();
        dma.complete(&cx, transfer);
        assert_eq!(dma.remaining(0), 1);
        let transfer = dma.next_transfer(&requests).unwrap();
        assert_eq!(
            transfer.destination, 0x2000_0001,
            "MINC advanced the pointer"
        );
        dma.complete(&cx, transfer);
        assert_eq!(dma.remaining(0), 2, "circular mode reloads CNDTR");
        assert_ne!(
            dma.read_reg(&cx, 0x00, AccessWidth::Word).unwrap() & TCIF,
            0
        );
    }

    #[test]
    fn memory_to_memory_channel_runs_without_requests() {
        let cx = ctx();
        let mut dma = engine();
        dma.write_reg(
            &cx,
            0x08,
            AccessWidth::Word,
            ccr::EN | ccr::MEM2MEM | ccr::MINC,
        )
        .unwrap();
        dma.write_reg(&cx, 0x0C, AccessWidth::Word, 3).unwrap();
        dma.write_reg(&cx, 0x10, AccessWidth::Word, 0x2000_0100)
            .unwrap();
        dma.write_reg(&cx, 0x14, AccessWidth::Word, 0x2000_0000)
            .unwrap();
        let transfer = dma.next_transfer(&[false; 7]).unwrap();
        assert_eq!(transfer.source, 0x2000_0100);
        assert_eq!(transfer.destination, 0x2000_0000);
    }

    #[test]
    fn higher_software_priority_wins() {
        let cx = ctx();
        let mut dma = engine();
        // Channel 0 keeps the default (low) priority, channel 1 is very high.
        dma.write_reg(&cx, 0x08, AccessWidth::Word, ccr::EN)
            .unwrap();
        dma.write_reg(&cx, 0x0C, AccessWidth::Word, 1).unwrap();
        dma.write_reg(&cx, 0x1C, AccessWidth::Word, ccr::EN | (3 << 12))
            .unwrap();
        dma.write_reg(&cx, 0x20, AccessWidth::Word, 1).unwrap();
        let transfer = dma.next_transfer(&[true; 7]).unwrap();
        assert_eq!(transfer.channel, 1);
    }

    #[test]
    fn ifcr_clears_flags() {
        let cx = ctx();
        let mut dma = engine();
        dma.write_reg(&cx, 0x08, AccessWidth::Word, ccr::EN | ccr::MINC)
            .unwrap();
        dma.write_reg(&cx, 0x0C, AccessWidth::Word, 1).unwrap();
        let transfer = dma.next_transfer(&[true; 7]).unwrap();
        dma.complete(&cx, transfer);
        assert_ne!(
            dma.read_reg(&cx, 0x00, AccessWidth::Word).unwrap() & TCIF,
            0
        );
        dma.write_reg(&cx, 0x04, AccessWidth::Word, TCIF | GIF)
            .unwrap();
        assert_eq!(
            dma.read_reg(&cx, 0x00, AccessWidth::Word).unwrap() & TCIF,
            0
        );
    }

    #[test]
    fn request_routing_matches_the_reference_manual() {
        assert_eq!(Request::Adc1.route(), (0, 0));
        assert_eq!(Request::Usart1Tx.route(), (0, 3));
        assert_eq!(Request::Usart1Rx.route(), (0, 4));
        assert_eq!(Request::Spi1Rx.route(), (0, 1));
        assert_eq!(Request::Spi1Tx.route(), (0, 2));
        assert_eq!(Request::Tim2Up.route(), (0, 1));
        assert_eq!(Request::I2c1Tx.route(), (0, 5));
    }
}

impl Dma {
    /// Labelled state for the peripheral status browser.
    pub fn status(&self, _clocks: &gsim_core::Clocks) -> Vec<(&'static str, String)> {
        let enabled = self
            .channels
            .iter()
            .filter(|channel| channel.ccr & 1 != 0)
            .count();
        let mut rows = vec![
            ("通道数", format!("{}", self.channels.len())),
            ("已使能", format!("{enabled}")),
        ];
        for (index, channel) in self.channels.iter().enumerate() {
            if channel.ccr & 1 == 0 {
                continue;
            }
            const NAMES: [&str; 7] = [
                "通道 1", "通道 2", "通道 3", "通道 4", "通道 5", "通道 6", "通道 7",
            ];
            let name = NAMES.get(index).copied().unwrap_or("通道");
            rows.push((
                name,
                format!(
                    "剩余 {} 次  0x{:08X} → 0x{:08X}",
                    channel.cndtr, channel.cpar, channel.cmar
                ),
            ));
        }
        rows
    }
}