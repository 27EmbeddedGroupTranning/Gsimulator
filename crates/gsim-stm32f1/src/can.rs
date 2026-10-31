//! bxCAN controller (CAN1, RM0008 chapter 24).
//!
//! The model covers what a firmware drives through the HAL: initialisation
//! mode with `INAK`/`SLAK` handshakes, the bit timing register, the three
//! transmit mailboxes with their completion flags, the two receive FIFOs, the
//! fourteen acceptance filter banks (32 and 16 bit, list and mask mode) and the
//! four interrupt lines.
//!
//! Frames cross the block boundary only through [`gsim_core::Event`]:
//!
//! * the controller reports a frame it put on its transmit pin as
//!   [`CanDirection::Tx`] (a transceiver model turns that into bus traffic),
//! * a frame it accepted from the bus into a FIFO as [`CanDirection::Rx`],
//! * a self reception in loopback mode as [`CanDirection::Loopback`].
//!
//! Incoming bus traffic is injected with [`Can::receive`], which is what the
//! external transceiver model calls for every frame the bus delivers.
//!
//! Simplifications, all documented in the README: a transmitted frame is
//! always acknowledged (there is no bit level arbitration), so the error
//! counters only move when a device model reports a fault explicitly.

use std::collections::VecDeque;

use super::map::can as reg;
use gsim_core::bus::{extract, merge, AccessWidth, BusError};
use gsim_core::can::{CanDirection, CanFrame};
use gsim_core::ctx::{Ctx, Event};
use gsim_core::irq::Irq;
use gsim_core::time::NANOS_PER_SEC;

/// `MCR` bits.
pub mod mcr {
    pub const INRQ: u32 = 1 << 0;
    pub const SLEEP: u32 = 1 << 1;
    pub const TXFP: u32 = 1 << 2;
    pub const RFLM: u32 = 1 << 3;
    pub const NART: u32 = 1 << 4;
    pub const AWUM: u32 = 1 << 5;
    pub const ABOM: u32 = 1 << 6;
    pub const TTCM: u32 = 1 << 7;
    pub const RESET: u32 = 1 << 15;
    pub const DBF: u32 = 1 << 16;
}

/// `MSR` bits.
pub mod msr {
    pub const INAK: u32 = 1 << 0;
    pub const SLAK: u32 = 1 << 1;
    pub const ERRI: u32 = 1 << 2;
    pub const WKUI: u32 = 1 << 3;
    pub const SLAKI: u32 = 1 << 4;
    pub const TXM: u32 = 1 << 8;
    pub const RXM: u32 = 1 << 9;
    pub const SAMP: u32 = 1 << 10;
}

/// `TSR` bits.
pub mod tsr {
    pub const RQCP0: u32 = 1 << 0;
    pub const TXOK0: u32 = 1 << 1;
    pub const ALST0: u32 = 1 << 2;
    pub const TERR0: u32 = 1 << 3;
    pub const ABRQ0: u32 = 1 << 7;
    pub const RQCP1: u32 = 1 << 8;
    pub const TXOK1: u32 = 1 << 9;
    pub const ALST1: u32 = 1 << 10;
    pub const TERR1: u32 = 1 << 11;
    pub const ABRQ1: u32 = 1 << 15;
    pub const RQCP2: u32 = 1 << 16;
    pub const TXOK2: u32 = 1 << 17;
    pub const ALST2: u32 = 1 << 18;
    pub const TERR2: u32 = 1 << 19;
    pub const ABRQ2: u32 = 1 << 23;
    pub const CODE: u32 = 0b11 << 24;
    pub const TME0: u32 = 1 << 26;
    pub const TME1: u32 = 1 << 27;
    pub const TME2: u32 = 1 << 28;
}

/// `RF0R`/`RF1R` bits.
pub mod rf {
    pub const FMP: u32 = 0b11;
    pub const FULL: u32 = 1 << 2;
    pub const FOVR: u32 = 1 << 3;
    pub const RFOM: u32 = 1 << 5;
}

/// `IER` bits.
pub mod ier {
    pub const TMEIE: u32 = 1 << 0;
    pub const FMPIE0: u32 = 1 << 1;
    pub const FFIE0: u32 = 1 << 2;
    pub const FOVRIE0: u32 = 1 << 3;
    pub const FMPIE1: u32 = 1 << 4;
    pub const FFIE1: u32 = 1 << 5;
    pub const FOVRIE1: u32 = 1 << 6;
    pub const EWGIE: u32 = 1 << 7;
    pub const EPVIE: u32 = 1 << 8;
    pub const BOFIE: u32 = 1 << 9;
    pub const LECIE: u32 = 1 << 10;
    pub const ERRIE: u32 = 1 << 11;
    pub const WKUIE: u32 = 1 << 12;
    pub const SLKIE: u32 = 1 << 13;
}

/// `ESR` bits.
pub mod esr {
    pub const EWGF: u32 = 1 << 0;
    pub const EPVF: u32 = 1 << 1;
    pub const BOFF: u32 = 1 << 2;
    pub const LEC: u32 = 0b111 << 4;
    pub const TEC: u32 = 0xFF << 16;
    pub const REC: u32 = 0xFF << 24;
}

/// `BTR` bits.
pub mod btr {
    pub const BRP: u32 = 0x3FF;
    pub const TS1: u32 = 0xF << 16;
    pub const TS2: u32 = 0b111 << 20;
    pub const SJW: u32 = 0b11 << 24;
    pub const LBKM: u32 = 1 << 30;
    pub const SILM: u32 = 1 << 31;
}

/// `TIR`/`RIR` bits.
pub mod tir {
    pub const TXRQ: u32 = 1 << 0;
    pub const RTR: u32 = 1 << 1;
    pub const IDE: u32 = 1 << 2;
    pub const EXID: u32 = 0x3FFFF << 3;
    pub const STID: u32 = 0x7FF << 21;
}

/// `FM1R`/`FS1R`/`FFA1R`/`FA1R` bit per bank plus the `FMR` control bits.
pub mod fmr {
    pub const FINIT: u32 = 1 << 0;
    pub const CAN2SB: u32 = 0x3F << 8;
}

/// Operating mode reported by [`Can::mode`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CanMode {
    /// `INRQ` set: configuration registers are writable, no traffic.
    Initialization,
    /// `SLEEP` set: the controller waits for bus activity.
    Sleep,
    /// `LBKM` set: transmissions are looped back internally.
    Loopback,
    /// `SILM` set: receive only, the transmit pin stays recessive.
    Silent,
    /// Normal operation on a two wire bus.
    Normal,
}

impl CanMode {
    pub fn label(self) -> &'static str {
        match self {
            CanMode::Initialization => "初始化",
            CanMode::Sleep => "睡眠",
            CanMode::Loopback => "回环",
            CanMode::Silent => "静默（只听）",
            CanMode::Normal => "正常",
        }
    }
}

/// Counters for the status browser and the bus window.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CanStats {
    pub tx_frames: u64,
    pub rx_frames: u64,
    pub loopback_frames: u64,
    pub aborted: u64,
    pub dropped: u64,
}

#[derive(Clone, Copy, Debug, Default)]
struct Mailbox {
    tir: u32,
    tdtr: u32,
    tdlr: u32,
    tdhr: u32,
}

#[derive(Clone, Copy, Debug)]
struct Pending {
    mailbox: usize,
    remaining_ns: u64,
}

#[derive(Clone, Debug)]
struct RxSlot {
    registers: Mailbox,
    frame: CanFrame,
}

/// One bxCAN instance.
#[derive(Debug, Clone)]
pub struct Can {
    index: u8,
    tx_irq: Irq,
    rx0_irq: Irq,
    rx1_irq: Irq,
    sce_irq: Irq,
    mcr: u32,
    tsr: u32,
    rf0r: u32,
    rf1r: u32,
    ier: u32,
    esr: u32,
    btr: u32,
    tx: [Mailbox; reg::MAILBOXES],
    pending: Vec<Pending>,
    rx: [VecDeque<RxSlot>; 2],
    fmr: u32,
    fm1r: u32,
    fs1r: u32,
    ffa1r: u32,
    fa1r: u32,
    filters: [[u32; 2]; reg::FILTER_BANKS],
    stats: CanStats,
    irq_state: [bool; 4],
}

impl Can {
    pub fn new(index: u8, tx_irq: Irq, rx0_irq: Irq, rx1_irq: Irq, sce_irq: Irq) -> Self {
        Can {
            index,
            tx_irq,
            rx0_irq,
            rx1_irq,
            sce_irq,
            mcr: 0,
            tsr: tsr::TME0 | tsr::TME1 | tsr::TME2,
            rf0r: 0,
            rf1r: 0,
            ier: 0,
            esr: 0,
            btr: 0,
            tx: [Mailbox::default(); reg::MAILBOXES],
            pending: Vec::new(),
            rx: [VecDeque::new(), VecDeque::new()],
            fmr: fmr::CAN2SB,
            fm1r: 0,
            fs1r: 0,
            ffa1r: 0,
            fa1r: 0,
            filters: [[0; 2]; reg::FILTER_BANKS],
            stats: CanStats::default(),
            irq_state: [false; 4],
        }
    }

    pub fn reset(&mut self) {
        let (index, tx, rx0, rx1, sce) = (
            self.index,
            self.tx_irq,
            self.rx0_irq,
            self.rx1_irq,
            self.sce_irq,
        );
        *self = Can::new(index, tx, rx0, rx1, sce);
    }

    /// Unit index (0 = CAN1).
    pub fn index(&self) -> u8 {
        self.index
    }

    pub fn mode(&self) -> CanMode {
        if self.mcr & mcr::INRQ != 0 {
            CanMode::Initialization
        } else if self.mcr & mcr::SLEEP != 0 {
            CanMode::Sleep
        } else if self.btr & btr::LBKM != 0 {
            CanMode::Loopback
        } else if self.btr & btr::SILM != 0 {
            CanMode::Silent
        } else {
            CanMode::Normal
        }
    }

    pub fn stats(&self) -> CanStats {
        self.stats
    }

    /// Error counters as `(transmit, receive)`.
    pub fn error_counters(&self) -> (u32, u32) {
        let tec = (self.esr & esr::TEC) >> 16;
        let rec = (self.esr & esr::REC) >> 24;
        (tec, rec)
    }

    /// Number of frames waiting in receive FIFO `fifo`.
    pub fn rx_level(&self, fifo: usize) -> usize {
        self.rx.get(fifo).map(VecDeque::len).unwrap_or(0)
    }

    /// Copies of the frames held by receive FIFO `fifo`, oldest first.
    pub fn rx_frames(&self, fifo: usize) -> Vec<CanFrame> {
        self.rx
            .get(fifo)
            .map(|queue| queue.iter().map(|slot| slot.frame).collect())
            .unwrap_or_default()
    }

    /// Transmit mailbox identifiers, `None` when the mailbox is empty.
    pub fn tx_frames(&self) -> [Option<CanFrame>; reg::MAILBOXES] {
        let mut frames = [None; reg::MAILBOXES];
        for (index, mailbox) in self.tx.iter().enumerate() {
            if self.tsr & (tsr::TME0 << index) == 0 {
                frames[index] = Some(decode_tir(mailbox.tir, mailbox.tdtr));
            }
        }
        frames
    }

    /// Number of currently active acceptance filters.
    pub fn active_filters(&self) -> usize {
        (0..reg::FILTER_BANKS)
            .filter(|bank| self.fa1r & (1 << bank) != 0)
            .count()
    }

    /// Filter bank description, used by the bus window.
    pub fn filter_summary(&self, bank: usize) -> Option<String> {
        if bank >= reg::FILTER_BANKS || self.fa1r & (1 << bank) == 0 {
            return None;
        }
        let list = self.fm1r & (1 << bank) != 0;
        let wide = self.fs1r & (1 << bank) != 0;
        let fifo = if self.ffa1r & (1 << bank) != 0 { 1 } else { 0 };
        let [fr1, fr2] = self.filters[bank];
        let mode = if list { "列表" } else { "掩码" };
        let bits = if wide { 32 } else { 16 };
        Some(format!(
            "bank{bank} {bits}位 {mode} FIFO{fifo} FR1=0x{fr1:08X} FR2=0x{fr2:08X}",
        ))
    }

    /// Nominal bit rate in bit/s for the current timing register.
    pub fn bitrate(&self, pclk_hz: u32) -> u32 {
        let divisor = self.bit_ticks();
        if divisor == 0 || pclk_hz == 0 {
            return 0;
        }
        pclk_hz / divisor as u32
    }

    /// Total number of clock ticks (of `pclk`) in one bit time.
    fn bit_ticks(&self) -> u64 {
        let brp = u64::from(self.btr & btr::BRP) + 1;
        let ts1 = u64::from((self.btr & btr::TS1) >> 16) + 1;
        let ts2 = u64::from((self.btr & btr::TS2) >> 20) + 1;
        brp * (1 + ts1 + ts2)
    }

    fn bit_ns(&self, cx: &Ctx) -> u64 {
        let pclk = u64::from(cx.clocks().pclk1_hz);
        if pclk == 0 {
            return 0;
        }
        self.bit_ticks() * NANOS_PER_SEC / pclk
    }

    // ------------------------------------------------------------- reception

    /// Delivers a frame from the bus to the acceptance filters.
    ///
    /// Returns `true` when the frame passed a filter and was queued in a
    /// receive FIFO.
    pub fn receive(&mut self, cx: &Ctx, frame: CanFrame) -> bool {
        if self.mcr & mcr::INRQ != 0 || self.fmr & fmr::FINIT != 0 {
            self.stats.dropped += 1;
            return false;
        }
        let Some((fifo, filter_index)) = self.accept(&frame) else {
            self.stats.dropped += 1;
            return false;
        };
        let queued = self.push(cx, fifo, frame, filter_index);
        if queued {
            self.stats.rx_frames += 1;
            cx.emit(Event::CanFrame {
                unit: self.index,
                direction: CanDirection::Rx,
                frame,
            });
        }
        queued
    }

    /// First active filter that accepts `frame`, as `(fifo, filter index)`.
    fn accept(&self, frame: &CanFrame) -> Option<(usize, u8)> {
        let image = filter_image(frame);
        for bank in 0..reg::FILTER_BANKS {
            if self.fa1r & (1 << bank) == 0 {
                continue;
            }
            let [fr1, fr2] = self.filters[bank];
            let list = self.fm1r & (1 << bank) != 0;
            let hit = if self.fs1r & (1 << bank) != 0 {
                if list {
                    image_matches(image, fr1) || image_matches(image, fr2)
                } else {
                    image & fr2 == fr1 & fr2
                }
            } else {
                // Two 16 bit filters per bank: FR1 holds both identifiers,
                // FR2 both masks.
                let check = |id: u32, mask: u32| {
                    if frame.extended {
                        false
                    } else if list {
                        id & 0xFFFF == image16(frame)
                    } else {
                        image16(frame) & mask == id & mask
                    }
                };
                check(fr1 & 0xFFFF, fr2 & 0xFFFF) || check(fr1 >> 16, fr2 >> 16)
            };
            if hit {
                let fifo = if self.ffa1r & (1 << bank) != 0 { 1 } else { 0 };
                return Some((fifo, bank as u8));
            }
        }
        None
    }

    fn push(&mut self, cx: &Ctx, fifo: usize, frame: CanFrame, filter_index: u8) -> bool {
        let registers = rx_registers(&frame, filter_index, (cx.now_ns() / 1000) as u32);
        let full = self.rx_level(fifo) >= reg::FIFO_DEPTH;
        if full {
            if self.mcr & mcr::RFLM != 0 {
                self.stats.dropped += 1;
                return false;
            }
            self.release(fifo);
            self.set_overflow(fifo);
        }
        if let Some(queue) = self.rx.get_mut(fifo) {
            queue.push_back(RxSlot { registers, frame });
        }
        self.refresh_irq(cx);
        true
    }

    fn set_overflow(&mut self, fifo: usize) {
        if fifo == 0 {
            self.rf0r |= rf::FOVR;
        } else {
            self.rf1r |= rf::FOVR;
        }
    }

    fn release(&mut self, fifo: usize) {
        if let Some(queue) = self.rx.get_mut(fifo) {
            queue.pop_front();
        }
    }

    // ------------------------------------------------------------- registers

    fn msr_value(&self) -> u32 {
        let mut value = 0;
        if self.mcr & mcr::INRQ != 0 {
            value |= msr::INAK;
        }
        if self.mcr & mcr::SLEEP != 0 {
            value |= msr::SLAK;
        }
        if self.error_flag() {
            value |= msr::ERRI;
        }
        if self.pending.is_empty() {
            value |= msr::RXM;
        } else {
            value |= msr::TXM;
        }
        value
    }

    fn error_flag(&self) -> bool {
        self.esr & (esr::EWGF | esr::EPVF | esr::BOFF) != 0
    }

    fn rf_value(&self, fifo: usize) -> u32 {
        let level = self.rx_level(fifo) as u32;
        let stored = if fifo == 0 { self.rf0r } else { self.rf1r };
        let mut value = level & rf::FMP;
        if level >= reg::FIFO_DEPTH as u32 {
            value |= rf::FULL;
        }
        value | stored & rf::FOVR
    }

    fn tsr_value(&self) -> u32 {
        let mut value = self.tsr;
        // CODE names the lowest free mailbox, which is what a driver uses to
        // pick a mailbox for the next frame.
        for index in 0..reg::MAILBOXES {
            if value & (tsr::TME0 << index) != 0 {
                value = (value & !tsr::CODE) | ((index as u32) << 24);
                break;
            }
        }
        value
    }

    fn refresh_irq(&mut self, cx: &Ctx) {
        let tx = self.ier & ier::TMEIE != 0 && self.mcr & mcr::INRQ == 0 && self.pending.is_empty();
        let f0 = self.rf_value(0);
        let f1 = self.rf_value(1);
        let rx0 = self.mcr & mcr::INRQ == 0
            && ((self.ier & ier::FMPIE0 != 0 && f0 & rf::FMP != 0)
                || (self.ier & ier::FFIE0 != 0 && f0 & rf::FULL != 0)
                || (self.ier & ier::FOVRIE0 != 0 && f0 & rf::FOVR != 0));
        let rx1 = self.mcr & mcr::INRQ == 0
            && ((self.ier & ier::FMPIE1 != 0 && f1 & rf::FMP != 0)
                || (self.ier & ier::FFIE1 != 0 && f1 & rf::FULL != 0)
                || (self.ier & ier::FOVRIE1 != 0 && f1 & rf::FOVR != 0));
        let sce = self.mcr & mcr::INRQ == 0
            && ((self.ier & ier::ERRIE != 0 && self.error_flag())
                || (self.ier & ier::BOFIE != 0 && self.esr & esr::BOFF != 0)
                || (self.ier & ier::EPVIE != 0 && self.esr & esr::EPVF != 0)
                || (self.ier & ier::EWGIE != 0 && self.esr & esr::EWGF != 0)
                || (self.ier & ier::LECIE != 0 && self.esr & esr::LEC != 0));
        let lines = [(self.tx_irq, tx), (self.rx0_irq, rx0), (self.rx1_irq, rx1), (self.sce_irq, sce)];
        for (line, (irq, high)) in lines.into_iter().enumerate() {
            if self.irq_state[line] != high {
                self.irq_state[line] = high;
                cx.irq(irq, high);
            }
        }
    }

    pub fn read_reg(&mut self, cx: &Ctx, offset: u32, width: AccessWidth) -> Result<u32, BusError> {
        let _ = cx;
        let value = match offset & !3 {
            reg::MCR => self.mcr,
            reg::MSR => self.msr_value(),
            reg::TSR => self.tsr_value(),
            reg::RF0R => self.rf_value(0),
            reg::RF1R => self.rf_value(1),
            reg::IER => self.ier,
            reg::ESR => self.esr,
            reg::BTR => self.btr,
            reg::FMR => self.fmr,
            reg::FM1R => self.fm1r,
            reg::FS1R => self.fs1r,
            reg::FFA1R => self.ffa1r,
            reg::FA1R => self.fa1r,
            other => {
                if let Some(value) = self.read_mailbox(other) {
                    value
                } else if let Some((bank, high)) = filter_offset(other) {
                    self.filters[bank][usize::from(high)]
                } else {
                    return Err(BusError::Unmapped {
                        addr: offset,
                        width,
                        write: false,
                    });
                }
            }
        };
        Ok(extract(value, offset, width))
    }

    fn read_mailbox(&self, offset: u32) -> Option<u32> {
        if (reg::TX_BASE..reg::TX_BASE + reg::MAILBOXES as u32 * reg::TX_STRIDE).contains(&offset)
        {
            let index = ((offset - reg::TX_BASE) / reg::TX_STRIDE) as usize;
            let mailbox = self.tx[index];
            return Some(match (offset - reg::TX_BASE) % reg::TX_STRIDE {
                reg::TIR => mailbox.tir,
                reg::TDTR => mailbox.tdtr,
                reg::TDLR => mailbox.tdlr,
                reg::TDHR => mailbox.tdhr,
                _ => 0,
            });
        }
        if (reg::RX_BASE..reg::RX_BASE + 2 * reg::RX_STRIDE).contains(&offset) {
            let fifo = ((offset - reg::RX_BASE) / reg::RX_STRIDE) as usize;
            let slot = self.rx.get(fifo).and_then(|queue| queue.front());
            let registers = slot.map(|slot| slot.registers).unwrap_or_default();
            return Some(match (offset - reg::RX_BASE) % reg::RX_STRIDE {
                reg::RIR => registers.tir,
                reg::RDTR => registers.tdtr,
                reg::RDLR => registers.tdlr,
                reg::RDHR => registers.tdhr,
                _ => 0,
            });
        }
        None
    }

    pub fn write_reg(
        &mut self,
        cx: &Ctx,
        offset: u32,
        width: AccessWidth,
        value: u32,
    ) -> Result<(), BusError> {
        match offset & !3 {
            reg::MCR => {
                if value & mcr::RESET != 0 {
                    self.reset();
                } else {
                    let previous = self.mcr;
                    self.mcr = merge(previous, offset, width, value)
                        & (mcr::INRQ
                            | mcr::SLEEP
                            | mcr::TXFP
                            | mcr::RFLM
                            | mcr::NART
                            | mcr::AWUM
                            | mcr::ABOM
                            | mcr::TTCM
                            | mcr::DBF);
                    if self.mcr & mcr::INRQ != 0 && previous & mcr::INRQ == 0 {
                        self.abort_all();
                    }
                    if self.mcr & mcr::SLEEP != 0 && self.mcr & mcr::AWUM != 0 {
                        // Auto wake-up leaves sleep as soon as the bus is active.
                        self.mcr &= !mcr::SLEEP;
                    }
                }
            }
            reg::MSR => {} // WKUI/SLAKI are cleared by a read
            reg::TSR => {
                for index in 0..reg::MAILBOXES {
                    if value & (tsr::ABRQ0 << (index * 8)) != 0 && !self.mailbox_empty(index) {
                        self.pending.retain(|pending| pending.mailbox != index);
                        self.set_mailbox_empty(index);
                        self.tsr |= tsr::ABRQ0 << (index * 8);
                        self.stats.aborted += 1;
                    }
                }
                // RQCPx/TXOKx/ALSTx/TERRx are cleared by writing one.
                self.tsr &= !(value
                    & (tsr::RQCP0
                        | tsr::TXOK0
                        | tsr::ALST0
                        | tsr::TERR0
                        | tsr::RQCP1
                        | tsr::TXOK1
                        | tsr::ALST1
                        | tsr::TERR1
                        | tsr::RQCP2
                        | tsr::TXOK2
                        | tsr::ALST2
                        | tsr::TERR2));
            }
            reg::RF0R => {
                if value & rf::RFOM != 0 {
                    self.release(0);
                }
                if value & rf::FOVR != 0 {
                    self.rf0r &= !rf::FOVR;
                }
            }
            reg::RF1R => {
                if value & rf::RFOM != 0 {
                    self.release(1);
                }
                if value & rf::FOVR != 0 {
                    self.rf1r &= !rf::FOVR;
                }
            }
            reg::IER => self.ier = merge(self.ier, offset, width, value) & 0x3FFF,
            reg::ESR => {
                let merged = merge(self.esr, offset, width, value);
                self.esr = (self.esr & !esr::LEC) | (merged & esr::LEC);
            }
            reg::BTR => {
                if self.mcr & mcr::INRQ != 0 {
                    self.btr = merge(self.btr, offset, width, value);
                }
            }
            reg::FMR => {
                self.fmr = merge(self.fmr, offset, width, value) & (fmr::FINIT | fmr::CAN2SB)
            }
            reg::FM1R => self.fm1r = merge(self.fm1r, offset, width, value) & bank_mask(),
            reg::FS1R => self.fs1r = merge(self.fs1r, offset, width, value) & bank_mask(),
            reg::FFA1R => self.ffa1r = merge(self.ffa1r, offset, width, value) & bank_mask(),
            reg::FA1R => self.fa1r = merge(self.fa1r, offset, width, value) & bank_mask(),
            other => {
                if let Some(index) = tx_mailbox_index(other) {
                    self.write_tx_mailbox(cx, index, offset, width, value);
                } else if let Some((bank, high)) = filter_offset(other) {
                    let slot = &mut self.filters[bank][usize::from(high)];
                    *slot = merge(*slot, offset, width, value);
                } else if (reg::RX_BASE..reg::RX_BASE + 2 * reg::RX_STRIDE).contains(&other) {
                    // Receive mailboxes are read-only in hardware.
                } else {
                    return Err(BusError::Unmapped {
                        addr: offset,
                        width,
                        write: true,
                    });
                }
            }
        }
        self.refresh_irq(cx);
        Ok(())
    }

    fn write_tx_mailbox(
        &mut self,
        cx: &Ctx,
        index: usize,
        offset: u32,
        width: AccessWidth,
        value: u32,
    ) {
        let register = (offset - reg::TX_BASE) % reg::TX_STRIDE;
        let byte = offset & 3;
        let mailbox = &mut self.tx[index];
        match register {
            reg::TIR => {
                mailbox.tir = merge(mailbox.tir, byte, width, value);
                if mailbox.tir & tir::TXRQ != 0 && self.mcr & mcr::INRQ == 0 {
                    let frame = frame_of(mailbox);
                    self.pending.retain(|pending| pending.mailbox != index);
                    self.tsr &= !(tsr::TME0 << index);
                    self.tsr &= !((tsr::RQCP0 | tsr::TXOK0 | tsr::ALST0 | tsr::TERR0) << (index * 8));
                    let bits = frame.bit_count() * self.bit_ns(cx);
                    self.pending.push(Pending {
                        mailbox: index,
                        remaining_ns: bits.max(1),
                    });
                    if self.btr & btr::SILM != 0 {
                        // Silent mode: the frame is discarded instead of sent.
                        self.stats.dropped += 1;
                    }
                }
            }
            reg::TDTR => mailbox.tdtr = merge(mailbox.tdtr, byte, width, value) & 0xFFFF_000F,
            reg::TDLR => mailbox.tdlr = merge(mailbox.tdlr, byte, width, value),
            reg::TDHR => mailbox.tdhr = merge(mailbox.tdhr, byte, width, value),
            _ => {}
        }
    }

    fn mailbox_empty(&self, index: usize) -> bool {
        self.tsr & (tsr::TME0 << index) != 0
    }

    fn set_mailbox_empty(&mut self, index: usize) {
        self.tsr |= tsr::TME0 << index;
    }

    fn abort_all(&mut self) {
        self.pending.clear();
        self.tsr |= tsr::TME0 | tsr::TME1 | tsr::TME2;
    }

    pub fn advance(&mut self, cx: &Ctx, elapsed_ns: u64) {
        if self.mcr & mcr::INRQ != 0 {
            self.refresh_irq(cx);
            return;
        }
        let mut due = Vec::new();
        for pending in &mut self.pending {
            if elapsed_ns >= pending.remaining_ns {
                due.push(pending.mailbox);
            } else {
                pending.remaining_ns -= elapsed_ns;
            }
        }
        for index in due {
            self.pending.retain(|pending| pending.mailbox != index);
            self.complete_transmission(cx, index);
        }
        self.refresh_irq(cx);
    }

    fn complete_transmission(&mut self, cx: &Ctx, index: usize) {
        let mailbox = self.tx[index];
        let frame = frame_of(&mailbox);
        self.set_mailbox_empty(index);
        self.tsr |= (tsr::RQCP0 | tsr::TXOK0) << (index * 8);
        self.tx[index].tir &= !tir::TXRQ;

        if self.btr & btr::SILM != 0 {
            self.stats.dropped += 1;
            return;
        }
        if self.btr & btr::LBKM != 0 {
            self.stats.loopback_frames += 1;
            cx.emit(Event::CanFrame {
                unit: self.index,
                direction: CanDirection::Loopback,
                frame,
            });
            if self.fmr & fmr::FINIT == 0 {
                if let Some((fifo, filter_index)) = self.accept(&frame) {
                    self.push(cx, fifo, frame, filter_index);
                }
            }
        } else {
            self.stats.tx_frames += 1;
            cx.emit(Event::CanFrame {
                unit: self.index,
                direction: CanDirection::Tx,
                frame,
            });
        }
    }

    /// Reports a fault from the bus side, updating the error counters the way
    /// hardware would after a failed transmission or reception.
    pub fn report_error(&mut self, cx: &Ctx, transmit: bool) {
        let tec = ((self.esr & esr::TEC) >> 16).saturating_add(if transmit { 8 } else { 0 });
        let rec = ((self.esr & esr::REC) >> 24).saturating_add(if transmit { 0 } else { 1 });
        self.esr = (self.esr & esr::LEC)
            | ((tec.min(0xFF)) << 16)
            | ((rec.min(0xFF)) << 24)
            | 0b011 << 4;
        if tec >= 128 || rec >= 128 {
            self.esr |= esr::EPVF;
        }
        if tec >= 255 {
            self.esr |= esr::BOFF;
            if self.mcr & mcr::ABOM != 0 {
                self.abort_all();
                self.esr &= !esr::BOFF;
            }
        }
        self.refresh_irq(cx);
    }
}

crate::impl_peripheral!(Can, "CAN");

fn bank_mask() -> u32 {
    (1 << reg::FILTER_BANKS) - 1
}

fn tx_mailbox_index(offset: u32) -> Option<usize> {
    if (reg::TX_BASE..reg::TX_BASE + reg::MAILBOXES as u32 * reg::TX_STRIDE).contains(&offset) {
        Some(((offset - reg::TX_BASE) / reg::TX_STRIDE) as usize)
    } else {
        None
    }
}

fn filter_offset(offset: u32) -> Option<(usize, bool)> {
    if offset < reg::FILTER_BASE {
        return None;
    }
    let relative = offset - reg::FILTER_BASE;
    let bank = (relative / reg::FILTER_STRIDE) as usize;
    if bank >= reg::FILTER_BANKS {
        return None;
    }
    Some((bank, (relative % reg::FILTER_STRIDE) / 4 != 0))
}

fn image_matches(image: u32, filter: u32) -> bool {
    // List mode: the stored identifier must match the relevant identifier
    // bits exactly; RTR/IDE have to agree as well.
    image & (tir::STID | tir::EXID | tir::IDE | tir::RTR)
        == filter & (tir::STID | tir::EXID | tir::IDE | tir::RTR)
}

/// Frame as it appears inside a 32 bit filter register.
fn filter_image(frame: &CanFrame) -> u32 {
    let remote = if frame.remote { tir::RTR } else { 0 };
    if frame.extended {
        ((frame.id & 0x3FFFF) << 3) | tir::IDE | remote
    } else {
        ((frame.id & 0x7FF) << 21) | remote
    }
}

/// Frame as it appears inside a 16 bit filter register (standard identifiers).
fn image16(frame: &CanFrame) -> u32 {
    let remote = if frame.remote { 1 << 3 } else { 0 };
    ((frame.id & 0x7FF) << 5) | remote
}

fn decode_tir(tir_value: u32, tdtr: u32) -> CanFrame {
    let dlc = (tdtr & 0xF).min(8) as u8;
    CanFrame {
        id: if tir_value & tir::IDE != 0 {
            (tir_value & tir::EXID) >> 3
        } else {
            (tir_value & tir::STID) >> 21
        },
        extended: tir_value & tir::IDE != 0,
        remote: tir_value & tir::RTR != 0,
        dlc,
        data: [0; 8],
    }
}

/// Complete frame held by a transmit mailbox.
fn frame_of(mailbox: &Mailbox) -> CanFrame {
    let mut frame = decode_tir(mailbox.tir, mailbox.tdtr);
    for index in 0..4 {
        frame.data[index] = (mailbox.tdlr >> (index * 8)) as u8;
        frame.data[index + 4] = (mailbox.tdhr >> (index * 8)) as u8;
    }
    frame
}

fn rx_registers(frame: &CanFrame, filter_index: u8, time_us: u32) -> Mailbox {
    let mut front = 0u32;
    let mut back = 0u32;
    for index in 0..4 {
        front |= u32::from(frame.data[index]) << (index * 8);
        back |= u32::from(frame.data[index + 4]) << (index * 8);
    }
    Mailbox {
        tir: filter_image(frame),
        tdtr: (u32::from(frame.dlc) & 0xF)
            | ((u32::from(filter_index) & 0xFF) << 8)
            | ((time_us & 0xFFFF) << 16),
        tdlr: front,
        tdhr: back,
    }
}

impl Can {
    /// Labelled state for the peripheral status browser and the CAN window.
    pub fn status(&self, clocks: &gsim_core::Clocks) -> Vec<(&'static str, String)> {
        let (tec, rec) = self.error_counters();
        let busy = (0..reg::MAILBOXES)
            .filter(|index| !self.mailbox_empty(*index))
            .count();
        vec![
            ("模式", self.mode().label().to_string()),
            ("波特率", crate::status::hz(self.bitrate(clocks.pclk1_hz))),
            ("发送邮箱", format!("{busy}/{} 使用中", reg::MAILBOXES)),
            (
                "接收 FIFO",
                format!("FIFO0 {} 帧，FIFO1 {} 帧", self.rx_level(0), self.rx_level(1)),
            ),
            ("验收滤波器", format!("{} 组启用", self.active_filters())),
            ("错误计数", format!("TEC {tec} / REC {rec}")),
            (
                "累计帧",
                format!(
                    "发送 {} · 接收 {} · 回环 {}",
                    self.stats.tx_frames, self.stats.rx_frames, self.stats.loopback_frames
                ),
            ),
        ]
    }
}