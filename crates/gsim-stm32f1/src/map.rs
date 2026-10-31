//! STM32F103 address map and register offsets (RM0008).
//!
//! Every constant here matches the reference manual so that register access
//! sequences written against `stm32f1xx.h` from the HAL work unchanged.

/// Flash memory alias (the part number modelled is an STM32F103C8: 64 KiB).
pub const FLASH_BASE: u32 = 0x0800_0000;
pub const FLASH_SIZE: u32 = 64 * 1024;

/// SRAM (20 KiB on STM32F103C8).
pub const SRAM_BASE: u32 = 0x2000_0000;
pub const SRAM_SIZE: u32 = 20 * 1024;

/// APB1 peripheral window.
pub const APB1_BASE: u32 = 0x4000_0000;
/// APB2 peripheral window.
pub const APB2_BASE: u32 = 0x4001_0000;
/// AHB peripheral window.
pub const AHB_BASE: u32 = 0x4002_0000;
/// Cortex-M3 private peripheral bus.
pub const CORTEX_BASE: u32 = 0xE000_0000;
pub const DBGMCU_BASE: u32 = 0xE004_2000;

/// Size of the address window reserved for one peripheral.
pub const PERIPHERAL_WINDOW: u32 = 0x400;

pub mod rcc {
    pub const BASE: u32 = super::AHB_BASE + 0x1000;
    pub const CR: u32 = 0x00;
    pub const CFGR: u32 = 0x04;
    pub const CIR: u32 = 0x08;
    pub const APB2RSTR: u32 = 0x0C;
    pub const APB1RSTR: u32 = 0x10;
    pub const AHBENR: u32 = 0x14;
    pub const APB2ENR: u32 = 0x18;
    pub const APB1ENR: u32 = 0x1C;
    pub const BDCR: u32 = 0x20;
    pub const CSR: u32 = 0x24;
}

pub mod flash {
    pub const BASE: u32 = super::AHB_BASE + 0x2000;
    pub const ACR: u32 = 0x00;
    pub const KEYR: u32 = 0x04;
    pub const OPTKEYR: u32 = 0x08;
    pub const SR: u32 = 0x0C;
    pub const CR: u32 = 0x10;
    pub const AR: u32 = 0x14;
    pub const OBR: u32 = 0x1C;
    pub const WRPR: u32 = 0x20;
}

pub mod dma1 {
    pub const BASE: u32 = super::AHB_BASE;
    pub const ISR: u32 = 0x00;
    pub const IFCR: u32 = 0x04;
    /// Offset of channel `n` (0 based) register block.
    pub const fn channel(n: u32) -> u32 {
        0x08 + n * 0x14
    }
    pub const CCR: u32 = 0x00;
    pub const CNDTR: u32 = 0x04;
    pub const CPAR: u32 = 0x08;
    pub const CMAR: u32 = 0x0C;
}

pub mod dma2 {
    pub const BASE: u32 = super::AHB_BASE + 0x0400;
}

pub mod crc {
    pub const BASE: u32 = super::AHB_BASE + 0x3000;
    pub const DR: u32 = 0x00;
    pub const IDR: u32 = 0x04;
}

pub mod gpio {
    use super::APB2_BASE;
    pub const BASE_A: u32 = APB2_BASE + 0x0800;
    pub const BASE_B: u32 = APB2_BASE + 0x0C00;
    pub const BASE_C: u32 = APB2_BASE + 0x1000;
    pub const BASE_D: u32 = APB2_BASE + 0x1400;
    pub const BASE_E: u32 = APB2_BASE + 0x1800;
    pub const PORTS: [u32; 5] = [BASE_A, BASE_B, BASE_C, BASE_D, BASE_E];

    pub const CRL: u32 = 0x00;
    pub const CRH: u32 = 0x04;
    pub const IDR: u32 = 0x08;
    pub const ODR: u32 = 0x0C;
    pub const BSRR: u32 = 0x10;
    pub const BRR: u32 = 0x14;
    pub const LCKR: u32 = 0x18;
}

pub mod afio {
    pub const BASE: u32 = super::APB2_BASE;
    pub const EVCR: u32 = 0x00;
    pub const MAPR: u32 = 0x04;
    pub const EXTICR1: u32 = 0x08;
    pub const EXTICR2: u32 = 0x0C;
    pub const EXTICR3: u32 = 0x10;
    pub const EXTICR4: u32 = 0x14;
}

pub mod exti {
    pub const BASE: u32 = super::APB2_BASE + 0x0400;
    pub const IMR: u32 = 0x00;
    pub const EMR: u32 = 0x04;
    pub const RTSR: u32 = 0x08;
    pub const FTSR: u32 = 0x0C;
    pub const SWIER: u32 = 0x10;
    pub const PR: u32 = 0x14;
}

pub mod adc {
    use super::APB2_BASE;
    pub const BASE_1: u32 = APB2_BASE + 0x2400;
    pub const BASE_2: u32 = APB2_BASE + 0x2800;
    pub const BASES: [u32; 2] = [BASE_1, BASE_2];

    pub const SR: u32 = 0x00;
    pub const CR1: u32 = 0x04;
    pub const CR2: u32 = 0x08;
    pub const SMPR1: u32 = 0x0C;
    pub const SMPR2: u32 = 0x10;
    pub const JOFR1: u32 = 0x14;
    /// Last `JOFR` register (`JOFR4`).
    pub const JOFR_LAST: u32 = 0x1C;
    pub const HTR: u32 = 0x20;
    pub const LTR: u32 = 0x24;
    pub const SQR1: u32 = 0x2C;
    pub const SQR2: u32 = 0x30;
    pub const SQR3: u32 = 0x34;
    pub const JSQR: u32 = 0x38;
    pub const JDR1: u32 = 0x3C;
    /// Last injected data register (`JDR4`).
    pub const JDR_LAST: u32 = 0x48;
    pub const DR: u32 = 0x4C;
}

pub mod tim {
    use super::{APB1_BASE, APB2_BASE};
    pub const BASE_1: u32 = APB2_BASE + 0x2C00;
    pub const BASE_2: u32 = APB1_BASE;
    pub const BASE_3: u32 = APB1_BASE + 0x0400;
    pub const BASE_4: u32 = APB1_BASE + 0x0800;
    pub const BASES: [u32; 4] = [BASE_1, BASE_2, BASE_3, BASE_4];

    pub const CR1: u32 = 0x00;
    pub const CR2: u32 = 0x04;
    pub const SMCR: u32 = 0x08;
    pub const DIER: u32 = 0x0C;
    pub const SR: u32 = 0x10;
    pub const EGR: u32 = 0x14;
    pub const CCMR1: u32 = 0x18;
    pub const CCMR2: u32 = 0x1C;
    pub const CCER: u32 = 0x20;
    pub const CNT: u32 = 0x24;
    pub const PSC: u32 = 0x28;
    pub const ARR: u32 = 0x2C;
    pub const RCR: u32 = 0x30;
    pub const CCR1: u32 = 0x34;
    pub const CCR2: u32 = 0x38;
    pub const CCR3: u32 = 0x3C;
    pub const CCR4: u32 = 0x40;
    pub const BDTR: u32 = 0x44;
    pub const DCR: u32 = 0x48;
    pub const DMAR: u32 = 0x4C;
}

pub mod spi {
    use super::{APB1_BASE, APB2_BASE};
    pub const BASE_1: u32 = APB2_BASE + 0x3000;
    pub const BASE_2: u32 = APB1_BASE + 0x3800;
    pub const BASES: [u32; 2] = [BASE_1, BASE_2];

    pub const CR1: u32 = 0x00;
    pub const CR2: u32 = 0x04;
    pub const SR: u32 = 0x08;
    pub const DR: u32 = 0x0C;
    pub const CRCPR: u32 = 0x10;
    pub const RXCRCR: u32 = 0x14;
    pub const TXCRCR: u32 = 0x18;
    pub const I2SCFGR: u32 = 0x1C;
    pub const I2SPR: u32 = 0x20;
}

pub mod usart {
    use super::{APB1_BASE, APB2_BASE};
    pub const BASE_1: u32 = APB2_BASE + 0x3800;
    pub const BASE_2: u32 = APB1_BASE + 0x4400;
    pub const BASE_3: u32 = APB1_BASE + 0x4800;
    pub const BASES: [u32; 3] = [BASE_1, BASE_2, BASE_3];

    pub const SR: u32 = 0x00;
    pub const DR: u32 = 0x04;
    pub const BRR: u32 = 0x08;
    pub const CR1: u32 = 0x0C;
    pub const CR2: u32 = 0x10;
    pub const CR3: u32 = 0x14;
    pub const GTPR: u32 = 0x18;
}

pub mod i2c {
    use super::APB1_BASE;
    pub const BASE_1: u32 = APB1_BASE + 0x5400;
    pub const BASE_2: u32 = APB1_BASE + 0x5800;
    pub const BASES: [u32; 2] = [BASE_1, BASE_2];

    pub const CR1: u32 = 0x00;
    pub const CR2: u32 = 0x04;
    pub const OAR1: u32 = 0x08;
    pub const OAR2: u32 = 0x0C;
    pub const DR: u32 = 0x10;
    pub const SR1: u32 = 0x14;
    pub const SR2: u32 = 0x18;
    pub const CCR: u32 = 0x1C;
    pub const TRISE: u32 = 0x20;
}

pub mod pwr {
    pub const BASE: u32 = super::APB1_BASE + 0x7000;
    pub const CR: u32 = 0x00;
    pub const CSR: u32 = 0x04;
}

pub mod bkp {
    pub const BASE: u32 = super::APB1_BASE + 0x6C00;
    pub const RTCCR: u32 = 0x00;
    pub const CR: u32 = 0x04;
    pub const CSR: u32 = 0x08;
    pub const DR1: u32 = 0x0C;
    pub const DR_LAST: u32 = 0x30;
}

pub mod rtc {
    pub const BASE: u32 = super::APB1_BASE + 0x2800;
    pub const CRH: u32 = 0x00;
    pub const CRL: u32 = 0x04;
    pub const PRLH: u32 = 0x08;
    pub const PRLL: u32 = 0x0C;
    pub const DIVH: u32 = 0x10;
    pub const DIVL: u32 = 0x14;
    pub const CNTH: u32 = 0x18;
    pub const CNTL: u32 = 0x1C;
    pub const ALRH: u32 = 0x20;
    pub const ALRL: u32 = 0x24;
}

pub mod iwdg {
    pub const BASE: u32 = super::APB1_BASE + 0x3000;
    pub const KR: u32 = 0x00;
    pub const PR: u32 = 0x04;
    pub const RLR: u32 = 0x08;
    pub const SR: u32 = 0x0C;
}

pub mod wwdg {
    pub const BASE: u32 = super::APB1_BASE + 0x2C00;
    pub const CR: u32 = 0x00;
    pub const CFR: u32 = 0x04;
    pub const SR: u32 = 0x08;
}

/// bxCAN controller (CAN1). CAN2 only exists on the connectivity line parts
/// and shares the filter banks of CAN1, so it is not modelled.
pub mod can {
    pub const BASE_1: u32 = super::APB1_BASE + 0x6400;
    pub const BASES: [u32; 1] = [BASE_1];

    pub const MCR: u32 = 0x00;
    pub const MSR: u32 = 0x04;
    pub const TSR: u32 = 0x08;
    pub const RF0R: u32 = 0x0C;
    pub const RF1R: u32 = 0x10;
    pub const IER: u32 = 0x14;
    pub const ESR: u32 = 0x18;
    pub const BTR: u32 = 0x1C;

    /// Transmit mailbox register blocks (three of them, 16 bytes each).
    pub const TX_BASE: u32 = 0x180;
    pub const TX_STRIDE: u32 = 0x10;
    pub const TIR: u32 = 0x00;
    pub const TDTR: u32 = 0x04;
    pub const TDLR: u32 = 0x08;
    pub const TDHR: u32 = 0x0C;

    /// Receive FIFO register blocks (two of them, 16 bytes each).
    pub const RX_BASE: u32 = 0x1B0;
    pub const RX_STRIDE: u32 = 0x10;
    pub const RIR: u32 = 0x00;
    pub const RDTR: u32 = 0x04;
    pub const RDLR: u32 = 0x08;
    pub const RDHR: u32 = 0x0C;

    pub const FMR: u32 = 0x200;
    pub const FM1R: u32 = 0x204;
    pub const FS1R: u32 = 0x20C;
    pub const FFA1R: u32 = 0x214;
    pub const FA1R: u32 = 0x21C;
    /// First filter bank register (`CAN_F0R1`); bank `n` starts at
    /// `FILTER_BASE + n * 8`, `FR1` at `+0` and `FR2` at `+4`.
    pub const FILTER_BASE: u32 = 0x240;
    pub const FILTER_STRIDE: u32 = 0x08;

    /// Number of filter banks on the STM32F103.
    pub const FILTER_BANKS: usize = 14;
    /// Depth of each receive FIFO.
    pub const FIFO_DEPTH: usize = 3;
    /// Number of transmit mailboxes.
    pub const MAILBOXES: usize = 3;
}

pub mod dbgmcu {
    pub const BASE: u32 = super::DBGMCU_BASE;
    pub const IDCODE: u32 = 0x00;
    pub const CR: u32 = 0x04;
}

/// Device identifier reported by `DBGMCU->IDCODE` for the STM32F103C8.
pub const DBGMCU_IDCODE: u32 = 0x2003_6410;
