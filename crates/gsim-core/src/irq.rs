//! Interrupt line numbering for the Cortex-M3 NVIC.
//!
//! Interrupt lines are plain numbers because the NVIC only ever deals with
//! numbers: `IRQn` maps to exception number `16 + IRQn`. Named constants are
//! provided for the lines that phase 1 peripherals drive; a later phase adds
//! its own `pub const` entries here without touching any other module.

/// An NVIC interrupt line (external interrupt).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Irq(pub u16);

impl Irq {
    pub const fn new(number: u16) -> Self {
        Irq(number)
    }

    /// NVIC interrupt number (`IRQn`).
    pub const fn number(self) -> u16 {
        self.0
    }

    /// Cortex-M exception number for this interrupt.
    pub const fn exception(self) -> u16 {
        self.0 + 16
    }
}

impl From<Irq> for u16 {
    fn from(irq: Irq) -> u16 {
        irq.0
    }
}

/// Number of external interrupt lines implemented by the NVIC.
pub const NUM_IRQ_LINES: u16 = 240;

/// Cortex-M exception numbers.
pub mod exception {
    pub const NMI: u16 = 2;
    pub const HARD_FAULT: u16 = 3;
    pub const MEM_MANAGE: u16 = 4;
    pub const BUS_FAULT: u16 = 5;
    pub const USAGE_FAULT: u16 = 6;
    pub const SVCALL: u16 = 11;
    pub const DEBUG_MONITOR: u16 = 12;
    pub const PEND_SV: u16 = 14;
    pub const SYSTICK: u16 = 15;
}

/// STM32F103 interrupt lines driven by the peripherals implemented in phase 1.
pub mod lines {
    use super::Irq;

    pub const WWDG: Irq = Irq::new(0);
    pub const PVD: Irq = Irq::new(1);
    pub const TAMPER: Irq = Irq::new(2);
    pub const RTC: Irq = Irq::new(3);
    pub const FLASH: Irq = Irq::new(4);
    pub const RCC: Irq = Irq::new(5);
    pub const EXTI0: Irq = Irq::new(6);
    pub const EXTI1: Irq = Irq::new(7);
    pub const EXTI2: Irq = Irq::new(8);
    pub const EXTI3: Irq = Irq::new(9);
    pub const EXTI4: Irq = Irq::new(10);
    pub const DMA1_CHANNEL1: Irq = Irq::new(11);
    pub const DMA1_CHANNEL2: Irq = Irq::new(12);
    pub const DMA1_CHANNEL3: Irq = Irq::new(13);
    pub const DMA1_CHANNEL4: Irq = Irq::new(14);
    pub const DMA1_CHANNEL5: Irq = Irq::new(15);
    pub const DMA1_CHANNEL6: Irq = Irq::new(16);
    pub const DMA1_CHANNEL7: Irq = Irq::new(17);
    pub const ADC1_2: Irq = Irq::new(18);
    /// `USB_HP_CAN1_TX_IRQn`.
    pub const CAN1_TX: Irq = Irq::new(19);
    /// `USB_LP_CAN1_RX0_IRQn`.
    pub const CAN1_RX0: Irq = Irq::new(20);
    pub const CAN1_RX1: Irq = Irq::new(21);
    /// Status change (errors, wake-up) interrupt of CAN1.
    pub const CAN1_SCE: Irq = Irq::new(22);
    pub const EXTI9_5: Irq = Irq::new(23);
    pub const TIM1_BRK: Irq = Irq::new(24);
    pub const TIM1_UP: Irq = Irq::new(25);
    pub const TIM1_TRG_COM: Irq = Irq::new(26);
    pub const TIM1_CC: Irq = Irq::new(27);
    pub const TIM2: Irq = Irq::new(28);
    pub const TIM3: Irq = Irq::new(29);
    pub const TIM4: Irq = Irq::new(30);
    pub const I2C1_EV: Irq = Irq::new(31);
    pub const I2C1_ER: Irq = Irq::new(32);
    pub const I2C2_EV: Irq = Irq::new(33);
    pub const I2C2_ER: Irq = Irq::new(34);
    pub const SPI1: Irq = Irq::new(35);
    pub const SPI2: Irq = Irq::new(36);
    pub const USART1: Irq = Irq::new(37);
    pub const USART2: Irq = Irq::new(38);
    pub const USART3: Irq = Irq::new(39);
    pub const EXTI15_10: Irq = Irq::new(40);
    pub const RTC_ALARM: Irq = Irq::new(41);
    pub const DMA2_CHANNEL1: Irq = Irq::new(56);
    pub const DMA2_CHANNEL2: Irq = Irq::new(57);
    pub const DMA2_CHANNEL3: Irq = Irq::new(58);
    pub const DMA2_CHANNEL4: Irq = Irq::new(59);
    pub const DMA2_CHANNEL5: Irq = Irq::new(60);

    /// EXTI line number to NVIC interrupt line.
    pub const EXTI: [Irq; 16] = [
        EXTI0, EXTI1, EXTI2, EXTI3, EXTI4, EXTI9_5, EXTI9_5, EXTI9_5, EXTI9_5, EXTI9_5, EXTI15_10,
        EXTI15_10, EXTI15_10, EXTI15_10, EXTI15_10, EXTI15_10,
    ];
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exception_number_is_irq_plus_16() {
        assert_eq!(lines::USART1.exception(), 53);
        assert_eq!(lines::EXTI0.number(), 6);
        assert_eq!(exception::SYSTICK + 16, 31);
    }

    #[test]
    fn exti_line_table_is_monotonic_within_groups() {
        assert_eq!(lines::EXTI[0], lines::EXTI0);
        assert_eq!(lines::EXTI[4], lines::EXTI4);
        assert_eq!(lines::EXTI[9], lines::EXTI9_5);
        assert_eq!(lines::EXTI[10], lines::EXTI15_10);
        assert_eq!(lines::EXTI[15], lines::EXTI15_10);
    }
}
