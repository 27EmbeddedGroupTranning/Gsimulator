//! The peripheral status browser is data driven: `Soc::peripheral_status()`
//! has to describe every modelled block, including the ones a front end shows
//! without touching any register.

use gsim_core::{AccessWidth, MemoryBus};
use gsim_stm32f1::{map, Soc};

#[test]
fn every_modelled_peripheral_is_listed() {
    let soc = Soc::new();
    let statuses = soc.peripheral_status();
    for expected in [
        "RCC", "GPIOA", "GPIOB", "GPIOC", "AFIO", "EXTI", "USART1", "USART2", "USART3", "SPI1",
        "SPI2", "I2C1", "I2C2", "CAN1", "TIM1", "TIM2", "TIM3", "TIM4", "ADC1", "ADC2", "DMA1",
        "DMA2", "FLASH", "PWR", "BKP", "RTC", "IWDG", "WWDG", "CRC", "DBGMCU", "NVIC", "SCB",
        "SysTick",
    ] {
        assert!(
            statuses.iter().any(|status| status.name == expected),
            "{expected} is missing from the peripheral status list"
        );
    }
}

#[test]
fn the_controller_status_decodes_mode_bitrate_and_fifos() {
    let mut soc = Soc::new();
    // Clock the controller and put it in loopback at 200 kbit/s.
    soc.write(
        map::rcc::BASE + map::rcc::APB1ENR,
        AccessWidth::Word,
        1 << 25,
    )
    .unwrap();
    soc.write(map::can::BASE_1 + map::can::MCR, AccessWidth::Word, 1)
        .unwrap();
    let btr = (2 - 1) << 16 | (2 - 1) << 20 | (36 - 1) | (1 << 30);
    soc.write(map::can::BASE_1 + map::can::BTR, AccessWidth::Word, btr)
        .unwrap();
    // Leave initialisation mode so the controller is running.
    soc.write(map::can::BASE_1 + map::can::MCR, AccessWidth::Word, 0)
        .unwrap();

    let statuses = soc.peripheral_status();
    let can = statuses
        .iter()
        .find(|status| status.name == "CAN1")
        .expect("CAN1 status");
    let row = |label: &str| {
        can.rows
            .iter()
            .find(|(name, _)| *name == label)
            .map(|(_, value)| value.clone())
            .unwrap_or_default()
    };
    assert_eq!(row("模式"), "回环");
    // 8 MHz PCLK1 and 180 clock ticks per bit is 44.4 kbit/s.
    assert_eq!(row("波特率"), "44.4 kHz");
    assert!(row("接收 FIFO").contains("FIFO0 0 帧"));
    assert!(row("累计帧").contains("发送 0"));
}

#[test]
fn clock_gating_is_visible_in_the_status_list() {
    let mut soc = Soc::new();
    let statuses = soc.peripheral_status();
    let usart1 = statuses
        .iter()
        .find(|status| status.name == "USART1")
        .unwrap();
    assert_eq!(usart1.clock_hz, 0, "APB2 gate is closed after reset");

    soc.write(
        map::rcc::BASE + map::rcc::APB2ENR,
        AccessWidth::Word,
        1 << 14,
    )
    .unwrap();
    let statuses = soc.peripheral_status();
    let usart1 = statuses
        .iter()
        .find(|status| status.name == "USART1")
        .unwrap();
    assert_eq!(usart1.clock_hz, 8_000_000);
}
