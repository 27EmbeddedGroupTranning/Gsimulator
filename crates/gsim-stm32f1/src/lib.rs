//! STM32F103 on-chip peripheral models.
//!
//! Phase 1 covers the whole on-chip peripheral set of the STM32F103 that does
//! not require an off-chip counterpart: RCC, GPIO, AFIO, EXTI, USART, SPI, I2C,
//! TIM, ADC, DMA, FLASH, PWR/BKP, RTC, IWDG/WWDG, CRC and the Cortex-M3 core
//! peripherals (NVIC, SCB, SysTick). Phase 2 adds the bxCAN controller (CAN1);
//! the off-chip counterparts (CAN transceiver, MAX7219 LED matrix) live in the
//! `gsim-devices` crate and talk to this crate only through events and the
//! frame injection API.
//!
//! Entry point is [`Soc`], which implements [`gsim_core::MemoryBus`]: a
//! firmware (or a CPU model added later) drives the device purely through
//! register reads and writes.

#[macro_use]
mod macros;

pub mod adc;
pub mod afio;
pub mod bitband;
pub mod can;
pub mod crc;
pub mod dma;
pub mod exti;
pub mod gpio;
pub mod i2c;
pub mod map;
pub mod memory;
pub mod pwr;
pub mod rcc;
pub mod rtc;
pub mod soc;
pub mod spi;
pub mod status;
pub mod tim;
pub mod usart;
pub mod watchdog;

pub use adc::Adc;
pub use afio::Afio;
pub use can::{Can, CanMode, CanStats};
pub use crc::{Crc, DbgMcu};
pub use dma::{Dma, Request as DmaRequest};
pub use exti::Exti;
pub use gpio::{GpioPort, PORT_COUNT, PORT_NAMES};
pub use i2c::I2c;
pub use memory::{Erase, FlashInterface, Memory};
pub use pwr::{Bkp, Pwr};
pub use rcc::Rcc;
pub use rtc::Rtc;
pub use soc::{Soc, Stats};
pub use spi::Spi;
pub use tim::Tim;
pub use usart::Usart;
pub use watchdog::{Iwdg, Wwdg};
