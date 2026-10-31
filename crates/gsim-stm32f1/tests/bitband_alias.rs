//! Cortex-M3 bit-band alias regions.
//!
//! The STM32 HAL writes many single bits through `PERIPH_BB_BASE` /
//! `SRAM_BB_BASE` (`RCC_CR_PLLON_BB`, `GPIOA_ODR_BB`, ...), so the alias
//! regions have to behave exactly like the hardware: a read returns 0 or 1 and
//! a write sets or clears one bit of the target word without touching its
//! neighbours.

mod common;

use common::*;
use gsim_core::{AccessWidth, MemoryBus};
use gsim_stm32f1::{bitband, map, Soc};

#[test]
fn peripheral_alias_sets_and_clears_a_single_bit() {
    let mut soc = Soc::new();
    let rcc = map::rcc::BASE;

    // __HAL_RCC_PLL_DISABLE() -> *(RCC_CR_PLLON_BB) = DISABLE
    let pllon = bitband::encode(rcc + map::rcc::CR, 24).expect("PLLON alias");
    assert_eq!(pllon, 0x4242_0060);

    w32(&mut soc, rcc + map::rcc::CR, (1 << 24) | (1 << 16) | 1);
    w32(&mut soc, pllon, 0);
    assert_eq!(r32(&mut soc, rcc + map::rcc::CR) & (1 << 24), 0);
    // The neighbouring bits are untouched.
    assert_ne!(r32(&mut soc, rcc + map::rcc::CR) & (1 << 16), 0);
    assert_ne!(r32(&mut soc, rcc + map::rcc::CR) & 1, 0);

    w32(&mut soc, pllon, 1);
    assert_ne!(r32(&mut soc, rcc + map::rcc::CR) & (1 << 24), 0);
    assert_eq!(r32(&mut soc, pllon), 1);

    // Only bit 0 of the written value matters, like on silicon.
    w32(&mut soc, pllon, 0xFFFF_FFFE);
    assert_eq!(r32(&mut soc, pllon), 0);
}

#[test]
fn peripheral_alias_reaches_a_gpio_output_register() {
    let mut soc = Soc::new();
    enable_apb2(&mut soc, 1 << 2); // GPIOA
    set_pin_nibble(&mut soc, 0, 5, PIN_OUTPUT_PP_2MHZ);

    let odr5 = bitband::encode(map::gpio::PORTS[0] + map::gpio::ODR, 5).unwrap();
    w32(&mut soc, odr5, 1);
    assert_ne!(
        r32(&mut soc, map::gpio::PORTS[0] + map::gpio::ODR) & (1 << 5),
        0
    );

    // Reading the ordinary register or the alias agrees, and an off bit reads 0.
    let odr4 = bitband::encode(map::gpio::PORTS[0] + map::gpio::ODR, 4).unwrap();
    assert_eq!(r32(&mut soc, odr4), 0);

    w32(&mut soc, odr5, 0);
    assert_eq!(
        r32(&mut soc, map::gpio::PORTS[0] + map::gpio::ODR) & (1 << 5),
        0
    );
}

#[test]
fn sram_alias_manipulates_one_bit_of_a_word() {
    let mut soc = Soc::new();
    let target = map::SRAM_BASE + 0x40;

    w32(&mut soc, target, 0xAAAA_AAAA);
    let bit17 = bitband::encode(target, 17).unwrap();
    w32(&mut soc, bit17, 1);
    assert_eq!(r32(&mut soc, target), 0xAAAA_AAAA | (1 << 17));

    w32(&mut soc, bit17, 0);
    assert_eq!(r32(&mut soc, target), 0xAAAA_AAAA & !(1 << 17));

    // A byte access through the alias behaves the same way.
    soc.write(bit17, AccessWidth::Byte, 1).unwrap();
    assert_eq!(r32(&mut soc, target) & (1 << 17), 1 << 17);
}

#[test]
fn bit_band_is_not_a_valid_target_for_flash() {
    let mut soc = Soc::new();
    assert!(bitband::encode(map::FLASH_BASE, 0).is_none());
    assert!(soc.read(0x4300_0000, AccessWidth::Word).is_err());
}
