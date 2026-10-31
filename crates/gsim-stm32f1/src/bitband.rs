//! Cortex-M3 bit-band alias regions.
//!
//! The ARMv7-M architecture exposes the first megabyte of SRAM and the first
//! megabyte of the peripheral space twice: once as ordinary memory and once
//! through an alias region where every 32-bit word addresses a *single bit* of
//! the target word. `stm32f1xx.h` relies on this heavily, e.g.
//!
//! ```c
//! #define RCC_CR_PLLON_BB (PERIPH_BB_BASE + (RCC_CR_OFFSET * 32) + (RCC_CR_PLLON_Pos * 4))
//! #define __HAL_RCC_PLL_DISABLE() (*(__IO uint32_t *) RCC_CR_PLLON_BB = DISABLE)
//! ```
//!
//! so a simulator that ignores the alias regions cannot run the STM32 HAL.
//!
//! Reading an alias word returns 0 or 1 (only bit 0 is defined). Writing an
//! alias word only inspects bit 0 of the value: 1 sets, 0 clears the target bit.

/// First byte of the bit-bandable peripheral region.
pub const PERIPHERAL_REGION: u32 = 0x4000_0000;
/// Alias base of the peripheral region (`PERIPH_BB_BASE`).
pub const PERIPHERAL_ALIAS: u32 = 0x4200_0000;
/// First byte of the bit-bandable SRAM region.
pub const SRAM_REGION: u32 = 0x2000_0000;
/// Alias base of the SRAM region (`SRAM_BB_BASE`).
pub const SRAM_ALIAS: u32 = 0x2200_0000;
/// Size of a bit-bandable region: 1 MiB.
pub const REGION_SIZE: u32 = 0x0010_0000;
/// Size of an alias region: 32 bits per byte of the target region.
pub const ALIAS_SIZE: u32 = REGION_SIZE * 32;

/// A bit-band alias resolved to the target word and bit mask it refers to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AliasedBit {
    /// Address of the 32-bit word that contains the target bit.
    pub word: u32,
    /// Mask selecting the target bit inside [`AliasedBit::word`].
    pub mask: u32,
}

impl AliasedBit {
    /// Extracts the addressed bit as `0` or `1`, the value an alias read yields.
    pub fn extract(self, word: u32) -> u32 {
        u32::from(word & self.mask != 0)
    }

    /// Applies a bit-band write (`set` comes from bit 0 of the written value).
    pub fn apply(self, word: u32, set: bool) -> u32 {
        if set {
            word | self.mask
        } else {
            word & !self.mask
        }
    }
}

/// Resolves `addr` if it falls inside one of the two alias regions.
///
/// Address bits `[1:0]` are ignored by the hardware, so a word-aligned access
/// is not strictly required; they are masked off here for the same reason.
pub fn decode(addr: u32) -> Option<AliasedBit> {
    let (alias, region) = if (PERIPHERAL_ALIAS..PERIPHERAL_ALIAS + ALIAS_SIZE).contains(&addr) {
        (PERIPHERAL_ALIAS, PERIPHERAL_REGION)
    } else if (SRAM_ALIAS..SRAM_ALIAS + ALIAS_SIZE).contains(&addr) {
        (SRAM_ALIAS, SRAM_REGION)
    } else {
        return None;
    };
    let offset = (addr - alias) & !3;
    // 32 alias words per target byte, 4 bytes per target word.
    let byte = region + (offset >> 5);
    let bit_in_byte = (offset >> 2) & 7;
    Some(AliasedBit {
        word: byte & !3,
        mask: 1 << ((byte & 3) * 8 + bit_in_byte),
    })
}

/// Encodes a target address plus bit index into its alias address.
///
/// Returns `None` when the target is not bit-bandable.
pub fn encode(addr: u32, bit: u32) -> Option<u32> {
    if bit > 31 {
        return None;
    }
    let (alias, region) = if (PERIPHERAL_REGION..PERIPHERAL_REGION + REGION_SIZE).contains(&addr) {
        (PERIPHERAL_ALIAS, PERIPHERAL_REGION)
    } else if (SRAM_REGION..SRAM_REGION + REGION_SIZE).contains(&addr) {
        (SRAM_ALIAS, SRAM_REGION)
    } else {
        return None;
    };
    let byte = addr + bit / 8;
    Some(alias + (byte - region) * 32 + (bit % 8) * 4)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decodes_hal_style_rcc_aliases() {
        // RCC_CR_PLLON_BB: RCC base 0x40021000, CR offset 0, PLLON bit 24.
        let pllon = decode(0x4242_0060).expect("PLLON alias");
        assert_eq!(pllon.word, 0x4002_1000);
        assert_eq!(pllon.mask, 1 << 24);
        // RCC_CSR_LSION_BB: CSR offset 0x24, LSION bit 0.
        let lsion = decode(0x4242_0480).expect("LSION alias");
        assert_eq!(lsion.word, 0x4002_1024);
        assert_eq!(lsion.mask, 1);
        // RCC_CR_CSSON_BB: bit 19.
        let csson = decode(0x4242_004C).expect("CSSON alias");
        assert_eq!(csson.mask, 1 << 19);
    }

    #[test]
    fn encodes_hal_style_rcc_aliases() {
        assert_eq!(encode(0x4002_1000, 24), Some(0x4242_0060)); // RCC_CR_PLLON_BB
        assert_eq!(encode(0x4002_1024, 0), Some(0x4242_0480)); // RCC_CSR_LSION_BB
        assert_eq!(encode(0x4002_1000, 19), Some(0x4242_004C)); // RCC_CR_CSSON_BB
        for bit in 0..32 {
            let alias = encode(0x4002_1000, bit).unwrap();
            let decoded = decode(alias).unwrap();
            assert_eq!(decoded.word, 0x4002_1000);
            assert_eq!(decoded.mask, 1 << bit);
        }
    }

    #[test]
    fn encodes_sram_bits() {
        let alias = encode(0x2000_0004, 3).expect("sram alias");
        assert_eq!(alias, SRAM_ALIAS + 4 * 32 + 3 * 4);
        assert_eq!(decode(alias).unwrap().word, 0x2000_0004);
        assert_eq!(decode(alias).unwrap().mask, 1 << 3);
    }

    #[test]
    fn rejects_addresses_outside_the_regions() {
        assert_eq!(decode(0x0800_0000), None);
        assert_eq!(decode(0x0800_0000 + 32), None);
        assert_eq!(decode(0x41FF_FFFF), None);
        assert_eq!(decode(0x2400_0000), None);
        assert_eq!(encode(0x0800_0000, 0), None);
        assert_eq!(encode(0x2000_0000, 32), None);
    }

    #[test]
    fn ignored_low_address_bits() {
        let aligned = decode(0x4242_0060).unwrap();
        for low in 1..4 {
            assert_eq!(decode(0x4242_0060 + low).unwrap(), aligned);
        }
    }

    #[test]
    fn extract_and_apply() {
        let bit = decode(0x4242_0060).unwrap();
        assert_eq!(bit.extract(0), 0);
        assert_eq!(bit.extract(1 << 24), 1);
        assert_eq!(bit.apply(0, true), 1 << 24);
        assert_eq!(bit.apply(u32::MAX, false), !(1 << 24));
    }
}
