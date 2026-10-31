//! External crystals fitted to the oscillator pins.
//!
//! A crystal has no behaviour of its own: it is the passive part that makes the
//! RCC's `HSEON`/`LSEON` start-up succeed. The model only carries the kind and
//! frequency so the board can tell the SoC what is soldered where, and the
//! firmware decides which oscillator it actually clocks itself (or the RTC)
//! from.

/// Which oscillator a crystal belongs to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CrystalKind {
    /// High-speed crystal on `OSC_IN`/`OSC_OUT` (`PD0`/`PD1`).
    Hse,
    /// 32.768 kHz crystal on `OSC32_IN`/`OSC32_OUT` (`PC14`/`PC15`).
    Lse,
}

impl CrystalKind {
    pub fn label(self) -> &'static str {
        match self {
            CrystalKind::Hse => "HSE 晶振",
            CrystalKind::Lse => "LSE 晶振",
        }
    }
}

/// One crystal soldered to the board.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Crystal {
    kind: CrystalKind,
    hz: u32,
}

impl Crystal {
    /// High-speed crystal; the Blue Pill's part is 8 MHz.
    pub fn hse(hz: u32) -> Self {
        Crystal {
            kind: CrystalKind::Hse,
            hz,
        }
    }

    /// 32.768 kHz watch crystal for the RTC.
    pub fn lse() -> Self {
        Crystal {
            kind: CrystalKind::Lse,
            hz: 32_768,
        }
    }

    pub fn kind(&self) -> CrystalKind {
        self.kind
    }

    pub fn hz(&self) -> u32 {
        self.hz
    }

    /// Human readable frequency, e.g. `8 MHz` or `32.768 kHz`.
    pub fn frequency_label(&self) -> String {
        if self.hz >= 1_000_000 {
            format!("{} MHz", format_decimal(f64::from(self.hz) / 1e6))
        } else if self.hz >= 1_000 {
            format!("{} kHz", format_decimal(f64::from(self.hz) / 1e3))
        } else {
            format!("{} Hz", self.hz)
        }
    }

    pub fn label(&self) -> String {
        format!("{} {}", self.kind.label(), self.frequency_label())
    }
}

fn format_decimal(value: f64) -> String {
    let text = format!("{value:.3}");
    text.trim_end_matches('0').trim_end_matches('.').to_string()
}

#[cfg(test)]
mod tests {
    use super::{Crystal, CrystalKind};

    #[test]
    fn hse_keeps_its_frequency() {
        let crystal = Crystal::hse(8_000_000);
        assert_eq!(crystal.kind(), CrystalKind::Hse);
        assert_eq!(crystal.hz(), 8_000_000);
        assert_eq!(crystal.frequency_label(), "8 MHz");
        assert_eq!(crystal.label(), "HSE 晶振 8 MHz");
    }

    #[test]
    fn lse_is_the_watch_crystal() {
        let crystal = Crystal::lse();
        assert_eq!(crystal.kind(), CrystalKind::Lse);
        assert_eq!(crystal.hz(), 32_768);
        assert_eq!(crystal.frequency_label(), "32.768 kHz");
    }

    #[test]
    fn odd_high_speed_crystals_still_format() {
        assert_eq!(Crystal::hse(12_000_000).frequency_label(), "12 MHz");
        assert_eq!(Crystal::hse(25_000_000).frequency_label(), "25 MHz");
        assert_eq!(Crystal::hse(4_000_000).frequency_label(), "4 MHz");
    }
}
