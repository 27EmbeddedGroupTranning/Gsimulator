//! Time base used by every peripheral model.
//!
//! Global simulation time is expressed in nanoseconds. A peripheral converts
//! elapsed nanoseconds into its own clock cycles with [`NanoAccumulator`],
//! which keeps the sub-cycle remainder so that no drift accumulates no matter
//! how coarse or fine the simulation step is.

/// Nanoseconds per second.
pub const NANOS_PER_SEC: u64 = 1_000_000_000;

/// Converts nanoseconds into clock cycles without losing the remainder.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct NanoAccumulator {
    remainder: u64,
}

impl NanoAccumulator {
    pub const fn new() -> Self {
        Self { remainder: 0 }
    }

    /// Drops the accumulated remainder, used on peripheral reset.
    pub fn reset(&mut self) {
        self.remainder = 0;
    }

    /// Number of `hz` cycles that elapse in `elapsed_ns` nanoseconds.
    ///
    /// Returns `0` when the peripheral is not clocked (`hz == 0`).
    pub fn cycles(&mut self, elapsed_ns: u64, hz: u32) -> u64 {
        if hz == 0 || elapsed_ns == 0 {
            return 0;
        }
        let scaled = elapsed_ns.saturating_mul(u64::from(hz)) + self.remainder;
        self.remainder = scaled % NANOS_PER_SEC;
        scaled / NANOS_PER_SEC
    }
}

/// Converts a cycle count at `hz` into nanoseconds (rounding up).
pub fn cycles_to_ns(cycles: u64, hz: u32) -> u64 {
    if hz == 0 {
        return 0;
    }
    let ns = cycles.saturating_mul(NANOS_PER_SEC);
    ns.div_ceil(u64::from(hz))
}

/// Converts nanoseconds into cycles at `hz` (rounding up).
pub fn ns_to_cycles(ns: u64, hz: u32) -> u64 {
    if hz == 0 {
        return 0;
    }
    ns.saturating_mul(u64::from(hz)).div_ceil(NANOS_PER_SEC)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn accumulator_is_drift_free() {
        let mut acc = NanoAccumulator::new();
        // 72 MHz over 1000 steps of 7 ns must equal the exact product.
        let mut total = 0;
        for _ in 0..1000 {
            total += acc.cycles(7, 72_000_000);
        }
        assert_eq!(total, 1000 * 7 * 72_000_000 / NANOS_PER_SEC);
    }

    #[test]
    fn unclocked_peripheral_never_counts() {
        let mut acc = NanoAccumulator::new();
        assert_eq!(acc.cycles(1_000_000, 0), 0);
    }

    #[test]
    fn conversions() {
        assert_eq!(cycles_to_ns(72, 72_000_000), 1_000);
        assert_eq!(cycles_to_ns(1, 72_000_000), 14);
        assert_eq!(ns_to_cycles(1_000, 72_000_000), 72);
    }
}
