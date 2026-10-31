//! Plain two terminal resistor.
//!
//! The part has no state of its own: the schematic decides where its two pins
//! land, and the electrical rules of the board (the CAN termination network in
//! particular) read [`Resistor::ohms`] to work out what it does to the wire.

/// Default value of the CAN termination resistor.
pub const CAN_TERMINATION_OHM: f64 = 120.0;

/// Lowest and highest value the part accepts, in ohms.
const MIN_OHM: f64 = 0.1;
const MAX_OHM: f64 = 1_000_000.0;

/// A resistor with a value in ohms.
#[derive(Clone, Debug, PartialEq)]
pub struct Resistor {
    ohms: f64,
}

impl Default for Resistor {
    fn default() -> Self {
        Resistor::new(CAN_TERMINATION_OHM)
    }
}

impl Resistor {
    /// Values offered by the parts panel.
    pub const PRESETS: [f64; 5] = [30.0, 60.0, 120.0, 240.0, 1_000.0];

    pub fn new(ohms: f64) -> Self {
        Resistor { ohms: clamp(ohms) }
    }

    pub fn ohms(&self) -> f64 {
        self.ohms
    }

    pub fn set_ohms(&mut self, ohms: f64) {
        self.ohms = clamp(ohms);
    }

    /// Conductance in siemens, which is what parallel networks add up.
    pub fn conductance(&self) -> f64 {
        1.0 / self.ohms
    }

    /// Short label for the schematic body and the status panel.
    pub fn label(&self) -> String {
        format_ohms(self.ohms)
    }
}

fn clamp(ohms: f64) -> f64 {
    if ohms.is_finite() {
        ohms.clamp(MIN_OHM, MAX_OHM)
    } else {
        CAN_TERMINATION_OHM
    }
}

/// Formats a resistance the way a schematic would print it.
pub fn format_ohms(ohms: f64) -> String {
    if ohms >= 1_000.0 {
        format!("{:.0} kΩ", ohms / 1_000.0)
    } else if ohms >= 10.0 {
        format!("{ohms:.0} Ω")
    } else {
        format!("{ohms:.1} Ω")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_default_part_is_the_can_termination() {
        assert_eq!(Resistor::default().ohms(), CAN_TERMINATION_OHM);
        assert_eq!(Resistor::default().label(), "120 Ω");
    }

    #[test]
    fn conductance_follows_the_value() {
        assert!((Resistor::new(120.0).conductance() - 1.0 / 120.0).abs() < 1e-12);
    }

    #[test]
    fn values_are_clamped_to_the_supported_range() {
        assert_eq!(Resistor::new(-5.0).ohms(), MIN_OHM);
        assert_eq!(Resistor::new(1e9).ohms(), MAX_OHM);
        assert_eq!(Resistor::new(f64::NAN).ohms(), CAN_TERMINATION_OHM);
    }

    #[test]
    fn the_label_picks_a_readable_unit() {
        assert_eq!(format_ohms(60.0), "60 Ω");
        assert_eq!(format_ohms(1_000.0), "1 kΩ");
        assert_eq!(format_ohms(4.7), "4.7 Ω");
    }
}
