//! Virtual oscilloscope that watches the two wires of a CAN pair.
//!
//! The part is a passive probe: the bridge that owns the wiring pushes one
//! sample per simulated bit, the scope keeps a ring buffer trimmed to the on
//! screen time window and the scope window renders it. The same samples feed a
//! [`TraceDecoder`], which rebuilds the frames on the wire bit by bit so the
//! window can caption what every single bit means. The probe never drives the
//! bus, so it cannot change what the simulated network does.

use std::collections::VecDeque;

use gsim_core::can::{TraceDecoder, TraceFrame};

/// Default width of the on screen window.
pub const DEFAULT_WINDOW_NS: u64 = 2_000_000;
/// Default number of stored samples.
pub const DEFAULT_CAPACITY: usize = 8192;

/// One sample of the differential pair.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ScopeSample {
    pub time_ns: u64,
    /// Voltage of `CANH` in volts.
    pub can_h: f32,
    /// Voltage of `CANL` in volts.
    pub can_l: f32,
    /// `true` when the pair was dominant (differential above the threshold).
    pub dominant: bool,
    /// `true` when only ringing put the level there, so the bit is suspect.
    pub glitch: bool,
}

impl ScopeSample {
    /// Differential voltage `CANH - CANL`.
    pub fn differential(&self) -> f32 {
        self.can_h - self.can_l
    }
}

/// A two channel probe on a CAN pair.
#[derive(Clone, Debug)]
pub struct Oscilloscope {
    name: String,
    capacity: usize,
    window_ns: u64,
    samples: VecDeque<ScopeSample>,
    /// Set while the trace is frozen.
    frozen: bool,
    /// Rebuilds the frames the sampled levels belong to.
    decoder: TraceDecoder,
}

impl Default for Oscilloscope {
    fn default() -> Self {
        Oscilloscope::new("示波器")
    }
}

impl Oscilloscope {
    /// Channel names, in drawing order.
    pub const CHANNELS: [&'static str; 2] = ["CANH", "CANL"];

    pub fn new(name: &str) -> Self {
        Oscilloscope {
            name: name.to_string(),
            capacity: DEFAULT_CAPACITY,
            window_ns: DEFAULT_WINDOW_NS,
            samples: VecDeque::new(),
            frozen: false,
            decoder: TraceDecoder::new(),
        }
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn set_name(&mut self, name: &str) {
        self.name = name.to_string();
    }

    pub fn window_ns(&self) -> u64 {
        self.window_ns
    }

    pub fn set_window_ns(&mut self, window_ns: u64) {
        self.window_ns = window_ns.clamp(10_000, 500_000_000);
        self.trim();
    }

    pub fn frozen(&self) -> bool {
        self.frozen
    }

    pub fn set_frozen(&mut self, frozen: bool) {
        self.frozen = frozen;
    }

    /// Adds one sample, dropping everything that fell out of the window.
    pub fn record(&mut self, sample: ScopeSample) {
        if self.frozen {
            return;
        }
        self.decoder.feed(sample.dominant, sample.time_ns);
        self.samples.push_back(sample);
        self.trim();
    }

    /// Frames rebuilt from the sampled bits, oldest first.
    pub fn frames(&self) -> &VecDeque<TraceFrame> {
        self.decoder.frames()
    }

    /// `true` while a frame is being shifted in.
    pub fn decoding(&self) -> bool {
        self.decoder.active()
    }

    /// Frozen scopes keep what they have; running ones forget old samples.
    fn trim(&mut self) {
        let Some(last) = self.samples.back().map(|sample| sample.time_ns) else {
            return;
        };
        let oldest = last.saturating_sub(self.window_ns);
        while self
            .samples
            .front()
            .is_some_and(|sample| sample.time_ns < oldest)
        {
            self.samples.pop_front();
        }
        while self.samples.len() > self.capacity {
            self.samples.pop_front();
        }
    }

    pub fn samples(&self) -> &VecDeque<ScopeSample> {
        &self.samples
    }

    pub fn clear(&mut self) {
        self.samples.clear();
        self.decoder.clear();
    }

    /// Simulated time covered by the stored trace, `None` when it is empty.
    pub fn span_ns(&self) -> Option<(u64, u64)> {
        let first = self.samples.front()?.time_ns;
        let last = self.samples.back()?.time_ns;
        Some((first, last))
    }

    /// Mean time between two stored samples, used to scale the time axis.
    pub fn sample_step_ns(&self) -> u64 {
        let Some((first, last)) = self.span_ns() else {
            return 1;
        };
        let steps = self.samples.len().saturating_sub(1).max(1) as u64;
        ((last - first) / steps).max(1)
    }

    /// Peak differential voltage of the stored trace.
    pub fn peak_differential(&self) -> f32 {
        self.samples
            .iter()
            .fold(0.0_f32, |peak, sample| peak.max(sample.differential()))
    }

    /// Number of samples the wire was too slow (or too poorly terminated) to
    /// carry cleanly.
    pub fn glitches(&self) -> u64 {
        self.samples.iter().filter(|sample| sample.glitch).count() as u64
    }

    /// Labels of the two channels with their current level, for the panel.
    pub fn channel_levels(&self) -> [Option<(f32, f32)>; 2] {
        let Some(sample) = self.samples.back() else {
            return [None, None];
        };
        [
            Some((sample.can_h, sample.differential())),
            Some((sample.can_l, sample.differential())),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample(time_ns: u64, dominant: bool) -> ScopeSample {
        ScopeSample {
            time_ns,
            can_h: if dominant { 3.5 } else { 2.5 },
            can_l: if dominant { 1.5 } else { 2.5 },
            dominant,
            glitch: false,
        }
    }

    #[test]
    fn samples_older_than_the_window_are_dropped() {
        let mut scope = Oscilloscope::new("s");
        scope.set_window_ns(20_000);
        for step in 0..10 {
            scope.record(sample(step * 5_000, true));
        }
        let (first, last) = scope.span_ns().expect("trace");
        assert!(last - first <= 20_000, "{first}..{last}");
        assert!(scope.samples().len() < 10);
    }

    #[test]
    fn the_differential_follows_the_bits() {
        let dominant = sample(0, true);
        assert!((dominant.differential() - 2.0).abs() < 1e-6);
        let recessive = sample(0, false);
        assert!(recessive.differential().abs() < 1e-6);
    }

    #[test]
    fn a_frozen_scope_keeps_its_trace() {
        let mut scope = Oscilloscope::new("s");
        scope.record(sample(0, true));
        scope.set_frozen(true);
        scope.record(sample(10, false));
        assert_eq!(scope.samples().len(), 1);
    }

    #[test]
    fn the_peak_reports_the_strongest_level() {
        let mut scope = Oscilloscope::new("s");
        scope.record(sample(0, false));
        scope.record(sample(10, true));
        assert!((scope.peak_differential() - 2.0).abs() < 1e-6);
    }

    #[test]
    fn sample_step_is_never_zero() {
        let scope = Oscilloscope::new("s");
        assert_eq!(scope.sample_step_ns(), 1);
        let mut scope = Oscilloscope::new("s");
        scope.record(sample(0, true));
        scope.record(sample(5_000, false));
        assert_eq!(scope.sample_step_ns(), 5_000);
    }

    /// Feeds the whole burst of `frame` the way the bridge does: one sample
    /// per bit, with a receiver pulling the ACK slot dominant.
    fn feed_frame(scope: &mut Oscilloscope, frame: gsim_core::CanFrame) {
        use crate::can_bus::WireLevels;
        use gsim_core::can::{WireFrame, DOMINANT};

        let wire = WireFrame::encode(&frame);
        let step = 5_000;
        for (index, bit) in wire.bits().iter().enumerate() {
            let level = if index == wire.ack_slot() {
                DOMINANT
            } else {
                *bit
            };
            let levels = WireLevels::of(level);
            scope.record(ScopeSample {
                time_ns: index as u64 * step,
                can_h: levels.can_h,
                can_l: levels.can_l,
                dominant: level == DOMINANT,
                glitch: false,
            });
        }
    }

    #[test]
    fn the_probe_captions_the_bits_of_a_frame_it_captured() {
        let mut scope = Oscilloscope::new("s");
        scope.set_window_ns(10_000_000);
        let frame = gsim_core::CanFrame::standard(0x321, b"GSIM");
        feed_frame(&mut scope, frame);

        let frames = scope.frames();
        assert_eq!(frames.len(), 1, "the probe decodes the burst it saw");
        let trace = &frames[0];
        assert!(trace.complete);
        assert!(trace.crc_ok);
        assert!(trace.acknowledged);
        assert_eq!(trace.frame, Some(frame));
        assert_eq!(trace.bits.len(), scope.samples().len());
        assert_eq!(trace.bits[0].caption(), "SOF");
        assert!(trace.bits.iter().any(|bit| bit.caption() == "数据31"));
        assert!(trace.bits.iter().any(|bit| bit.caption() == "EOF0"));
        assert!(trace.end_ns > trace.start_ns);
        assert!(!scope.decoding());
    }

    #[test]
    fn a_frozen_probe_stops_decoding_and_clearing_drops_the_frames() {
        let mut scope = Oscilloscope::new("s");
        scope.set_window_ns(10_000_000);
        feed_frame(&mut scope, gsim_core::CanFrame::standard(0x100, &[1]));
        assert_eq!(scope.frames().len(), 1);

        scope.set_frozen(true);
        feed_frame(&mut scope, gsim_core::CanFrame::standard(0x200, &[2]));
        assert_eq!(scope.frames().len(), 1);

        scope.set_frozen(false);
        scope.clear();
        assert!(scope.frames().is_empty());
        assert!(scope.samples().is_empty());
    }
}
