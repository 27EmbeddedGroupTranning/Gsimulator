//! Rotary encoder with quadrature `A`/`B` outputs and a push button.
//!
//! The part is an input device: it never reads the bus, it only pulls its three
//! contacts low. One mechanical detent is four quarter steps of a Gray code, so
//! a firmware that decodes both edges sees the direction, and one that polls
//! slowly still sees a valid sequence.
//!
//! Edges are scheduled instead of applied at once: [`Encoder::rotate`] queues
//! them and [`Encoder::poll`] releases them at [`Encoder::edge_interval_ns`]
//! apart, so the pins change while the firmware keeps running (the front end
//! services the device between instruction slices).

use std::collections::VecDeque;

/// One quadrature state: the level of the `A` and `B` contacts.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Phase {
    pub a: bool,
    pub b: bool,
}

impl Phase {
    /// Contacts released, both pulled up by the MCU.
    pub const IDLE: Phase = Phase { a: true, b: true };

    /// Quarter steps in clockwise order; neighbours differ by one contact.
    pub const CLOCKWISE: [Phase; 4] = [
        Phase { a: true, b: true },
        Phase { a: false, b: true },
        Phase { a: false, b: false },
        Phase { a: true, b: false },
    ];
}

/// Which way the knob was turned.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Direction {
    Clockwise,
    CounterClockwise,
}

impl Direction {
    fn delta(self) -> i32 {
        match self {
            Direction::Clockwise => 1,
            Direction::CounterClockwise => -1,
        }
    }
}

/// Quarter steps per mechanical detent.
pub const EDGES_PER_DETENT: i32 = 4;

/// A rotary encoder with detents and a push button.
#[derive(Clone, Debug)]
pub struct Encoder {
    /// Index into [`Phase::CLOCKWISE`] of the current phase.
    index: usize,
    /// Quarter steps accumulated since the last detent (0..4 in magnitude).
    partial: i32,
    detents: i32,
    pending: VecDeque<i32>,
    next_edge_ns: u64,
    edge_interval_ns: u64,
    button: bool,
    presses: u64,
    edges: u64,
}

impl Default for Encoder {
    fn default() -> Self {
        Encoder::new()
    }
}

impl Encoder {
    /// Time between two quarter steps of a turn: fast enough to feel immediate,
    /// slow enough for a firmware polling every millisecond to see every edge.
    pub const DEFAULT_EDGE_INTERVAL_NS: u64 = 2_000_000;

    pub fn new() -> Self {
        Encoder::with_edge_interval_ns(Self::DEFAULT_EDGE_INTERVAL_NS)
    }

    pub fn with_edge_interval_ns(edge_interval_ns: u64) -> Self {
        Encoder {
            index: 0,
            partial: 0,
            detents: 0,
            pending: VecDeque::new(),
            next_edge_ns: 0,
            edge_interval_ns: edge_interval_ns.max(1),
            button: false,
            presses: 0,
            edges: 0,
        }
    }

    pub fn edge_interval_ns(&self) -> u64 {
        self.edge_interval_ns
    }

    /// Levels currently present on the `A` / `B` contacts.
    pub fn phase(&self) -> Phase {
        Phase::CLOCKWISE[self.index % Phase::CLOCKWISE.len()]
    }

    /// Signed detent counter (positive = clockwise).
    pub fn detents(&self) -> i32 {
        self.detents
    }

    /// Quarter steps still waiting to be released.
    pub fn pending_edges(&self) -> usize {
        self.pending.len()
    }

    /// Total quarter steps released since power-on.
    pub fn edges(&self) -> u64 {
        self.edges
    }

    /// Button state; `true` while the shaft is pushed.
    pub fn button(&self) -> bool {
        self.button
    }

    /// Number of button presses (falling edges).
    pub fn presses(&self) -> u64 {
        self.presses
    }

    /// Queues a turn of `detents` detents, starting no earlier than `now_ns`.
    pub fn rotate(&mut self, direction: Direction, detents: u32, now_ns: u64) {
        if detents == 0 {
            return;
        }
        let delta = direction.delta();
        for _ in 0..detents * EDGES_PER_DETENT as u32 {
            self.pending.push_back(delta);
        }
        if self.next_edge_ns < now_ns {
            self.next_edge_ns = now_ns;
        }
    }

    /// Releases every quarter step that is due at `now_ns`.
    ///
    /// Returns how many edges were applied; the new contact levels are
    /// available through [`Encoder::phase`].
    pub fn poll(&mut self, now_ns: u64) -> u32 {
        let mut applied = 0;
        while let Some(delta) = self.pending.front().copied() {
            if now_ns < self.next_edge_ns {
                break;
            }
            self.pending.pop_front();
            self.apply(delta);
            self.next_edge_ns = self.next_edge_ns.saturating_add(self.edge_interval_ns);
            applied += 1;
        }
        applied
    }

    fn apply(&mut self, delta: i32) {
        let len = Phase::CLOCKWISE.len() as i32;
        self.index = (self.index as i32 + delta).rem_euclid(len) as usize;
        self.partial += delta;
        if self.partial.abs() >= EDGES_PER_DETENT {
            self.detents += self.partial.signum();
            self.partial = 0;
        }
        self.edges += 1;
    }

    /// Pushes (`true`) or releases the shaft.
    pub fn press(&mut self, pressed: bool) {
        if pressed && !self.button {
            self.presses += 1;
        }
        self.button = pressed;
    }

    /// Drops queued edges and returns the contacts to rest, as on power-up.
    pub fn reset(&mut self) {
        self.index = 0;
        self.partial = 0;
        self.detents = 0;
        self.pending.clear();
        self.next_edge_ns = 0;
        self.button = false;
        self.presses = 0;
        self.edges = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn drain(encoder: &mut Encoder, now_ns: u64, limit: u32) -> (Vec<Phase>, u64) {
        let mut phases = Vec::new();
        let mut now = now_ns;
        for _ in 0..limit {
            if encoder.poll(now) > 0 {
                phases.push(encoder.phase());
            }
            now += encoder.edge_interval_ns();
        }
        (phases, now)
    }

    #[test]
    fn a_fresh_encoder_rests_with_both_contacts_high() {
        let encoder = Encoder::new();
        assert_eq!(encoder.phase(), Phase::IDLE);
        assert_eq!(encoder.detents(), 0);
        assert!(!encoder.button());
        assert_eq!(encoder.pending_edges(), 0);
    }

    #[test]
    fn one_clockwise_detent_walks_four_quarter_steps_and_returns_near_idle() {
        let mut encoder = Encoder::new();
        encoder.rotate(Direction::Clockwise, 1, 0);
        assert_eq!(encoder.pending_edges(), 4);

        let (phases, _now) = drain(&mut encoder, 0, 8);
        assert_eq!(
            phases,
            vec![
                Phase { a: false, b: true },
                Phase { a: false, b: false },
                Phase { a: true, b: false },
                Phase::IDLE,
            ]
        );
        assert_eq!(encoder.detents(), 1);
        assert_eq!(encoder.edges(), 4);
        assert_eq!(encoder.pending_edges(), 0);
    }

    #[test]
    fn counter_clockwise_walks_the_code_backwards() {
        let mut encoder = Encoder::new();
        encoder.rotate(Direction::CounterClockwise, 1, 0);
        let (phases, _now) = drain(&mut encoder, 0, 8);
        assert_eq!(
            phases,
            vec![
                Phase { a: true, b: false },
                Phase { a: false, b: false },
                Phase { a: false, b: true },
                Phase::IDLE,
            ]
        );
        assert_eq!(encoder.detents(), -1);
    }

    #[test]
    fn edges_are_spaced_in_time_instead_of_applied_at_once() {
        let mut encoder = Encoder::with_edge_interval_ns(1_000_000);
        encoder.rotate(Direction::Clockwise, 1, 0);
        // Due immediately, then only once the interval has elapsed.
        assert_eq!(encoder.poll(0), 1);
        assert_eq!(encoder.poll(0), 0);
        assert_eq!(encoder.poll(999_999), 0);
        assert_eq!(encoder.poll(1_000_000), 1);
    }

    #[test]
    fn neighbouring_phases_always_differ_by_exactly_one_contact() {
        let phases = Phase::CLOCKWISE;
        for index in 0..phases.len() {
            let current = phases[index];
            let next = phases[(index + 1) % phases.len()];
            let changed = u8::from(current.a != next.a) + u8::from(current.b != next.b);
            assert_eq!(changed, 1, "phase {index} is not a Gray code step");
            assert_ne!(current, next);
        }
    }

    #[test]
    fn several_detents_count_up_and_down() {
        let mut encoder = Encoder::new();
        encoder.rotate(Direction::Clockwise, 3, 0);
        drain(&mut encoder, 0, 20);
        assert_eq!(encoder.detents(), 3);

        let now = 100_000_000;
        encoder.rotate(Direction::CounterClockwise, 5, now);
        drain(&mut encoder, now, 30);
        assert_eq!(encoder.detents(), -2);
        assert_eq!(encoder.pending_edges(), 0);
    }

    #[test]
    fn the_button_tracks_presses() {
        let mut encoder = Encoder::new();
        encoder.press(true);
        encoder.press(true);
        assert!(encoder.button());
        assert_eq!(encoder.presses(), 1);
        encoder.press(false);
        assert!(!encoder.button());
        encoder.press(true);
        assert_eq!(encoder.presses(), 2);
    }

    #[test]
    fn reset_returns_to_the_power_on_state() {
        let mut encoder = Encoder::new();
        encoder.rotate(Direction::Clockwise, 2, 0);
        drain(&mut encoder, 0, 10);
        encoder.press(true);
        encoder.reset();
        assert_eq!(encoder.phase(), Phase::IDLE);
        assert_eq!(encoder.detents(), 0);
        assert_eq!(encoder.pending_edges(), 0);
        assert!(!encoder.button());
    }
}
