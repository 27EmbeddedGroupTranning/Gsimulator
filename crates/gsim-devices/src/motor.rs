//! CAN controlled motor.
//!
//! The part is a small gear motor with an integrated driver sitting on the
//! bus. It listens to exactly one standard data frame - the command frame - and
//! follows it:
//!
//! * `data[0]` and `data[1]` are the target speed in rpm as a **big endian
//!   signed 16 bit** number: positive turns the shaft one way, negative the
//!   other, zero lets it coast to a stop,
//! * every other identifier, and any frame that is remote or carries fewer
//!   than two data bytes, is ignored.
//!
//! The shaft ramps towards the target with a fixed acceleration instead of
//! jumping, which is what the schematic draws, and the rotation is integrated
//! over the simulated time the bridge feeds it.
//!
//! Like every other off-chip part it only sees the wire: it registers itself as
//! a bit level node on the [`CanBus`], acknowledges the frames it decodes and
//! never touches the controller's registers.

use std::cell::RefCell;
use std::rc::Rc;

use gsim_core::can::CanFrame;

use crate::can_bus::{CanBus, NodeId};

/// Fastest the shaft may turn in either direction, in rpm.
pub const MAX_RPM: f32 = 3_000.0;
/// Default acceleration, in rpm per second (half a second to full speed).
pub const DEFAULT_ACCEL_RPM_PER_S: f32 = 6_000.0;
/// Identifier the driver listens to out of the box.
pub const DEFAULT_COMMAND_ID: u16 = 0x200;

/// Which way the shaft is turning.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Spin {
    Stopped,
    Forward,
    Reverse,
}

impl Spin {
    pub fn label(self) -> &'static str {
        match self {
            Spin::Stopped => "停转",
            Spin::Forward => "正转",
            Spin::Reverse => "反转",
        }
    }
}

/// A motor with a CAN driver on the virtual bus.
#[derive(Debug)]
pub struct Motor {
    bus: Rc<RefCell<CanBus>>,
    node: NodeId,
    name: String,
    command_id: u16,
    /// Speed the last command asked for, in rpm (signed).
    target_rpm: f32,
    /// Speed the shaft actually turns at, in rpm (signed).
    rpm: f32,
    accel_rpm_per_s: f32,
    /// Revolutions the shaft turned since the part was placed.
    turns: f64,
    /// Frames the node accepted from the wire.
    frames: u64,
    /// Frames that carried a command this motor follows.
    commands: u64,
    /// `false` while the part is unpowered or off the pair.
    enabled: bool,
    /// Simulated time of the last integration step.
    last_ns: Option<u64>,
}

impl Motor {
    /// Attaches a motor to `bus` under `name`, registering its bus node.
    pub fn attach(bus: Rc<RefCell<CanBus>>, name: &str) -> Self {
        let node = bus.borrow_mut().add_peer(name);
        Motor {
            bus,
            node,
            name: name.to_string(),
            command_id: DEFAULT_COMMAND_ID,
            target_rpm: 0.0,
            rpm: 0.0,
            accel_rpm_per_s: DEFAULT_ACCEL_RPM_PER_S,
            turns: 0.0,
            frames: 0,
            commands: 0,
            enabled: true,
            last_ns: None,
        }
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn node(&self) -> NodeId {
        self.node
    }

    /// Identifier of the command frame.
    pub fn command_id(&self) -> u16 {
        self.command_id
    }

    /// Standard identifiers only: the driver has a single 11 bit filter.
    pub fn set_command_id(&mut self, command_id: u16) {
        self.command_id = command_id & 0x7FF;
    }

    pub fn enabled(&self) -> bool {
        self.enabled
    }

    /// Powers the part up or down; an off part neither acknowledges nor hears
    /// anything and its shaft coasts to a stop.
    pub fn set_enabled(&mut self, enabled: bool) {
        if self.enabled == enabled {
            return;
        }
        self.enabled = enabled;
        self.bus.borrow_mut().set_peer_enabled(self.node, enabled);
    }

    pub fn target_rpm(&self) -> f32 {
        self.target_rpm
    }

    pub fn rpm(&self) -> f32 {
        self.rpm
    }

    pub fn accel_rpm_per_s(&self) -> f32 {
        self.accel_rpm_per_s
    }

    pub fn set_accel_rpm_per_s(&mut self, accel: f32) {
        self.accel_rpm_per_s = accel.max(1.0);
    }

    /// Revolutions turned since the part was placed (signed).
    pub fn turns(&self) -> f64 {
        self.turns
    }

    /// Shaft angle in degrees, `0` pointing up.
    pub fn angle_deg(&self) -> f32 {
        (self.turns.rem_euclid(1.0) * 360.0) as f32
    }

    /// Frames the node accepted from the wire.
    pub fn frames(&self) -> u64 {
        self.frames
    }

    /// Frames that carried a command this motor follows.
    pub fn commands(&self) -> u64 {
        self.commands
    }

    pub fn spin(&self) -> Spin {
        if self.rpm > 0.5 {
            Spin::Forward
        } else if self.rpm < -0.5 {
            Spin::Reverse
        } else {
            Spin::Stopped
        }
    }

    /// Asks for a speed without a frame, used by the panel's shortcut.
    pub fn set_target_rpm(&mut self, rpm: f32) {
        self.target_rpm = rpm.clamp(-MAX_RPM, MAX_RPM);
    }

    /// Stops the shaft as if a "zero rpm" command had arrived.
    pub fn stop(&mut self) {
        self.set_target_rpm(0.0);
    }

    /// Decodes one frame; returns `true` when it carried a command.
    pub fn on_frame(&mut self, frame: &CanFrame) -> bool {
        if frame.extended || frame.remote || frame.dlc < 2 || frame.id != u32::from(self.command_id)
        {
            return false;
        }
        let rpm = i16::from_be_bytes([frame.data[0], frame.data[1]]);
        self.set_target_rpm(f32::from(rpm));
        self.commands += 1;
        true
    }

    /// Drains the frames the bus decoded for this node and advances the shaft
    /// to `now_ns`.
    pub fn service(&mut self, now_ns: u64) {
        let frames = self.bus.borrow_mut().take_received(self.node);
        self.frames += frames.len() as u64;
        if self.enabled {
            for frame in &frames {
                self.on_frame(frame);
            }
        }
        let target = if self.enabled { self.target_rpm } else { 0.0 };
        self.advance(now_ns, target);
    }

    fn advance(&mut self, now_ns: u64, target: f32) {
        let Some(last) = self.last_ns else {
            self.last_ns = Some(now_ns);
            return;
        };
        self.last_ns = Some(now_ns);
        let dt = now_ns.saturating_sub(last) as f64 / 1e9;
        if dt <= 0.0 {
            return;
        }
        let accel = f64::from(self.accel_rpm_per_s);
        let previous = f64::from(self.rpm);
        let delta = f64::from(target) - previous;
        // Exact integral of "ramp at `accel`, then hold": the shaft either
        // reaches the target inside this step or is still on the ramp.
        let (rpm, travelled) = if delta.abs() <= accel * dt {
            let ramp = if accel > 0.0 { delta.abs() / accel } else { dt };
            let ramp_part = (previous + f64::from(target)) / 2.0 * ramp;
            (target, ramp_part + f64::from(target) * (dt - ramp))
        } else {
            let next = previous + accel * dt * delta.signum();
            (next as f32, (previous + next) / 2.0 * dt)
        };
        self.rpm = rpm;
        self.turns += travelled / 60.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn motor() -> Motor {
        Motor::attach(Rc::new(RefCell::new(CanBus::new())), "M1")
    }

    fn command(id: u16, rpm: i16) -> CanFrame {
        let [high, low] = rpm.to_be_bytes();
        CanFrame::standard(id, &[high, low])
    }

    #[test]
    fn the_command_frame_sets_the_target_speed() {
        let mut motor = motor();
        assert_eq!(motor.command_id(), DEFAULT_COMMAND_ID);
        assert!(motor.on_frame(&command(0x200, 1200)));
        assert_eq!(motor.target_rpm(), 1200.0);
        assert_eq!(motor.commands(), 1);
        // Other identifiers belong to other parts on the bus.
        assert!(!motor.on_frame(&command(0x201, 900)));
        assert_eq!(motor.target_rpm(), 1200.0);
        assert_eq!(motor.commands(), 1);
    }

    #[test]
    fn a_negative_command_reverses_the_shaft() {
        let mut motor = motor();
        assert!(motor.on_frame(&command(0x200, -800)));
        assert_eq!(motor.target_rpm(), -800.0);
        motor.service(0);
        motor.service(100_000_000);
        assert!(motor.rpm() < 0.0);
        assert!(motor.turns() < 0.0);
        assert_eq!(motor.spin(), Spin::Reverse);
    }

    #[test]
    fn the_command_is_clamped_to_the_maximum_speed() {
        let mut motor = motor();
        assert!(motor.on_frame(&command(0x200, i16::MAX)));
        assert_eq!(motor.target_rpm(), MAX_RPM);
        assert!(motor.on_frame(&command(0x200, i16::MIN)));
        assert_eq!(motor.target_rpm(), -MAX_RPM);
    }

    #[test]
    fn frames_that_carry_no_command_are_ignored() {
        let mut motor = motor();
        assert!(!motor.on_frame(&CanFrame::standard(0x200, &[0x01])));
        assert!(!motor.on_frame(&CanFrame::remote(CanFrame::standard(0x200, &[1, 2]))));
        assert!(!motor.on_frame(&CanFrame::extended(0x200, &[1, 2])));
        assert_eq!(motor.commands(), 0);
        assert_eq!(motor.target_rpm(), 0.0);
    }

    #[test]
    fn the_shaft_ramps_up_instead_of_jumping() {
        let mut motor = motor();
        motor.on_frame(&command(0x200, 3000));
        motor.service(0);
        // 100 ms at 6000 rpm/s: a quarter of the way to 3000 rpm.
        motor.service(100_000_000);
        assert!((motor.rpm() - 600.0).abs() < 1e-3, "{}", motor.rpm());
        // Half a second later the shaft runs at the speed that was asked for.
        for step in 1..=5 {
            motor.service(100_000_000 * (step + 1));
        }
        assert!((motor.rpm() - 3000.0).abs() < 1e-3, "{}", motor.rpm());
        // Half a second of ramp (average 1500 rpm = 25 turns/s) plus 100 ms at
        // full speed (50 turns/s) is 17.5 turns.
        assert!((motor.turns() - 17.5).abs() < 0.1, "{}", motor.turns());
        assert_eq!(motor.spin(), Spin::Forward);
    }

    #[test]
    fn a_zero_command_lets_the_shaft_coast_to_a_stop() {
        let mut motor = motor();
        motor.on_frame(&command(0x200, 1200));
        for step in 0..=10 {
            motor.service(step * 50_000_000);
        }
        assert!((motor.rpm() - 1200.0).abs() < 1e-3, "{}", motor.rpm());
        motor.on_frame(&command(0x200, 0));
        for step in 11..=21 {
            motor.service(step * 50_000_000);
        }
        assert!(motor.rpm().abs() < 1e-3, "{}", motor.rpm());
        assert_eq!(motor.spin(), Spin::Stopped);
    }

    #[test]
    fn an_unpowered_motor_hears_nothing_and_coasts() {
        let mut motor = motor();
        motor.on_frame(&command(0x200, 3000));
        for step in 0..=5 {
            motor.service(step * 100_000_000);
        }
        let rolling = motor.rpm();
        assert!(rolling > 2500.0, "{rolling}");

        motor.set_enabled(false);
        assert!(!motor.enabled());
        let commands = motor.commands();
        for step in 6..=12 {
            motor.service(step * 100_000_000);
        }
        // The shaft slows down even though the target is still 3000 rpm.
        assert!(motor.rpm() < rolling, "{} < {rolling}", motor.rpm());
        assert_eq!(motor.commands(), commands);
        assert_eq!(motor.target_rpm(), 3000.0);
    }

    #[test]
    fn the_angle_follows_the_revolutions() {
        let mut motor = motor();
        // A shaft that reaches its speed at once keeps the arithmetic readable.
        motor.set_accel_rpm_per_s(1.0e9);
        motor.set_target_rpm(600.0);
        motor.service(0);
        // 600 rpm is ten revolutions per second: after 250 ms the shaft is two
        // and a half turns in, so its mark points the other way.
        motor.service(250_000_000);
        assert!((motor.turns() - 2.5).abs() < 1e-4, "{}", motor.turns());
        assert!(
            (motor.angle_deg() - 180.0).abs() < 1e-2,
            "{}",
            motor.angle_deg()
        );
    }
}
