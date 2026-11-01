//! End-to-end test of the motor the `firmware/CANSPI` demo commands over CAN.
//!
//! The drawing under test is `docs/canspi-heart.gsim`, so the schematic the
//! documentation points at is the one these assertions exercise: the firmware
//! has to clock itself from the HSE crystal, shift its heart out over SPI and
//! put the command frame on the pair where the motor decodes it.
//!
//! Only the first half cycle is walked through: the simulation runs the
//! firmware instruction by instruction, and every further heart beat costs
//! hundreds of milliseconds of simulated time. The ramp of the shaft towards
//! the commanded speed is covered by the device's own unit tests.
//!
//! The image is a build artefact: when it is missing the test only reports the
//! skip, so a fresh checkout still passes.

use std::cell::RefCell;
use std::path::PathBuf;
use std::rc::Rc;

use gsimulator_gui::board::{Board, DeviceId, DeviceKind};
use gsimulator_gui::bridge;
use gsimulator_gui::sim::Simulation;

/// Speed the demo asks for in its forward phase, in rpm.
const DEMO_RPM: i16 = 600;
/// Identifier of the motor's command frame.
const COMMAND_ID: u32 = 0x200;

fn firmware() -> Option<PathBuf> {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../firmware/CANSPI/build/Debug/CANSPI.elf");
    if path.exists() {
        Some(path)
    } else {
        eprintln!(
            "skipping: {} is missing (build firmware/CANSPI with CMake first)",
            path.display()
        );
        None
    }
}

fn drawing() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../docs/canspi-heart.gsim")
}

fn motor_of(board: &Board, id: DeviceId) -> Rc<RefCell<gsim_devices::Motor>> {
    board
        .device(id)
        .expect("the motor is placed")
        .hardware
        .motor()
        .cloned()
        .expect("motor model")
}

/// Runs slices of about a millisecond until `done` says so or the budget runs out.
fn run_until(
    board: &mut Board,
    sim: &mut Simulation,
    slices: u32,
    done: impl Fn() -> bool,
) -> bool {
    for _ in 0..slices {
        bridge::run_frame(board, sim, 200_000, 1_000_000);
        if done() {
            return true;
        }
    }
    false
}

#[test]
fn the_firmware_commands_the_motor_over_the_drawn_can_bus() {
    let Some(path) = firmware() else {
        return;
    };
    let mut board =
        gsimulator_gui::project::open(drawing()).expect("docs/canspi-heart.gsim parses");
    let motor = board
        .first_of(DeviceKind::Motor)
        .expect("the drawing places a motor")
        .id;
    let network = board.can_network();
    assert_eq!(network.motors.len(), 1, "the motor sits on the pair");
    let ohms = network.termination_ohm.expect("the drawing terminates");
    assert!((ohms - 60.0).abs() < 1e-6, "{ohms}");

    let mut sim = Simulation::new();
    sim.set_crystals(board.crystals());
    sim.load_elf(&path).expect("the firmware loads");
    sim.set_running(true);

    let handle = motor_of(&board, motor);

    // The first command leaves the mailbox as soon as CAN1 is out of
    // initialisation mode, long before the first heart beat ends.
    assert!(
        run_until(&mut board, &mut sim, 40, || handle.borrow().commands() > 0),
        "no command reached the motor; the bus carried {:?}",
        board.can_bus().borrow().stats()
    );
    {
        let motor = handle.borrow();
        assert!(motor.enabled(), "the motor is powered and on the pair");
        assert_eq!(motor.target_rpm(), f32::from(DEMO_RPM));
        assert_eq!(motor.spin(), gsim_devices::Spin::Forward);
    }

    // The frame really travelled: it was acknowledged by the motor and carries
    // the target speed as a big endian signed 16 bit number.
    {
        let bus = board.can_bus();
        let bus = bus.borrow();
        let commands: Vec<_> = bus
            .records()
            .iter()
            .filter(|record| record.frame.id == COMMAND_ID)
            .collect();
        assert!(
            !commands.is_empty(),
            "the controller never put a command frame on the wire"
        );
        assert!(
            commands.iter().any(|record| record.acknowledged),
            "nobody acknowledged the command frame"
        );
        let expected = DEMO_RPM.to_be_bytes();
        assert_eq!(commands[0].frame.bytes(), &expected[..]);
    }

    // The shaft follows the command instead of jumping to it, and the display
    // path of the same firmware is alive next to it.
    assert!(
        run_until(&mut board, &mut sim, 10, || handle.borrow().turns() > 0.0),
        "the shaft never turned: {} rpm",
        handle.borrow().rpm()
    );
    let matrix = board
        .first_of(DeviceKind::Matrix)
        .expect("the drawing places the matrix")
        .hardware
        .matrix()
        .cloned()
        .expect("matrix model");
    let matrix = matrix.borrow();
    assert!(matrix.words() > 0, "the heart never reached the matrix");
    assert!(matrix.chips()[0].enabled(), "the matrix is still shut down");
}
