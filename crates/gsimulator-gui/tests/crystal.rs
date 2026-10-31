//! End-to-end tests for the external crystals: the board decides whether the
//! oscillators exist, the firmware decides which one it clocks itself from.

use gsim_core::{AccessWidth, MemoryBus};
use gsim_stm32f1::map;
use gsimulator_gui::board::{Board, Crystals, DeviceKind};
use gsimulator_gui::sim::Simulation;

const HSEON: u32 = 1 << 16;
const HSERDY: u32 = 1 << 17;
const LSEON: u32 = 1 << 0;
const LSERDY: u32 = 1 << 1;

/// Advances simulated time by `ns` (the SoC runs at 8 MHz after power-on).
fn advance_ns(sim: &mut Simulation, ns: u64) {
    let sysclk = u64::from(sim.clocks().sysclk_hz.max(1));
    let budget = (ns.saturating_mul(sysclk) / 1_000_000_000).max(1);
    sim.advance_frame(budget as u32);
}

fn read(sim: &mut Simulation, offset: u32) -> u32 {
    sim.with_soc(|soc| soc.read(offset, AccessWidth::Word).expect("read"))
}

fn write(sim: &mut Simulation, offset: u32, value: u32) {
    sim.with_soc(|soc| soc.write(offset, AccessWidth::Word, value).expect("write"));
}

#[test]
fn hse_only_starts_when_the_crystal_is_fitted() {
    let mut sim = Simulation::new();
    sim.set_crystals(Crystals::default());
    let cr = map::rcc::BASE + map::rcc::CR;

    write(&mut sim, cr, HSEON);
    advance_ns(&mut sim, 5_000_000);
    assert_eq!(read(&mut sim, cr) & HSERDY, 0, "bare chip has no HSERDY");

    sim.set_crystals(Crystals {
        hse_hz: Some(8_000_000),
        lse: false,
    });
    advance_ns(&mut sim, 5_000_000);
    assert_ne!(
        read(&mut sim, cr) & HSERDY,
        0,
        "the crystal lets the start-up finish"
    );
}

#[test]
fn lse_only_starts_when_the_watch_crystal_is_fitted() {
    let mut sim = Simulation::new();
    sim.set_crystals(Crystals::default());
    let bdcr = map::rcc::BASE + map::rcc::BDCR;

    write(&mut sim, bdcr, LSEON);
    advance_ns(&mut sim, 3_000_000_000);
    assert_eq!(read(&mut sim, bdcr) & LSERDY, 0);

    sim.set_crystals(Crystals {
        hse_hz: None,
        lse: true,
    });
    advance_ns(&mut sim, 3_000_000_000);
    assert_ne!(read(&mut sim, bdcr) & LSERDY, 0);
}

#[test]
fn the_firmware_picks_which_oscillator_to_enable() {
    let mut sim = Simulation::new();
    sim.set_crystals(Crystals {
        hse_hz: Some(8_000_000),
        lse: true,
    });
    let cr = map::rcc::BASE + map::rcc::CR;
    let bdcr = map::rcc::BASE + map::rcc::BDCR;

    advance_ns(&mut sim, 5_000_000);
    assert_eq!(
        read(&mut sim, cr) & HSERDY,
        0,
        "HSEON is still clear on reset"
    );
    assert_eq!(
        read(&mut sim, bdcr) & LSERDY,
        0,
        "LSEON is still clear on reset"
    );

    // A HSE->PLL firmware only switches HSE on...
    write(&mut sim, cr, HSEON | 1);
    advance_ns(&mut sim, 5_000_000);
    assert_ne!(read(&mut sim, cr) & HSERDY, 0);
    // ...and leaves the RTC oscillator alone.
    assert_eq!(read(&mut sim, bdcr) & LSERDY, 0);
}

#[test]
fn running_oscillators_drive_the_crystal_pins() {
    let mut sim = Simulation::new();
    sim.set_crystals(Crystals {
        hse_hz: Some(8_000_000),
        lse: true,
    });
    let cr = map::rcc::BASE + map::rcc::CR;
    let bdcr = map::rcc::BASE + map::rcc::BDCR;
    write(&mut sim, cr, HSEON | 1);
    write(&mut sim, bdcr, LSEON);
    advance_ns(&mut sim, 5_000_000);
    assert!(sim.hse_ready());
    advance_ns(&mut sim, 3_000_000_000);
    assert!(sim.lse_ready());

    let mut hse_levels = std::collections::BTreeSet::new();
    let mut lse_levels = std::collections::BTreeSet::new();
    for _ in 0..400 {
        sim.with_soc(|soc| soc.advance_ns(20_000));
        let (hse, lse) = sim.with_soc(|soc| {
            (
                (soc.gpio[3].input_level(0), soc.gpio[3].input_level(1)),
                (soc.gpio[2].input_level(14), soc.gpio[2].input_level(15)),
            )
        });
        hse_levels.insert(hse);
        lse_levels.insert(lse);
    }

    assert_eq!(hse_levels.len(), 2, "PD0/PD1 must carry the HSE phase");
    assert!(
        hse_levels.iter().all(|(input, output)| input != output),
        "OSC_IN and OSC_OUT are in antiphase"
    );
    assert_eq!(lse_levels.len(), 2, "PC14/PC15 must carry the LSE phase");
    assert!(lse_levels.iter().all(|(input, output)| input != output));

    // Losing the crystals releases the pins instead of freezing them high.
    sim.set_crystals(Crystals::default());
    sim.with_soc(|soc| soc.advance_ns(20_000));
    let levels = sim.pin_levels();
    for (port, pin) in [(3, 0), (3, 1), (2, 14), (2, 15)] {
        assert!(!levels.level_of(port, pin), "P{port}.{pin} still driven");
    }
}

#[test]
fn the_demo_board_fits_the_hse_crystal_and_removing_it_takes_it_away() {
    let mut board = Board::demo();
    assert_eq!(
        board.crystals(),
        Crystals {
            hse_hz: Some(8_000_000),
            lse: true,
        }
    );

    let hse = board
        .first_of(DeviceKind::HseCrystal)
        .expect("demo HSE crystal")
        .id;
    board.remove_device(hse);
    assert_eq!(
        board.crystals(),
        Crystals {
            hse_hz: None,
            lse: true,
        },
        "removing the HSE crystal takes it away from the RCC"
    );
}
