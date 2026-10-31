//! Register level tests of the bxCAN controller, written as the sequences the
//! STM32 HAL performs so they double as documentation.

mod common;

use common::*;
use gsim_core::can::CanDirection;
use gsim_core::{AccessWidth, Event, MemoryBus};
use gsim_stm32f1::map;

const BASE: u32 = map::can::BASE_1;

/// `MCR`/`MSR` bits used by the tests.
const MCR_INRQ: u32 = 1 << 0;
const MCR_NART: u32 = 1 << 4;
const MCR_RFLM: u32 = 1 << 3;
const MSR_INAK: u32 = 1 << 0;
const TSR_TME0: u32 = 1 << 26;
const TSR_TXOK0: u32 = 1 << 1;
const TSR_RQCP0: u32 = 1 << 0;
const RF_FMP: u32 = 0b11;
const RF_FULL: u32 = 1 << 2;
const RF_FOVR: u32 = 1 << 3;
const RF_RFOM: u32 = 1 << 5;
const IER_TMEIE: u32 = 1 << 0;
const IER_FMPIE0: u32 = 1 << 1;
const BTR_LBKM: u32 = 1 << 30;

/// `CAN_MODE_LOOPBACK`, 36 MHz PCLK1 / 36, BS1 2TQ, BS2 2TQ: 200 kbit/s.
fn timing(loopback: bool) -> u32 {
    (2 - 1) << 16 | (2 - 1) << 20 | (36 - 1) | if loopback { BTR_LBKM } else { 0 }
}

/// Runs `HAL_CAN_Init` + `HAL_CAN_Start` for a single 32 bit mask filter that
/// accepts standard identifier `0x123` into FIFO0.
fn initialise(soc: &mut gsim_stm32f1::Soc, loopback: bool) {
    // 72 MHz SYSCLK, PCLK1 = 36 MHz: the CAN bit timing below then gives
    // 200 kbit/s, the same configuration the CubeMX firmware uses.
    configure_72mhz(soc);
    enable_apb1(soc, 1 << 25);
    w32(soc, BASE + map::can::MCR, MCR_INRQ);
    wait_for_bit(soc, BASE + map::can::MSR, MSR_INAK, true, 10_000);
    w32(soc, BASE + map::can::MCR, MCR_INRQ | MCR_NART);
    w32(soc, BASE + map::can::BTR, timing(loopback));

    // Filter bank 0: 32 bit scale, mask mode, FIFO0, STID = 0x123.
    w32(soc, BASE + map::can::FMR, 1);
    w32(soc, BASE + map::can::FS1R, 1);
    w32(soc, BASE + map::can::FM1R, 0);
    w32(soc, BASE + map::can::FFA1R, 0);
    w32(soc, BASE + map::can::FILTER_BASE, 0x123 << 21);
    w32(soc, BASE + map::can::FILTER_BASE + 4, 0x7FF << 21);
    w32(soc, BASE + map::can::FA1R, 1);
    w32(soc, BASE + map::can::FMR, 0);

    // HAL_CAN_Start: leave initialisation mode.
    w32(soc, BASE + map::can::MCR, MCR_NART);
    wait_for_bit(soc, BASE + map::can::MSR, MSR_INAK, false, 10_000);
}

/// `HAL_CAN_AddTxMessage` for a standard data frame.
fn send(soc: &mut gsim_stm32f1::Soc, id: u32, data: &[u8]) {
    let tsr = r32(soc, BASE + map::can::TSR);
    assert!(tsr & TSR_TME0 != 0, "mailbox 0 is busy");
    let mut low = 0u32;
    let mut high = 0u32;
    for index in 0..4 {
        low |= u32::from(*data.get(index).unwrap_or(&0)) << (index * 8);
        high |= u32::from(*data.get(index + 4).unwrap_or(&0)) << (index * 8);
    }
    w32(soc, BASE + map::can::TX_BASE, id << 21);
    w32(soc, BASE + map::can::TX_BASE + map::can::TDTR, data.len() as u32);
    w32(soc, BASE + map::can::TX_BASE + map::can::TDHR, high);
    w32(soc, BASE + map::can::TX_BASE + map::can::TDLR, low);
    // TXRQ is the last write, exactly like the HAL does.
    w32(
        soc,
        BASE + map::can::TX_BASE,
        (id << 21) | 1,
    );
}

/// `HAL_CAN_GetRxMessage` for FIFO0.
fn receive(soc: &mut gsim_stm32f1::Soc) -> (u32, u32, Vec<u8>) {
    let rir = r32(soc, BASE + map::can::RX_BASE);
    let rdtr = r32(soc, BASE + map::can::RX_BASE + map::can::RDTR);
    let dlc = rdtr & 0xF;
    let low = r32(soc, BASE + map::can::RX_BASE + map::can::RDLR);
    let high = r32(soc, BASE + map::can::RX_BASE + map::can::RDHR);
    let mut data = Vec::new();
    for index in 0..dlc {
        let word = if index < 4 { low } else { high };
        data.push(((word >> ((index % 4) * 8)) & 0xFF) as u8);
    }
    w32(soc, BASE + map::can::RF0R, RF_RFOM);
    ((rir & (0x7FF << 21)) >> 21, dlc, data)
}

#[test]
fn initialisation_mode_is_reported_through_inak() {
    let mut soc = gsim_stm32f1::Soc::new();
    enable_apb1(&mut soc, 1 << 25);

    assert_eq!(r32(&mut soc, BASE + map::can::MSR) & MSR_INAK, 0);
    w32(&mut soc, BASE + map::can::MCR, MCR_INRQ);
    assert_eq!(r32(&mut soc, BASE + map::can::MSR) & MSR_INAK, MSR_INAK);

    w32(&mut soc, BASE + map::can::BTR, timing(false));
    assert_eq!(r32(&mut soc, BASE + map::can::BTR), timing(false));

    w32(&mut soc, BASE + map::can::MCR, 0);
    assert_eq!(r32(&mut soc, BASE + map::can::MSR) & MSR_INAK, 0);
}

#[test]
fn bit_timing_register_is_only_writable_in_initialisation_mode() {
    let mut soc = gsim_stm32f1::Soc::new();
    enable_apb1(&mut soc, 1 << 25);
    w32(&mut soc, BASE + map::can::MCR, MCR_INRQ);
    w32(&mut soc, BASE + map::can::BTR, timing(false));
    w32(&mut soc, BASE + map::can::MCR, 0);

    w32(&mut soc, BASE + map::can::BTR, 0x1234);
    assert_eq!(r32(&mut soc, BASE + map::can::BTR), timing(false));
}

#[test]
fn loopback_frame_is_received_after_its_wire_time() {
    let mut soc = gsim_stm32f1::Soc::new();
    let log = EventLog::new();
    log.attach(&mut soc);
    initialise(&mut soc, true);

    send(&mut soc, 0x123, b"Hello");
    assert_eq!(r32(&mut soc, BASE + map::can::RF0R) & RF_FMP, 0);
    log.clear();

    // 87 bits at 5 us each is ~435 us.
    soc.advance_ns(500_000);

    let events = log.events();
    assert!(events.iter().any(|event| matches!(
        event,
        Event::CanFrame {
            direction: CanDirection::Loopback,
            frame,
            ..
        } if frame.id == 0x123 && frame.bytes() == b"Hello"
    )));
    assert_eq!(r32(&mut soc, BASE + map::can::RF0R) & RF_FMP, 1);
    let tsr = r32(&mut soc, BASE + map::can::TSR);
    assert_eq!(tsr & (TSR_RQCP0 | TSR_TXOK0 | TSR_TME0), TSR_RQCP0 | TSR_TXOK0 | TSR_TME0);

    let (id, dlc, data) = receive(&mut soc);
    assert_eq!(id, 0x123);
    assert_eq!(dlc, 5);
    assert_eq!(data, b"Hello");
    assert_eq!(r32(&mut soc, BASE + map::can::RF0R) & RF_FMP, 0);
}

#[test]
fn acceptance_filter_drops_frames_with_another_identifier() {
    let mut soc = gsim_stm32f1::Soc::new();
    initialise(&mut soc, true);
    send(&mut soc, 0x456, b"nope");
    soc.advance_ns(1_000_000);
    assert_eq!(r32(&mut soc, BASE + map::can::RF0R) & RF_FMP, 0);
    assert_eq!(soc.can.stats().tx_frames, 0);
}

#[test]
fn receive_fifo_raises_the_rx0_interrupt() {
    let mut soc = gsim_stm32f1::Soc::new();
    initialise(&mut soc, true);
    w32(&mut soc, BASE + map::can::IER, IER_FMPIE0);

    let frame = gsim_core::CanFrame::standard(0x123, b"abcd");
    assert!(soc.can_receive(0, frame));
    assert_eq!(r32(&mut soc, BASE + map::can::RF0R) & RF_FMP, 1);
    assert!(soc.nvic.is_pending(gsim_core::irq::lines::CAN1_RX0));

    // Releasing the frame drops the interrupt line again.
    w32(&mut soc, BASE + map::can::RF0R, RF_RFOM);
    assert!(!soc.nvic.is_pending(gsim_core::irq::lines::CAN1_RX0));
}

#[test]
fn rx_fifo_overrun_is_flagged_and_the_oldest_frame_is_dropped() {
    let mut soc = gsim_stm32f1::Soc::new();
    initialise(&mut soc, true);
    for value in 0..4u8 {
        let frame = gsim_core::CanFrame::standard(0x123, &[value]);
        soc.can_receive(0, frame);
    }
    let status = r32(&mut soc, BASE + map::can::RF0R);
    assert_eq!(status & RF_FMP, 3);
    assert_eq!(status & RF_FULL, RF_FULL);
    assert_eq!(status & RF_FOVR, RF_FOVR);

    // The oldest frame was overwritten, so the first one read is data 1.
    let (_, _, data) = receive(&mut soc);
    assert_eq!(data, vec![1]);

    // Clearing the overrun flag by writing one.
    w32(&mut soc, BASE + map::can::RF0R, RF_FOVR);
    assert_eq!(r32(&mut soc, BASE + map::can::RF0R) & RF_FOVR, 0);
}

#[test]
fn locked_fifo_refuses_new_frames_instead_of_overwriting() {
    let mut soc = gsim_stm32f1::Soc::new();
    initialise(&mut soc, true);
    w32(&mut soc, BASE + map::can::MCR, MCR_INRQ | MCR_RFLM);
    w32(&mut soc, BASE + map::can::MCR, MCR_RFLM);
    for value in 0..4u8 {
        soc.can_receive(0, gsim_core::CanFrame::standard(0x123, &[value]));
    }
    assert_eq!(r32(&mut soc, BASE + map::can::RF0R) & RF_FMP, 3);
    assert_eq!(r32(&mut soc, BASE + map::can::RF0R) & RF_FOVR, 0);
    let (_, _, data) = receive(&mut soc);
    assert_eq!(data, vec![0]);
}

#[test]
fn aborting_a_mailbox_frees_it_and_counts_the_frame() {
    let mut soc = gsim_stm32f1::Soc::new();
    initialise(&mut soc, true);
    send(&mut soc, 0x123, b"abc");
    assert_eq!(r32(&mut soc, BASE + map::can::TSR) & TSR_TME0, 0);

    w32(&mut soc, BASE + map::can::TSR, 1 << 7); // ABRQ0
    assert_eq!(r32(&mut soc, BASE + map::can::TSR) & TSR_TME0, TSR_TME0);
    assert_eq!(soc.can.stats().aborted, 1);
}

#[test]
fn normal_mode_reports_frames_on_the_transmit_pin() {
    let mut soc = gsim_stm32f1::Soc::new();
    let log = EventLog::new();
    log.attach(&mut soc);
    initialise(&mut soc, false);
    w32(&mut soc, BASE + map::can::IER, IER_TMEIE);

    send(&mut soc, 0x123, b"Hello");
    soc.advance_ns(500_000);

    let events = log.events();
    assert!(events.iter().any(|event| matches!(
        event,
        Event::CanFrame {
            direction: CanDirection::Tx,
            frame,
            ..
        } if frame.id == 0x123
    )));
    // Nothing was looped back internally in normal mode.
    assert_eq!(r32(&mut soc, BASE + map::can::RF0R) & RF_FMP, 0);
    assert_eq!(soc.can.stats().tx_frames, 1);
}

#[test]
fn transmit_interrupt_fires_when_the_last_mailbox_empties() {
    let mut soc = gsim_stm32f1::Soc::new();
    initialise(&mut soc, true);
    w32(&mut soc, BASE + map::can::IER, IER_TMEIE);
    assert!(soc.nvic.is_pending(gsim_core::irq::lines::CAN1_TX));

    send(&mut soc, 0x123, b"Hello");
    assert!(!soc.nvic.is_pending(gsim_core::irq::lines::CAN1_TX));
    soc.advance_ns(500_000);
    assert!(soc.nvic.is_pending(gsim_core::irq::lines::CAN1_TX));
}

#[test]
fn the_controller_does_not_advance_without_its_apb1_clock() {
    let mut soc = gsim_stm32f1::Soc::new();
    initialise(&mut soc, true);
    send(&mut soc, 0x123, b"Hello");

    // Turning the CAN1 clock off stops the transfer mid frame.
    w32(&mut soc, map::rcc::BASE + map::rcc::APB1ENR, 0);
    soc.advance_ns(5_000_000);
    assert_eq!(r32(&mut soc, BASE + map::can::RF0R) & RF_FMP, 0);

    enable_apb1(&mut soc, 1 << 25);
    soc.advance_ns(500_000);
    assert_eq!(r32(&mut soc, BASE + map::can::RF0R) & RF_FMP, 1);
}

#[test]
fn reset_returns_the_controller_to_its_power_on_state() {
    let mut soc = gsim_stm32f1::Soc::new();
    initialise(&mut soc, true);
    w32(&mut soc, BASE + map::can::IER, IER_FMPIE0);
    send(&mut soc, 0x123, b"Hello");
    soc.advance_ns(500_000);

    // MCR.RESET is the software reset the HAL uses on de-initialisation.
    w32(&mut soc, BASE + map::can::MCR, 1 << 15);
    assert_eq!(r32(&mut soc, BASE + map::can::BTR), 0);
    assert_eq!(r32(&mut soc, BASE + map::can::IER), 0);
    assert_eq!(r32(&mut soc, BASE + map::can::RF0R) & RF_FMP, 0);
}

#[test]
fn rejects_access_beyond_the_controller_window() {
    let mut soc = gsim_stm32f1::Soc::new();
    let error = soc
        .read(BASE + 0x300, AccessWidth::Word)
        .expect_err("0x300 is not a CAN register");
    assert!(matches!(error, gsim_core::BusError::Unmapped { .. }));
}

#[test]
fn loopback_reception_raises_the_rx0_interrupt() {
    let mut soc = gsim_stm32f1::Soc::new();
    initialise(&mut soc, true);
    w32(&mut soc, BASE + map::can::IER, IER_FMPIE0);
    send(&mut soc, 0x123, b"Hello");
    soc.advance_ns(500_000);
    assert_eq!(r32(&mut soc, BASE + map::can::RF0R) & RF_FMP, 1);
    assert!(
        soc.nvic.is_pending(gsim_core::irq::lines::CAN1_RX0),
        "the RX0 line should be pending, MSR=0x{:08X} IER=0x{:08X}",
        r32(&mut soc, BASE + map::can::MSR),
        r32(&mut soc, BASE + map::can::IER)
    );
}