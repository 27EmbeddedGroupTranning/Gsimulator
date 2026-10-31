//! End to end test of the CAN transceiver: a frame transmitted by the bxCAN
//! controller is put on the virtual bus, and a frame the bus delivers is handed
//! back to the controller's receive FIFO.

use std::cell::RefCell;
use std::rc::Rc;

use gsim_core::{AccessWidth, CanFrame, Event, MemoryBus};
use gsim_devices::{CanBus, CanTransceiver};
use gsim_stm32f1::{map, Soc};

const CAN: u32 = map::can::BASE_1;

fn write(soc: &mut Soc, address: u32, value: u32) {
    soc.write(address, AccessWidth::Word, value).unwrap();
}

fn read(soc: &mut Soc, address: u32) -> u32 {
    soc.read(address, AccessWidth::Word).unwrap()
}

/// Brings CAN1 up in normal mode with a single 32 bit mask filter for `0x123`.
fn controller() -> Soc {
    let mut soc = Soc::new();
    write(&mut soc, map::rcc::BASE + map::rcc::APB1ENR, 1 << 25);
    write(&mut soc, CAN + map::can::MCR, 1 << 0); // INRQ
    write(
        &mut soc,
        CAN + map::can::BTR,
        (2 - 1) << 16 | (2 - 1) << 20 | 35,
    );
    write(&mut soc, CAN + map::can::FMR, 1);
    write(&mut soc, CAN + map::can::FS1R, 1);
    write(&mut soc, CAN + map::can::FILTER_BASE, 0x123 << 21);
    write(&mut soc, CAN + map::can::FILTER_BASE + 4, 0x7FF << 21);
    write(&mut soc, CAN + map::can::FA1R, 1);
    write(&mut soc, CAN + map::can::FMR, 0);
    write(&mut soc, CAN + map::can::MCR, 0); // leave initialisation
    soc
}

/// `HAL_CAN_AddTxMessage` for a standard data frame.
fn transmit(soc: &mut Soc, id: u32, data: &[u8]) {
    let mut low = 0u32;
    let mut high = 0u32;
    for index in 0..4 {
        low |= u32::from(*data.get(index).unwrap_or(&0)) << (index * 8);
        high |= u32::from(*data.get(index + 4).unwrap_or(&0)) << (index * 8);
    }
    write(soc, CAN + map::can::TX_BASE, id << 21);
    write(
        soc,
        CAN + map::can::TX_BASE + map::can::TDTR,
        data.len() as u32,
    );
    write(soc, CAN + map::can::TX_BASE + map::can::TDHR, high);
    write(soc, CAN + map::can::TX_BASE + map::can::TDLR, low);
    write(soc, CAN + map::can::TX_BASE, (id << 21) | 1); // TXRQ last
}

/// Transceiver wired to CAN1 of `soc`, plus the shared bus it sits on.
fn wired(soc: &mut Soc) -> (Rc<RefCell<CanBus>>, Rc<RefCell<CanTransceiver>>) {
    let bus = Rc::new(RefCell::new(CanBus::new()));
    let transceiver = Rc::new(RefCell::new(CanTransceiver::attach(
        Rc::clone(&bus),
        "CAN1 节点",
        0,
    )));
    let sink = Rc::clone(&transceiver);
    soc.add_event_sink(Box::new(move |event: &Event| {
        sink.borrow_mut().on_event(event, 0);
    }));
    (bus, transceiver)
}

/// Steps the pair for `bits` bit times, with a second node that acknowledges
/// every frame it decodes and never starts one.
fn run_wire(soc: &mut Soc, bits: u32) {
    use gsim_core::can::{FrameDecoder, RxStep, DOMINANT, RECESSIVE};
    soc.can_set_on_bus(0, true);
    let mut decoder = FrameDecoder::new();
    let mut ack = false;
    for _ in 0..bits {
        let dominant = soc.can_driven_level(0) == Some(DOMINANT) || ack;
        let level = if dominant { DOMINANT } else { RECESSIVE };
        // The peer pulls the acknowledge slot dominant on the next bit.
        ack = matches!(decoder.feed(level), RxStep::CrcDelimiter { ok: true });
        soc.can_sample_bit(0, level);
        soc.advance_ns(1_000);
    }
    soc.can_set_on_bus(0, false);
}

#[test]
fn a_transmitted_frame_reaches_the_bus() {
    let mut soc = controller();
    let (bus, transceiver) = wired(&mut soc);

    transmit(&mut soc, 0x123, b"Hello");
    // The controller is clocked by the wire, and only completes the frame once
    // a receiver acknowledges it.
    run_wire(&mut soc, 200);

    assert_eq!(bus.borrow().stats().frames, 1);
    assert_eq!(transceiver.borrow().stats().tx_frames, 1);
    let records = bus.borrow().records().clone();
    assert_eq!(records.len(), 1);
    assert_eq!(records[0].node_name, "CAN1 节点");
    assert_eq!(records[0].frame.bytes(), b"Hello");
}

#[test]
fn a_frame_nobody_acknowledges_never_reaches_the_bus() {
    let mut soc = controller();
    let (bus, transceiver) = wired(&mut soc);

    transmit(&mut soc, 0x123, b"Hello");
    // Same loop, but the "peer" never pulls the acknowledge slot dominant.
    use gsim_core::can::{DOMINANT, RECESSIVE};
    soc.can_set_on_bus(0, true);
    for _ in 0..200 {
        let level = soc.can_driven_level(0).unwrap_or(RECESSIVE);
        let level = if level == DOMINANT {
            DOMINANT
        } else {
            RECESSIVE
        };
        soc.can_sample_bit(0, level);
        soc.advance_ns(1_000);
    }
    soc.can_set_on_bus(0, false);

    assert_eq!(bus.borrow().stats().frames, 0);
    assert_eq!(transceiver.borrow().stats().tx_frames, 0);
    assert!(soc.can.stats().tx_errors >= 1, "an acknowledge error");
    assert!(soc.can.error_counters().0 >= 8);
}

#[test]
fn a_frame_from_another_node_lands_in_the_receive_fifo() {
    let mut soc = controller();
    let (bus, transceiver) = wired(&mut soc);

    // A second ECU on the same wire.
    let peer = CanTransceiver::attach(Rc::clone(&bus), "对端节点", 0);
    let frame = CanFrame::standard(0x123, b"ping");
    {
        let mut peer = peer;
        peer.on_event(
            &Event::CanFrame {
                unit: 0,
                direction: gsim_core::CanDirection::Tx,
                frame,
            },
            0,
        );
    }

    // The transceiver hands what the wire delivered to the controller.
    let received = transceiver.borrow_mut().poll();
    assert_eq!(received, Some(frame));
    assert!(soc.can_receive(0, received.unwrap()));
    assert_eq!(soc.can.rx_level(0), 1);
    assert_eq!(soc.can.stats().rx_frames, 1);

    let queued = soc.can.rx_frames(0);
    assert_eq!(queued, vec![frame]);
    assert_eq!(read(&mut soc, CAN + map::can::RF0R) & 0b11, 1);
}

#[test]
fn a_silent_transceiver_refuses_to_drive_the_wire() {
    let mut soc = controller();
    let (bus, transceiver) = wired(&mut soc);
    transceiver
        .borrow_mut()
        .set_mode(gsim_devices::TransceiverMode::Silent);
    transceiver.borrow_mut().on_event(
        &Event::CanFrame {
            unit: 0,
            direction: gsim_core::CanDirection::Tx,
            frame: CanFrame::standard(0x123, b"nope"),
        },
        0,
    );

    assert_eq!(bus.borrow().stats().frames, 0);
    assert_eq!(transceiver.borrow().stats().refused, 1);
}

#[test]
fn a_silent_controller_keeps_its_transmitter_off_the_pair() {
    let mut soc = controller();
    // `SILM`: the block still runs, but `CAN_TX` stays released.
    write(&mut soc, CAN + map::can::MCR, 1);
    write(
        &mut soc,
        CAN + map::can::BTR,
        (2 - 1) << 16 | (2 - 1) << 20 | 35 | (1 << 31),
    );
    write(&mut soc, CAN + map::can::MCR, 0);
    transmit(&mut soc, 0x123, b"nope");

    for _ in 0..200 {
        let level = soc.can_driven_level(0);
        assert_eq!(level, None, "a silent block never drives the pair");
        soc.advance_ns(1_000);
    }
}

#[test]
fn frames_that_fail_the_filter_never_reach_the_fifo() {
    let mut soc = controller();
    let (_bus, _transceiver) = wired(&mut soc);
    assert!(!soc.can_receive(0, CanFrame::standard(0x456, b"x")));
    assert_eq!(soc.can.rx_level(0), 0);
    assert_eq!(soc.can.stats().dropped, 1);
}
