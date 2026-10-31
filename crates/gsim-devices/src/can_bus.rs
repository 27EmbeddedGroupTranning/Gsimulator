//! Virtual CAN bus and the transceiver that attaches one controller to it.
//!
//! A [`CanBus`] is shared by every node on the wire (`Rc<RefCell<CanBus>>`), so
//! several [`CanTransceiver`]s - one per simulated STM32, or one per simulated
//! STM32 plus the GUI's inject panel - can exchange frames. The bus keeps a
//! chronological log of everything that happened on the wire, which is what the
//! CAN bus status window renders.
//!
//! The transceiver is the physical layer between a bxCAN controller and the
//! bus: a frame the controller puts on `TXD` ([`CanDirection::Tx`]) is driven
//! onto the wire, and every frame the wire carries is handed to the controller
//! through [`CanTransceiver::poll`]. A frame the controller receives in internal
//! loopback mode never reaches the wire and is logged as
//! [`CanDirection::Loopback`] instead.

use std::cell::RefCell;
use std::collections::VecDeque;
use std::rc::Rc;

use gsim_core::can::{CanDirection, CanFrame};
use gsim_core::Event;

/// Identifier of a node attached to a bus.
pub type NodeId = u32;

/// Electrical mode of the transceiver.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TransceiverMode {
    /// TXD drives the wire, RXD follows the wire.
    Normal,
    /// The part keeps receiving but never drives the wire (listen only).
    Silent,
    /// Standby: the part is powered down and isolates the controller.
    Standby,
}

impl TransceiverMode {
    pub fn label(self) -> &'static str {
        match self {
            TransceiverMode::Normal => "正常",
            TransceiverMode::Silent => "只听",
            TransceiverMode::Standby => "待机",
        }
    }

    /// `true` when the part may drive the bus.
    pub fn can_transmit(self) -> bool {
        self == TransceiverMode::Normal
    }

    /// `true` when the part forwards bus traffic to the controller.
    pub fn can_receive(self) -> bool {
        self != TransceiverMode::Standby
    }
}

/// One entry of the bus log.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BusRecord {
    pub time_ns: u64,
    pub node: NodeId,
    pub node_name: String,
    pub direction: CanDirection,
    pub frame: CanFrame,
    /// `true` when at least one other node acknowledged the frame.
    pub acknowledged: bool,
}

/// Counters for the bus window.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CanBusStats {
    /// Frames the wire carried (transmissions of every node).
    pub frames: u64,
    /// Frame/node pairs delivered to a receiver.
    pub deliveries: u64,
    /// Frames that never left a controller because of internal loopback.
    pub loopbacks: u64,
    /// Frames refused by a transceiver (standby, silent, no receiver).
    pub refused: u64,
}

#[derive(Debug)]
struct NodeSlot {
    id: NodeId,
    name: String,
    rx: VecDeque<CanFrame>,
}

/// The wire every transceiver shares.
#[derive(Debug)]
pub struct CanBus {
    nodes: Vec<NodeSlot>,
    next_id: NodeId,
    records: VecDeque<BusRecord>,
    capacity: usize,
    stats: CanBusStats,
}

impl Default for CanBus {
    fn default() -> Self {
        CanBus::new()
    }
}

impl CanBus {
    pub fn new() -> Self {
        CanBus::with_history(512)
    }

    /// Bus that keeps at most `capacity` log entries.
    pub fn with_history(capacity: usize) -> Self {
        CanBus {
            nodes: Vec::new(),
            next_id: 1,
            records: VecDeque::new(),
            capacity: capacity.max(1),
            stats: CanBusStats::default(),
        }
    }

    /// Registers a node and returns its identifier.
    pub fn attach(&mut self, name: &str) -> NodeId {
        let id = self.next_id;
        self.next_id += 1;
        self.nodes.push(NodeSlot {
            id,
            name: name.to_string(),
            rx: VecDeque::new(),
        });
        id
    }

    /// Removes a node, dropping everything queued for it.
    pub fn detach(&mut self, node: NodeId) {
        self.nodes.retain(|slot| slot.id != node);
    }

    pub fn node_name(&self, node: NodeId) -> Option<&str> {
        self.nodes
            .iter()
            .find(|slot| slot.id == node)
            .map(|slot| slot.name.as_str())
    }

    /// Names of every attached node, for the status window.
    pub fn node_names(&self) -> Vec<(NodeId, String)> {
        self.nodes
            .iter()
            .map(|slot| (slot.id, slot.name.clone()))
            .collect()
    }

    pub fn stats(&self) -> CanBusStats {
        self.stats
    }

    /// Log entries, oldest first.
    pub fn records(&self) -> &VecDeque<BusRecord> {
        &self.records
    }

    pub fn clear_records(&mut self) {
        self.records.clear();
    }

    /// Puts a frame on the wire on behalf of `from`.
    ///
    /// Every other node that is able to receive gets a copy; the sender is
    /// acknowledged when at least one node accepted the frame. Returns the
    /// number of nodes the frame was delivered to.
    pub fn transmit(&mut self, from: NodeId, frame: CanFrame, time_ns: u64) -> usize {
        let name = self.node_name(from).unwrap_or("?").to_string();
        let mut delivered = 0;
        let mut records = Vec::new();
        for slot in &mut self.nodes {
            if slot.id == from {
                continue;
            }
            slot.rx.push_back(frame);
            delivered += 1;
            records.push(BusRecord {
                time_ns,
                node: slot.id,
                node_name: slot.name.clone(),
                direction: CanDirection::Rx,
                frame,
                acknowledged: true,
            });
        }
        self.stats.frames += 1;
        self.stats.deliveries += delivered as u64;
        for record in records {
            self.push_record(record);
        }
        self.push_record(BusRecord {
            time_ns,
            node: from,
            node_name: name,
            direction: CanDirection::Tx,
            frame,
            acknowledged: delivered > 0,
        });
        delivered
    }

    /// Takes the next frame the bus holds for `node`.
    pub fn poll(&mut self, node: NodeId) -> Option<CanFrame> {
        self.nodes
            .iter_mut()
            .find(|slot| slot.id == node)
            .and_then(|slot| slot.rx.pop_front())
    }

    /// Number of frames still queued for `node`.
    pub fn pending(&self, node: NodeId) -> usize {
        self.nodes
            .iter()
            .find(|slot| slot.id == node)
            .map(|slot| slot.rx.len())
            .unwrap_or(0)
    }

    /// Records a frame the controller received in internal loopback mode.
    pub fn record_loopback(&mut self, node: NodeId, frame: CanFrame, time_ns: u64) {
        let name = self.node_name(node).unwrap_or("?").to_string();
        self.stats.loopbacks += 1;
        self.push_record(BusRecord {
            time_ns,
            node,
            node_name: name,
            direction: CanDirection::Loopback,
            frame,
            acknowledged: true,
        });
    }

    /// Records a frame a controller accepted from the wire, so the log shows
    /// the receiving side even when it was injected from outside the simulator.
    pub fn record_reception(&mut self, node: NodeId, frame: CanFrame, time_ns: u64) {
        let name = self.node_name(node).unwrap_or("?").to_string();
        self.push_record(BusRecord {
            time_ns,
            node,
            node_name: name,
            direction: CanDirection::Rx,
            frame,
            acknowledged: true,
        });
    }

    /// Notes that a node refused to send because its transceiver was not in
    /// normal mode.
    pub fn record_refusal(&mut self) {
        self.stats.refused += 1;
    }

    /// Injects a frame from outside the simulated network (a diagnostics tool
    /// or a second ECU). Every attached node receives it.
    pub fn inject(&mut self, frame: CanFrame, time_ns: u64) -> usize {
        let mut delivered = 0;
        let mut records = Vec::new();
        for slot in &mut self.nodes {
            slot.rx.push_back(frame);
            delivered += 1;
            records.push(BusRecord {
                time_ns,
                node: slot.id,
                node_name: slot.name.clone(),
                direction: CanDirection::Rx,
                frame,
                acknowledged: true,
            });
        }
        self.stats.frames += 1;
        self.stats.deliveries += delivered as u64;
        for record in records {
            self.push_record(record);
        }
        self.push_record(BusRecord {
            time_ns,
            node: 0,
            node_name: "外部节点".to_string(),
            direction: CanDirection::Tx,
            frame,
            acknowledged: delivered > 0,
        });
        delivered
    }

    fn push_record(&mut self, record: BusRecord) {
        while self.records.len() >= self.capacity {
            self.records.pop_front();
        }
        self.records.push_back(record);
    }
}

/// Counters displayed next to one transceiver.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TransceiverStats {
    pub tx_frames: u64,
    pub rx_frames: u64,
    pub loopback_frames: u64,
    pub refused: u64,
}

/// A CAN transceiver (physical layer part) between one controller and a bus.
#[derive(Debug)]
pub struct CanTransceiver {
    bus: Rc<RefCell<CanBus>>,
    node: NodeId,
    name: String,
    unit: u8,
    mode: TransceiverMode,
    tx_pin: (u8, u8),
    rx_pin: (u8, u8),
    stats: TransceiverStats,
}

impl CanTransceiver {
    /// Attaches a new node to `bus` and returns the transceiver that connects
    /// controller `unit` to it.
    pub fn attach(bus: Rc<RefCell<CanBus>>, name: &str, unit: u8) -> Self {
        let node = bus.borrow_mut().attach(name);
        CanTransceiver {
            bus,
            node,
            name: name.to_string(),
            unit,
            mode: TransceiverMode::Normal,
            tx_pin: (0, 12),
            rx_pin: (0, 11),
            stats: TransceiverStats::default(),
        }
    }

    /// Overrides the pins the part is wired to (`TXD`, `RXD`).
    pub fn with_pins(mut self, tx: (u8, u8), rx: (u8, u8)) -> Self {
        self.tx_pin = tx;
        self.rx_pin = rx;
        self
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn node(&self) -> NodeId {
        self.node
    }

    pub fn bus(&self) -> Rc<RefCell<CanBus>> {
        Rc::clone(&self.bus)
    }

    pub fn unit(&self) -> u8 {
        self.unit
    }

    pub fn pins(&self) -> ((u8, u8), (u8, u8)) {
        (self.tx_pin, self.rx_pin)
    }

    pub fn mode(&self) -> TransceiverMode {
        self.mode
    }

    pub fn set_mode(&mut self, mode: TransceiverMode) {
        self.mode = mode;
    }

    pub fn stats(&self) -> TransceiverStats {
        self.stats
    }

    /// Number of frames waiting to be handed to the controller.
    pub fn pending(&self) -> usize {
        self.bus.borrow().pending(self.node)
    }

    /// Feeds one peripheral event to the part.
    pub fn on_event(&mut self, event: &Event, time_ns: u64) {
        let Event::CanFrame {
            unit,
            direction,
            frame,
        } = event
        else {
            return;
        };
        if *unit != self.unit {
            return;
        }
        match direction {
            CanDirection::Tx => {
                if self.mode.can_transmit() {
                    self.stats.tx_frames += 1;
                    self.bus.borrow_mut().transmit(self.node, *frame, time_ns);
                } else {
                    self.stats.refused += 1;
                    self.bus.borrow_mut().record_refusal();
                }
            }
            CanDirection::Loopback => {
                self.stats.loopback_frames += 1;
                self.bus
                    .borrow_mut()
                    .record_loopback(self.node, *frame, time_ns);
            }
            CanDirection::Rx => {
                if self.mode.can_receive() {
                    self.bus
                        .borrow_mut()
                        .record_reception(self.node, *frame, time_ns);
                }
            }
        }
    }

    /// Takes the next frame the bus delivered to this part, if any.
    pub fn poll(&mut self) -> Option<CanFrame> {
        if !self.mode.can_receive() {
            return None;
        }
        let frame = self.bus.borrow_mut().poll(self.node);
        if frame.is_some() {
            self.stats.rx_frames += 1;
        }
        frame
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bus_with_two_nodes() -> (Rc<RefCell<CanBus>>, CanTransceiver, CanTransceiver) {
        let bus = Rc::new(RefCell::new(CanBus::new()));
        let left = CanTransceiver::attach(Rc::clone(&bus), "节点A", 0);
        let right = CanTransceiver::attach(Rc::clone(&bus), "节点B", 0);
        (bus, left, right)
    }

    #[test]
    fn a_transmitted_frame_reaches_every_other_node() {
        let (bus, mut left, mut right) = bus_with_two_nodes();
        let frame = CanFrame::standard(0x123, b"Hello");
        left.on_event(
            &Event::CanFrame {
                unit: 0,
                direction: CanDirection::Tx,
                frame,
            },
            1_000,
        );

        assert_eq!(right.poll(), Some(frame));
        assert_eq!(left.poll(), None);
        assert_eq!(bus.borrow().stats().frames, 1);
        assert_eq!(left.stats().tx_frames, 1);
        assert_eq!(right.stats().rx_frames, 1);

        let records = bus.borrow().records().clone();
        assert_eq!(records.len(), 2);
        assert_eq!(records[0].direction, CanDirection::Rx);
        assert_eq!(records[0].node_name, "节点B");
        assert_eq!(records[1].direction, CanDirection::Tx);
        assert_eq!(records[1].node_name, "节点A");
        assert!(records[1].acknowledged);
    }

    #[test]
    fn a_silent_transceiver_does_not_drive_the_wire() {
        let (bus, mut left, mut right) = bus_with_two_nodes();
        left.set_mode(TransceiverMode::Silent);
        left.on_event(
            &Event::CanFrame {
                unit: 0,
                direction: CanDirection::Tx,
                frame: CanFrame::standard(0x10, &[1]),
            },
            0,
        );
        assert_eq!(right.poll(), None);
        assert_eq!(left.stats().refused, 1);
        assert_eq!(bus.borrow().stats().refused, 1);
    }

    #[test]
    fn a_standby_transceiver_isolates_the_controller() {
        let (bus, mut left, mut right) = bus_with_two_nodes();
        right.set_mode(TransceiverMode::Standby);
        left.on_event(
            &Event::CanFrame {
                unit: 0,
                direction: CanDirection::Tx,
                frame: CanFrame::standard(0x10, &[1]),
            },
            0,
        );
        // The bus still delivers to the node queue, but the part is isolated.
        assert_eq!(right.poll(), None);
        assert!(!right.mode().can_receive());
        assert_eq!(bus.borrow().pending(right.node()), 1);
    }

    #[test]
    fn loopback_frames_are_logged_but_stay_off_the_wire() {
        let (bus, mut left, mut right) = bus_with_two_nodes();
        left.on_event(
            &Event::CanFrame {
                unit: 0,
                direction: CanDirection::Loopback,
                frame: CanFrame::standard(0x321, &[9]),
            },
            42,
        );
        assert_eq!(right.poll(), None);
        let records = bus.borrow().records().clone();
        assert_eq!(records.len(), 1);
        assert_eq!(records[0].direction, CanDirection::Loopback);
        assert_eq!(records[0].time_ns, 42);
        assert_eq!(bus.borrow().stats().loopbacks, 1);
    }

    #[test]
    fn injecting_a_frame_from_outside_reaches_every_node() {
        let (bus, mut left, mut right) = bus_with_two_nodes();
        let frame = CanFrame::extended(0x18DA_F110, &[0xAA]);
        assert_eq!(bus.borrow_mut().inject(frame, 7), 2);
        assert_eq!(left.poll(), Some(frame));
        assert_eq!(right.poll(), Some(frame));
        assert_eq!(bus.borrow().records().len(), 3);
    }

    #[test]
    fn events_of_another_controller_are_ignored() {
        let (bus, mut left, _right) = bus_with_two_nodes();
        left.on_event(
            &Event::CanFrame {
                unit: 1,
                direction: CanDirection::Tx,
                frame: CanFrame::standard(0x10, &[1]),
            },
            0,
        );
        assert_eq!(bus.borrow().stats().frames, 0);
    }

    #[test]
    fn the_log_keeps_only_the_requested_number_of_entries() {
        let bus = Rc::new(RefCell::new(CanBus::with_history(4)));
        let mut node = CanTransceiver::attach(Rc::clone(&bus), "节点A", 0);
        for index in 0..10u8 {
            node.on_event(
                &Event::CanFrame {
                    unit: 0,
                    direction: CanDirection::Loopback,
                    frame: CanFrame::standard(0x100, &[index]),
                },
                u64::from(index),
            );
        }
        let records = bus.borrow().records().clone();
        assert_eq!(records.len(), 4);
        assert_eq!(records.back().unwrap().frame.data[0], 9);
    }
}
