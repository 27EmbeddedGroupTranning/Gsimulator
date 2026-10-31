//! Virtual CAN bus and the transceiver that attaches one controller to it.
//!
//! A [`CanBus`] is shared by every node on the wire (`Rc<RefCell<CanBus>>`), so
//! several [`CanTransceiver`]s - one per simulated STM32, or one per simulated
//! STM32 plus the GUI's inject panel - can exchange frames. The bus keeps a
//! chronological log of everything that happened on the wire, which is what the
//! CAN bus status window renders.
//!
//! The bus itself is a **bit level** model, not a frame queue:
//!
//! * every node drives one level per bit time and the wire is the wired-AND of
//!   all of them ([`CanBus::step`]), so two nodes that start talking at the same
//!   time really do arbitrate, bit by bit, and the loser has to retry,
//! * a receiver pulls the acknowledge slot dominant, a missing acknowledge is an
//!   acknowledge error and starts an error frame (six dominant plus eight
//!   recessive bits),
//! * the parallel resistance of the termination network decides
//!   [`CanBus::quality`]; a badly terminated pair rings and flips bits, which the
//!   bus injects as [`CanBus::glitches`].
//!
//! The transceiver is the physical layer between a bxCAN controller and the
//! bus: the level the controller puts on `TXD` ([`CanTransceiver::driven`]) goes
//! onto the wire and the level the wire settles to is handed back to the
//! controller. A frame the controller receives in internal loopback mode never
//! reaches the wire and is logged as [`CanDirection::Loopback`] instead.
//!
//! Nodes without a controller (the GUI's inject panel, a second ECU, the demo)
//! are [`Peer`]s: they run the same bit machine against the wire.

use std::cell::RefCell;
use std::collections::VecDeque;
use std::rc::Rc;

use gsim_core::can::{
    CanDirection, CanErrorKind, CanFrame, FrameDecoder, RxStep, WireFrame, DOMINANT, RECESSIVE,
};
use gsim_core::Event;

/// Identifier of a node attached to a bus.
pub type NodeId = u32;

/// Ideal parallel resistance of the two 120 Ω terminators at the ends of the
/// wire.
pub const IDEAL_TERMINATION_OHM: f64 = 60.0;

/// Voltages the differential pair settles to, in volts.
pub const CAN_H_DOMINANT: f32 = 3.5;
pub const CAN_H_RECESSIVE: f32 = 2.5;
pub const CAN_L_DOMINANT: f32 = 1.5;
pub const CAN_L_RECESSIVE: f32 = 2.5;

/// Electrical quality of the wire for a parallel termination of `ohms`.
///
/// `60 Ω` (two `120 Ω` parts, one at each end) is the textbook load and scores
/// `1.0`; an open pair scores next to nothing because the reflections swamp the
/// differential signal. Values beyond the ideal by three times the ideal (or an
/// open pair) fall to the floor.
pub fn quality_of(ohms: Option<f64>) -> f64 {
    let Some(ohms) = ohms.filter(|ohms| *ohms > 0.0 && ohms.is_finite()) else {
        return 0.03;
    };
    let error = ((ohms - IDEAL_TERMINATION_OHM) / IDEAL_TERMINATION_OHM).abs();
    (1.0 - error / 3.0).clamp(0.03, 1.0)
}

/// Probability that a single bit reaches the receivers inverted.
///
/// Poor termination does not fail every bit: it fails a few, which is exactly
/// what makes a marginal bus so annoying to debug.
pub fn bit_error_rate(quality: f64) -> f64 {
    let shortfall = 1.0 - quality.clamp(0.0, 1.0);
    shortfall * shortfall * shortfall * 0.03
}

/// Voltages of one sampled bit, as an oscilloscope sees them.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WireLevels {
    pub can_h: f32,
    pub can_l: f32,
}

impl WireLevels {
    /// Voltages of a clean bit of the given level.
    pub fn of(level: bool) -> Self {
        if level == DOMINANT {
            WireLevels {
                can_h: CAN_H_DOMINANT,
                can_l: CAN_L_DOMINANT,
            }
        } else {
            WireLevels {
                can_h: CAN_H_RECESSIVE,
                can_l: CAN_L_RECESSIVE,
            }
        }
    }
}

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
    /// Bits the wire carried since the simulation started.
    pub bits: u64,
    /// Bits a badly terminated pair flipped.
    pub glitches: u64,
    /// Frames lost during arbitration by a node that then retried.
    pub arbitration_lost: u64,
    /// Error frames signalled on the wire.
    pub error_frames: u64,
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
    /// Duration of one bit, taken from the controllers' `BTR`.
    bit_ns: u64,
    /// Parallel resistance of the termination network, `None` when nothing
    /// bridges `CANH` and `CANL`.
    termination: Option<f64>,
    /// Electrical quality derived from `termination`.
    quality: f64,
    /// Bit level nodes without a controller (inject panel, second ECU).
    peers: Vec<Peer>,
    /// Receiver that follows the wire so the log and the counters do not depend
    /// on any controller being present.
    decoder: FrameDecoder,
    /// Consecutive recessive bits, the idle condition that lets a node start.
    idle_bits: u32,
    /// Small deterministic generator that decides which bits the wire flips.
    rng: u32,
    /// Simulated time of the last bit sampled.
    time_ns: u64,
    /// Last error frame a node signalled, for the status window.
    last_error: Option<(NodeId, CanErrorKind)>,
    /// Last frame that lost an arbitration round.
    last_arbitration: Option<(NodeId, CanFrame)>,
    /// `true` when the last sampled bit was inverted by poor termination.
    last_glitch: bool,
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
            bit_ns: 5_000,
            termination: None,
            quality: quality_of(None),
            peers: Vec::new(),
            decoder: FrameDecoder::new(),
            idle_bits: 0,
            rng: 0x1234_5678,
            time_ns: 0,
            last_error: None,
            last_arbitration: None,
            last_glitch: false,
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

    // ------------------------------------------------------------- the wire

    /// Duration of one bit, taken from the controllers' timing register.
    pub fn set_bit_time(&mut self, bit_ns: u64) {
        self.bit_ns = bit_ns.max(1);
    }

    pub fn bit_time(&self) -> u64 {
        self.bit_ns
    }

    /// Sets the parallel resistance that bridges `CANH` and `CANL`, `None` when
    /// nothing terminates the pair.
    pub fn set_termination(&mut self, ohms: Option<f64>) {
        self.termination = ohms;
        self.quality = quality_of(ohms);
    }

    pub fn termination(&self) -> Option<f64> {
        self.termination
    }

    /// Electrical quality of the pair, `1.0` for the textbook `60 Ω`.
    pub fn quality(&self) -> f64 {
        self.quality
    }

    /// Probability that one bit is corrupted by the current termination.
    pub fn error_rate(&self) -> f64 {
        bit_error_rate(self.quality)
    }

    pub fn glitches(&self) -> u64 {
        self.stats.glitches
    }

    /// Last error frame signalled on the wire, for the status window.
    pub fn last_error(&self) -> Option<(NodeId, CanErrorKind)> {
        self.last_error
    }

    /// Last frame that lost an arbitration round.
    pub fn last_arbitration(&self) -> Option<(NodeId, CanFrame)> {
        self.last_arbitration
    }

    /// `true` when the wire only reached the last sampled bit because it rang.
    pub fn last_glitch(&self) -> bool {
        self.last_glitch
    }

    /// `true` once the wire has been recessive for the intermission, which is
    /// when a node may start a frame.
    pub fn idle(&self) -> bool {
        self.idle_bits >= 3
    }

    /// Steps the wire for exactly one bit time.
    ///
    /// `controller` is the level the MCU's CAN transmitter drives (`None` when
    /// it is not driving at all), which makes the wire the wired-AND of every
    /// driver. Returns the level the receivers have to sample, glitch included.
    pub fn step(&mut self, controller: Option<bool>, time_ns: u64) -> bool {
        self.time_ns = time_ns;
        self.stats.bits += 1;
        if self.idle_bits >= 3 && !self.decoder.active() {
            for peer in &mut self.peers {
                peer.start_if_idle();
            }
        }

        let mut dominant = controller == Some(DOMINANT);
        dominant |= self
            .peers
            .iter()
            .any(|peer| peer.driven() == Some(DOMINANT));
        let line = if dominant { DOMINANT } else { RECESSIVE };

        let active = controller.is_some()
            || self.decoder.active()
            || dominant
            || self.peers.iter().any(|peer| peer.busy());
        let mut sampled = line;
        self.last_glitch = active && self.flip_due();
        if self.last_glitch {
            sampled = !line;
            self.stats.glitches += 1;
        }

        let mut events = Vec::new();
        for peer in &mut self.peers {
            peer.sample(sampled, &mut events);
        }
        self.apply(&events, time_ns);

        // Frames are logged by the node that sent them and by every node that
        // accepted them, so the wire only keeps the error counters.
        if let RxStep::Error(_) = self.decoder.feed(sampled) {
            self.stats.error_frames += 1;
        }

        if sampled == DOMINANT || self.decoder.active() {
            self.idle_bits = 0;
        } else {
            self.idle_bits += 1;
        }
        sampled
    }

    fn flip_due(&mut self) -> bool {
        let rate = bit_error_rate(self.quality);
        if rate <= 0.0 {
            return false;
        }
        self.rng ^= self.rng << 13;
        self.rng ^= self.rng >> 17;
        self.rng ^= self.rng << 5;
        f64::from(self.rng) / f64::from(u32::MAX) < rate
    }

    fn apply(&mut self, events: &[PeerEvent], time_ns: u64) {
        for event in events {
            match event {
                PeerEvent::Sent(peer, frame) => {
                    let name = self.peer_name(*peer);
                    self.stats.frames += 1;
                    self.push_record(BusRecord {
                        time_ns,
                        node: *peer,
                        node_name: name,
                        direction: CanDirection::Tx,
                        frame: *frame,
                        acknowledged: true,
                    });
                }
                PeerEvent::Received(peer, frame) => {
                    let name = self.peer_name(*peer);
                    self.stats.deliveries += 1;
                    self.push_record(BusRecord {
                        time_ns,
                        node: *peer,
                        node_name: name,
                        direction: CanDirection::Rx,
                        frame: *frame,
                        acknowledged: true,
                    });
                }
                PeerEvent::Error(node, kind) => {
                    self.stats.error_frames += 1;
                    self.last_error = Some((*node, *kind));
                }
                PeerEvent::ArbitrationLost(node, frame) => {
                    self.stats.arbitration_lost += 1;
                    self.last_arbitration = Some((*node, *frame));
                }
            }
        }
    }

    fn peer_name(&self, node: NodeId) -> String {
        self.peers
            .iter()
            .find(|peer| peer.id() == node)
            .map(|peer| peer.name().to_string())
            .unwrap_or_else(|| "?".to_string())
    }

    /// Registers a bit level node without a controller and returns its id.
    pub fn add_peer(&mut self, name: &str) -> NodeId {
        let id = self.next_id;
        self.next_id += 1;
        self.peers.push(Peer::new(id, name));
        id
    }

    /// Bit level nodes on this bus, oldest first.
    pub fn peers(&self) -> &[Peer] {
        &self.peers
    }

    /// Queues a frame on a bit level node; it goes out bit by bit as soon as
    /// the wire is idle, exactly like a controller's mailbox.
    pub fn queue_frame(&mut self, node: NodeId, frame: CanFrame) -> bool {
        let Some(peer) = self.peers.iter_mut().find(|peer| peer.id() == node) else {
            return false;
        };
        peer.drive(frame);
        true
    }

    /// Queues a frame on the bus' own "外部节点", creating it on first use.
    ///
    /// The frame really travels on the wire, so it arbitrates against the MCU
    /// and a lower identifier wins.
    pub fn inject_bit_frame(&mut self, frame: CanFrame, time_ns: u64) -> NodeId {
        let _ = time_ns;
        let node = match self.peers.iter().find(|peer| peer.name() == EXTERNAL_PEER) {
            Some(peer) => peer.id(),
            None => self.add_peer(EXTERNAL_PEER),
        };
        self.queue_frame(node, frame);
        node
    }

    /// Logs a frame a controller finished transmitting on the wire.
    pub fn record_transmission(&mut self, node: NodeId, frame: CanFrame, time_ns: u64) {
        let name = self.node_name(node).unwrap_or("?").to_string();
        self.stats.frames += 1;
        self.push_record(BusRecord {
            time_ns,
            node,
            node_name: name,
            direction: CanDirection::Tx,
            frame,
            acknowledged: true,
        });
    }

    fn push_record(&mut self, record: BusRecord) {
        while self.records.len() >= self.capacity {
            self.records.pop_front();
        }
        self.records.push_back(record);
    }
}

/// Name of the peer the inject panel talks through.
pub const EXTERNAL_PEER: &str = "外部节点";

/// Counters of one bit level node.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PeerStats {
    /// Frames the node put on the wire.
    pub tx_frames: u64,
    /// Frames the node accepted from the wire.
    pub rx_frames: u64,
    /// Frames it lost during arbitration (they are retried).
    pub arbitration_lost: u64,
    /// Error frames it signalled.
    pub errors: u64,
}

/// What a peer did during one sampled bit, replayed by the bus.
#[derive(Debug)]
enum PeerEvent {
    Sent(NodeId, CanFrame),
    Received(NodeId, CanFrame),
    Error(NodeId, CanErrorKind),
    ArbitrationLost(NodeId, CanFrame),
}

/// The frame a peer is shifting out, bit by bit.
#[derive(Debug)]
struct PeerTx {
    wire: WireFrame,
    index: usize,
}

/// A node without a controller.
///
/// It runs the same bit machine as the bxCAN block - stuff bits come with the
/// [`WireFrame`] - so it arbitrates against the MCU, acknowledges frames it
/// decoded correctly and signals error frames.
#[derive(Debug)]
pub struct Peer {
    id: NodeId,
    name: String,
    queue: VecDeque<CanFrame>,
    active: Option<PeerTx>,
    decoder: FrameDecoder,
    ack_pending: bool,
    error_dominant: u8,
    error_recessive: u8,
    self_frame: Option<CanFrame>,
    stats: PeerStats,
}

impl Peer {
    fn new(id: NodeId, name: &str) -> Self {
        Peer {
            id,
            name: name.to_string(),
            queue: VecDeque::new(),
            active: None,
            decoder: FrameDecoder::new(),
            ack_pending: false,
            error_dominant: 0,
            error_recessive: 0,
            self_frame: None,
            stats: PeerStats::default(),
        }
    }

    pub fn id(&self) -> NodeId {
        self.id
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn stats(&self) -> PeerStats {
        self.stats
    }

    /// Frames waiting for the wire.
    pub fn queued(&self) -> usize {
        self.queue.len()
    }

    /// `true` while the node is using the wire or signalling an error.
    pub fn busy(&self) -> bool {
        self.active.is_some() || self.error_dominant > 0 || self.error_recessive > 0
    }

    fn drive(&mut self, frame: CanFrame) {
        self.queue.push_back(frame);
    }

    fn start_if_idle(&mut self) {
        if self.busy() {
            return;
        }
        let Some(frame) = self.queue.pop_front() else {
            return;
        };
        self.self_frame = Some(frame);
        self.active = Some(PeerTx {
            wire: WireFrame::encode(&frame),
            index: 0,
        });
    }

    /// Level the node drives on the pair during this bit.
    fn driven(&self) -> Option<bool> {
        if self.error_dominant > 0 {
            return Some(DOMINANT);
        }
        if self.error_recessive > 0 {
            return Some(RECESSIVE);
        }
        if self.ack_pending {
            return Some(DOMINANT);
        }
        self.active
            .as_ref()
            .and_then(|tx| tx.wire.bits().get(tx.index).copied())
    }

    fn sample(&mut self, level: bool, events: &mut Vec<PeerEvent>) {
        self.ack_pending = false;
        if self.error_dominant > 0 {
            self.error_dominant -= 1;
            return;
        }
        if self.error_recessive > 0 {
            self.error_recessive -= 1;
            if self.error_recessive == 0 {
                self.decoder.reset();
            }
            return;
        }

        let mut lost = false;
        let mut failure = None;
        let mut finished = None;
        if let Some(tx) = self.active.as_mut() {
            let index = tx.index;
            if index == tx.wire.ack_slot() {
                if level != DOMINANT {
                    failure = Some(CanErrorKind::Acknowledge);
                }
            } else if let Some(expected) = tx.wire.bits().get(index).copied() {
                if expected == RECESSIVE && level == DOMINANT && index <= tx.wire.arbitration_end()
                {
                    lost = true;
                } else if expected == DOMINANT && level == RECESSIVE {
                    failure = Some(CanErrorKind::Bit);
                }
            }
            if !lost && failure.is_none() {
                tx.index += 1;
                if tx.index >= tx.wire.bits().len() {
                    finished = Some(*tx.wire.frame());
                }
            }
        }

        if lost {
            if let Some(tx) = self.active.take() {
                let frame = *tx.wire.frame();
                self.stats.arbitration_lost += 1;
                self.queue.push_front(frame);
                events.push(PeerEvent::ArbitrationLost(self.id, frame));
            }
        } else if let Some(kind) = failure {
            if let Some(tx) = self.active.take() {
                self.queue.push_front(*tx.wire.frame());
            }
            self.signal_error(kind);
            events.push(PeerEvent::Error(self.id, kind));
        } else if let Some(frame) = finished {
            self.active = None;
            self.self_frame = None;
            self.stats.tx_frames += 1;
            events.push(PeerEvent::Sent(self.id, frame));
        }

        match self.decoder.feed(level) {
            RxStep::CrcDelimiter { ok } => {
                if ok && self.active.is_none() {
                    self.ack_pending = true;
                }
            }
            RxStep::Frame(frame) => {
                if self.self_frame != Some(frame) {
                    self.stats.rx_frames += 1;
                    events.push(PeerEvent::Received(self.id, frame));
                }
            }
            RxStep::Error(kind) => {
                if self.active.is_none() {
                    self.signal_error(kind);
                    events.push(PeerEvent::Error(self.id, kind));
                }
            }
            _ => {}
        }
    }

    fn signal_error(&mut self, _kind: CanErrorKind) {
        self.stats.errors += 1;
        self.error_dominant = 6;
        self.error_recessive = 8;
        self.ack_pending = false;
        self.decoder.reset();
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
    /// `true` when the controller on this part is modelled bit by bit, so the
    /// part only has to log the frames instead of forwarding them.
    bit_level: bool,
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
            bit_level: false,
            stats: TransceiverStats::default(),
        }
    }

    /// Tells the part that its controller runs the bit level protocol, so a
    /// transmitted frame is only logged instead of being replayed frame by
    /// frame onto the wire.
    pub fn set_bit_level(&mut self, bit_level: bool) {
        self.bit_level = bit_level;
    }

    pub fn bit_level(&self) -> bool {
        self.bit_level
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
                    if self.bit_level {
                        self.bus
                            .borrow_mut()
                            .record_transmission(self.node, *frame, time_ns);
                    } else {
                        self.bus.borrow_mut().transmit(self.node, *frame, time_ns);
                    }
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

    /// Steps the wire for `bits` bit times, as the front end does.
    fn run(bus: &mut CanBus, bits: u64) {
        for bit in 0..bits {
            let now = bit * bus.bit_time();
            bus.step(None, now);
        }
    }

    #[test]
    fn a_bit_level_node_puts_its_frame_on_the_wire() {
        let mut bus = CanBus::new();
        bus.set_termination(Some(60.0));
        let node = bus.add_peer("节点A");
        bus.add_peer("节点B");
        bus.queue_frame(node, CanFrame::standard(0x123, b"Hi"));
        run(&mut bus, 400);

        let stats = bus.stats();
        assert_eq!(stats.frames, 1, "{stats:?}");
        assert_eq!(stats.arbitration_lost, 0);
        assert_eq!(stats.glitches, 0);
        assert_eq!(bus.peers()[0].stats().tx_frames, 1);
        // The listener logs the frame it accepted, the sender logs the end of
        // its transmission.
        let records = bus.records().clone();
        assert_eq!(records.len(), 2);
        assert_eq!(records[0].direction, CanDirection::Rx);
        assert_eq!(records[1].direction, CanDirection::Tx);
        assert_eq!(records[1].frame.bytes(), b"Hi");
        assert_eq!(bus.peers()[1].stats().rx_frames, 1);
    }

    #[test]
    fn two_nodes_that_start_together_arbitrate_and_the_lower_identifier_wins() {
        let mut bus = CanBus::new();
        bus.set_termination(Some(60.0));
        let slow = bus.add_peer("节点A");
        let quick = bus.add_peer("节点B");
        bus.queue_frame(slow, CanFrame::standard(0x200, &[0x11]));
        bus.queue_frame(quick, CanFrame::standard(0x100, &[0x22]));
        run(&mut bus, 800);

        let stats = bus.stats();
        assert!(stats.arbitration_lost >= 1, "{stats:?}");
        // The loser retried, so both frames reached the wire.
        let ids: Vec<u32> = bus
            .records()
            .iter()
            .filter(|record| record.direction == CanDirection::Tx)
            .map(|record| record.frame.id)
            .collect();
        assert!(ids.contains(&0x100), "{ids:?}");
        assert!(ids.contains(&0x200), "{ids:?}");
        // And both nodes also received the frame that won.
        assert!(bus.peers().iter().all(|peer| peer.stats().rx_frames >= 1));
    }

    #[test]
    fn a_frame_nobody_acknowledges_becomes_an_acknowledge_error() {
        let mut bus = CanBus::new();
        bus.set_termination(Some(60.0));
        let lonely = bus.add_peer("孤立节点");
        bus.queue_frame(lonely, CanFrame::standard(0x10, &[1]));
        run(&mut bus, 260);

        assert!(bus.stats().error_frames >= 1, "{:?}", bus.stats());
        assert!(bus.peers()[0].stats().errors >= 1);
        assert!(
            bus.records()
                .iter()
                .all(|record| record.direction != CanDirection::Tx),
            "an unacknowledged frame must not be logged as sent"
        );
    }

    #[test]
    fn an_open_pair_flips_bits_while_a_terminated_one_does_not() {
        let frame = CanFrame::standard(0x321, b"termination");

        let mut good = CanBus::new();
        good.set_termination(Some(60.0));
        let node = good.add_peer("节点A");
        good.add_peer("节点B");
        for _ in 0..4 {
            good.queue_frame(node, frame);
        }
        run(&mut good, 1_600);
        assert_eq!(good.glitches(), 0);
        assert_eq!(good.stats().frames, 4);

        let mut open = CanBus::new();
        open.set_termination(None);
        let node = open.add_peer("节点A");
        open.add_peer("节点B");
        for _ in 0..4 {
            open.queue_frame(node, frame);
        }
        run(&mut open, 1_600);
        assert!(open.glitches() > 0, "an unterminated pair must ring");
        assert!(open.quality() < 0.1);
        assert!(open.error_rate() > 0.0);
    }

    #[test]
    fn the_quality_peaks_at_the_textbook_termination() {
        assert!((quality_of(Some(60.0)) - 1.0).abs() < 1e-9);
        assert!(quality_of(Some(120.0)) < quality_of(Some(60.0)));
        assert!(quality_of(None) < quality_of(Some(120.0)));
        assert_eq!(bit_error_rate(1.0), 0.0);
        assert!(bit_error_rate(quality_of(None)) > bit_error_rate(quality_of(Some(120.0))));
    }

    #[test]
    fn an_injected_frame_travels_bit_by_bit_on_the_wire() {
        let mut bus = CanBus::new();
        bus.set_termination(Some(60.0));
        bus.add_peer("节点A");
        bus.inject_bit_frame(CanFrame::standard(0x222, &[0x42]), 0);
        assert_eq!(bus.peers().len(), 2, "the inject panel adds its own node");
        assert_eq!(bus.records().len(), 0, "nothing is on the wire yet");
        run(&mut bus, 400);
        assert_eq!(bus.stats().frames, 1);
        assert!(bus
            .records()
            .iter()
            .any(|record| record.node_name == EXTERNAL_PEER));
    }
}
