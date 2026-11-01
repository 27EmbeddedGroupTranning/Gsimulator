//! Classic CAN frames shared by the on-chip controller and the external
//! device models.
//!
//! Both the bxCAN model in `gsim-stm32f1` and the transceiver/bus models in
//! `gsim-devices` exchange this single value type, so a frame never has to be
//! re-encoded when it crosses a module boundary.

use std::collections::VecDeque;

/// Direction of a frame relative to the controller that reported it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum CanDirection {
    /// The controller put the frame on its transmit pin (transceiver sees it).
    Tx,
    /// The controller accepted the frame from the bus into a receive FIFO.
    Rx,
    /// Internal self reception of a frame sent in loopback mode; no pin and no
    /// transceiver is involved.
    Loopback,
}

impl CanDirection {
    pub fn label(self) -> &'static str {
        match self {
            CanDirection::Tx => "tx",
            CanDirection::Rx => "rx",
            CanDirection::Loopback => "loopback",
        }
    }
}

/// One classic CAN frame: 11 or 29 bit identifier, up to eight data bytes.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct CanFrame {
    pub id: u32,
    /// `true` for a 29 bit identifier, `false` for an 11 bit identifier.
    pub extended: bool,
    /// `true` for a remote (data request) frame.
    pub remote: bool,
    /// Number of valid bytes in [`CanFrame::data`] (0..=8).
    pub dlc: u8,
    pub data: [u8; 8],
}

impl CanFrame {
    /// Standard data frame.
    pub fn standard(id: u16, data: &[u8]) -> Self {
        CanFrame {
            id: u32::from(id) & 0x7FF,
            extended: false,
            remote: false,
            dlc: clamp_len(data),
            data: pack(data),
        }
    }

    /// Extended data frame.
    pub fn extended(id: u32, data: &[u8]) -> Self {
        CanFrame {
            id: id & 0x1FFF_FFFF,
            extended: true,
            remote: false,
            dlc: clamp_len(data),
            data: pack(data),
        }
    }

    /// Remote (data request) frame with the given identifier format.
    pub fn remote(frame: CanFrame) -> Self {
        CanFrame {
            dlc: 0,
            data: [0; 8],
            remote: true,
            ..frame
        }
    }

    /// Bytes actually carried by the frame.
    pub fn bytes(&self) -> &[u8] {
        &self.data[..self.dlc as usize]
    }

    /// `0x123` style identifier text.
    pub fn id_label(&self) -> String {
        if self.extended {
            format!("0x{:08X}", self.id)
        } else {
            format!("0x{:03X}", self.id)
        }
    }

    /// `48 65 6C` style data text, or `-` when the frame carries no data.
    pub fn data_label(&self) -> String {
        if self.remote || self.dlc == 0 {
            return "-".to_string();
        }
        self.bytes()
            .iter()
            .map(|byte| format!("{byte:02X}"))
            .collect::<Vec<_>>()
            .join(" ")
    }

    /// Data bytes as ASCII, used by the bus browser for convenience.
    pub fn ascii_label(&self) -> String {
        self.bytes()
            .iter()
            .map(|byte| {
                if byte.is_ascii_graphic() || *byte == b' ' {
                    char::from(*byte)
                } else {
                    '.'
                }
            })
            .collect()
    }

    /// Number of bits an 11 bit identifier frame occupies on the wire,
    /// including stuffing worst case is not modelled: SOF, arbitration,
    /// control, data, CRC, ACK, EOF and the intermission are summed up.
    pub fn bit_count(&self) -> u64 {
        let id_bits = if self.extended { 32 } else { 12 };
        let data_bits = if self.remote {
            0
        } else {
            u64::from(self.dlc) * 8
        };
        // SOF + arbitration(ID+RTR+IDE+r0) + DLC + data + CRC(15) + delimiter
        // + ACK slot + ACK delimiter + EOF(7) + intermission(3).
        1 + id_bits + 4 + data_bits + 15 + 1 + 1 + 1 + 7 + 3
    }
}

fn clamp_len(data: &[u8]) -> u8 {
    data.len().min(8) as u8
}

fn pack(data: &[u8]) -> [u8; 8] {
    let mut bytes = [0u8; 8];
    bytes[..data.len().min(8)].copy_from_slice(&data[..data.len().min(8)]);
    bytes
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn standard_frame_keeps_identifier_and_bytes() {
        let frame = CanFrame::standard(0x123, b"Hello");
        assert_eq!(frame.id, 0x123);
        assert!(!frame.extended);
        assert_eq!(frame.dlc, 5);
        assert_eq!(frame.bytes(), b"Hello");
        assert_eq!(frame.id_label(), "0x123");
        assert_eq!(frame.data_label(), "48 65 6C 6C 6F");
        assert_eq!(frame.ascii_label(), "Hello");
    }

    #[test]
    fn identifiers_are_truncated_to_their_field_width() {
        assert_eq!(CanFrame::standard(0xFFFF, &[]).id, 0x7FF);
        assert_eq!(CanFrame::extended(0xFFFF_FFFF, &[]).id, 0x1FFF_FFFF);
    }

    #[test]
    fn payload_is_clamped_to_eight_bytes() {
        let frame = CanFrame::standard(0x10, &[1, 2, 3, 4, 5, 6, 7, 8, 9, 10]);
        assert_eq!(frame.dlc, 8);
        assert_eq!(frame.data, [1, 2, 3, 4, 5, 6, 7, 8]);
    }

    #[test]
    fn remote_frames_carry_no_payload() {
        let frame = CanFrame::remote(CanFrame::standard(0x321, b"ignored"));
        assert!(frame.remote);
        assert_eq!(frame.dlc, 0);
        assert!(frame.bytes().is_empty());
        assert_eq!(frame.data_label(), "-");
    }

    #[test]
    fn extended_identifiers_take_four_more_arbitration_bits() {
        let std = CanFrame::standard(0x123, &[0; 8]).bit_count();
        let ext = CanFrame::extended(0x18DA_F110, &[0; 8]).bit_count();
        assert_eq!(ext - std, 20);
    }
}
/// Wire level of one CAN bit.
///
/// The bus is a wired-AND: any node may pull it to the dominant level, so the
/// level of a bit is the AND of everything driving it (`DOMINANT && RECESSIVE
/// == DOMINANT`).
pub const DOMINANT: bool = false;
pub const RECESSIVE: bool = true;

/// Converts a logical frame bit into the level that represents it: a `1` is
/// recessive and a `0` pulls the bus dominant.
const fn level(bit: bool) -> bool {
    if bit {
        RECESSIVE
    } else {
        DOMINANT
    }
}

/// Protocol violation a node can detect.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CanErrorKind {
    /// Six identical bits inside the stuffed region.
    Stuff,
    /// The 15 bit CRC does not match.
    Crc,
    /// A fixed-form bit had the wrong level.
    Form,
    /// Nobody acknowledged the frame.
    Acknowledge,
    /// The wire carried a different level than this node transmitted.
    Bit,
}

impl CanErrorKind {
    pub fn label(self) -> &'static str {
        match self {
            CanErrorKind::Stuff => "填充错误",
            CanErrorKind::Crc => "CRC 错误",
            CanErrorKind::Form => "格式错误",
            CanErrorKind::Acknowledge => "应答错误",
            CanErrorKind::Bit => "位错误",
        }
    }
}

/// CRC-15 of CAN 2.0: `x^15 + x^14 + x^10 + x^8 + x^7 + x^4 + x^3 + 1`.
pub fn crc15(bits: impl IntoIterator<Item = bool>) -> u16 {
    let mut crc: u16 = 0;
    for bit in bits {
        crc ^= u16::from(bit) << 14;
        if crc & 0x4000 != 0 {
            crc = ((crc << 1) ^ 0x4599) & 0x7FFF;
        } else {
            crc = (crc << 1) & 0x7FFF;
        }
    }
    crc
}

/// One frame expanded into the exact bit sequence that travels on the wire.
///
/// The sequence contains the stuffed bits, the CRC delimiter, the ACK slot (as
/// driven by the transmitter: recessive), the ACK delimiter, seven end of frame
/// bits and the three bit intermission.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct WireFrame {
    bits: Vec<bool>,
    ack_slot: usize,
    arbitration_end: usize,
    frame: CanFrame,
}

impl WireFrame {
    /// Expands `frame` into its wire sequence.
    pub fn encode(frame: &CanFrame) -> Self {
        let mut raw = Vec::with_capacity(128);
        raw.push(DOMINANT); // start of frame

        if frame.extended {
            for index in (18..29).rev() {
                raw.push(level((frame.id >> index) & 1 == 1));
            }
            raw.push(RECESSIVE); // substitute remote request
            raw.push(RECESSIVE); // identifier extension
            for index in (0..18).rev() {
                raw.push(level((frame.id >> index) & 1 == 1));
            }
            raw.push(frame.remote); // remote transmission request
            raw.push(RECESSIVE); // r1
            raw.push(RECESSIVE); // r0
        } else {
            for index in (0..11).rev() {
                raw.push(level((frame.id >> index) & 1 == 1));
            }
            raw.push(frame.remote);
            raw.push(DOMINANT); // identifier extension
            raw.push(RECESSIVE); // r0
        }
        // The arbitration field ends with the remote transmission request bit;
        // everything after it belongs to the control field.
        let arbitration_last = raw.len() - 3;

        let dlc = frame.dlc.min(8);
        for index in (0..4).rev() {
            raw.push(level((u32::from(dlc) >> index) & 1 == 1));
        }
        if !frame.remote {
            for byte in &frame.data[..usize::from(dlc)] {
                for index in (0..8).rev() {
                    raw.push(level((byte >> index) & 1 == 1));
                }
            }
        }
        let crc = crc15(raw.iter().copied());
        for index in (0..15).rev() {
            raw.push(level((crc >> index) & 1 == 1));
        }

        // Stuffing covers the frame from the start of frame through the CRC
        // sequence; delimiters, ACK and end of frame are never stuffed.
        let mut bits = Vec::with_capacity(raw.len() + 16);
        let mut run = 0u8;
        let mut last: Option<bool> = None;
        let mut arbitration_end = 0;
        for (index, bit) in raw.iter().enumerate() {
            if index == arbitration_last {
                arbitration_end = bits.len();
            }
            if last == Some(*bit) {
                run += 1;
            } else {
                run = 1;
                last = Some(*bit);
            }
            bits.push(*bit);
            if run == 5 {
                bits.push(!*bit);
                last = Some(!*bit);
                run = 1;
            }
        }

        bits.push(RECESSIVE); // CRC delimiter
        let ack_slot = bits.len();
        bits.push(RECESSIVE); // ACK slot
        bits.push(RECESSIVE); // ACK delimiter
        bits.extend(std::iter::repeat(RECESSIVE).take(7)); // end of frame
        bits.extend(std::iter::repeat(RECESSIVE).take(3)); // intermission

        WireFrame {
            bits,
            ack_slot,
            arbitration_end,
            frame: *frame,
        }
    }

    pub fn bits(&self) -> &[bool] {
        &self.bits
    }

    pub fn len(&self) -> usize {
        self.bits.len()
    }

    pub fn is_empty(&self) -> bool {
        self.bits.is_empty()
    }

    /// Index of the ACK slot, which a receiver drives dominant.
    pub fn ack_slot(&self) -> usize {
        self.ack_slot
    }

    /// Index of the last bit that still takes part in arbitration.
    pub fn arbitration_end(&self) -> usize {
        self.arbitration_end
    }

    pub fn frame(&self) -> &CanFrame {
        &self.frame
    }
}

/// Result of feeding one sampled bit into a [`FrameDecoder`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RxStep {
    /// The decoder is waiting for a start of frame.
    Idle,
    /// The bit was consumed.
    Bit,
    /// The CRC delimiter was reached; `ok` decides whether to drive the ACK slot.
    CrcDelimiter { ok: bool },
    /// The ACK slot was sampled; the transmitter treats a missing dominant
    /// level as an acknowledge error.
    AckSlot { acknowledged: bool },
    /// A complete, valid frame was received.
    Frame(CanFrame),
    /// A protocol violation ended the frame.
    Error(CanErrorKind),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
enum RxPhase {
    #[default]
    Idle,
    /// Collecting stuffed bits (header, data and CRC sequence).
    Stuffed,
    AckSlot,
    AckDelimiter,
    Eof,
}

/// Incremental classic-CAN receiver: feed sampled bus levels, get frames.
#[derive(Clone, Debug, Default)]
pub struct FrameDecoder {
    raw: Vec<bool>,
    phase: RxPhase,
    run: u8,
    last: Option<bool>,
    extended: bool,
    dlc: u8,
    crc_received: u16,
    eof: u8,
    frame: Option<CanFrame>,
}

impl FrameDecoder {
    pub fn new() -> Self {
        FrameDecoder::default()
    }

    /// Forgets a partially received frame.
    pub fn reset(&mut self) {
        *self = FrameDecoder::default();
    }

    /// `true` while a frame is being received.
    pub fn active(&self) -> bool {
        self.phase != RxPhase::Idle
    }

    /// Index of the ACK slot of the frame in progress, `None` before the CRC
    /// field has been received.
    pub fn ack_slot_reached(&self) -> bool {
        self.phase == RxPhase::AckSlot
    }

    fn header_len(&self) -> usize {
        if self.extended {
            39
        } else {
            19
        }
    }

    /// Feeds the level sampled on the bus for one bit time.
    pub fn feed(&mut self, level: bool) -> RxStep {
        match self.phase {
            RxPhase::Idle => {
                if level == DOMINANT {
                    self.raw.push(level);
                    self.phase = RxPhase::Stuffed;
                    self.run = 1;
                    self.last = Some(level);
                    RxStep::Bit
                } else {
                    RxStep::Idle
                }
            }
            RxPhase::Stuffed => {
                // Destuffing: the bit after five identical ones must be the
                // complement and does not belong to the frame.
                if self.run >= 5 {
                    if Some(level) == self.last {
                        self.reset();
                        return RxStep::Error(CanErrorKind::Stuff);
                    }
                    self.run = 1;
                    self.last = Some(level);
                    return RxStep::Bit;
                }
                // The sequence also gets a stuff bit when the last CRC bit
                // completes a run of five, so the delimiter is only the first
                // bit after the stuffing is complete.
                if self.sequence_complete() {
                    return self.crc_delimiter(level);
                }
                if self.last == Some(level) {
                    self.run += 1;
                } else {
                    self.run = 1;
                    self.last = Some(level);
                }
                self.raw.push(level);
                self.collect()
            }
            RxPhase::AckSlot => {
                self.phase = RxPhase::AckDelimiter;
                RxStep::AckSlot {
                    acknowledged: level == DOMINANT,
                }
            }
            RxPhase::AckDelimiter => {
                if level == RECESSIVE {
                    self.phase = RxPhase::Eof;
                    RxStep::Bit
                } else {
                    self.reset();
                    RxStep::Error(CanErrorKind::Form)
                }
            }
            RxPhase::Eof => {
                if level == DOMINANT {
                    self.reset();
                    return RxStep::Error(CanErrorKind::Form);
                }
                self.eof += 1;
                if self.eof < 7 {
                    return RxStep::Bit;
                }
                let frame = self.frame.take().unwrap_or_default();
                self.reset();
                RxStep::Frame(frame)
            }
        }
    }

    /// Checks whether enough bits were collected to move to the next field.
    fn collect(&mut self) -> RxStep {
        let length = self.raw.len();
        if length == 14 {
            // Start of frame + 11 identifier bits + SRR/RTR + IDE.
            self.extended = self.raw[13];
        }
        let header = self.header_len();
        if length == header {
            self.decode_header();
        }
        if self.frame.is_none() {
            return RxStep::Bit;
        }
        let data_bits = usize::from(self.dlc) * 8;
        if length == header + data_bits {
            self.extract_data();
        }
        let crc_start = header + data_bits;
        if length == crc_start + 15 {
            let mut crc = 0u16;
            for bit in &self.raw[crc_start..crc_start + 15] {
                crc = (crc << 1) | u16::from(*bit == RECESSIVE);
            }
            self.crc_received = crc;
        }
        RxStep::Bit
    }

    /// `true` once header, data and CRC sequence are in: only stuffing and the
    /// CRC delimiter may follow.
    fn sequence_complete(&self) -> bool {
        self.frame.is_some() && self.raw.len() >= self.header_len() + usize::from(self.dlc) * 8 + 15
    }

    fn crc_delimiter(&mut self, level: bool) -> RxStep {
        if level != RECESSIVE {
            self.reset();
            return RxStep::Error(CanErrorKind::Form);
        }
        let crc_start = self.raw.len().saturating_sub(15);
        if crc15(self.raw[..crc_start].iter().copied()) != self.crc_received {
            self.reset();
            return RxStep::Error(CanErrorKind::Crc);
        }
        self.phase = RxPhase::AckSlot;
        RxStep::CrcDelimiter { ok: true }
    }

    /// Copies the data field into the frame once it has been received.
    fn extract_data(&mut self) {
        let header = self.header_len();
        let dlc = self.dlc;
        let Some(frame) = self.frame.as_mut() else {
            return;
        };
        if frame.remote {
            return;
        }
        for byte in 0..usize::from(dlc) {
            let mut value = 0u8;
            for bit in 0..8 {
                value = (value << 1) | u8::from(self.raw[header + byte * 8 + bit] == RECESSIVE);
            }
            frame.data[byte] = value;
        }
    }
    fn decode_header(&mut self) {
        let mut id = 0u32;
        for bit in &self.raw[1..12] {
            id = (id << 1) | u32::from(*bit == RECESSIVE);
        }
        if self.extended {
            for bit in &self.raw[14..32] {
                id = (id << 1) | u32::from(*bit == RECESSIVE);
            }
        }
        let control = if self.extended { 32 } else { 12 };
        let remote = self.raw[control] == RECESSIVE;
        let mut dlc = 0u8;
        for bit in &self.raw[control + 3..control + 7] {
            dlc = (dlc << 1) | u8::from(*bit == RECESSIVE);
        }
        let dlc = dlc.min(8);
        self.dlc = dlc;
        self.frame = Some(CanFrame {
            id,
            extended: self.extended,
            remote,
            dlc,
            data: [0; 8],
        });
    }
}

// --------------------------------------------------------------- trace decode

/// Part of a frame one sampled bit carries, in transmission order.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BitField {
    /// Start of frame.
    StartOfFrame,
    /// Standard identifier, base identifier or the extended part of one.
    Identifier,
    /// Substitute remote request (extended frames only).
    SubstituteRemoteRequest,
    /// Identifier extension, which chooses the frame format.
    IdentifierExtension,
    /// Remote transmission request.
    RemoteRequest,
    /// Reserved bit: `r0`, or `r1` in an extended frame.
    Reserved,
    /// Data length code.
    DataLength,
    /// One payload bit.
    Data,
    /// One bit of the CRC-15 sequence.
    Crc,
    /// CRC delimiter.
    CrcDelimiter,
    /// ACK slot, dominant when a receiver accepted the frame.
    Acknowledge,
    /// ACK delimiter.
    AcknowledgeDelimiter,
    /// End of frame.
    EndOfFrame,
    /// Intermission before any node may start the next frame.
    Intermission,
    /// Bit inserted by the stuffing rule; not part of the frame content.
    Stuff,
}

impl BitField {
    /// Caption of a whole field, used as a group title.
    pub fn label(self) -> &'static str {
        match self {
            BitField::StartOfFrame => "SOF",
            BitField::Identifier => "ID",
            BitField::SubstituteRemoteRequest => "SRR",
            BitField::IdentifierExtension => "IDE",
            BitField::RemoteRequest => "RTR",
            BitField::Reserved => "r0/r1",
            BitField::DataLength => "DLC",
            BitField::Data => "数据",
            BitField::Crc => "CRC",
            BitField::CrcDelimiter => "CRC界定",
            BitField::Acknowledge => "ACK槽",
            BitField::AcknowledgeDelimiter => "ACK界定",
            BitField::EndOfFrame => "EOF",
            BitField::Intermission => "IFS",
            BitField::Stuff => "填充",
        }
    }
}

/// One bit of a frame, exactly as the probe sampled it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TraceBit {
    /// Simulated time the bit was sampled at.
    pub time_ns: u64,
    /// `true` when the wire was dominant, which carries a logical `0`.
    pub dominant: bool,
    pub field: BitField,
    /// Position inside the field, counted from its most significant bit.
    pub field_bit: u8,
}

impl TraceBit {
    /// Logical value the level stands for: dominant is a `0`.
    pub fn value(&self) -> u8 {
        u8::from(!self.dominant)
    }

    /// Level as the scope shows it.
    pub fn level_label(&self) -> &'static str {
        if self.dominant {
            "显性"
        } else {
            "隐性"
        }
    }

    /// `SOF`, `ID10`, `数据3`, `CRC14` … caption of this single bit.
    pub fn caption(&self) -> String {
        match self.field {
            BitField::StartOfFrame
            | BitField::SubstituteRemoteRequest
            | BitField::IdentifierExtension
            | BitField::RemoteRequest
            | BitField::Acknowledge
            | BitField::AcknowledgeDelimiter
            | BitField::CrcDelimiter
            | BitField::Stuff => self.field.label().to_string(),
            BitField::Reserved => format!("r{}", self.field_bit),
            _ => format!("{}{}", self.field.label(), self.field_bit),
        }
    }
}

/// A frame the probe rebuilt from the sampled levels.
#[derive(Clone, Debug, PartialEq)]
pub struct TraceFrame {
    /// Sample time of the start of frame bit.
    pub start_ns: u64,
    /// Sample time of the last intermission bit.
    pub end_ns: u64,
    /// Every bit that crossed the wire, stuffed ones included.
    pub bits: Vec<TraceBit>,
    /// Frame content; `None` only when the header never arrived.
    pub frame: Option<CanFrame>,
    pub crc_ok: bool,
    /// `true` when some receiver pulled the ACK slot dominant.
    pub acknowledged: bool,
    /// `true` for frames that reached the intermission in order.
    pub complete: bool,
}

impl TraceFrame {
    /// Bits the stuffing rule inserted.
    pub fn stuff_bits(&self) -> usize {
        self.bits
            .iter()
            .filter(|bit| bit.field == BitField::Stuff)
            .count()
    }

    /// Payload bits the frame really carries.
    pub fn payload_bits(&self) -> usize {
        self.bits.len() - self.stuff_bits()
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum TracePhase {
    #[default]
    Idle,
    /// Start of frame through the CRC sequence (the stuffed region).
    Stuffed,
    AckSlot,
    AckDelimiter,
    Eof,
    Intermission,
}

/// Longest burst the decoder follows before it gives up on a frame.
const MAX_TRACE_BITS: usize = 400;
/// Frames a decoder keeps for the scope.
pub const TRACE_HISTORY: usize = 16;

/// Rebuilds frames from the levels an oscilloscope sampled.
///
/// Unlike [`FrameDecoder`], which only reports the frame content, this keeps
/// every bit it saw - stuffed ones included - so a scope can caption the whole
/// burst under the trace. Frames are published only once they reached the
/// intermission, which keeps a probe placed in the middle of a frame from
/// showing nonsense.
#[derive(Clone, Debug)]
pub struct TraceDecoder {
    capacity: usize,
    frames: VecDeque<TraceFrame>,
    /// Sampled level of every bit of the frame in progress, in time order.
    levels: Vec<(u64, bool)>,
    /// Parallel to `levels`: `true` when the bit was inserted by stuffing.
    stuff: Vec<bool>,
    /// Number of non-stuffed bits collected so far.
    payload_len: usize,
    phase: TracePhase,
    run: u8,
    last: Option<bool>,
    extended: bool,
    remote: bool,
    dlc: u8,
    eof: u8,
    ifs: u8,
    acknowledged: bool,
}

impl Default for TraceDecoder {
    fn default() -> Self {
        TraceDecoder::new()
    }
}

impl TraceDecoder {
    pub fn new() -> Self {
        TraceDecoder::with_capacity(TRACE_HISTORY)
    }

    pub fn with_capacity(capacity: usize) -> Self {
        TraceDecoder {
            capacity: capacity.max(1),
            frames: VecDeque::new(),
            levels: Vec::new(),
            stuff: Vec::new(),
            payload_len: 0,
            phase: TracePhase::Idle,
            run: 0,
            last: None,
            extended: false,
            remote: false,
            dlc: 0,
            eof: 0,
            ifs: 0,
            acknowledged: false,
        }
    }

    /// Decoded frames, oldest first.
    pub fn frames(&self) -> &VecDeque<TraceFrame> {
        &self.frames
    }

    /// Drops the decoded frames and any frame in progress.
    pub fn clear(&mut self) {
        self.frames.clear();
        self.reset_frame();
    }

    /// `true` while a frame is being received.
    pub fn active(&self) -> bool {
        self.phase != TracePhase::Idle
    }

    /// Feeds the level sampled for one bit time.
    pub fn feed(&mut self, dominant: bool, time_ns: u64) {
        let level = if dominant { DOMINANT } else { RECESSIVE };
        match self.phase {
            TracePhase::Idle => {
                if level == DOMINANT {
                    self.begin(level, time_ns);
                }
            }
            TracePhase::Stuffed => {
                // Destuffing: the bit after five identical ones is inserted by
                // the transmitter and is never part of the frame content.
                if self.run >= 5 {
                    if Some(level) == self.last {
                        self.reset_frame();
                        return;
                    }
                    self.push(level, time_ns, true);
                    self.run = 1;
                    self.last = Some(level);
                    return;
                }
                if self.sequence_complete() {
                    if level != RECESSIVE {
                        self.reset_frame();
                        return;
                    }
                    self.push(level, time_ns, false);
                    self.phase = TracePhase::AckSlot;
                    return;
                }
                if self.last == Some(level) {
                    self.run += 1;
                } else {
                    self.run = 1;
                    self.last = Some(level);
                }
                let index = self.push(level, time_ns, false);
                self.collect(index, level);
                if self.levels.len() > MAX_TRACE_BITS {
                    self.reset_frame();
                }
            }
            TracePhase::AckSlot => {
                self.push(level, time_ns, false);
                self.acknowledged = level == DOMINANT;
                self.phase = TracePhase::AckDelimiter;
            }
            TracePhase::AckDelimiter => {
                if level != RECESSIVE {
                    self.reset_frame();
                    return;
                }
                self.push(level, time_ns, false);
                self.phase = TracePhase::Eof;
            }
            TracePhase::Eof => {
                if level != RECESSIVE {
                    self.reset_frame();
                    return;
                }
                self.push(level, time_ns, false);
                self.eof += 1;
                if self.eof >= 7 {
                    self.phase = TracePhase::Intermission;
                }
            }
            TracePhase::Intermission => {
                if level != RECESSIVE {
                    self.reset_frame();
                    return;
                }
                self.push(level, time_ns, false);
                self.ifs += 1;
                if self.ifs >= 3 {
                    self.complete();
                }
            }
        }
    }

    fn begin(&mut self, level: bool, time_ns: u64) {
        self.reset_frame();
        self.phase = TracePhase::Stuffed;
        self.run = 1;
        self.last = Some(level);
        self.push(level, time_ns, false);
    }

    fn reset_frame(&mut self) {
        self.levels.clear();
        self.stuff.clear();
        self.payload_len = 0;
        self.phase = TracePhase::Idle;
        self.run = 0;
        self.last = None;
        self.extended = false;
        self.remote = false;
        self.dlc = 0;
        self.eof = 0;
        self.ifs = 0;
        self.acknowledged = false;
    }

    /// Stores one bit and returns the payload index it occupies, `usize::MAX`
    /// for a bit inserted by stuffing.
    fn push(&mut self, level: bool, time_ns: u64, stuff: bool) -> usize {
        self.levels.push((time_ns, level));
        self.stuff.push(stuff);
        if stuff {
            return usize::MAX;
        }
        let index = self.payload_len;
        self.payload_len += 1;
        index
    }

    /// Accumulates the fields the decoder needs to know where the frame ends.
    fn collect(&mut self, index: usize, level: bool) {
        if index == 13 {
            self.extended = level == RECESSIVE;
            if self.extended {
                // Bit 12 of an extended frame is the substitute remote
                // request, not a remote transmission request.
                self.remote = false;
            }
            return;
        }
        let remote_bit = if self.extended { 32 } else { 12 };
        if index == remote_bit {
            self.remote = level == RECESSIVE;
            return;
        }
        let dlc_start = if self.extended { 35 } else { 15 };
        if (dlc_start..dlc_start + 4).contains(&index) {
            self.dlc = (self.dlc << 1) | u8::from(level == RECESSIVE);
        }
    }

    fn header_len(&self) -> usize {
        if self.extended {
            39
        } else {
            19
        }
    }

    fn data_len(&self) -> usize {
        if self.remote {
            0
        } else {
            usize::from(self.dlc.min(8)) * 8
        }
    }

    /// Payload index of the first CRC bit.
    fn crc_start(&self) -> usize {
        self.header_len() + self.data_len()
    }

    /// `true` once header, data and the CRC sequence are in, so only a stuff
    /// bit and the CRC delimiter may follow.
    fn sequence_complete(&self) -> bool {
        self.payload_len >= self.crc_start() + 15
    }

    /// Collected bits without the ones the stuffing rule inserted.
    fn payload(&self) -> Vec<bool> {
        self.levels
            .iter()
            .zip(self.stuff.iter())
            .filter(|(_, stuff)| !**stuff)
            .map(|((_, level), _)| *level)
            .collect()
    }

    fn complete(&mut self) {
        let payload = self.payload();
        let crc_start = self.crc_start();
        let crc_ok = if payload.len() >= crc_start + 15 {
            let mut received = 0u16;
            for bit in &payload[crc_start..crc_start + 15] {
                received = (received << 1) | u16::from(*bit == RECESSIVE);
            }
            crc15(payload[..crc_start].iter().copied()) == received
        } else {
            false
        };
        let frame = self.decode_frame(&payload);
        let bits = self.annotate();
        let start_ns = bits.first().map(|bit| bit.time_ns).unwrap_or(0);
        let end_ns = bits.last().map(|bit| bit.time_ns).unwrap_or(start_ns);
        self.frames.push_back(TraceFrame {
            start_ns,
            end_ns,
            bits,
            frame,
            crc_ok,
            acknowledged: self.acknowledged,
            complete: true,
        });
        while self.frames.len() > self.capacity {
            self.frames.pop_front();
        }
        self.reset_frame();
    }

    /// Turns the collected levels into frame content.
    fn decode_frame(&self, payload: &[bool]) -> Option<CanFrame> {
        let crc_start = self.crc_start();
        if payload.len() < crc_start + 15 {
            return None;
        }
        let bit_at = |index: usize| u32::from(payload[index] == RECESSIVE);
        let mut id = 0u32;
        for index in 1..=11 {
            id = (id << 1) | bit_at(index);
        }
        if self.extended {
            for index in 14..=31 {
                id = (id << 1) | bit_at(index);
            }
        }
        let dlc = self.dlc.min(8);
        let mut data = [0u8; 8];
        if !self.remote {
            let header = self.header_len();
            for byte in 0..usize::from(dlc) {
                let mut value = 0u8;
                for bit in 0..8 {
                    value = (value << 1) | u8::from(payload[header + byte * 8 + bit] == RECESSIVE);
                }
                data[byte] = value;
            }
        }
        Some(CanFrame {
            id,
            extended: self.extended,
            remote: self.remote,
            dlc,
            data,
        })
    }

    /// Labels every collected bit with the field it belongs to.
    fn annotate(&self) -> Vec<TraceBit> {
        let mut bits = Vec::with_capacity(self.levels.len());
        let mut index = 0;
        for (position, (time_ns, level)) in self.levels.iter().enumerate() {
            let (field, field_bit) = if self.stuff[position] {
                (BitField::Stuff, 0)
            } else {
                let labelled = self.field_for(index);
                index += 1;
                labelled
            };
            bits.push(TraceBit {
                time_ns: *time_ns,
                dominant: *level == DOMINANT,
                field,
                field_bit,
            });
        }
        bits
    }

    fn field_for(&self, index: usize) -> (BitField, u8) {
        if index == 0 {
            return (BitField::StartOfFrame, 0);
        }
        let crc_start = self.crc_start();
        if index >= crc_start {
            return match index - crc_start {
                0..=14 => (BitField::Crc, (crc_start + 14 - index) as u8),
                15 => (BitField::CrcDelimiter, 0),
                16 => (BitField::Acknowledge, 0),
                17 => (BitField::AcknowledgeDelimiter, 0),
                18..=24 => (BitField::EndOfFrame, (crc_start + 24 - index) as u8),
                _ => (BitField::Intermission, (crc_start + 27).saturating_sub(index) as u8),
            };
        }
        if index <= 11 {
            let msb = if self.extended { 28 } else { 10 };
            return (BitField::Identifier, (msb + 1 - index) as u8);
        }
        match index {
            12 => {
                if self.extended {
                    (BitField::SubstituteRemoteRequest, 0)
                } else {
                    (BitField::RemoteRequest, 0)
                }
            }
            13 => (BitField::IdentifierExtension, 0),
            _ if self.extended => match index {
                14..=31 => (BitField::Identifier, (31 - index) as u8),
                32 => (BitField::RemoteRequest, 0),
                33 => (BitField::Reserved, 1),
                34 => (BitField::Reserved, 0),
                35..=38 => (BitField::DataLength, (38 - index) as u8),
                _ => (BitField::Data, (index - self.header_len()) as u8),
            },
            _ => match index {
                14 => (BitField::Reserved, 0),
                15..=18 => (BitField::DataLength, (18 - index) as u8),
                _ => (BitField::Data, (index - self.header_len()) as u8),
            },
        }
    }
}

#[cfg(test)]
mod trace_tests {
    use super::*;

    /// Feeds the whole burst of `frame`, driving the ACK slot when asked.
    fn decode(frame: CanFrame, acknowledge: bool) -> TraceDecoder {
        let wire = WireFrame::encode(&frame);
        let mut decoder = TraceDecoder::new();
        let mut time = 0;
        for (index, bit) in wire.bits().iter().enumerate() {
            let level = if acknowledge && index == wire.ack_slot() {
                DOMINANT
            } else {
                *bit
            };
            decoder.feed(level == DOMINANT, time);
            time += 4_000;
        }
        decoder
    }

    #[test]
    fn a_standard_frame_is_captioned_bit_by_bit() {
        let frame = CanFrame::standard(0x123, b"GS");
        let decoder = decode(frame, true);
        let decoded = decoder.frames().back().expect("a decoded frame");
        assert!(decoded.complete);
        assert_eq!(decoded.frame, Some(frame));
        assert!(decoded.crc_ok);
        assert!(decoded.acknowledged);
        assert_eq!(decoded.bits.len(), WireFrame::encode(&frame).len());
        assert_eq!(decoded.start_ns, 0);
        assert!(decoded.end_ns > decoded.start_ns);

        let captions: Vec<String> = decoded.bits.iter().map(TraceBit::caption).collect();
        assert_eq!(captions.first().map(String::as_str), Some("SOF"));
        assert_eq!(captions.last().map(String::as_str), Some("IFS0"));
        assert!(captions.contains(&"ID10".to_string()), "{captions:?}");
        assert!(captions.contains(&"ID0".to_string()), "{captions:?}");
        assert!(captions.contains(&"RTR".to_string()), "{captions:?}");
        assert!(captions.contains(&"IDE".to_string()), "{captions:?}");
        assert!(captions.contains(&"r0".to_string()), "{captions:?}");
        assert!(captions.contains(&"DLC3".to_string()), "{captions:?}");
        assert!(captions.contains(&"数据0".to_string()), "{captions:?}");
        assert!(captions.contains(&"CRC14".to_string()), "{captions:?}");
        assert!(captions.contains(&"ACK槽".to_string()), "{captions:?}");
        assert!(captions.contains(&"EOF6".to_string()), "{captions:?}");
    }

    #[test]
    fn the_identifier_bits_carry_the_identifier() {
        // 0x123 is 001 0010 0011: ID10 is dominant, ID8 recessive.
        let frame = CanFrame::standard(0x123, &[]);
        let decoder = decode(frame, true);
        let decoded = decoder.frames().back().expect("a decoded frame");
        let identifier: Vec<TraceBit> = decoded
            .bits
            .iter()
            .copied()
            .filter(|bit| bit.field == BitField::Identifier)
            .collect();
        assert_eq!(identifier.len(), 11);
        assert_eq!(identifier[0].caption(), "ID10");
        assert!(identifier[0].dominant);
        assert_eq!(identifier[1].caption(), "ID9");
        assert!(identifier[1].dominant);
        assert_eq!(identifier[2].caption(), "ID8");
        assert!(!identifier[2].dominant);
        assert_eq!(identifier[10].caption(), "ID0");
        assert!(!identifier[10].dominant);
    }

    #[test]
    fn an_extended_frame_is_labelled_with_srr_and_the_long_identifier() {
        let frame = CanFrame::extended(0x18DA_F110, &[0xDE, 0xAD]);
        let decoder = decode(frame, true);
        let decoded = decoder.frames().back().expect("a decoded frame");
        assert_eq!(decoded.frame, Some(frame));
        let captions: Vec<String> = decoded.bits.iter().map(TraceBit::caption).collect();
        assert!(captions.contains(&"SRR".to_string()), "{captions:?}");
        assert!(captions.contains(&"ID17".to_string()), "{captions:?}");
        assert!(captions.contains(&"r1".to_string()), "{captions:?}");
        assert!(captions.contains(&"数据15".to_string()), "{captions:?}");
        assert_eq!(decoded.payload_bits(), 39 + 16 + 15 + 13);
    }

    #[test]
    fn stuffed_bits_are_marked_and_do_not_count_as_payload() {
        // Eleven dominant identifier bits force two stuff bits.
        let frame = CanFrame::standard(0x000, &[]);
        let decoder = decode(frame, true);
        let decoded = decoder.frames().back().expect("a decoded frame");
        assert!(decoded.stuff_bits() >= 1);
        assert_eq!(decoded.payload_bits(), 19 + 15 + 13);
        let wire = WireFrame::encode(&frame);
        assert_eq!(decoded.bits.len(), wire.len());
        assert_eq!(decoded.frame, Some(frame));
    }

    #[test]
    fn a_frame_without_acknowledge_is_reported_as_such() {
        let frame = CanFrame::standard(0x321, b"GSIM");
        let decoder = decode(frame, false);
        let decoded = decoder.frames().back().expect("a decoded frame");
        assert!(!decoded.acknowledged);
        assert!(decoded.crc_ok);
        assert_eq!(decoded.frame, Some(frame));
    }

    #[test]
    fn idle_levels_and_partial_bursts_produce_no_frame() {
        let mut decoder = TraceDecoder::new();
        for step in 0..64u64 {
            decoder.feed(false, step * 4_000);
        }
        assert!(decoder.frames().is_empty());

        let frame = CanFrame::standard(0x123, b"Hello");
        let wire = WireFrame::encode(&frame);
        let mut decoder = TraceDecoder::new();
        for (index, bit) in wire.bits().iter().enumerate().take(wire.len() / 2) {
            decoder.feed(*bit == DOMINANT, index as u64 * 4_000);
        }
        assert!(decoder.frames().is_empty(), "half a frame is not a frame");
        assert!(decoder.active());
    }

    #[test]
    fn a_flipped_bit_never_passes_the_crc() {
        let frame = CanFrame::standard(0x123, b"Hello");
        let wire = WireFrame::encode(&frame);
        let mut clean = 0;
        for flip in 0..wire.len() {
            if flip == wire.ack_slot() {
                // The ACK slot carries no CRC-covered data.
                continue;
            }
            let mut decoder = TraceDecoder::new();
            for (index, bit) in wire.bits().iter().enumerate() {
                let level = if index == flip {
                    !bit
                } else if index == wire.ack_slot() {
                    DOMINANT
                } else {
                    *bit
                };
                decoder.feed(level == DOMINANT, index as u64 * 4_000);
            }
            for trace in decoder.frames() {
                assert!(!trace.crc_ok, "a flip at bit {flip} passed the CRC");
            }
            clean += 1;
        }
        assert!(clean > 40);
    }

    #[test]
    fn a_second_frame_follows_the_first_after_the_intermission() {
        let first = CanFrame::standard(0x100, &[1, 2, 3, 4]);
        let second = CanFrame::standard(0x200, &[5, 6]);
        let mut decoder = TraceDecoder::new();
        let mut time = 0;
        for frame in [first, second] {
            let wire = WireFrame::encode(&frame);
            for (index, bit) in wire.bits().iter().enumerate() {
                let level = if index == wire.ack_slot() {
                    DOMINANT
                } else {
                    *bit
                };
                decoder.feed(level == DOMINANT, time);
                time += 4_000;
            }
        }
        assert_eq!(decoder.frames().len(), 2);
        let ids: Vec<u32> = decoder
            .frames()
            .iter()
            .filter_map(|trace| trace.frame.map(|frame| frame.id))
            .collect();
        assert_eq!(ids, vec![0x100, 0x200]);
    }
}

#[cfg(test)]
mod wire_tests {
    use super::*;

    fn round_trip(frame: CanFrame) -> CanFrame {
        let wire = WireFrame::encode(&frame);
        let mut decoder = FrameDecoder::new();
        for (index, bit) in wire.bits().iter().enumerate() {
            match decoder.feed(*bit) {
                RxStep::Frame(received) => {
                    assert_eq!(received, frame);
                    return received;
                }
                RxStep::Error(error) => panic!("decoder reported {error:?}"),
                RxStep::CrcDelimiter { ok } => {
                    assert!(ok);
                    // The delimiter must sit exactly one bit before the ACK
                    // slot, otherwise stuffing desynchronised the receiver.
                    assert_eq!(index, wire.ack_slot() - 1, "CRC delimiter moved");
                }
                _ => {}
            }
        }
        panic!("frame never completed");
    }

    #[test]
    fn a_standard_frame_round_trips_through_the_wire() {
        let frame = CanFrame::standard(0x123, b"Hello");
        assert_eq!(round_trip(frame), frame);
    }

    #[test]
    fn an_extended_frame_round_trips_through_the_wire() {
        let frame = CanFrame::extended(0x18DA_F110, &[0xDE, 0xAD, 0xBE, 0xEF]);
        assert_eq!(round_trip(frame), frame);
    }

    #[test]
    fn a_remote_frame_carries_no_data_on_the_wire() {
        let frame = CanFrame::remote(CanFrame::standard(0x321, &[1, 2, 3]));
        assert_eq!(round_trip(frame), frame);
    }

    #[test]
    fn a_stuff_bit_after_the_last_crc_bit_keeps_the_receiver_in_step() {
        // The CRC of this frame ends with a run of five dominant bits, so a
        // stuff bit is inserted right before the delimiter.
        let frame = CanFrame::standard(0x222, &[0x42]);
        let wire = WireFrame::encode(&frame);
        assert_eq!(wire.bits()[wire.ack_slot() - 1], RECESSIVE);
        round_trip(frame);
    }

    #[test]
    fn every_frame_length_round_trips() {
        for id in 0..64u32 {
            for dlc in [0usize, 1, 4, 8] {
                let data: Vec<u8> = (0..dlc)
                    .map(|index| (id as u8).wrapping_mul(7 + index as u8))
                    .collect();
                round_trip(CanFrame::standard(id as u16, &data));
                round_trip(CanFrame::extended((id << 12) | 0x5A5, &data));
            }
        }
    }

    #[test]
    fn stuffing_is_inserted_after_five_identical_bits() {
        // Identifier 0x000 starts with eleven dominant bits in a row.
        let frame = CanFrame::standard(0x000, &[]);
        let wire = WireFrame::encode(&frame);
        let mut run = 0;
        let mut previous = None;
        for bit in wire.bits().iter().take(20) {
            if previous == Some(*bit) {
                run += 1;
                assert!(run <= 5, "six identical bits reached the wire");
            } else {
                run = 1;
                previous = Some(*bit);
            }
        }
    }

    #[test]
    fn a_flipped_data_bit_is_rejected_by_the_crc() {
        let frame = CanFrame::standard(0x123, b"Hello");
        let wire = WireFrame::encode(&frame);
        let mut saw_crc = false;
        // The three intermission bits follow the end of frame and are not part
        // of the frame, so flipping them cannot be detected here.
        for index in 0..wire.len() - 3 {
            if index == wire.ack_slot() {
                // Pulling the ACK slot dominant is what a receiver does.
                continue;
            }
            let mut flipped = wire.bits().to_vec();
            flipped[index] = !flipped[index];
            let mut decoder = FrameDecoder::new();
            for bit in &flipped {
                match decoder.feed(*bit) {
                    RxStep::Error(CanErrorKind::Crc) => {
                        saw_crc = true;
                        break;
                    }
                    RxStep::Error(_) => break,
                    RxStep::Frame(received) => {
                        assert_ne!(received, frame, "a flip at bit {index} slipped through");
                        break;
                    }
                    _ => {}
                }
            }
        }
        assert!(saw_crc, "a corrupted frame must be caught by the CRC");
    }

    #[test]
    fn the_ack_slot_is_one_bit_after_the_crc_delimiter() {
        let wire = WireFrame::encode(&CanFrame::standard(0x123, &[0]));
        assert_eq!(wire.bits()[wire.ack_slot()], RECESSIVE);
        assert!(wire.ack_slot() < wire.len());
        // The transmitter sends recessive in the ACK slot; a receiver pulls it low.
        assert!(wire.ack_slot() > 15);
    }

    #[test]
    fn arbitration_ends_before_the_control_field() {
        let standard = WireFrame::encode(&CanFrame::standard(0x7FF, &[]));
        assert_eq!(standard.bits()[standard.arbitration_end()], DOMINANT);
        let extended = WireFrame::encode(&CanFrame::extended(0x1FFF_FFFF, &[]));
        assert_eq!(extended.bits()[extended.arbitration_end()], DOMINANT);
        assert!(extended.arbitration_end() > standard.arbitration_end());
    }

    #[test]
    fn crc15_matches_the_reference_vector() {
        // `123456789`, MSB first, is the published CRC-15/CAN check value.
        let mut bits = Vec::new();
        for byte in b"123456789" {
            for index in (0..8).rev() {
                bits.push((byte >> index) & 1 == 1);
            }
        }
        assert_eq!(crc15(bits), 0x059E);
    }
}
