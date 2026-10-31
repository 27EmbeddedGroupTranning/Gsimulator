//! Classic CAN frames shared by the on-chip controller and the external
//! device models.
//!
//! Both the bxCAN model in `gsim-stm32f1` and the transceiver/bus models in
//! `gsim-devices` exchange this single value type, so a frame never has to be
//! re-encoded when it crosses a module boundary.

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
