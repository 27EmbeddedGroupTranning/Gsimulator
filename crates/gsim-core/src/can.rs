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
