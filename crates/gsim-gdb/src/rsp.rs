//! Framing of the GDB remote serial protocol.
//!
//! A packet is `$<payload>#<checksum>`, where the checksum is the low byte of
//! the sum of the payload bytes. The characters `#`, `$`, `}` and `*` are
//! escaped inside the payload with `}` followed by the byte XORed with `0x20`.
//! Control character `0x03` sent outside a packet interrupts a running target.

/// Control-C, the byte GDB sends to interrupt a running target.
pub const INTERRUPT: u8 = 0x03;

/// Escape prefix of a payload byte.
const ESCAPE: u8 = b'}';

const HEX_DIGITS: [u8; 16] = *b"0123456789abcdef";

/// Characters that have to be escaped inside a payload.
fn needs_escape(byte: u8) -> bool {
    matches!(byte, b'#' | b'$' | b'*' | ESCAPE)
}

/// Low byte of the sum of `payload`; the remote protocol checksum.
pub fn checksum(payload: &[u8]) -> u8 {
    payload
        .iter()
        .fold(0u8, |sum, byte| sum.wrapping_add(*byte))
}

/// Encodes a payload as it appears on the wire, `$<escaped>#<checksum>`.
///
/// The checksum covers the unescaped payload, which is what GDB computes.
pub fn encode(payload: &str) -> Vec<u8> {
    let bytes = payload.as_bytes();
    let mut wire = Vec::with_capacity(bytes.len() + 8);
    wire.push(b'$');
    for byte in bytes {
        if needs_escape(*byte) {
            wire.push(ESCAPE);
            wire.push(byte ^ 0x20);
        } else {
            wire.push(*byte);
        }
    }
    wire.push(b'#');
    let sum = checksum(bytes);
    wire.push(HEX_DIGITS[usize::from(sum >> 4)]);
    wire.push(HEX_DIGITS[usize::from(sum & 0x0F)]);
    wire
}

/// Removes the `}` escapes from a payload.
pub fn unescape(payload: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(payload.len());
    let mut index = 0;
    while index < payload.len() {
        if payload[index] == ESCAPE && index + 1 < payload.len() {
            out.push(payload[index + 1] ^ 0x20);
            index += 2;
        } else {
            out.push(payload[index]);
            index += 1;
        }
    }
    out
}

/// One decoded element of the receive stream.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Frame {
    /// A payload with a valid checksum, already unescaped.
    Packet(Vec<u8>),
    /// A payload whose checksum did not match; the peer has to retransmit.
    BadChecksum,
    /// A control-C outside of a packet.
    Interrupt,
    /// The buffer does not hold a complete frame yet.
    Incomplete,
}

/// Pulls the first frame out of `buffer`, dropping the bytes it consumed.
///
/// Acknowledgements (`+`) and negative acknowledgements (`-`) are consumed
/// silently: the server does not depend on them, but a peer may still send them.
pub fn next_frame(buffer: &mut Vec<u8>) -> Frame {
    // Drop everything in front of the packet start; a control-C there is an
    // interrupt request rather than junk.
    while let Some(first) = buffer.first() {
        match *first {
            b'$' => break,
            INTERRUPT => {
                buffer.remove(0);
                return Frame::Interrupt;
            }
            _ => {
                buffer.remove(0);
            }
        }
    }
    if buffer.is_empty() {
        return Frame::Incomplete;
    }

    let Some(end) = payload_end(buffer) else {
        return Frame::Incomplete;
    };
    let payload = unescape(&buffer[1..end]);
    let wire = buffer[1..end].to_vec();
    let declared = read_checksum(&buffer[end + 1..end + 3]);
    buffer.drain(..end + 3);

    match declared {
        // GDB and OpenOCD sum the unescaped payload; older peers summed the
        // escaped bytes, so accept either to stay compatible.
        Some(sum) if sum == checksum(&payload) || sum == checksum(&wire) => Frame::Packet(payload),
        _ => Frame::BadChecksum,
    }
}

/// Offset of the `#` that closes the packet, skipping escaped bytes.
fn payload_end(buffer: &[u8]) -> Option<usize> {
    let mut index = 1;
    while index < buffer.len() {
        match buffer[index] {
            ESCAPE => index += 2,
            b'#' => return (index + 2 < buffer.len()).then_some(index),
            _ => index += 1,
        }
    }
    None
}

fn read_checksum(digits: &[u8]) -> Option<u8> {
    let high = (*digits.first()? as char).to_digit(16)?;
    let low = (*digits.get(1)? as char).to_digit(16)?;
    Some((high * 16 + low) as u8)
}

/// Encodes bytes as lower case hex text.
pub fn to_hex(bytes: &[u8]) -> String {
    let mut out = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        out.push(HEX_DIGITS[usize::from(byte >> 4)] as char);
        out.push(HEX_DIGITS[usize::from(byte & 0x0F)] as char);
    }
    out
}

/// Decodes hex text; returns `None` for an odd length or a bad digit.
pub fn from_hex(text: &[u8]) -> Option<Vec<u8>> {
    if text.len() % 2 != 0 {
        return None;
    }
    let mut out = Vec::with_capacity(text.len() / 2);
    for pair in text.chunks(2) {
        let high = (pair[0] as char).to_digit(16)?;
        let low = (pair[1] as char).to_digit(16)?;
        out.push((high * 16 + low) as u8);
    }
    Some(out)
}

/// Parses a hexadecimal number without a `0x` prefix.
pub fn parse_number(text: &str) -> Option<u64> {
    let text = text.trim();
    if text.is_empty() {
        return None;
    }
    u64::from_str_radix(text, 16).ok()
}

/// Little endian hex text of a 32 bit value, as used by the register packets.
pub fn word_to_hex(value: u32) -> String {
    to_hex(&value.to_le_bytes())
}

/// Splits register packet text into 32 bit values.
pub fn words_from_hex(text: &[u8]) -> Option<Vec<u32>> {
    let bytes = from_hex(text)?;
    if bytes.len() % 4 != 0 {
        return None;
    }
    Some(
        bytes
            .chunks(4)
            .map(|chunk| u32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]))
            .collect(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn wire(payload: &[u8]) -> Vec<u8> {
        let mut buffer = Vec::new();
        buffer.push(b'$');
        for byte in payload {
            if needs_escape(*byte) {
                buffer.push(ESCAPE);
                buffer.push(byte ^ 0x20);
            } else {
                buffer.push(*byte);
            }
        }
        buffer.push(b'#');
        let sum = checksum(payload);
        buffer.push(HEX_DIGITS[usize::from(sum >> 4)]);
        buffer.push(HEX_DIGITS[usize::from(sum & 0x0F)]);
        buffer
    }

    #[test]
    fn a_packet_round_trips_through_the_framing() {
        let mut buffer = encode("m8000000,4");
        // A trailing acknowledgement belongs to the next read.
        buffer.push(b'+');
        assert_eq!(
            next_frame(&mut buffer),
            Frame::Packet(b"m8000000,4".to_vec())
        );
        assert_eq!(next_frame(&mut buffer), Frame::Incomplete);
        assert!(buffer.is_empty());
    }

    #[test]
    fn checksum_is_the_low_byte_of_the_sum() {
        assert_eq!(checksum(b"OK"), 0x9A);
        assert_eq!(checksum(b""), 0);
    }

    #[test]
    fn escaped_payloads_are_decoded() {
        let payload = b"a}$*#b".to_vec();
        let mut buffer = wire(&payload);
        assert_eq!(next_frame(&mut buffer), Frame::Packet(payload));
    }

    #[test]
    fn an_incomplete_packet_is_reported_without_being_consumed() {
        let mut buffer = encode("qSupported");
        buffer.truncate(buffer.len() - 3);
        let before = buffer.clone();
        assert_eq!(next_frame(&mut buffer), Frame::Incomplete);
        assert_eq!(buffer, before);
    }

    #[test]
    fn a_bad_checksum_is_detected() {
        let mut buffer = encode("g");
        let length = buffer.len();
        buffer[length - 1] = b'0';
        assert_eq!(next_frame(&mut buffer), Frame::BadChecksum);
    }

    #[test]
    fn acknowledging_bytes_are_ignored() {
        let mut buffer = vec![b'+'];
        buffer.extend_from_slice(&encode("?"));
        assert_eq!(next_frame(&mut buffer), Frame::Packet(b"?".to_vec()));
    }

    #[test]
    fn a_control_c_between_packets_interrupts() {
        let mut buffer = vec![b'+', INTERRUPT];
        assert_eq!(next_frame(&mut buffer), Frame::Interrupt);
        assert!(buffer.is_empty());
    }

    #[test]
    fn words_use_little_endian_hex() {
        assert_eq!(word_to_hex(0x1234_5678), "78563412");
        assert_eq!(
            words_from_hex(b"7856341200000000"),
            Some(vec![0x1234_5678, 0])
        );
    }

    #[test]
    fn hex_helpers_reject_malformed_text() {
        assert_eq!(from_hex(b"abc"), None);
        assert_eq!(from_hex(b"zz"), None);
        assert_eq!(parse_number("08000000"), Some(0x0800_0000));
        assert_eq!(parse_number(""), None);
    }
}
