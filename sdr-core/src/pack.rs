//! Packing captured dump words into wire formats (`main/common/burst_capture.h`).
//!
//! A dump word holds I in bits 0..9, Q in bits 10..19 (both 10-bit two's
//! complement) and the active gain index in bits 20..27. The packers work in
//! place on the capture buffer viewed as little-endian bytes: every output
//! format is no larger than the words it replaces, and each word is read
//! before any byte of it is overwritten.

use crate::command::Format;

/// Bytes on the wire for `n` samples in `format`.
pub const fn wire_size(n: usize, format: Format) -> usize {
    match format {
        Format::Iq8 => n * 2,
        Format::Iq10 => (n * 20 + 7) / 8,
        Format::Raw32 => n * 4,
    }
}

fn word(bytes: &[u8], j: usize) -> u32 {
    u32::from_le_bytes([bytes[4 * j], bytes[4 * j + 1], bytes[4 * j + 2], bytes[4 * j + 3]])
}

/// Pack the first `n` words of `bytes` (at least `4 * n` long) in place and
/// return the payload length.
pub fn pack_in_place(bytes: &mut [u8], n: usize, format: Format) -> usize {
    assert!(bytes.len() >= 4 * n);
    match format {
        Format::Raw32 => {}
        Format::Iq8 => {
            for j in 0..n {
                let w = word(bytes, j);
                bytes[2 * j] = (w >> 2) as u8;
                bytes[2 * j + 1] = (w >> 12) as u8;
            }
        }
        Format::Iq10 => {
            let mut j = 0;
            while j < n {
                let a = word(bytes, j) & 0xfffff;
                let has_b = j + 1 < n;
                let b = if has_b { word(bytes, j + 1) & 0xfffff } else { 0 };
                let p = 5 * (j / 2);
                bytes[p] = a as u8;
                bytes[p + 1] = (a >> 8) as u8;
                bytes[p + 2] = ((a >> 16) | (b << 4)) as u8;
                if has_b {
                    bytes[p + 3] = (b >> 4) as u8;
                    bytes[p + 4] = (b >> 12) as u8;
                }
                j += 2;
            }
        }
    }
    wire_size(n, format)
}

/// Decode one IQ10 sample from a packed payload (host-side helper and tests).
pub fn unpack_iq10(payload: &[u8], index: usize) -> (i16, i16) {
    let bit = index * 20;
    let mut v = 0u32;
    for k in 0..3 {
        if let Some(b) = payload.get(bit / 8 + k) {
            v |= u32::from(*b) << (8 * k);
        }
    }
    let v = (v >> (bit % 8)) & 0xfffff;
    let sign10 = |x: u32| ((x as i16) << 6) >> 6;
    (sign10(v & 0x3ff), sign10(v >> 10))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn words_to_bytes(words: &[u32]) -> [u8; 64] {
        let mut out = [0u8; 64];
        for (j, w) in words.iter().enumerate() {
            out[4 * j..4 * j + 4].copy_from_slice(&w.to_le_bytes());
        }
        out
    }

    fn dump_word(i: i16, q: i16, gain: u32) -> u32 {
        (i as u32 & 0x3ff) | ((q as u32 & 0x3ff) << 10) | (gain << 20)
    }

    #[test]
    fn sizes() {
        assert_eq!(wire_size(4096, Format::Iq8), 8192);
        assert_eq!(wire_size(4096, Format::Iq10), 10240);
        assert_eq!(wire_size(3, Format::Iq10), 8);
        assert_eq!(wire_size(16380, Format::Raw32), 65520);
    }

    #[test]
    fn iq8_takes_upper_eight_bits() {
        let words = [dump_word(-512, 511, 40), dump_word(4, -4, 40), dump_word(-1, 3, 0)];
        let mut b = words_to_bytes(&words);
        assert_eq!(pack_in_place(&mut b, 3, Format::Iq8), 6);
        let got: [i8; 6] = core::array::from_fn(|k| b[k] as i8);
        assert_eq!(got, [-128, 127, 1, -1, -1, 0]);
    }

    #[test]
    fn iq10_round_trips_including_odd_tail() {
        let samples: [(i16, i16); 7] =
            [(-512, 511), (0, -1), (1, 2), (-3, 300), (511, -512), (7, -7), (-100, 100)];
        let mut words = [0u32; 7];
        for (w, (i, q)) in words.iter_mut().zip(samples) {
            *w = dump_word(i, q, 0x55);
        }
        let mut b = words_to_bytes(&words);
        let len = pack_in_place(&mut b, 7, Format::Iq10);
        assert_eq!(len, 18);
        for (k, s) in samples.iter().enumerate() {
            assert_eq!(unpack_iq10(&b[..len], k), *s, "sample {k}");
        }
    }
}
