//! CRC-32 with the zlib polynomial, as sent in `DATA` headers.
//!
//! The firmware uses the chip ROM's `esp_rom_crc32_le`, which computes the same
//! value. This software version is the reference for tests and host tools.

const fn table() -> [u32; 256] {
    let mut t = [0u32; 256];
    let mut i = 0;
    while i < 256 {
        let mut c = i as u32;
        let mut k = 0;
        while k < 8 {
            c = if c & 1 != 0 { 0xedb8_8320 ^ (c >> 1) } else { c >> 1 };
            k += 1;
        }
        t[i] = c;
        i += 1;
    }
    t
}

static TABLE: [u32; 256] = table();

/// Continue a CRC-32 over `data`. Start with `crc = 0`.
pub fn crc32(crc: u32, data: &[u8]) -> u32 {
    let mut c = !crc;
    for &b in data {
        c = TABLE[((c ^ u32::from(b)) & 0xff) as usize] ^ (c >> 8);
    }
    !c
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn check_value() {
        assert_eq!(crc32(0, b"123456789"), 0xcbf4_3926);
        assert_eq!(crc32(0, b""), 0);
    }

    #[test]
    fn continues_across_chunks() {
        let whole = crc32(0, b"hello world");
        assert_eq!(crc32(crc32(0, b"hello "), b"world"), whole);
    }
}
