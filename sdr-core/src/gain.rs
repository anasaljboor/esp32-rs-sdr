//! Gain-code math for the S3 manual-gain check (`main/common/rx_gain_verify.c`).
//!
//! The PHY writes one gain word per table index. Bits 17.. of its first word
//! hold a 16-bit code: RF gain in [15:8], baseband LNA in [7:3], fine gain in
//! [2:0]. When a gain really latches, two PBUS registers show that code in a
//! rearranged form. These functions compute the expected register fields.

/// Number of Wi-Fi gain-table slots on the S3.
pub const S3_GAIN_SLOTS: usize = 83;

/// Highest manual index the S3 accepts. A raw maximum above 82 means the
/// table is not calibrated, so no manual index is exposed (0).
pub const fn s3_gain_max(raw_max: u32) -> u32 {
    if raw_max <= 82 { raw_max } else { 0 }
}

/// The gain code the PHY stores for one table slot.
pub const fn code_from_gain_word(low_word: u32) -> u16 {
    (low_word >> 17) as u16
}

/// Expected 9-bit RF field of PBUS register `0x60006118` (bits 9..17).
pub const fn expected_rf(code: u16) -> u32 {
    0x180 | (code as u32 >> 8)
}

/// Expected 9-bit baseband field of PBUS register `0x6000611c` (bits 18..26).
/// The PBUS word reverses the LNA bits and moves fine gain to [8:6].
pub const fn expected_bb(code: u16) -> u32 {
    let c = code as u32;
    ((c & 7) << 6)
        | ((c & 8) << 2)
        | (c & 0x10)
        | ((c & 0x20) >> 2)
        | ((c & 0x40) >> 4)
        | ((c & 0x80) >> 6)
        | ((c >> 8) & 1)
}

/// Compare live PBUS registers against a gain code.
pub const fn pbus_matches(code: u16, reg_6118: u32, reg_611c: u32) -> bool {
    ((reg_6118 >> 9) & 0x1ff) == expected_rf(code) && ((reg_611c >> 18) & 0x1ff) == expected_bb(code)
}

/// The forced-gain register (`0x6001c02c`) field at bits 23.. reads
/// `1 | index << 1` while `index` is forced.
pub const fn forced_field_matches(reg: u32, index: u32) -> bool {
    (reg >> 23) == (1 | (index << 1))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bb_bit_permutation() {
        assert_eq!(expected_bb(0b111), 0b111 << 6);
        assert_eq!(expected_bb(0x08), 0x20);
        assert_eq!(expected_bb(0x10), 0x10);
        assert_eq!(expected_bb(0x20), 0x08);
        assert_eq!(expected_bb(0x40), 0x04);
        assert_eq!(expected_bb(0x80), 0x02);
        assert_eq!(expected_bb(0x100), 0x01);
    }

    #[test]
    fn rf_field_and_match() {
        let code = 0x3a5;
        assert_eq!(expected_rf(code), 0x183);
        let r6118 = expected_rf(code) << 9;
        let r611c = expected_bb(code) << 18;
        assert!(pbus_matches(code, r6118, r611c));
        assert!(!pbus_matches(code, r6118 ^ (1 << 9), r611c));
    }

    #[test]
    fn limits_and_forced_field() {
        assert_eq!(s3_gain_max(82), 82);
        assert_eq!(s3_gain_max(83), 0);
        assert!(forced_field_matches((1 | 40 << 1) << 23, 40));
        assert!(!forced_field_matches((40 << 1) << 23, 40));
        assert_eq!(code_from_gain_word(0x1234_5678), 0x091a);
    }
}
