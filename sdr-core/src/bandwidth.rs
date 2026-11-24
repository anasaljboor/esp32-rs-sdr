//! Analog low-pass filter code for a requested bandwidth (`main/common/rx_bandwidth.h`).
//!
//! The S3 curve was measured by the C project: noise spectrum at 2300 MHz,
//! gain 75, 80 MS/s, approximate -3 dB full width.

/// `(filter code, approximate full bandwidth in MHz)`, widest first.
pub type Curve = &'static [(u8, u8)];

pub const ESP32S3: Curve = &[
    (0, 69),
    (4, 51),
    (8, 45),
    (16, 33),
    (24, 25),
    (32, 21),
    (48, 16),
    (60, 13),
];

/// Narrowest and widest characterized bandwidth.
pub fn range(curve: Curve) -> (u32, u32) {
    (u32::from(curve[curve.len() - 1].1), u32::from(curve[0].1))
}

/// Interpolate the filter code for `mhz`; 0 selects the widest setting.
pub fn filter_code(curve: Curve, mhz: u32) -> u8 {
    let first = curve[0];
    let last = curve[curve.len() - 1];
    if mhz == 0 || mhz >= u32::from(first.1) {
        return first.0;
    }
    if mhz <= u32::from(last.1) {
        return last.0;
    }
    for pair in curve.windows(2) {
        let (wide, narrow) = (pair[0], pair[1]);
        if mhz >= u32::from(narrow.1) {
            let span = u32::from(wide.1 - narrow.1);
            let steps = u32::from(narrow.0 - wide.0) * (u32::from(wide.1) - mhz);
            return wide.0 + ((steps + span / 2) / span) as u8;
        }
    }
    last.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn endpoints_and_points() {
        assert_eq!(range(ESP32S3), (13, 69));
        assert_eq!(filter_code(ESP32S3, 0), 0);
        assert_eq!(filter_code(ESP32S3, 69), 0);
        assert_eq!(filter_code(ESP32S3, 13), 60);
        assert_eq!(filter_code(ESP32S3, 33), 16);
        assert_eq!(filter_code(ESP32S3, 21), 32);
    }

    #[test]
    fn interpolates_with_rounding() {
        // Between (32,21) and (48,16): 18 MHz -> 32 + round(16*3/5) = 42.
        assert_eq!(filter_code(ESP32S3, 18), 42);
        // Between (0,69) and (4,51): 60 MHz -> round(4*9/18) = 2.
        assert_eq!(filter_code(ESP32S3, 60), 2);
    }
}
