//! Tuning math (`main/common/rx_tuning.h`, `rx_lo.h`).

/// Software attempt limits, independent of where the PLL actually locks.
pub const FREQ_MIN_MHZ: u32 = 100;
pub const FREQ_MAX_MHZ: u32 = 6000;

pub const fn frequency_valid(mhz: u32) -> bool {
    mhz >= FREQ_MIN_MHZ && mhz <= FREQ_MAX_MHZ
}

/// How to reach a requested receive LO.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LoPlan {
    /// Whole-MHz PLL frequency to program.
    pub mhz: u32,
    /// Remaining offset in kHz.
    pub offset_khz: i32,
    /// Divide the PLL by 6/5 to reach 1842..2209 MHz.
    pub alternate: bool,
}

/// 1842..2209 MHz maps into the 2210..2651 MHz PLL range through the 5/6 LO
/// divider; everything else is a direct PLL attempt.
pub const fn lo_plan(mhz: u32) -> LoPlan {
    let alternate = mhz >= 1842 && mhz < 2210;
    let khz = mhz * if alternate { 1200 } else { 1000 };
    LoPlan {
        mhz: khz / 1000,
        offset_khz: (khz % 1000) as i32,
        alternate,
    }
}

/// True for 2.4 GHz Wi-Fi channel centres, which the PHY can tune directly.
pub const fn is_wifi_channel(mhz: u32) -> bool {
    (mhz >= 2412 && mhz <= 2472 && (mhz - 2412) % 5 == 0) || mhz == 2484
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn direct_and_divided_plans() {
        assert_eq!(lo_plan(2442), LoPlan { mhz: 2442, offset_khz: 0, alternate: false });
        assert_eq!(lo_plan(1842), LoPlan { mhz: 2210, offset_khz: 400, alternate: true });
        assert_eq!(lo_plan(2209), LoPlan { mhz: 2650, offset_khz: 800, alternate: true });
        assert_eq!(lo_plan(2210), LoPlan { mhz: 2210, offset_khz: 0, alternate: false });
        assert_eq!(lo_plan(1841).alternate, false);
    }

    #[test]
    fn wifi_channels() {
        assert!(is_wifi_channel(2412));
        assert!(is_wifi_channel(2472));
        assert!(is_wifi_channel(2484));
        assert!(!is_wifi_channel(2413));
        assert!(!is_wifi_channel(2477));
    }
}
