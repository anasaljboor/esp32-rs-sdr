//! Receiver control and dump-engine snapshots for the ESP32-S3.
//!
//! Follows `main/targets/esp32s3/receiver.c`, `burst_gain.h`,
//! `rx_gain_verify.c` and `rx_recalibration.c` (S3 paths) from the C firmware.

use core::sync::atomic::Ordering;

use esp_hal::time::{Duration, Instant};
use sdr_core::gain;
use sdr_core::tuning::{is_wifi_channel, lo_plan};

use crate::hooks::{self, DC_GENERATION, MEASUREMENT_MHZ};
use crate::phy::{self, reg_read, reg_write};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GainMode {
    Hardware,
    Manual,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CaptureError {
    /// Rate code without a native S3 dump clock.
    Rate,
    /// A manual gain was requested but did not latch.
    Gain,
    /// The engine did not finish or left words unwritten.
    Timeout,
}

impl CaptureError {
    pub fn reply(self) -> &'static str {
        match self {
            CaptureError::Rate => "ERR rate\n",
            CaptureError::Gain => "ERR gain\n",
            CaptureError::Timeout => "ERR capture_timeout\n",
        }
    }
}

/// Sample rate for each accepted rate code. Bit 15 of the control register
/// halves 80 to 40 MS/s; bit 16 selects the 16 MS/s path (verified by the C
/// project with a B210).
fn rate_bits(rate: u32) -> Option<u32> {
    match rate {
        0 => Some(0),
        1 => Some(1 << 15),
        6 => Some(1 << 16),
        _ => None,
    }
}

pub struct Receiver {
    frequency_mhz: u32,
    fofs_khz: i32,
    rx_ready: bool,
    /// `None` keeps the PHY-calibrated automatic filter.
    filter_code: Option<u8>,
    filter_saved: [u32; 2],
    gain_mode: GainMode,
    gain_code: u32,
    gain_applied: bool,
    calibrated_mhz: u32,
    calibrated: bool,
    calibrated_generation: u32,
}

impl Default for Receiver {
    fn default() -> Self {
        Self::new()
    }
}

impl Receiver {
    pub const fn new() -> Self {
        Self {
            frequency_mhz: 2412,
            fofs_khz: 0,
            rx_ready: false,
            filter_code: None,
            filter_saved: [0; 2],
            gain_mode: GainMode::Hardware,
            gain_code: 40,
            gain_applied: true,
            calibrated_mhz: 0,
            calibrated: false,
            calibrated_generation: 0,
        }
    }

    pub fn frequency_mhz(&self) -> u32 {
        self.frequency_mhz
    }

    pub fn gain_mode(&self) -> GainMode {
        self.gain_mode
    }

    pub fn gain_code(&self) -> u32 {
        self.gain_code
    }

    pub fn filter_code(&self) -> Option<u8> {
        self.filter_code
    }

    pub fn set_filter_code(&mut self, code: Option<u8>) {
        self.filter_code = code;
    }

    /// Highest calibrated manual gain index (0 if the table is not calibrated).
    pub fn gain_max(&self) -> u32 {
        gain::s3_gain_max((reg_read(phy::GAIN_REG) >> 8) & 127)
    }

    /// Hardware flag "gain is forced" (`GAIN?` last field).
    pub fn gain_forced_flag(&self) -> u32 {
        (reg_read(phy::GAIN_REG) >> 23) & 1
    }

    // ---- calibration -------------------------------------------------------

    fn recalibration_stale(&self) -> bool {
        !self.calibrated || DC_GENERATION.load(Ordering::Acquire) != self.calibrated_generation
    }

    /// Measure fresh RX DC at `mhz` and rebuild the gain tables, keeping the
    /// IQ correction from startup. Never runs the transmitting IQ step.
    fn recalibrate(&mut self, mhz: u32) {
        MEASUREMENT_MHZ.store(mhz, Ordering::Release);
        // A previous manual gain must not override the measurement's sweep.
        reg_write(phy::GAIN_REG, reg_read(phy::GAIN_REG) & !(1 << 23));
        hooks::measurement_tune(mhz);
        let flags = phy::phy_param_word(phy::PHY_PARAM_DC_FLAGS);
        // SAFETY: phy_param is the PHY's static parameter block; this field is
        // the S3 DC-valid flag word (bit 0x200), see phy.rs.
        unsafe {
            flags.write_volatile(flags.read_volatile() & !0x200);
            phy::set_rx_gain_table(lo_plan(mhz).mhz, 0);
        }
        MEASUREMENT_MHZ.store(0, Ordering::Release);
        self.calibrated_generation = DC_GENERATION.load(Ordering::Acquire);
        self.calibrated = true;
    }

    // ---- tuning ------------------------------------------------------------

    fn tune(&mut self, mhz: u32) {
        if self.calibrated_mhz != mhz || self.recalibration_stale() {
            self.recalibrate(mhz);
            self.calibrated_mhz = mhz;
        }
        let plan = lo_plan(mhz);
        // A PLL offset also requires direct tuning on Wi-Fi channel frequencies.
        let channel = self.fofs_khz == 0 && is_wifi_channel(mhz);
        phy::lo_select(false);
        unsafe {
            phy::set_chanfreq(if channel { mhz } else { 2412 }, 0);
            if !channel {
                phy::set_rf_freq_offset(0, plan.mhz, plan.offset_khz + self.fofs_khz);
            }
        }
    }

    pub fn set_frequency(&mut self, mhz: u32) {
        self.frequency_mhz = mhz;
        self.rx_ready = false;
        self.prepare_rx();
    }

    pub fn set_fofs(&mut self, khz: i32) {
        self.fofs_khz = khz;
        self.rx_ready = false;
        self.prepare_rx();
    }

    /// Bring the receiver into dump-ready state at the current frequency.
    pub fn prepare_rx(&mut self) {
        if self.rx_ready && !self.recalibration_stale() {
            if self.gain_mode == GainMode::Manual {
                self.gain_applied = verified_check(self.gain_code);
            }
            return;
        }
        let mhz = self.frequency_mhz;
        self.tune(mhz);
        unsafe {
            phy::stop_tx_tone(1);
            phy::rom_pbus_workmode();
            phy::rom_pbus_xpd_tx_off();
            phy::rom_pbus_xpd_rx_on(1);
            phy::rom_set_rxclk_en(1);
        }
        phy::lo_select(lo_plan(mhz).alternate);
        phy::delay_us(3000);
        self.gain_apply();
        self.rx_ready = true;
    }

    // ---- gain --------------------------------------------------------------

    fn gain_apply(&mut self) {
        let max = self.gain_max();
        if self.gain_code > max {
            self.gain_code = max;
        }
        self.gain_applied = verified_apply(self.gain_mode == GainMode::Manual, self.gain_code);
    }

    /// Changing gain needs no channel/PBUS reconfiguration unless the
    /// calibration went stale.
    fn gain_reconfigure(&mut self) {
        if !self.rx_ready || self.recalibration_stale() {
            self.prepare_rx();
        } else {
            self.gain_apply();
        }
    }

    pub fn set_gain_hardware(&mut self) -> bool {
        self.gain_mode = GainMode::Hardware;
        self.gain_reconfigure();
        self.gain_applied
    }

    /// Returns `None` if `code` is outside `0..=gain_max()`, else whether the
    /// gain latched.
    pub fn set_gain_manual(&mut self, code: u32) -> Option<bool> {
        if code > self.gain_max() {
            return None;
        }
        self.gain_mode = GainMode::Manual;
        self.gain_code = code;
        self.gain_reconfigure();
        Some(self.gain_applied)
    }

    // ---- filter ------------------------------------------------------------

    fn filter_apply(&mut self) {
        for j in 0..2 {
            self.filter_saved[j] = phy::i2c_read(phy::FILTER_BLOCK, 0, 4 + j as u32);
            if let Some(code) = self.filter_code {
                let value = (self.filter_saved[j] & !63) | u32::from(code);
                phy::i2c_write(phy::FILTER_BLOCK, 0, 4 + j as u32, value);
            }
        }
    }

    fn filter_restore(&mut self) {
        if self.filter_code.is_some() {
            for j in 0..2 {
                phy::i2c_write(phy::FILTER_BLOCK, 0, 4 + j as u32, self.filter_saved[j]);
            }
        }
    }

    /// Current filter codes of the I and Q paths (`LPF?`).
    pub fn filter_registers(&self) -> [u32; 2] {
        [
            phy::i2c_read(phy::FILTER_BLOCK, 0, 4) & 63,
            phy::i2c_read(phy::FILTER_BLOCK, 0, 5) & 63,
        ]
    }

    // ---- capture -----------------------------------------------------------

    /// Record `n` samples at `rate` into the dump buffer. Returns the capture
    /// time in microseconds; read the samples with [`capture_buffer`].
    pub fn capture(&mut self, n: usize, rate: u32) -> Result<u32, CaptureError> {
        let rate_bits = rate_bits(rate).ok_or(CaptureError::Rate)?;
        assert!((1..=phy::IQ_WORDS).contains(&n));
        self.prepare_rx();
        if !self.gain_applied {
            return Err(CaptureError::Gain);
        }

        // SAFETY: memory.x keeps 0x3FCB0000..0x3FCE0000 out of every linker
        // section and heap, so nothing else lives in the dump buffer.
        unsafe {
            for j in 0..n {
                phy::IQ_BUFFER.add(j).write_volatile(phy::MARKER);
            }
        }

        self.filter_apply();
        let (done, elapsed) = critical_section::with(|_| {
            let owner = reg_read(phy::SRAM_OWNER_REG);
            let start = Instant::now();
            reg_write(phy::DUMP_CTRL_REG, 0);
            // Lanes 0..3 select the receiver's I and Q outputs.
            reg_write(phy::DUMP_LANES_REG, (1 << 6) | (2 << 12) | (3 << 18));
            // Give the 0x3FCD0000 bank to the dump engine.
            reg_write(phy::SRAM_OWNER_REG, (owner & !15) | 4);
            let ctrl = 0x8000_0000 | rate_bits | n as u32;
            reg_write(phy::DUMP_CTRL_REG, ctrl);
            reg_write(phy::DUMP_CTRL_REG, ctrl | (1 << 19));
            reg_write(phy::DUMP_CTRL_REG, ctrl);
            let deadline = Duration::from_millis(20);
            while reg_read(phy::DUMP_CTRL_REG) & (1 << 18) == 0 && start.elapsed() < deadline {}
            let done = reg_read(phy::DUMP_CTRL_REG) & (1 << 18) != 0;
            let elapsed = start.elapsed().as_micros() as u32;
            reg_write(phy::DUMP_CTRL_REG, 0);
            reg_write(phy::SRAM_OWNER_REG, owner);
            (done, elapsed)
        });
        self.filter_restore();

        if !done {
            return Err(CaptureError::Timeout);
        }
        // SAFETY: the bank is back with the CPU; same region as above.
        let unwritten =
            unsafe { (0..n).any(|j| phy::IQ_BUFFER.add(j).read_volatile() == phy::MARKER) };
        if unwritten {
            return Err(CaptureError::Timeout);
        }
        Ok(elapsed)
    }
}

/// The first `n` captured words as bytes, for packing and sending.
///
/// # Safety
/// No capture may run while the returned slice is alive, and `n` must not
/// exceed [`phy::IQ_WORDS`].
pub unsafe fn capture_buffer(n: usize) -> &'static mut [u8] {
    debug_assert!(n <= phy::IQ_WORDS);
    unsafe { core::slice::from_raw_parts_mut(phy::IQ_BUFFER.cast::<u8>(), 4 * n) }
}

// ---- verified manual gain (rx_gain_verify.c) --------------------------------

fn gain_set(manual: bool, index: u32) {
    unsafe { phy::rom_rx_gain_force(u32::from(manual), index) };
}

/// The gain-force signal crosses into the receiver clock domain. Hold it low,
/// then high, with settling time, instead of two back-to-back writes.
fn gain_force_pulse(index: u32) {
    gain_set(false, index);
    phy::delay_us(2);
    gain_set(true, index);
    phy::delay_us(2);
}

fn pbus_matches(code: u16) -> bool {
    gain::pbus_matches(code, reg_read(phy::PBUS_RF_GAIN_REG), reg_read(phy::PBUS_BB_GAIN_REG))
}

/// Apply AGC (`manual == false`) or force `index`, and check that the analog
/// PBUS settings really changed. The forced-index field alone is not proof:
/// it is copied into dump samples even when the gain did not latch.
fn verified_apply(manual: bool, index: u32) -> bool {
    critical_section::with(|_| {
        if !manual {
            gain_set(false, index);
            return true;
        }
        let ok = match hooks::gain_code(index) {
            Some(code) if hooks::gain_hook_installed() => (0..4).any(|_| {
                gain_force_pulse(index);
                pbus_matches(code)
            }),
            _ => false,
        };
        if !ok {
            gain_set(false, index);
        }
        ok
    })
}

fn verified_check(index: u32) -> bool {
    let ok = critical_section::with(|_| {
        hooks::gain_code(index).is_some_and(|code| {
            gain::forced_field_matches(reg_read(phy::GAIN_REG), index) && pbus_matches(code)
        })
    });
    ok || verified_apply(true, index)
}
