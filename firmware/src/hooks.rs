//! Linker `--wrap` hooks into the PHY library (build.rs adds the flags).
//!
//! - `set_rx_gain_table`: on every gain-table rebuild, install a hook in the
//!   PHY's gain-memory writer to record the gain code of each slot. Manual gain
//!   is later verified against these codes (`rx_gain_verify.c`).
//! - `set_rx_gain_cal_dc`: count DC measurements, so a calibration replaced by
//!   the PHY is detected (`rx_calibration_state.c`).
//! - `chip_v7_set_chan_ana`: while we recalibrate, the PHY tunes its fixed
//!   reference channel. Retune to the requested frequency right after, so the
//!   DC is measured where we will receive (`rx_recalibration.c`).

use core::sync::atomic::{AtomicBool, AtomicU16, AtomicU32, AtomicUsize, Ordering};

use sdr_core::gain::{S3_GAIN_SLOTS, code_from_gain_word};
use sdr_core::tuning::lo_plan;

use crate::phy;

type GainWrite = unsafe extern "C" fn(u32, u32, u32);

static GAIN_WRITE: AtomicUsize = AtomicUsize::new(0);
static CODES: [AtomicU16; S3_GAIN_SLOTS] = [const { AtomicU16::new(0) }; S3_GAIN_SLOTS];
static VALID: [AtomicBool; S3_GAIN_SLOTS] = [const { AtomicBool::new(false) }; S3_GAIN_SLOTS];

/// Number of RX DC measurements the PHY has made.
pub static DC_GENERATION: AtomicU32 = AtomicU32::new(0);
/// Non-zero while `receiver::recalibrate` runs: the LO to measure at.
pub static MEASUREMENT_MHZ: AtomicU32 = AtomicU32::new(0);

/// The gain code recorded for `index`, if the PHY has written that slot.
pub fn gain_code(index: u32) -> Option<u16> {
    let i = index as usize;
    (i < S3_GAIN_SLOTS && VALID[i].load(Ordering::Acquire)).then(|| CODES[i].load(Ordering::Relaxed))
}

/// True once the gain-memory hook is installed.
pub fn gain_hook_installed() -> bool {
    GAIN_WRITE.load(Ordering::Acquire) != 0
}

#[esp_hal::ram]
unsafe extern "C" fn capture_gain(low: u32, high: u32, index: u32) {
    if (index as usize) < S3_GAIN_SLOTS {
        CODES[index as usize].store(code_from_gain_word(low), Ordering::Relaxed);
        VALID[index as usize].store(true, Ordering::Release);
    }
    let original = GAIN_WRITE.load(Ordering::Acquire);
    if original != 0 {
        // SAFETY: `original` is the PHY's own writer, saved from `g_phyFuns`.
        unsafe { core::mem::transmute::<usize, GainWrite>(original)(low, high, index) }
    }
}

unsafe extern "C" {
    fn __real_set_rx_gain_table(mhz: u32, skip: u32);
    fn __real_chip_v7_set_chan_ana(channel: u32);
    fn __real_set_rx_gain_cal_dc(
        a: usize, b: usize, c: usize, d: usize, e: usize,
        f: usize, g: usize, h: usize, i: usize, j: usize,
    );
}

/// # Safety
/// Called only by the PHY library and our receiver, with the PHY's arguments.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn __wrap_set_rx_gain_table(mhz: u32, skip: u32) {
    unsafe {
        let table = phy::g_phyFuns;
        if !table.is_null() {
            let slot = table.add(phy::GAIN_WRITE_SLOT);
            let hook = capture_gain as GainWrite as *mut core::ffi::c_void;
            let current = *slot;
            if current != hook && !current.is_null() {
                GAIN_WRITE.store(current as usize, Ordering::Release);
                *slot = hook;
            }
        }
        __real_set_rx_gain_table(mhz, skip);
    }
}

/// Private ten-word calling convention from the pinned PHY; forward unchanged.
///
/// # Safety
/// Called only by the PHY library, with the PHY's arguments.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn __wrap_set_rx_gain_cal_dc(
    a: usize, b: usize, c: usize, d: usize, e: usize,
    f: usize, g: usize, h: usize, i: usize, j: usize,
) {
    unsafe { __real_set_rx_gain_cal_dc(a, b, c, d, e, f, g, h, i, j) };
    DC_GENERATION.fetch_add(1, Ordering::AcqRel);
}

/// # Safety
/// Called only by the PHY library, with the PHY's arguments.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn __wrap_chip_v7_set_chan_ana(channel: u32) {
    unsafe { __real_chip_v7_set_chan_ana(channel) };
    let mhz = MEASUREMENT_MHZ.load(Ordering::Acquire);
    if mhz != 0 {
        measurement_tune(mhz);
    }
}

/// Tune the PLL straight to `mhz` (through the 5/6 divider when needed).
pub fn measurement_tune(mhz: u32) {
    let plan = lo_plan(mhz);
    unsafe { phy::set_rf_freq_offset(0, plan.mhz, plan.offset_khz) };
    phy::lo_select(plan.alternate);
}
