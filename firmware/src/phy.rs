//! Private PHY-library and ROM interfaces used by the receiver.
//!
//! None of this is documented by Espressif. The functions live in `libphy.a`
//! (linked by esp-radio through `esp-wifi-sys-esp32s3`) or in the S3 mask ROM
//! (`esp32s3.rom.ld`); signatures and offsets come from the C firmware, which
//! qualified them against the PHY library pinned in its ESP-IDF version.
//! A different PHY library version may move `phy_param` fields or the
//! `g_phyFuns` slot, so re-check them when esp-radio updates its blobs.
#![allow(non_upper_case_globals)]

use core::ffi::c_void;

/// Dump-engine registers and memory (docs/how-it-works.md, per-chip table).
pub const IQ_BUFFER: *mut u32 = 0x3fcd_0000 as *mut u32;
pub const IQ_WORDS: usize = 16380;
pub const SRAM_OWNER_REG: usize = 0x600c_101c;
pub const DUMP_CTRL_REG: usize = 0x6003_3d5c;
pub const DUMP_LANES_REG: usize = 0x6003_3d90;
/// AGC/forced gain control. Bits 8..14: calibrated maximum index.
/// Bit 23 and up: forced-gain flag and index.
pub const GAIN_REG: usize = 0x6001_c02c;
/// PBUS registers that show the analog gain that actually latched.
pub const PBUS_RF_GAIN_REG: usize = 0x6000_6118;
pub const PBUS_BB_GAIN_REG: usize = 0x6000_611c;

/// Every capture word is pre-filled with this, to detect unwritten words.
pub const MARKER: u32 = 0xa5a0_055a;

/// Analog baseband filter: I2C block 0x67, host 0, registers 4 and 5, bits 0..5.
pub const FILTER_BLOCK: u32 = 0x67;
/// 5/6 LO divider select: I2C block 0x65, host 1, register 0, bit 4.
pub const LO_BLOCK: u32 = 0x65;
pub const LO_HOST: u32 = 1;
pub const LO_REG: u32 = 0;
pub const LO_MASK: u32 = 0x10;

/// `g_phyFuns` slot holding the gain-memory writer (rx_gain_verify.c).
pub const GAIN_WRITE_SLOT: usize = 11;
/// `phy_param` word whose bit 0x200 marks the RX DC calibration valid.
pub const PHY_PARAM_DC_FLAGS: usize = 288;

unsafe extern "C" {
    // libphy.a
    pub fn set_chanfreq(mhz: u32, bandwidth: u32);
    pub fn set_rf_freq_offset(mode: u32, mhz: u32, offset_khz: i32);
    pub fn stop_tx_tone(enable: u32);
    /// Wrapped: calls from our code reach `hooks::__wrap_set_rx_gain_table`.
    pub fn set_rx_gain_table(mhz: u32, skip: u32);
    pub static mut phy_param: [u8; 0];
    pub static mut g_phyFuns: *mut *mut c_void;

    // S3 mask ROM
    pub fn rom_pbus_workmode();
    pub fn rom_pbus_xpd_rx_on(enable: u32);
    pub fn rom_pbus_xpd_tx_off();
    pub fn rom_set_rxclk_en(enable: u32);
    pub fn rom_chip_i2c_readReg(block: u32, host: u32, reg: u32) -> u32;
    pub fn rom_chip_i2c_writeReg(block: u32, host: u32, reg: u32, value: u32);
    pub fn rom_rx_gain_force(manual: u32, index: u32);
    pub fn esp_rom_crc32_le(crc: u32, buf: *const u8, len: u32) -> u32;
    pub fn esp_rom_delay_us(us: u32);
}

#[inline(always)]
pub fn reg_read(addr: usize) -> u32 {
    // SAFETY: callers pass fixed, always-mapped peripheral register addresses.
    unsafe { core::ptr::read_volatile(addr as *const u32) }
}

#[inline(always)]
pub fn reg_write(addr: usize, value: u32) {
    // SAFETY: as for `reg_read`.
    unsafe { core::ptr::write_volatile(addr as *mut u32, value) }
}

pub fn i2c_read(block: u32, host: u32, reg: u32) -> u32 {
    unsafe { rom_chip_i2c_readReg(block, host, reg) }
}

pub fn i2c_write(block: u32, host: u32, reg: u32, value: u32) {
    unsafe { rom_chip_i2c_writeReg(block, host, reg, value) }
}

pub fn delay_us(us: u32) {
    unsafe { esp_rom_delay_us(us) }
}

pub fn crc32(data: &[u8]) -> u32 {
    unsafe { esp_rom_crc32_le(0, data.as_ptr(), data.len() as u32) }
}

/// Select (`true`) or release the 6/5 LO divider. Returns whether it changed.
pub fn lo_select(alternate: bool) -> bool {
    let old = i2c_read(LO_BLOCK, LO_HOST, LO_REG);
    let value = (old & !LO_MASK) | if alternate { LO_MASK } else { 0 };
    if value != old {
        i2c_write(LO_BLOCK, LO_HOST, LO_REG, value);
    }
    value != old
}

/// Pointer to a 32-bit field inside the PHY's private `phy_param` block.
pub fn phy_param_word(offset: usize) -> *mut u32 {
    unsafe { (&raw mut phy_param).cast::<u8>().add(offset).cast::<u32>() }
}
