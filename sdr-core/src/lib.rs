//! Hardware-independent parts of the ESP-SDR burst protocol.
//!
//! Everything here is plain `no_std` Rust with no register access, so it can be
//! unit-tested on the host with `cargo test`. The firmware crate owns all
//! hardware access and calls into this crate for parsing, packing and math.
//!
//! The behaviour follows the C firmware in `ESPARGOS/esp-sdr`
//! (`main/common/burst_*.h`, `rx_lo.h`, `rx_bandwidth.h`, `docs/protocol.md`).
#![no_std]

pub mod bandwidth;
pub mod command;
pub mod crc;
pub mod gain;
pub mod line;
pub mod pack;
pub mod parse;
pub mod tuning;
