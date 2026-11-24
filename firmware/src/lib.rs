//! ESP-SDR burst receiver for the ESP32-S3 on esp-hal.
//!
//! A Rust port of `main/targets/esp32s3/receiver.c` and the shared burst code
//! from ESPARGOS/esp-sdr. Everything hardware-independent lives in `sdr-core`.
//!
//! - [`phy`]: declarations of the private PHY/ROM functions and registers.
//! - [`hooks`]: linker `--wrap` hooks into the PHY library.
//! - [`receiver`]: tuning, gain and the dump-engine snapshot.
//! - [`serial`]: the line protocol transport over USB Serial/JTAG and UART0.
//! - [`commands`]: the command dispatcher.
#![no_std]

pub mod commands;
pub mod hooks;
pub mod phy;
pub mod receiver;
pub mod serial;
