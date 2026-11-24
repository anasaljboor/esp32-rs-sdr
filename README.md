# esp-sdr-rs

A Rust port of the [ESP-SDR](https://github.com/ESPARGOS/esp-sdr) burst
receiver firmware for the **ESP32-S3**, built on
[esp-hal](https://github.com/esp-rs/esp-hal) and esp-radio instead of ESP-IDF.

It uses the same undocumented RF "dump engine" as the C firmware to copy raw
I/Q samples from the Wi-Fi receiver into SRAM, and speaks the same serial
protocol (`INFO` reports `S3SDR 6 burst 16380`). Clients written for the C
firmware, such as the [browser viewer](https://espargos.net/espsdr/app/),
should work unchanged.

## Layout

- `sdr-core/`: hardware-independent `no_std` code with host unit tests:
  command parsing, IQ8/IQ10 packing, CRC32, LO planning (including the 5/6
  divider), the bandwidth→filter-code table and the gain-code math.
- `firmware/`: the esp-hal application.
  - `memory.x` moves the stack and heap out of `0x3FCB0000..0x3FCE0000`,
    where the dump engine writes. esp-hal's default puts the main stack
    there.
  - `build.rs` adds the `--wrap` linker hooks into the PHY library and
    embeds the `VERSION?` record.
  - `src/phy.rs`: private PHY/ROM functions, registers and offsets.
  - `src/hooks.rs`: the `--wrap` hooks (gain-code capture, DC-measurement
    counter, retune during recalibration).
  - `src/receiver.rs`: tuning, recalibration, verified manual gain, filter,
    and the snapshot itself.
  - `src/serial.rs`, `src/commands.rs`: USB Serial/JTAG + UART0 transport,
    receiver lease, command dispatch.
- `tools/smoke_test.py`: drives a connected board through the protocol and
  checks every CRC. Standard library only. Works against the C firmware too.

## Build, flash, test

Install the Xtensa toolchain once with `espup install`, then:

```sh
cd sdr-core && cargo +stable test          # host unit tests
cd ../firmware && source ~/export-esp.sh
cargo build --release
espflash flash --chip esp32s3 target/xtensa-esp32s3-none-elf/release/esp-sdr-rs
python3 ../tools/smoke_test.py /dev/cu.usbmodem1101
```

Don't use `cargo run`: its `--monitor` would hold the port. Wait a few
seconds after flashing so the radio calibration can finish.

## Ported so far

| Feature | C firmware (S3) | This port |
| --- | --- | --- |
| `SYNC INFO CAPS VERSION? TRANSPORT? RELEASE RANGE? LIMITS?` | ✅ | ✅ |
| `CAP` / `CAP16` / `CAP20`, `RXRUN` at 80/40/16 MS/s | ✅ | ✅ |
| `FREQ` (incl. 5/6 LO divider), `FOFS`, receive-only DC recalibration | ✅ | ✅ |
| `GAIN HARDWARE/MANUAL/?` with PBUS readback verification | ✅ | ✅ |
| `BANDWIDTH`, `LPF`, `LPF?` | ✅ | ✅ |
| USB Serial/JTAG + UART0 at 2 Mbaud, `BAUD`, lease | ✅ | ✅ |
| `GPIO` outputs | ✅ | ❌ not yet |
| Continuous capture: `RING`, `RINGCAP`, `SPEC*`, `IQS`, `DUAL`, `DC` | ✅ | ❌ not yet |

`CAPS` only lists what is implemented, so clients hide the rest.

## Differences from the C firmware

- Wi-Fi comes up in station mode (idle, never connecting) instead of
  `WIFI_MODE_NULL`, because that's how esp-radio's `WifiController` starts.
  Promiscuous mode, channel 1 and no power saving match the C setup.
- The snapshot runs inside a critical section: interrupts are off for at most
  about 1 ms (16,380 samples at 16 MS/s).
- There's no PHY mutex: esp-phy has no background calibration task, so
  nothing else calibrates concurrently.
- `VERSION?` reports the profile `esp32s3-rust`. The viewer then finds no
  matching official build and doesn't prompt you to "update" to the C
  firmware.

## Caveats

The PHY functions, `phy_param` offsets and the `g_phyFuns` slot come from
the PHY library the C project pinned. This port links the `libphy.a`
shipped in `esp-wifi-sys-esp32s3 0.3.0` instead. Every symbol resolves and
the hooks are wired, but the private offsets are only proven on hardware.
`tools/smoke_test.py` is the check to run after any esp-radio update.

## License

GPL-3.0-or-later, like ESP-SDR. The capture mechanism and register sequences
come from ESP-SDR by Florian Euchner, Zoltan Doczi and contributors.
