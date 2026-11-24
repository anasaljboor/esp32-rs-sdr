//! Command dispatch and the receiver lease (`burst_commands.h`,
//! `burst_serial_serve`, and the S3 `handle_command`).

use core::fmt::Write;

use esp_hal::time::{Duration, Instant};
use sdr_core::bandwidth;
use sdr_core::command::{Command, Format, Limits};
use sdr_core::line::Line;
use sdr_core::pack;
use sdr_core::tuning::{FREQ_MAX_MHZ, FREQ_MIN_MHZ};

use crate::phy::{self, IQ_WORDS};
use crate::receiver::{self, GainMode, Receiver};
use crate::serial::{COMMAND_SIZE, Poll, Port, Serial};

pub const LIMITS: Limits = Limits {
    max_words: IQ_WORDS as u32,
    max_rate: 6,
    bandwidth_mhz: (13, 69),
};

/// Flags for features this port implements. The C S3 firmware also offers
/// GPIO, RING, SPEC*, IQS and DCT, which are not ported yet.
const CAPS: &str =
    "CAPS VERSION UARTBAUD RXLIMITS SERIALLEASE DUALSERIAL TUNEEXT RX40 RX16 LPFANA GAIN HWAGC IQ8\n";
const VERSION_JSON: &str = env!("ESP_SDR_VERSION_JSON");
const LEASE: Duration = Duration::from_secs(5);

pub struct Server<'d> {
    pub rx: Receiver,
    pub serial: Serial<'d>,
    owner: Option<Port>,
    lease_deadline: Instant,
}

impl<'d> Server<'d> {
    pub fn new(rx: Receiver, serial: Serial<'d>) -> Self {
        Self { rx, serial, owner: None, lease_deadline: Instant::EPOCH }
    }

    /// Handle at most one command. Returns `false` when there was nothing to do.
    pub fn step(&mut self) -> bool {
        if Instant::now() >= self.lease_deadline {
            self.owner = None;
        }
        let mut buf = [0u8; COMMAND_SIZE];
        let status = self.serial.poll_line(&mut buf);
        let len = match status {
            Poll::Idle => return false,
            Poll::TooLong => 0,
            Poll::Line(len) => len,
        };
        let port = self.serial.port();
        if self.owner.is_some_and(|owner| owner != port) {
            self.serial.send_str("ERR busy\n");
            return true;
        }
        if matches!(status, Poll::TooLong) {
            self.serial.send_str("ERR command_length\n");
            return true;
        }
        self.owner = Some(port);
        match core::str::from_utf8(&buf[..len]) {
            Ok("RELEASE") => {
                self.serial.send_str("OK\n");
                self.owner = None;
            }
            Ok(line) => self.handle(line),
            Err(_) => {
                self.serial.send_str("ERR command\n");
            }
        }
        // The lease covers the whole binary transaction.
        self.lease_deadline = Instant::now() + LEASE;
        true
    }

    fn reply(&mut self, s: &str) {
        self.serial.send_str(s);
    }

    fn reply_fmt(&mut self, args: core::fmt::Arguments) {
        let mut line = Line::<256>::new();
        if line.write_fmt(args).is_ok() {
            self.serial.send(line.as_bytes());
        }
    }

    fn handle(&mut self, line: &str) {
        match Command::parse(line, &LIMITS) {
            Command::Sync(nonce) => self.reply_fmt(format_args!("SYNC {nonce}\n")),
            Command::Info => self.reply_fmt(format_args!("S3SDR 6 burst {IQ_WORDS}\n")),
            Command::Caps => self.reply(CAPS),
            Command::Version => self.reply_fmt(format_args!("VERSION {VERSION_JSON}\n")),
            Command::Transport => {
                let name = if self.serial.port() == Port::Uart { "UART" } else { "USB" };
                let baud = self.serial.baud();
                self.reply_fmt(format_args!("TRANSPORT {name} {baud}\n"));
            }
            Command::Range => self.reply_fmt(format_args!("RANGE {FREQ_MIN_MHZ} {FREQ_MAX_MHZ} 1\n")),
            Command::Limits => {
                let (lo, hi) = LIMITS.bandwidth_mhz;
                let max = self.rx.gain_max();
                self.reply_fmt(format_args!(
                    "LIMITS {{\"gain\":[0,{max},1],\"bandwidth\":[{lo},{hi},1,0],\
                     \"rates\":[80000000,40000000,16000000],\"bits\":[8,10]}}\n"
                ));
            }
            Command::Freq(mhz) => {
                self.rx.set_frequency(mhz);
                self.reply("OK\n");
            }
            Command::Fofs(khz) => {
                self.rx.set_fofs(khz);
                self.reply("OK\n");
            }
            Command::FofsInvalid => self.reply("ERR args\n"),
            Command::GainQuery => {
                let (mode, code) = match self.rx.gain_mode() {
                    GainMode::Hardware => ("HARDWARE", -1),
                    GainMode::Manual => ("MANUAL", self.rx.gain_code() as i32),
                };
                let (max, forced) = (self.rx.gain_max(), self.rx.gain_forced_flag());
                self.reply_fmt(format_args!("GAIN {mode} {code} 0 {max} {forced}\n"));
            }
            Command::GainHardware => {
                let ok = self.rx.set_gain_hardware();
                self.reply(if ok { "OK\n" } else { "ERR gain\n" });
            }
            Command::GainManual(code) => match self.rx.set_gain_manual(code) {
                None => self.reply("ERR command\n"),
                Some(true) => self.reply("OK\n"),
                Some(false) => self.reply("ERR gain\n"),
            },
            Command::Bandwidth(mhz) => {
                self.rx.set_filter_code(Some(bandwidth::filter_code(bandwidth::ESP32S3, mhz)));
                self.reply("OK\n");
            }
            Command::LpfAuto => {
                self.rx.set_filter_code(None);
                self.reply("OK\n");
            }
            Command::Lpf(code) => {
                self.rx.set_filter_code(Some(code as u8));
                self.reply("OK\n");
            }
            Command::LpfQuery => {
                let current = self.rx.filter_code().map_or(-1, i32::from);
                let [i, q] = self.rx.filter_registers();
                self.reply_fmt(format_args!("LPF {current} {i} {q}\n"));
            }
            Command::Capture { samples, rate, format } => {
                self.capture_and_send(samples as usize, rate, format);
            }
            Command::RxRun { samples, rate, repeats, format } => {
                // Separate captures, with RF gaps during each transfer.
                let ok = (0..repeats).all(|_| self.capture_and_send(samples as usize, rate, format));
                if ok {
                    self.reply("END\n");
                }
            }
            Command::Release => self.reply("OK\n"),
            Command::BaudQuery | Command::Baud(_) | Command::BaudInvalid | Command::Unknown => {
                self.reply("ERR command\n")
            }
        }
    }

    /// Capture, pack in place and send `DATA <n> <crc32> <us>` plus payload.
    fn capture_and_send(&mut self, n: usize, rate: u32, format: Format) -> bool {
        let elapsed = match self.rx.capture(n, rate) {
            Ok(us) => us,
            Err(e) => {
                self.reply(e.reply());
                return false;
            }
        };
        // SAFETY: no capture runs until this function returns.
        let buf = unsafe { receiver::capture_buffer(n) };
        let bytes = pack::pack_in_place(buf, n, format);
        let payload = &buf[..bytes];
        let crc = phy::crc32(payload);
        let mut header = Line::<64>::new();
        let _ = writeln!(header, "DATA {n} {crc:08x} {elapsed}");
        self.serial.send(header.as_bytes()) && self.serial.send(payload)
    }
}
