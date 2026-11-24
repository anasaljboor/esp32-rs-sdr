//! Line-protocol transport over native USB Serial/JTAG and UART0
//! (`main/common/burst_serial.c`).
//!
//! Each port has its own line parser. A complete line selects that port for
//! all replies and payloads. `BAUD?` and `BAUD <rate>` are handled here.

use esp_hal::Blocking;
use esp_hal::time::{Duration, Instant};
use esp_hal::uart::{Config as UartConfig, Uart};
use esp_hal::usb::usb_serial_jtag::{UsbSerialJtagRx, UsbSerialJtagTx};
use sdr_core::command::Command;

pub const DEFAULT_UART_BAUD: u32 = 2_000_000;

/// USB Serial/JTAG endpoint 1 configuration/status register (ESP32-S3).
/// Bit 0 WR_DONE: commit the TX FIFO as a packet. Bit 1 SERIAL_IN_EP_DATA_FREE:
/// the TX FIFO has space; it stays 0 after WR_DONE until the host reads.
const USB_EP1_CONF: usize = 0x6003_8004;

fn usb_tx_writable() -> bool {
    crate::phy::reg_read(USB_EP1_CONF) & 0b10 != 0
}

fn usb_tx_flush() {
    crate::phy::reg_write(USB_EP1_CONF, crate::phy::reg_read(USB_EP1_CONF) | 1);
}
pub const COMMAND_SIZE: usize = 128;
const USB_PACKET: usize = 64;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Port {
    Usb = 0,
    Uart = 1,
}

impl Port {
    fn other(self) -> Port {
        match self {
            Port::Usb => Port::Uart,
            Port::Uart => Port::Usb,
        }
    }
}

struct Input {
    line: [u8; COMMAND_SIZE],
    used: usize,
    overflow: bool,
    last_byte: Instant,
}

impl Input {
    const fn new() -> Self {
        Self { line: [0; COMMAND_SIZE], used: 0, overflow: false, last_byte: Instant::EPOCH }
    }

    fn clear(&mut self) {
        self.used = 0;
        self.overflow = false;
    }
}

pub enum Poll {
    Idle,
    /// The line did not fit in [`COMMAND_SIZE`].
    TooLong,
    /// A complete command of this length is in the caller's buffer.
    Line(usize),
}

pub struct Serial<'d> {
    usb_rx: UsbSerialJtagRx<'d, Blocking>,
    usb_tx: UsbSerialJtagTx<'d, Blocking>,
    uart: Uart<'d, Blocking>,
    uart_baud: u32,
    active: Port,
    next: Port,
    inputs: [Input; 2],
}

impl<'d> Serial<'d> {
    pub fn new(
        usb_rx: UsbSerialJtagRx<'d, Blocking>,
        usb_tx: UsbSerialJtagTx<'d, Blocking>,
        uart: Uart<'d, Blocking>,
    ) -> Self {
        Self {
            usb_rx,
            usb_tx,
            uart,
            uart_baud: DEFAULT_UART_BAUD,
            active: Port::Usb,
            next: Port::Usb,
            inputs: [Input::new(), Input::new()],
        }
    }

    pub fn port(&self) -> Port {
        self.active
    }

    /// UART baud rate if the active port is the UART, else 0 (`TRANSPORT?`).
    pub fn baud(&self) -> u32 {
        if self.active == Port::Uart { self.uart_baud } else { 0 }
    }

    fn read_byte(&mut self, port: Port) -> Option<u8> {
        match port {
            Port::Usb => self.usb_rx.read_byte().ok(),
            Port::Uart => {
                let mut b = [0u8; 1];
                match self.uart.read_buffered(&mut b) {
                    Ok(1) => Some(b[0]),
                    _ => None,
                }
            }
        }
    }

    /// Poll both ports, starting after the one served last, for one line.
    pub fn poll_line(&mut self, out: &mut [u8; COMMAND_SIZE]) -> Poll {
        for _ in 0..2 {
            let port = self.next;
            self.next = port.other();
            let now = Instant::now();
            let input = &mut self.inputs[port as usize];
            if now - input.last_byte > Duration::from_secs(3) {
                input.clear();
            }
            // Bounded, so an unterminated command cannot starve the other port.
            for _ in 0..COMMAND_SIZE {
                let Some(ch) = self.read_byte(port) else { break };
                let input = &mut self.inputs[port as usize];
                input.last_byte = now;
                match ch {
                    b'\r' => continue,
                    b'\n' => {}
                    _ => {
                        if input.used < COMMAND_SIZE - 1 {
                            input.line[input.used] = ch;
                            input.used += 1;
                        } else {
                            input.overflow = true;
                        }
                        continue;
                    }
                }
                let (used, overflow) = (input.used, input.overflow);
                out[..used].copy_from_slice(&input.line[..used]);
                input.clear();
                if used == 0 && !overflow {
                    continue;
                }
                self.active = port;
                self.next = port.other();
                if overflow {
                    return Poll::TooLong;
                }
                if self.baud_command(&out[..used]) {
                    return Poll::Idle;
                }
                return Poll::Line(used);
            }
        }
        Poll::Idle
    }

    /// Handle `BAUD?` / `BAUD <rate>`. The acknowledgement is sent completely
    /// at the old rate before UART0 changes. The rate lasts until reset.
    fn baud_command(&mut self, line: &[u8]) -> bool {
        let Ok(text) = core::str::from_utf8(line) else { return false };
        let cmd = Command::parse(text, &crate::commands::LIMITS);
        if !matches!(cmd, Command::BaudQuery | Command::Baud(_) | Command::BaudInvalid) {
            return false;
        }
        if self.active != Port::Uart {
            self.send(b"ERR baud_transport\n");
            return true;
        }
        match cmd {
            Command::BaudQuery => {
                let mut l = sdr_core::line::Line::<32>::new();
                let _ = core::fmt::Write::write_fmt(&mut l, format_args!("BAUD {}\n", self.uart_baud));
                self.send(l.as_bytes());
            }
            Command::Baud(baud) => {
                let mut l = sdr_core::line::Line::<32>::new();
                let _ = core::fmt::Write::write_fmt(&mut l, format_args!("OK BAUD {baud}\n"));
                if self.send(l.as_bytes())
                    && self.uart.flush().is_ok()
                    && self.uart.apply_config(&UartConfig::default().with_baudrate(baud)).is_ok()
                {
                    self.uart_baud = baud;
                    while self.read_byte(Port::Uart).is_some() {}
                    self.inputs[Port::Uart as usize].clear();
                }
            }
            _ => {
                self.send(b"ERR baud_args\n");
            }
        }
        true
    }

    fn deadline(&self, size: usize) -> Instant {
        // 8N1 needs ten wire bits per byte; leave two seconds for the host.
        let timeout = match self.active {
            Port::Uart => {
                Duration::from_micros(2_000_000 + size as u64 * 10_000_000 / u64::from(self.uart_baud))
            }
            Port::Usb => Duration::from_secs(3),
        };
        Instant::now() + timeout
    }

    /// Send everything on the active port, or give up at the deadline (for
    /// example when the host stopped reading). Returns `false` on timeout.
    pub fn send(&mut self, data: &[u8]) -> bool {
        let deadline = self.deadline(data.len());
        match self.active {
            Port::Uart => {
                let mut rest = data;
                while !rest.is_empty() {
                    if Instant::now() >= deadline {
                        return false;
                    }
                    if !self.uart.write_ready() {
                        continue;
                    }
                    match self.uart.write(rest) {
                        Ok(n) => rest = &rest[n..],
                        Err(_) => return false,
                    }
                }
                true
            }
            Port::Usb => {
                // Same sequence as ESP-IDF's usb_serial_jtag_ll, as used by the
                // C firmware: wait for FIFO space without touching WR_DONE, fill
                // up to one packet, then signal WR_DONE exactly once. Setting
                // WR_DONE again while the host has not yet read the previous
                // packet lost whole packets in testing.
                let mut rest = data;
                while !rest.is_empty() {
                    if Instant::now() >= deadline {
                        return false;
                    }
                    if !usb_tx_writable() {
                        continue;
                    }
                    let limit = rest.len().min(USB_PACKET);
                    let mut n = 0;
                    while n < limit && self.usb_tx.write_byte_nb(rest[n]).is_ok() {
                        n += 1;
                    }
                    usb_tx_flush();
                    rest = &rest[n..];
                }
                // A transfer ending on a full packet needs one more flush so the
                // host sees the end of the transfer.
                if !data.is_empty() && data.len().is_multiple_of(USB_PACKET) {
                    while !usb_tx_writable() {
                        if Instant::now() >= deadline {
                            return false;
                        }
                    }
                    usb_tx_flush();
                }
                true
            }
        }
    }

    pub fn send_str(&mut self, s: &str) -> bool {
        self.send(s.as_bytes())
    }
}
