//! ESP-SDR burst receiver firmware for the ESP32-S3.
#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::uart::{Config as UartConfig, Uart};
use esp_hal::usb::usb_serial_jtag::UsbSerialJtag;
use esp_radio::wifi::{ControllerConfig, SecondaryChannel, WifiController};
use esp_sdr_rs::commands::Server;
use esp_sdr_rs::receiver::Receiver;
use esp_sdr_rs::serial::{DEFAULT_UART_BAUD, Serial};

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "main owns the drivers and the server for the whole program"
)]
#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
    // No logger: the USB Serial/JTAG port carries the binary protocol.
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));

    // memory.x leaves 0x3FCB0000..0x3FCE0000 to the RF dump engine, so the
    // heap is the reclaimed bootloader area above it plus a block in DRAM.
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 0xD710);
    esp_alloc::heap_allocator!(size: 40 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);

    // Wi-Fi brings up and calibrates the radio. After that the modem is
    // ignored: promiscuous mode on channel 1, no power saving, no traffic.
    let config = ControllerConfig::default().with_static_rx_buf_num(4);
    let mut wifi = WifiController::new(peripherals.WIFI, config).expect("Wi-Fi init failed");
    let sniffer = wifi.sniffer();
    sniffer.set_promiscuous_mode(true).expect("promiscuous mode failed");
    wifi.set_channel(1, SecondaryChannel::None).expect("set channel failed");

    let mut rx = Receiver::new();
    rx.prepare_rx();

    let (usb_rx, usb_tx) = UsbSerialJtag::new(peripherals.USB_DEVICE).split();
    let uart = Uart::new(peripherals.UART0, UartConfig::default().with_baudrate(DEFAULT_UART_BAUD))
        .expect("UART0 init failed")
        .with_tx(peripherals.GPIO43)
        .with_rx(peripherals.GPIO44);

    let mut server = Server::new(rx, Serial::new(usb_rx, usb_tx, uart));
    loop {
        if !server.step() {
            Timer::after(Duration::from_millis(1)).await;
        }
    }
}
