#![no_std]
#![no_main]

extern crate alloc;

use alloc::string::ToString;
use blocking_network_stack::Stack;
use defmt_rtt as _;
use embedded_io::*;
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    main,
    rng::Rng,
    time::{self, Duration},
    timer::timg::TimerGroup,
};
use esp_println::println;
use esp_radio::wifi::{
    AuthenticationMethodConfig, Config, Interface, Password, Ssid, sta::StationConfig,
};
use smoltcp::{
    iface::{SocketSet, SocketStorage},
    wire::IpAddress,
};

// ← Change these
const SSID: &str = "ssid";
const PASSWORD: &str = "pass";

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    // Optional: enable logging
    // esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // Heap – required for WiFi
    esp_alloc::heap_allocator!(size: 72 * 1024);

    // IMPORTANT: start the RTOS scheduler BEFORE initializing the radio
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);

    // Initialize radio
    //let esp_radio_ctrl = esp_radio::init();

    let mut controller =
        esp_radio::wifi::WifiController::new(peripherals.WIFI, Default::default()).unwrap();

    // Create the station interface
    let mut device = Interface::station();

    // Configure station
    let station_config = Config::Station(
        StationConfig::default()
            .with_ssid(SSID.try_into().unwrap())
            .with_authentication(AuthenticationMethodConfig::Wpa2Personal(
                PASSWORD.try_into().unwrap(),
            )),
    );

    controller.set_config(&station_config).unwrap();

    println!("Waiting for IP address...");

    loop {
        if controller.is_connected() {
            println!("connected")
        }

        // Small delay
        let start = time::Instant::now();
        while start.elapsed() < Duration::from_millis(50) {}
    }
}
// Helper function (same as in the official examples)
fn create_interface(device: &mut impl smoltcp::phy::Device) -> smoltcp::iface::Interface {
    use smoltcp::iface::Config;
    use smoltcp::wire::{EthernetAddress, HardwareAddress};

    let config = Config::new(HardwareAddress::Ethernet(EthernetAddress([
        0x02, 0x00, 0x00, 0x00, 0x00, 0x01,
    ])));
    smoltcp::iface::Interface::new(config, device, smoltcp::time::Instant::ZERO)
}
