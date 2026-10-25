//! ESP32 -> Wi-Fi (station) -> DHCP -> TCP client talking to a PC.
//!
//! Based on the official esp-radio 1.0.0-beta.1 `embassy_dhcp` example.

#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_net::{Ipv4Address, Runner, StackResources, tcp::TcpSocket};
use embassy_time::{Duration, Timer};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{clock::CpuClock, ram, rng::Rng, timer::timg::TimerGroup};
use esp_println::println;
use esp_radio::wifi::{
    AuthenticationMethodConfig, Config, ControllerConfig, Interface, WifiController,
    scan::ScanConfig, sta::StationConfig,
};

esp_bootloader_esp_idf::esp_app_desc!();

// ← Change these
const SSID: &str = env!("ssid");
const PASSWORD: &str = env!("password");

// The PC running pc_server.py (find it with `ipconfig` / `ip a`).
const PC_IP: Ipv4Address = Ipv4Address::new(192, 168, 1, 100);
const PC_PORT: u16 = 8080;

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

#[esp_hal::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // Heap – required by the Wi-Fi driver.
    //esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024);
    esp_alloc::heap_allocator!(size: 72 * 1024);
    //esp_alloc::heap_allocator!(size: 36 * 1024);

    // The RTOS scheduler (and embassy time driver) must be running before Wi-Fi starts.
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);

    // Wi-Fi: station config is passed in when the controller is created.
    let station_config = Config::Station(
        StationConfig::default()
            .with_ssid(SSID.try_into().unwrap())
            .with_bssid([234, 61, 103, 37, 4, 100]) // stronger AP from your log
            .with_authentication(AuthenticationMethodConfig::Wpa2Personal(
                PASSWORD.try_into().unwrap(),
            )),
    );

    println!("Starting wifi");
    let wifi_interface = Interface::station();
    let mut controller = WifiController::new(
        peripherals.WIFI,
        ControllerConfig::default()
            .with_initial_config(station_config)
            .with_sta_disconnected_pm(false),
    )
    .unwrap();

    // Network stack (embassy-net) on top of the Wi-Fi interface, DHCP for the IP.
    let rng = Rng::new();
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;

    let (stack, runner) = embassy_net::new(
        wifi_interface,
        embassy_net::Config::dhcpv4(Default::default()),
        mk_static!(StackResources<3>, StackResources::<3>::new()),
        seed,
    );

    let scan_config = ScanConfig::default().with_max(10);
    let result = controller.scan_async(&scan_config).await.unwrap();
    for ap in result {
        println!("{:?}", ap);
    }

    spawner.spawn(connection(controller).unwrap());
    spawner.spawn(net_task(runner).unwrap());

    println!("Waiting for IP address...");
    stack.wait_config_up().await;
    if let Some(cfg) = stack.config_v4() {
        println!("Got IP: {}", cfg.address);
    }

    // Talk to the PC.
    let mut rx_buffer = [0u8; 1024];
    let mut tx_buffer = [0u8; 1024];

    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(Duration::from_secs(10)));

        println!("Connecting to {}:{} ...", PC_IP, PC_PORT);
        if let Err(e) = socket.connect((PC_IP, PC_PORT)).await {
            println!("TCP connect failed: {:?}", e);
            Timer::after(Duration::from_secs(2)).await;
            continue;
        }
        println!("Connected to PC");

        let mut reply = [0u8; 128];
        loop {
            if let Err(e) = send_all(&mut socket, b"hello from esp32\n").await {
                println!("Write error: {:?}", e);
                break;
            }
            match socket.read(&mut reply).await {
                Ok(0) => {
                    println!("PC closed the connection");
                    break;
                }
                Ok(n) => println!(
                    "PC says: {}",
                    core::str::from_utf8(&reply[..n]).unwrap_or("<non-utf8>")
                ),
                Err(e) => {
                    println!("Read error: {:?}", e);
                    break;
                }
            }
            Timer::after(Duration::from_secs(1)).await;
        }

        socket.abort();
        Timer::after(Duration::from_secs(2)).await;
    }
}

async fn send_all(
    socket: &mut TcpSocket<'_>,
    mut data: &[u8],
) -> Result<(), embassy_net::tcp::Error> {
    while !data.is_empty() {
        let n = socket.write(data).await?;
        data = &data[n..];
    }
    socket.flush().await
}

/// Keeps the Wi-Fi association alive (reconnects after drops).
#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) {
    println!("start connection task");
    loop {
        println!("About to connect...");
        match controller.connect_async().await {
            Ok(info) => {
                println!("Wifi connected: {:?}", info);
                // Wait until we're no longer connected, then loop to reconnect.
                let info = controller.wait_for_disconnect_async().await.ok();
                println!("Disconnected: {:?}", info);
            }
            Err(e) => {
                println!("Failed to connect to wifi: {:?}", e);
                let _ = controller.disconnect_async().await;
            }
        }
        Timer::after(Duration::from_millis(5000)).await;
    }
}

/// Drives the network stack (must run continuously).
#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, Interface>) {
    runner.run().await
}
