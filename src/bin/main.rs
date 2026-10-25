#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use blocking_network_stack::Stack;
use defmt_rtt as _;
use defmt_rtt as _;
use embedded_hal::pwm::SetDutyCycle;
use embedded_io::{Read, Write};
use esp_alloc as _; // ← this is required
use esp_backtrace as _;
use esp_hal::analog::adc::{Adc, AdcConfig, Attenuation};
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::gpio::{DriveMode, Level, Output, OutputConfig};
use esp_hal::i2c::master::I2c;
use esp_hal::ledc::channel::ChannelIFace;
use esp_hal::ledc::timer::TimerIFace;
use esp_hal::ledc::{HighSpeed, LSGlobalClkSource, Ledc, LowSpeed, channel, timer};
use esp_hal::main;
use esp_hal::rng::Rng;
use esp_hal::time::{Duration, Instant, Rate};
use esp_hal::timer::timg::TimerGroup;
use esp_hal::uart::{Config, Uart};
use esp_println as _; // ← important for defmt
use esp_println::logger::init_logger;
use esp_println::println;
use mpu6050_driver::{Address, Mpu6050};
use nb;
use smoltcp::iface::SocketStorage;
//extern crate alloc;

// Define a heap of 32 KB (adjust size to your needs)
//extern crate alloc;

//#[panic_handler]
//fn panic(_: &core::panic::PanicInfo) -> ! {
//    loop {}
//}

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[main]
fn main() -> ! {
    esp_alloc::heap_allocator!(size: 32 * 1024); // or whatever size you need
    //esp_alloc::heap_allocator!(size: 32 * 1024);
    init_logger(log::LevelFilter::Info);
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let mut led = Output::new(peripherals.GPIO2, Level::High, OutputConfig::default());
    let mut servo = peripherals.GPIO33;

    let adc_pin = peripherals.GPIO34;

    let mut adc_config = AdcConfig::new();
    let mut pin = adc_config.enable_pin(adc_pin, Attenuation::_11dB);

    let mut adc1 = Adc::new(peripherals.ADC1, adc_config);

    let mut ledc = Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let mut hstimer0 = ledc.timer::<HighSpeed>(timer::Number::Timer0);
    hstimer0
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty12Bit,
            clock_source: timer::HSClockSource::APBClk,
            frequency: Rate::from_hz(50),
        })
        .unwrap();

    let mut channel0 = ledc.channel(channel::Number::Channel0, servo.reborrow());
    channel0
        .configure(channel::config::Config {
            timer: &hstimer0,
            duty_pct: 10,
            drive_mode: DriveMode::PushPull,
        })
        .unwrap();

    let i2c = esp_hal::i2c::master::I2c::new(
        peripherals.I2C0,
        esp_hal::i2c::master::Config::default().with_frequency(Rate::from_khz(100)),
    )
    .unwrap()
    .with_scl(peripherals.GPIO18)
    .with_sda(peripherals.GPIO23);

    //let mut delay = Delay::new();
    //let mut mpu = Mpu6050::new(i2c, Address::Ad0Low); // or Ad0High
    //
    //mpu.wake().expect("wake failed");

    //esp_println::println!("MPU6050 initialized successfully!");
    //let mut lstimer0 = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    //lstimer0
    //    .configure(timer::config::Config {
    //        duty: timer::config::Duty::Duty5Bit,
    //        clock_source: timer::LSClockSource::APBClk,
    //        frequency: Rate::from_khz(24),
    //    })
    //    .unwrap();

    //let mut channel0 = ledc.channel(channel::Number::Channel0, led);
    //channel0
    //    .configure(channel::config::Config {
    //        timer: &lstimer0,
    //        duty_pct: 10,
    //        drive_mode: DriveMode::PushPull,
    //    })
    //    .unwrap();

    let max_duty_cycle = channel0.max_duty_cycle() as u32;

    // Minimum duty (2.5%)
    // For 12bit -> 25 * 4096 /1000 => ~ 102
    let min_duty = (25 * max_duty_cycle) / 1000;
    // Maximum duty (12.5%)
    // For 12bit -> 125 * 4096 /1000 => 512
    let max_duty = (125 * max_duty_cycle) / 1000;
    // 512 - 102 => 410
    let duty_gap = max_duty - min_duty;

    let mut uart = Uart::new(
        peripherals.UART0,
        esp_hal::uart::Config::default().with_baudrate(115200),
    )
    .unwrap();
    writeln!(uart, "Hello from ESP32 Rust!").ok();

    let delay = Delay::new();

    loop {
        let adc_value = nb::block!(adc1.read_oneshot(&mut pin)).unwrap();
        ////let adc_value: u16 = adc1.read_oneshot(&mut pin).unwrap();

        let percant = adc_value as f32 / 4095.;
        let deg = (180. * percant) as u32;
        println!("degg: {}", deg);

        //let res = mpu.read_raw_accel_gyro_temp().unwrap();
        //println!(
        //    "Gyro: {:?} | Accel: {:?} | Temp: {}",
        //    res.gyro, res.accel, res.temp
        //);
        //let mut buf = [0u8; 64];

        //w/riteln!(uart, "Gyro: {:?}", res.gyro).ok();

        //match uart.read(&mut buf) {
        //    Ok(0) => {} // nothing available
        //    Ok(n) => {
        //        // Got n bytes
        //        let _ = uart.write(&buf[..n]); // echo
        //        println!("Got {} bytes: {:?}", n, &buf[..n]);
        //    }
        //    Err(_) => {}
        //}
        //println!("RAW: {} | Deg: {} perc: {}", adc_value, deg, percant);

        //blocking_delay(Duration::from_millis(500));
        //led.toggle();

        //channel0.start_duty_fade(0, 100, 1000).unwrap();
        //while channel0.is_duty_fade_running() {}
        //channel0.start_duty_fade(100, 0, 1000).unwrap();
        //while channel0.is_duty_fade_running() {}

        let duty = duty_from_angle(deg, min_duty, duty_gap);
        channel0.set_duty_cycle(duty).unwrap();
        //for deg in 0..=180 {
        //    let duty = duty_from_angle(deg, min_duty, duty_gap);
        //    channel0.set_duty_cycle(duty).unwrap();
        //    delay.delay_millis(10);
        //}
        //delay.delay_millis(500);

        //for deg in (0..=180).rev() {
        //    let duty = duty_from_angle(deg, min_duty, duty_gap);
        //    channel0.set_duty_cycle(duty).unwrap();
        //    delay.delay_millis(10);
        //}
        delay.delay_millis(10);
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.1.0/examples
}

fn blocking_delay(duration: Duration) {
    let delay_start = Instant::now();
    while delay_start.elapsed() < duration {}
}
fn duty_from_angle(deg: u32, min_duty: u32, duty_gap: u32) -> u16 {
    let duty = min_duty + ((deg * duty_gap) / 180);
    duty as u16
}

//pub fn create_interface(device: &mut esp_radio::wifi::WifiDevice) -> smoltcp::iface::Interface {
//    // users could create multiple instances but since they only have one WifiDevice
//    // they probably can't do anything bad with that
//    smoltcp::iface::Interface::new(
//        smoltcp::iface::Config::new(smoltcp::wire::HardwareAddress::Ethernet(
//            smoltcp::wire::EthernetAddress::from_bytes(&device.mac_address()),
//        )),
//        device,
//        timestamp(),
//    )
//}
//
//// some smoltcp boilerplate
//fn timestamp() -> smoltcp::time::Instant {
//    smoltcp::time::Instant::from_micros(
//        esp_hal::time::Instant::now()
//            .duration_since_epoch()
//            .as_micros() as i64,
//    )
//}
//
//fn init_hardware() -> Peripherals {
//    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
//    let peripherals = esp_hal::init(config);
//    esp_alloc::heap_allocator!(size: 72 * 1024);
//    peripherals
//}
//
//fn configure_wifi(controller: &mut WifiController<'_>) {
//    controller
//        .set_power_saving(esp_radio::wifi::PowerSaveMode::None)
//        .unwrap();
//
//    let client_config = ModeConfig::Client(
//        ClientConfig::default()
//            .with_ssid(SSID.into())
//            .with_password(PASSWORD.into()),
//    );
//    let res = controller.set_config(&client_config);
//    println!("wifi_set_configuration returned {:?}", res);
//
//    controller.start().unwrap();
//    println!("is wifi started: {:?}", controller.is_started());
//}
//
//fn scan_wifi(controller: &mut WifiController<'_>) {
//    println!("Start Wifi Scan");
//    let scan_config = ScanConfig::default().with_max(10);
//    let res = controller.scan_with_config(scan_config).unwrap();
//    for ap in res {
//        println!("{:?}", ap);
//    }
//}
//
//fn connect_wifi(controller: &mut WifiController<'_>) {
//    println!("{:?}", controller.capabilities());
//    println!("wifi_connect {:?}", controller.connect());
//
//    println!("Wait to get connected");
//    loop {
//        match controller.is_connected() {
//            Ok(true) => break,
//            Ok(false) => {}
//            Err(err) => panic!("{:?}", err),
//        }
//    }
//    println!("Connected: {:?}", controller.is_connected());
//}
//
//fn obtain_ip(stack: &mut Stack<'_, esp_radio::wifi::WifiDevice<'_>>) {
//    println!("Wait for IP address");
//    loop {
//        stack.work();
//        if stack.is_iface_up() {
//            println!("IP acquired: {:?}", stack.get_ip_info());
//            break;
//        }
//    }
//}
//
//fn http_loop(
//    mut socket: blocking_network_stack::Socket<'_, '_, esp_radio::wifi::WifiDevice<'_>>,
//) -> ! {
//    println!("Starting HTTP client loop");
//    let delay = Delay::new();
//    loop {
//        println!("Making HTTP request");
//        socket.work();
//
//        let remote_addr = IpAddress::v4(142, 250, 185, 115);
//        socket.open(remote_addr, 80).unwrap();
//        socket
//            .write(b"GET / HTTP/1.0\r\nHost: www.mobile-j.de\r\n\r\n")
//            .unwrap();
//        socket.flush().unwrap();
//
//        let deadline = Instant::now() + Duration::from_secs(20);
//        let mut buffer = [0u8; 512];
//        while let Ok(len) = socket.read(&mut buffer) {
//            // let text = unsafe { core::str::from_utf8_unchecked(&buffer[..len]) };
//            let Ok(text) = core::str::from_utf8(&buffer[..len]) else {
//                panic!("Invalid UTF-8 sequence encountered");
//            };
//
//            println!("{}", text);
//
//            if Instant::now() > deadline {
//                println!("Timeout");
//                break;
//            }
//        }
//
//        socket.disconnect();
//        let deadline = Instant::now() + Duration::from_secs(5);
//        while Instant::now() < deadline {
//            socket.work();
//        }
//
//        delay.delay_millis(1000);
//    }
//}
