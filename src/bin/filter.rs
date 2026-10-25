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
use mpu6050_driver::{Address, Mpu6050, RawAccelGyroTemp};
use nb;

esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[main]
fn main() -> ! {
    esp_alloc::heap_allocator!(size: 32 * 1024);
    init_logger(log::LevelFilter::Info);
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let i2c = esp_hal::i2c::master::I2c::new(
        peripherals.I2C0,
        esp_hal::i2c::master::Config::default().with_frequency(Rate::from_khz(100)),
    )
    .unwrap()
    .with_scl(peripherals.GPIO18)
    .with_sda(peripherals.GPIO23);

    let mut delay = Delay::new();
    let mut mpu = Mpu6050::new(i2c, Address::Ad0Low); // or Ad0High

    mpu.wake().expect("wake failed");

    esp_println::println!("MPU6050 initialized successfully!");
    let mut uart = Uart::new(
        peripherals.UART0,
        esp_hal::uart::Config::default().with_baudrate(115200),
    )
    .unwrap();
    writeln!(uart, "Hello from ESP32 Rust!").ok();

    loop {
        let res = mpu.read_raw_accel_gyro_temp().unwrap();

        writeln!(uart, "{:?}|{:?}", res.gyro, res.accel);

        let mut buf = [0u8; 64];

        blocking_delay(Duration::from_millis(10));
    }
}
fn blocking_delay(duration: Duration) {
    let delay_start = Instant::now();
    while delay_start.elapsed() < duration {}
}

fn complementary_filter(data: RawAccelGyroTemp) -> (i16, i16, i16, i16, i16, i16) {
    let acc_x = data.accel[0];
    let acc_y = data.accel[1];
    let acc_z = data.accel[2];

    let acc_x_angle = acc_x
    
}
