#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]
use defmt_rtt as _;
use embedded_hal::{delay::DelayNs, pwm::SetDutyCycle};
use embedded_io::{Read, Write};
use esp_alloc as _; // ← this is required
use esp_backtrace as _;
use esp_hal::analog::adc::{Adc, AdcConfig, Attenuation};
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::gpio::{DriveMode, Level, Output, OutputConfig};
use esp_hal::i2c::master::I2c;
use esp_hal::ledc::channel::{ChannelHW, ChannelIFace};
use esp_hal::ledc::timer::TimerIFace;
use esp_hal::ledc::{HighSpeed, LSGlobalClkSource, Ledc, LowSpeed, channel, timer};
use esp_hal::main;
use esp_hal::rng::Rng;
use esp_hal::time::{Duration, Instant, Rate};
use esp_hal::timer::timg::TimerGroup;
use esp_hal::uart::{Config, Uart};
use esp_println as _; // ← important for defmt
use esp_println::{logger::init_logger, println};
use nb::block;

esp_bootloader_esp_idf::esp_app_desc!();
#[main]
fn main() -> ! {
    esp_alloc::heap_allocator!(size: 32 * 1024);
    init_logger(log::LevelFilter::Info);

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    let mut delay = Delay::new();

    // ---------- ADC (potentiometer on GPIO34) ----------
    let mut adc_config = AdcConfig::new();
    let mut pot_pin = adc_config.enable_pin(peripherals.GPIO34, Attenuation::_11dB);
    let mut adc = Adc::new(peripherals.ADC1, adc_config);

    // ---------- LEDC PWM for ESC (GPIO5) ----------
    let mut ledc = Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let mut hstimer0 = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    hstimer0
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty14Bit, // good resolution
            clock_source: timer::LSClockSource::APBClk,
            frequency: Rate::from_hz(50), // 50 Hz for ESC
        })
        .unwrap();

    let mut channel0 = ledc.channel(channel::Number::Channel0, peripherals.GPIO5);
    channel0
        .configure(channel::config::Config {
            timer: &hstimer0,
            duty_pct: 0,
            drive_mode: esp_hal::gpio::DriveMode::PushPull,
        })
        .unwrap();

    // Helper: microseconds → duty for 50 Hz (20_000 µs period), 14-bit
    // max_duty = 2^14 - 1 = 16383
    let max_duty = 16383u32;
    let period_us = 20_000u32;

    let us_to_duty = |us: u32| -> u32 {
        // duty = us / period * max_duty
        (us * max_duty) / period_us
    };

    // ---------- ARM sequence (most ESCs) ----------
    // 1. Stick low (1000 µs) for a couple of seconds
    println!("Arming: low throttle...");
    //channel0.set_duty_cycle_percent(10).unwrap();
    //delay.delay_ms(3000);
    //channel0.set_duty_cycle_percent(7).unwrap();
    //delay.delay_ms(3000);

    channel0.set_duty_hw(us_to_duty(1000));
    delay.delay_millis(4000); // wait for arm

    // Some ESCs beep and arm here. If not, check your ESC manual
    // (some want max then min, etc.)
    println!("Armed. Use pot for throttle.");

    loop {
        // Read pot (0..4095 typical for 12-bit)
        let raw = nb::block!(adc.read_oneshot(&mut pot_pin)).unwrap();

        // Map 0..4095 → 1000..2000 µs
        let pulse_us: u32 = 1000 + (raw as u32 * 1000) / 4095;
        let pulse_us: u32 = pulse_us.clamp(1000, 2000);

        channel0.set_duty_hw(us_to_duty(pulse_us));

        println!("raw={}  pulse={} us", raw, pulse_us);
        delay.delay_millis(50);
    }
}
