#![no_std]
#![no_main]

use defmt_rtt as _;
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    delay::Delay,
    gpio::Level,
    ledc::{
        LSGlobalClkSource, Ledc, LowSpeed,
        channel::{self, ChannelIFace},
        timer::{self, TimerIFace},
    },
    main,
    time::Rate,
};
use esp_println::println;

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    let delay = Delay::new();

    // Change this pin to the one connected to your ESC signal wire
    let esc_pin = peripherals.GPIO18;

    let mut ledc = Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let mut lstimer0 = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    lstimer0
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty14Bit, // Higher resolution is better
            clock_source: timer::LSClockSource::APBClk,
            frequency: Rate::from_hz(50), // 50 Hz for ESC
        })
        .unwrap();

    let mut channel0 = ledc.channel(channel::Number::Channel0, esc_pin);
    channel0
        .configure(channel::config::Config {
            timer: &lstimer0,
            duty_pct: 0,
            drive_mode: esp_hal::gpio::DriveMode::PushPull,
        })
        .unwrap();

    println!("ESC starting...");

    // === ARMING SEQUENCE (very important) ===
    // Most ESCs need low throttle for 2–3 seconds to arm
    set_esc_throttle(&mut channel0, 1000); // 1000 µs = stop
    delay.delay_millis(3000);

    println!("ESC armed!");

    // Example: slowly ramp up and down
    loop {
        // Ramp up
        for throttle in (1000..=2000).step_by(20) {
            set_esc_throttle(&mut channel0, throttle);
            delay.delay_millis(30);
        }

        // Ramp down
        for throttle in (1000..=2000).rev().step_by(20) {
            set_esc_throttle(&mut channel0, throttle);
            delay.delay_millis(30);
        }
    }
}

/// us: 1000 = stop, 2000 = full throttle
fn set_esc_throttle<'a>(channel: &mut impl ChannelIFace<'a, LowSpeed>, us: u16) {
    // 50 Hz → period = 20_000 µs
    // 14-bit resolution → max duty = 16383
    let duty = ((us as u32) * 16383) / 20_000;
    channel.set_duty(duty as u8);
}

//// Set ESC throttle in microseconds (1000 = stop, 2000 = full)
//fn set_esc_throttle_2<C>(channel: &mut C, us: u16)
//where
//    C: ChannelIFace<LowSpeed>,
//{
//    // With 50 Hz and 14-bit resolution:
//    // Period = 20_000 µs
//    // Max duty = 16383 (2^14 - 1)
//    let duty = ((us as u32) * 16383) / 20_000;
//    channel.set_duty(duty as u8);
//}
