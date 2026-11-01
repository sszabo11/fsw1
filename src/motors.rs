#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use esp_hal::ledc::{LowSpeed, channel::ChannelIFace};
use num_traits::float::FloatCore;

pub fn duty_from_angle(deg: u32, min_duty: u32, duty_gap: u32) -> u16 {
    let duty = min_duty + ((deg * duty_gap) / 180);
    duty as u16
}
pub fn duty_from_bank_angle(deg: f32, min_duty: u32, duty_gap: u32) -> u16 {
    let deg = deg.clamp(-90.0, 90.0);
    let t = (deg + 90.0) / 180.0; // 0.0 … 1.0
    let duty = min_duty as f32 + t * duty_gap as f32;
    duty.round() as u16
}
pub fn map_bank_to_angle(v: f32) -> u32 {
    // v: 0..100 from joystick intensity code
    // center at 90 deg, left/right +/- 45
    // If your PC sends signed bank later, change this.
    let clamped = v.clamp(0.0, 100.0);
    let angle = 45.0 + (clamped / 100.0) * 90.0; // 45..135
    angle as u32
}

pub fn set_esc_throttle<'a>(channel: &mut impl ChannelIFace<'a, LowSpeed>, throttle_0_to_100: u16) {
    let max_duty = 16_383u32; // 14-bit
    let period_us = 20_000u32; // 50 Hz

    // clamp 0..=100
    let t = (throttle_0_to_100 as u32).clamp(0, 100);

    // map 0..100 → 1000..2000 µs
    let pulse_us = 1000 + (t * 1000) / 100; // 1000..2000
    // same as: 1000 + t * 10

    let duty = (pulse_us * max_duty) / period_us;

    // IMPORTANT: do not cast duty to u8
    channel.set_duty(duty as u8).ok();
    // if your API is set_duty_cycle_fraction / different width, match that type
}

pub fn set_esc_throttle2<'a>(
    channel: &mut impl ChannelIFace<'a, LowSpeed>,
    throttle_0_to_100: u16,
) {
    let max_duty = 255u32; // 8-bit
    let period_us = 20_000u32; // 50 Hz

    let t = (throttle_0_to_100 as u32).clamp(0, 100);

    // 0..100 → 1000..2000 µs
    let pulse_us = 1000 + t * 10;

    // pulse → duty
    let duty = (pulse_us * max_duty) / period_us; // ~12..25
    let duty = duty.clamp(0, 255) as u8;

    channel.set_duty(duty);
}
