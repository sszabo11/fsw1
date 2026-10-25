#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

// Crate-free SX1262 driver: every command is raw SPI, so nothing can hang inside
// the sx1262 / regiface crates. Replace your main.rs with this file.

use core::fmt::Write;
use defmt_rtt as _;
use embedded_hal::delay::DelayNs;
use embedded_hal::pwm::SetDutyCycle;
use embedded_hal::spi::{Operation, SpiDevice};
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_io::{Read, Write as EmbeddedWrite};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::gpio::{DriveMode, Input, InputConfig, Level, Output, OutputConfig, Pull};
use esp_hal::ledc::channel::{ChannelHW, ChannelIFace};
use esp_hal::ledc::timer::TimerIFace;
use esp_hal::ledc::{HighSpeed, LSGlobalClkSource, Ledc, LowSpeed, channel, timer};
use esp_hal::main;
use esp_hal::spi::Mode;
use esp_hal::spi::master::{Config, Spi};
use esp_hal::time::{Duration, Instant, Rate};
use esp_hal::uart::Uart;
use esp_println as _;
use esp_println::logger::init_logger;
use esp_println::println;
use heapless::{String, Vec, format};
use regiface::Command;

esp_bootloader_esp_idf::esp_app_desc!();

const IS_TX: bool = false;

const FREQUENCY_HZ: u64 = 915_200_000;
const PAYLOAD_LEN: u8 = 16;

const IRQ_TX_DONE: u16 = 0x0001;
const IRQ_RX_DONE: u16 = 0x0002;
const IRQ_PREAMBLE: u16 = 0x0004;
const IRQ_SYNC_WORD: u16 = 0x0008;
const IRQ_HEADER_VALID: u16 = 0x0010;
const IRQ_HEADER_ERR: u16 = 0x0020;
const IRQ_CRC_ERR: u16 = 0x0040;
const IRQ_TIMEOUT: u16 = 0x0200;

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

    let mut delay = Delay::new();

    let nss = Output::new(peripherals.GPIO18, Level::High, OutputConfig::default());
    let mut reset = Output::new(peripherals.GPIO23, Level::High, OutputConfig::default());
    let busy = Input::new(
        peripherals.GPIO25,
        InputConfig::default().with_pull(Pull::None),
    );
    let dio1 = Input::new(
        peripherals.GPIO22,
        InputConfig::default().with_pull(Pull::None),
    );

    let mut uart = Uart::new(
        peripherals.UART0,
        esp_hal::uart::Config::default().with_baudrate(115200),
    )
    .unwrap()
    .with_rx(peripherals.GPIO3)
    .with_tx(peripherals.GPIO1);

    let spi = Spi::new(
        peripherals.SPI2,
        Config::default()
            .with_frequency(Rate::from_mhz(1))
            .with_mode(Mode::_0),
    )
    .unwrap()
    .with_sck(peripherals.GPIO21)
    .with_mosi(peripherals.GPIO15)
    .with_miso(peripherals.GPIO4);
    let mut spi = ExclusiveDevice::new(spi, nss, delay).unwrap();

    // Hardware reset
    reset.set_low();
    delay.delay_ms(10);
    reset.set_high();
    delay.delay_ms(20);
    wait_busy(&busy, &mut delay);

    configure_lora(&mut spi, &busy, &mut delay);
    println!("SX1262 ready, mode = {}", if IS_TX { "TX" } else { "RX" });

    let mut motor = peripherals.GPIO26;
    let mut servo = peripherals.GPIO19;
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

    let mut lstimer0 = ledc.timer::<LowSpeed>(timer::Number::Timer1);
    lstimer0
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty14Bit, // Higher resolution is better
            clock_source: timer::LSClockSource::APBClk,
            frequency: Rate::from_hz(50), // 50 Hz for ESC
        })
        .unwrap();

    let mut channel1 = ledc.channel(channel::Number::Channel1, motor);
    channel1
        .configure(channel::config::Config {
            timer: &lstimer0,
            duty_pct: 0,
            drive_mode: esp_hal::gpio::DriveMode::PushPull,
        })
        .unwrap();

    println!("ESC starting...");

    let max_duty = 16383u32;
    let period_us = 20_000u32;
    let us_to_duty = |us: u32| -> u32 {
        // duty = us / period * max_duty
        (us * max_duty) / period_us
    };
    // === ARMING SEQUENCE (very important) ===
    // Most ESCs need low throttle for 2–3 seconds to arm
    //set_esc_throttle(&mut channel1, 1000); // 1000 µs = stop
    //delay.delay_millis(3000);
    channel1.set_duty_hw(us_to_duty(1000));
    delay.delay_millis(4000); // wait for arm

    println!("ESC armed!");
    for _ in 0..150 {
        set_esc_throttle2(&mut channel1, 0);
        delay.delay_millis(20);
    }

    let max_duty_cycle = channel0.max_duty_cycle() as u32;

    // Minimum duty (2.5%)
    // For 12bit -> 25 * 4096 /1000 => ~ 102
    let min_duty = (25 * max_duty_cycle) / 1000;
    // Maximum duty (12.5%)
    // For 12bit -> 125 * 4096 /1000 => 512
    let max_duty = (125 * max_duty_cycle) / 1000;
    // 512 - 102 => 410
    let duty_gap = max_duty - min_duty;

    let mut last_tx = Instant::now();
    let mut last_print = Instant::now();
    let mut c = ControlCommand::None;

    let mut byte = [0u8; 1];
    let mut line = heapless::String::<64>::new();

    // Controller -> Plane
    // Get control inputs from UART serial
    // Transmit controls to plane
    // Receive telemtery back
    if IS_TX {
        let mut n = 0u32;
        let mut last_sent = ControlCommand::None;
        loop {
            //line = format!(64; "BANK {}", n).unwrap();
            while uart.read_ready() {
                match uart.read(&mut byte) {
                    Ok(1) => {
                        let b = byte[0];

                        if b == b'\n' || b == b'\r' {
                            if !line.is_empty() {
                                if let Some(command) = decode_command(line.as_bytes()) {
                                    c = command
                                }
                                line.clear();
                            }
                        } else {
                            let _ = line.push(b as char);
                        }
                    }
                    Ok(_) => {}
                    Err(err) => {
                        println!("error: {}", err);
                    }
                }
            }

            if last_tx.elapsed() >= Duration::from_millis(20) {
                last_tx = Instant::now();
                let mut msg = heapless::String::<32>::new();
                match c {
                    ControlCommand::Bank(v) => {
                        let _ = write!(msg, "BANK {:.2}", v);
                    }
                    ControlCommand::Pitch(v) => {
                        let _ = write!(msg, "PITCH {:.2}", v);
                    }
                    //ControlCommand::None => {
                    //    // optional: skip TX, or send keepalive
                    //    let _ = write!(msg, "BANK {:.2}", 0.0);
                    //}
                    _ => {}
                }
                if !msg.is_empty() {
                    let ok = transmit(&mut spi, &busy, &mut delay, msg.as_bytes());
                    if ok {
                        last_sent = c;
                        n = n.wrapping_add(1);
                    }
                }
                //if transmit(&mut spi, &busy, &mut delay, msg) {
                //    //println!("TX OK #{}", n);
                //} else {
                //    //println!("TX FAILED #{}", n);
                //}
                //n = n.wrapping_add(1);
            }
            if last_print.elapsed() >= Duration::from_millis(500) {
                last_print = Instant::now();
                println!("command: {:?}  tx#{}", c, n);
            }
        }
    } else {
        // Plane -> Controller
        // Receive controls from controller
        // Transmit telemtery back
        let mut current = ControlCommand::None;
        let mut bank: f32 = 0.0;
        let mut pitch: f32 = 0.0;
        let mut throttle: f32 = 0.0; // 0..100
        loop {
            match receive(&mut spi, &busy, &dio1, &mut delay, 50) {
                Some((buf, len)) => {
                    //println!("RX {} bytes: {:?}", len, &buf[..len]);
                    //println!("RX text: {:?}", core::str::from_utf8(&buf[..len]));

                    if let Some(cmd) = decode_command(&buf[..len]) {
                        match cmd {
                            ControlCommand::Bank(v) => bank = v,
                            ControlCommand::Pitch(v) => pitch = v,
                            ControlCommand::Throttle(v) => throttle = v.clamp(0.0, 100.0),
                            ControlCommand::None => {}
                            _ => {}
                        }
                    }
                    //println!("c: {:?}", c);
                }
                None => {
                    println!("RX timeout (nothing valid received)")
                }
            }
            let angle = map_bank_to_angle(bank);
            let duty = duty_from_angle(angle, min_duty, duty_gap);
            let _ = channel0.set_duty_cycle(duty);

            set_esc_throttle2(&mut channel1, throttle as u16);

            //match current {
            //    ControlCommand::Bank(v) => {
            //        //println!("deg: {} ", deg);

            //        let angle = map_bank_to_angle(v);
            //        let duty = duty_from_angle(angle, min_duty, duty_gap);
            //        let _ = channel0.set_duty_cycle(duty);
            //    }
            //    ControlCommand::Throttle(v) => {
            //        set_esc_throttle2(&mut channel1, v as u16);
            //    }
            //    _ => {}
            //}
            delay.delay_millis(20);
        }
    }
}

fn wait_busy(busy: &Input, delay: &mut Delay) {
    for _ in 0..10_000 {
        if busy.is_low() {
            return;
        }
        delay.delay_us(100);
    }
    panic!("SX1262 BUSY stuck high (check reset/BUSY wiring, TCXO, power)");
}

fn write_cmd<S: SpiDevice>(spi: &mut S, busy: &Input, delay: &mut Delay, op: u8, args: &[u8]) {
    wait_busy(busy, delay);
    if spi
        .transaction(&mut [Operation::Write(&[op]), Operation::Write(args)])
        .is_err()
    {
        println!("SPI error on opcode {:#04X}", op);
    }
    wait_busy(busy, delay);
}

fn read_cmd<S: SpiDevice>(spi: &mut S, busy: &Input, delay: &mut Delay, op: u8, out: &mut [u8]) {
    wait_busy(busy, delay);
    if spi
        .transaction(&mut [Operation::Write(&[op, 0x00]), Operation::Read(out)])
        .is_err()
    {
        println!("SPI error on opcode {:#04X}", op);
    }
    wait_busy(busy, delay);
}

fn read_register<S: SpiDevice>(
    spi: &mut S,
    busy: &Input,
    delay: &mut Delay,
    addr: u16,
    out: &mut [u8],
) {
    wait_busy(busy, delay);
    let a = addr.to_be_bytes();
    if spi
        .transaction(&mut [
            Operation::Write(&[0x1D, a[0], a[1], 0x00]),
            Operation::Read(out),
        ])
        .is_err()
    {
        println!("SPI error on ReadRegister");
    }
    wait_busy(busy, delay);
}

fn write_buffer<S: SpiDevice>(spi: &mut S, busy: &Input, delay: &mut Delay, data: &[u8]) {
    wait_busy(busy, delay);
    if spi
        .transaction(&mut [Operation::Write(&[0x0E, 0x00]), Operation::Write(data)])
        .is_err()
    {
        println!("SPI error on WriteBuffer");
    }
    wait_busy(busy, delay);
}

fn read_buffer<S: SpiDevice>(
    spi: &mut S,
    busy: &Input,
    delay: &mut Delay,
    offset: u8,
    out: &mut [u8],
) {
    wait_busy(busy, delay);
    if spi
        .transaction(&mut [
            Operation::Write(&[0x1E, offset, 0x00]),
            Operation::Read(out),
        ])
        .is_err()
    {
        println!("SPI error on ReadBuffer");
    }
    wait_busy(busy, delay);
}

fn get_status<S: SpiDevice>(spi: &mut S, busy: &Input, delay: &mut Delay) -> u8 {
    wait_busy(busy, delay);
    let mut s = [0u8; 1];
    let _ = spi.transaction(&mut [Operation::Write(&[0xC0]), Operation::Read(&mut s)]);
    wait_busy(busy, delay);
    s[0]
}

fn print_status<S: SpiDevice>(spi: &mut S, busy: &Input, delay: &mut Delay, label: &str) {
    let s = get_status(spi, busy, delay);
    let mode = match (s >> 4) & 0x07 {
        2 => "STDBY_RC",
        3 => "STDBY_XOSC",
        4 => "FS",
        5 => "RX",
        6 => "TX",
        _ => "unknown",
    };
    println!("Status {:#04X}: chip mode {} -- {}", s, mode, label);
}

fn get_irq<S: SpiDevice>(spi: &mut S, busy: &Input, delay: &mut Delay) -> u16 {
    let mut b = [0u8; 2];
    read_cmd(spi, busy, delay, 0x12, &mut b);
    u16::from_be_bytes(b)
}

fn clear_irq<S: SpiDevice>(spi: &mut S, busy: &Input, delay: &mut Delay) {
    write_cmd(spi, busy, delay, 0x02, &[0xFF, 0xFF]);
}

fn configure_lora<S: SpiDevice>(spi: &mut S, busy: &Input, delay: &mut Delay) {
    // STDBY_RC
    write_cmd(spi, busy, delay, 0x80, &[0x00]);

    // TCXO on DIO3: 1.8 V (0x02), start delay 10 ms = 640 * 15.625 us = 0x000280.
    // Use 0x07 for 3.3 V. REMOVE this command if the module has a plain crystal.
    write_cmd(spi, busy, delay, 0x97, &[0x02, 0x00, 0x02, 0x80]);
    delay.delay_ms(15);

    // Calibrate everything (required after enabling the TCXO)
    write_cmd(spi, busy, delay, 0x89, &[0x7F]);
    delay.delay_ms(10);

    // Clear device errors (XOSC_START_ERR is often set before the TCXO is up), then read
    write_cmd(spi, busy, delay, 0x07, &[0x00, 0x00]);
    let mut err = [0u8; 2];
    read_cmd(spi, busy, delay, 0x17, &mut err);
    println!(
        "Device errors = {:#06X} (0 is good; 0x0020 = XOSC start error, check TCXO)",
        u16::from_be_bytes(err)
    );

    // DIO2 controls the RF switch (only for modules wired that way)
    write_cmd(spi, busy, delay, 0x9D, &[0x01]);

    // Packet type: LoRa
    write_cmd(spi, busy, delay, 0x8A, &[0x01]);

    // Image calibration for 902-928 MHz
    write_cmd(spi, busy, delay, 0x98, &[0xE1, 0xE9]);

    // RF frequency: reg = freq * 2^25 / 32 MHz
    let frf = ((FREQUENCY_HZ << 25) / 32_000_000) as u32;
    write_cmd(spi, busy, delay, 0x86, &frf.to_be_bytes());

    // PA config for SX1262: dutyCycle 0x04, hpMax 0x07, deviceSel 0x00, paLut 0x01
    write_cmd(spi, busy, delay, 0x95, &[0x04, 0x07, 0x00, 0x01]);

    // TX params: +14 dBm, ramp 40 us (0x02)
    write_cmd(spi, busy, delay, 0x8E, &[14, 0x02]);

    // Buffer base addresses
    write_cmd(spi, busy, delay, 0x8F, &[0x00, 0x00]);

    // Modulation: SF7 (0x07), BW 125 kHz (0x04), CR 4/5 (0x01), LDRO off (0x00)
    write_cmd(spi, busy, delay, 0x8B, &[0x07, 0x04, 0x01, 0x00]);

    // Packet params: preamble 8, explicit header, payload len, CRC on, standard IQ
    write_cmd(
        spi,
        busy,
        delay,
        0x8C,
        &[0x00, 0x08, 0x00, PAYLOAD_LEN, 0x01, 0x00],
    );

    // IRQs: enable all (0x03FF), route all to DIO1
    write_cmd(
        spi,
        busy,
        delay,
        0x08,
        &[0x03, 0xFF, 0x03, 0xFF, 0x00, 0x00, 0x00, 0x00],
    );

    clear_irq(spi, busy, delay);
    print_status(spi, busy, delay, "after config (expect STDBY_RC)");
}

fn transmit<S: SpiDevice>(spi: &mut S, busy: &Input, delay: &mut Delay, data: &[u8]) -> bool {
    clear_irq(spi, busy, delay);
    write_buffer(spi, busy, delay, data);

    write_cmd(
        spi,
        busy,
        delay,
        0x8C,
        &[0x00, 0x08, 0x00, data.len() as u8, 0x01, 0x00],
    );

    write_cmd(spi, busy, delay, 0x83, &[0x00, 0x00, 0x00]);
    //print_status(spi, busy, delay, "right after SetTx (expect TX)");

    for ms in 0..2000u32 {
        let bits = get_irq(spi, busy, delay);
        if bits & IRQ_TX_DONE != 0 {
            clear_irq(spi, busy, delay);
            //println!("TX_DONE after {} ms (IRQ={:#06X})", ms, bits);
            return true;
        }
        if bits & IRQ_TIMEOUT != 0 {
            println!("TX timeout flag (IRQ={:#06X})", bits);
            return false;
        }
        delay.delay_ms(1);
    }
    println!("No TX_DONE within 2 s");
    false
}

fn receive<S: SpiDevice>(
    spi: &mut S,
    busy: &Input,
    dio1: &Input,
    delay: &mut Delay,
    timeout_ms: u32,
) -> Option<([u8; 64], usize)> {
    clear_irq(spi, busy, delay);

    // SetRx, 0xFFFFFF = continuous
    write_cmd(spi, busy, delay, 0x82, &[0xFF, 0xFF, 0xFF]);
    print_status(spi, busy, delay, "right after SetRx (expect RX)");
    println!("DIO1 pin = {}", dio1.is_high());

    for ms in 0..timeout_ms {
        let bits = get_irq(spi, busy, delay);

        if bits != 0 {
            println!(
                "t={} ms IRQ={:#06X} [{}{}{}{}{}{}{}] DIO1={}",
                ms,
                bits,
                if bits & IRQ_PREAMBLE != 0 {
                    "PREAMBLE "
                } else {
                    ""
                },
                if bits & IRQ_SYNC_WORD != 0 {
                    "SYNC "
                } else {
                    ""
                },
                if bits & IRQ_HEADER_VALID != 0 {
                    "HDR_OK "
                } else {
                    ""
                },
                if bits & IRQ_HEADER_ERR != 0 {
                    "HDR_ERR "
                } else {
                    ""
                },
                if bits & IRQ_CRC_ERR != 0 {
                    "CRC_ERR "
                } else {
                    ""
                },
                if bits & IRQ_RX_DONE != 0 {
                    "RX_DONE "
                } else {
                    ""
                },
                if bits & IRQ_TIMEOUT != 0 {
                    "TIMEOUT"
                } else {
                    ""
                },
                dio1.is_high()
            );
            clear_irq(spi, busy, delay);

            if bits & IRQ_RX_DONE != 0 && bits & (IRQ_CRC_ERR | IRQ_HEADER_ERR) == 0 {
                // GetRxBufferStatus -> [payload length, start pointer]
                let mut st = [0u8; 2];
                read_cmd(spi, busy, delay, 0x13, &mut st);
                let len = (st[0] as usize).min(64);
                let mut buf = [0u8; 64];
                read_buffer(spi, busy, delay, st[1], &mut buf[..len]);
                return Some((buf, len));
            }
        }
        delay.delay_ms(1);
    }

    // Back to STDBY_RC
    write_cmd(spi, busy, delay, 0x80, &[0x00]);
    None
}

#[derive(Debug, Clone, Copy)]
pub enum ControlCommand {
    Bank(f32),
    Pitch(f32),
    Yaw(f32),
    Throttle(f32),
    None,
}

fn decode_command(raw: &[u8]) -> Option<ControlCommand> {
    //println!("b: {:?}", raw);
    let s = str::from_utf8(raw).ok()?.trim();
    if s.is_empty() {
        return Some(ControlCommand::None);
    };
    //println!("decode: {}", s);

    let mut parts = s.split_whitespace();
    let key = parts.next()?;
    let val_str = parts.next()?;
    let val: f32 = val_str.parse().ok()?;

    let cmd = match key {
        "BANK" => ControlCommand::Bank(val),
        "PITCH" => ControlCommand::Pitch(val),
        "YAW" => ControlCommand::Yaw(val),
        "THROTTLE" => ControlCommand::Throttle(val),
        _ => ControlCommand::None,
    };

    Some(cmd)
}

fn package_command(cmd: &ControlCommand) -> &[u8] {
    match cmd {
        ControlCommand::None => "".as_bytes(),
        ControlCommand::Bank(val) => "BANK {}".as_bytes(),
        ControlCommand::Pitch(val) => "PITCH {}".as_bytes(),
        _ => "".as_bytes(),
    }
}
fn duty_from_angle(deg: u32, min_duty: u32, duty_gap: u32) -> u16 {
    let duty = min_duty + ((deg * duty_gap) / 180);
    duty as u16
}
fn map_bank_to_angle(v: f32) -> u32 {
    // v: 0..100 from joystick intensity code
    // center at 90 deg, left/right +/- 45
    // If your PC sends signed bank later, change this.
    let clamped = v.clamp(0.0, 100.0);
    let angle = 45.0 + (clamped / 100.0) * 90.0; // 45..135
    angle as u32
}
/// us: 1000 = stop, 2000 = full throttle
//fn set_esc_throttle<'a>(channel: &mut impl ChannelIFace<'a, LowSpeed>, us: u16) {
//    let max_duty = 16383u32;
//    let period_us = 20_000u32;
//    let us_to_duty = |us: u32| -> u32 {
//        // duty = us / period * max_duty
//        (us * max_duty) / period_us
//    };
//    // 50 Hz → period = 20_000 µs
//    // 14-bit resolution → max duty = 16383
//    //let duty = ((us as u32) * 16383) / 20_000;
//    //channel.set_duty(duty as u8);
//    let pulse_us: u32 = 1000 + (us as u32 * 1000) / 4095;
//    let pulse_us: u32 = pulse_us.clamp(1000, 2000);
//
//    channel.set_duty(us_to_duty(pulse_us) as u8);
//
//    //println!("raw={}  pulse={} us", raw, pulse_us);
//}

fn set_esc_throttle<'a>(channel: &mut impl ChannelIFace<'a, LowSpeed>, throttle_0_to_100: u16) {
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

fn set_esc_throttle2<'a>(channel: &mut impl ChannelIFace<'a, LowSpeed>, throttle_0_to_100: u16) {
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
