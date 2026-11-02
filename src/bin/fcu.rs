#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

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
use fsw1::cmd::{ControlCommand, decode_command};
use fsw1::data::{PayloadData, TelemetryPacket};
use fsw1::motors::{duty_from_bank_angle, set_esc_throttle, set_esc_throttle2};
use fsw1::radio::Radio;
use fsw1::utils::{configure_lora, wait_busy};
use heapless::{String, Vec, format};
use num_traits::float::FloatCore;
use regiface::Command;

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

    println!("SX1262 ready, mode = RX");

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

    //println!("ESC starting...");

    //let max_duty = 16383u32;
    //let period_us = 20_000u32;
    //let us_to_duty = |us: u32| -> u32 {
    //    // duty = us / period * max_duty
    //    (us * max_duty) / period_us
    //};
    //// === ARMING SEQUENCE (very important) ===
    //// Most ESCs need low throttle for 2–3 seconds to arm
    //set_esc_throttle(&mut channel1, 1000); // 1000 µs = stop
    //delay.delay_millis(3000);
    //channel1.set_duty_hw(us_to_duty(1000));
    //delay.delay_millis(4000); // wait for arm

    //println!("ESC armed!");
    //for _ in 0..150 {
    //    set_esc_throttle2(&mut channel1, 0);
    //    delay.delay_millis(20);
    //}
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
    let mut prev_throttle = 0.0;

    // Plane -> Controller
    // Receive controls from controller
    // Transmit telemtery back
    let mut controller = Controller::new(spi, busy, delay);
    loop {
        controller.recieve_controls();
        let duty = duty_from_bank_angle(controller.state.controls.bank, min_duty, duty_gap);
        let _ = channel0.set_duty_cycle(duty);

        //controller.send_telementary();

        //let _ = channel0.set_duty_cycle(mid as u16);

        //let raw = (throttle / 100.) * 4095.;
        ////set_esc_throttle2(&mut channel1, throttle as u16);
        //let pulse_us: u32 = 1000 + (raw as u32 * 1000) / 4095;
        //let pulse_us: u32 = pulse_us.clamp(1000, 2000);

        //channel0.set_duty_hw(us_to_duty(pulse_us));

        //println!("raw={}  pulse={} us", raw, pulse_us);
        delay.delay_millis(20);
    }
}

struct FlightState {
    n: u8,
    outbound: TelemetryPacket,
    controls: Controls,
    byte: Vec<u8, 32>,
    line: heapless::String<64>,
    t_last_tx: Instant,
    t_last_print: Instant,
}
struct Controls {
    bank: f32,
    pitch: f32,
    yaw: f32,
    throttle: f32,
}

struct Controller<'a, S: SpiDevice> {
    radio: Radio<'a, S>,
    state: FlightState,
}

impl<'a, S: SpiDevice> Controller<'a, S> {
    pub fn new(spi: S, busy: Input<'a>, delay: Delay) -> Self {
        Self {
            radio: Radio::new(spi, busy, delay),
            state: FlightState {
                byte: Vec::new(),
                line: heapless::String::<64>::new(),
                n: 0,
                controls: Controls {
                    bank: 0.,
                    pitch: 0.,
                    yaw: 0.,
                    throttle: 0.,
                },
                outbound: TelemetryPacket {
                    altitude: 0.,
                    speed: 0.,
                    seq: 0,
                },
                t_last_tx: Instant::now(),
                t_last_print: Instant::now(),
            },
        }
    }

    pub fn recieve_controls(&mut self) {
        match self.radio.receive(50) {
            PayloadData::Control(p) => {
                //println!("p:{:?}", p);
                self.state.controls.bank = p.bank;
                self.state.controls.pitch = p.pitch;
                self.state.controls.yaw = p.yaw;
                self.state.controls.throttle = p.throttle;
            }

            PayloadData::Timeout => {
                println!("RX timeout (nothing valid received)")
            }
            _ => {}
        }
    }

    pub fn send_telementary(&mut self) {
        if self.state.t_last_tx.elapsed() >= Duration::from_millis(20) {
            self.state.t_last_tx = Instant::now();
            let payload = PayloadData::Telemetry(TelemetryPacket {
                seq: self.state.n as u8,
                speed: 0.,
                altitude: 0.,
            });
            let ok = self.radio.send(payload);
            if ok {
                self.state.n = self.state.n.wrapping_add(1);
            }
        }
    }
    //pub fn print_status(&mut self) {
    //    if self.state.t_last_print.elapsed() >= Duration::from_millis(500) {
    //        self.state.t_last_print = Instant::now();
    //        println!("command: {:?}  tx#{}", self.state.outbound, self.state.n);
    //    }
    //}
}
