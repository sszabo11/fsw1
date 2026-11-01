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
use esp_alloc as _;
use esp_backtrace as _;
use esp_println as _;
use esp_println::logger::init_logger;
use esp_println::println;

use embedded_hal::delay::DelayNs;
use embedded_hal::spi::{Operation, SpiDevice};
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_io::{Read, Write as EmbeddedWrite};
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::gpio::{DriveMode, Input, InputConfig, Level, Output, OutputConfig, Pull};
use esp_hal::spi::Mode;
use esp_hal::spi::master::{Config, Spi};
use esp_hal::time::{Duration, Instant, Rate};
use esp_hal::uart::Uart;
use esp_hal::{Blocking, main};
use fsw1::cmd::{ControlCommand, decode_command};
use fsw1::data::{ControlPacket, PayloadData, encode_packet};
use fsw1::rx::Receiver;
use fsw1::tx::{Transmitter, transmit};
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

    println!("SX1262 ready, mode = TX");

    let mut controller = Controller::new(spi, busy, delay, uart);
    // Controller -> Plane
    // Get control inputs from UART serial
    // Transmit controls to plane
    // Receive telemtery back
    loop {
        controller.read_uart();
        controller.send_command();
        controller.print_status();
    }
}

struct LoraConfig<'a, S: SpiDevice> {
    spi: S,
    busy: Input<'a>,
    delay: Delay,
}

struct ControllerState {
    n: u32,
    outbound: ControlCommand,
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
    rx: Receiver<'a, S>,
    tx: Transmitter<'a, S>,
    uart: Uart<'a, Blocking>,
    state: ControllerState,
}

impl<'a, S: SpiDevice> Controller<'a, S> {
    pub fn new(spi: S, busy: Input<'a>, delay: Delay, uart: Uart<'a, Blocking>) -> Self {
        Self {
            rx: Receiver::new(spi, busy, delay),
            tx: Transmitter::new(spi, busy, delay),
            uart,
            state: ControllerState {
                byte: Vec::new(),
                line: heapless::String::<64>::new(),
                n: 0,
                controls: Controls {
                    bank: 0.,
                    pitch: 0.,
                    yaw: 0.,
                    throttle: 0.,
                },
                outbound: ControlCommand::None,
                t_last_tx: Instant::now(),
                t_last_print: Instant::now(),
            },
        }
    }

    pub fn read_uart(&mut self) {
        while self.uart.read_ready() {
            let mut buf = [0u8; 1];
            match self.uart.read(&mut buf) {
                Ok(1) => {
                    let b = buf[0];
                    self.state.byte[0] = b;

                    if b == b'\n' || b == b'\r' {
                        if !self.state.line.is_empty() {
                            if let Some(command) = decode_command(self.state.line.as_bytes()) {
                                self.state.outbound = command
                            }
                            self.state.line.clear();
                        }
                    } else {
                        let _ = self.state.line.push(b as char);
                    }
                }
                Ok(_) => {}
                Err(err) => {
                    println!("error: {}", err);
                }
            }
        }
    }

    pub fn send_command(&mut self) {
        if self.state.t_last_tx.elapsed() >= Duration::from_millis(20) {
            self.state.t_last_tx = Instant::now();
            //let mut msg = heapless::String::<32>::new();
            //match self.state.outbound {
            //    ControlCommand::Bank(v) => {
            //        self.state.bank = v;
            //    }
            //    ControlCommand::Pitch(v) => {
            //        //let _ = write!(msg, "PITCH {:.2}", v);
            //    }
            //    //ControlCommand::Throttle(v) => {
            //    //    if (v - prev_throttle).abs() > 1. {
            //    //        let _ = write!(msg, "THROTTLE {:.2}", v);
            //    //        prev_throttle = v;
            //    //    }
            //    //}
            //    //ControlCommand::None => {
            //    //    // optional: skip TX, or send keepalive
            //    //    let _ = write!(msg, "BANK {:.2}", 0.0);
            //    //}
            //    _ => {}
            //};
            let payload = PayloadData::Control(ControlPacket {
                seq,
                bank: self.state.controls.bank,
                pitch: self.state.controls.pitch,
                yaw: self.state.controls.yaw,
                throttle: self.state.controls.throttle,
            });
            //if !msg.is_empty() {
            let ok = self.tx.send(payload);
            if ok {
                //self.state.last_sent = self.state.outbound;
                self.state.n = self.state.n.wrapping_add(1);
            }
            //}
        }
    }
    pub fn print_status(&mut self) {
        if self.state.t_last_print.elapsed() >= Duration::from_millis(500) {
            self.state.t_last_print = Instant::now();
            println!("command: {:?}  tx#{}", self.state.outbound, self.state.n);
        }
    }
}
