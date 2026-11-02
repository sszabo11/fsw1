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

    let mut uart = Uart::new(
        peripherals.UART0,
        esp_hal::uart::Config::default().with_baudrate(921_600),
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

        //controller.print_status();
    }
}

struct ControllerState {
    n: u8,
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
    radio: Radio<'a, S>,
    uart: Uart<'a, Blocking>,
    state: ControllerState,
}

impl<'a, S: SpiDevice> Controller<'a, S> {
    pub fn new(spi: S, busy: Input<'a>, delay: Delay, uart: Uart<'a, Blocking>) -> Self {
        Self {
            radio: Radio::new(spi, busy, delay),
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
        let mut buf = [0u8; 32];
        let mut budget = 128usize;
        while budget > 0 && self.uart.read_ready() {
            match self.uart.read(&mut buf) {
                Ok(n) => {
                    budget = budget.saturating_sub(n.max(1));
                    for &b in &buf[..n] {
                        if b == b'\n' || b == b'\r' {
                            if !self.state.line.is_empty() {
                                if let Some(command) = decode_command(self.state.line.as_bytes()) {
                                    match command {
                                        ControlCommand::Bank(v) => self.state.controls.bank = v,
                                        ControlCommand::Pitch(v) => self.state.controls.pitch = v,
                                        ControlCommand::Throttle(v) => {
                                            self.state.controls.throttle = v
                                        }
                                        _ => {}
                                    }
                                }
                                self.state.line.clear();
                            }
                        } else if b.is_ascii() && self.state.line.push(b as char).is_err() {
                            self.state.line.clear(); // line too long, drop it
                        }
                    }
                }
                //Ok(1) => {
                //    let b = buf[0];
                //    self.state.byte[0] = b;

                //    //println!("b: {}", b);
                //    if b == b'\n' || b == b'\r' {
                //        if !self.state.line.is_empty() {
                //            if let Some(command) = decode_command(self.state.line.as_bytes()) {
                //                println!("setting outbound to: {:?}", command);
                //                self.state.outbound = command
                //            }
                //            self.state.line.clear();
                //        }
                //    } else {
                //        println!("E");
                //        let _ = self.state.line.push(b as char);
                //    }
                //}
                //Ok(_) => {
                //    println!("qwe")
                //}
                Err(err) => {
                    println!("error: {}", err);
                    self.state.line.clear();
                }
            }
        }
    }

    pub fn send_command(&mut self) {
        if self.state.t_last_tx.elapsed() >= Duration::from_millis(20) {
            self.state.t_last_tx = Instant::now();
            //println!("sedning: {} b: {}", self.state.n, self.state.controls.bank);
            let payload = PayloadData::Control(ControlPacket {
                seq: self.state.n as u8,
                bank: self.state.controls.bank,
                pitch: self.state.controls.pitch,
                yaw: self.state.controls.yaw,
                throttle: self.state.controls.throttle,
            });
            let ok = self.radio.send(payload);
            if ok {
                self.state.n = self.state.n.wrapping_add(1);
            }
        }
    }
    pub fn print_status(&mut self) {
        if self.state.t_last_print.elapsed() >= Duration::from_millis(500) {
            self.state.t_last_print = Instant::now();
            println!("command: {:?}  tx#{}", self.state.outbound, self.state.n);
        }
    }
}
