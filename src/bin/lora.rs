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
use embedded_hal::delay::DelayNs;
use embedded_hal::pwm::SetDutyCycle;
use embedded_hal::spi::SpiDevice;
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_io::{Read, Write};
use esp_alloc as _; // ← this is required
use esp_backtrace as _;
use esp_hal::analog::adc::{Adc, AdcConfig, Attenuation};
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::gpio::{DriveMode, Input, InputConfig, Level, Output, OutputConfig, Pull};
use esp_hal::i2c::master::I2c;
use esp_hal::ledc::channel::ChannelIFace;
use esp_hal::ledc::timer::TimerIFace;
use esp_hal::ledc::{HighSpeed, LSGlobalClkSource, Ledc, LowSpeed, channel, timer};
use esp_hal::main;
use esp_hal::rng::Rng;
use esp_hal::spi::Mode;
use esp_hal::spi::master::{Command, Config, Spi};
use esp_hal::time::{Duration, Instant, Rate};
use esp_hal::timer::timg::TimerGroup;
use esp_println as _; // ← important for defmt
use esp_println::logger::init_logger;
use esp_println::println;
use mpu6050_driver::{Address, Mpu6050, RawAccelGyroTemp};
use nb;
use sx1262::{
    BufferBaseAddressConfig, ClearIrqStatus, CodingRate, Device, DeviceSelect, DioIrqConfig, Error,
    GetIrqStatus, GetRxBufferStatus, GetStatus, IrqMask, LoRaBandwidth, LoRaModParams,
    ModulationParams, PaConfig, PacketParams, PacketType, RampTime, RfFrequencyConfig,
    RfSwitchConfig, SetBufferBaseAddress, SetDio2AsRfSwitchCtrl, SetDio3AsTcxoCtrl,
    SetDioIrqParams, SetModulationParams, SetPaConfig, SetPacketParams, SetPacketType,
    SetRfFrequency, SetRx, SetStandby, SetTx, SetTxParams, SpreadingFactor, StandbyConfig,
    TcxoConfig, TcxoVoltage, Timeout, TxParams,
};

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

    //let i2c = esp_hal::i2c::master::I2c::new(
    //    peripherals.I2C0,
    //    esp_hal::i2c::master::Config::default().with_frequency(Rate::from_khz(100)),
    //)
    //.unwrap()
    //.with_scl(peripherals.GPIO18)
    //.with_sda(peripherals.GPIO23);

    let mut delay = Delay::new();
    //let mut mpu = Mpu6050::new(i2c, Address::Ad0Low); // or Ad0High

    //let spi = Spi::new(peripherals.SPI0, Config::default()?)?
    //    .with_sck(sclk)
    //    .with_mosi(cs)
    //    .with_miso(miso);

    let mut nss = Output::new(peripherals.GPIO18, Level::High, OutputConfig::default());
    let mut reset = Output::new(peripherals.GPIO23, Level::High, OutputConfig::default());
    let busy = Input::new(
        peripherals.GPIO25,
        InputConfig::default().with_pull(Pull::None),
    );

    let dio1 = Input::new(
        peripherals.GPIO22, // change to whatever you wired
        InputConfig::default().with_pull(Pull::None),
    );

    // --- SPI bus ---
    let spi = Spi::new(
        peripherals.SPI2,
        Config::default()
            .with_frequency(Rate::from_mhz(1)) // start slow (1 MHz is safe)
            .with_mode(Mode::_0),
    )
    .unwrap()
    .with_sck(peripherals.GPIO21)
    .with_mosi(peripherals.GPIO15)
    .with_miso(peripherals.GPIO4);
    let spi_dev = ExclusiveDevice::new(spi, nss, delay).unwrap();

    // --- Hardware reset sequence ---
    reset.set_low();
    delay.delay_ms(10);
    reset.set_high();
    delay.delay_ms(20);

    // Wait until radio is ready
    wait_busy(&busy, &mut delay);

    // --- Create SX1262 device ---
    let mut radio = Device::new(spi_dev);

    // ========== 2. Basic Configuration ==========
    configure_lora(&mut radio, &busy, &mut delay).expect("Config failed");

    esp_println::println!("SX1262 ready");

    // ========== 3. Main loop ==========
    let mut counter = 0u32;
    //loop {
    //    let msg = b"Hello ESP32!!!!!!"; // 16 bytes
    //    match transmit_dio(&mut radio, &busy, &dio1, &mut delay, msg) {
    //        Ok(()) => esp_println::println!("TX OK"),
    //        Err(e) => esp_println::println!("TX err: {:?}", e),
    //    }
    //    delay.delay_ms(1000);
    //}
    loop {
        match receive_poll(&mut radio, &busy, &dio1, &mut delay, 15_000) {
            Ok(Some((buf, len))) => {
                esp_println::println!("RX {} bytes: {:?}", len, core::str::from_utf8(&buf[..len]))
            }
            Ok(None) => esp_println::println!("RX timeout"),
            Err(e) => esp_println::println!("RX err: {:?}", e),
        }
    }
    //loop {
    //    exec(
    //        &mut radio,
    //        &busy,
    //        &mut delay,
    //        SetRx {
    //            mode: sx1262::RxMode::Timed(0x00_FF_FF_FF),
    //        },
    //    )
    //    .ok();

    //    esp_println::println!("listening 5s...");
    //    delay.delay_ms(5000);

    //    let mut buf = [0u8; 16];
    //    wait_busy(&busy, &mut delay);
    //    let _ = radio.read_buffer(0, &mut buf);
    //    wait_busy(&busy, &mut delay);

    //    esp_println::println!("buf={:?}", buf);
    //    esp_println::println!("str={:?}", core::str::from_utf8(&buf));

    //    let _ = exec(
    //        &mut radio,
    //        &busy,
    //        &mut delay,
    //        SetStandby {
    //            config: StandbyConfig::Rc,
    //        },
    //    );
    //}
    //loop {
    // TX
    //let msg = b"Hello from ESP32!!!!"; // fixed-ish length for easier RX
    //let msg = b"Hello ESP32!!!!!!"; // 16 bytes
    //match transmit_dio(&mut radio, &busy, &dio1, &mut delay, msg) {
    //    Ok(()) => esp_println::println!("TX OK"),
    //    Err(e) => esp_println::println!("TX err: {:?}", e),
    //}
    //delay.delay_ms(1500);

    //loop {
    //    // RX window
    //    match receive_dio(&mut radio, &busy, &dio1, &mut delay, 15_000) {
    //        Ok(Some(payload)) => {
    //            // print as text if possible
    //            if let Ok(s) = core::str::from_utf8(&payload) {
    //                esp_println::println!("RX: {}", s);
    //            } else {
    //                //esp_println::println!("RX binary: {:?}", &payload[..16]);
    //                esp_println::println!("RX: {:?}", core::str::from_utf8(&payload[..16]));
    //            }
    //        }
    //        Ok(None) => esp_println::println!("RX timeout"),
    //        Err(e) => esp_println::println!("RX err: {:?}", e),
    //    }
    //}

    //loop {
    //    // --- Transmit ---
    //    let msg = b"Hello from ESP32!";
    //    if let Err(e) = transmit(&mut radio, &busy, &mut delay, msg) {
    //        esp_println::println!("TX error: {:?}", e);
    //    } else {
    //        esp_println::println!("TX OK #{}", counter);
    //    }

    //    // --- Receive for 3 seconds ---
    //    if let Ok(Some(payload)) = receive(&mut radio, &busy, &mut delay, 3000) {
    //        esp_println::println!(
    //            "RX: {:?}",
    //            core::str::from_utf8(&payload).unwrap_or("binary")
    //        );
    //    }

    //    counter += 1;
    //    delay.delay_ms(2000);
    //}
}
fn blocking_delay(duration: Duration) {
    let delay_start = Instant::now();
    while delay_start.elapsed() < duration {}
}

fn receive_poll<SPI: embedded_hal::spi::SpiDevice>(
    radio: &mut Device<SPI>,
    busy: &Input,
    dio1: &Input,
    delay: &mut Delay,
    timeout_ms: u32,
) -> Result<Option<([u8; 64], usize)>, sx1262::Error> {
    exec(
        radio,
        busy,
        delay,
        ClearIrqStatus {
            irq_mask: IrqMask::all(),
        },
    )?;
    exec(
        radio,
        busy,
        delay,
        SetRx {
            mode: sx1262::RxMode::Timed(0x00_FF_FF_FF),
        },
    )?;

    for ms in 0..timeout_ms {
        if dio1.is_high() {
            let irq = exec(radio, busy, delay, GetIrqStatus)?;
            let bits = irq.irq_mask.bits();
            esp_println::println!("IRQ={:#06X} at {} ms", bits, ms);
            exec(
                radio,
                busy,
                delay,
                ClearIrqStatus {
                    irq_mask: IrqMask::all(),
                },
            )?;

            if bits & 0x0060 != 0 {
                esp_println::println!("header/CRC error");
                continue; // still in continuous RX
            }
            if bits & 0x0002 != 0 {
                let st = exec(radio, busy, delay, GetRxBufferStatus)?;
                let len = (st.buffer_status.payload_length as usize).min(64);
                let mut buf = [0u8; 64];
                radio.read_buffer(st.buffer_status.buffer_pointer, &mut buf[..len])?;
                wait_busy(busy, delay);
                return Ok(Some((buf, len)));
            }
        }
        delay.delay_ms(1);
    }

    let _ = exec(
        radio,
        busy,
        delay,
        SetStandby {
            config: StandbyConfig::Rc,
        },
    );
    Ok(None)
}

fn transmit_dio<SPI: embedded_hal::spi::SpiDevice>(
    radio: &mut Device<SPI>,
    busy: &Input,
    dio1: &Input,
    delay: &mut Delay,
    data: &[u8],
) -> Result<(), sx1262::Error> {
    // 1) Load payload
    wait_busy(busy, delay);
    radio.write_buffer(0, data)?;
    wait_busy(busy, delay);

    // 2) Update length (LoRa packet params)
    // [preamble_hi, preamble_lo, header_type, payload_len, crc_on, iq, 0, 0, 0]
    exec(
        radio,
        busy,
        delay,
        SetPacketParams {
            params: PacketParams {
                params: [0, 8, 0, data.len() as u8, 1, 0, 0, 0, 0],
            },
        },
    )?;
    let mut finished = false;
    for _ in 0..500 {
        if dio1.is_high() {
            finished = true;
            break;
        }
        delay.delay_ms(1);
    }
    exec(
        radio,
        busy,
        delay,
        ClearIrqStatus {
            irq_mask: IrqMask::all(),
        },
    )?;

    // 3) Start TX
    wait_busy(busy, delay);
    radio.execute_command(SetTx {
        timeout: Timeout(0), // no timeout
    })?;
    esp_println::println!("SetTx issued");

    // 4) Wait for completion via BUSY or DIO1
    // During TX, BUSY is high; when done it goes low.
    // If DIO1 is mapped to TxDone, it will go high.
    let mut finished = false;
    for _ in 0..5000 {
        // either condition is enough
        if busy.is_low() || dio1.is_high() {
            finished = true;
            break;
        }
        delay.delay_us(100);
    }

    if !finished {
        esp_println::println!("TX wait timeout");
        return Err(sx1262::Error::BusError);
    }

    // small settle time
    delay.delay_ms(5);
    esp_println::println!("TX done");
    Ok(())
}
fn receive_dio<SPI: embedded_hal::spi::SpiDevice>(
    radio: &mut Device<SPI>,
    busy: &Input,
    dio1: &Input,
    delay: &mut Delay,
    timeout_ms: u32,
) -> Result<Option<[u8; 64]>, sx1262::Error> {
    // Start continuous RX (or long timed RX)
    // If your SetRx API differs, keep whatever compiled for you earlier.
    exec(
        radio,
        busy,
        delay,
        SetRx {
            mode: sx1262::RxMode::Timed(0x00_FF_FF_FF),
        },
    )?;
    esp_println::println!("RX started");

    // Wait for DIO1 (RxDone) or timeout
    let mut got_packet = false;
    for _ in 0..timeout_ms {
        //if dio1.is_high() {
        //    got_packet = true;
        //    break;
        //}
        //delay.delay_ms(1);

        esp_println::println!("RX started, DIO1={}", dio1.is_high());

        let mut got_packet = false;
        for i in 0..timeout_ms {
            if dio1.is_high() {
                got_packet = true;
                esp_println::println!("DIO1 HIGH at {} ms", i);
                break;
            }
            if i % 1000 == 0 {
                esp_println::println!("wait {} ms, DIO1={}", i, dio1.is_high());
            }
            delay.delay_ms(1);
        }
    }

    println!("rie");
    if !got_packet {
        println!("dont have packet");
        // timeout → back to standby
        let _ = exec(
            radio,
            busy,
            delay,
            SetStandby {
                config: StandbyConfig::Rc,
            },
        );
        return Ok(None);
    }

    println!("rie2 / have pakcet");
    // Packet indicated by DIO1.
    // We avoid GetRxBufferStatus (can hang). For testing, read a fixed max length
    // from offset 0. Better: both sides use fixed payload length.
    let mut buf = [0u8; 64];
    wait_busy(busy, delay);
    radio.read_buffer(0, &mut buf)?;
    wait_busy(busy, delay);

    println!("rie3");
    // Return to standby
    let _ = exec(
        radio,
        busy,
        delay,
        SetStandby {
            config: StandbyConfig::Rc,
        },
    );

    Ok(Some(buf))
}
fn transmit<SPI: embedded_hal::spi::SpiDevice>(
    radio: &mut Device<SPI>,
    busy: &Input,
    delay: &mut Delay,
    data: &[u8],
) -> Result<(), sx1262::Error> {
    exec(
        radio,
        busy,
        delay,
        ClearIrqStatus {
            irq_mask: IrqMask::all(),
        },
    )?;
    esp_println::println!("1. IRQs cleared");
    wait_busy(busy, delay);
    // Write payload into the radio buffer
    radio.write_buffer(0, data)?;
    wait_busy(busy, delay);
    esp_println::println!("2. write_buffer OK");

    // Update packet length
    exec(
        radio,
        busy,
        delay,
        SetPacketParams {
            params: PacketParams {
                params: [0, 8, 0, data.len() as u8, 1, 0, 0, 0, 0],
            },
        },
    )?;
    esp_println::println!("3. SetPacketParams OK");

    // Start transmission (timeout = 0 → no timeout)
    exec(
        radio,
        busy,
        delay,
        SetTx {
            timeout: Timeout(0),
        },
    )?;
    delay.delay_ms(300);
    wait_busy(busy, delay);
    radio.execute_command(SetTx {
        timeout: Timeout(0),
    })?;
    //let status = exec(radio, busy, delay, GetStatus)?;
    //esp_println::println!("Chip status after SetTx: {:?}", status);
    //esp_println::println!("4. SetTx OK — waiting for TxDone...");

    // Wait for TxDone (simple polling version)
    // In real code you should use DIO1 interrupt
    for i in 0..1000 {
        esp_println::println!("before GetIrqStatus");
        let irq = exec(radio, busy, delay, GetIrqStatus)?;
        esp_println::println!("GetIrqStatus result: {:?}", irq);
        if i % 100 == 0 {
            esp_println::println!("   irq = {:04X}", irq.irq_mask.bits());
        }
        if irq.irq_mask.contains(IrqMask::TX_DONE) {
            // Clear the flag
            exec(
                radio,
                busy,
                delay,
                ClearIrqStatus {
                    irq_mask: IrqMask::TX_DONE,
                },
            )?;
            esp_println::println!("5. TxDone!");
            return Ok(());
        }
        if irq.irq_mask.contains(IrqMask::TIMEOUT) {
            esp_println::println!("TIMEOUT flag set");
            return Err(sx1262::Error::BusError);
        }
        delay.delay_ms(1);
    }

    esp_println::println!("No TxDone after 1 second");
    Err(sx1262::Error::BusError) // timeout
}
fn configure_lora<SPI: embedded_hal::spi::SpiDevice>(
    radio: &mut Device<SPI>,
    busy: &Input,
    delay: &mut Delay,
) -> Result<(), sx1262::Error> {
    // Must be in STDBY_RC for configuration
    exec(
        radio,
        busy,
        delay,
        SetStandby {
            config: StandbyConfig::Rc,
        },
    )?;
    exec(
        radio,
        busy,
        delay,
        SetDio3AsTcxoCtrl {
            config: TcxoConfig {
                voltage: TcxoVoltage::V1_8, // try Volt3_3 if 1.8 fails
                delay: 5_00,                // ~5 ms in 15.625 µs steps (adjust if needed)
            },
        },
    )?;
    esp_println::println!("TCXO enabled");

    // Give it time to start
    delay.delay_ms(10);

    // 1. Packet type MUST be first
    exec(
        radio,
        busy,
        delay,
        SetPacketType {
            packet_type: PacketType::LoRa,
        },
    )?;

    exec(
        radio,
        busy,
        delay,
        sx1262::CalibrateImage {
            config: sx1262::ImageCalibConfig {
                freq1: 0xE1,
                freq2: 0xE9,
            },
        },
    )?;
    esp_println::println!("Image calibration done");

    // 2. Frequency (example: 868.1 MHz)
    exec(
        radio,
        busy,
        delay,
        SetRfFrequency {
            config: RfFrequencyConfig {
                frequency: 915_200_000,
            },
        },
    )?;

    // 3. Modulation parameters (SF7, BW125, CR4/5 is a good starting point)
    exec(
        radio,
        busy,
        delay,
        SetModulationParams {
            params: ModulationParams::LoRa(LoRaModParams {
                spreading_factor: SpreadingFactor::SF7,
                bandwidth: LoRaBandwidth::Bw117,
                coding_rate: CodingRate::Cr45,
                low_data_rate_opt: false,
            }),
        },
    )?;

    // 4. Packet parameters
    exec(
        radio,
        busy,
        delay,
        SetPacketParams {
            params: PacketParams {
                params: [0, 8, 0, 64, 1, 0, 0, 0, 0],
            },
        },
    )?;

    // 5. Buffer base addresses
    exec(
        radio,
        busy,
        delay,
        SetBufferBaseAddress {
            config: BufferBaseAddressConfig {
                tx_base_addr: 0x00,
                rx_base_addr: 0x00,
            },
        },
    )?;

    // 6. PA + TX power (SX1262, +14 dBm is safe for most modules)
    exec(
        radio,
        busy,
        delay,
        SetPaConfig {
            config: PaConfig {
                duty_cycle: 0x04,
                hp_max: 0x07,
                device_sel: DeviceSelect::Sx1262,
                pa_lut: 0x01,
            },
        },
    )?;

    exec(
        radio,
        busy,
        delay,
        SetDioIrqParams {
            config: DioIrqConfig {
                irq_mask: IrqMask::all(),
                dio1_mask: IrqMask::RX_DONE | IrqMask::TX_DONE | IrqMask::TIMEOUT,
                dio2_mask: IrqMask::empty(),
                dio3_mask: IrqMask::empty(),
            },
        },
    )?;

    exec(
        radio,
        busy,
        delay,
        SetTxParams {
            params: TxParams {
                power: 14, // dBm
                ramp_time: RampTime::Micros40,
            },
        },
    )?;

    // Optional but recommended: DIO2 as RF switch (most modules need this)
    exec(
        radio,
        busy,
        delay,
        SetDio2AsRfSwitchCtrl {
            config: RfSwitchConfig { enable: true },
        },
    )?;
    //let errors = exec(radio, busy, delay, sx1262::GetDeviceErrors)?;
    //esp_println::println!("Device errors: {:?}", errors);

    // Clear them
    //exec(radio, busy, delay, sx1262::ClearDeviceErrors)?;
    Ok(())
}
fn receive<SPI: embedded_hal::spi::SpiDevice>(
    radio: &mut Device<SPI>,
    busy: &Input,
    delay: &mut Delay,
    timeout_ms: u32,
) -> Result<Option<[u8; 64]>, sx1262::Error> {
    exec(
        radio,
        busy,
        delay,
        SetRx {
            mode: sx1262::RxMode::Timed(0x00_FF_FF_FF),
        },
    )?;
    // Poll for RxDone
    for _ in 0..timeout_ms {
        let irq = exec(radio, busy, delay, GetIrqStatus)?;

        if irq.irq_mask.contains(IrqMask::RX_DONE) {
            // Clear flag
            exec(
                radio,
                busy,
                delay,
                ClearIrqStatus {
                    irq_mask: IrqMask::RX_DONE | IrqMask::TIMEOUT,
                },
            )?;

            // Get how many bytes arrived and where they are
            let status = exec(radio, busy, delay, GetRxBufferStatus)?;

            let mut buf = [0u8; 64];
            let len = status.buffer_status.payload_length as usize;

            radio.read_buffer(status.buffer_status.buffer_pointer, &mut buf[..len])?;
            wait_busy(busy, delay);

            return Ok(Some(buf));
        }

        delay.delay_ms(1);
    }

    // Timeout – put radio back to standby
    exec(
        radio,
        busy,
        delay,
        SetStandby {
            config: StandbyConfig::Rc,
        },
    )?;

    Ok(None)
}

/// Wait until BUSY pin goes low
fn wait_busy(busy: &Input, delay: &mut Delay) {
    // Timeout after ~100 ms
    for _ in 0..1000 {
        if busy.is_low() {
            return;
        }
        delay.delay_us(100);
    }
    panic!("SX1262 BUSY timeout");
}

fn exec<SPI, C>(
    radio: &mut Device<SPI>,
    busy: &Input,
    delay: &mut Delay,
    cmd: C,
) -> Result<C::ResponseParameters, sx1262::Error>
where
    SPI: embedded_hal::spi::SpiDevice,
    C: regiface::Command<IdType = u8>,
    C::CommandParameters: regiface::ToByteArray<Error = core::convert::Infallible>,
{
    wait_busy(busy, delay);
    let res = radio.execute_command(cmd)?;
    wait_busy(busy, delay);
    Ok(res)
}
