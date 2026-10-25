#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use defmt_rtt as _;
use embedded_hal::delay::DelayNs;
use embedded_hal::spi::{Operation, SpiDevice};
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull};
use esp_hal::main;
use esp_hal::spi::Mode;
use esp_hal::spi::master::{Config, Spi};
use esp_hal::time::Rate;
use esp_println as _;
use esp_println::logger::init_logger;
use esp_println::println;
use sx1262::{
    BufferBaseAddressConfig, ClearIrqStatus, CodingRate, Device, DeviceSelect, DioIrqConfig,
    GetIrqStatus, GetRxBufferStatus, IrqMask, LoRaBandwidth, LoRaModParams, ModulationParams,
    PaConfig, PacketParams, PacketType, RampTime, RfFrequencyConfig, RfSwitchConfig,
    SetBufferBaseAddress, SetDio2AsRfSwitchCtrl, SetDio3AsTcxoCtrl, SetDioIrqParams,
    SetModulationParams, SetPaConfig, SetPacketParams, SetPacketType, SetRfFrequency, SetRx,
    SetStandby, SetTx, SetTxParams, SpreadingFactor, StandbyConfig, TcxoConfig, TcxoVoltage,
    Timeout, TxParams,
};

esp_bootloader_esp_idf::esp_app_desc!();

// ====================== SETTINGS ======================
/// Flash one board with `true` (sender) and the other with `false` (receiver).
const IS_TX: bool = true;

/// Both boards MUST use the same frequency.
const FREQUENCY_HZ: u32 = 915_200_000;

/// Fixed payload length used by both sides (explicit header, so RX reads the real length anyway).
const PAYLOAD_LEN: u8 = 16;

// Raw IRQ bit masks (SX126x datasheet, table "IRQ registers")
const IRQ_TX_DONE: u16 = 0x0001;
const IRQ_RX_DONE: u16 = 0x0002;
const IRQ_PREAMBLE: u16 = 0x0004;
const IRQ_SYNC_WORD: u16 = 0x0008;
const IRQ_HEADER_VALID: u16 = 0x0010;
const IRQ_HEADER_ERR: u16 = 0x0020;
const IRQ_CRC_ERR: u16 = 0x0040;
const IRQ_TIMEOUT: u16 = 0x0200;
// ======================================================

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

    // --- Pins ---
    let nss = Output::new(peripherals.GPIO18, Level::High, OutputConfig::default());
    let mut reset = Output::new(peripherals.GPIO23, Level::High, OutputConfig::default());
    let busy = Input::new(
        peripherals.GPIO25,
        InputConfig::default().with_pull(Pull::None),
    );
    // DIO1 is only printed for information. The code below does NOT depend on it.
    let dio1 = Input::new(
        peripherals.GPIO22,
        InputConfig::default().with_pull(Pull::None),
    );

    // If your module has separate RXEN / TXEN pins (e.g. Ebyte E22), wire them to
    // free GPIOs and drive them here. Example (change the pins to yours):
    //   let mut rxen = Output::new(peripherals.GPIO26, Level::High, OutputConfig::default());
    //   let mut txen = Output::new(peripherals.GPIO27, Level::Low,  OutputConfig::default());
    // RX: rxen HIGH, txen LOW.  TX: rxen LOW, txen HIGH.

    // --- SPI bus ---
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
    let mut spi_dev = ExclusiveDevice::new(spi, nss, delay).unwrap();

    // --- Hardware reset ---
    reset.set_low();
    delay.delay_ms(10);
    reset.set_high();
    delay.delay_ms(20);
    wait_busy(&busy, &mut delay);

    // --- Step 1: prove SPI READS work (raw ReadRegister 0x1D, address 0x0740) ---
    // Default LoRa sync word is 0x1424 (private network). Anything else means a
    // wiring problem (MISO / NSS / SCK / power) and nothing else can work yet.
    let sw = read_sync_word(&mut spi_dev, &busy, &mut delay);
    println!(
        "Sync word register = {:02X} {:02X}  (expect 14 24)",
        sw[0], sw[1]
    );
    if sw == [0x00, 0x00] || sw == [0xFF, 0xFF] {
        println!("!! SPI read looks dead: check MISO, NSS, SCK wiring and 3.3V supply");
    }

    // --- Create SX1262 device ---
    let mut radio = Device::new(spi_dev);

    configure_lora(&mut radio, &busy, &mut delay).expect("Config failed");
    println!("SX1262 ready, mode = {}", if IS_TX { "TX" } else { "RX" });

    if IS_TX {
        let mut n = 0u32;
        loop {
            let msg = b"Hello ESP32!!!!!"; // exactly 16 bytes
            match transmit(&mut radio, &busy, &mut delay, msg) {
                Ok(()) => println!("TX OK #{}", n),
                Err(e) => println!("TX FAILED #{}: {:?}", n, e),
            }
            n = n.wrapping_add(1);
            delay.delay_ms(1000);
        }
    } else {
        loop {
            match receive(&mut radio, &busy, &dio1, &mut delay, 15_000) {
                Ok(Some((buf, len))) => {
                    println!("RX {} bytes: {:?}", len, &buf[..len]);
                    println!("RX text: {:?}", core::str::from_utf8(&buf[..len]));
                }
                Ok(None) => println!("RX timeout (nothing valid received)"),
                Err(e) => println!("RX err: {:?}", e),
            }
        }
    }
}

/// Raw SPI read of the 2-byte LoRa sync word register (0x0740).
fn read_sync_word<SPI: SpiDevice>(spi: &mut SPI, busy: &Input, delay: &mut Delay) -> [u8; 2] {
    wait_busy(busy, delay);
    let mut out = [0u8; 2];
    // ReadRegister: opcode 0x1D, addr hi, addr lo, NOP (status), then data bytes.
    let res = spi.transaction(&mut [
        Operation::Write(&[0x1D, 0x07, 0x40, 0x00]),
        Operation::Read(&mut out),
    ]);
    if res.is_err() {
        println!("SPI transaction error");
    }
    wait_busy(busy, delay);
    out
}

fn configure_lora<SPI: SpiDevice>(
    radio: &mut Device<SPI>,
    busy: &Input,
    delay: &mut Delay,
) -> Result<(), sx1262::Error> {
    exec(
        radio,
        busy,
        delay,
        SetStandby {
            config: StandbyConfig::Rc,
        },
    )?;

    // TCXO via DIO3. REMOVE this command if your module has a plain crystal (no TCXO).
    // Try V3_3 if 1.8 V doesn't work (check your module's datasheet).
    exec(
        radio,
        busy,
        delay,
        SetDio3AsTcxoCtrl {
            config: TcxoConfig {
                voltage: TcxoVoltage::V1_8,
                delay: 5_00,
            },
        },
    )?;
    println!("TCXO enabled");
    delay.delay_ms(10);

    // Packet type must come before modulation / packet params.
    exec(
        radio,
        busy,
        delay,
        SetPacketType {
            packet_type: PacketType::LoRa,
        },
    )?;

    // Image calibration for the 902-928 MHz band.
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
    println!("Image calibration done");

    exec(
        radio,
        busy,
        delay,
        SetRfFrequency {
            config: RfFrequencyConfig {
                frequency: FREQUENCY_HZ,
            },
        },
    )?;

    // NOTE: SX126x LoRa bandwidths are 7.81 ... 125, 250, 500 kHz. Check the crate's enum
    // and use the variant that maps to 125 kHz (register value 0x04). Both boards must match.
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

    // [preamble_hi, preamble_lo, header_type (0 = explicit), payload_len, crc_on, iq_std, ...]
    exec(
        radio,
        busy,
        delay,
        SetPacketParams {
            params: PacketParams {
                params: [0, 8, 0, PAYLOAD_LEN, 1, 0, 0, 0, 0],
            },
        },
    )?;

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

    // Enable ALL IRQs so we can see preamble / sync / header / CRC events while debugging.
    exec(
        radio,
        busy,
        delay,
        SetDioIrqParams {
            config: DioIrqConfig {
                irq_mask: IrqMask::all(),
                dio1_mask: IrqMask::all(),
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
                power: 14,
                ramp_time: RampTime::Micros40,
            },
        },
    )?;

    // Only works if the module wires DIO2 to the RF switch. If it has RXEN/TXEN pins,
    // you must drive those from the MCU instead (see main()).
    exec(
        radio,
        busy,
        delay,
        SetDio2AsRfSwitchCtrl {
            config: RfSwitchConfig { enable: true },
        },
    )?;

    Ok(())
}

/// Transmit one packet and wait for TX_DONE by polling the IRQ register over SPI.
fn transmit<SPI: SpiDevice>(
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

    wait_busy(busy, delay);
    radio.write_buffer(0, data)?;
    wait_busy(busy, delay);

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

    // Timeout(0) = no timeout. TX_DONE ends the transmission.
    exec(
        radio,
        busy,
        delay,
        SetTx {
            timeout: Timeout(0),
        },
    )?;

    // 16 bytes at SF7 / 125 kHz is roughly 50 ms of airtime. Allow 2 s.
    for ms in 0..2000u32 {
        println!("E1");
        let irq = exec(radio, busy, delay, GetIrqStatus)?;
        println!("E2");
        let bits = irq.irq_mask.bits();
        if bits & IRQ_TX_DONE != 0 {
            exec(
                radio,
                busy,
                delay,
                ClearIrqStatus {
                    irq_mask: IrqMask::all(),
                },
            )?;
            println!("TX_DONE after {} ms (IRQ={:#06X})", ms, bits);
            return Ok(());
        }
        if bits & IRQ_TIMEOUT != 0 {
            println!("TX timeout flag (IRQ={:#06X})", bits);
            return Err(sx1262::Error::BusError);
        }
        delay.delay_ms(1);
    }

    println!("No TX_DONE within 2 s");
    Err(sx1262::Error::BusError)
}

/// Receive for up to `timeout_ms`. Polls the IRQ register over SPI (does not rely on DIO1)
/// and prints every event so you can see how far a packet gets.
fn receive<SPI: SpiDevice>(
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
            mode: sx1262::RxMode::Timed(0x00_FF_FF_FF), // 0xFFFFFF = continuous RX
        },
    )?;
    println!("RX started (DIO1 pin now = {})", dio1.is_high());

    for ms in 0..timeout_ms {
        let irq = exec(radio, busy, delay, GetIrqStatus)?;
        let bits = irq.irq_mask.bits();

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

            exec(
                radio,
                busy,
                delay,
                ClearIrqStatus {
                    irq_mask: IrqMask::all(),
                },
            )?;

            // Good packet: RX_DONE set and no CRC / header error.
            if bits & IRQ_RX_DONE != 0 && bits & (IRQ_CRC_ERR | IRQ_HEADER_ERR) == 0 {
                let st = exec(radio, busy, delay, GetRxBufferStatus)?;
                let len = (st.buffer_status.payload_length as usize).min(64);
                let mut buf = [0u8; 64];
                wait_busy(busy, delay);
                radio.read_buffer(st.buffer_status.buffer_pointer, &mut buf[..len])?;
                wait_busy(busy, delay);
                return Ok(Some((buf, len)));
            }
        }

        delay.delay_ms(1);
    }

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

/// Wait until BUSY goes low (timeout ~1 s).
fn wait_busy(busy: &Input, delay: &mut Delay) {
    for _ in 0..10_000 {
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
    SPI: SpiDevice,
    C: regiface::Command<IdType = u8>,
    C::CommandParameters: regiface::ToByteArray<Error = core::convert::Infallible>,
{
    wait_busy(busy, delay);
    let res = radio.execute_command(cmd)?;
    wait_busy(busy, delay);
    Ok(res)
}
