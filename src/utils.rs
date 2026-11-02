#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use embedded_hal::delay::DelayNs;
use embedded_hal::spi::{Operation, SpiDevice};
use esp_hal::{delay::Delay, gpio::Input};
use esp_println::println;

pub const FREQUENCY_HZ: u64 = 915_200_000;
pub const PAYLOAD_LEN: u8 = 16;

pub const IRQ_TX_DONE: u16 = 0x0001;
pub const IRQ_RX_DONE: u16 = 0x0002;
pub const IRQ_PREAMBLE: u16 = 0x0004;
pub const IRQ_SYNC_WORD: u16 = 0x0008;
pub const IRQ_HEADER_VALID: u16 = 0x0010;
pub const IRQ_HEADER_ERR: u16 = 0x0020;
pub const IRQ_CRC_ERR: u16 = 0x0040;
pub const IRQ_TIMEOUT: u16 = 0x0200;

pub fn wait_busy(busy: &Input, delay: &mut Delay) {
    for _ in 0..10_000 {
        if busy.is_low() {
            return;
        }
        delay.delay_us(100);
    }
    panic!("SX1262 BUSY stuck high (check reset/BUSY wiring, TCXO, power)");
}

pub fn write_cmd<S: SpiDevice>(spi: &mut S, busy: &Input, delay: &mut Delay, op: u8, args: &[u8]) {
    wait_busy(busy, delay);
    if spi
        .transaction(&mut [Operation::Write(&[op]), Operation::Write(args)])
        .is_err()
    {
        println!("SPI error on opcode {:#04X}", op);
    }
    wait_busy(busy, delay);
}

pub fn read_register<S: SpiDevice>(
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

pub fn write_buffer<S: SpiDevice>(spi: &mut S, busy: &Input, delay: &mut Delay, data: &[u8]) {
    wait_busy(busy, delay);
    if spi
        .transaction(&mut [Operation::Write(&[0x0E, 0x00]), Operation::Write(data)])
        .is_err()
    {
        println!("SPI error on WriteBuffer");
    }
    wait_busy(busy, delay);
}

pub fn read_cmd<S: SpiDevice>(
    spi: &mut S,
    busy: &Input,
    delay: &mut Delay,
    op: u8,
    out: &mut [u8],
) {
    wait_busy(busy, delay);
    if spi
        .transaction(&mut [Operation::Write(&[op, 0x00]), Operation::Read(out)])
        .is_err()
    {
        println!("SPI error on opcode {:#04X}", op);
    }
    wait_busy(busy, delay);
}
pub fn read_buffer<S: SpiDevice>(
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

pub fn get_status<S: SpiDevice>(spi: &mut S, busy: &Input, delay: &mut Delay) -> u8 {
    wait_busy(busy, delay);
    let mut s = [0u8; 1];
    let _ = spi.transaction(&mut [Operation::Write(&[0xC0]), Operation::Read(&mut s)]);
    wait_busy(busy, delay);
    s[0]
}

pub fn print_status<S: SpiDevice>(spi: &mut S, busy: &Input, delay: &mut Delay, label: &str) {
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

pub fn get_irq<S: SpiDevice>(spi: &mut S, busy: &Input, delay: &mut Delay) -> u16 {
    let mut b = [0u8; 2];
    read_cmd(spi, busy, delay, 0x12, &mut b);
    u16::from_be_bytes(b)
}

pub fn clear_irq<S: SpiDevice>(spi: &mut S, busy: &Input, delay: &mut Delay) {
    write_cmd(spi, busy, delay, 0x02, &[0xFF, 0xFF]);
}

pub fn configure_lora<S: SpiDevice>(spi: &mut S, busy: &Input, delay: &mut Delay) {
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
    //print_status(spi, busy, delay, "after config (expect STDBY_RC)");
}

pub struct LoraConfig<'a, S: SpiDevice> {
    pub spi: S,
    pub busy: Input<'a>,
    pub delay: Delay,
}
