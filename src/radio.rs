#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use crate::{
    data::{PayloadData, decode_packet, encode_packet},
    rx::listen,
    tx::transmit,
    utils::{
        IRQ_CRC_ERR, IRQ_HEADER_ERR, IRQ_RX_DONE, LoraConfig, clear_irq, get_irq, read_buffer,
        read_cmd, write_cmd,
    },
};
use defmt_rtt as _;
use embedded_hal::{delay::DelayNs, spi::SpiDevice};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{delay::Delay, gpio::Input};
use esp_println::println;
use serde::Deserialize;

pub struct Radio<'a, S: SpiDevice> {
    lora: LoraConfig<'a, S>,
}

impl<'a, S: SpiDevice> Radio<'a, S> {
    pub fn new(spi: S, busy: Input<'a>, delay: Delay) -> Self {
        Self {
            lora: LoraConfig { spi, busy, delay },
        }
    }

    pub fn receive(&mut self, timeout_ms: u32) -> PayloadData {
        let res = listen(&mut self.lora, timeout_ms);

        if let Some(raw) = res {
            if let Ok(data) = decode_packet(raw) {
                //println!("data: {:?}", data);
                return data;
            } else {
                println!("inv from rec");
                return PayloadData::Invalid;
            }
        };
        PayloadData::Timeout
    }
    pub fn send(&mut self, data: PayloadData) -> bool {
        let packet = encode_packet(data);
        //println!("pa: {:?}", packet);
        let sent = transmit(&mut self.lora, &packet);

        sent
    }
}
