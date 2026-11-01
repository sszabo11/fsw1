#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use crate::data::encode_packet;
use crate::utils::{IRQ_TIMEOUT, IRQ_TX_DONE, clear_irq, get_irq, write_buffer, write_cmd};
use crate::{
    data::{PayloadData, decode_packet},
    utils::{IRQ_CRC_ERR, IRQ_HEADER_ERR, IRQ_RX_DONE, LoraConfig, read_buffer, read_cmd},
};
use defmt::println;
use defmt_rtt as _;
use embedded_hal::{delay::DelayNs, spi::SpiDevice};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{delay::Delay, gpio::Input};

pub struct Transmitter<'a, S: SpiDevice> {
    lora: LoraConfig<'a, S>,
}

impl<'a, S: SpiDevice> Transmitter<'a, S> {
    pub fn new(spi: S, busy: Input<'a>, delay: Delay) -> Self {
        Self {
            lora: LoraConfig { spi, busy, delay },
        }
    }

    pub fn send(&mut self, data: PayloadData) -> bool {
        let packet = encode_packet(data);
        let sent = self.transmit(&packet);

        sent
    }

    pub fn transmit(&mut self, data: &[u8]) -> bool {
        let spi = &mut self.lora.spi;
        let busy = &self.lora.busy;
        let delay = &mut self.lora.delay;

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
}
