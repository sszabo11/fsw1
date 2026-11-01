#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use defmt_rtt as _;
use embedded_hal::{delay::DelayNs, spi::SpiDevice};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{delay::Delay, gpio::Input};
use esp_println::println;
use serde::Deserialize;

use crate::{
    data::{PayloadData, decode_packet},
    utils::{
        IRQ_CRC_ERR, IRQ_HEADER_ERR, IRQ_RX_DONE, LoraConfig, clear_irq, get_irq, read_buffer,
        read_cmd, write_cmd,
    },
};

pub struct Receiver<'a, S: SpiDevice> {
    lora: LoraConfig<'a, S>,
}

impl<'a, S: SpiDevice> Receiver<'a, S> {
    pub fn new(spi: S, busy: Input<'a>, delay: Delay) -> Self {
        Self {
            lora: LoraConfig { spi, busy, delay },
        }
    }

    pub fn receive(&mut self, timeout_ms: u32) -> PayloadData {
        let res = self.listen(timeout_ms);

        if let Some(raw) = res {
            if let Ok(data) = decode_packet(raw) {
                println!("data: {:?}", data);
                return data;
            } else {
                return PayloadData::Invalid;
            }
        };
        PayloadData::None
    }

    pub fn listen(&mut self, timeout_ms: u32) -> Option<[u8; 8]> {
        let spi = &mut self.lora.spi;
        let busy = &self.lora.busy;
        let delay = &mut self.lora.delay;
        clear_irq(spi, busy, delay);

        // SetRx, 0xFFFFFF = continuous
        write_cmd(spi, busy, delay, 0x82, &[0xFF, 0xFF, 0xFF]);
        //print_status(spi, busy, delay, "right after SetRx (expect RX)");
        //println!("DIO1 pin = {}", dio1.is_high());

        for ms in 0..timeout_ms {
            let bits = get_irq(spi, busy, delay);

            if bits != 0 {
                //println!(
                //    "t={} ms IRQ={:#06X} [{}{}{}{}{}{}{}] DIO1={}",
                //    ms,
                //    bits,
                //    if bits & IRQ_PREAMBLE != 0 {
                //        "PREAMBLE "
                //    } else {
                //        ""
                //    },
                //    if bits & IRQ_SYNC_WORD != 0 {
                //        "SYNC "
                //    } else {
                //        ""
                //    },
                //    if bits & IRQ_HEADER_VALID != 0 {
                //        "HDR_OK "
                //    } else {
                //        ""
                //    },
                //    if bits & IRQ_HEADER_ERR != 0 {
                //        "HDR_ERR "
                //    } else {
                //        ""
                //    },
                //    if bits & IRQ_CRC_ERR != 0 {
                //        "CRC_ERR "
                //    } else {
                //        ""
                //    },
                //    if bits & IRQ_RX_DONE != 0 {
                //        "RX_DONE "
                //    } else {
                //        ""
                //    },
                //    if bits & IRQ_TIMEOUT != 0 {
                //        "TIMEOUT"
                //    } else {
                //        ""
                //    },
                //    dio1.is_high()
                //);
                clear_irq(spi, busy, delay);

                if bits & IRQ_RX_DONE != 0 && bits & (IRQ_CRC_ERR | IRQ_HEADER_ERR) == 0 {
                    // GetRxBufferStatus -> [payload length, start pointer]
                    //let mut st = [0u8; 2];
                    //read_cmd(spi, busy, delay, 0x13, &mut st);
                    //let len = (st[0] as usize).min(64);
                    let mut buf = [0u8; 8];
                    read_buffer(spi, busy, delay, 8, &mut buf);
                    return Some(buf);
                }
            }
            delay.delay_ms(1);
        }

        // Back to STDBY_RC
        write_cmd(spi, busy, delay, 0x80, &[0x00]);
        None
    }
}

pub fn receive<S: SpiDevice>(
    spi: &mut S,
    busy: &Input,
    dio1: &Input,
    delay: &mut Delay,
    timeout_ms: u32,
) -> Option<([u8; 64], usize)> {
    clear_irq(spi, busy, delay);

    // SetRx, 0xFFFFFF = continuous
    write_cmd(spi, busy, delay, 0x82, &[0xFF, 0xFF, 0xFF]);
    //print_status(spi, busy, delay, "right after SetRx (expect RX)");
    //println!("DIO1 pin = {}", dio1.is_high());

    for ms in 0..timeout_ms {
        let bits = get_irq(spi, busy, delay);

        if bits != 0 {
            //println!(
            //    "t={} ms IRQ={:#06X} [{}{}{}{}{}{}{}] DIO1={}",
            //    ms,
            //    bits,
            //    if bits & IRQ_PREAMBLE != 0 {
            //        "PREAMBLE "
            //    } else {
            //        ""
            //    },
            //    if bits & IRQ_SYNC_WORD != 0 {
            //        "SYNC "
            //    } else {
            //        ""
            //    },
            //    if bits & IRQ_HEADER_VALID != 0 {
            //        "HDR_OK "
            //    } else {
            //        ""
            //    },
            //    if bits & IRQ_HEADER_ERR != 0 {
            //        "HDR_ERR "
            //    } else {
            //        ""
            //    },
            //    if bits & IRQ_CRC_ERR != 0 {
            //        "CRC_ERR "
            //    } else {
            //        ""
            //    },
            //    if bits & IRQ_RX_DONE != 0 {
            //        "RX_DONE "
            //    } else {
            //        ""
            //    },
            //    if bits & IRQ_TIMEOUT != 0 {
            //        "TIMEOUT"
            //    } else {
            //        ""
            //    },
            //    dio1.is_high()
            //);
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
