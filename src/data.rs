use esp_println::println;
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug)]
pub enum PayloadData {
    Telemetry(TelemetryPacket),
    Control(ControlPacket),
    None,
    Invalid,
}

//impl TryFrom<PayloadError> for PayloadData {
//    type Error = PayloadError;
//
//    fn try_from(raw: ([u8; 64], usize)) -> Result<Self, Self::Error> {
//        println!("{:?}", raw);
//        Ok(PayloadData::None)
//    }
//}
//impl TryFrom<RawEnumInput> for PayloadData {
//    type Error = MyTryFromError;
//
//    fn try_from(raw: RawEnumInput) -> Result<Self, Self::Error> {
//        match raw {
//            RawEnumInput::KnownVariant => Ok(MyEnum::First),
//        }
//    }
//}

#[derive(Serialize, Deserialize, Debug)]
pub struct TelemetryPacket {
    altitude: f32,
    speed: f32,
    seq: u8,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct ControlPacket {
    pub seq: u8,
    pub bank: f32,
    pub pitch: f32,
    pub throttle: f32,
    pub yaw: f32,
}

pub fn encode_packet(p: PayloadData) -> [u8; 8] {
    match p {
        PayloadData::Control(data) => {
            let mut p = [0u8; 8];

            p[0] = 0xC1;
            p[1] = 0;
            p[2] = data.seq;
            // Control packet
            let bank = p[3..5].copy_from_slice(&((data.bank * 10.) as u8).to_le_bytes());
            let pitch = p[5..7].copy_from_slice(&((data.pitch * 10.) as u8).to_le_bytes());
            p
        }
        PayloadData::Telemetry(_) => {
            // Telemetry packet
            let mut p = [0u8; 8];
            p[0] = 0xC1;
            p[1] = 1;
            p
        }
        _ => {
            let mut p = [0u8; 8];
            p[0] = 0xC1;
            p[1] = 3;
            p
        }
    }
}

pub fn decode_packet(p: [u8; 8]) -> Result<PayloadData, PayloadData> {
    if p.len() < 8 || p[0] != 0xC1 {
        return Ok(PayloadData::None);
    }

    let kind = p[1]; // 0, 1
    println!("kind: {}", kind);

    match kind {
        0 => {
            // Control packet
            let seq = p[2];
            let bank = i16::from_le_bytes([p[3], p[4]]);
            let pitch = i16::from_le_bytes([p[5], p[6]]);
            Ok(PayloadData::Control(ControlPacket {
                seq,
                bank: bank as f32 / 10.,
                pitch: pitch as f32 / 10.,
                throttle: 0.,
                yaw: 0.,
            }))
        }
        1 => {
            // Telemetry packet

            Ok(PayloadData::Invalid)
        }
        _ => Err(PayloadData::Invalid),
    }
}
