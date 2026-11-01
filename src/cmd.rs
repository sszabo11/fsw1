#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

#[derive(Debug, Clone, Copy)]
pub enum ControlCommand {
    Bank(f32),
    Pitch(f32),
    Yaw(f32),
    Throttle(f32),
    None,
}

pub fn decode_command(raw: &[u8]) -> Option<ControlCommand> {
    //println!("b: {:?}", raw);
    let s = str::from_utf8(raw).ok()?.trim();
    if s.is_empty() {
        return Some(ControlCommand::None);
    };
    //println!("decode: {}", s);

    let mut parts = s.split_whitespace();
    let key = parts.next()?;
    let val_str = parts.next()?;
    let val: f32 = val_str.parse().ok()?;

    //println!("key: {} | val: {}", key, val);
    let cmd = match key.trim() {
        "BANK" => ControlCommand::Bank(val),
        "PITCH" => ControlCommand::Pitch(val),
        "YAW" => ControlCommand::Yaw(val),
        "THROTTLE" => ControlCommand::Throttle(val),
        _ => ControlCommand::None,
    };

    Some(cmd)
}

fn package_command(cmd: &ControlCommand) -> &[u8] {
    match cmd {
        ControlCommand::None => "".as_bytes(),
        ControlCommand::Bank(val) => "BANK {}".as_bytes(),
        ControlCommand::Throttle(val) => "THROTTLE {}".as_bytes(),
        ControlCommand::Pitch(val) => "PITCH {}".as_bytes(),
        _ => "".as_bytes(),
    }
}
