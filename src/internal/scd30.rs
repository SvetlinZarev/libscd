use crate::internal::crc::crc8;

pub const I2C_ADDRESS: u8 = 0x61;

pub const READ_DELAY_MS: u32 = 3;

pub const START_CONTINUOUS_MEASUREMENT: Command = Command(0x0010);
pub const STOP_CONTINUOUS_MEASUREMENT: Command = Command(0x0104);
pub const GET_SET_MEASUREMENT_INTERVAL: Command = Command(0x4600);
pub const GET_DATA_READY_STATUS: Command = Command(0x0202);
pub const READ_MEASUREMENT: Command = Command(0x0300);
pub const MANAGE_AUTOMATIC_SELF_CALIBRATION: Command = Command(0x5306);
pub const SET_FORCED_RECALIBRATION_VALUE: Command = Command(0x5204);
pub const GET_SET_TEMPERATURE_OFFSET: Command = Command(0x5403);
pub const SET_ALTITUDE_COMPENSATION: Command = Command(0x5102);
pub const READ_FIRMWARE_VERSION: Command = Command(0xD100);
pub const SOFT_RESET: Command = Command(0xD304);

#[derive(Copy, Clone)]
pub struct Command(u16);

impl Command {
    pub fn to_be_bytes(self) -> [u8; 2] {
        self.0.to_be_bytes()
    }
}

pub fn command_with_data_to_payload(cmd: Command, data: u16) -> [u8; 5] {
    let c = cmd.to_be_bytes();
    let d = data.to_be_bytes();

    let mut buf = [0; 5];
    buf[0..2].copy_from_slice(&c);
    buf[2..4].copy_from_slice(&d);
    buf[4] = crc8(&d);

    buf
}

pub fn assert_valid_read_buf_len(read_buf: &[u8]) {
    assert_eq!(
        read_buf.len() % 3,
        0,
        "The read buffer length must be a multiple of 3"
    );
}
