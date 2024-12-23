use crate::internal::communication::opcode_with_data_into_payload;

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
    opcode_with_data_into_payload(cmd.0, data)
}
