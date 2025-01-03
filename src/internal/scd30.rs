use crate::internal::common::opcode_with_data_into_payload;
use core::ops::Range;

pub const I2C_ADDRESS: u8 = 0x61;
pub const WRITE_DELAY_MILLIS: u32 = 5;
pub const AMBIENT_PRESSURE_DISABLE_COMPENSATION: u16 = 0;
pub const AMBIENT_PRESSURE_RANGE_HPA: Range<u16> = 700..1401;
pub const MEASUREMENT_INTERVAL_RANGE: Range<u16> = 2..1801;

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
    pub const fn prepare(self) -> [u8; 2] {
        self.0.to_be_bytes()
    }

    pub const fn prepare_with_data(self, data: u16) -> [u8; 5] {
        opcode_with_data_into_payload(self.0, data)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_prepare_command() {
        assert_eq!([0x00, 0x10], START_CONTINUOUS_MEASUREMENT.prepare());
    }

    #[test]
    fn test_prepare_command_with_data() {
        assert_eq!(
            [0x54, 0x03, 0x01, 0xF4, 0x33],
            GET_SET_TEMPERATURE_OFFSET.prepare_with_data(0x01F4)
        );
    }
}
