use crate::internal::common::opcode_with_data_into_payload;
use core::ops::Range;

pub const I2C_ADDRESS: u8 = 0x62;
pub const MAX_ALTITUDE: u16 = 3_000;
pub const AMBIENT_PRESSURE_RANGE_PA: Range<u32> = 70_000..120_001;

// Section 3.8.1 from the datasheet
// A return value of 0xFFFF indicates that the FRC has failed
// because the sensor was not operated before sending the command.
pub const FRC_FAILED: u16 = 0xFFFF;

pub const START_PERIODIC_MEASUREMENT: Command = Command::new(0x21b1, 0, false);
pub const START_LOW_POWER_PERIODIC_MEASUREMENT: Command = Command::new(0x21ac, 0, false);
pub const STOP_PERIODIC_MEASUREMENT: Command = Command::new(0x3f86, 500, true);

pub const GET_DATA_READY_STATUS: Command = Command::new(0xe4b8, 1, true);
pub const READ_MEASUREMENT: Command = Command::new(0xec05, 1, true);

pub const SET_TEMPERATURE_OFFSET: Command = Command::new(0x241d, 1, false);
pub const GET_TEMPERATURE_OFFSET: Command = Command::new(0x2318, 1, false);

pub const SET_SENSOR_ALTITUDE: Command = Command::new(0x2427, 1, false);
pub const GET_SENSOR_ALTITUDE: Command = Command::new(0x2322, 1, false);

pub const SET_AMBIENT_PRESSURE: Command = Command::new(0xe000, 1, true);
pub const GET_AMBIENT_PRESSURE: Command = Command::new(0xe000, 1, true);

pub const SET_AUTOMATIC_SELF_CALIBRATION_ENABLED: Command = Command::new(0x2416, 1, false);
pub const GET_AUTOMATIC_SELF_CALIBRATION_ENABLED: Command = Command::new(0x2313, 1, false);

pub const SET_AUTOMATIC_SELF_CALIBRATION_TARGET: Command = Command::new(0x243a, 1, false);
pub const GET_AUTOMATIC_SELF_CALIBRATION_TARGET: Command = Command::new(0x233f, 1, false);
pub const PERFORM_FORCED_RECALIBRATION: Command = Command::new(0x362f, 400, false);

pub const PERSIST_SETTINGS: Command = Command::new(0x3615, 800, false);
pub const GET_SERIAL_NUMBER: Command = Command::new(0x3682, 1, false);

pub const PERFORM_SELF_TEST: Command = Command::new(0x3639, 10_000, false);
pub const PERFORM_FACTORY_RESET: Command = Command::new(0x3632, 1_200, false);
pub const REINIT: Command = Command::new(0x3646, 30, false);

#[cfg(feature = "scd41")]
pub const MEASURE_SINGLE_SHOT: Command = Command::new(0x219d, 5_000, false);

#[cfg(feature = "scd41")]
pub const MEASURE_SINGLE_SHOT_RHT_ONLY: Command = Command::new(0x2196, 50, false);

#[cfg(feature = "scd41")]
pub const POWER_DOWN: Command = Command::new(0x36e0, 1, false);

#[cfg(feature = "scd41")]
pub const WAKE_UP: Command = Command::new(0x36f6, 30, false);

#[cfg(feature = "scd41")]
pub const SET_AUTOMATIC_SELF_CALIBRATION_INITIAL_PERIOD: Command = Command::new(0x2445, 1, false);

#[cfg(feature = "scd41")]
pub const GET_AUTOMATIC_SELF_CALIBRATION_INITIAL_PERIOD: Command = Command::new(0x2340, 1, false);

#[cfg(feature = "scd41")]
pub const SET_AUTOMATIC_SELF_CALIBRATION_STANDARD_PERIOD: Command = Command::new(0x244e, 1, false);

#[cfg(feature = "scd41")]
pub const GET_AUTOMATIC_SELF_CALIBRATION_STANDARD_PERIOD: Command = Command::new(0x234b, 1, false);

#[derive(Copy, Clone)]
pub struct Command {
    pub op_code: u16,
    pub exec_time: u16,
    pub allowed_while_running: bool,
}

impl Command {
    const fn new(op_code: u16, exec_time: u16, allowed_while_running: bool) -> Self {
        Self {
            op_code,
            exec_time,
            allowed_while_running,
        }
    }

    pub const fn prepare(self) -> [u8; 2] {
        self.op_code.to_be_bytes()
    }

    pub const fn prepare_with_data(self, data: u16) -> [u8; 5] {
        opcode_with_data_into_payload(self.op_code, data)
    }
}

pub fn decode_serial_number(buf: [u8; 9]) -> u64 {
    u64::from(buf[0]) << 40
        | u64::from(buf[1]) << 32
        | u64::from(buf[3]) << 24
        | u64::from(buf[4]) << 16
        | u64::from(buf[6]) << 8
        | u64::from(buf[7])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_decode_serial_number() {
        let response = [0xF8, 0x96, 0x31, 0x9F, 0x07, 0xC2, 0x3B, 0xBE, 0x89];
        let serial_number = decode_serial_number(response);
        assert_eq!(273_325_796_834_238, serial_number);
    }

    #[test]
    fn test_prepare_command() {
        assert_eq!([0x36, 0x82], GET_SERIAL_NUMBER.prepare());
    }

    #[test]
    fn test_prepare_command_with_data() {
        assert_eq!(
            [0x24, 0x1D, 0x07, 0xE6, 0x48],
            SET_TEMPERATURE_OFFSET.prepare_with_data(0x07E6)
        );
    }
}
