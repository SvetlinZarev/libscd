use crate::internal::common::opcode_with_data_into_payload;
use crate::measurement::Measurement;
use core::ops::Range;

// Section 1.1.1
pub const I2C_ADDRESS: u8 = 0x61;

// Section 1.1.2.
// The datasheet is ambiguous whether the driver should wait after each write
// command. For some commands (1.4.4-GetDataReady, 1.4.5-DataMeasurement)
// it's explicitly specified that the implementations must wait at least 3ms
// before reading the response. For other commands, such as 1.4.6-FRC/ASC, it
// is not explicitly specified, but then it would contradict the diagram
// at 1.1.2. So take the safer route and always perform a delay after a write
// command
pub const WRITE_DELAY_MILLIS: u32 = 5;

// Section 1.1. Boot delay is at most 2s.
pub const BOOT_DELAY_MILLIS: u32 = 2_000;

// Section 1.4.1
pub const AMBIENT_PRESSURE_DISABLE_COMPENSATION: u16 = 0;
pub const AMBIENT_PRESSURE_RANGE_HPA: Range<u16> = 700..1401;

// Section 1.4.3
pub const MEASUREMENT_INTERVAL_RANGE: Range<u16> = 2..1801;

// Section 1.4.6
pub const FRC_PPM_RANGE: Range<u16> = 400..2001;

pub const START_CONTINUOUS_MEASUREMENT: Command = Command(0x0010);
pub const STOP_CONTINUOUS_MEASUREMENT: Command = Command(0x0104);
pub const GET_SET_MEASUREMENT_INTERVAL: Command = Command(0x4600);
pub const GET_DATA_READY_STATUS: Command = Command(0x0202);
pub const READ_MEASUREMENT: Command = Command(0x0300);
pub const MANAGE_AUTOMATIC_SELF_CALIBRATION: Command = Command(0x5306);
pub const SET_FORCED_RECALIBRATION_VALUE: Command = Command(0x5204);
pub const GET_SET_TEMPERATURE_OFFSET: Command = Command(0x5403);
pub const GET_SET_ALTITUDE_COMPENSATION: Command = Command(0x5102);
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

pub fn decode_measurement_data(buf: [u8; 18]) -> Measurement {
    let co2 = f32::from_be_bytes([buf[0], buf[1], buf[3], buf[4]]);
    let tmp = f32::from_be_bytes([buf[6], buf[7], buf[9], buf[10]]);
    let hum = f32::from_be_bytes([buf[12], buf[13], buf[15], buf[16]]);

    Measurement {
        temperature: tmp,
        humidity: hum,
        co2: co2 as u16,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const F32_TOLERANCE: f32 = 0.05;

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

    #[test]
    fn test_decode_measurement_data() {
        const EXPECTED_HUMIDITY: f32 = 48.8;
        const EXPECTED_TEMPERATURE: f32 = 27.2;

        let buf = [
            0x43, 0xDB, 0xCB, // CO2: MMSB, MLSB, CRC
            0x8C, 0x2E, 0x8F, // CO2: LMSB, LLSB, CRC
            0x41, 0xD9, 0x70, // TMP: MMSB, MLSB, CRC
            0xE7, 0xFF, 0xF5, // TMP: LMSB, LLSB, CRC
            0x42, 0x43, 0xBF, // RH%: MMSB, MLSB, CRC
            0x3A, 0x1B, 0x74, // RH%: LMSB, LLSB, CRC
        ];

        let m = decode_measurement_data(buf);
        assert_eq!(439, m.co2);
        assert!(
            (EXPECTED_HUMIDITY - m.humidity).abs() < F32_TOLERANCE,
            "Expected: {}; Actual: {}",
            EXPECTED_HUMIDITY,
            m.humidity
        );
        assert!(
            (EXPECTED_TEMPERATURE - m.temperature).abs() < F32_TOLERANCE,
            "Expected: {}; Actual: {}",
            EXPECTED_TEMPERATURE,
            m.temperature
        );
    }
}
