use crate::error::Error;
use crate::internal::common::opcode_with_data_into_payload;
use crate::measurement::Measurement;
use core::ops::Range;

pub const I2C_ADDRESS: u8 = 0x62;

// Section 3.7.3 of teh datasheet
pub const MAX_ALTITUDE: u16 = 3_000;

// Section 3.7.5 of the datasheet
pub const AMBIENT_PRESSURE_RANGE_HPA: Range<u16> = 700..1201;

// Constant used in several data conversions such as in the temperature offset
const TWO_P16_M1: f32 = u16::MAX as f32; // `2.pow(16) - 1`

// Constant used in the temperature data conversion
const TEMP_K1: f32 = 175.0f32;

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

pub fn decode_measurement(buf: [u8; 9]) -> Measurement {
    Measurement {
        temperature: decode_temp_measurement(buf[3], buf[4]),
        humidity: decode_humidity_measurement(buf[6], buf[7]),
        co2: decode_co2_measurement(buf[0], buf[1]),
    }
}

fn decode_temp_measurement(msb: u8, lsb: u8) -> f32 {
    let raw = u16::from_be_bytes([msb, lsb]);
    raw as f32 * TEMP_K1 / TWO_P16_M1 - 45.0
}

fn decode_humidity_measurement(msb: u8, lsb: u8) -> f32 {
    let raw = u16::from_be_bytes([msb, lsb]);
    raw as f32 * 100.0 / TWO_P16_M1
}

fn decode_co2_measurement(msb: u8, lsb: u8) -> u16 {
    u16::from_be_bytes([msb, lsb])
}

pub fn encode_temperature_offset<E>(offset: f32) -> Result<u16, Error<E>> {
    if !offset.is_finite() || offset.is_sign_negative() {
        return Err(Error::InvalidInput);
    }

    Ok((offset * TWO_P16_M1 / TEMP_K1) as u16)
}

pub fn decode_temperature_offset(buf: [u8; 3]) -> f32 {
    let offset = u16::from_be_bytes([buf[0], buf[1]]);
    offset as f32 * TEMP_K1 / TWO_P16_M1
}

pub fn decode_frc_status(buf: [u8; 3]) -> Option<i16> {
    // Section 3.8.1 from the datasheet
    // A return value of 0xFFFF indicates that the FRC has failed
    // because the sensor was not operated before sending the command.
    const FRC_FAILED: u16 = 0xFFFF;

    let result = u16::from_be_bytes([buf[0], buf[1]]);
    if FRC_FAILED == result {
        return None;
    }

    let frc_correction = result as i32 - 0x8000;
    Some(frc_correction as i16)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::internal::crc::crc8;

    const F32_TOLERANCE: f32 = 0.005;

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

    #[test]
    fn test_decode_temperature_offset_1() {
        // Section 3.7.1 of the datasheet
        let raw = [0x07, 0x0E6, 0x48];
        let offset = decode_temperature_offset(raw);
        assert!(offset.is_finite());
        assert!((5.4 - offset).abs() < F32_TOLERANCE);
    }

    #[test]
    fn test_decode_temperature_offset_2() {
        // Section 3.7.2 of the datasheet
        let raw = [0x09, 0x012, 0x63];
        let offset = decode_temperature_offset(raw);
        assert!(offset.is_finite());
        assert!((6.2 - offset).abs() < F32_TOLERANCE);
    }

    #[test]
    fn test_encode_temperature_offset() {
        // Section 3.7.1 of the datasheet
        let word = encode_temperature_offset::<()>(5.4).unwrap();
        assert_eq!(0x07E6, word);
    }

    #[test]
    fn test_encode_decode_temperature_offset() {
        const MIN_OFFSET: f32 = 0.0;
        const MAX_OFFSET: f32 = 20.0;
        const INCREMENT: f32 = 0.01;

        let mut offset = MIN_OFFSET;
        while offset <= MAX_OFFSET {
            let encoded = encode_temperature_offset::<()>(offset)
                .unwrap()
                .to_be_bytes();

            let wire_format = [encoded[0], encoded[1], crc8(&encoded)];
            let decoded = decode_temperature_offset(wire_format);

            assert!(
                (decoded - offset).abs() < F32_TOLERANCE,
                "Offset={}; Decoded={}",
                offset,
                decoded
            );
            offset += INCREMENT;
        }
    }

    #[test]
    fn test_encode_temperature_offset_rejects_negative() {
        assert_eq!(
            Err(Error::InvalidInput),
            encode_temperature_offset::<()>(-1.0)
        );
    }

    #[test]
    fn test_encode_temperature_offset_rejects_nan() {
        assert_eq!(
            Err(Error::InvalidInput),
            encode_temperature_offset::<()>(f32::NAN)
        );
    }

    #[test]
    fn test_decode_temp_measurement() {
        const EXPECTED: f32 = 25.0;

        let decoded = decode_temp_measurement(0x66, 0x67);
        assert!(decoded.is_finite());
        assert!(
            (EXPECTED - decoded).abs() < F32_TOLERANCE,
            "Expected: {}; Decoded: {}",
            EXPECTED,
            decoded
        );
    }

    #[test]
    fn test_decode_co2_measurement() {
        let decoded = decode_co2_measurement(0x01, 0xF4);
        assert_eq!(500, decoded)
    }

    #[test]
    fn test_decode_humidity_measurement() {
        const EXPECTED: f32 = 37.0;

        let decoded = decode_humidity_measurement(0x5E, 0xB9);
        assert!(decoded.is_finite());
        assert!(
            (EXPECTED - decoded).abs() < F32_TOLERANCE,
            "Expected: {}; Decoded: {}",
            EXPECTED,
            decoded
        );
    }

    #[test]
    fn test_decode_measurement() {
        let m = decode_measurement([0x01, 0xF4, 0x33, 0x66, 0x67, 0xA2, 0x5E, 0xB9, 0x3C]);
        assert_eq!(500, m.co2);
        assert!((25.0 - m.temperature).abs() < F32_TOLERANCE);
        assert!((37.0 - m.humidity).abs() < F32_TOLERANCE);
    }

    #[test]
    fn test_decode_frc_status() {
        let status = decode_frc_status([0x7F, 0xCE, 0x7B]);
        assert_eq!(Some(-50), status);
    }

    #[test]
    fn test_decode_frc_status_failed() {
        let status = decode_frc_status([0xFF, 0xFF, crc8(&[0xFF, 0xFF])]);
        assert_eq!(None, status);
    }
}
