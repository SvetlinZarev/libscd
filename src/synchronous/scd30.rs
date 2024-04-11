use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;

use crate::error::Error;
use crate::internal::crc::{crc8, crc8_verify_chunked_3};
pub use crate::internal::measurement::Measurement;
use crate::internal::scd30::{
    Command, GET_DATA_READY_STATUS, I2C_ADDRESS, MANAGE_AUTOMATIC_SELF_CALIBRATION, READ_DELAY_MS,
    READ_FIRMWARE_VERSION, READ_MEASUREMENT, SET_ALTITUDE_COMPENSATION,
    SET_FORCED_RECALIBRATION_VALUE, SET_MEASUREMENT_INTERVAL, SET_TEMPERATURE_OFFSET, SOFT_RESET,
    START_CONTINUOUS_MEASUREMENT, STOP_CONTINUOUS_MEASUREMENT,
};

pub struct Scd30<I2C, D> {
    i2c: I2C,
    delay: D,
}

impl<I2C, D, E> Scd30<I2C, D>
where
    I2C: I2c<Error = E>,
    D: DelayNs,
{
    pub fn new(i2c: I2C, delay: D) -> Self {
        Self { i2c, delay }
    }

    /// Release the I2C bus held by this sensor
    pub fn release(self) -> I2C {
        self.i2c
    }

    fn write_command(&mut self, cmd: Command) -> Result<(), Error<E>> {
        self.i2c
            .write(I2C_ADDRESS, &cmd.to_be_bytes())
            .map_err(|e| Error::I2C(e))?;
        Ok(())
    }

    fn write_command_with_data(&mut self, cmd: Command, data: u16) -> Result<(), Error<E>> {
        let c = cmd.to_be_bytes();
        let d = data.to_be_bytes();

        let mut buf = [0; 5];
        buf[0..2].copy_from_slice(&c);
        buf[2..4].copy_from_slice(&d);
        buf[4] = crc8(&d);

        self.i2c
            .write(I2C_ADDRESS, &buf)
            .map_err(|e| Error::I2C(e))?;

        Ok(())
    }

    fn read_command(&mut self, cmd: Command, buf: &mut [u8]) -> Result<(), Error<E>> {
        assert_eq!(buf.len() % 3, 0, "The buffer length must a multiple of 3");

        self.write_command(cmd)?;
        self.delay.delay_ms(READ_DELAY_MS);
        self.i2c.read(I2C_ADDRESS, buf).map_err(|e| Error::I2C(e))?;

        if !crc8_verify_chunked_3(buf) {
            return Err(Error::CRC);
        }

        Ok(())
    }

    /// Starts continuous measurement of the SCD30 to measure CO2 concentration, humidity and temperature. Measurement data
    /// which is not read from the sensor will be overwritten. The measurement interval is adjustable via the command documented in
    /// chapter 1.4.3, initial measurement rate is 2s.
    ///
    /// Continuous measurement status is saved in non-volatile memory. When the sensor is powered down while continuous
    /// measurement mode is active SCD30 will measure continuously after repowering without sending the measurement command.
    ///
    /// The CO2 measurement value can be compensated for ambient pressure by feeding the pressure value in mBar to the sensor.
    /// Setting the ambient pressure will overwrite previous settings of altitude compensation. Setting the argument to zero will
    /// deactivate the ambient pressure compensation (default ambient pressure = 1013.25 mBar). For setting a new ambient pressure
    /// when continuous measurement is running the whole command has to be written to SCD30.
    ///
    /// The valid range for the ambient pressure is 0 (disable) and `700..=1400` HPa.
    pub fn start_continuous_measurement(
        &mut self,
        ambient_pressure_hpa: u16,
    ) -> Result<(), Error<E>> {
        if !(700..=1400).contains(&ambient_pressure_hpa) {
            return Err(Error::InvalidInput);
        }

        self.write_command_with_data(START_CONTINUOUS_MEASUREMENT, ambient_pressure_hpa)?;

        Ok(())
    }

    /// Stops the continuous measurement of the SCD30.
    pub fn stop_continuous_measurement(&mut self) -> Result<(), Error<E>> {
        self.write_command(STOP_CONTINUOUS_MEASUREMENT)?;
        Ok(())
    }

    /// Sets the interval used by the SCD30 sensor to measure in continuous
    /// measurement mode (see chapter 1.4.1). Initial value is 2 s.
    ///
    /// The chosen measurement interval is saved in non-volatile memory and thus
    /// is not reset to its initial value after power up.
    ///
    /// The valid range is `2..=1800` seconds
    pub fn set_measurement_interval(&mut self, interval_seconds: u16) -> Result<(), Error<E>> {
        if !(2..=1800).contains(&interval_seconds) {
            return Err(Error::InvalidInput);
        }

        self.write_command_with_data(SET_MEASUREMENT_INTERVAL, interval_seconds)?;

        Ok(())
    }

    /// Data ready command is used to determine if a measurement can be read
    /// from the sensor’s buffer. Whenever there is a measurement available
    /// from the internal buffer this command returns `true` and `false`
    /// otherwise.
    ///
    /// As soon as the measurement has been read, the return value changes
    /// to `false`.
    ///
    /// It is recommended to use data ready status byte before
    /// readout of the measurement values.
    pub fn data_ready(&mut self) -> Result<bool, Error<E>> {
        let mut buf = [0; 3];
        self.read_command(GET_DATA_READY_STATUS, &mut buf)?;

        let val = u16::from_be_bytes([buf[0], buf[1]]);
        Ok(val == 1)
    }

    /// When new measurement data is available it can be read out with the
    /// following command. Note that the read header should be send with a
    /// delay of > 3ms following the write sequence. Make sure that the
    /// measurement is completed by reading the data ready status bit
    /// before read out.
    pub fn measurement(&mut self) -> Result<Measurement, Error<E>> {
        let mut buf = [0; 18];
        self.read_command(READ_MEASUREMENT, &mut buf)?;

        let co2 = f32::from_be_bytes([buf[0], buf[1], buf[3], buf[4]]);
        let tmp = f32::from_be_bytes([buf[6], buf[7], buf[9], buf[10]]);
        let hum = f32::from_be_bytes([buf[12], buf[13], buf[15], buf[16]]);

        Ok(Measurement {
            temperature: tmp,
            humidity: hum,
            co2: co2 as u16,
        })
    }

    /// Continuous automatic self-calibration can be (de-)activated with the
    /// following command. When activated for the first time a period of
    /// minimum 7 days is needed so that the algorithm can find its initial
    /// parameter set for ASC. The sensor has to be exposed to fresh air for
    /// at least 1 hour every day. Also during that period, the sensor may not
    /// be disconnected from the power supply, otherwise the procedure to find
    /// calibration parameters is aborted and has to be restarted from the
    /// beginning. The successfully calculated parameters are stored in
    /// non-volatile memory of the SCD30 having the effect that after a
    /// restart the previously found parameters for ASC are still present.
    ///
    /// Note that the most recently found self-calibration parameters will be
    /// actively used for self-calibration disregarding the status of this
    /// feature. Finding a new parameter set by the here described method will
    /// always overwrite the settings from external recalibration
    /// (see chapter 0) and vice-versa. The feature is switched off by default.
    ///
    /// To work properly SCD30 has to see fresh air on a regular basis. Optimal
    /// working conditions are given when the sensor sees fresh air for one
    /// hour every day so that ASC can constantly re-calibrate. ASC only works
    /// in continuous measurement mode.
    ///
    /// ASC status is saved in non-volatile memory. When the sensor is powered
    /// down while ASC is activated SCD30 will continue with automatic
    /// self-calibration after repowering without sending the command.
    pub fn enable_automatic_self_calibration(&mut self, enable: bool) -> Result<(), Error<E>> {
        self.write_command_with_data(MANAGE_AUTOMATIC_SELF_CALIBRATION, enable as u16)?;
        Ok(())
    }

    /// Forced recalibration (FRC) is used to compensate for sensor drifts when
    /// a reference value of the CO2 concentration in close proximity to the
    /// SCD30 is available. For best results, the sensor has to be run in a
    /// stable environment in continuous mode at a measurement rate of 2s for
    /// at least two minutes before applying the FRC command and sending the
    /// reference value. Setting a reference CO2 concentration by the method
    /// described here will always supersede corrections from the ASC
    /// (see chapter 1.4.6) and vice-versa. The reference CO2 concentration has
    /// to be within the range 400 ppm ≤ cref(CO2) ≤ 2000 ppm.
    ///
    ///  The FRC method imposes a permanent update of the CO2 calibration curve
    /// which persists after repowering the sensor. The most recently used
    /// reference value is retained in volatile memory and can be read out
    /// with the command sequence given below.
    ///
    ///  After repowering the sensor, the command will return the standard
    /// reference value of 400 ppm.
    pub fn set_forced_recalibration_value(&mut self, ppm: u16) -> Result<(), Error<E>> {
        self.write_command_with_data(SET_FORCED_RECALIBRATION_VALUE, ppm)?;
        Ok(())
    }

    /// The on-board RH/T sensor is influenced by thermal self-heating of
    /// SCD30 and other electrical components. Design-in alters the thermal
    /// properties of SCD30 such that temperature and humidity offsets may
    /// occur when operating the sensor in end-customer devices.
    /// Compensation of those effects is achievable by writing the temperature
    /// offset found in continuous operation of the device into the sensor.
    ///
    /// Temperature offset value is saved in non-volatile memory. The last
    /// set value will be used for temperature offset compensation after
    /// repowering.
    ///
    /// Unit: C * 100 => one tick corresponds to 0.01 degrees Celsius
    pub fn set_temperature_offset(&mut self, offset: u16) -> Result<(), Error<E>> {
        self.write_command_with_data(SET_TEMPERATURE_OFFSET, offset)?;
        Ok(())
    }

    /// Measurements of CO2 concentration based on the NDIR principle are
    /// influenced by altitude. SCD30 offers to compensate deviations due to
    /// altitude by using the following command. Setting altitude is
    /// disregarded when an ambient pressure is given to the sensor,
    /// please see section 1.4.1.
    ///
    ///  Altitude value is saved in non-volatile memory. The last set value
    /// will be used for altitude compensation after repowering.
    pub fn set_altitude_compensation(&mut self, altitude: u16) -> Result<(), Error<E>> {
        self.write_command_with_data(SET_ALTITUDE_COMPENSATION, altitude)?;
        Ok(())
    }

    /// Following command can be used to read out the firmware version of
    /// SCD30 module. The returned value is in the format `(Major, Minor)`
    pub fn read_firmware_version(&mut self) -> Result<(u8, u8), Error<E>> {
        let mut buf = [0; 3];
        self.read_command(READ_FIRMWARE_VERSION, &mut buf)?;

        Ok((buf[0], buf[1]))
    }

    /// The SCD30 provides a soft reset mechanism that forces the sensor into
    /// the same state as after powering up without the need for removing the
    /// power-supply. It does so by restarting its system controller.
    /// After soft reset the sensor will reload all calibrated data.
    ///
    /// However, it is worth noting that the sensor reloads calibration data
    /// prior to every measurement by default. This includes previously set
    /// reference values from ASC or FRC as well as temperature offset values
    /// last setting.
    ///
    /// The sensor is able to receive the command at any time, regardless of
    /// its internal state.
    pub fn soft_reset(&mut self) -> Result<(), Error<E>> {
        self.write_command(SOFT_RESET)?;
        Ok(())
    }
}
