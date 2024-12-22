#[derive(Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<E> {
    /// I2C bus error
    I2C(E),

    /// CRC validation failed
    CRC,

    /// The operation cannot be executed with the current state of the sensor
    NotAllowed,

    /// An invalid input was passed as a parameter
    InvalidInput,

    /// Forced recalibration failed because the sensor was not operated before running the command
    FrcFailed,
}
