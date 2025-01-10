/// Module containing the driver implementation for the SCD30 sensor
#[cfg(feature = "scd30")]
pub mod scd30;

/// Module containing the driver implementation for the SCD40 and SCD41 sensors
#[cfg(any(feature = "scd4x", feature = "scd41"))]
pub mod scd4x;

/// Common utilities for I2C communication as described by the SCD datasheets
mod i2c;
