#![deny(unexpected_cfgs)]

#[cfg(feature = "scd30")]
const SCD_30_ADDRESS: u8 = 0x61;

#[cfg(any(feature = "scd4x", feature = "scd41"))]
const SCD_4X_ADDRESS: u8 = 0x62;

#[test]
#[cfg(all(feature = "scd30", feature = "async"))]
pub fn scd30_async_has_i2c_address() {
    assert_eq!(SCD_30_ADDRESS, libscd::asynchronous::scd30::I2C_ADDRESS);
}

#[test]
#[cfg(all(feature = "scd4x", feature = "async"))]
pub fn scd40_async_has_i2c_address() {
    assert_eq!(SCD_4X_ADDRESS, libscd::asynchronous::scd4x::I2C_ADDRESS);
}

#[test]
#[cfg(all(feature = "scd41", feature = "async"))]
pub fn scd41_async_has_i2c_address() {
    assert_eq!(SCD_4X_ADDRESS, libscd::asynchronous::scd4x::I2C_ADDRESS);
}

#[test]
#[cfg(all(feature = "scd30", feature = "sync"))]
pub fn scd30_sync_has_i2c_address() {
    assert_eq!(SCD_30_ADDRESS, libscd::synchronous::scd30::I2C_ADDRESS);
}

#[test]
#[cfg(all(feature = "scd4x", feature = "sync"))]
pub fn scd40_sync_has_i2c_address() {
    assert_eq!(SCD_4X_ADDRESS, libscd::synchronous::scd4x::I2C_ADDRESS);
}

#[test]
#[cfg(all(feature = "scd41", feature = "sync"))]
pub fn scd41_sync_has_i2c_address() {
    assert_eq!(SCD_4X_ADDRESS, libscd::synchronous::scd4x::I2C_ADDRESS);
}
