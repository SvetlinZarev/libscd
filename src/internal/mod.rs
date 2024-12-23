pub mod crc;
pub mod measurement;
pub mod communication;

#[cfg(feature = "scd30")]
pub mod scd30;

#[cfg(any(feature = "scd40", feature = "scd41"))]
pub mod scd4x;
