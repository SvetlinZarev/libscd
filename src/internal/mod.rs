pub mod common;
pub mod crc;

#[cfg(feature = "scd30")]
pub mod scd30;

#[cfg(any(feature = "scd4x", feature = "scd41"))]
pub mod scd4x;
