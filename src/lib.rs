#![no_std]
#![deny(unsafe_code)]
#![deny(warnings)]
#![deny(unused_must_use)]
#![deny(unexpected_cfgs)]

//! LibSCD is a crate providing both synchronous and asynchronous driver
//! implementations for SCD30 and SCD4x CO2 sensors using the
//! [embedded-hal](https://crates.io/crates/embedded-hal) and
//! [embedded-hal-async](https://crates.io/crates/embedded-hal-async)
//! interfaces
//!
//! ## Feature Flags
//!
//! - `defmt`: Derive `defmt::Format` for the error type
//! - `sync`: Enable the blocking driver implementation for the selected sensors
//! - `async`: Enable the async driver implementation for the selected sensors
//! - `scd30`: Enable the driver for the SCD30 sensor
//! - `scd4x`: Enable the driver for the SCD4x family of sensors
//! - `scd41`: Enable additional features of the SCD4x driver that available only on SCD41 sensors

/// Error type used by the library
pub mod error;

/// Shared measurement type used by the various sensors
pub mod measurement;

/// Synchronous (blocking) driver implementations using embedded-hal. This
/// module needs to be enabled via the `sync` feature flag
#[cfg(feature = "sync")]
pub mod synchronous;

/// Asynchronous driver implementations using embedded-hal-async. This
/// module needs to be enabled via the `async` feature flag
#[cfg(feature = "async")]
pub mod asynchronous;

/// Shared code across the sync/async implementations
#[doc(hidden)]
pub(crate) mod internal;

#[cfg(not(all(
    any(feature = "sync", feature = "async"),
    any(feature = "scd30", feature = "scd4x", feature = "scd41")
)))]
compile_error!("You must select at least one sensor (scd30/scd4x/scd41) and at least one mode of operation (sync/async)");
