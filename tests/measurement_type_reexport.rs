#[test]
#[cfg(all(feature = "scd30", feature = "async"))]
pub fn scd30_async_has_measurement_type_reexport() {
    _ = libscd::asynchronous::scd30::Measurement {
        temperature: 0.0,
        humidity: 0.0,
        co2: 0,
    }
}

#[test]
#[cfg(all(feature = "scd40", feature = "async"))]
pub fn scd40_async_has_measurement_type_reexport() {
    _ = libscd::asynchronous::scd4x::Measurement {
        temperature: 0.0,
        humidity: 0.0,
        co2: 0,
    }
}

#[test]
#[cfg(all(feature = "scd41", feature = "async"))]
pub fn scd41_async_has_measurement_type_reexport() {
    _ = libscd::asynchronous::scd4x::Measurement {
        temperature: 0.0,
        humidity: 0.0,
        co2: 0,
    }
}

#[test]
#[cfg(all(feature = "scd30", feature = "sync"))]
pub fn scd30_sync_has_measurement_type_reexport() {
    _ = libscd::synchronous::scd30::Measurement {
        temperature: 0.0,
        humidity: 0.0,
        co2: 0,
    }
}

#[test]
#[cfg(all(feature = "scd40", feature = "sync"))]
pub fn scd40_sync_has_measurement_type_reexport() {
    _ = libscd::synchronous::scd4x::Measurement {
        temperature: 0.0,
        humidity: 0.0,
        co2: 0,
    }
}

#[test]
#[cfg(all(feature = "scd41", feature = "sync"))]
pub fn scd41_sync_has_measurement_type_reexport() {
    _ = libscd::synchronous::scd4x::Measurement {
        temperature: 0.0,
        humidity: 0.0,
        co2: 0,
    }
}
