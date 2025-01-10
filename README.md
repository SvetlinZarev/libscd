# Lib SCD

![License](https://img.shields.io/crates/l/libscd)
[![Version](https://img.shields.io/crates/v/libscd)](https://crates.io/crates/libscd)
[![Downloads](https://img.shields.io/crates/d/libscd)](https://crates.io/crates/libscd)
[![Build](https://img.shields.io/github/actions/workflow/status/SvetlinZarev/libscd/run_tests.yml)](https://github.com/SvetlinZarev/libscd/actions)

LibSCD is s Rust driver implementation for SCD30, SCD40 and SCD41 sensors
using the `embedded-hal` and `embedded-hal-async` interfaces.

## Usage

Add the crate as a dependency in `Cargo.toml` and select the required features:

```toml
[dependencies.libscd]
version = "0.4"
features = ["defmt", "sync", "scd40"]
```

Then we can start consuming data from SCD40 using blocking I2C communication:

```rust
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    let i2c = I2c::new_blocking(
        p.I2C2,
        p.PB10,
        p.PB3,
        Hertz(100_000),
        Default::default(),
    );

    let mut scd = Scd4x::new(i2c, Delay);

    // When re-programming, the controller will be restarted,
    // but not the sensor. We try to stop it in order to
    // prevent the rest of the commands failing.
    _ = scd.stop_periodic_measurement();

    info!("Sensor serial number: {:?}", scd.serial_number());
    if let Err(e) = scd.start_periodic_measurement() {
        defmt::panic!("Failed to start periodic measurement: {:?}", e);
    }

    loop {
        if scd.data_ready().unwrap() {
            let m = scd.read_measurement().unwrap();
            info!("CO2: {}\nHumidity: {}\nTemperature: {}", m.co2, m.humidity, m.temperature)
        }

        Delay.delay_ms(1000)
    }
}
```

## Crate Feature Flags

The support for each sensor and sync/async mode is controlled by a feature:

| Feature | Description                                                                 |
|---------|-----------------------------------------------------------------------------|
| sync    | Enables the blocking driver implementation for the selected sensors         |
| async   | Enables the async driver implementation for the selected sensors            |
| scd30   | Enables the driver for the SCD30 sensor                                     |
| scd4x   | Enables the driver for the SCD4x family of sensors                          |
| scd41   | Enables additional SCD4x driver features available only on the SCD41 sensor |
| defmt   | Derive `defmt::Format` for the error type                                   |

## License

The project is dual licensed under [MIT](https://opensource.org/licenses/MIT)
or [APACHE-2.0](https://opensource.org/licenses/Apache-2.0)
