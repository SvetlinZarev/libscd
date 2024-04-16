# Lib SCD

![License](https://img.shields.io/crates/l/libscd)
![Version](https://img.shields.io/crates/v/libscd)
![Downloads](https://img.shields.io/crates/dv/libscd)
![Build](https://img.shields.io/github/actions/workflow/status/SvetlinZarev/libscd/run_tests.yml)

LibSCD is s Rust driver implementation for SCD30, SCD40 and SCD41 sensors
using the `embedded-hal` and `embedded-hal-async` interfaces.

## Usage

Add the crate as a dependency in `Cargo.toml`

```toml
[dependencies.libscd]
version = "0.3"
features = []
```

The support for each sensor and sync/async mode is controlled by a feature:

| Feature | Description                                                         |
|---------|---------------------------------------------------------------------|
| sync    | Enables the blocking driver implementation for the selected sensors |
| async   | Enables the async driver implementation for the selected sensors    |
| scd30   | Enables the driver for the SCD30 sensor                             |
| scd40   | Enables the driver for the SCD40 sensor                             |
| scd41   | Enables the driver for the SCD41 sensor                             |
| defmt   | Derive `defmt::Format` for the error type                           |

## License

The project is dual licensed under [MIT](https://opensource.org/licenses/MIT)
or [APACHE-2.0](https://opensource.org/licenses/Apache-2.0)
