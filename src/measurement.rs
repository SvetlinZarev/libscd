/// Structure containing the measurements from a CO2 sensor
#[derive(Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Measurement {
    /// Measured temperature in Celsius
    pub temperature: f32,

    /// Measured humidity (RH%)
    pub humidity: f32,

    /// Measured CO2 concentration in PPM
    pub co2: u16,
}
