#[derive(Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Measurement {
    pub temperature: f32,
    pub humidity: f32,
    pub co2: u16,
}
