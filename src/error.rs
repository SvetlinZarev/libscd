#[derive(Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<E> {
    I2C(E),
    CRC,
    NotAllowed,
    InvalidInput,
}
