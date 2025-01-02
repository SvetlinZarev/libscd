use crate::error::Error;
use crate::internal::common::{assert_chunked_with_len3, crc8_verify_chunked_3};
use embedded_hal::i2c::I2c;

pub(crate) fn i2c_read<E, I2C: I2c<Error = E>>(
    i2c: &mut I2C,
    i2c_addr: u8,
    read_buf: &mut [u8],
) -> Result<(), Error<E>> {
    assert_chunked_with_len3(read_buf);

    i2c.read(i2c_addr, read_buf).map_err(|e| Error::I2C(e))?;

    if !crc8_verify_chunked_3(read_buf) {
        return Err(Error::CRC);
    }

    Ok(())
}

pub(crate) fn i2c_write<E, I2C: I2c<Error = E>>(
    i2c: &mut I2C,
    i2c_addr: u8,
    payload: &[u8],
) -> Result<(), Error<E>> {
    i2c.write(i2c_addr, payload).map_err(|e| Error::I2C(e))?;
    Ok(())
}
