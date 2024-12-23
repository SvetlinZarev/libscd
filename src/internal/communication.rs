use crate::internal::crc::crc8;

pub fn assert_chunked_with_len3(buf: &[u8]) {
    assert_eq!(
        buf.len() % 3,
        0,
        "The read buffer length must be a multiple of 3"
    );
}

pub fn opcode_with_data_into_payload(opcode: u16, data: u16) -> [u8; 5] {
    let c = opcode.to_be_bytes();
    let d = data.to_be_bytes();

    let mut buf = [0; 5];
    buf[0..2].copy_from_slice(&c);
    buf[2..4].copy_from_slice(&d);
    buf[4] = crc8(&d);

    buf
}

#[cfg(feature = "sync")]
pub mod sync {
    use crate::error::Error;
    use crate::internal::communication::assert_chunked_with_len3;
    use crate::internal::crc::crc8_verify_chunked_3;
    use embedded_hal::i2c::I2c;

    pub fn i2c_read<E, I2C: I2c<Error = E>>(
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

    pub fn i2c_write<E, I2C: I2c<Error = E>>(
        i2c: &mut I2C,
        i2c_addr: u8,
        payload: &[u8],
    ) -> Result<(), Error<E>> {
        i2c.write(i2c_addr, payload).map_err(|e| Error::I2C(e))?;
        Ok(())
    }
}

#[cfg(feature = "async")]
pub mod asynch {
    use crate::error::Error;
    use crate::internal::communication::assert_chunked_with_len3;
    use crate::internal::crc::crc8_verify_chunked_3;
    use embedded_hal_async::i2c::I2c;

    pub async fn i2c_read<E, I2C: I2c<Error = E>>(
        i2c: &mut I2C,
        i2c_addr: u8,
        read_buf: &mut [u8],
    ) -> Result<(), Error<E>> {
        assert_chunked_with_len3(read_buf);

        i2c.read(i2c_addr, read_buf)
            .await
            .map_err(|e| Error::I2C(e))?;

        if !crc8_verify_chunked_3(read_buf) {
            return Err(Error::CRC);
        }

        Ok(())
    }

    pub async fn i2c_write<E, I2C: I2c<Error = E>>(
        i2c: &mut I2C,
        i2c_addr: u8,
        payload: &[u8],
    ) -> Result<(), Error<E>> {
        i2c.write(i2c_addr, payload)
            .await
            .map_err(|e| Error::I2C(e))?;

        Ok(())
    }
}
