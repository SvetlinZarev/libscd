const CRC8_POLY: u8 = 0x31;
const CRC8_INITIAL: u8 = 0xFF;
const BYTE_MSB: u8 = 1 << 7;

pub const fn crc8(data: &[u8]) -> u8 {
    let mut crc = CRC8_INITIAL;

    let mut idx = 0;
    while idx < data.len() {
        crc ^= data[idx];
        idx += 1;

        let mut bit = 0;
        while bit < 8 {
            bit += 1;

            let msb = crc & BYTE_MSB;
            crc <<= 1;

            if msb != 0 {
                crc ^= CRC8_POLY;
            }
        }
    }

    crc
}

#[cfg(test)]
mod tests {
    use super::crc8;

    #[test]
    fn test_crc8() {
        let checksum = crc8(&[0xBE, 0xEF]);
        assert_eq!(0x92, checksum);
    }
}
