use crate::internal::crc::crc8;

pub fn assert_chunked_with_len3(buf: &[u8]) {
    assert_eq!(
        buf.len() % 3,
        0,
        "The read buffer length must be a multiple of 3"
    );
}

pub fn crc8_verify_chunked_3(data: &[u8]) -> bool {
    data.chunks_exact(3)
        .map(|w| (crc8(&w[..2]), w[2]))
        .all(|(x, y)| x == y)
}

pub const fn opcode_with_data_into_payload(opcode: u16, data: u16) -> [u8; 5] {
    let c = opcode.to_be_bytes();
    let d = data.to_be_bytes();

    let mut buf = [0; 5];

    // Replace with `buf[0..2].copy_from_slice(&c);` when it becomes a `const` function
    buf[0] = c[0];
    buf[1] = c[1];

    // Replace with `buf[2..4].copy_from_slice(&d);` when it becomes a `const` function
    buf[2] = d[0];
    buf[3] = d[1];

    buf[4] = crc8(&d);

    buf
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_buf_len_is_multiple_of_three() {
        assert_chunked_with_len3(&[0; 3]);
        assert_chunked_with_len3(&[0; 6]);
        assert_chunked_with_len3(&[0; 9]);
    }

    #[test]
    #[should_panic]
    fn test_buf_len_is_not_multiple_of_three() {
        assert_chunked_with_len3(&[0; 4]);
    }

    #[test]
    fn test_chunked_crc8() {
        assert!(crc8_verify_chunked_3(&[0xBE, 0xEF, 0x92]));
        assert!(crc8_verify_chunked_3(&[0xBE, 0xEF, 0x92, 0xBE, 0xEF, 0x92]))
    }

    #[test]
    #[should_panic]
    fn test_chunked_crc8_with_wrong_checksum_1() {
        assert!(crc8_verify_chunked_3(&[0xBE, 0xEF, 0x90]));
    }

    #[test]
    #[should_panic]
    fn test_chunked_crc8_with_wrong_checksum_2() {
        assert!(crc8_verify_chunked_3(&[0xBE, 0xEF, 0x91, 0xBE, 0xEF, 0x92]))
    }

    #[test]
    #[should_panic]
    fn test_chunked_crc8_with_wrong_checksum_3() {
        assert!(crc8_verify_chunked_3(&[0xBE, 0xEF, 0x92, 0xBE, 0xEF, 0x91]))
    }

    #[test]
    fn test_opcode_with_data_into_payload() {
        let result = opcode_with_data_into_payload(0x0102, 0x0304);
        assert_eq!([0x01, 0x02, 0x03, 0x04, 0x68], result);
    }
}
