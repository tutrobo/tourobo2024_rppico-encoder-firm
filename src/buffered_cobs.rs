pub struct BufferedCobs<const BUFFER_SIZE: usize> {
    buffer: [u8; BUFFER_SIZE],
    buffer_index: usize,
    buffer_full: bool,
    data_ready: u16,
}

impl<const BUFFER_SIZE: usize> BufferedCobs<BUFFER_SIZE> {
    pub fn new() -> BufferedCobs<BUFFER_SIZE> {
        BufferedCobs::<BUFFER_SIZE> {
            buffer: [0u8; BUFFER_SIZE],
            buffer_index: 0,
            buffer_full: false,
            data_ready: 0,
        }
    }
    pub fn put(&mut self, byte: u8) -> Result<(), ()> {
        if self.buffer_full {
            return Err(());
        }
        self.buffer[self.buffer_index] = byte;
        self.buffer_index += 1;
        if self.buffer_index == BUFFER_SIZE {
            self.buffer_full = true;
        }
        if byte == 0 {
            self.data_ready += 1;
        }
        Ok(())
    }
    pub fn put_slice(&mut self, slice: &[u8]) -> Result<(), ()> {
        for byte in slice {
            self.put(*byte)?;
        }
        Ok(())
    }
    fn slide(&mut self, index: usize) {
        for i in 0..(BUFFER_SIZE - index) {
            self.buffer[i] = self.buffer[i + index];
        }
        self.buffer_index -= index;
        if index > 0 {
            self.buffer_full = false;
        }
    }
    pub fn read(&mut self) -> Result<(), ()> {
        todo!();
    }

    // ENCODED_DATA_LEN must be DATA_LEN +2
    pub fn encode<const DATA_LEN: usize, const ENCODED_DATA_LEN: usize>(
        data: [u8; DATA_LEN],
    ) -> [u8; ENCODED_DATA_LEN] {
        let mut encoded = [0u8; ENCODED_DATA_LEN];
        let mut index_last_zero = 0;
        for i in 0..DATA_LEN {
            if data[i] == 0 {
                encoded[index_last_zero] = (i + 1 - index_last_zero) as u8;
                index_last_zero = i + 1;
            } else {
                encoded[i + 1] = data[i];
            }
        }
        encoded[index_last_zero] = (DATA_LEN + 1 - index_last_zero) as u8;
        encoded[DATA_LEN + 1] = 0;
        encoded
    }
}
