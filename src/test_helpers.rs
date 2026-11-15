//! Test fakes: recording I2C bus, pin, and delay.
#![allow(clippy::unwrap_used, clippy::panic, dead_code)]

use std::collections::VecDeque;

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{ErrorType, InputPin, OutputPin};
use embedded_hal::i2c::{Error as I2cError, ErrorKind, I2c, Operation};

/// Fake I2C error (never actually returned; satisfies trait bounds).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FakeI2cError;

impl I2cError for FakeI2cError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

/// Recording I2C fake. Every `write` is recorded as `(addr, bytes)`;
/// every `write_read` records its register byte in `reads` and serves
/// bytes from the scripted response queue.
#[derive(Debug, Default)]
pub struct FakeI2c {
    pub writes: Vec<(u8, Vec<u8>)>,
    pub reads: Vec<u8>,
    responses: VecDeque<Vec<u8>>,
}

impl FakeI2c {
    pub fn new() -> Self {
        Self::default()
    }

    /// Queue one read response (consumed FIFO by `write_read`).
    pub fn script(&mut self, response: &[u8]) -> &mut Self {
        self.responses.push_back(response.to_vec());
        self
    }

    pub fn with_script(mut self, responses: &[&[u8]]) -> Self {
        for r in responses {
            self.responses.push_back(r.to_vec());
        }
        self
    }

    fn serve(&mut self, buf: &mut [u8]) {
        let resp = self
            .responses
            .pop_front()
            .expect("FakeI2c: read with no scripted response left");
        assert_eq!(
            resp.len(),
            buf.len(),
            "FakeI2c: scripted response length != read buffer length"
        );
        buf.copy_from_slice(&resp);
    }
}

impl embedded_hal::i2c::ErrorType for FakeI2c {
    type Error = FakeI2cError;
}

impl I2c for FakeI2c {
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        // Default I2c methods (write/write_read/read) funnel here.
        let mut pending_write: Vec<u8> = Vec::new();
        let mut saw_write = false;
        for op in operations.iter_mut() {
            match op {
                Operation::Write(bytes) => {
                    saw_write = true;
                    pending_write.extend_from_slice(bytes);
                }
                Operation::Read(buf) => {
                    if saw_write {
                        self.reads.push(pending_write[0]);
                        pending_write.clear();
                        saw_write = false;
                    }
                    self.serve(buf);
                }
            }
        }
        if saw_write {
            self.writes.push((address, pending_write));
        }
        Ok(())
    }
}

#[cfg(feature = "async")]
#[allow(clippy::unused_async, clippy::unused_async_trait_impl)]
impl embedded_hal_async::i2c::I2c for FakeI2c {
    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        I2c::transaction(self, address, operations)
    }
}

/// Recording pin fake: `levels` holds each driven level (true = high).
#[derive(Debug, Default)]
pub struct FakePin {
    pub levels: Vec<bool>,
    pub waits: usize,
    state: bool,
}

impl FakePin {
    pub fn new() -> Self {
        Self::default()
    }
}

impl ErrorType for FakePin {
    type Error = core::convert::Infallible;
}

impl OutputPin for FakePin {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.state = false;
        self.levels.push(false);
        Ok(())
    }
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.state = true;
        self.levels.push(true);
        Ok(())
    }
}

impl InputPin for FakePin {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.state)
    }
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!self.state)
    }
}

/// Async `Wait` fake: returns immediately, records the wait.
#[cfg(feature = "async")]
#[allow(clippy::unused_async, clippy::unused_async_trait_impl)]
impl embedded_hal_async::digital::Wait for FakePin {
    async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
        self.waits += 1;
        Ok(())
    }
    async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
        self.waits += 1;
        Ok(())
    }
    async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
        self.waits += 1;
        Ok(())
    }
    async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
        self.waits += 1;
        Ok(())
    }
    async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
        self.waits += 1;
        Ok(())
    }
}

/// Delay fake accumulating total delayed time.
#[derive(Debug, Default)]
pub struct FakeDelay {
    pub total_ns: u64,
}

impl FakeDelay {
    pub fn new() -> Self {
        Self::default()
    }
    pub fn total_ms(&self) -> u64 {
        self.total_ns / 1_000_000
    }
}

impl DelayNs for FakeDelay {
    fn delay_ns(&mut self, ns: u32) {
        self.total_ns += u64::from(ns);
    }
    fn delay_us(&mut self, us: u32) {
        self.total_ns += u64::from(us) * 1_000;
    }
    fn delay_ms(&mut self, ms: u32) {
        self.total_ns += u64::from(ms) * 1_000_000;
    }
}

#[cfg(feature = "async")]
#[allow(clippy::unused_async, clippy::unused_async_trait_impl)]
impl embedded_hal_async::delay::DelayNs for FakeDelay {
    async fn delay_ns(&mut self, ns: u32) {
        self.total_ns += u64::from(ns);
    }
    async fn delay_us(&mut self, us: u32) {
        self.total_ns += u64::from(us) * 1_000;
    }
    async fn delay_ms(&mut self, ms: u32) {
        self.total_ns += u64::from(ms) * 1_000_000;
    }
}
