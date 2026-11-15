//! Blocking vs async parity: identical wire transcripts, read
//! sequences, and delay totals for the same scripted sequence.
#![cfg(feature = "async")]
#![allow(clippy::unwrap_used, clippy::panic)]

use std::collections::VecDeque;

use cst816s::config::{LowPowerConfig, ReportMode};
use cst816s::{Cst816s, Cst816sAsync};
use embedded_hal::delay::DelayNs as BlockingDelay;
use embedded_hal::digital::{ErrorType as DigitalErrorType, InputPin, OutputPin};
use embedded_hal::i2c::{Error as I2cError, ErrorKind, I2c as BlockingI2c, Operation};
use futures::executor::block_on;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct FakeErr;
impl I2cError for FakeErr {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

#[derive(Debug, Default)]
struct FakeI2c {
    writes: Vec<(u8, Vec<u8>)>,
    reads: Vec<u8>,
    responses: VecDeque<Vec<u8>>,
}

impl FakeI2c {
    fn scripted() -> Self {
        // init probe (chip, fw, proj) + one touch frame.
        let mut f = Self::default();
        for r in [
            vec![0xB4],
            vec![0x01],
            vec![0x02],
            vec![0x05, 0x01, 0x01, 0x20, 0x02, 0x40],
            vec![0x05, 0x01, 0x01, 0x20, 0x02, 0x40],
        ] {
            f.responses.push_back(r);
        }
        f
    }
    fn serve(&mut self, buf: &mut [u8]) {
        let resp = self.responses.pop_front().expect("no scripted response");
        assert_eq!(resp.len(), buf.len());
        buf.copy_from_slice(&resp);
    }
    fn run_ops(&mut self, address: u8, operations: &mut [Operation<'_>]) {
        let mut pending: Vec<u8> = Vec::new();
        let mut saw_write = false;
        for op in operations.iter_mut() {
            match op {
                Operation::Write(bytes) => {
                    saw_write = true;
                    pending.extend_from_slice(bytes);
                }
                Operation::Read(buf) => {
                    if saw_write {
                        self.reads.push(pending[0]);
                        pending.clear();
                        saw_write = false;
                    }
                    self.serve(buf);
                }
            }
        }
        if saw_write {
            self.writes.push((address, pending));
        }
    }
}

impl embedded_hal::i2c::ErrorType for FakeI2c {
    type Error = FakeErr;
}

impl BlockingI2c for FakeI2c {
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        self.run_ops(address, operations);
        Ok(())
    }
}

#[allow(clippy::unused_async)]
impl embedded_hal_async::i2c::I2c for FakeI2c {
    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        self.run_ops(address, operations);
        Ok(())
    }
}

#[derive(Debug, Default)]
struct FakePin {
    levels: Vec<bool>,
    waits: usize,
    state: bool,
}

impl DigitalErrorType for FakePin {
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

#[allow(clippy::unused_async)]
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

#[derive(Debug, Default)]
struct FakeDelay {
    total_ns: u64,
}

impl BlockingDelay for FakeDelay {
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

#[allow(clippy::unused_async)]
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

#[test]
fn blocking_async_parity() {
    let lp = LowPowerConfig {
        scan_raw1: Some(0x1234),
        ..LowPowerConfig::default()
    };

    // Blocking run.
    let mut b =
        Cst816s::new(FakeI2c::scripted(), FakeDelay::default()).with_reset_pin(FakePin::default());
    let b_info = b.init().unwrap();
    b.set_report_mode(ReportMode::PointAndGesture).unwrap();
    let b_report = b.read_touch().unwrap();
    b.apply_low_power_config(&lp).unwrap();
    b.enter_deep_sleep().unwrap();
    let (b_bus, b_delay, b_rst, _) = b.release();

    // Async run: same script, plus one wait_for_touch before read_touch
    // would diverge reads — instead assert wait separately below.
    let mut a = Cst816sAsync::new(FakeI2c::scripted(), FakeDelay::default())
        .with_reset_pin(FakePin::default())
        .with_interrupt_pin(FakePin::default());
    let (a_info, a_report, waited_report) = block_on(async {
        let info = a.init().await.unwrap();
        a.set_report_mode(ReportMode::PointAndGesture)
            .await
            .unwrap();
        let waited = a.wait_for_touch().await.unwrap();
        let report = a.read_touch().await.unwrap();
        a.apply_low_power_config(&lp).await.unwrap();
        a.enter_deep_sleep().await.unwrap();
        (info, report, waited)
    });
    let (a_bus, a_delay, a_rst, a_int) = a.release();

    assert_eq!(b_info, a_info);
    assert_eq!(b_report, a_report);
    assert_eq!(waited_report, b_report);
    assert_eq!(a_int.waits, 1, "wait_for_touch must record one wait");

    // Async performed one extra frame read (wait_for_touch). Compare
    // writes + delays exactly; reads equal after removing the extra
    // 0x01 frame read from the async sequence.
    assert_eq!(b_bus.writes, a_bus.writes);
    assert_eq!(b_delay.total_ns, a_delay.total_ns);
    assert_eq!(b_rst.levels, a_rst.levels);
    let mut a_reads = a_bus.reads.clone();
    let pos = a_reads.iter().position(|r| *r == 0x01).unwrap();
    a_reads.remove(pos);
    assert_eq!(b_bus.reads, a_reads);
}
