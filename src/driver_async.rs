//! Async CST816 driver (`embedded-hal-async`).
//!
//! Hand-written twin of the blocking driver: every public method
//! name and wire behaviour matches `Cst816s`. Shared logic lives in
//! `data`/`config`/`registers`; only transport differs.

use embedded_hal::digital::OutputPin;
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::digital::Wait;
use embedded_hal_async::i2c::I2c;

use crate::config::{Config, IoCtl, IrqCtl, LowPowerConfig, MotionMask, ReportMode};
use crate::data::{ChipId, CoordinateTransform, DeviceInfo, TouchReport};
use crate::error::Error;
use crate::interface::{AsyncRegisterIo, DeviceAddress, I2cInterface, NoPin};
use crate::registers::{REG_DATA_START, RESET_LOW_MS, RESET_SETTLE_MS, Register, SLEEP_COMMAND};

#[inline]
fn check_range(value: u8, range: core::ops::RangeInclusive<u8>) -> Result<(), ()> {
    if range.contains(&value) {
        Ok(())
    } else {
        Err(())
    }
}

/// Async CST816 driver over I2C. Twin of [`crate::Cst816s`].
pub struct Cst816sAsync<I, D, RST = NoPin, INT = NoPin> {
    io: I2cInterface<I>,
    delay: D,
    rst: RST,
    int: INT,
    config: Config,
    chip: Option<ChipId>,
    transform: CoordinateTransform,
    width: u16,
    height: u16,
}

impl<I, D, RST, INT> core::fmt::Debug for Cst816sAsync<I, D, RST, INT> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Cst816sAsync")
            .field("chip", &self.chip)
            .field("config", &self.config)
            .field("transform", &self.transform)
            .field("width", &self.width)
            .field("height", &self.height)
            .finish_non_exhaustive()
    }
}

impl<I, D> Cst816sAsync<I, D, NoPin, NoPin> {
    /// New driver at the primary address, no pins attached.
    pub fn new(bus: I, delay: D) -> Self {
        Self::new_with_address(bus, DeviceAddress::Primary, delay)
    }

    /// New driver with an explicit address, no pins attached.
    pub fn new_with_address(bus: I, address: DeviceAddress, delay: D) -> Self {
        Self {
            io: I2cInterface::new(bus, address),
            delay,
            rst: NoPin,
            int: NoPin,
            config: Config::default(),
            chip: None,
            transform: CoordinateTransform::default(),
            width: 240,
            height: 240,
        }
    }
}

impl<I, D, RST, INT> Cst816sAsync<I, D, RST, INT> {
    /// Attach a reset pin (retypes the driver).
    pub fn with_reset_pin<R2: OutputPin>(self, pin: R2) -> Cst816sAsync<I, D, R2, INT> {
        Cst816sAsync {
            io: self.io,
            delay: self.delay,
            rst: pin,
            int: self.int,
            config: self.config,
            chip: self.chip,
            transform: self.transform,
            width: self.width,
            height: self.height,
        }
    }

    /// Attach an interrupt pin (retypes the driver).
    pub fn with_interrupt_pin<N2>(self, pin: N2) -> Cst816sAsync<I, D, RST, N2> {
        Cst816sAsync {
            io: self.io,
            delay: self.delay,
            rst: self.rst,
            int: pin,
            config: self.config,
            chip: self.chip,
            transform: self.transform,
            width: self.width,
            height: self.height,
        }
    }

    /// Replace the stored configuration (applied by `init`/`wake`).
    #[must_use]
    pub fn with_config(mut self, config: Config) -> Self {
        self.config = config;
        self
    }

    /// Set the coordinate transform and panel dimensions.
    #[must_use]
    pub fn with_transform(
        mut self,
        transform: CoordinateTransform,
        width: u16,
        height: u16,
    ) -> Self {
        self.transform = transform;
        self.width = width;
        self.height = height;
        self
    }

    /// Release bus, delay, and pins.
    pub fn release(self) -> (I, D, RST, INT) {
        (self.io.bus, self.delay, self.rst, self.int)
    }

    /// Chip identified by the last `probe`/`init`, if any.
    #[must_use]
    pub fn chip(&self) -> Option<ChipId> {
        self.chip
    }

    /// Stored configuration.
    #[must_use]
    pub fn config(&self) -> Config {
        self.config
    }
}

impl<I, D, RST, INT> Cst816sAsync<I, D, RST, INT>
where
    I: I2c,
    D: DelayNs,
    RST: OutputPin,
{
    /// Pulse RST low for 10 ms, release, settle 100 ms.
    pub async fn reset(&mut self) -> Result<(), Error<I::Error, RST::Error>> {
        self.rst.set_low().map_err(Error::Pin)?;
        self.delay.delay_ms(RESET_LOW_MS).await;
        self.rst.set_high().map_err(Error::Pin)?;
        self.delay.delay_ms(RESET_SETTLE_MS).await;
        Ok(())
    }

    /// Identify the chip. Unknown IDs are not an error.
    pub async fn probe(&mut self) -> Result<DeviceInfo, Error<I::Error, RST::Error>> {
        let chip_raw = self.io.read_register(Register::ChipId.addr()).await?;
        let (chip, firmware_version) = if chip_raw != 0x00 {
            let fw = self
                .io
                .read_register(Register::FirmwareVersion.addr())
                .await?;
            (ChipId::from_chip_register(chip_raw), fw)
        } else {
            let factory = self.io.read_register(Register::FactoryId.addr()).await?;
            let fw = self
                .io
                .read_register(Register::FirmwareVersionAlt.addr())
                .await?;
            (ChipId::from_factory_register(factory), fw)
        };
        let project_id = self.io.read_register(Register::ProjectId.addr()).await?;
        self.chip = Some(chip);
        let info = DeviceInfo {
            chip,
            project_id,
            firmware_version,
        };
        #[cfg(feature = "log")]
        log::debug!("cst816s probe: {info:?}");
        Ok(info)
    }

    /// Reset, probe, and apply stored configuration.
    pub async fn init(&mut self) -> Result<DeviceInfo, Error<I::Error, RST::Error>> {
        self.reset().await?;
        let info = self.probe().await?;
        self.apply_config().await?;
        Ok(info)
    }

    /// Read one touch frame (transform applied, unfiltered).
    pub async fn read_touch(&mut self) -> Result<TouchReport, Error<I::Error, RST::Error>> {
        let mut frame = [0u8; 6];
        self.io.read_registers(REG_DATA_START, &mut frame).await?;
        let mut report = TouchReport::from_frame(frame);
        let (x, y) = self
            .transform
            .apply(report.x, report.y, self.width, self.height);
        report.x = x;
        report.y = y;
        #[cfg(feature = "log")]
        log::trace!("cst816s touch: {report:?}");
        Ok(report)
    }

    /// Enter deep sleep (resets first; see blocking twin).
    pub async fn enter_deep_sleep(&mut self) -> Result<(), Error<I::Error, RST::Error>> {
        self.reset().await?;
        let reg = self.chip.unwrap_or(ChipId::Cst816s).sleep_register();
        self.io.write_register(reg.addr(), SLEEP_COMMAND).await?;
        Ok(())
    }

    /// Wake from deep sleep: full re-init.
    pub async fn wake(&mut self) -> Result<(), Error<I::Error, RST::Error>> {
        let _ = self.init().await?;
        Ok(())
    }

    /// Escape hatch: read any register.
    pub async fn read_register(
        &mut self,
        reg: Register,
    ) -> Result<u8, Error<I::Error, RST::Error>> {
        Ok(self.io.read_register(reg.addr()).await?)
    }

    /// Escape hatch: write any register.
    pub async fn write_register(
        &mut self,
        reg: Register,
        value: u8,
    ) -> Result<(), Error<I::Error, RST::Error>> {
        self.io.write_register(reg.addr(), value).await?;
        Ok(())
    }

    /// Write stored configuration (same fixed order as blocking twin).
    pub async fn apply_config(&mut self) -> Result<(), Error<I::Error, RST::Error>> {
        let c = self.config;
        self.set_motion_mask(c.motion_mask).await?;
        self.set_irq_pulse_width(c.irq_pulse_width).await?;
        self.set_normal_scan_period(c.normal_scan_period).await?;
        self.set_motion_slide_angle(c.motion_slide_angle).await?;
        self.set_auto_sleep_time(c.auto_sleep_time_s).await?;
        self.set_irq_ctl(c.irq_ctl).await?;
        self.set_auto_reset(c.auto_reset_s).await?;
        self.set_long_press_time(c.long_press_time_s).await?;
        self.set_disable_auto_sleep(c.disable_auto_sleep).await?;
        Ok(())
    }

    /// Write low-power profile (same burst as blocking twin).
    pub async fn apply_low_power_config(
        &mut self,
        lp: &LowPowerConfig,
    ) -> Result<(), Error<I::Error, RST::Error>> {
        check_range(lp.auto_wake_time_min, 1..=5).map_err(|()| Error::InvalidValue)?;
        if let Some(v) = lp.scan_raw1 {
            self.set_lp_scan_raw1(v).await?;
        }
        if let Some(v) = lp.scan_raw2 {
            self.set_lp_scan_raw2(v).await?;
        }
        let frame = [
            Register::LpAutoWakeTime.addr(),
            lp.auto_wake_time_min,
            lp.scan_threshold,
            lp.scan_window,
            lp.scan_frequency,
            lp.scan_idac,
        ];
        self.io.write_frame(&frame).await?;
        Ok(())
    }

    async fn get_u8(&mut self, reg: Register) -> Result<u8, Error<I::Error, RST::Error>> {
        Ok(self.io.read_register(reg.addr()).await?)
    }

    async fn set_u8(
        &mut self,
        reg: Register,
        value: u8,
    ) -> Result<(), Error<I::Error, RST::Error>> {
        self.io.write_register(reg.addr(), value).await?;
        Ok(())
    }

    /// Read MotionMask.
    pub async fn motion_mask(&mut self) -> Result<MotionMask, Error<I::Error, RST::Error>> {
        Ok(MotionMask::from_raw(
            self.get_u8(Register::MotionMask).await?,
        ))
    }
    /// Write MotionMask.
    pub async fn set_motion_mask(
        &mut self,
        value: MotionMask,
    ) -> Result<(), Error<I::Error, RST::Error>> {
        self.config.motion_mask = value;
        self.set_u8(Register::MotionMask, value.to_raw()).await
    }

    /// Read IRQ pulse width.
    pub async fn irq_pulse_width(&mut self) -> Result<u8, Error<I::Error, RST::Error>> {
        self.get_u8(Register::IrqPulseWidth).await
    }
    /// Write IRQ pulse width, 1..=200.
    pub async fn set_irq_pulse_width(
        &mut self,
        value: u8,
    ) -> Result<(), Error<I::Error, RST::Error>> {
        check_range(value, 1..=200).map_err(|()| Error::InvalidValue)?;
        self.config.irq_pulse_width = value;
        self.set_u8(Register::IrqPulseWidth, value).await
    }

    /// Read normal scan period.
    pub async fn normal_scan_period(&mut self) -> Result<u8, Error<I::Error, RST::Error>> {
        self.get_u8(Register::NormalScanPeriod).await
    }
    /// Write normal scan period, 1..=30.
    pub async fn set_normal_scan_period(
        &mut self,
        value: u8,
    ) -> Result<(), Error<I::Error, RST::Error>> {
        check_range(value, 1..=30).map_err(|()| Error::InvalidValue)?;
        self.config.normal_scan_period = value;
        self.set_u8(Register::NormalScanPeriod, value).await
    }

    /// Read motion slide angle.
    pub async fn motion_slide_angle(&mut self) -> Result<u8, Error<I::Error, RST::Error>> {
        self.get_u8(Register::MotionSlideAngle).await
    }
    /// Write motion slide angle.
    pub async fn set_motion_slide_angle(
        &mut self,
        value: u8,
    ) -> Result<(), Error<I::Error, RST::Error>> {
        self.config.motion_slide_angle = value;
        self.set_u8(Register::MotionSlideAngle, value).await
    }

    /// Read low-power scan reference 1 (big-endian H/L pair).
    pub async fn lp_scan_raw1(&mut self) -> Result<u16, Error<I::Error, RST::Error>> {
        let h = self.get_u8(Register::LpScanRaw1H).await?;
        let l = self.get_u8(Register::LpScanRaw1L).await?;
        Ok(u16::from_be_bytes([h, l]))
    }
    /// Write low-power scan reference 1 (big-endian H/L pair).
    pub async fn set_lp_scan_raw1(
        &mut self,
        value: u16,
    ) -> Result<(), Error<I::Error, RST::Error>> {
        self.set_u8(Register::LpScanRaw1H, (value >> 8) as u8)
            .await?;
        self.set_u8(Register::LpScanRaw1L, value as u8).await
    }

    /// Read low-power scan reference 2 (big-endian H/L pair).
    pub async fn lp_scan_raw2(&mut self) -> Result<u16, Error<I::Error, RST::Error>> {
        let h = self.get_u8(Register::LpScanRaw2H).await?;
        let l = self.get_u8(Register::LpScanRaw2L).await?;
        Ok(u16::from_be_bytes([h, l]))
    }
    /// Write low-power scan reference 2 (big-endian H/L pair).
    pub async fn set_lp_scan_raw2(
        &mut self,
        value: u16,
    ) -> Result<(), Error<I::Error, RST::Error>> {
        self.set_u8(Register::LpScanRaw2H, (value >> 8) as u8)
            .await?;
        self.set_u8(Register::LpScanRaw2L, value as u8).await
    }

    /// Read low-power auto-wake time (minutes).
    pub async fn lp_auto_wake_time(&mut self) -> Result<u8, Error<I::Error, RST::Error>> {
        self.get_u8(Register::LpAutoWakeTime).await
    }
    /// Write low-power auto-wake time, 1..=5 minutes.
    pub async fn set_lp_auto_wake_time(
        &mut self,
        value: u8,
    ) -> Result<(), Error<I::Error, RST::Error>> {
        check_range(value, 1..=5).map_err(|()| Error::InvalidValue)?;
        self.set_u8(Register::LpAutoWakeTime, value).await
    }

    /// Read low-power scan threshold.
    pub async fn lp_scan_threshold(&mut self) -> Result<u8, Error<I::Error, RST::Error>> {
        self.get_u8(Register::LpScanThreshold).await
    }
    /// Write low-power scan threshold.
    pub async fn set_lp_scan_threshold(
        &mut self,
        value: u8,
    ) -> Result<(), Error<I::Error, RST::Error>> {
        self.set_u8(Register::LpScanThreshold, value).await
    }

    /// Read low-power scan window.
    pub async fn lp_scan_window(&mut self) -> Result<u8, Error<I::Error, RST::Error>> {
        self.get_u8(Register::LpScanWindow).await
    }
    /// Write low-power scan window (documented 0..=3; all accepted).
    pub async fn set_lp_scan_window(
        &mut self,
        value: u8,
    ) -> Result<(), Error<I::Error, RST::Error>> {
        self.set_u8(Register::LpScanWindow, value).await
    }

    /// Read low-power scan frequency.
    pub async fn lp_scan_frequency(&mut self) -> Result<u8, Error<I::Error, RST::Error>> {
        self.get_u8(Register::LpScanFrequency).await
    }
    /// Write low-power scan frequency.
    pub async fn set_lp_scan_frequency(
        &mut self,
        value: u8,
    ) -> Result<(), Error<I::Error, RST::Error>> {
        self.set_u8(Register::LpScanFrequency, value).await
    }

    /// Read low-power scan IDAC.
    pub async fn lp_scan_idac(&mut self) -> Result<u8, Error<I::Error, RST::Error>> {
        self.get_u8(Register::LpScanIdac).await
    }
    /// Write low-power scan IDAC.
    pub async fn set_lp_scan_idac(&mut self, value: u8) -> Result<(), Error<I::Error, RST::Error>> {
        self.set_u8(Register::LpScanIdac, value).await
    }

    /// Read auto sleep time (seconds).
    pub async fn auto_sleep_time(&mut self) -> Result<u8, Error<I::Error, RST::Error>> {
        self.get_u8(Register::AutoSleepTime).await
    }
    /// Write auto sleep time (seconds).
    pub async fn set_auto_sleep_time(
        &mut self,
        value: u8,
    ) -> Result<(), Error<I::Error, RST::Error>> {
        self.config.auto_sleep_time_s = value;
        self.set_u8(Register::AutoSleepTime, value).await
    }

    /// Read IRQ control.
    pub async fn irq_ctl(&mut self) -> Result<IrqCtl, Error<I::Error, RST::Error>> {
        Ok(IrqCtl::from_raw(self.get_u8(Register::IrqCtl).await?))
    }
    /// Write IRQ control.
    pub async fn set_irq_ctl(&mut self, value: IrqCtl) -> Result<(), Error<I::Error, RST::Error>> {
        self.config.irq_ctl = value;
        self.set_u8(Register::IrqCtl, value.to_raw()).await
    }

    /// Write a [`ReportMode`] preset to IrqCtl.
    pub async fn set_report_mode(
        &mut self,
        mode: ReportMode,
    ) -> Result<(), Error<I::Error, RST::Error>> {
        let raw = mode.irq_ctl_raw();
        self.config.irq_ctl = IrqCtl::from_raw(raw);
        self.set_u8(Register::IrqCtl, raw).await
    }

    /// Read auto reset time (seconds).
    pub async fn auto_reset(&mut self) -> Result<u8, Error<I::Error, RST::Error>> {
        self.get_u8(Register::AutoReset).await
    }
    /// Write auto reset time (seconds; 0 disables).
    pub async fn set_auto_reset(&mut self, value: u8) -> Result<(), Error<I::Error, RST::Error>> {
        self.config.auto_reset_s = value;
        self.set_u8(Register::AutoReset, value).await
    }

    /// Read long-press time (seconds).
    pub async fn long_press_time(&mut self) -> Result<u8, Error<I::Error, RST::Error>> {
        self.get_u8(Register::LongPressTime).await
    }
    /// Write long-press time (seconds).
    pub async fn set_long_press_time(
        &mut self,
        value: u8,
    ) -> Result<(), Error<I::Error, RST::Error>> {
        self.config.long_press_time_s = value;
        self.set_u8(Register::LongPressTime, value).await
    }

    /// Read IO control.
    pub async fn io_ctl(&mut self) -> Result<IoCtl, Error<I::Error, RST::Error>> {
        Ok(IoCtl::from_raw(self.get_u8(Register::IoCtl).await?))
    }
    /// Write IO control.
    pub async fn set_io_ctl(&mut self, value: IoCtl) -> Result<(), Error<I::Error, RST::Error>> {
        self.set_u8(Register::IoCtl, value.to_raw()).await
    }

    /// Read disable-auto-sleep flag.
    pub async fn disable_auto_sleep(&mut self) -> Result<bool, Error<I::Error, RST::Error>> {
        Ok(self.get_u8(Register::DisableAutoSleep).await? != 0)
    }
    /// Write disable-auto-sleep flag.
    pub async fn set_disable_auto_sleep(
        &mut self,
        value: bool,
    ) -> Result<(), Error<I::Error, RST::Error>> {
        self.config.disable_auto_sleep = value;
        self.set_u8(Register::DisableAutoSleep, u8::from(value))
            .await
    }
}

impl<I, D, RST, INT> Cst816sAsync<I, D, RST, INT>
where
    I: I2c,
    D: DelayNs,
    RST: OutputPin,
    INT: Wait,
{
    /// Wait for the INT pin to go low (chip signals with low pulses).
    ///
    /// Error-type note: this method (and `wait_for_touch`) use
    /// `Error<I::Error, INT::Error>` — the pin in question is INT,
    /// not RST. All other methods use `Error<I::Error, RST::Error>`.
    pub async fn wait_for_interrupt(&mut self) -> Result<(), Error<I::Error, INT::Error>> {
        self.int.wait_for_low().await.map_err(Error::Pin)
    }

    /// Wait for INT low, then read a touch frame. See
    /// [`wait_for_interrupt`](Self::wait_for_interrupt) for the error
    /// type split: bus/reset errors from the read would need
    /// `RST::Error`, so the read's pin channel is unused here — I2C
    /// errors surface as `Error::Bus`, INT errors as `Error::Pin`.
    pub async fn wait_for_touch(&mut self) -> Result<TouchReport, Error<I::Error, INT::Error>> {
        self.wait_for_interrupt().await?;
        let mut frame = [0u8; 6];
        self.io
            .read_registers(REG_DATA_START, &mut frame)
            .await
            .map_err(Error::Bus)?;
        let mut report = TouchReport::from_frame(frame);
        let (x, y) = self
            .transform
            .apply(report.x, report.y, self.width, self.height);
        report.x = x;
        report.y = y;
        #[cfg(feature = "log")]
        log::trace!("cst816s touch: {report:?}");
        Ok(report)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_helpers::{FakeDelay, FakeI2c, FakePin};
    use futures::executor::block_on;

    type TestDriver = Cst816sAsync<FakeI2c, FakeDelay, FakePin, FakePin>;

    fn driver(bus: FakeI2c) -> TestDriver {
        Cst816sAsync::new(bus, FakeDelay::new())
            .with_reset_pin(FakePin::new())
            .with_interrupt_pin(FakePin::new())
    }

    #[test]
    fn async_init_and_wait_for_touch() {
        let bus = FakeI2c::new().with_script(&[
            &[0xB4],
            &[0x01],
            &[0x02],
            &[0x05, 0x01, 0x01, 0x20, 0x02, 0x40],
        ]);
        let mut d = driver(bus);
        let (info, report) = block_on(async {
            let info = d.init().await.unwrap();
            let report = d.wait_for_touch().await.unwrap();
            (info, report)
        });
        assert_eq!(info.chip, ChipId::Cst816s);
        assert_eq!((report.x, report.y), (288, 576));
        let (bus, delay, rst, int) = d.release();
        assert_eq!(int.waits, 1);
        assert_eq!(rst.levels, vec![false, true]);
        assert_eq!(delay.total_ms(), 110);
        assert_eq!(bus.reads, vec![0xA7, 0xA9, 0xA8, 0x01]);
        assert_eq!(bus.writes.len(), 9);
    }
}
