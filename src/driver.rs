//! Blocking CST816 driver.

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use embedded_hal::i2c::I2c;

use crate::config::{Config, IoCtl, IrqCtl, LowPowerConfig, MotionMask, ReportMode};
use crate::data::{ChipId, CoordinateTransform, DeviceInfo, TouchReport};
use crate::error::Error;
use crate::interface::{DeviceAddress, I2cInterface, NoPin, RegisterIo};
use crate::registers::{REG_DATA_START, RESET_LOW_MS, RESET_SETTLE_MS, Register, SLEEP_COMMAND};

#[inline]
fn check_range(value: u8, range: core::ops::RangeInclusive<u8>) -> Result<(), ()> {
    if range.contains(&value) {
        Ok(())
    } else {
        Err(())
    }
}

/// Blocking CST816 driver over I2C.
///
/// `RST`/`INT` default to [`NoPin`]; attach real pins with
/// [`with_reset_pin`](Self::with_reset_pin) /
/// [`with_interrupt_pin`](Self::with_interrupt_pin).
pub struct Cst816s<I, D, RST = NoPin, INT = NoPin> {
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

impl<I, D, RST, INT> core::fmt::Debug for Cst816s<I, D, RST, INT> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Cst816s")
            .field("chip", &self.chip)
            .field("config", &self.config)
            .field("transform", &self.transform)
            .field("width", &self.width)
            .field("height", &self.height)
            .finish_non_exhaustive()
    }
}

impl<I, D> Cst816s<I, D, NoPin, NoPin> {
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

impl<I, D, RST, INT> Cst816s<I, D, RST, INT> {
    /// Attach a reset pin (retypes the driver).
    pub fn with_reset_pin<R2: OutputPin>(self, pin: R2) -> Cst816s<I, D, R2, INT> {
        Cst816s {
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
    pub fn with_interrupt_pin<N2>(self, pin: N2) -> Cst816s<I, D, RST, N2> {
        Cst816s {
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

impl<I, D, RST, INT> Cst816s<I, D, RST, INT>
where
    I: I2c,
    D: DelayNs,
    RST: OutputPin,
{
    /// Pulse RST low for 10 ms, release, settle 100 ms.
    pub fn reset(&mut self) -> Result<(), Error<I::Error, RST::Error>> {
        self.rst.set_low().map_err(Error::Pin)?;
        self.delay.delay_ms(RESET_LOW_MS);
        self.rst.set_high().map_err(Error::Pin)?;
        self.delay.delay_ms(RESET_SETTLE_MS);
        Ok(())
    }

    /// Identify the chip. Unknown IDs are not an error; only bus
    /// errors fail. Stores the chip for sleep-register selection.
    pub fn probe(&mut self) -> Result<DeviceInfo, Error<I::Error, RST::Error>> {
        let chip_raw = self.io.read_register(Register::ChipId.addr())?;
        let (chip, firmware_version) = if chip_raw != 0x00 {
            let fw = self.io.read_register(Register::FirmwareVersion.addr())?;
            (ChipId::from_chip_register(chip_raw), fw)
        } else {
            let factory = self.io.read_register(Register::FactoryId.addr())?;
            let fw = self.io.read_register(Register::FirmwareVersionAlt.addr())?;
            (ChipId::from_factory_register(factory), fw)
        };
        let project_id = self.io.read_register(Register::ProjectId.addr())?;
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
    pub fn init(&mut self) -> Result<DeviceInfo, Error<I::Error, RST::Error>> {
        self.reset()?;
        let info = self.probe()?;
        self.apply_config()?;
        Ok(info)
    }

    /// Read one touch frame; coordinates pass through the configured
    /// transform. Returned unfiltered.
    pub fn read_touch(&mut self) -> Result<TouchReport, Error<I::Error, RST::Error>> {
        let mut frame = [0u8; 6];
        self.io.read_registers(REG_DATA_START, &mut frame)?;
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

    /// Enter deep sleep. Resets first: in low-power auto-sleep the
    /// I2C interface NACKs (vendor debug doc §3).
    pub fn enter_deep_sleep(&mut self) -> Result<(), Error<I::Error, RST::Error>> {
        self.reset()?;
        let reg = self.chip.unwrap_or(ChipId::Cst816s).sleep_register();
        self.io.write_register(reg.addr(), SLEEP_COMMAND)?;
        Ok(())
    }

    /// Wake from deep sleep: full re-init (reset restores register
    /// defaults, so configuration is re-applied).
    pub fn wake(&mut self) -> Result<(), Error<I::Error, RST::Error>> {
        let _ = self.init()?;
        Ok(())
    }

    /// Escape hatch: read any register.
    pub fn read_register(&mut self, reg: Register) -> Result<u8, Error<I::Error, RST::Error>> {
        Ok(self.io.read_register(reg.addr())?)
    }

    /// Escape hatch: write any register.
    pub fn write_register(
        &mut self,
        reg: Register,
        value: u8,
    ) -> Result<(), Error<I::Error, RST::Error>> {
        self.io.write_register(reg.addr(), value)?;
        Ok(())
    }

    /// Write the stored configuration, in the fixed order:
    /// MotionMask, IrqPulseWidth, NormalScanPeriod, MotionSlideAngle,
    /// AutoSleepTime, IrqCtl, AutoReset, LongPressTime,
    /// DisableAutoSleep. (IoCtl excluded on purpose.)
    pub fn apply_config(&mut self) -> Result<(), Error<I::Error, RST::Error>> {
        let c = self.config;
        self.set_motion_mask(c.motion_mask)?;
        self.set_irq_pulse_width(c.irq_pulse_width)?;
        self.set_normal_scan_period(c.normal_scan_period)?;
        self.set_motion_slide_angle(c.motion_slide_angle)?;
        self.set_auto_sleep_time(c.auto_sleep_time_s)?;
        self.set_irq_ctl(c.irq_ctl)?;
        self.set_auto_reset(c.auto_reset_s)?;
        self.set_long_press_time(c.long_press_time_s)?;
        self.set_disable_auto_sleep(c.disable_auto_sleep)?;
        Ok(())
    }

    /// Write the low-power profile: calibrated scan references only
    /// if set, then one 5-byte burst from LpAutoWakeTime.
    pub fn apply_low_power_config(
        &mut self,
        lp: &LowPowerConfig,
    ) -> Result<(), Error<I::Error, RST::Error>> {
        check_range(lp.auto_wake_time_min, 1..=5).map_err(|()| Error::InvalidValue)?;
        if let Some(v) = lp.scan_raw1 {
            self.set_lp_scan_raw1(v)?;
        }
        if let Some(v) = lp.scan_raw2 {
            self.set_lp_scan_raw2(v)?;
        }
        let frame = [
            Register::LpAutoWakeTime.addr(),
            lp.auto_wake_time_min,
            lp.scan_threshold,
            lp.scan_window,
            lp.scan_frequency,
            lp.scan_idac,
        ];
        self.io.write_frame(&frame)?;
        Ok(())
    }

    fn get_u8(&mut self, reg: Register) -> Result<u8, Error<I::Error, RST::Error>> {
        Ok(self.io.read_register(reg.addr())?)
    }

    fn set_u8(&mut self, reg: Register, value: u8) -> Result<(), Error<I::Error, RST::Error>> {
        self.io.write_register(reg.addr(), value)?;
        Ok(())
    }

    /// Read MotionMask.
    pub fn motion_mask(&mut self) -> Result<MotionMask, Error<I::Error, RST::Error>> {
        Ok(MotionMask::from_raw(self.get_u8(Register::MotionMask)?))
    }
    /// Write MotionMask.
    pub fn set_motion_mask(
        &mut self,
        value: MotionMask,
    ) -> Result<(), Error<I::Error, RST::Error>> {
        self.config.motion_mask = value;
        self.set_u8(Register::MotionMask, value.to_raw())
    }

    /// Read IRQ pulse width.
    pub fn irq_pulse_width(&mut self) -> Result<u8, Error<I::Error, RST::Error>> {
        self.get_u8(Register::IrqPulseWidth)
    }
    /// Write IRQ pulse width, 1..=200.
    pub fn set_irq_pulse_width(&mut self, value: u8) -> Result<(), Error<I::Error, RST::Error>> {
        check_range(value, 1..=200).map_err(|()| Error::InvalidValue)?;
        self.config.irq_pulse_width = value;
        self.set_u8(Register::IrqPulseWidth, value)
    }

    /// Read normal scan period.
    pub fn normal_scan_period(&mut self) -> Result<u8, Error<I::Error, RST::Error>> {
        self.get_u8(Register::NormalScanPeriod)
    }
    /// Write normal scan period, 1..=30.
    pub fn set_normal_scan_period(&mut self, value: u8) -> Result<(), Error<I::Error, RST::Error>> {
        check_range(value, 1..=30).map_err(|()| Error::InvalidValue)?;
        self.config.normal_scan_period = value;
        self.set_u8(Register::NormalScanPeriod, value)
    }

    /// Read motion slide angle.
    pub fn motion_slide_angle(&mut self) -> Result<u8, Error<I::Error, RST::Error>> {
        self.get_u8(Register::MotionSlideAngle)
    }
    /// Write motion slide angle.
    pub fn set_motion_slide_angle(&mut self, value: u8) -> Result<(), Error<I::Error, RST::Error>> {
        self.config.motion_slide_angle = value;
        self.set_u8(Register::MotionSlideAngle, value)
    }

    /// Read low-power scan reference 1 (big-endian H/L pair).
    pub fn lp_scan_raw1(&mut self) -> Result<u16, Error<I::Error, RST::Error>> {
        let h = self.get_u8(Register::LpScanRaw1H)?;
        let l = self.get_u8(Register::LpScanRaw1L)?;
        Ok(u16::from_be_bytes([h, l]))
    }
    /// Write low-power scan reference 1 (big-endian H/L pair).
    pub fn set_lp_scan_raw1(&mut self, value: u16) -> Result<(), Error<I::Error, RST::Error>> {
        self.set_u8(Register::LpScanRaw1H, (value >> 8) as u8)?;
        self.set_u8(Register::LpScanRaw1L, value as u8)
    }

    /// Read low-power scan reference 2 (big-endian H/L pair).
    pub fn lp_scan_raw2(&mut self) -> Result<u16, Error<I::Error, RST::Error>> {
        let h = self.get_u8(Register::LpScanRaw2H)?;
        let l = self.get_u8(Register::LpScanRaw2L)?;
        Ok(u16::from_be_bytes([h, l]))
    }
    /// Write low-power scan reference 2 (big-endian H/L pair).
    pub fn set_lp_scan_raw2(&mut self, value: u16) -> Result<(), Error<I::Error, RST::Error>> {
        self.set_u8(Register::LpScanRaw2H, (value >> 8) as u8)?;
        self.set_u8(Register::LpScanRaw2L, value as u8)
    }

    /// Read low-power auto-wake time (minutes).
    pub fn lp_auto_wake_time(&mut self) -> Result<u8, Error<I::Error, RST::Error>> {
        self.get_u8(Register::LpAutoWakeTime)
    }
    /// Write low-power auto-wake time, 1..=5 minutes.
    pub fn set_lp_auto_wake_time(&mut self, value: u8) -> Result<(), Error<I::Error, RST::Error>> {
        check_range(value, 1..=5).map_err(|()| Error::InvalidValue)?;
        self.set_u8(Register::LpAutoWakeTime, value)
    }

    /// Read low-power scan threshold.
    pub fn lp_scan_threshold(&mut self) -> Result<u8, Error<I::Error, RST::Error>> {
        self.get_u8(Register::LpScanThreshold)
    }
    /// Write low-power scan threshold.
    pub fn set_lp_scan_threshold(&mut self, value: u8) -> Result<(), Error<I::Error, RST::Error>> {
        self.set_u8(Register::LpScanThreshold, value)
    }

    /// Read low-power scan window.
    pub fn lp_scan_window(&mut self) -> Result<u8, Error<I::Error, RST::Error>> {
        self.get_u8(Register::LpScanWindow)
    }
    /// Write low-power scan window. Documented 0..=3, but values
    /// above 3 are observed working (Zephyr binding); all accepted.
    pub fn set_lp_scan_window(&mut self, value: u8) -> Result<(), Error<I::Error, RST::Error>> {
        self.set_u8(Register::LpScanWindow, value)
    }

    /// Read low-power scan frequency.
    pub fn lp_scan_frequency(&mut self) -> Result<u8, Error<I::Error, RST::Error>> {
        self.get_u8(Register::LpScanFrequency)
    }
    /// Write low-power scan frequency.
    pub fn set_lp_scan_frequency(&mut self, value: u8) -> Result<(), Error<I::Error, RST::Error>> {
        self.set_u8(Register::LpScanFrequency, value)
    }

    /// Read low-power scan IDAC.
    pub fn lp_scan_idac(&mut self) -> Result<u8, Error<I::Error, RST::Error>> {
        self.get_u8(Register::LpScanIdac)
    }
    /// Write low-power scan IDAC.
    pub fn set_lp_scan_idac(&mut self, value: u8) -> Result<(), Error<I::Error, RST::Error>> {
        self.set_u8(Register::LpScanIdac, value)
    }

    /// Read auto sleep time (seconds).
    pub fn auto_sleep_time(&mut self) -> Result<u8, Error<I::Error, RST::Error>> {
        self.get_u8(Register::AutoSleepTime)
    }
    /// Write auto sleep time (seconds).
    pub fn set_auto_sleep_time(&mut self, value: u8) -> Result<(), Error<I::Error, RST::Error>> {
        self.config.auto_sleep_time_s = value;
        self.set_u8(Register::AutoSleepTime, value)
    }

    /// Read IRQ control.
    pub fn irq_ctl(&mut self) -> Result<IrqCtl, Error<I::Error, RST::Error>> {
        Ok(IrqCtl::from_raw(self.get_u8(Register::IrqCtl)?))
    }
    /// Write IRQ control.
    pub fn set_irq_ctl(&mut self, value: IrqCtl) -> Result<(), Error<I::Error, RST::Error>> {
        self.config.irq_ctl = value;
        self.set_u8(Register::IrqCtl, value.to_raw())
    }

    /// Write a [`ReportMode`] preset to IrqCtl. Verify via
    /// [`irq_ctl`](Self::irq_ctl); there is no mode getter.
    pub fn set_report_mode(&mut self, mode: ReportMode) -> Result<(), Error<I::Error, RST::Error>> {
        let raw = mode.irq_ctl_raw();
        self.config.irq_ctl = IrqCtl::from_raw(raw);
        self.set_u8(Register::IrqCtl, raw)
    }

    /// Read auto reset time (seconds).
    pub fn auto_reset(&mut self) -> Result<u8, Error<I::Error, RST::Error>> {
        self.get_u8(Register::AutoReset)
    }
    /// Write auto reset time (seconds; 0 disables).
    pub fn set_auto_reset(&mut self, value: u8) -> Result<(), Error<I::Error, RST::Error>> {
        self.config.auto_reset_s = value;
        self.set_u8(Register::AutoReset, value)
    }

    /// Read long-press time (seconds).
    pub fn long_press_time(&mut self) -> Result<u8, Error<I::Error, RST::Error>> {
        self.get_u8(Register::LongPressTime)
    }
    /// Write long-press time (seconds).
    pub fn set_long_press_time(&mut self, value: u8) -> Result<(), Error<I::Error, RST::Error>> {
        self.config.long_press_time_s = value;
        self.set_u8(Register::LongPressTime, value)
    }

    /// Read IO control.
    pub fn io_ctl(&mut self) -> Result<IoCtl, Error<I::Error, RST::Error>> {
        Ok(IoCtl::from_raw(self.get_u8(Register::IoCtl)?))
    }
    /// Write IO control.
    pub fn set_io_ctl(&mut self, value: IoCtl) -> Result<(), Error<I::Error, RST::Error>> {
        self.set_u8(Register::IoCtl, value.to_raw())
    }

    /// Read disable-auto-sleep flag (`raw != 0`).
    pub fn disable_auto_sleep(&mut self) -> Result<bool, Error<I::Error, RST::Error>> {
        Ok(self.get_u8(Register::DisableAutoSleep)? != 0)
    }
    /// Write disable-auto-sleep flag (0x01/0x00).
    pub fn set_disable_auto_sleep(
        &mut self,
        value: bool,
    ) -> Result<(), Error<I::Error, RST::Error>> {
        self.config.disable_auto_sleep = value;
        self.set_u8(Register::DisableAutoSleep, u8::from(value))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::data::{Gesture, TouchEventKind};
    use crate::test_helpers::{FakeDelay, FakeI2c, FakePin};

    type TestDriver = Cst816s<FakeI2c, FakeDelay, FakePin, FakePin>;

    fn driver(bus: FakeI2c) -> TestDriver {
        Cst816s::new(bus, FakeDelay::new())
            .with_reset_pin(FakePin::new())
            .with_interrupt_pin(FakePin::new())
    }

    #[test]
    fn probe_chip_register_path() {
        let bus = FakeI2c::new().with_script(&[&[0xB4], &[0x03], &[0x42]]);
        let mut d = driver(bus);
        let info = d.probe().unwrap();
        assert_eq!(info.chip, ChipId::Cst816s);
        assert_eq!(info.firmware_version, 0x03);
        assert_eq!(info.project_id, 0x42);
        let (bus, ..) = d.release();
        assert_eq!(bus.reads, vec![0xA7, 0xA9, 0xA8]);
    }

    #[test]
    fn probe_factory_fallback_path() {
        let bus = FakeI2c::new().with_script(&[&[0x00], &[0x11], &[0x07], &[0x09]]);
        let mut d = driver(bus);
        let info = d.probe().unwrap();
        assert_eq!(info.chip, ChipId::Cst826);
        assert_eq!(info.firmware_version, 0x07);
        assert_eq!(info.project_id, 0x09);
        let (bus, ..) = d.release();
        assert_eq!(bus.reads, vec![0xA7, 0xAA, 0xA6, 0xA8]);
    }

    #[test]
    fn deep_sleep_resets_before_sleep_write() {
        let mut d = driver(FakeI2c::new());
        d.enter_deep_sleep().unwrap();
        let (bus, delay, rst, _) = d.release();
        assert_eq!(rst.levels, vec![false, true]);
        assert_eq!(delay.total_ms(), 110);
        assert_eq!(bus.writes, vec![(0x15, vec![0xE5, 0x03])]);
    }

    #[test]
    fn read_touch_parses_frame() {
        let bus = FakeI2c::new().with_script(&[&[0x05, 0x01, 0x01, 0x20, 0x02, 0x40]]);
        let mut d = driver(bus);
        let r = d.read_touch().unwrap();
        assert_eq!(r.gesture, Gesture::SingleClick);
        assert_eq!(r.event, TouchEventKind::Down);
        assert_eq!((r.x, r.y), (288, 576));
        assert!(r.pressed);
        let (bus, ..) = d.release();
        assert_eq!(bus.reads, vec![0x01]);
    }

    #[test]
    fn init_transcript() {
        let bus = FakeI2c::new().with_script(&[&[0xB4], &[0x01], &[0x02]]);
        let mut d = driver(bus);
        let info = d.init().unwrap();
        assert_eq!(info.chip, ChipId::Cst816s);
        let (bus, delay, rst, _) = d.release();
        assert_eq!(rst.levels, vec![false, true]);
        assert_eq!(delay.total_ms(), 110);
        assert_eq!(bus.reads, vec![0xA7, 0xA9, 0xA8]);
        // apply_config defaults, exact order.
        let expected: Vec<(u8, Vec<u8>)> = [
            (0xEC, 0x00), // MotionMask
            (0xED, 10),   // IrqPulseWidth
            (0xEE, 1),    // NormalScanPeriod
            (0xEF, 0),    // MotionSlideAngle
            (0xF9, 2),    // AutoSleepTime
            (0xFA, 0x70), // IrqCtl
            (0xFB, 5),    // AutoReset
            (0xFC, 10),   // LongPressTime
            (0xFE, 0),    // DisableAutoSleep
        ]
        .iter()
        .map(|(r, v)| (0x15, vec![*r, *v]))
        .collect();
        assert_eq!(bus.writes, expected);
    }

    #[test]
    fn validation_rejects_without_traffic() {
        let mut d = driver(FakeI2c::new());
        assert_eq!(d.set_irq_pulse_width(0), Err(Error::InvalidValue));
        assert_eq!(d.set_irq_pulse_width(201), Err(Error::InvalidValue));
        assert_eq!(d.set_normal_scan_period(0), Err(Error::InvalidValue));
        assert_eq!(d.set_normal_scan_period(31), Err(Error::InvalidValue));
        assert_eq!(d.set_lp_auto_wake_time(0), Err(Error::InvalidValue));
        assert_eq!(d.set_lp_auto_wake_time(6), Err(Error::InvalidValue));
        let bad_lp = LowPowerConfig {
            auto_wake_time_min: 9,
            ..LowPowerConfig::default()
        };
        assert_eq!(d.apply_low_power_config(&bad_lp), Err(Error::InvalidValue));
        let (bus, ..) = d.release();
        assert_eq!(bus.writes.len(), 0);
    }

    #[test]
    fn lp_scan_raw_big_endian() {
        let mut d = driver(FakeI2c::new());
        d.set_lp_scan_raw1(0x1234).unwrap();
        let (bus, ..) = d.release();
        assert_eq!(
            bus.writes,
            vec![(0x15, vec![0xF0, 0x12]), (0x15, vec![0xF1, 0x34])]
        );
    }

    #[test]
    fn low_power_burst_frame() {
        let mut d = driver(FakeI2c::new());
        let lp = LowPowerConfig {
            scan_raw1: Some(0x00FF),
            ..LowPowerConfig::default()
        };
        d.apply_low_power_config(&lp).unwrap();
        let (bus, ..) = d.release();
        assert_eq!(
            bus.writes,
            vec![
                (0x15, vec![0xF0, 0x00]),
                (0x15, vec![0xF1, 0xFF]),
                (0x15, vec![0xF4, 5, 48, 3, 7, 200]),
            ]
        );
    }
}
