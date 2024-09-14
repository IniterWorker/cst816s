use embedded_hal::{
    delay::DelayNs,
    digital::OutputPin,
    i2c::{I2c, SevenBitAddress},
};

use crate::command::{constants, IrqCtl, MotionMask, Register, TouchEvent};

#[repr(u8)]
#[derive(Debug, Clone, Copy)]
pub enum DeviceAddress {
    Primary = 0x15,
}

impl From<DeviceAddress> for u8 {
    fn from(value: DeviceAddress) -> Self {
        match value {
            DeviceAddress::Primary => 0x15,
        }
    }
}

impl Default for DeviceAddress {
    fn default() -> Self {
        Self::Primary
    }
}

/// `Cst816s` driver's `Error`
#[derive(Debug)]
#[cfg_attr(feature = "std", derive(thiserror::Error))]
pub enum Error<E> {
    /// Error related to the driver communication
    #[cfg_attr(feature = "std", error("Error communication: {0:?}"))]
    Communication(E),
}

impl<E> From<E> for Error<E> {
    fn from(value: E) -> Self {
        Self::Communication(value)
    }
}

/// Cst816s Driver
pub struct Cst816s<I, D> {
    /// I2C Interface
    pub(crate) interface: I,
    /// I2C Address
    pub(crate) addr: DeviceAddress,
    /// Delay
    pub(crate) _delay: D,
}

impl<I, D> Cst816s<I, D>
where
    I: I2c<SevenBitAddress>,
    D: DelayNs,
{
    /// Reset the IC.
    ///
    ///
    /// # Errors
    ///
    /// See `OutputPin` definition for more information.
    #[allow(clippy::needless_pass_by_ref_mut)]
    pub fn reset<RST, DELAY>(&mut self, rst: &mut RST, delay: &mut DELAY) -> Result<(), RST::Error>
    where
        RST: OutputPin,
        DELAY: DelayNs,
    {
        rst.set_low()?;
        delay.delay_ms(constants::CST78XX_RESET_DURATION_LOW_MS);
        rst.set_high()?;
        delay.delay_ms(constants::CST78XX_RESET_DURATION_HIGH_MS);
        Ok(())
    }

    /// Wake UP the IC
    ///
    /// # Errors
    ///
    /// See `OutputPin` definition for more information.
    #[allow(clippy::needless_pass_by_ref_mut)]
    pub fn wake_up<RST, DELAY>(
        &mut self,
        rst: &mut RST,
        delay: &mut DELAY,
    ) -> Result<(), RST::Error>
    where
        RST: OutputPin,
        DELAY: DelayNs,
    {
        self.reset(rst, delay)
    }
}

impl<I, D> Cst816s<I, D>
where
    I: I2c<SevenBitAddress>,
    D: DelayNs,
{
    /// Create a new Cst816s driver with default I2C address.
    #[allow(clippy::missing_const_for_fn)]
    pub fn new(interface: I, delay: D) -> Self {
        Self {
            interface,
            addr: DeviceAddress::Primary,
            _delay: delay,
        }
    }

    /// Give back the I2C interface
    pub fn release(self) -> I {
        self.interface
    }

    #[cfg(not(feature = "loglib"))]
    #[allow(clippy::needless_pass_by_ref_mut)]
    pub fn dump_register(&mut self) {}

    #[cfg(feature = "loglib")]
    pub fn dump_register(&mut self) {
        log::info!("read_motion_mask: {:?}", self.read_motion_mask());
        log::info!("read_irq_pulse_width: {:?}", self.read_irq_pulse_width());
        log::info!("read_nor_scan_per: {:?}", self.read_nor_scan_per());
        log::info!("read_motion_s1_angle: {:?}", self.read_motion_s1_angle());
        log::info!("read_lp_scan_raw_1: {:?}", self.read_lp_scan_raw_1());
        log::info!("read_lp_scan_raw_2: {:?}", self.read_lp_scan_raw_2());
        log::info!("read_lp_auto_wakeup: {:?}", self.read_lp_auto_wakeup());
        log::info!("read_lp_scan_th: {:?}", self.read_lp_scan_th());
        log::info!("read_lp_scan_win: {:?}", self.read_lp_scan_win());
        log::info!("read_lp_scan_freq: {:?}", self.read_lp_scan_freq());
        log::info!("read_lp_scan_idac: {:?}", self.read_lp_scan_idac());
        log::info!("read_auto_sleep_time: {:?}", self.read_auto_sleep_time());
        log::info!("read_irq_ctl: {:?}", self.read_irq_ctl());
        log::info!("read_auto_reset: {:?}", self.read_auto_reset());
        log::info!("read_long_press_time: {:?}", self.read_long_press_time());
        log::info!(
            "read_disable_auto_sleep: {:?}",
            self.read_disable_auto_sleep()
        );
    }

    /// Sets the Deep Sleep Mode
    ///
    /// # Errors
    ///
    /// This method may return an error if there are communication issues with the sensor.
    pub fn deep_sleep(&mut self) -> Result<(), Error<I::Error>> {
        Register::DeepSleep.write(
            &mut self.interface,
            self.addr.into(),
            constants::CST816S_REG_DEEP_SLEEP,
        )?;
        Ok(())
    }

    /// Returns Motion Mask
    ///
    /// # Errors
    ///
    /// This method may return an error if there are communication issues with the sensor.
    pub fn read_motion_mask(&mut self) -> Result<MotionMask, Error<I::Error>> {
        let data = Register::MotionMask.read(&mut self.interface, self.addr.into())?;
        Ok(MotionMask(data))
    }

    /// Returns Interrupt low-pluse output width
    /// Unit: 0.1ms, Range: 1～200, Default:10
    ///
    /// # Errors
    ///
    /// This method may return an error if there are communication issues with the sensor.
    pub fn read_irq_pulse_width(&mut self) -> Result<u8, Error<I::Error>> {
        Ok(Register::IrqPluseWidth.read(&mut self.interface, self.addr.into())?)
    }

    /// Writes Interrupt low-pluse output width
    /// Unit: 0.1ms, Range: 1～200, Default:10
    ///
    /// # Errors
    ///
    /// This method may return an error if there are communication issues with the sensor.
    pub fn write_irq_pulse_width(&mut self, value: u8) -> Result<(), Error<I::Error>> {
        Register::IrqPluseWidth.write(&mut self.interface, self.addr.into(), value)?;
        Ok(())
    }

    /// Returns Normal quick-scanning Period
    /// This value will affect `LpAutoWakeTimeandAutoSleepTime`. Unit: 10ms, Range: 1～30,Default: 1
    ///
    /// # Errors
    ///
    /// This method may return an error if there are communication issues with the sensor.
    pub fn read_nor_scan_per(&mut self) -> Result<u8, Error<I::Error>> {
        Ok(Register::NorScanPer.read(&mut self.interface, self.addr.into())?)
    }

    /// Writes  Normal quick-scanning Period
    /// This value will affect `LpAutoWakeTimeandAutoSleepTime`. Unit: 10ms, Range: 1～30,Default: 1
    ///
    /// # Errors
    ///
    /// This method may return an error if there are communication issues with the sensor.
    pub fn write_nor_scan_per(&mut self, value: u8) -> Result<(), Error<I::Error>> {
        Register::NorScanPer.write(&mut self.interface, self.addr.into(), value)?;
        Ok(())
    }

    /// Returns Gesture detection sliding area angle control.
    /// Angle=tan(c)*10 where c is the angle withrespect to the positive x-axis.
    ///
    /// # Errors
    ///
    /// This method may return an error if there are communication issues with the sensor.
    pub fn read_motion_s1_angle(&mut self) -> Result<u8, Error<I::Error>> {
        Ok(Register::MotionSlAngle.read(&mut self.interface, self.addr.into())?)
    }

    /// Writes Gesture detection sliding area angle control.
    /// Angle=tan(c)*10 where c is the angle withrespect to the positive x-axis.
    ///
    /// # Errors
    ///
    /// This method may return an error if there are communication issues with the sensor.
    pub fn write_motion_s1_angle(&mut self, value: u8) -> Result<(), Error<I::Error>> {
        Register::MotionSlAngle.write(&mut self.interface, self.addr.into(), value)?;
        Ok(())
    }

    /// Returns `LpScanRaw1` the the reference value for low-power scanning channel 2
    ///
    /// # Errors
    ///
    /// This method may return an error if there are communication issues with the sensor.
    pub fn read_lp_scan_raw_1(&mut self) -> Result<u16, Error<I::Error>> {
        let high = Register::LpScanRaw1H.read(&mut self.interface, self.addr.into())?;
        let low = Register::LpScanRaw1L.read(&mut self.interface, self.addr.into())?;
        Ok((u16::from(high) << 8) | u16::from(low))
    }

    /// Writes  `LpScanRaw1`
    ///
    /// # Errors
    ///
    /// This method may return an error if there are communication issues with the sensor.
    pub fn write_lp_scan_raw_1(&mut self, value: u16) -> Result<(), Error<I::Error>> {
        Register::LpScanRaw1H.write(
            &mut self.interface,
            self.addr.into(),
            (value & 0xff00 >> 8) as u8,
        )?;
        Register::LpScanRaw1L.write(
            &mut self.interface,
            self.addr.into(),
            (value & 0x00ff) as u8,
        )?;
        Ok(())
    }

    /// Returns `LpScanRaw2` the the reference value for low-power scanning channel 2
    ///
    /// # Errors
    ///
    /// This method may return an error if there are communication issues with the sensor.
    pub fn read_lp_scan_raw_2(&mut self) -> Result<u16, Error<I::Error>> {
        let high = Register::LpScanRaw2H.read(&mut self.interface, self.addr.into())?;
        let low = Register::LpScanRaw2L.read(&mut self.interface, self.addr.into())?;
        Ok((u16::from(high) << 8) | u16::from(low))
    }

    /// Writes  `LpScanRaw2`
    ///
    /// # Errors
    ///
    /// This method may return an error if there are communication issues with the sensor.
    pub fn write_lp_scan_raw_2(&mut self, value: u16) -> Result<(), Error<I::Error>> {
        Register::LpScanRaw2H.write(
            &mut self.interface,
            self.addr.into(),
            (value & 0xff00 >> 8) as u8,
        )?;
        Register::LpScanRaw2L.write(
            &mut self.interface,
            self.addr.into(),
            (value & 0x00ff) as u8,
        )?;
        Ok(())
    }

    /// Returns Automatic recalibration period duringlowpower mode. Unit: 1 minute, Range: 1～5,Default:5
    ///
    /// # Errors
    ///
    /// This method may return an error if there are communication issues with the sensor.
    pub fn read_lp_auto_wakeup(&mut self) -> Result<u8, Error<I::Error>> {
        Ok(Register::LpAutoWakeTime.read(&mut self.interface, self.addr.into())?)
    }

    /// Returns Automatic recalibration period duringlowpower mode. Unit: 1 minute, Range: 1～5,Default:5
    ///
    /// # Errors
    ///
    /// This method may return an error if there are communication issues with the sensor.
    pub fn write_lp_auto_wakeup(&mut self, value: u8) -> Result<(), Error<I::Error>> {
        Register::LpAutoWakeTime.write(&mut self.interface, self.addr.into(), value)?;
        Ok(())
    }

    /// Returns Low power scanning wake-up threshold.Thesmaller it is, the more sensitive itis.Range: 1～255, Default: 48
    ///
    /// # Errors
    ///
    /// This method may return an error if there are communication issues with the sensor.
    pub fn read_lp_scan_th(&mut self) -> Result<u8, Error<I::Error>> {
        Ok(Register::LpScanTH.read(&mut self.interface, self.addr.into())?)
    }

    /// Returns Low power scanning wake-up threshold.Thesmaller it is, the more sensitive itis.Range: 1～255, Default: 48
    ///
    /// # Errors
    ///
    /// This method may return an error if there are communication issues with the sensor.
    pub fn write_lp_scan_th(&mut self, value: u8) -> Result<(), Error<I::Error>> {
        Register::LpScanTH.write(&mut self.interface, self.addr.into(), value)?;
        Ok(())
    }

    /// Returns Low-power scanning measurement range.Thegreater it is, the more sensitive andthemore power consumption it is.
    /// Range: 0, 1, 2, 3; Default: 3
    ///
    /// # Errors
    ///
    /// This method may return an error if there are communication issues with the sensor.
    pub fn read_lp_scan_win(&mut self) -> Result<u8, Error<I::Error>> {
        Ok(Register::LpScanWin.read(&mut self.interface, self.addr.into())?)
    }

    /// Range: 0, 1, 2, 3; Default: 3
    ///
    /// # Errors
    ///
    /// This method may return an error if there are communication issues with the sensor.
    pub fn write_lp_scan_win(&mut self, value: u8) -> Result<(), Error<I::Error>> {
        Register::LpScanWin.write(&mut self.interface, self.addr.into(), value)?;
        Ok(())
    }

    /// Returns Low-power scanning frequency, the smalleritis, the more sensitive it is.
    /// Range: 1～255; Default: 7
    ///
    /// # Errors
    ///
    /// This method may return an error if there are communication issues with the sensor.
    pub fn read_lp_scan_freq(&mut self) -> Result<u8, Error<I::Error>> {
        Ok(Register::LpScanFreq.read(&mut self.interface, self.addr.into())?)
    }

    /// Range: 1～255; Default: 7
    ///
    /// # Errors
    ///
    /// This method may return an error if there are communication issues with the sensor.
    pub fn write_lp_scan_freq(&mut self, value: u8) -> Result<(), Error<I::Error>> {
        Register::LpScanFreq.write(&mut self.interface, self.addr.into(), value)?;
        Ok(())
    }

    /// Returns Low-power scanning current. The smalleritis,the more sensitive it is. Range: 1～255
    ///
    /// # Errors
    ///
    /// This method may return an error if there are communication issues with the sensor.
    pub fn read_lp_scan_idac(&mut self) -> Result<u8, Error<I::Error>> {
        Ok(Register::LpScanIdac.read(&mut self.interface, self.addr.into())?)
    }

    /// Returns Low-power scanning current. The smalleritis,the more sensitive it is. Range: 1～255
    ///
    /// # Errors
    ///
    /// This method may return an error if there are communication issues with the sensor.
    pub fn write_lp_scan_idac(&mut self, value: u8) -> Result<(), Error<I::Error>> {
        Register::LpScanIdac.write(&mut self.interface, self.addr.into(), value)?;
        Ok(())
    }

    /// Returns Automatically enter low-power mode ifthereis no touch in x seconds. Unit: 1s,Default:2s.
    ///
    /// # Errors
    ///
    /// This method may return an error if there are communication issues with the sensor.
    pub fn read_auto_sleep_time(&mut self) -> Result<u8, Error<I::Error>> {
        Ok(Register::AutoSleepTime.read(&mut self.interface, self.addr.into())?)
    }

    /// Returns Automatically enter low-power mode ifthereis no touch in x seconds. Unit: 1s,Default:2s.
    ///
    /// # Errors
    ///
    /// This method may return an error if there are communication issues with the sensor.
    pub fn write_auto_sleep_time(&mut self, value: u8) -> Result<(), Error<I::Error>> {
        Register::AutoSleepTime.write(&mut self.interface, self.addr.into(), value)?;
        Ok(())
    }

    /// Writes Irq configuration
    ///
    /// # Errors
    ///
    /// This method may return an error if there are communication issues with the sensor.
    pub fn write_irq_ctl(&mut self, value: IrqCtl) -> Result<(), Error<I::Error>> {
        Register::IrqCtl.write(&mut self.interface, self.addr.into(), value.0)?;
        Ok(())
    }

    /// Returns Irq configuration
    ///
    /// # Errors
    ///
    /// This method may return an error if there are communication issues with the sensor.
    pub fn read_irq_ctl(&mut self) -> Result<IrqCtl, Error<I::Error>> {
        let irq_ctl = Register::IrqCtl.read(&mut self.interface, self.addr.into())?;
        Ok(IrqCtl(irq_ctl))
    }

    /// Returns Automatically reset if there is touchbutnovalid gesture within x seconds. Unit: 1S, Disable: 0, Default:
    ///
    /// # Errors
    ///
    /// This method may return an error if there are communication issues with the sensor.
    pub fn read_auto_reset(&mut self) -> Result<u8, Error<I::Error>> {
        Ok(Register::AutoReset.read(&mut self.interface, self.addr.into())?)
    }

    /// Returns Auto reset after long press x secondsUnit: 1S, Disable: 0, Default: 10
    ///
    /// # Errors
    ///
    /// This method may return an error if there are communication issues with the sensor.
    pub fn read_long_press_time(&mut self) -> Result<u8, Error<I::Error>> {
        Ok(Register::LongPressTime.read(&mut self.interface, self.addr.into())?)
    }

    /// Returns 0 by default, enable automatic entryintolow-power mode
    /// When non-zero, disable automatic entryintolow-power mode
    ///
    /// # Errors
    ///
    /// This method may return an error if there are communication issues with the sensor.
    pub fn read_disable_auto_sleep(&mut self) -> Result<u8, Error<I::Error>> {
        Ok(Register::DisAutoSleep.read(&mut self.interface, self.addr.into())?)
    }

    /// Returns the [`TouchEvent`] if it is availabe.
    ///
    /// # Errors
    ///
    /// This method may return an error if there are communication issues with the sensor.
    pub fn read_event(&mut self) -> Result<TouchEvent, Error<I::Error>> {
        Ok(TouchEvent::read(&mut self.interface, self.addr.into())?)
    }
}
