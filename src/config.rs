//! Typed configuration: bit-field structs and register presets.
//!
//! Plain bool-field structs with `to_raw`/`from_raw`; no bitfield crate.

/// MotionMask register (0xEC): which continuous motions / double-click
/// reporting are enabled.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MotionMask {
    /// Bit 2: continuous left/right slide actions.
    pub continuous_left_right: bool,
    /// Bit 1: continuous up/down slide actions.
    pub continuous_up_down: bool,
    /// Bit 0: double-click action.
    pub double_click: bool,
}

impl MotionMask {
    /// Pack to register byte.
    #[must_use]
    pub const fn to_raw(self) -> u8 {
        ((self.continuous_left_right as u8) << 2)
            | ((self.continuous_up_down as u8) << 1)
            | (self.double_click as u8)
    }

    /// Unpack from register byte.
    #[must_use]
    pub const fn from_raw(raw: u8) -> Self {
        Self {
            continuous_left_right: raw & (1 << 2) != 0,
            continuous_up_down: raw & (1 << 1) != 0,
            double_click: raw & (1 << 0) != 0,
        }
    }
}

/// IrqCtl register (0xFA). Bits 3..=1 are reserved and written 0.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct IrqCtl {
    /// Bit 7: test interrupt (periodic pulse for wiring checks).
    pub test: bool,
    /// Bit 6: interrupt on touch point data.
    pub touch: bool,
    /// Bit 5: interrupt on touch state change.
    pub change: bool,
    /// Bit 4: interrupt on motion/gesture.
    pub motion: bool,
    /// Bit 0: long-press fires once only.
    pub once_long_press: bool,
}

impl IrqCtl {
    /// Pack to register byte.
    #[must_use]
    pub const fn to_raw(self) -> u8 {
        ((self.test as u8) << 7)
            | ((self.touch as u8) << 6)
            | ((self.change as u8) << 5)
            | ((self.motion as u8) << 4)
            | (self.once_long_press as u8)
    }

    /// Unpack from register byte.
    #[must_use]
    pub const fn from_raw(raw: u8) -> Self {
        Self {
            test: raw & (1 << 7) != 0,
            touch: raw & (1 << 6) != 0,
            change: raw & (1 << 5) != 0,
            motion: raw & (1 << 4) != 0,
            once_long_press: raw & (1 << 0) != 0,
        }
    }
}

/// IoCtl register (0xFD). Changing bus electrical mode mid-session is
/// a footgun; the driver never writes this during `apply_config`.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct IoCtl {
    /// Bit 2: software reset.
    pub soft_reset: bool,
    /// Bit 1: I2C pins open-drain.
    pub i2c_open_drain: bool,
    /// Bit 0: 1.8 V IO mode.
    pub enable_1v8: bool,
}

impl IoCtl {
    /// Pack to register byte.
    #[must_use]
    pub const fn to_raw(self) -> u8 {
        ((self.soft_reset as u8) << 2)
            | ((self.i2c_open_drain as u8) << 1)
            | (self.enable_1v8 as u8)
    }

    /// Unpack from register byte.
    #[must_use]
    pub const fn from_raw(raw: u8) -> Self {
        Self {
            soft_reset: raw & (1 << 2) != 0,
            i2c_open_drain: raw & (1 << 1) != 0,
            enable_1v8: raw & (1 << 0) != 0,
        }
    }
}

/// Reporting mode presets (vendor debug doc §4.4), written to IrqCtl.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ReportMode {
    /// Point data only (IrqCtl raw 0x60).
    Point,
    /// Gestures only (IrqCtl raw 0x11).
    Gesture,
    /// Point data and gestures (IrqCtl raw 0x71).
    PointAndGesture,
}

impl ReportMode {
    /// IrqCtl raw preset for this mode.
    #[must_use]
    pub const fn irq_ctl_raw(self) -> u8 {
        match self {
            Self::Point => 0x60,
            Self::Gesture => 0x11,
            Self::PointAndGesture => 0x71,
        }
    }
}

/// Main configuration written by `apply_config` (and re-applied on
/// wake, since reset restores register defaults).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// Motion mask (default: all off).
    pub motion_mask: MotionMask,
    /// IRQ pulse width, 1..=200 (default 10).
    pub irq_pulse_width: u8,
    /// Normal scan period, 1..=30 (default 1).
    pub normal_scan_period: u8,
    /// Motion slide angle (default 0; no documented default).
    pub motion_slide_angle: u8,
    /// Auto sleep time, seconds per vendor doc (default 2). Enforcement
    /// is firmware-defined: on the Waveshare CST816T module firmware
    /// (project 105) writing 2 puts the chip into bus-NACKing sleep
    /// ~0.5 s after the last transaction batch. Polling applications
    /// should set `disable_auto_sleep` or expect sleep + touch/reset wake.
    pub auto_sleep_time_s: u8,
    /// IRQ control (default: touch+change+motion = raw 0x70).
    pub irq_ctl: IrqCtl,
    /// Auto reset time, seconds (default 5; 0 disables — auto-reset
    /// kills live points, per InfiniTime fleet experience).
    pub auto_reset_s: u8,
    /// Long-press time, seconds (default 10).
    pub long_press_time_s: u8,
    /// Disable auto sleep (default false).
    pub disable_auto_sleep: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            motion_mask: MotionMask::default(),
            irq_pulse_width: 10,
            normal_scan_period: 1,
            motion_slide_angle: 0,
            auto_sleep_time_s: 2,
            irq_ctl: IrqCtl {
                touch: true,
                change: true,
                motion: true,
                ..IrqCtl::default()
            },
            auto_reset_s: 5,
            long_press_time_s: 10,
            disable_auto_sleep: false,
        }
    }
}

/// Low-power (standby scan) profile, written by
/// `apply_low_power_config` as the Zephyr-style burst.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LowPowerConfig {
    /// Module-calibrated scan reference 1; never overwritten unless set.
    pub scan_raw1: Option<u16>,
    /// Module-calibrated scan reference 2; never overwritten unless set.
    pub scan_raw2: Option<u16>,
    /// Auto-wake time, 1..=5 minutes (default 5).
    pub auto_wake_time_min: u8,
    /// Scan threshold (default 48).
    pub scan_threshold: u8,
    /// Scan window (default 3).
    pub scan_window: u8,
    /// Scan frequency (default 7).
    pub scan_frequency: u8,
    /// Scan IDAC (default 200; no datasheet default — Zephyr-derived).
    pub scan_idac: u8,
}

impl Default for LowPowerConfig {
    fn default() -> Self {
        Self {
            scan_raw1: None,
            scan_raw2: None,
            auto_wake_time_min: 5,
            scan_threshold: 48,
            scan_window: 3,
            scan_frequency: 7,
            scan_idac: 200,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn motion_mask_round_trip() {
        let m = MotionMask {
            continuous_left_right: true,
            continuous_up_down: false,
            double_click: true,
        };
        assert_eq!(m.to_raw(), 0b101);
        assert_eq!(MotionMask::from_raw(0b101), m);
    }

    #[test]
    fn irq_ctl_round_trip_and_default_raw() {
        assert_eq!(Config::default().irq_ctl.to_raw(), 0x70);
        let c = IrqCtl::from_raw(0x71);
        assert!(c.touch && c.change && c.motion && c.once_long_press && !c.test);
        assert_eq!(c.to_raw(), 0x71);
    }

    #[test]
    fn io_ctl_round_trip() {
        let c = IoCtl {
            soft_reset: false,
            i2c_open_drain: true,
            enable_1v8: true,
        };
        assert_eq!(c.to_raw(), 0b011);
        assert_eq!(IoCtl::from_raw(0b011), c);
    }

    #[test]
    fn report_mode_presets() {
        assert_eq!(ReportMode::Point.irq_ctl_raw(), 0x60);
        assert_eq!(ReportMode::Gesture.irq_ctl_raw(), 0x11);
        assert_eq!(ReportMode::PointAndGesture.irq_ctl_raw(), 0x71);
    }
}
