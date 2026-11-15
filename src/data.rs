//! Data types: chip identity, gestures, touch reports, transforms.

use crate::registers::Register;

/// Identified Hynitron CST8xx variant.
///
/// CST826/830/836U report `ChipId == 0x00` and are identified through
/// the `FactoryId` register instead.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ChipId {
    /// CST716 (chip register 0x20).
    Cst716,
    /// CST816S (chip register 0xB4).
    Cst816s,
    /// CST816T (chip register 0xB5).
    Cst816t,
    /// CST816D (chip register 0xB6).
    Cst816d,
    /// CST820 (chip register 0xB7).
    Cst820,
    /// CST826 (factory register 0x11).
    Cst826,
    /// CST830 (factory register 0x12).
    Cst830,
    /// CST836U (factory register 0x13).
    Cst836u,
    /// Unrecognised ID, kept verbatim. Probe is non-fatal on unknown
    /// IDs: per-project firmware may report anything.
    Unknown(u8),
}

impl ChipId {
    /// Map a `Register::ChipId` (0xA7) value.
    #[must_use]
    pub const fn from_chip_register(raw: u8) -> Self {
        match raw {
            0x20 => Self::Cst716,
            0xB4 => Self::Cst816s,
            0xB5 => Self::Cst816t,
            0xB6 => Self::Cst816d,
            0xB7 => Self::Cst820,
            other => Self::Unknown(other),
        }
    }

    /// Map a `Register::FactoryId` (0xAA) value.
    #[must_use]
    pub const fn from_factory_register(raw: u8) -> Self {
        match raw {
            0x11 => Self::Cst826,
            0x12 => Self::Cst830,
            0x13 => Self::Cst836u,
            other => Self::Unknown(other),
        }
    }

    /// Register that accepts the deep-sleep command on this variant.
    #[must_use]
    pub const fn sleep_register(self) -> Register {
        match self {
            Self::Cst716 | Self::Cst826 | Self::Cst830 | Self::Cst836u => Register::PowerMode,
            _ => Register::SleepMode,
        }
    }

    /// Whether the variant can wake from touch (only 816S/816T).
    #[must_use]
    pub const fn has_touch_wake(self) -> bool {
        matches!(self, Self::Cst816s | Self::Cst816t)
    }
}

/// Identity read back by `probe`/`init`.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DeviceInfo {
    /// Identified chip variant (may be `Unknown`).
    pub chip: ChipId,
    /// Per-project firmware identifier (`Register::ProjectId`).
    pub project_id: u8,
    /// Firmware version byte.
    pub firmware_version: u8,
}

/// Gesture reported in `Register::GestureId`.
///
/// Directions are panel-relative per the vendor register doc and may
/// appear inverted depending on panel mounting/firmware (InfiniTime
/// uses the opposite up/down naming). Fix orientation with
/// [`CoordinateTransform`], never by renaming variants.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Gesture {
    /// No gesture (0x00).
    None,
    /// Slide up (0x01).
    SlideUp,
    /// Slide down (0x02).
    SlideDown,
    /// Slide left (0x03).
    SlideLeft,
    /// Slide right (0x04).
    SlideRight,
    /// Single click (0x05).
    SingleClick,
    /// Double click (0x0B).
    DoubleClick,
    /// Long press (0x0C).
    LongPress,
    /// Unrecognised gesture byte, kept verbatim.
    Unknown(u8),
}

impl Gesture {
    /// Map the raw gesture byte (full byte, no masking).
    #[must_use]
    pub const fn from_raw(raw: u8) -> Self {
        match raw {
            0x00 => Self::None,
            0x01 => Self::SlideUp,
            0x02 => Self::SlideDown,
            0x03 => Self::SlideLeft,
            0x04 => Self::SlideRight,
            0x05 => Self::SingleClick,
            0x0B => Self::DoubleClick,
            0x0C => Self::LongPress,
            other => Self::Unknown(other),
        }
    }

    /// Raw gesture byte.
    #[must_use]
    pub const fn as_raw(self) -> u8 {
        match self {
            Self::None => 0x00,
            Self::SlideUp => 0x01,
            Self::SlideDown => 0x02,
            Self::SlideLeft => 0x03,
            Self::SlideRight => 0x04,
            Self::SingleClick => 0x05,
            Self::DoubleClick => 0x0B,
            Self::LongPress => 0x0C,
            Self::Unknown(raw) => raw,
        }
    }

    /// Whether this is one of the four slide gestures (raw 0x01..=0x04).
    #[must_use]
    pub const fn is_slide(self) -> bool {
        matches!(
            self,
            Self::SlideUp | Self::SlideDown | Self::SlideLeft | Self::SlideRight
        )
    }
}

/// Touch event kind from XPosH bits `[7:6]`.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum TouchEventKind {
    /// Finger down (0).
    Down = 0,
    /// Finger up (1).
    Up = 1,
    /// Contact / move (2).
    Contact = 2,
    /// No event (3).
    None = 3,
}

impl TouchEventKind {
    const fn from_bits(bits: u8) -> Self {
        match bits & 0x03 {
            0 => Self::Down,
            1 => Self::Up,
            2 => Self::Contact,
            _ => Self::None,
        }
    }
}

/// One parsed 6-byte touch frame. The CST816 frame carries exactly
/// one point; there are no pressure/area fields.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TouchReport {
    /// Gesture byte decoded.
    pub gesture: Gesture,
    /// Finger count (low nibble of FingerNumber).
    pub fingers: u8,
    /// Event kind from XPosH bits `[7:6]`.
    pub event: TouchEventKind,
    /// X coordinate, 12-bit.
    pub x: u16,
    /// Y coordinate, 12-bit.
    pub y: u16,
    /// Touch ID from YPosH bits `[7:4]`.
    pub touch_id: u8,
    /// `fingers > 0 && event is Down|Contact` (Zephyr rule).
    pub pressed: bool,
}

impl TouchReport {
    /// Parse a burst read starting at `REG_DATA_START` (0x01).
    /// Indexes are relative to the frame start.
    #[must_use]
    pub fn from_frame(frame: [u8; 6]) -> Self {
        let fingers = frame[1] & 0x0F;
        let event = TouchEventKind::from_bits(frame[2] >> 6);
        Self {
            gesture: Gesture::from_raw(frame[0]),
            fingers,
            event,
            x: (u16::from(frame[2] & 0x0F) << 8) | u16::from(frame[3]),
            y: (u16::from(frame[4] & 0x0F) << 8) | u16::from(frame[5]),
            touch_id: frame[4] >> 4,
            pressed: fingers > 0 && matches!(event, TouchEventKind::Down | TouchEventKind::Contact),
        }
    }
}

/// Panel-mounting correction applied to reported coordinates.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CoordinateTransform {
    /// Swap X and Y (applied first).
    pub swap_xy: bool,
    /// Mirror X within the (post-swap) width.
    pub invert_x: bool,
    /// Mirror Y within the (post-swap) height.
    pub invert_y: bool,
}

impl CoordinateTransform {
    /// Apply swap, then inversion against post-swap dimensions.
    #[must_use]
    pub fn apply(self, x: u16, y: u16, width: u16, height: u16) -> (u16, u16) {
        let (mut x, mut y, w, h) = if self.swap_xy {
            (y, x, height, width)
        } else {
            (x, y, width, height)
        };
        if self.invert_x {
            x = (w - 1).saturating_sub(x);
        }
        if self.invert_y {
            y = (h - 1).saturating_sub(y);
        }
        (x, y)
    }
}

/// Slide-gesture de-duplication (pure, no I/O), wasp-os semantics.
///
/// Slides fire repeatedly while the finger moves; emit once, then
/// suppress until a finger-release report or the cooldown elapses
/// (the chip may never report release). Non-slide gestures always
/// pass through.
#[derive(Clone, Copy, Debug)]
pub struct GestureLatch {
    cooldown_ms: u32,
    released: bool,
    last_slide_ms: u32,
}

impl Default for GestureLatch {
    fn default() -> Self {
        Self::new(300)
    }
}

impl GestureLatch {
    /// New latch with the given slide cooldown in milliseconds.
    #[must_use]
    pub const fn new(cooldown_ms: u32) -> Self {
        Self {
            cooldown_ms,
            released: true,
            last_slide_ms: 0,
        }
    }

    /// Feed one report at monotonic time `now_ms`; returns the gesture
    /// to act on, if any.
    pub fn feed(&mut self, report: &TouchReport, now_ms: u32) -> Option<Gesture> {
        let gesture = report.gesture;
        if report.fingers == 0 {
            // Release reopens the latch for the *next* slide; a slide
            // byte on the release frame itself is stale, suppress it.
            self.released = true;
            if gesture.is_slide() {
                return None;
            }
        }
        if gesture == Gesture::None {
            return None;
        }
        if !gesture.is_slide() {
            return Some(gesture);
        }
        let cooldown_elapsed = now_ms.wrapping_sub(self.last_slide_ms) >= self.cooldown_ms;
        if self.released || cooldown_elapsed {
            self.released = false;
            self.last_slide_ms = now_ms;
            Some(gesture)
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frame_single_click_down() {
        let r = TouchReport::from_frame([0x05, 0x01, 0x01, 0x20, 0x02, 0x40]);
        assert_eq!(r.gesture, Gesture::SingleClick);
        assert_eq!(r.fingers, 1);
        assert_eq!(r.event, TouchEventKind::Down);
        assert_eq!(r.x, 288);
        assert_eq!(r.y, 576);
        assert_eq!(r.touch_id, 0);
        assert!(r.pressed);
    }

    #[test]
    fn frame_slide_up_release() {
        let r = TouchReport::from_frame([0x01, 0x00, 0x40, 0x10, 0x10, 0x10]);
        assert_eq!(r.gesture, Gesture::SlideUp);
        assert_eq!(r.event, TouchEventKind::Up);
        assert!(!r.pressed);
    }

    #[test]
    fn frame_unknown_gesture_and_none_event() {
        let r = TouchReport::from_frame([0x42, 0x01, 0xC0, 0x00, 0x00, 0x00]);
        assert_eq!(r.gesture, Gesture::Unknown(0x42));
        assert_eq!(r.event, TouchEventKind::None);
        assert!(!r.pressed);
    }

    #[test]
    fn gesture_raw_round_trip() {
        for raw in [0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x0B, 0x0C, 0x99] {
            assert_eq!(Gesture::from_raw(raw).as_raw(), raw);
        }
    }

    #[test]
    fn chip_sleep_register_split() {
        assert_eq!(ChipId::Cst816s.sleep_register(), Register::SleepMode);
        assert_eq!(ChipId::Unknown(0x77).sleep_register(), Register::SleepMode);
        assert_eq!(ChipId::Cst716.sleep_register(), Register::PowerMode);
        assert_eq!(ChipId::Cst826.sleep_register(), Register::PowerMode);
        assert!(ChipId::Cst816s.has_touch_wake());
        assert!(!ChipId::Cst820.has_touch_wake());
    }

    #[test]
    fn transform_swap_invert_square() {
        let swap = CoordinateTransform {
            swap_xy: true,
            ..Default::default()
        };
        assert_eq!(swap.apply(10, 200, 240, 240), (200, 10));
        let inv = CoordinateTransform {
            invert_x: true,
            invert_y: true,
            ..Default::default()
        };
        assert_eq!(inv.apply(0, 239, 240, 240), (239, 0));
    }

    #[test]
    fn transform_swap_invert_non_square() {
        // 320x240 panel, swap then invert against post-swap dims (240x320).
        let t = CoordinateTransform {
            swap_xy: true,
            invert_x: true,
            invert_y: true,
        };
        assert_eq!(t.apply(10, 20, 320, 240), (219, 309));
        let t = CoordinateTransform {
            swap_xy: true,
            invert_x: true,
            invert_y: false,
        };
        assert_eq!(t.apply(10, 20, 320, 240), (219, 10));
    }

    fn slide_report(fingers: u8) -> TouchReport {
        TouchReport {
            gesture: Gesture::SlideDown,
            fingers,
            event: TouchEventKind::Contact,
            x: 0,
            y: 0,
            touch_id: 0,
            pressed: fingers > 0,
        }
    }

    #[test]
    fn latch_suppresses_repeat_slide() {
        let mut latch = GestureLatch::default();
        assert_eq!(latch.feed(&slide_report(1), 0), Some(Gesture::SlideDown));
        assert_eq!(latch.feed(&slide_report(1), 50), None);
        assert_eq!(latch.feed(&slide_report(1), 100), None);
    }

    #[test]
    fn latch_release_reopens() {
        let mut latch = GestureLatch::default();
        assert!(latch.feed(&slide_report(1), 0).is_some());
        let _ = latch.feed(&slide_report(0), 110);
        assert_eq!(latch.feed(&slide_report(1), 120), Some(Gesture::SlideDown));
    }

    #[test]
    fn latch_cooldown_reopens_without_release() {
        let mut latch = GestureLatch::new(300);
        assert!(latch.feed(&slide_report(1), 0).is_some());
        assert_eq!(latch.feed(&slide_report(1), 301), Some(Gesture::SlideDown));
    }

    #[test]
    fn latch_click_passes_while_latched() {
        let mut latch = GestureLatch::default();
        assert!(latch.feed(&slide_report(1), 0).is_some());
        let mut click = slide_report(1);
        click.gesture = Gesture::SingleClick;
        assert_eq!(latch.feed(&click, 50), Some(Gesture::SingleClick));
    }
}
