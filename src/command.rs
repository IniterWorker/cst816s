#![allow(clippy::doc_markdown)]
#![allow(dead_code)]

use bitfield::{bitfield, BitRange, BitRangeMut};
use embedded_hal::i2c::{I2c, SevenBitAddress};

pub mod constants {
    /// Duration of reset pin low (in ms)
    pub const CST78XX_RESET_DURATION_LOW_MS: u32 = 20;
    /// Duration of reset pin high (in ms)
    pub const CST78XX_RESET_DURATION_HIGH_MS: u32 = 50;
    /// Register Touch Data
    pub const CST78XX_REG_DATA: u8 = 0x00;
    /// Register Motion
    pub const CST78XX_MOTION: u8 = 0xEC;
    /// Register Chip ID
    pub const CST78XX_REG_CHIPID: u8 = 0xA7;
    /// Register Project ID
    pub const CST78XX_REG_PROJECTID: u8 = 0xA8;
    /// Register Firmware Version
    pub const CST78XX_REG_FIRMWARE_VERSION: u8 = 0xA9;
    /// Command deep sleep
    pub const CST816S_REG_DEEP_SLEEP: u8 = 0xE5;
    pub const CST816D_REG_DEEP_SLEEP: u8 = 0xE5;
    pub const CST816T_REG_DEEP_SLEEP: u8 = 0xE5;
    pub const CST716_REG_DEEP_SLEEP: u8 = 0xA5;
    pub const CST78XX_CMD_DEEP_SLEEP: u8 = 0x03;
}

bitfield! {
    /// Motion mask configuration
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct MotionMask(u8);
    impl Debug;
    #[doc = "Reserved"]
    pub u8, reserved, set_reserved: 7, 3;
    #[doc = "Enable Continous Left-Right (LR) Scrolling Action"]
    pub bool, en_continuous_left_right, set_en_continuous_left_right: 2;
    #[doc = "Enable Continous Up-Down (UD) Scrolling Action"]
    pub bool, en_continuous_up_down, set_en_continuous_up_down: 1;
    #[doc = "Enable Double Click Action"]
    pub bool, en_double_click, set_en_double_click: 0;
}

bitfield! {
    /// Motion mask configuration
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct IrqCtl(u8);
    impl Debug;
    #[doc = "Interrupt pin test, automatically generates low pulsesperiodically after being enabled"]
    pub bool, en_test, set_en_test: 5;
    #[doc = "Enable Continous Left-Right (LR) Scrolling Action"]
    pub bool, en_touch, set_en_touch: 6;
    #[doc = "Enable Continous Up-Down (UD) Scrolling Action"]
    pub bool, en_change, set_en_change: 5;
    #[doc = "Enable Double Click Action"]
    pub bool, en_once_wlp, set_en_once_wlp: 4;
    #[doc = "Reserved"]
    pub u8, reserved, set_reserved: 3, 0;
}

bitfield! {
    /// Motion mask configuration
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct XposH(u8);
    impl Debug;
    #[doc = "Reserved"]
    pub Touch, touch, set_touch: 7, 4;
    #[doc = "Reserved"]
    pub u8, x_pos_high, set_x_pos_high: 3, 0;
}

/// CST816s, CST816D, CST816T
#[derive(Debug, Clone, Copy)]
#[repr(u8)]
pub enum Register {
    /// Gesture ID
    GestureId = 0x01,
    /// Finger Numbers
    FingerNumber = 0x02,
    /// High 4 bits of the X coordinate
    XposH = 0x03,
    /// Low 8 bits of the X coordinate
    XposL = 0x04,
    /// High 4 bits of the Y coordinate
    YposH = 0x05,
    /// Low 8 bits of the Y coordinate
    YposL = 0x06,
    /// High 8 bits of the BPC0H value
    BPC0H = 0xB0,
    /// Low 8 bits of the BPC0L value
    BPC0L = 0xB1,
    /// High 8 bits of the BPC1H value
    BPC1H = 0xB2,
    /// Low 8 bits of the BPC1L value
    BPC1L = 0xB3,
    /// Chip Id
    ChipId = 0xA7,
    /// Project Id
    ProjectId = 0xA8,
    /// Firmware Version
    FirmwareVersion = 0xA9,
    /// Deep Sleep
    DeepSleep = 0xE5,
    /// Motion Mask
    MotionMask = 0xEC,
    /// IrqPulseWidth
    IrqPluseWidth = 0xED,
    /// NorScanPer
    NorScanPer = 0xEE,
    /// MotionSlAngle
    MotionSlAngle = 0xEF,
    /// LpScanRaw1H
    LpScanRaw1H = 0xF0,
    /// LpScanRaw1L
    LpScanRaw1L = 0xF1,
    /// LpScanRaw2H
    LpScanRaw2H = 0xF2,
    /// LpScanRaw2L
    LpScanRaw2L = 0xF3,
    /// LpAutoWakeTime
    LpAutoWakeTime = 0xF4,
    /// LpScanTH
    LpScanTH = 0xF5,
    /// LpScanWin
    LpScanWin = 0xF6,
    /// LpScanFreq
    LpScanFreq = 0xF7,
    /// LpScanIdac
    LpScanIdac = 0xF8,
    /// AutoSleepTime
    AutoSleepTime = 0xF9,
    /// IrqCtl
    IrqCtl = 0xFA,
    /// AutoReset
    AutoReset = 0xFB,
    /// LongPressTime
    LongPressTime = 0xFC,
    /// IOCtl
    IOCtl = 0xFD,
    /// DisAutoSleep
    DisAutoSleep = 0xFE,
}

impl Register {
    /// Returns the register
    /// # Errors
    ///
    /// This method may return an error if there are communication issues with the sensor.
    pub fn read<I>(&self, interface: &mut I, addr: u8) -> Result<u8, I::Error>
    where
        I: I2c<SevenBitAddress>,
    {
        let read = [*self as u8; 1];
        let mut buffer = [0; 1];
        interface.write_read(addr, &read, &mut buffer)?;
        Ok(buffer[0])
    }

    /// Writes the register
    /// # Errors
    ///
    /// This method may return an error if there are communication issues with the sensor.
    pub fn write<I>(&self, interface: &mut I, addr: u8, value: u8) -> Result<(), I::Error>
    where
        I: I2c<SevenBitAddress>,
    {
        let buffer = [*self as u8, value];
        interface.write(addr, &buffer)?;
        Ok(())
    }
}

/// Read chip ID
///
/// # Notes
///
/// The chip is blank when it leaves the factory, IIC is not connected, there is no chip ID, and
/// the value can only be read after burning and upgrading the firmeware.
#[derive(Debug)]
pub enum ChipId {
    Factory = 0x00,
    Cst716 = 0x20,
    Cst816s = 0xB4,
    Cst816t = 0xB5,
    Cst816d = 0xB6,
}

#[derive(Debug)]
/// CST816S Touch event touch state
pub enum Touch {
    Down = 0x00,
    Up = 0x04,
    Contact = 0x08,
}

impl BitRange<Touch> for u8 {
    fn bit_range(&self, msb: usize, lsb: usize) -> Touch {
        let mask = ((1 << (msb - lsb + 1)) - 1) << lsb;
        match (*self & mask) >> lsb {
            0x04 => Touch::Up,
            0x08 => Touch::Contact,
            _ => Touch::Down, // 0x00
        }
    }
}

impl BitRangeMut<Touch> for u8 {
    fn set_bit_range(&mut self, msb: usize, lsb: usize, value: Touch) {
        let mask = !(((1 << (msb - lsb + 1)) - 1) << lsb);
        *self &= mask;
        *self |= (value as Self) << lsb;
    }
}

impl From<u8> for Touch {
    fn from(value: u8) -> Self {
        match value {
            0x04 => Self::Up,
            0x08 => Self::Contact,
            _ => Self::Down, // 0x00
        }
    }
}

#[derive(Debug)]
/// CST816S Gesture types
pub enum Gesture {
    /// No gesture detected
    None = 0x00,
    /// Downward slide detected
    SlideDown = 0x01,
    /// Upward slide detected
    SlideUp = 0x02,
    /// Left slide detected
    SlideLeft = 0x03,
    /// Right slide detected
    SlideRight = 0x04,
    /// Single click detected
    SingleClick = 0x05,
    /// Double click detected
    DoubleClick = 0x0b,
    /// Long press detected
    LongPress = 0x0c,
}

impl From<u8> for Gesture {
    fn from(value: u8) -> Self {
        match value {
            0x01 => Self::SlideDown,
            0x02 => Self::SlideUp,
            0x03 => Self::SlideLeft,
            0x04 => Self::SlideRight,
            0x05 => Self::SingleClick,
            0x0b => Self::DoubleClick,
            0x0c => Self::LongPress,
            _ => Self::None,
        }
    }
}

#[derive(Debug)]
pub struct TouchEvent {
    /// Touch id
    pub touch_id: u8,
    // Finger number
    pub finger_number: u8,
    /// Detected gesture
    pub gesture_type: Gesture,
    /// Press or release event
    pub touch_type: Touch,
    /// X coordinate
    pub x: u16,
    /// Y coordinate
    pub y: u16,
}

impl TouchEvent {
    /// Returns the registers
    /// # Errors
    ///
    /// This method may return an error if there are communication issues with the sensor.
    pub fn read<I>(interface: &mut I, addr: u8) -> Result<Self, I::Error>
    where
        I: I2c<SevenBitAddress>,
    {
        let read = [constants::CST78XX_REG_DATA; 1];
        let mut buffer = [0; 7];
        interface.write_read(addr, &read, &mut buffer)?;
        Ok(Self::from_buffer(&buffer))
    }

    fn from_buffer(buffer: &[u8]) -> Self {
        let gesture_type = Gesture::from(buffer[Register::GestureId as usize] & 0x0f);

        let finger_number = buffer[Register::FingerNumber as usize];

        let action_id = (buffer[Register::XposH as usize] & 0xf0) >> 4;

        let touch_id = (buffer[Register::YposH as usize] & 0xf0) >> 4;

        let x_high = buffer[Register::XposH as usize] & 0x0f;
        let x_low = buffer[Register::XposL as usize];

        let y_high = buffer[Register::YposH as usize] & 0x0f;
        let y_low = buffer[Register::YposL as usize];

        let touch_type = Touch::from(action_id); /* 0 = Touch Down, 1 = Touch Up, 2 = Contact */

        let x: u16 = (u16::from(x_high) << 8) | u16::from(x_low);
        let y: u16 = (u16::from(y_high) << 8) | u16::from(y_low);

        Self {
            touch_id,
            finger_number,
            gesture_type,
            touch_type,
            x,
            y,
        }
    }
}
