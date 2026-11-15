//! Register addresses (CST816S Register Declaration, 2019-08-30).

/// First register of the 6-byte touch frame burst read.
pub const REG_DATA_START: u8 = 0x01;
/// How long RST is held low during [`crate::Cst816s::reset`].
pub const RESET_LOW_MS: u32 = 10;
/// Settle time after RST release (CST816D datasheet T_ron).
pub const RESET_SETTLE_MS: u32 = 100;
/// Value written to the sleep register to enter deep sleep.
pub const SLEEP_COMMAND: u8 = 0x03;

/// CST816 family register addresses.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Register {
    /// Gesture ID (read-only).
    GestureId = 0x01,
    /// Finger count, low nibble (read-only).
    FingerNumber = 0x02,
    /// X high: event in bits `[7:6]`, coordinate bits `[3:0]`.
    XPosH = 0x03,
    /// X low byte.
    XPosL = 0x04,
    /// Y high: touch ID in bits `[7:4]`, coordinate bits `[3:0]`.
    YPosH = 0x05,
    /// Y low byte.
    YPosL = 0x06,
    /// Undocumented (semantics not in vendor register doc).
    Bpc0H = 0xB0,
    /// Undocumented.
    Bpc0L = 0xB1,
    /// Undocumented.
    Bpc1H = 0xB2,
    /// Undocumented.
    Bpc1L = 0xB3,
    /// Power mode; sleep command target on CST716/826/830/836U.
    PowerMode = 0xA5,
    /// Firmware version on parts whose ChipId reads 0x00.
    FirmwareVersionAlt = 0xA6,
    /// Chip ID.
    ChipId = 0xA7,
    /// Project ID (per-project firmware identifier).
    ProjectId = 0xA8,
    /// Firmware version.
    FirmwareVersion = 0xA9,
    /// Factory ID; identifies CST826/830/836U when ChipId is 0x00.
    FactoryId = 0xAA,
    /// Sleep mode; sleep command target on CST816S/T/D and CST820.
    SleepMode = 0xE5,
    /// Motion (gesture) mask.
    MotionMask = 0xEC,
    /// IRQ pulse width, 1..=200 (unit per vendor doc).
    IrqPulseWidth = 0xED,
    /// Normal scan period, 1..=30.
    NormalScanPeriod = 0xEE,
    /// Motion slide angle.
    MotionSlideAngle = 0xEF,
    /// Low-power scan reference 1, high byte.
    LpScanRaw1H = 0xF0,
    /// Low-power scan reference 1, low byte.
    LpScanRaw1L = 0xF1,
    /// Low-power scan reference 2, high byte.
    LpScanRaw2H = 0xF2,
    /// Low-power scan reference 2, low byte.
    LpScanRaw2L = 0xF3,
    /// Low-power auto-wake time, 1..=5 minutes.
    LpAutoWakeTime = 0xF4,
    /// Low-power scan threshold.
    LpScanThreshold = 0xF5,
    /// Low-power scan window (documented 0..=3; see driver setter).
    LpScanWindow = 0xF6,
    /// Low-power scan frequency.
    LpScanFrequency = 0xF7,
    /// Low-power scan IDAC.
    LpScanIdac = 0xF8,
    /// Auto sleep time, seconds.
    AutoSleepTime = 0xF9,
    /// IRQ control.
    IrqCtl = 0xFA,
    /// Auto reset time, seconds (0 disables).
    AutoReset = 0xFB,
    /// Long-press time, seconds.
    LongPressTime = 0xFC,
    /// IO control (soft reset, open-drain, 1.8 V mode).
    IoCtl = 0xFD,
    /// Disable auto sleep (write 0x01 to disable).
    DisableAutoSleep = 0xFE,
}

impl Register {
    /// Raw I2C register address.
    #[must_use]
    pub const fn addr(self) -> u8 {
        self as u8
    }
}
