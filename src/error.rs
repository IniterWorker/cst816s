//! Driver error type.

/// Errors returned by CST816 driver operations.
///
/// `BusE` is the I2C bus error, `PinE` the GPIO pin error (reset pin
/// for bus/reset operations; interrupt pin for the async wait methods
/// — see `Cst816sAsync::wait_for_interrupt`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error<BusE, PinE> {
    /// I2C bus error.
    Bus(BusE),
    /// GPIO pin error.
    Pin(PinE),
    /// Value outside the documented range; no I2C traffic was emitted.
    InvalidValue,
}

impl<BusE, PinE> From<BusE> for Error<BusE, PinE> {
    fn from(err: BusE) -> Self {
        Self::Bus(err)
    }
}

#[cfg(feature = "defmt")]
impl<BusE, PinE> defmt::Format for Error<BusE, PinE> {
    fn format(&self, fmt: defmt::Formatter<'_>) {
        // Variant names only: avoids requiring BusE/PinE: Format.
        match self {
            Self::Bus(_) => defmt::write!(fmt, "Bus(..)"),
            Self::Pin(_) => defmt::write!(fmt, "Pin(..)"),
            Self::InvalidValue => defmt::write!(fmt, "InvalidValue"),
        }
    }
}
