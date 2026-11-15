//! I2C transport, device address, and the no-op pin placeholder.

use core::convert::Infallible;

use embedded_hal::digital::{ErrorType, OutputPin};

/// I2C device address. The CST816 family has a single fixed address.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum DeviceAddress {
    /// Primary (and only) address, 0x15.
    #[default]
    Primary = 0x15,
}

impl DeviceAddress {
    /// Raw 7-bit address.
    #[must_use]
    pub const fn as_u8(self) -> u8 {
        self as u8
    }
}

/// Placeholder for an unwired RST/INT pin.
///
/// Implements blocking `OutputPin` as a no-op so `reset()` works
/// uniformly. Deliberately does **not** implement async `Wait`:
/// `wait_for_interrupt` requires a real interrupt pin.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub struct NoPin;

impl ErrorType for NoPin {
    type Error = Infallible;
}

impl OutputPin for NoPin {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

/// Blocking register access used by the synchronous driver.
pub trait RegisterIo {
    /// Bus error type.
    type Error;
    /// Read one register.
    fn read_register(&mut self, reg: u8) -> Result<u8, Self::Error>;
    /// Read a burst starting at `reg`.
    fn read_registers(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error>;
    /// Write one register.
    fn write_register(&mut self, reg: u8, value: u8) -> Result<(), Self::Error>;
    /// Write a raw frame (first byte is the start register).
    fn write_frame(&mut self, frame: &[u8]) -> Result<(), Self::Error>;
}

/// I2C transport over an `embedded-hal` I2C bus.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct I2cInterface<I> {
    /// Underlying I2C bus.
    pub bus: I,
    /// Device address.
    pub address: DeviceAddress,
}

impl<I> I2cInterface<I> {
    /// New interface with an explicit address.
    pub const fn new(bus: I, address: DeviceAddress) -> Self {
        Self { bus, address }
    }
}

impl<I: embedded_hal::i2c::I2c> RegisterIo for I2cInterface<I> {
    type Error = I::Error;

    fn read_register(&mut self, reg: u8) -> Result<u8, Self::Error> {
        let mut buf = [0u8; 1];
        self.bus
            .write_read(self.address.as_u8(), &[reg], &mut buf)?;
        Ok(buf[0])
    }

    fn read_registers(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
        self.bus.write_read(self.address.as_u8(), &[reg], buf)
    }

    fn write_register(&mut self, reg: u8, value: u8) -> Result<(), Self::Error> {
        self.bus.write(self.address.as_u8(), &[reg, value])
    }

    fn write_frame(&mut self, frame: &[u8]) -> Result<(), Self::Error> {
        self.bus.write(self.address.as_u8(), frame)
    }
}

/// Async register access used by the async driver.
#[cfg(feature = "async")]
#[allow(async_fn_in_trait)]
pub trait AsyncRegisterIo {
    /// Bus error type.
    type Error;
    /// Read one register.
    async fn read_register(&mut self, reg: u8) -> Result<u8, Self::Error>;
    /// Read a burst starting at `reg`.
    async fn read_registers(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error>;
    /// Write one register.
    async fn write_register(&mut self, reg: u8, value: u8) -> Result<(), Self::Error>;
    /// Write a raw frame (first byte is the start register).
    async fn write_frame(&mut self, frame: &[u8]) -> Result<(), Self::Error>;
}

#[cfg(feature = "async")]
impl<I: embedded_hal_async::i2c::I2c> AsyncRegisterIo for I2cInterface<I> {
    type Error = I::Error;

    async fn read_register(&mut self, reg: u8) -> Result<u8, Self::Error> {
        let mut buf = [0u8; 1];
        self.bus
            .write_read(self.address.as_u8(), &[reg], &mut buf)
            .await?;
        Ok(buf[0])
    }

    async fn read_registers(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
        self.bus.write_read(self.address.as_u8(), &[reg], buf).await
    }

    async fn write_register(&mut self, reg: u8, value: u8) -> Result<(), Self::Error> {
        self.bus.write(self.address.as_u8(), &[reg, value]).await
    }

    async fn write_frame(&mut self, frame: &[u8]) -> Result<(), Self::Error> {
        self.bus.write(self.address.as_u8(), frame).await
    }
}
