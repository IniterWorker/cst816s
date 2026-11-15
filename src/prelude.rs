//! Common imports for application code.

pub use crate::data::{Gesture, TouchReport};
pub use crate::driver::Cst816s;
#[cfg(feature = "async")]
pub use crate::driver_async::Cst816sAsync;
