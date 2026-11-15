//! # CST816 capacitive touch driver
//!
//! A `no_std` driver for the Hynitron CST816 family (CST716, CST816S,
//! CST816T, CST816D, CST820, CST826, CST830, CST836U), built on
//! `embedded-hal` 1.0 (blocking, [`Cst816s`]) and, behind the `async`
//! feature, `embedded-hal-async` 1.0 (`Cst816sAsync`). I2C only.
//!
//! The CST816 ships blank from the factory and is flashed with
//! per-project firmware by the module vendor, so register behaviour
//! is firmware-defined: this driver reports identity verbatim and
//! never fails probe on an unknown chip ID. Firmware-specific
//! registers remain reachable through the `read_register` /
//! `write_register` escape hatch.
//!
//! ## Supported chips
//!
//! | Variant | ID | Sleep register | Touch wake |
//! |---|---|---|---|
//! | CST716 | 0x20 @ ChipId | PowerMode (0xA5) | no |
//! | CST816S | 0xB4 @ ChipId | SleepMode (0xE5) | yes |
//! | CST816T | 0xB5 @ ChipId | SleepMode (0xE5) | yes |
//! | CST816D | 0xB6 @ ChipId | SleepMode (0xE5) | no |
//! | CST820 | 0xB7 @ ChipId | SleepMode (0xE5) | no |
//! | CST826 | 0x11 @ FactoryId | PowerMode (0xA5) | no |
//! | CST830 | 0x12 @ FactoryId | PowerMode (0xA5) | no |
//! | CST836U | 0x13 @ FactoryId | PowerMode (0xA5) | no |
//! | Unknown | verbatim | SleepMode (0xE5) | no |
//!
//! ## Features
//!
//! | Feature | Default | Effect |
//! |---|---|---|
//! | `async` | no | async driver `Cst816sAsync` over `embedded-hal-async` |
//! | `defmt` | no | `defmt::Format` implementations for public types |
//! | `log` | no | `log` diagnostics (probe result, touch reports) |
//!
//! The blocking driver is always available.
//!
//! ## Quickstart (blocking)
//!
//! ```no_run
//! use cst816s::{Cst816s, NoPin};
//! # fn demo<I, D, P>(i2c: I, delay: D, rst: P)
//! # where
//! #     I: embedded_hal::i2c::I2c,
//! #     D: embedded_hal::delay::DelayNs,
//! #     P: embedded_hal::digital::OutputPin,
//! # {
//! let mut touch = Cst816s::new(i2c, delay).with_reset_pin(rst);
//! let info = touch.init().unwrap();
//! let report = touch.read_touch().unwrap();
//! if report.pressed {
//!     // report.x, report.y, report.gesture
//! }
//! # let _ = info;
//! # }
//! ```
//!
//! ## Async (Embassy-style, `async` feature)
//!
//! The async driver is executor-agnostic; it only needs
//! `embedded-hal-async` bus and delay implementations from your HAL.
//!
//! ```ignore
//! use cst816s::Cst816sAsync;
//!
//! // `i2c`, `delay`, `rst`, `int` are your HAL's types; `int` must
//! // implement embedded_hal_async::digital::Wait for wait_for_touch.
//! let mut touch = Cst816sAsync::new(i2c, delay)
//!     .with_reset_pin(rst)
//!     .with_interrupt_pin(int);
//! touch.init().await?;
//! let report = touch.wait_for_touch().await?;
//! ```
//!
//! ## Sources
//!
//! - CST816S Datasheet V1.4 EN (Waveshare)
//! - CST816S Register Declaration (SDK register description,
//!   2019-08-30, Waveshare)
//! - cst78XX debug doc V2.3 (Hynitron: bootloader/sleep/mode facts)
//! - Zephyr `drivers/input/input_cst8xx.c`, Linux
//!   `hynitron-cst816x.c`, InfiniTime, wasp-os, ESPHome, Espressif
//!   `esp_lcd_touch_cst816s`
#![cfg_attr(not(test), no_std)]
// clippy warning level lints
#![warn(
    missing_docs,
    clippy::pedantic,
    clippy::nursery,
    clippy::dbg_macro,
    clippy::unwrap_used,
    clippy::map_err_ignore,
    clippy::panic,
    clippy::unimplemented,
    clippy::unreachable,
    clippy::indexing_slicing
)]
// clippy deny/error level lints
#![deny(
    missing_debug_implementations,
    clippy::rc_buffer,
    clippy::rc_mutex,
    clippy::rest_pat_in_fully_bound_structs,
    clippy::self_named_module_files,
    clippy::separated_literal_suffix,
    clippy::str_to_string,
    clippy::string_add,
    clippy::unnecessary_self_imports,
    clippy::unneeded_field_pattern,
    clippy::verbose_file_reads
)]
// allowed rules
#![allow(
    clippy::module_name_repetitions,
    clippy::similar_names,
    clippy::cast_possible_truncation,
    clippy::redundant_pub_crate,
    // Register/config structs faithfully model hardware bit fields.
    clippy::struct_excessive_bools,
    clippy::doc_markdown,
    clippy::trivially_copy_pass_by_ref,
    clippy::pattern_type_mismatch,
    clippy::missing_const_for_fn,
    // Every fallible method returns the one documented `Error` type.
    clippy::missing_errors_doc,
    clippy::missing_panics_doc
)]
#![cfg_attr(
    test,
    allow(clippy::unwrap_used, clippy::panic, clippy::indexing_slicing)
)]

/// Typed configuration types.
pub mod config;
/// Data types: chip identity, gestures, touch reports.
pub mod data;
/// I2C transport, device address, pin placeholder.
pub mod interface;
/// Register addresses and timing constants.
pub mod registers;

/// Common imports for application code.
pub mod prelude;

mod driver;
#[cfg(feature = "async")]
mod driver_async;
mod error;

#[cfg(test)]
mod test_helpers;

pub use driver::Cst816s;
#[cfg(feature = "async")]
pub use driver_async::Cst816sAsync;
pub use error::Error;
pub use interface::{DeviceAddress, NoPin};
