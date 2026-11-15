# Changelog

Rust implementation I2C driver of the cst816s, cst816t, cst816d touch screen IC.

<!-- next-header -->

## [Unreleased] - ReleaseDate

## [0.2.0] - 2026-11-15

Complete rewrite (breaking):

- Blocking `Cst816s` and async `Cst816sAsync` (`async` feature) drivers
  with identical wire behaviour, built on embedded-hal 1.0 /
  embedded-hal-async 1.0. Optional driver-owned RST/INT pins (`NoPin`
  placeholders), async `wait_for_interrupt`/`wait_for_touch`.
- Whole CST816 family: CST716, CST816S/T/D, CST820, CST826, CST830,
  CST836U identified via ChipId/FactoryId; probe is non-fatal on
  unknown IDs (per-project firmware defines behaviour).
- Correct 6-byte touch frame parsing, `Gesture`, `TouchEventKind`,
  coordinate transforms, and a `GestureLatch` slide de-duplicator.
- Full configuration surface: motion mask, IRQ control/pulse width,
  scan periods, low-power profile burst, report-mode presets, deep
  sleep (reset-first) and wake (full re-init).
- Fixed defects of 0.1.0: frame off-by-one indexing, invented
  10-point pressure/area fields, masked/swapped gestures, the
  `(value & 0xff00 >> 8)` precedence bug, overlapping IrqCtl fields.
- Removed `bitfield`/`libm`/`thiserror` dependencies; features are now
  `async`, `defmt`, `log` (old `loglib`/`std` removed, no aliases).
- Edition 2024, MSRV 1.85. Nix flake dev environment with host-test
  check package.

## [0.1.0]

- Initial release.
