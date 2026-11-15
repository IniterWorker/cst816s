# CST816S

<p align="left">
    <a href="https://github.com/initerworker/cst816s/actions/workflows/rust-release.yml"><img src="https://github.com/initerworker/cst816s/actions/workflows/rust-release.yml/badge.svg" alt="Github CI/CD"></a>
    <a href="https://crates.io/crates/cst816s-rs"><img src="https://img.shields.io/crates/v/cst816s-rs.svg" alt="Crates.io"></a>
    <a href="https://docs.rs/cst816s-rs"><img src="https://docs.rs/cst816s-rs/badge.svg" alt="Docs.rs"></a>
</p>

Rust I2C driver for the Hynitron CST816 capacitive touch family
(CST716, CST816S, CST816T, CST816D, CST820, CST826, CST830, CST836U),
blocking and async over embedded-hal 1.0.

## Firmware variants

The CST816 ships blank from the factory; module vendors flash
per-project firmware, so register behaviour, gesture sets, and even
reported chip IDs are firmware-defined rather than silicon-defined.
This driver therefore treats probe as informational (an unknown chip
ID is never a fatal error), models the register set from the vendor
Register Declaration, and keeps every register reachable through the
`read_register`/`write_register` escape hatch for firmware-specific
extensions. Bootloader/firmware update (0x6A programming mode) is
deliberately not supported: firmware blobs are per-project
proprietary, and a bad update bricks the module.

If you have any additional information, please do not hesitate to submit an issue with it.

## Open-To-Idea

Please feel free to open an issue with your design improvements/considerations.

## CST816 capacitive touch driver

A `no_std` driver for the Hynitron CST816 family (CST716, CST816S,
CST816T, CST816D, CST820, CST826, CST830, CST836U), built on
`embedded-hal` 1.0 (blocking, [`Cst816s`]) and, behind the `async`
feature, `embedded-hal-async` 1.0 (`Cst816sAsync`). I2C only.

The CST816 ships blank from the factory and is flashed with
per-project firmware by the module vendor, so register behaviour
is firmware-defined: this driver reports identity verbatim and
never fails probe on an unknown chip ID. Firmware-specific
registers remain reachable through the `read_register` /
`write_register` escape hatch.

### Supported chips

| Variant | ID | Sleep register | Touch wake |
|---|---|---|---|
| CST716 | 0x20 @ ChipId | PowerMode (0xA5) | no |
| CST816S | 0xB4 @ ChipId | SleepMode (0xE5) | yes |
| CST816T | 0xB5 @ ChipId | SleepMode (0xE5) | yes |
| CST816D | 0xB6 @ ChipId | SleepMode (0xE5) | no |
| CST820 | 0xB7 @ ChipId | SleepMode (0xE5) | no |
| CST826 | 0x11 @ FactoryId | PowerMode (0xA5) | no |
| CST830 | 0x12 @ FactoryId | PowerMode (0xA5) | no |
| CST836U | 0x13 @ FactoryId | PowerMode (0xA5) | no |
| Unknown | verbatim | SleepMode (0xE5) | no |

### Features

| Feature | Default | Effect |
|---|---|---|
| `async` | no | async driver `Cst816sAsync` over `embedded-hal-async` |
| `defmt` | no | `defmt::Format` implementations for public types |
| `log` | no | `log` diagnostics (probe result, touch reports) |

The blocking driver is always available.

### Quickstart (blocking)

```rust
use cst816s::{Cst816s, NoPin};
let mut touch = Cst816s::new(i2c, delay).with_reset_pin(rst);
let info = touch.init().unwrap();
let report = touch.read_touch().unwrap();
if report.pressed {
    // report.x, report.y, report.gesture
}
```

### Async (Embassy-style, `async` feature)

The async driver is executor-agnostic; it only needs
`embedded-hal-async` bus and delay implementations from your HAL.

```rust
use cst816s::Cst816sAsync;

// `i2c`, `delay`, `rst`, `int` are your HAL's types; `int` must
// implement embedded_hal_async::digital::Wait for wait_for_touch.
let mut touch = Cst816sAsync::new(i2c, delay)
    .with_reset_pin(rst)
    .with_interrupt_pin(int);
touch.init().await?;
let report = touch.wait_for_touch().await?;
```

### Sources

- CST816S Datasheet V1.4 EN (Waveshare)
- CST816S Register Declaration (SDK register description,
  2019-08-30, Waveshare)
- cst78XX debug doc V2.3 (Hynitron: bootloader/sleep/mode facts)
- Zephyr `drivers/input/input_cst8xx.c`, Linux
  `hynitron-cst816x.c`, InfiniTime, wasp-os, ESPHome, Espressif
  `esp_lcd_touch_cst816s`

## Development

A reproducible development environment is provided as a Nix flake
(stable Rust with clippy/rustfmt/rust-analyzer, `thumbv7em-none-eabihf` /
`thumbv7m-none-eabi` / `riscv32imc-unknown-none-elf` targets for the
`no_std` cross checks, `just`, `cargo-hack`, and `cargo-readme`):

```sh
nix develop        # or: direnv allow (uses .envrc)
just ci            # fmt, clippy, tests, cross-target feature matrix, docs, README
```

`nix flake check` builds the crate and runs the host test suites with
all features in the Nix sandbox (`nix build` produces the same check
as a package).

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the
work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any
additional terms or conditions.
