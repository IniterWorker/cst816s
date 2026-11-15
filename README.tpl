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

{{readme}}

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
