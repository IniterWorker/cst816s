# CST816S

<p align="left">
    <a href="https://github.com/initerworker/cst816s/actions/workflows/rust-release.yml"><img src="https://github.com/initerworker/cst816s/actions/workflows/rust-release.yml/badge.svg" alt="Github CI/CD"></a>
    <a href="https://crates.io/crates/cst816s-rs"><img src="https://img.shields.io/crates/v/cst816s-rs.svg" alt="Crates.io"></a>
    <a href="https://docs.rs/cst816s-rs"><img src="https://docs.rs/cst816s-rs/badge.svg" alt="Docs.rs"></a>
</p>

Rust implementation I2C driver of the cst816s, cst816t, cst816d touch screen IC.

## Disclaimer

The CST816S IC firmware and documentation are incomplete and lack critical information.
There are several undocumented magic values, features that are claimed but do not work,
and important details that are absent from both the datasheet and the documentation.
As a result, it is recommended to limit the usage of this IC to basic point-and-click actions.

If you have any additional information, please do not hesitate to submit an issue with it.

WIP...

## Open-To-Idea

Please feel free to open an issue with your design improvements/considerations.

## CST816S Touch Screen IC

### Support

- [Embedded-hal v1.0.0](https://github.com/rust-embedded/embedded-hal/tree/embedded-hal-v1.0.0)

### Example

- [Waveshare esp32-s3-touch-lcd-1-28](https://github.com/IniterWorker/esp32-s3-touch-lcd-1-28)


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
