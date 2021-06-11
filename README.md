![Maintenance](https://img.shields.io/badge/maintenance-experimental-blue.svg)
[![crates.io](https://img.shields.io/crates/v/libftd2xx-cc1101.svg)](https://crates.io/crates/libftd2xx-cc1101)
[![docs.rs](https://docs.rs/libftd2xx-cc1101/badge.svg)](https://docs.rs/libftd2xx-cc1101/)
[![CI](https://github.com/CirrusNeptune/libftd2xx-cc1101/workflows/CI/badge.svg)](https://github.com/newAM/libftd2xx-cc1101/actions)

# libftd2xx-cc1101

I/O library for using [CC1101] RF transceiver via [libftd2xx-rs].

## Usage
Simply add this crate as a dependency in your `Cargo.toml`.

```toml
[dependencies]
libftd2xx-cc1101 = "~0.1.0"
```

Also follow the README guidance in [libftd2xx-rs] to set up the connection with the FTDI device.

[RX] and [TX] examples are good starting points.

## References

* [CC1101 Datasheet]

[CC1101]: https://www.ti.com/product/CC1101
[libftd2xx-rs]: https://github.com/newAM/libftd2xx-rs
[RX]: https://github.com/CirrusNeptune/libftd2xx-cc1101/blob/main/examples/rx.rs
[TX]: https://github.com/CirrusNeptune/libftd2xx-cc1101/blob/main/examples/tx.rs
[CC1101 Datasheet]: https://www.ti.com/lit/gpn/cc1101
