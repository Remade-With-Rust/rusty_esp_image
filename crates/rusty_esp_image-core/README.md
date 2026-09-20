# rusty_esp_image-core

[![Remade With Rust](https://img.shields.io/badge/Remade%20With-Rust-000?logo=rust&logoColor=fff)](https://github.com/remade-with-rust) [![By Mata Network](https://img.shields.io/badge/by-Mata%20Network-5b2be0)](https://www.mata.network) [![crates.io](https://img.shields.io/crates/v/rusty_esp_image-core.svg)](https://crates.io/crates/rusty_esp_image-core) [![docs.rs](https://docs.rs/rusty_esp_image-core/badge.svg)](https://docs.rs/rusty_esp_image-core) [![License: MIT OR Apache-2.0](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue)](https://github.com/Remade-With-Rust/rusty_esp_image/blob/main/LICENSE-MIT)

The pure half of the camera package: the image source seam, a caller-owned frame pool, a JPEG header probe, the pixel kernels with their scalar oracles, sensor register tables as data, and register access over the two-wire control bus. `no_std`, `forbid(unsafe)`.

Sensor bring-up is **data, not code** — register sequences converted from the vendor's own headers, replayed with the delays the datasheet asks for, so a new sensor is a table rather than a driver.

## Where the evidence is

This crate is part of [`rusty_esp_image`](https://crates.io/crates/rusty_esp_image). The
hardware results, the method lines and the open defects live in that package's
[README](https://github.com/Remade-With-Rust/rusty_esp_image#readme) and in
[`docs/LEDGER.md`](https://github.com/Remade-With-Rust/rusty_esp_image/blob/main/docs/LEDGER.md), where no number
appears without the run that produced it.

## On an ESP32-S3

Turn on `pie-s3` and `rotate90_gray8` runs the chip's 128-bit vector twin —
**201,137 → 12,329 picoseconds per pixel, −93.9%** on a Seeed XIAO ESP32-S3
Sense, gated byte-identical against the tiled scalar loop that stays in the
tree.

```toml
rusty_esp_image-core = { version = "0.1", features = ["pie-s3"] }
```

The rotate is an 8x8 byte transpose in eight `ee.vzip` instructions, and the
destination column's reversal costs nothing: loading each tile's source rows
bottom-to-top emits the transposed bytes in the order the destination run
already wants. It takes only `w % 8 == 0`, `h % 8 == 0` and 16-byte aligned
buffers, and declines to the scalar loop otherwise.

`rotate180` has no twin and will not get one: reversing bytes complements the
lane index, and this unit's `zip`/`unzip` only rotate it. `crop` needs none —
it is already `copy_from_slice` per row.

## Part of Janus

**Janus** rebuilds the Espressif ESP32 and Arduino application portfolio as
independent, memory-safe Rust packages — so a hardware maker can ship a device
that the [MATA](https://www.mata.network) home computer discovers, catalogs honestly, adopts
under its own identity, and pays for. Ten packages, three layers, and the
dependency direction never reverses.

| layer | packages |
|---|---|
| **0 — the vocabulary** | [`rusty_esp_core`](https://crates.io/crates/rusty_esp_core) · [`rusty_esp_dsp`](https://crates.io/crates/rusty_esp_dsp) |
| **1 — the functions** | [`rusty_esp_image`](https://crates.io/crates/rusty_esp_image) · [`rusty_esp_video`](https://crates.io/crates/rusty_esp_video) · [`rusty_esp_audio`](https://crates.io/crates/rusty_esp_audio) · [`rusty_esp_signal`](https://crates.io/crates/rusty_esp_signal) · [`rusty_esp_mid`](https://crates.io/crates/rusty_esp_mid) · [`rusty_esp_iroh`](https://crates.io/crates/rusty_esp_iroh) |
| **2 — the surfaces** | [`rusty_esp_arduino`](https://crates.io/crates/rusty_esp_arduino) — the sketch facade · `espino` — the maker's CLI (not published) |

Every package is host-verified against an external oracle and keeps a ledger
in which no number appears without the run that produced it. **Five of seven
device profiles have now run their kill tests on real silicon**, three of them
over a Wi-Fi network the board hosts itself.

Also check out the rest of [Remade With Rust](https://github.com/remade-with-rust) — including
[`rusty_alloc`](https://crates.io/crates/rusty_alloc), the pure-Rust rebuild of
mimalloc that these firmwares run on, and
[`rusty_jpeg`](https://crates.io/crates/rusty_jpeg), the JPEG engine behind the
camera path — and our sister project
[remade_ffmpeg_rs](https://github.com/Remade-With-Rust/remade_ffmpeg_rs), a ground-up Rust rebuild of FFmpeg.

## About Mata Network

[Mata Network](https://www.mata.network) builds sovereign, self-hostable infrastructure.
**Remade With Rust** is our open-source home for the permissively-licensed
building blocks that work depends on.

## License

MIT OR Apache-2.0, at your option. See [LICENSE-MIT](https://github.com/Remade-With-Rust/rusty_esp_image/blob/main/LICENSE-MIT)
and [LICENSE-APACHE](https://github.com/Remade-With-Rust/rusty_esp_image/blob/main/LICENSE-APACHE).
