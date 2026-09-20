### In The Wild with 42 Active Installs

FREE RAG Converter Online -- <a href="https://RAGconverter.com">RAGconverter.com</a>

# rusty_esp_image

[![Remade With Rust](https://img.shields.io/badge/Remade%20With-Rust-000?logo=rust&logoColor=fff)](https://github.com/remade-with-rust) [![By Mata Network](https://img.shields.io/badge/by-Mata%20Network-5b2be0)](https://www.mata.network) [![crates.io](https://img.shields.io/crates/v/rusty_esp_image.svg)](https://crates.io/crates/rusty_esp_image) [![docs.rs](https://docs.rs/rusty_esp_image/badge.svg)](https://docs.rs/rusty_esp_image) [![License: MIT OR Apache-2.0](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue)](https://github.com/Remade-With-Rust/rusty_esp_image/blob/main/LICENSE-MIT)

Cameras for the **Janus** ESP32 family: sensor bring-up over the two-wire
control bus, a frame pool that hands out borrowed views, JPEG probing and
encoding, and a deterministic test pattern so a pipeline can be exercised with
no camera attached. Pure Rust, no C, no FFI, `no_std` by default.

* **A picture, on a five-dollar board.** A generated firmware brings up an
  OV2640, mints an identity nobody can reach, and serves a gated camera page —
  verified on two different boards, one of them provisioned entirely over
  Bluetooth from a browser.
* **The frame pool does not lie about exhaustion.** Measured on the chip across
  581 captures, including runs where the pool was starved on purpose: **every
  failure counter reads zero.** The driver does not report running out of
  buffers — it waits, silently, and the only trace is a lower frame rate. A
  firmware watching that counter would watch forever.
* **Sensor tables as data, not code.** Register sequences for the OV2640 and
  OV5640 converted from the vendor's own headers — 216 steps for the OV5640's
  nine tables including gamma and white balance — each replayed through the
  control bus with the delays the datasheet asks for.
* **JPEG in and out** through [`rusty_jpeg`](https://crates.io/crates/rusty_jpeg):
  probe width, height, components and the progressive flag from a header;
  trim the trailing DMA padding to the end marker; encode on the chip.

## What has run on hardware

| what | measured |
|---|---|
| frame buffers, fast consumer | one buffer **13.882 fps**, two **27.764 fps**, three **27.764 fps** |
| the second buffer | costs exactly half the frame rate to omit — a factor of **2.000003** from the raw counts, because the driver cannot fill the next frame while the caller holds the current one |
| the third buffer | **buys nothing**: two and three produced the same frame count over elapsed times less than one part in a million apart |
| pool exhaustion | **0 failures in all six runs, 581 captures**, including deliberately starved ones |
| frames off the board | decoded by an outside tool, three of three, three distinct checksums |

The firmware generator already asks for two buffers. This is the measurement
that says it was right to, and the third column is why the roadmap stopped
listing pool exhaustion as a quantity to count.

Every number, with the run that produced it:
[`docs/LEDGER.md`](https://github.com/Remade-With-Rust/rusty_esp_image/blob/main/docs/LEDGER.md).

## Using it

```rust
use rusty_esp_image::prelude::*;

let mut camera = IdfCamera::new(XIAO_ESP32S3_SENSE, Mode::new(FrameSize::Qvga, 12))?;
// The pool owns the memory; a grab hands back a borrowed view of it.
if let Some(frame) = camera.grab()? {
    let info = jpeg::probe(frame.bytes())?;      // width, height, components
    let clean = jpeg::find_eoi(frame.bytes());   // trim trailing DMA padding
}
```

## Two tracks

| track | what it is | this crate |
|---|---|---|
| **A** | `std` on ESP-IDF — the camera driver, the frame pool, JPEG | `rusty_esp_image-esp --features esp-idf` |
| **B** | `no_std` on `esp-hal` | `rusty_esp_image-core`, default |

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
