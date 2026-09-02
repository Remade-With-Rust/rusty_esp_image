# rusty_esp_image

[![license](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue.svg)](#license)

esp32-camera and Espressif's JPEG components remade in Rust: sensor bring-up
as **data**, DMA frame pools that never allocate, pixel kernels with scalar
oracles, a JPEG header probe, and SCCB register access over `embedded-hal` —
so a camera on an ESP32 hands a validated, borrowed frame to whoever asks for
one.

Part of **Janus**, the Remade-With-Rust programme that rebuilds the Espressif
ESP32 and Arduino application portfolio in memory-safe Rust for the MATA home
computer.

- This package's plan: [docs/plans/rusty_esp_image.md](docs/plans/rusty_esp_image.md)
- Numbers: [docs/LEDGER.md](docs/LEDGER.md)
- Third-party data: [LICENSE-THIRD-PARTY.md](LICENSE-THIRD-PARTY.md)
- The family plan: Janus `docs/plans/janus-mission.md` (umbrella repo)

**Claims discipline:** every number in this README is in the ledger with the
run that produced it. Nothing here has run on a chip yet.

## Status

**I0 shipped on the host (2026-09-01).** The core has everything a capture
driver needs *above* the DMA engine: the `ImageSource` seam, a caller-owned
`FramePool`, a JPEG header probe verified against the house encoder, pixel
kernels with test vectors, the OV2640 and OV5640 register tables as data
(derived mechanically from esp32-camera, Apache-2.0, attributed), sensor
descriptors, and SCCB register access. 17 tests pass; the core compiles for
riscv32 bare metal with and without `alloc`.

**J1 host half (2026-09-01):** `rusty_esp_image-esp::idf::IdfCamera` — the
Track A capture backend over esp32-camera — and the XIAO ESP32-S3 Sense and
AI-Thinker pin maps are written; the firmware that links them lives in
`rusty_esp_video/firmware/xiao-s3-sense-idf-mjpeg` and **builds** for the
XIAO ESP32-S3 Sense against esp32-camera 2.1.7 (`docs/LEDGER.md`). Not run
on a sensor yet.

Not yet: the board (I1's frame count), the DVP engine over `lcd_cam` (Track
B, I2), MIPI-CSI on P4 (I4), and on-chip JPEG encoding via `rusty_jpeg` (I3,
waiting on its `no_std` encoder).

## What is in the core

| Module | What |
|---|---|
| `source` | `ImageSource`: `geometry()` and `grab(out) -> Frame` into caller memory; `TestPattern` colour bars for host and smoke tests |
| `pool` | `FramePool<N>`: N equal slots over one buffer, acquire / fill / commit / frame / release, never blocks, never allocates |
| `jpeg` | `probe`: width, height, precision, components, progressive flag from the SOF segment, no decode; `find_eoi` trims DMA padding |
| `ops` | RGB565 ↔ RGB888, YUYV → RGB888 / RGB565 / Gray8, 2× box downscale (gray, RGB565), crop, rotate 90° and 180° — scalar, the oracles for any PIE twin |
| `sensor` | `SensorId` (18 parts, product ids), `SensorDesc` (OV2640, OV5640, OV3660, OV7670), `FrameSize`, `Mode`, `RegOp`; `ov2640` (302 steps) and `ov5640` (216 steps) register tables |
| `sccb` | `Sccb<I2c>`: 8/16-bit register read and write, `apply` a table with delay steps, `probe` a product id |

```rust
use rusty_esp_image::prelude::*;

// a DMA ring in user clothes: two slots over one static buffer
static mut RING: [u8; 2 * 64 * 1024] = [0; 2 * 64 * 1024];
let mut pool: FramePool<'_, 2> = FramePool::new(ring)?;
let slot = pool.acquire().ok_or(Error::Busy)?;
let n = camera.grab_into(pool.slot_mut(slot)?)?;        // the -esp backend fills it
pool.commit(slot, n)?;
let info = probe(pool.slot(slot)?)?;                    // geometry from the JPEG header
let frame = pool.frame(slot, info.geometry, clock.now(), seq)?;
```

## Layout

```text
crates/rusty_esp_image          facade
crates/rusty_esp_image-core     no_std + alloc; forbid(unsafe); the core above
crates/rusty_esp_image-esp      the WRAP crate: `esp-hal` | `esp-idf` capture engines (I1)
docs/plans/rusty_esp_image.md   the plan · docs/LEDGER.md the numbers
LICENSE-THIRD-PARTY.md          provenance of the register tables
```

## Build

```sh
cargo test --workspace
cargo check -p rusty_esp_image-core --no-default-features --target riscv32imac-unknown-none-elf
cargo check -p rusty_esp_image-core --no-default-features --features alloc --target riscv32imac-unknown-none-elf
```

## License

MIT OR Apache-2.0, at your option. The sensor register tables are derived
from Apache-2.0 material; see `LICENSE-THIRD-PARTY.md`.
