# rusty_esp_image — mission plan

**One sentence:** esp32-camera and Espressif's JPEG components remade in Rust —
sensor bring-up as data, DMA frame pools that never allocate, pixel operations
with scalar oracles, and JPEG/PNG through the Remade codecs — so a camera on an
ESP32 hands a validated, borrowed `Frame` to whoever asks for one.

Family plan: Janus `docs/plans/janus-mission.md`. Layer 1 · media. Depends on
`rusty_esp_core` only. `rusty_esp_video` depends on this crate's
`ImageSource`.

Written 2026-09-01. Status: **I0 shipped on the host** (`docs/LEDGER.md`); I1 needs a board.

---

## 1. Espressif map

| Espressif item | Job | Class | Janus |
|---|---|---|---|
| `esp32-camera` sensor drivers (`ov2640.c`, `ov3660.c`, `ov5640.c`, `ov7670.c`, `gc0308.c`, `gc032a.c`, `sc030iot.c`, `bf3005.c`) | register init tables, mode switch, flip/mirror/AE/AWB | **REMAKE as data** | `sensor::{ov2640, ov3660, ov5640, gc0308, …}` — `&'static [Register]` tables + a `Sensor` trait; re-derived from datasheets and the Apache-2.0 tables (attributed) |
| `esp32-camera` SCCB | I²C sensor control | REMAKE | `sccb::Sccb<I2c>` over `embedded-hal` 1.0 |
| `esp32-camera` `cam_hal` (S3 LCD_CAM DVP, classic ESP32 I2S-camera), XCLK via LEDC, DMA descriptors, frame buffers in PSRAM | capture | **WRAP** | `-esp`: Track A binds the C driver once (`esp-idf-sys`); Track B uses `esp_hal::lcd_cam::cam` (S3) |
| `esp_jpeg` (ROM `tjpgd` decode wrapper), `esp_new_jpeg` (SW encode/decode, S3 SIMD) | JPEG | REMAKE | `jpeg::{JpegProbe, JpegEncoder, JpegDecoder}` over `rusty_jpeg` (feature `jpeg`) |
| `esp_driver_jpeg` (P4 hardware JPEG) | HW JPEG | WRAP | `-esp` Track A P4 backend behind `JpegEncoder` |
| ESP-DL image preprocessing (resize, crop, normalize, color convert) | prep for models | REMAKE | `ops::*` |
| `esp_lcd`, LVGL glue | display | out of scope | a future `rusty_esp_display` |
| MIPI-CSI + ISP on P4 (`esp_video` V4L2-ish devices) | capture | WRAP (P2) | `-esp` Track A P4 backend |

## 2. Crate surface

### `rusty_esp_image-core` (`no_std`, `forbid(unsafe)`)

```rust
pub trait ImageSource {
    fn geometry(&self) -> Geometry;
    /// Capture one frame INTO `out`; the returned view borrows `out`.
    fn grab<'b>(&mut self, out: &'b mut [u8]) -> Result<Frame<'b>>;
}

/// N fixed buffers, no heap: the DMA ring in user clothes.
pub struct FramePool<'m, const N: usize> { /* slots over &'m mut [u8] */ }
impl FramePool { fn acquire(&mut self) -> Option<Slot>; fn release(&mut self, Slot); }

pub mod sensor {
    pub struct Register { pub addr: u16, pub value: u8 }   // tables are data
    pub struct Mode { pub geometry: Geometry, pub fps: u8, pub jpeg_quality: u8, pub flip: bool, pub mirror: bool }
    pub trait Sensor { fn id(&self) -> SensorId; fn init_table(mode: &Mode) -> &'static [Register]; /* … */ }
    pub mod ov2640; pub mod ov3660; pub mod ov5640; pub mod gc0308;
}
pub mod sccb   { pub struct Sccb<I: embedded_hal::i2c::I2c> { /* … */ } }   // 8/16-bit register access
pub mod jpeg   { pub struct JpegProbe;  /* SOF0/SOF2 → Geometry, no decode */
                 pub trait JpegEncoder { fn encode(&mut self, f: &Frame, quality: u8, out: &mut [u8]) -> Result<usize>; } }
pub mod ops    { rgb565_to_rgb888, rgb888_to_rgb565, yuyv_to_rgb565, yuyv_to_gray8,
                 downscale2x_gray8, downscale2x_rgb565, crop, rotate90_gray8, rotate180 }   // scalar oracles, tested
```

Rules: `grab` writes into caller memory; the pool is `const N` over a caller
slice; every `ops` kernel has a test vector and is the oracle for any later
PIE twin; `JpegProbe` parses only the marker headers it needs.

### `rusty_esp_image-esp`

| Feature | Backend | Notes |
|---|---|---|
| `esp-idf` | `IdfCamera: ImageSource` over the `esp32-camera` component (bound once through `esp-idf-sys`); P4 `esp_video` + `esp_driver_jpeg` | the honest label: *application in Rust, driver wrapped* |
| `esp-hal` | `HalCamera: ImageSource` over `esp_hal::lcd_cam::cam` (S3), DMA into PSRAM with cache alignment, `Sccb` over `esp_hal::i2c` | camera is behind esp-hal's `unstable` feature |

### `rusty_esp_image` (facade)

`prelude` = core prelude + `ImageSource`, `FramePool`, `Mode`, `JpegProbe`.

## 3. House crates

| Need | Use | Status / work item |
|---|---|---|
| JPEG encode | `rusty_jpeg` 0.3.3 encoder | its `JfifWrite` `no_std` trait exists; the crate dropped `#![no_std]` when encoder and decoder merged. **Upstream PR: feature ladder + `no_std` encoder + `platform_independent` verified on riscv32.** |
| JPEG decode (thumbnails, ops on sensor JPEG) | `rusty_jpeg` decoder | `std::io::Read`-based; needs a slice-based twin. Later. |
| PNG | `rusty_png` | DEFLATE backend blocks `no_std`; host-only for now |
| I²C | `embedded-hal` 1.0 | |
| Never | `esp_new_jpeg`'s C, `tjpgd` | |

## 4. Milestones and kill tests

| # | Deliverable | Kill test |
|---|---|---|
| **I0** ✅ 2026-09-01 | `ImageSource` + `TestPattern`, `FramePool`, `jpeg::probe` + `find_eoi`, `ops` with test vectors, `SensorId`/`SensorDesc`/`FrameSize`/`Mode`, OV2640 (302 steps) and OV5640 (216 steps) tables as data from esp32-camera (attributed), `Sccb` with delay steps and 1/2-byte PID probe | **passed:** 17 host tests; the probe reads geometry, components and the progressive flag back from 16 `rusty_jpeg`-encoded images (baseline + progressive, RGB + gray) — real sensor captures replace the encoder corpus at I1; the full OV2640 init table applies over a fake bus; riscv32 green with and without `alloc`; clippy clean. See `docs/LEDGER.md` |
| **I1** (J1) | Track A `IdfCamera` on XIAO S3 Sense (OV2640, JPEG mode) | 320×240 JPEG frames counted on serial at a recorded FPS for 10 minutes with zero pool exhaustion; every frame passes `Frame::packed` (starts `FF D8`) |
| **I2** | Track B `HalCamera` via `lcd_cam::cam` on S3, YUYV/RGB565 modes | same test, `no_std`; frame bytes byte-identical to Track A for a static test chart |
| **I3** | `rusty_jpeg` encoder `no_std` → on-chip encode of RGB565/YUYV frames | on-chip JPEG byte-identical to the host encoder for the same raw frame and quality |
| **I4** | P4: MIPI-CSI capture + hardware JPEG behind `JpegEncoder`; GC0308 / SC030IOT tables | 1080p JPEG at a recorded FPS; software vs hardware size/quality table |
| **I5** | PIE twins for `yuyv_to_rgb565`, `downscale2x`, `rgb565_to_rgb888` (via `rusty_esp_dsp` once it exists) | byte-identical vs scalar; ceiling probe recorded before writing any kernel |

## 5. Measurement

- Frame rate is a **counter** (frames/10 min) before it is a clock.
- Kernel wins are gated by the codec discipline: ceiling probe first, scalar
  oracle, byte-identity, a real-content fixture that provably enters the
  kernel (count the calls).
- `docs/LEDGER.md` from the first number.

## 6. Risks

| Risk | Mitigation |
|---|---|
| Sensor register tables copied rather than derived | tables re-derived from datasheets; Apache-2.0 sources attributed; a `LICENSE-THIRD-PARTY` note per table |
| PSRAM DMA cache coherency on S3/P4 | alignment asserted in the `-esp` backend; the one fenced `unsafe` per DMA boundary with its invariant |
| esp-hal camera driver still `unstable` | Track A first; Track B tracks the esp-hal release notes |
| `rusty_jpeg` upstream PR stalls | the encoder can ship behind a Janus-side `no_std` adapter of the `JfifWrite` path until merged; never a fork |

## 7. Decision log

| Date | Decision |
|---|---|
| 2026-09-01 | Sensor JPEG is the v1 codec (rusty-ESP-arduino §12). On-chip encode is I3, a choice not a requirement. |
| 2026-09-01 | Sensor drivers are register tables plus a tiny trait — data, not a driver framework. |
| 2026-09-01 | Display (`esp_lcd`) is a separate future package. |
