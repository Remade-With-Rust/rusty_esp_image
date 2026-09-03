# rusty_esp_image — ledger

Every number this package quotes lives here with the run that produced it.

## Correctness gates (host, 2026-09-01, I0)

| Gate | Result |
|---|---|
| `jpeg::probe` reads width, height, components and the progressive flag back from JPEGs encoded by `rusty_jpeg` 0.3 — baseline and progressive, RGB and grayscale, at 16×8, 160×120, 320×240 and 99×33 | pass, 16 images |
| `jpeg::probe` on hand-built headers (APP0 then SOF0 / SOF2), and rejection of non-JPEG, truncated, SOS-before-SOF and DNL (zero height) inputs | pass |
| `jpeg::find_eoi` trims trailing DMA padding to the EOI marker | pass |
| `ops` kernels against hand vectors: RGB565 pack/unpack extremes and round trip; BT.601 YUV points (gray, white, black, red, blue); YUYV → RGB888 / RGB565 / Gray8; 2× downscale gray and RGB565; crop; rotate 90° and 180° | pass |
| `FramePool`: acquire to exhaustion without blocking, commit, validated frame view, double-release refused, oversize commit refused | pass |
| `TestPattern`: deterministic across instances, marker row advances per frame, timestamps advance by `1/fps`, unsupported formats and small buffers refused | pass |
| `Sccb` over a fake I²C device: 8-bit and 16-bit register addressing, apply with delay steps reaching the caller's clock, probe on 1-byte (OV2640) and 2-byte (OV5640) product ids; the full OV2640 init table and OV5640 default table apply without error | pass |
| `sensor`: every product id round-trips; every descriptor agrees with its id; tables non-empty and bank-selected / 16-bit as expected | pass |
| `riscv32imac-unknown-none-elf` with `--no-default-features` and with `--features alloc` | compile |
| clippy `-D warnings`, all targets; `cargo fmt --check` | clean |

Unit tests: **17 pass**.

## Sizes

| Date | Quantity | Value | Method |
|---|---|---|---|
| 2026-09-01 | OV2640 register tables (7 tables) | **302 steps** | converter over esp32-camera `ov2640_settings.h` @ `202df95d7b1d`; terminators dropped |
| 2026-09-01 | OV5640 register tables (9 tables, incl. gamma and AWB) | **216 steps** (writes + delay steps) | converter over `ov5640_settings.h` @ same commit |

## J1 host half (2026-09-01)

`rusty_esp_image-esp` gained the Track A backend `idf::IdfCamera` (esp32-camera
behind `ImageSource`: init from a `CameraPins` + `Mode`, one copy per frame out
of the driver's PSRAM buffer, orientation control, deinit on drop) and the pin
maps `XIAO_ESP32S3_SENSE` and `AI_THINKER_ESP32_CAM`. It compiles only in an
ESP-IDF firmware build; the host gate (check, clippy, riscv32 both rungs) stays
green with the feature off.

**First Track A build (2026-09-01, this machine):** `IdfCamera` compiles for
`xtensa-esp32s3-espidf` against ESP-IDF v5.5.1 and esp32-camera **2.1.7**
(pulled by the IDF component manager, with `espressif/esp_jpeg` ^1.3.1 as its
dependency; pinned in the firmware's `components_esp32s3.lock`), through the
`esp_idf_sys::camera` bindings module. The union fields `pin_sccb_sda` /
`pin_sccb_scl` and `fb_count: usize` are as the bindings name them. Not run:
the sensor has not been powered yet.

## Not yet measured

- Frames per second, `empty_frames` and pool exhaustion on a XIAO ESP32-S3 Sense (I1; needs the board).
- Kernel throughput per chip; the PIE ceiling probes (I5).

## The no-panic gate (host, 2026-09-02)

Every parser that takes bytes from a wire, a store or a bus must return an
error on bad input, never panic — the house rule made a test:
`tests/no_panic.rs` feeds each one random inputs from an LCG (the same corpus
on every machine) and mutations of a valid encoding (bit flips, overwrites,
truncation, extension, insertion, removal), under `catch_unwind` so a failure
names the parser and prints the input.

| covered | result |
|---|---|
| `jpeg::probe`, `find_eoi`, `is_jpeg` (40 000: marker-biased random bytes and mutations of a minimal baseline JPEG) | no finding |

## JPEG encoding on the chip: `jpeg::encode` over rusty_jpeg 0.4 (host, 2026-09-03)

`rusty_jpeg` 0.4 is `no_std` + `alloc` with a caller-owned output
(`SliceWriter`) and packed YUYV input, which is what a camera pipeline on a
chip needed; `jpeg::encode` (feature `jpeg`) is the family's use of it: a
raw frame (YUYV as delivered, RGB888/BGR888/RGBA8888, Gray8) into a buffer
the caller sized with `max_bytes`, a baseline JPEG with the standard tables
out.

| gate | result |
|---|---|
| `cargo test --workspace --features rusty_esp_image-core/jpeg` | **18 pass** (3 new: colour bars round-trip through the house decoder with a mean error of 7 at quality 85 — the bars are all hard edges; a YUYV gray ramp coded as delivered comes back with its luma; the refusals name their reason: `BufferTooSmall { needed: max_bytes }`, `InvalidGeometry`, `Unsupported` for planar) |
| `image-core --no-default-features --features jpeg` | riscv32imac, riscv32imafc and `xtensa-esp32s3-none-elf` (esp toolchain, `build-std=core,alloc`): pass |
| `cargo clippy`, `cargo deny` | clean |
| the probe's oracle (`real_jpegs_from_the_house_encoder`) on 0.4 | pass, unchanged |
