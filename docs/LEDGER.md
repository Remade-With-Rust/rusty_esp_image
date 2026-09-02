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
