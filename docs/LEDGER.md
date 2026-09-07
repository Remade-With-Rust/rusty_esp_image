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

## I1 on the board: frame rate, the driver's counters, and a starved pool (2026-09-06)

`firmware/xiao-s3-sense-idf-capture` on a Seeed XIAO ESP32-S3 Sense, Track A
(ESP-IDF v5.5.1, esp32-camera 2.x), QVGA JPEG at quality 12 out of PSRAM.
No network: the row's quantities are the camera's, and a radio would only add
a variable. The sensor answered as an OV3660 (this board ships either that or
an OV2640; the board record says so).

Method line: `board=xiao-esp32s3-sense geometry=320x240 format=jpeg quality=12
metric=in-process-us secs_per_arm=5 arms=6 warmup=3-frames-discarded
work=frames-and-bytes-counted null_floor=not-established`. One arm per cell,
no ABBA: this is a cost and behaviour table for a part with no twin, not an
A/B, and nothing here is a speed claim about anything but this camera at this
geometry.

Six arms: three pool sizes crossed with two consumers. A pool only runs dry
when something downstream is slower than the sensor, so the second consumer
sleeps 60 ms after each grab — longer than any frame period this sensor
produces — to make that happen rather than hope for it.

| `fb_count` | consumer | frames | fps | mean JPEG | `no_frame` | `empty_frames` | `too_small` |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 | none | 70 | **13.882** | 3 719 B | 0 | 0 | 0 |
| 1 | 60 ms | 69 | 13.618 | 3 723 B | 0 | 0 | 0 |
| 2 | none | 139 | **27.764** | 3 718 B | 0 | 0 | 0 |
| 2 | 60 ms | 82 | 16.278 | 3 717 B | 0 | 0 | 0 |
| 3 | none | 139 | **27.764** | 3 716 B | 0 | 0 | 0 |
| 3 | 60 ms | 82 | 16.277 | 3 715 B | 0 | 0 | 0 |

### Three findings, and the third changes what a firmware should watch

**One buffer costs exactly half the frame rate.** 13.882 against 27.764 is a
factor of 2.000003 computed from the raw counts. With a single buffer the driver cannot fill the next frame
while the caller holds the current one, so it misses every second frame the
sensor produces. This is not a subtle penalty and it is invisible in any
counter.

**The third buffer buys nothing.** `fb_count` 2 and 3 measured 27.764 fps
both times, from 139 frames each, over 5 006 479 and 5 006 483 us — the two
elapsed times agree to 0.8 parts per million. Two buffers already decouple
the sensor from the consumer at this rate, so a streaming firmware should ask
for two and keep the third frame buffer's PSRAM. The generated projects
already pass 2 (`IdfCamera::init(&pins, &mode, 2)`); this is the measurement
that says they are right.

**Pool exhaustion does not appear in any counter.** Every arm reports
`no_frame` = `empty_frames` = `too_small` = 0, and `attempts` equals `frames`
in all six — 581 grabs, not one failure. `esp_camera_fb_get` **blocks** until
a buffer is ready rather than returning null, so a starved pool is never an
error on this driver. It is only ever a lower frame rate.

That is worth stating plainly because the row was written expecting the
opposite: **a firmware watching `no_frame` to detect a starved pool will
watch forever.** The signal is the rate. The counters earn their place by
proving the failures are absent — which is what makes the fps number mean
what it says — not by being the alarm.

The slow-consumer column confirms the mechanism from the other side. At
60 ms per frame a consumer cannot exceed 16.67 fps whatever the camera does;
`fb_count` 2 and 3 both measured 16.28, so the pool hands over as fast as the
consumer will take and the camera is no longer the limit. `fb_count` 1
measured 13.6, barely below its own 13.9 ceiling, because at 72 ms per frame
its own wait already hides a 60 ms sleep.

### The counters this needed, and why they were split

`CaptureStats` replaces a single public `empty_frames` field. The driver has
three distinct failures and they used to be one number, which made a starved
pool indistinguishable from a broken sensor:

- `no_frame` — `esp_camera_fb_get` returned null; nothing to hand over.
- `empty_frames` — a buffer arrived with zero length or a null pointer.
- `too_small` — the frame did not fit the caller's slot.

Plus `frames` and `bytes`, so a rate is derived from an exact count rather
than being the only thing measured. The frame slot is 96 KB against a
3.7 KB frame, so a non-zero `too_small` would mean something real.

### The oracle

The board's own frame count is a self-metric. `tools/decode-jpeg-dump.py`
reassembles frames from the serial dump and hands them to ffprobe and ffmpeg:

| frame | announced | arrived | SOI/EOI | ffprobe | ffmpeg |
|---|---:|---:|---|---|---|
| 0 (seq 10) | 3 721 | 3 721 | intact | mjpeg 320x240 yuvj422p | decoded, crc 0x5ddad35f |
| 1 (seq 11) | 3 734 | 3 734 | intact | mjpeg 320x240 yuvj422p | decoded, crc 0xab1d795f |
| 2 (seq 12) | 3 698 | 3 698 | intact | mjpeg 320x240 yuvj422p | decoded, crc 0xe76d99b9 |

3 of 3 survived every check. Both tools run because they answer different
questions: ffprobe reads a truncated frame's header happily, and only a full
decode catches a short buffer. The three checksums differ, so these are three
different pictures rather than one frame repeated — and the decoded image is
the room's ceiling fan, which is where the board was pointing.

Sequence numbers 10, 11, 12 are consecutive: no drops between them.

### What is not measured here

Frame rate over Wi-Fi, which is V1's row and needs an access point this bench
does not have. The number above is what the camera can produce, and it is the
ceiling any transport works under.
