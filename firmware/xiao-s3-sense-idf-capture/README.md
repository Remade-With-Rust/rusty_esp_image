# xiao-s3-sense-idf-capture

Janus **I1** firmware, Track A (std on ESP-IDF): the XIAO ESP32-S3 Sense
camera measured against the chip's own clock, with the driver's event
counters printed beside the rate, and the frame-buffer pool deliberately
starved so pool exhaustion is a number rather than a hope.

There is no network here. The I1 row's three quantities are the camera's, and
a radio would only add a variable. Frames still reach the laptop for an
outside opinion, over the serial link instead of a socket.

## What it measures

Six arms: three pool sizes (`fb_count` 1, 2, 3) crossed with two consumers.

| consumer | what it is | what it should show |
|---|---|---|
| `consumer_ms=0` | grab and discard | the sensor's own ceiling |
| `consumer_ms=60` | grab, then sleep longer than a frame period | the pool absorbing a slow consumer, or failing to |

A pool only runs dry when something downstream is slower than the sensor, so
the second column is the one that can produce `no_frame`. That counter is new:
`CaptureStats` splits the driver's failures into `no_frame` (nothing to hand
over, which is what a starved pool looks like from the caller's side),
`empty_frames` (a buffer arrived carrying nothing) and `too_small` (the
caller's slot was short). They used to be one number, which made a starved
pool indistinguishable from a broken sensor.

Counters are the primary evidence; the frame rate is derived from them and one
clock and is the confirmation. The rate arms print nothing while running, so
the measurement is the camera's and not the serial link's — the dump is a
separate pass afterwards (codec-measurement 13).

## Build and run

```sh
export CARGO_TARGET_DIR=C:/janus-s3     # Windows: the IDF build refuses long paths
cargo metadata --filter-platform=xtensa-esp32s3-espidf --format-version 1 >/dev/null
cargo build --release

espino flash   --board xiao-esp32s3-sense --port COM4 --app $CARGO_TARGET_DIR/xtensa-esp32s3-espidf/release/xiao-s3-sense-idf-capture
espino monitor --board xiao-esp32s3-sense --port COM4 --expect "== DONE ==" --timeout 300 > capture.txt
```

The esp environment must be in the shell first: the esp-clang and
xtensa-esp-elf `bin` directories on `PATH`, `LIBCLANG_PATH` at esp-clang's
`libclang.dll`, and no active Python virtual environment.

## The oracle

The board saying "I captured 150 frames" is a self-metric. `tools/` holds the
outside instrument:

```sh
python tools/decode-jpeg-dump.py capture.txt frames/
```

It reassembles the `JPEGDATA` hex lines into `.jpg` files and checks four
things per frame: the byte count the firmware announced against the bytes that
arrived, the SOI and EOI markers (a driver handing back a short buffer fails
exactly here, and a frame counter cannot see it), ffprobe's geometry against
what the firmware configured, and whether ffmpeg can decode the entropy-coded
data to completion. ffprobe reads a truncated frame's header happily; only the
decode catches it, which is why both run.

## Notes

- QVGA JPEG at quality 12. The frame slot is 96 KB, far above the 15-25 KB a
  frame of this size runs, so a non-zero `too_small` means something real.
- Each arm re-initialises the driver, because `fb_count` is fixed at
  `esp_camera_init`. `IdfCamera`'s `Drop` deinitialises.
- The first three frames of every arm are discarded untimed: the sensor's
  automatic exposure and gain move after a start, and those frames are neither
  typical nor the thing under measurement.
- Numbers go to `rusty_esp_image/docs/LEDGER.md` with their method line.
