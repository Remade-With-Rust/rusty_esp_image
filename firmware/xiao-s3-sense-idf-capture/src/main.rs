//! I1 on real silicon: what the XIAO ESP32-S3 Sense's camera delivers, and
//! what its frame-buffer pool does when the consumer is slow.
//!
//! The I1 row asks for three things: frames per second, `empty_frames`, and
//! pool exhaustion. The first two a single run answers. The third is not a
//! number you read off a working system, because a pool only runs dry when
//! something downstream is slower than the sensor, so this firmware **makes**
//! that happen: every pool size is measured twice, once with a consumer that
//! does nothing but grab, and once with one that sleeps longer than a frame
//! period. The difference between the two columns is the pool doing its job
//! or failing to.
//!
//! Counters are the primary evidence and the clock is confirmation, so the
//! driver's own event counts are printed beside the rate rather than folded
//! into it. `CaptureStats` keeps the three failure modes apart on purpose:
//! a starved pool and a broken sensor are different faults and used to be
//! the same number.
//!
//! Two passes, never one loop. The rate arms write nothing to the console,
//! so the frame rate is the camera's and not the serial link's; the dump
//! afterwards captures a fixed number of frames and its timing is
//! irrelevant (codec-measurement 13).
//!
//! Nothing here needs a network. Lines are prefixed `CAM` and `JPEG`.

use std::fmt::Write as _;
use std::thread::sleep;
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use esp_idf_svc::log::EspLogger;
use esp_idf_svc::sys::link_patches;
use rusty_esp_image_core::esp_core::frame::Planes;
use rusty_esp_image_core::sensor::{FrameSize, Mode};
use rusty_esp_image_core::source::ImageSource;
use rusty_esp_image_esp::idf::IdfCamera;
use rusty_esp_image_esp::XIAO_ESP32S3_SENSE;

/// Seconds each arm runs. Long enough that the sensor's own settling is a
/// rounding error against the frame count.
const MEASURE_SECS: u64 = 5;
/// Driver-side frame buffers to try. One is the degenerate case the driver
/// warns about, three is more than a streaming firmware would ask for.
const POOLS: [u8; 3] = [1, 2, 3];
/// Milliseconds the consumer sleeps after each grab. Zero is a consumer that
/// cannot be the bottleneck; 60 ms is slower than any frame period this
/// sensor produces, so the pool has to absorb it or starve.
const CONSUMER_MS: [u64; 2] = [0, 60];
/// The caller's frame slot. A QVGA JPEG at this quality runs 15-25 KB; this
/// is deliberately far above that so `too_small` stays at zero and a
/// non-zero reading means something real.
const FRAME_SLOT: usize = 96 * 1024;
/// Frames dumped for the laptop to judge.
const DUMP_FRAMES: usize = 3;

fn main() -> Result<()> {
    link_patches();
    EspLogger::initialize_default();

    let mut mode = Mode::jpeg(FrameSize::Qvga).context("jpeg mode")?;
    mode.jpeg_quality = 12;
    let (w, h) = (mode.geometry.width, mode.geometry.height);

    println!("== JANUS CAPTURE xiao-s3 ==");
    println!(
        "CAM geometry={w}x{h} format=jpeg quality={} secs_per_arm={MEASURE_SECS} slot_bytes={FRAME_SLOT}",
        mode.jpeg_quality
    );

    let mut buf = vec![0u8; FRAME_SLOT];

    for fb in POOLS {
        for consumer_ms in CONSUMER_MS {
            arm(&mode, fb, consumer_ms, &mut buf);
        }
    }

    dump(&mode, &mut buf);

    println!("== DONE ==");
    loop {
        sleep(Duration::from_secs(1));
    }
}

/// One arm: `fb` driver buffers against a consumer that sleeps `consumer_ms`
/// after each grab.
fn arm(mode: &Mode, fb: u8, consumer_ms: u64, buf: &mut [u8]) {
    let mut cam = match IdfCamera::init(&XIAO_ESP32S3_SENSE, mode, fb) {
        Ok(c) => c,
        Err(e) => {
            println!("CAM fb_count={fb} consumer_ms={consumer_ms} init_failed={e:?}");
            return;
        }
    };

    // Untimed: the sensor's automatic exposure and gain move for the first
    // frames after a start, and those frames are neither typical nor the
    // thing under measurement.
    for _ in 0..3 {
        let _ = cam.grab(buf);
    }

    let before = cam.stats();
    let start = Instant::now();
    while start.elapsed().as_secs() < MEASURE_SECS {
        let _ = cam.grab(buf);
        if consumer_ms > 0 {
            sleep(Duration::from_millis(consumer_ms));
        }
    }
    let elapsed = start.elapsed();
    let after = cam.stats();

    let frames = after.frames - before.frames;
    let bytes = after.bytes - before.bytes;
    let no_frame = after.no_frame - before.no_frame;
    let empty = after.empty_frames - before.empty_frames;
    let too_small = after.too_small - before.too_small;
    let us = elapsed.as_micros() as u64;

    // Work first: the counts are exact. The rate is derived from them and
    // one clock, and is the confirmation.
    println!(
        "CAM fb_count={fb} consumer_ms={consumer_ms} frames={frames} bytes={bytes} no_frame={no_frame} empty_frames={empty} too_small={too_small}"
    );
    println!(
        "CAM fb_count={fb} consumer_ms={consumer_ms} us={us} fps={:.3} mean_jpeg={} attempts={}",
        frames as f64 * 1e6 / us.max(1) as f64,
        if frames > 0 { bytes / frames } else { 0 },
        frames + no_frame + empty + too_small
    );
}

/// Capture a few frames and put them on the wire as hex, so an outside tool
/// decides whether they are real JPEGs of the right size. The board saying
/// "I captured 150 frames" is a self-metric; ffprobe reading the bytes back
/// is the oracle.
fn dump(mode: &Mode, buf: &mut [u8]) {
    let mut cam = match IdfCamera::init(&XIAO_ESP32S3_SENSE, mode, 2) {
        Ok(c) => c,
        Err(e) => {
            println!("CAM dump init_failed={e:?}");
            return;
        }
    };
    // Let the exposure settle so the dumped frames are a picture of the room
    // rather than of the sensor waking up.
    for _ in 0..10 {
        let _ = cam.grab(buf);
    }
    for index in 0..DUMP_FRAMES {
        let frame = match cam.grab(buf) {
            Ok(f) => f,
            Err(e) => {
                println!("JPEG index={index} grab_failed={e:?}");
                continue;
            }
        };
        let Planes::Packed(data) = frame.planes else {
            println!("JPEG index={index} unexpected_planes");
            continue;
        };
        println!(
            "JPEG begin index={index} bytes={} sequence={} width={} height={}",
            data.len(),
            frame.sequence,
            frame.geometry.width,
            frame.geometry.height
        );
        dump_hex(data);
        println!("JPEG end index={index}");
    }
}

/// Emit `bytes` as lines of hex, 64 bytes each, prefixed so a monitor can
/// pick them out of the log.
fn dump_hex(bytes: &[u8]) {
    const PER_LINE: usize = 64;
    let mut line = String::with_capacity(PER_LINE * 2);
    for chunk in bytes.chunks(PER_LINE) {
        line.clear();
        for b in chunk {
            let _ = write!(line, "{b:02x}");
        }
        println!("JPEGDATA {line}");
    }
}
