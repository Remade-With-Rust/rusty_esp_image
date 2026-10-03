//! The DVP capture engine: esp-hal's LCD_CAM camera streaming through GDMA
//! into one ring, frames cut at the JPEG markers.
//!
//! This is the shape of esp32-camera's `cam_hal.c` for a JPEG sensor, and
//! for a reason found on the bench: the OV3660's DVP VSYNC in JPEG mode
//! does not mark a picture, it marks an output window of a fixed number of
//! pixel clocks (2,300 × 783 at 10 MHz: 180 ms) in which the sensor
//! delivers whatever pictures it made meanwhile, back to back, each padded
//! to a 6 KB block. An engine that took one DMA transfer per VSYNC got one
//! picture per window — 5.6 fps from a sensor making 27.8. So the camera
//! is started once and never stopped; the DMA fills a ring;
//! [`DvpCamera::grab`] walks the ring for the next start-of-image, copies
//! bytes until the image's end-of-image (checked as a real one with the
//! core's segment-aware `find_first_eoi`, since `FF D9` can occur inside a
//! marker segment), and consumes what it read. Nothing is lost between
//! pictures because nothing stops.
//!
//! Two things about the ring. esp-hal's `Camera::receive` sets the
//! peripheral's stop-on-EOF bit, which would end the stream at the first
//! end-of-frame the peripheral raises; the bit lives in the LCD_CAM
//! register block the HAL does not expose, so this engine clears it through
//! the chip's register map after every start — the one register write in
//! this crate, and it is named here. And a ring the consumer falls behind
//! on fills up, at which point the DMA runs out of descriptors and stops
//! (esp-hal says so); the engine notices the silence — the sensor delivers
//! bytes every 36 ms at this geometry — and restarts the stream, counting
//! the restart, as the microphone's engine does (X1). A restart empties the
//! ring: one or two pictures are lost, and `restarts` says how often.
//!
//! Only JPEG is handled. A picture larger than the caller's slot is
//! counted and skipped to the next start-of-image.

use esp_hal::dma::DmaRxStreamBuf;
use esp_hal::lcd_cam::cam::{Camera, CameraTransfer};
use esp_hal::time::Instant;
use rusty_esp_image_core::esp_core::error::{Error, Result};
use rusty_esp_image_core::esp_core::frame::{Frame, Geometry, PixelFormat};
use rusty_esp_image_core::esp_core::time::Micros;
use rusty_esp_image_core::cut::{CutCounts, Cutter};
use rusty_esp_image_core::source::ImageSource;

/// Silence that means the stream has stopped, not that a picture is late.
/// The sensor delivers bytes every frame, 36 ms at QVGA, but a stream
/// just started waits for the next output window, up to 180 ms; this sits
/// above that, so a start is never taken for a stall.
const STALL_MS: u64 = 250;

/// Clear the stop-on-EOF bit esp-hal's `receive` sets, and apply it: the
/// camera then streams until told to stop, as esp32-camera leaves it.
#[allow(unsafe_code)]
fn keep_streaming() {
    // SAFETY: the LCD_CAM register block the HAL owns; one bit of the
    // control register the HAL wrote a moment ago, and the update bit.
    let regs = unsafe { &*esp32s3::LCD_CAM::PTR };
    regs.cam_ctrl().modify(|_, w| w.cam_stop_en().clear_bit());
    regs.cam_ctrl().modify(|_, w| w.cam_update().set_bit());
}

/// What the engine did, counted.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct CaptureStats {
    /// Frames handed to the caller.
    pub frames: u64,
    /// Bytes handed over across those frames (SOI to EOI).
    pub bytes: u64,
    /// Bytes skipped between pictures: the sensor's padding, and the tail
    /// of whatever was streaming when the engine started.
    pub skipped: u64,
    /// Pictures that did not fit the caller's slot and were skipped.
    pub too_small: u64,
    /// Grabs that saw no bytes within the timeout: the sensor is not
    /// streaming, or the peripheral stopped.
    pub timeouts: u64,
    /// `FF D9` sequences that were not a picture's end (inside a segment).
    pub false_eoi: u64,
    /// Streams restarted after a silence: the ring had filled and the DMA
    /// stopped, or the sensor stopped.
    pub restarts: u64,
}

/// Bytes one picture is assumed to hold before the first grab has measured
/// one: one of the OV3660's 6,144-byte blocks (a QVGA picture is ~4.4 KB).
const FIRST_SPAN: usize = 6144;

/// A camera streaming into a ring, one picture at a time out of it.
pub struct DvpCamera<'d> {
    running: Option<CameraTransfer<'d, DmaRxStreamBuf>>,
    idle: Option<(Camera<'d>, DmaRxStreamBuf)>,
    geometry: Geometry,
    sequence: u32,
    stats: CaptureStats,
    timeout_ms: u64,
    /// The last picture's bytes: what the next one is assumed to need,
    /// with one block of padding, before it is whole.
    last_span: usize,
    /// What `available_bytes` last saw, and when it last saw more: a ring
    /// that stops growing has a stopped DMA behind it.
    last_available: usize,
    quiet_since: Instant,
}

impl core::fmt::Debug for DvpCamera<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("DvpCamera")
            .field("geometry", &self.geometry)
            .field("stats", &self.stats)
            .finish_non_exhaustive()
    }
}

impl<'d> DvpCamera<'d> {
    /// A configured camera (pins, clock and config applied) and the ring it
    /// streams into; the stream is started here. Only JPEG geometries are
    /// accepted today.
    pub fn new(camera: Camera<'d>, ring: DmaRxStreamBuf, geometry: Geometry) -> Result<Self> {
        if geometry.format != PixelFormat::Jpeg {
            return Err(Error::Unsupported);
        }
        let mut engine = DvpCamera {
            running: None,
            idle: Some((camera, ring)),
            geometry,
            sequence: 0,
            stats: CaptureStats::default(),
            timeout_ms: 500,
            last_span: FIRST_SPAN,
            last_available: 0,
            quiet_since: Instant::now(),
        };
        engine.start()?;
        Ok(engine)
    }

    /// The engine's counters.
    #[must_use]
    pub fn stats(&self) -> CaptureStats {
        self.stats
    }

    /// How long one grab may wait for bytes (default 500 ms).
    pub fn set_timeout_ms(&mut self, ms: u64) {
        self.timeout_ms = ms;
    }

    /// Start the stream if it is not running, and keep it streaming.
    fn start(&mut self) -> Result<()> {
        if self.running.is_some() {
            return Ok(());
        }
        let (camera, ring) = self.idle.take().ok_or(Error::Busy)?;
        match camera.receive(ring) {
            Ok(transfer) => {
                self.running = Some(transfer);
                keep_streaming();
                Ok(())
            }
            Err((_, camera, ring)) => {
                self.idle = Some((camera, ring));
                Err(Error::Hardware)
            }
        }
    }

    /// Stop and start again: the ring is emptied.
    fn restart(&mut self) -> Result<()> {
        if let Some(transfer) = self.running.take() {
            self.idle = Some(transfer.stop());
        }
        self.stats.restarts += 1;
        self.start()
    }

    /// Whether bytes are waiting in the ring — a caller that would rather
    /// yield than spin polls this, sleeps a little, and `grab`s when it says
    /// so; `grab` then waits only for the rest of the picture.
    pub fn frame_ready(&mut self) -> bool {
        self.available_bytes() > 0
    }

    /// Whether a whole picture is probably in the ring, so that `grab`
    /// returns without waiting: the ring holds at least the last picture's
    /// bytes and one more of the sensor's blocks (the padding before the
    /// next picture is at most one). A heuristic, not a scan — the ring is not searched on every
    /// poll — and never wrong in the way that matters: `grab` is correct
    /// either way, it only blocks for the rest of a picture still arriving.
    /// A task that shares its core polls this and sleeps between polls;
    /// polling `frame_ready` instead put that task in `grab` for most of
    /// every frame period, spinning on the DMA while the next picture came
    /// in, and the page it shared the core with ran at a quarter of its rate
    /// (the X6 porch-cam twin's second board run).
    pub fn picture_ready(&mut self) -> bool {
        self.available_bytes() >= self.last_span + 6144
    }

    /// What `picture_ready` waits for: the ring bytes the last picture
    /// occupied plus one block.
    #[must_use]
    pub fn ready_threshold(&self) -> usize {
        self.last_span + 6144
    }

    /// Bytes waiting in the ring.
    ///
    /// A ring that has not grown for longer than any gap the sensor leaves
    /// has a stopped DMA behind it (it filled while nobody read it, which a
    /// start-up does), and a poller that only ever asks this question would
    /// wait for ever — only a `grab` restarted it until the X6 porch-cam
    /// twin's third board run found a camera task asleep on `picture_ready`
    /// with a stopped ring. So the stall is noticed here too, and the
    /// stream restarted and counted, as `grab` does.
    pub fn available_bytes(&mut self) -> usize {
        if self.start().is_err() {
            return 0;
        }
        let available = self.running.as_mut().map_or(0, |t| t.available_bytes());
        if available > self.last_available {
            self.quiet_since = Instant::now();
        } else if self.quiet_since.elapsed().as_millis() > STALL_MS {
            let _ = self.restart();
            self.quiet_since = Instant::now();
            self.last_available = 0;
            return 0;
        }
        self.last_available = available;
        available
    }

    /// Stop the stream and give the camera and the ring back.
    pub fn stop(mut self) -> Option<(Camera<'d>, DmaRxStreamBuf)> {
        match (self.running.take(), self.idle.take()) {
            (Some(transfer), _) => Some(transfer.stop()),
            (None, Some(idle)) => Some(idle),
            (None, None) => None,
        }
    }
}

impl ImageSource for DvpCamera<'_> {
    fn geometry(&self) -> Geometry {
        self.geometry
    }

    fn grab<'b>(&mut self, out: &'b mut [u8]) -> Result<Frame<'b>> {
        self.start()?;
        let timeout_ms = self.timeout_ms;
        let started = Instant::now();
        let mut quiet_since = Instant::now();
        let mut restarted = false;
        let mut cutter = Cutter::new();
        loop {
            let transfer = self.running.as_mut().ok_or(Error::Busy)?;
            let available = transfer.peek();
            if available.is_empty() {
                if started.elapsed().as_millis() > timeout_ms {
                    self.stats.timeouts += 1;
                    return Err(Error::Timeout);
                }
                // silence longer than any gap the sensor leaves: the ring
                // filled and the DMA stopped, so start over — once per grab
                if !restarted && quiet_since.elapsed().as_millis() > STALL_MS {
                    self.restart()?;
                    restarted = true;
                    cutter.reset();
                    quiet_since = Instant::now();
                }
                continue;
            }
            quiet_since = Instant::now();
            // the picture cut out of the ring: rusty_esp_image-core's `Cutter`
            let mut counts = CutCounts::default();
            let (consumed, done) = cutter.feed(available, out, &mut counts);
            // the counters as bytes go by, as the per-byte loop kept them
            self.stats.skipped += counts.skipped;
            self.stats.too_small += counts.too_small;
            self.stats.false_eoi += counts.false_eoi;
            transfer.consume(consumed);
            self.last_available = self.last_available.saturating_sub(consumed);
            self.quiet_since = Instant::now();
            if let Some(n) = done {
                self.stats.frames += 1;
                self.stats.bytes += n as u64;
                // the picture's own bytes: the padding before the next one is
                // at most a block, which `ready_threshold` adds. (The whole
                // consumed span was used first; after a restart it held the
                // stale ring the grab skipped, 40 KB, and the threshold then
                // sat above what the ring ever reports -- the porch-cam twin
                // waited for ever, restarting every 600 ms.)
                self.last_span = n;
                let seq = self.sequence;
                self.sequence = self.sequence.wrapping_add(1);
                let micros = Micros(Instant::now().duration_since_epoch().as_micros());
                return Frame::packed(self.geometry, micros, seq, &out[..n]);
            }
        }
    }
}
