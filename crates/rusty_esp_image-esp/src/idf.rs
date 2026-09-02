//! Track A: `IdfCamera`, the esp32-camera component behind `ImageSource`.
//!
//! esp32-camera owns the DVP/I²S DMA engine, the frame buffers (in PSRAM) and
//! the sensor drivers; this module owns the Rust API and the frame contract.
//! Each captured JPEG is copied out of the driver's buffer into the caller's
//! slot and the driver buffer is returned at once, so the driver never waits
//! on the network. The copy is one `memcpy` per frame (about 15 KB at QVGA);
//! a zero-copy variant that holds the driver buffer until the frame is sent
//! is a later optimisation, measured before it is written.
//!
//! The firmware that links this must declare the component:
//!
//! ```toml
//! [[package.metadata.esp-idf-sys.extra_components]]
//! remote_component = { name = "espressif/esp32-camera", version = "2.0" }
//! bindings_header = "bindings.h"
//! bindings_module = "camera"
//! ```
//!
//! This module compiles only in such a firmware build (the `esp-idf` feature
//! on an `*-espidf` target); the host never sees it.

#![allow(unsafe_code)]

use core::ffi::c_int;

use esp_idf_sys::camera as sys;
use rusty_esp_image_core::core::error::{Error, Result};
use rusty_esp_image_core::core::frame::{Frame, Geometry, PixelFormat};
use rusty_esp_image_core::core::time::Micros;
use rusty_esp_image_core::sensor::{FrameSize, Mode};
use rusty_esp_image_core::source::ImageSource;

use crate::CameraPins;

/// A running esp32-camera instance.
#[derive(Debug)]
pub struct IdfCamera {
    geometry: Geometry,
    sequence: u32,
    /// Frames the driver returned with a length of zero or a null buffer.
    pub empty_frames: u64,
}

/// Map a JPEG geometry onto esp32-camera's `framesize_t`.
fn framesize_for(geometry: &Geometry) -> Result<sys::framesize_t> {
    let (w, h) = (geometry.width, geometry.height);
    let size = [
        (FrameSize::S96x96, sys::framesize_t_FRAMESIZE_96X96),
        (FrameSize::Qqvga, sys::framesize_t_FRAMESIZE_QQVGA),
        (FrameSize::S128x128, sys::framesize_t_FRAMESIZE_128X128),
        (FrameSize::Qcif, sys::framesize_t_FRAMESIZE_QCIF),
        (FrameSize::Hqvga, sys::framesize_t_FRAMESIZE_HQVGA),
        (FrameSize::S240x240, sys::framesize_t_FRAMESIZE_240X240),
        (FrameSize::Qvga, sys::framesize_t_FRAMESIZE_QVGA),
        (FrameSize::S320x320, sys::framesize_t_FRAMESIZE_320X320),
        (FrameSize::Cif, sys::framesize_t_FRAMESIZE_CIF),
        (FrameSize::Hvga, sys::framesize_t_FRAMESIZE_HVGA),
        (FrameSize::Vga, sys::framesize_t_FRAMESIZE_VGA),
        (FrameSize::Svga, sys::framesize_t_FRAMESIZE_SVGA),
        (FrameSize::Xga, sys::framesize_t_FRAMESIZE_XGA),
        (FrameSize::Hd, sys::framesize_t_FRAMESIZE_HD),
        (FrameSize::Sxga, sys::framesize_t_FRAMESIZE_SXGA),
        (FrameSize::Uxga, sys::framesize_t_FRAMESIZE_UXGA),
    ]
    .into_iter()
    .find(|(fs, _)| fs.dimensions() == (w, h))
    .map(|(_, v)| v)
    .ok_or(Error::Unsupported)?;
    Ok(size)
}

impl IdfCamera {
    /// Initialise the driver for `mode` (JPEG only in J1) with `fb_count`
    /// driver-side frame buffers in PSRAM and a 20 MHz XCLK.
    pub fn init(pins: &CameraPins, mode: &Mode, fb_count: u8) -> Result<Self> {
        if mode.geometry.format != PixelFormat::Jpeg {
            return Err(Error::Unsupported);
        }
        let frame_size = framesize_for(&mode.geometry)?;
        // SAFETY: `camera_config_t` is a plain-old-data C struct for which
        // all-zero bytes is a valid (if useless) value; every field the driver
        // reads is set below before the pointer is handed to `esp_camera_init`.
        let mut cfg: sys::camera_config_t = unsafe { core::mem::zeroed() };
        cfg.pin_pwdn = pins.pwdn as c_int;
        cfg.pin_reset = pins.reset as c_int;
        cfg.pin_xclk = pins.xclk as c_int;
        cfg.__bindgen_anon_1.pin_sccb_sda = pins.sccb_sda as c_int;
        cfg.__bindgen_anon_2.pin_sccb_scl = pins.sccb_scl as c_int;
        cfg.pin_d7 = pins.d[7] as c_int;
        cfg.pin_d6 = pins.d[6] as c_int;
        cfg.pin_d5 = pins.d[5] as c_int;
        cfg.pin_d4 = pins.d[4] as c_int;
        cfg.pin_d3 = pins.d[3] as c_int;
        cfg.pin_d2 = pins.d[2] as c_int;
        cfg.pin_d1 = pins.d[1] as c_int;
        cfg.pin_d0 = pins.d[0] as c_int;
        cfg.pin_vsync = pins.vsync as c_int;
        cfg.pin_href = pins.href as c_int;
        cfg.pin_pclk = pins.pclk as c_int;
        cfg.xclk_freq_hz = 20_000_000;
        cfg.ledc_timer = sys::ledc_timer_t_LEDC_TIMER_0;
        cfg.ledc_channel = sys::ledc_channel_t_LEDC_CHANNEL_0;
        cfg.pixel_format = sys::pixformat_t_PIXFORMAT_JPEG;
        cfg.frame_size = frame_size;
        cfg.jpeg_quality = c_int::from(mode.jpeg_quality);
        cfg.fb_count = usize::from(fb_count.max(1));
        cfg.fb_location = sys::camera_fb_location_t_CAMERA_FB_IN_PSRAM;
        cfg.grab_mode = sys::camera_grab_mode_t_CAMERA_GRAB_LATEST;
        // SAFETY: `cfg` is fully initialised and outlives the call; the driver
        // copies what it needs. Called once per process by contract of this type.
        let err = unsafe { sys::esp_camera_init(&cfg) };
        if err != esp_idf_sys::ESP_OK {
            return Err(Error::Hardware);
        }
        Ok(IdfCamera {
            geometry: mode.geometry,
            sequence: 0,
            empty_frames: 0,
        })
    }

    /// Set the sensor's vertical flip and horizontal mirror.
    pub fn set_orientation(&mut self, flip: bool, mirror: bool) -> Result<()> {
        // SAFETY: `esp_camera_sensor_get` returns the driver's static sensor
        // handle or null; the handle and its function pointers live for the
        // driver's lifetime, and we call them only when non-null.
        unsafe {
            let sensor = sys::esp_camera_sensor_get();
            if sensor.is_null() {
                return Err(Error::Hardware);
            }
            let s = &*sensor;
            if let Some(f) = s.set_vflip {
                f(sensor, c_int::from(flip));
            }
            if let Some(f) = s.set_hmirror {
                f(sensor, c_int::from(mirror));
            }
        }
        Ok(())
    }
}

impl Drop for IdfCamera {
    fn drop(&mut self) {
        // SAFETY: pairs with the successful `esp_camera_init` in `init`.
        unsafe {
            sys::esp_camera_deinit();
        }
    }
}

impl ImageSource for IdfCamera {
    fn geometry(&self) -> Geometry {
        self.geometry
    }

    fn grab<'b>(&mut self, out: &'b mut [u8]) -> Result<Frame<'b>> {
        // SAFETY: `esp_camera_fb_get` returns a driver-owned buffer or null;
        // we copy its `len` bytes and return it with `esp_camera_fb_return`
        // on every path before touching the driver again.
        let fb = unsafe { sys::esp_camera_fb_get() };
        if fb.is_null() {
            return Err(Error::Timeout);
        }
        // SAFETY: `fb` is non-null and valid until returned.
        let (len, ptr, ts) = unsafe { ((*fb).len, (*fb).buf, (*fb).timestamp) };
        if len == 0 || ptr.is_null() {
            self.empty_frames += 1;
            // SAFETY: returning the buffer we were handed.
            unsafe { sys::esp_camera_fb_return(fb) };
            return Err(Error::Hardware);
        }
        if out.len() < len {
            // SAFETY: as above.
            unsafe { sys::esp_camera_fb_return(fb) };
            return Err(Error::BufferTooSmall { needed: len });
        }
        // SAFETY: `ptr` points at `len` readable bytes owned by the driver
        // until `esp_camera_fb_return`; `out` has room for `len` bytes and the
        // two regions cannot overlap (driver memory vs caller memory).
        unsafe { core::ptr::copy_nonoverlapping(ptr, out.as_mut_ptr(), len) };
        // SAFETY: returning the buffer we were handed.
        unsafe { sys::esp_camera_fb_return(fb) };
        let micros = Micros((ts.tv_sec as u64) * 1_000_000 + (ts.tv_usec as u64));
        let seq = self.sequence;
        self.sequence = self.sequence.wrapping_add(1);
        Frame::packed(self.geometry, micros, seq, &out[..len])
    }
}
