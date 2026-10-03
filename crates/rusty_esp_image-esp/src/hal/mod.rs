//! Track B backends: the DVP capture engine over esp-hal's `lcd_cam`
//! camera and GDMA — what esp32-camera's `cam_hal.c` and `ll_cam.c` do in
//! C — behind the core's `ImageSource`.
//!
//! The sensor itself is driven by the core's [`rusty_esp_image_core::driver`]
//! over esp-hal's I²C (the SCCB bus), and its master clock comes out of
//! the camera peripheral (`with_master_clock`), so no LEDC and no C.
//!
//! What lands where: [`dvp::DvpCamera`] takes a configured `Camera` and
//! two DMA receive buffers and hands out whole frames; the frame pool a
//! firmware keeps in PSRAM is the core's `FramePool` over a buffer the
//! firmware allocates there.

pub mod dvp;

pub use dvp::{CaptureStats, DvpCamera};
