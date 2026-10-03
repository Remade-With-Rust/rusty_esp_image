//! Sensor drivers: the state machines esp32-camera's C drivers encode,
//! over the register tables [`crate::sensor`] carries as data and any
//! `embedded-hal` I²C bus.
//!
//! Each driver is a faithful port of one C file — the same registers in
//! the same order, the same arithmetic for the computed ones — so that a
//! sensor programmed by either produces the same frames. That is the
//! oracle the killing-C plan's X5 row names: the sensor's own colour-bar
//! test pattern, JPEG-encoded by the sensor, byte-identical between the
//! two drivers at the same registers. Being pure, every write a driver
//! makes can be listed on the host: the tests pin the sequences for the
//! modes the boards use, derived by hand from the C.
//!
//! The board on the bench (a XIAO ESP32-S3 Sense) answered as an
//! [`Ov3660`]; the [`Ov2640`] is the ESP32-CAM's and the other part the
//! XIAO ships with, ported alongside and proved when one is on a bench.

pub mod ov2640;
pub mod ov3660;

pub use ov2640::Ov2640;
pub use ov3660::Ov3660;
