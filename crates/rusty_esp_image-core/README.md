# rusty_esp_image-core

The pure `no_std + alloc` core of [`rusty_esp_image`](https://crates.io/crates/rusty_esp_image):
the `ImageSource` seam, a caller-owned `FramePool`, a JPEG header probe, pixel
kernels with scalar oracles, sensor descriptors and register tables as data,
and SCCB register access over `embedded-hal` 1.0. `forbid(unsafe)`. No
drivers; the capture engines live in `rusty_esp_image-esp`.

Feature ladder: `std` ⊃ `alloc` ⊃ core-only. Nothing here needs a heap.

The OV2640 and OV5640 tables are derived mechanically from esp32-camera
(Apache-2.0); see `LICENSE-THIRD-PARTY.md` in the repository.

Part of Janus (Remade With Rust). Plan: `docs/plans/rusty_esp_image.md` in the repo.
