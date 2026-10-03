# Third-party material in rusty_esp_image

## Sensor register tables (Apache-2.0)

`crates/rusty_esp_image-core/src/sensor/ov2640.rs` and
`crates/rusty_esp_image-core/src/sensor/ov5640.rs` are derived mechanically
from the following files of **espressif/esp32-camera**
(https://github.com/espressif/esp32-camera), commit `202df95d7b1d`
(2026-06-05), licensed under the Apache License, Version 2.0:

- `sensors/private_include/ov2640_settings.h`
- `sensors/private_include/ov2640_regs.h` (macro values only)
- `sensors/private_include/ov5640_settings.h`
- `sensors/private_include/ov5640_regs.h` (macro values only)

Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD.

The conversion resolved macro names to their numeric values, dropped the
list terminators (`{0xff, 0xff}` for OV2640, `REGLIST_TAIL` for OV5640),
turned `REG_DLY` entries into explicit delay steps, and changed nothing else.
The product ids in `sensor/mod.rs` come from `driver/include/sensor.h` and the
frame-size table from `driver/sensor.c` of the same commit; the SCCB bus
addresses from the `camera_sccb_addr_t` enum in `driver/include/sensor.h`.

## OV3660 tables and the two driver ports (Apache-2.0)

`crates/rusty_esp_image-core/src/sensor/ov3660.rs` is derived the same way
(by `tools/convert-sensor-tables.py`) from `sensors/private_include/
ov3660_settings.h` and `ov3660_regs.h` of the same project at release
2.1.7 (the ESP-IDF managed component `espressif/esp32-camera`), and
`crates/rusty_esp_image-core/src/driver/ov3660.rs` and `driver/ov2640.rs`
are ports of the register logic of `sensors/ov3660.c` and `sensors/ov2640.c`
of that release (reset and configuration order, the window and clock
arithmetic, the register masks for quality, format and the test pattern),
rewritten in Rust as a state machine over a bus trait; the register names
in them are the C headers'. Same licence, same copyright holder.

The Apache License, Version 2.0 is compatible with this repository's
MIT OR Apache-2.0 licence. A copy is in `LICENSE-APACHE`. The NOTICE
requirement of §4(d) is satisfied by this file.
