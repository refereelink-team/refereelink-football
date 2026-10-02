# UWB / IMU UART evidence and implementation boundary

Verified against public primary sources on 2026-10-02, with an IMU follow-up on 2026-10-03. These are protocol/software checks, not physical-device acceptance. Vendor application code, screenshots, firmware and PDFs are not redistributed in this repository.

## BP-TWR-30 three-anchor positioning

Sources: [requested manual](https://doc.51uwb.cn/user_manual/twr-30/twr-30/) and [official host manual, §10.2](https://doc.51uwb.cn/tools/Landian_UWB_TWR_Host_USER_MANUAL/#102-mr). The second source resolves distance units and byte order absent from the first page's data table.

| Offset | Contents |
| --- | --- |
| 0–2 | ASCII `mr`, binary version `02` |
| 3 | tag ID (one byte; supplied default tag 05) |
| 4–5 | unsigned LE16 sequence |
| 6–7, 8–9, 10–11 | unsigned LE16 ranges, centimetres, anchor addresses 0001/0002/0003 |
| 12–13 | copy of the first range in the three-anchor profile |
| 14–15 | CRLF; the host manual also accepts LFCR |

UART: 115200 baud. There is **no checksum field**. Frame boundary checks cannot detect every bit flip; the parser also rejects missing/repeated range mismatches and ranges outside 0.2–30 m, then the service rejects wrong tags, repeated/reversed sequences, inconsistent triangle geometry and excessive residuals. The raw fourth range never becomes a fourth anchor. Reported rejection counts are software diagnostics, not radio RSSI or vendor accuracy estimates.

Three anchors and the tag must share one physical plane. Coordinates are configured in metres in that local plane, with IDs corresponding to UART range order. The common `plane_height_m` is installation metadata; reported `z_m` is that configuration, **not measured altitude**. Noncoplanar or nearly collinear layouts are refused. `x_m/y_m` result from three range circles; the residual is the maximum range-model mismatch. A position outside the anchor triangle or with a residual above 0.2 m is marked `degraded`; a residual above the configured maximum (default 0.5 m) is rejected. This validates tag tracking in a small controlled area, not full-field football ground truth.

## Current Yahboom IMU-Sensor, not the older 55 protocol

The requested [study page](https://www.yahboom.com/study/IMU_Sensor/) is the current 6/9/10-axis module. Its [official repository](https://github.com/YahboomTechnology/IMU-Sensor) was inspected at `61eda112c249cab6947d1ac1313a26749aa9f593`.

Primary evidence:

- Full `Communication Protocol.xlsx`, downloaded on 2026-10-03 from the [official communication-protocol folder](https://drive.google.com/drive/folders/17CkhcHhOCH9c50Tb1vVkmqqZZD2DKsU0) linked by the repository's `Annex_Download_Link.txt`. SHA-256: `c46f433662abfa7a85191d263e8f09d15d3e4e985ebb2e54a3273f60186fee51`. Both serial and I2C worksheets were inspected; the workbook and extraction code are not redistributed here.
- [Official STM32 serial tutorial PDF](https://github.com/YahboomTechnology/IMU-Sensor/blob/61eda112c249cab6947d1ac1313a26749aa9f593/2.%20Multi-master%20communication%20case/2.%20Serial%20communication/1.STM32.pdf) shows total-frame length, sum8 calculation, first three signed acceleration values and scaling `16/32767` in g.
- [Public PC communication tutorial](https://www.yahboom.com/build.html?id=15173&cid=725) and its [raw serial screenshot](https://admin.yahboom.com/public/upload/upload-html/1761275864/pc.png) establish actual header, function byte, little-endian order and a checksum-consistent sample.
- [Current module parameter diagram](https://admin.yahboom.com/public/upload/upload-html/1761275770/image-20251016174439021.png) documents 115200 baud, a default output rate of 25 Hz, and an adjustable rate of 10–100 Hz. The PC screenshot confirms 8N1.

Transcribed sample (23 bytes):

```text
7E 23 17 04 51 00 99 FF F7 07 F5 FF 00 00 00 00 1B 01 40 FA 5D FD 47
```

| Offset | Contents |
| --- | --- |
| 0–1 | `7E 23` |
| 2 | total length (raw sample `17` hex = 23 bytes) |
| 3 | raw acceleration/gyro/magnetic function `04` |
| 4–9 | signed LE16 acceleration X/Y/Z |
| 10–15 | signed LE16 angular-velocity X/Y/Z |
| 16–21 | signed LE16 magnetometer X/Y/Z |
| 22 | sum of all preceding bytes, modulo 256 (`47` in the sample) |

Sample acceleration XYZ counts are `81, -103, 2039`. Acceleration is `count × 16/32767 × 9.80665 m/s²`; it includes gravity. Angular velocity is `count × 2000/32767 × π/180 rad/s`. These scale factors are confirmed in serial worksheet cell `Y52` and STM32 tutorial p. 3. The parser also exposes signed magnetometer counts. The workbook lists a magnetic multiplier of `800/32767` without identifying its physical unit, so no magnetic value is labelled as microteslas.

### Automatic serial telemetry

The complete protocol resolves the additional output profiles that the original acceleration-only bridge discarded:

| Function | Total bytes | Payload, starting at offset 4 | Serial worksheet evidence |
| --- | ---: | --- | --- |
| `04` | 23 | Nine signed LE16 values: acceleration, gyro, magnetic X/Y/Z | Rows 52-54 |
| `16` | 21 | Four LE float32 values: quaternion `w, x, y, z` | Rows 56-58; note `W56` |
| `26` | 17 | Three LE float32 values: roll, pitch, yaw in radians | Rows 60-62; note `S60` |
| `32` | 21 | Four LE float32 values: height relative to startup in m, temperature in degrees Celsius, pressure in Pa, startup reference pressure in Pa | Rows 64-66; note `W64` |

The I2C worksheet explicitly identifies the attitude and barometer registers as `float`; the STM32 tutorial reads the same values with `to_float`. The C decoder supports 32-bit IEEE 754 binary32 hosts, checks floating-point dimensions at compile time and verifies native integer/float bit layout using `1.0f` and `-0.0f` before gateway initialization or float decoding. It assembles little-endian bits without pointer casts and rejects NaN and infinity bit patterns before float interpretation. It preserves finite values without normalizing quaternions, converting angles to degrees or imposing undocumented physical bounds.

All four supported telemetry profiles are validated, decoded into distinct sample kinds, and forwarded byte-for-byte through the existing callback. Each supported function's fixed length is checked as soon as its four-byte header is available. A corrupt in-range length cannot hold the next complete supported frame waiting for extra bytes. Length and checksum errors recover by scanning one byte at a time. Unknown checksum-valid frames and firmware/calibration replies are consumed as complete packets without becoming telemetry or exposing any embedded header as a sample. Buffering remains bounded at 68 bytes; the largest listed telemetry packet is 23 bytes, and the buffer is not a promise to support future vendor extensions.

The protocol lists automatic return at 25 Hz by default, adjustable to 10-100 Hz (serial `P50`, `P37`). Passive reception needs no new initialization command. The adapter therefore keeps the existing 115200 baud/8N1 integration contract and sends no configuration, calibration or reset writes. The protocol's temperature-calibration row has inconsistent length/index values (`D30` versus `I31`), and the reset description appears copied from algorithm switching (`P45`); neither ambiguous command is implemented. I2C is a separate transport and is outside this UART gateway.

Six-axis modules have no magnetometer/barometer data; nine-axis modules have no barometer data (STM32 tutorial p. 4). Barometer output is explicitly ten-axis only (`W64`). The raw profile retains the documented 23-byte layout; a shorter six-axis layout is not inferred. A field's presence or zero value does not establish that its sensor exists on the attached module.

The enlarged C sample structure requires all consumers to rebuild and check the IMU sample kind before interpreting its fields. The current showcase host parser still emits acceleration only and safely consumes the additional checksum-valid profiles. This football update preserves their transport and C decoding; it does not add their presentation to the showcase UI.

Installation orientation, sensor-to-ball/field rotation, sensor timestamps and cross-device synchronization are not supplied by this protocol. Acceleration is not rotated into field axes or integrated into ball position. Euler/quaternion values are vendor sensor attitude, and barometer height is relative to startup rather than UWB altitude or ball ground truth.

The older Yahboom 10-axis `55 51` frame is a different product profile and is deliberately unsupported here.

## State, transport and hardware acceptance

`idle`, `simulation`, `serial` are distinct `source_mode` values. Simulation is explicit and passes generated UART frames through the same parsers. It never marks hardware connected. Serial opening marks a port `connected`; freshness separately depends on accepted samples, not arbitrary received bytes. A peer unplug becomes `error`, last error remains visible, and old history remains with latest sample marked stale/invalid. No HTTP endpoint accepts arbitrary telemetry injection.

[pySerial's API](https://pyserial.readthedocs.io/en/latest/pyserial_api.html) supports the exclusive POSIX port opens used here. Linux `/dev/ttyUSB*`, `/dev/ttyACM*`, `/dev/ttyS*`, `/dev/ttyAMA*` and `/dev/serial/by-id/*` character devices are allowed; URLs and ordinary files are refused. `/dev/pts/*` is constructor-enabled only in tests. The public API cannot enable it. Each device uses its own port, 8N1, 100 ms read timeout, no writes or auto-reconnect. Physical cable placement, UART voltage compatibility, USB identity, anchors/antenna delays and RF accuracy still require device arrival and measurement.
