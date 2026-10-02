# RefereeLink football telemetry gateway

Original C11 software for the **BP-TWR-30 three-anchor/one-tag** UART output and the current **Yahboom IMU-Sensor 6/9/10-axis** UART output. It decodes distances and sensor X/Y/Z acceleration, rejects malformed frames and passes complete accepted manufacturer frames to a board-selected transport callback. No vendor firmware or SDK is bundled.

**Hardware has not arrived and has not been tested.** This repository is a portable integration layer, not a ready-to-flash project for an unidentified board. MCU model, physical pins, voltage levels, UART instances, clocks, interrupt priorities and host transport are supplied by the actual Cube/HAL project. The matching showcase backend computes same-plane 2D tag position and shows acceleration/history; it never claims full-pitch football ground-truth accuracy or measured altitude.

The current supported backend integration uses **two distinct host UART devices**, one per stream. Direct USB/CH340 adapters are supported by the showcase. The MCU bridge's output callback preserves each channel's frames separately; routing two channels over one multiplexed USB interface is outside this implementation.

Files:

- `include/rf_telemetry.h`, `src/rf_telemetry.c`: vendor parsers, unit conversion and two bounded ISR rings.
- `include/rf_hal_adapter.h`, `src/rf_hal_adapter.c`: HAL handle injection, receive-complete rearming and deferred error recovery.
- `examples/hal_application.c`: application wiring without assigning pins or replacing existing HAL callbacks.
- `examples/serial_bridge.c`: functioning host stdin/stdout bridge for one selected manufacturer stream.
- `tests/`: portable parser/overflow tests, a clearly identified HAL stub and black-box bridge tests.
- [HAL integration](docs/HAL_INTEGRATION.md), [protocol evidence](docs/PROTOCOLS.md), [software validation](docs/VALIDATION.md).

Software checks on a remote Linux host (C11 compiler, make and Python 3.12+):

```sh
make test PYTHON=/path/to/python
```

`make` builds `build/serial_bridge`. For a saved capture:

```sh
build/serial_bridge imu < imu_capture.bin > imu_forwarded.bin
build/serial_bridge uwb < uwb_capture.bin > uwb_forwarded.bin
```

Only accepted raw frames go to stdout; counts go to stderr. Replayed/synthetic captures are software evidence, not connected hardware. The source uses no heap allocation. Each ISR ring stores at most 511 bytes; on overflow the consumer discards the pending ring and partial parser state to prevent splicing pre-loss bytes into a new frame. Overruns and discarded byte counts remain visible. `rf_gateway_process()` bounds consumed bytes per channel; the application must also bound its output callback duration.

License: MIT for the original code in this repository. Primary sources are linked for protocol attribution; vendor sources retain their own terms and are not copied here.

Contribution and test instructions: [CONTRIBUTING.md](CONTRIBUTING.md). Security reports and integration boundaries: [SECURITY.md](SECURITY.md). CI runs the software suites and sanitizer checks; it does not flash or validate hardware.
