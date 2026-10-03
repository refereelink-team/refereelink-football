# Software validation

## Baseline, 2026-10-02

The baseline validation below ran via SSH on **a remote Linux CUDA host**. No local Python backend was started. Temporary remote directories were `/tmp/refereelink-telemetry-test` and `/tmp/refereelink-football-test`.

| Check | Observed result | Boundary |
| --- | --- | --- |
| Showcase telemetry pytest | 15 passed | Frame parsing, 2D solve/rejection, freshness, bounded history/WS queue, idle/simulation separation and API lifecycle |
| Actual pyserial through Linux PTY | Included in 15 passed | Separate UWB and IMU reader paths, fragmented reads, peer disconnect/error, stale latest values; no physical device |
| Strict C11 build | Passed with `-Wall -Wextra -Werror -pedantic` | Core parser, HAL adapter against explicit stub, application integration example, binary bridge |
| Core C assertion suite | Passed | Official raw IMU sample, signed little-endian units, bad checksum/boundaries/ranges/tags, two channels, queue overflow recovery and 200,000 deterministic noise bytes |
| HAL stub assertion suite | Passed | ISR deferral, receive rearm, unrelated UART, deferred abort/restart, partial-frame reset and retry flags |
| Bridge black-box Python tests | 4 passed | Byte-preserving output, diagnostics on stderr, invalid packets withheld, channel isolation |
| AddressSanitizer + UndefinedBehaviorSanitizer | Both core/HAL suites passed | No reported host memory/undefined-behavior findings in those exercised cases |

Commands used on the remote host:

```sh
cd /tmp/refereelink-telemetry-test
PYTHONPATH=server /path/to/cuda-venv/bin/python -m pytest -q tests/test_telemetry_parsers.py tests/test_telemetry_service.py

make -C /tmp/refereelink-football-test test PYTHON=/path/to/cuda-venv/bin/python
```

For sanitizer checks, both C suites were compiled separately with `-std=c11 -g -O1 -fsanitize=address,undefined -fno-omit-frame-pointer` and executed successfully. The HAL suite defines `RF_HAL_HEADER="fake_hal.h"` and adds the test include directory; it **does not link or exercise real STM32 peripherals**.

The Python run emits one environment warning: installed Starlette reports its `httpx` TestClient integration as deprecated. Tests still passed; no dependency migration was performed as part of this hardware scope. pySerial 3.5 was installed into the existing remote virtual environment for the PTY reader checks.

## Complete IMU protocol follow-up, 2026-10-03

These additional checks ran on **local macOS** with its host C compiler and the Codex bundled Python runtime. They exercised the portable C gateway and an isolated host decoder; no Python backend service or model inference was started. The vendor protocol workbook was read without modification and remains outside Git.

| Check | Observed result | Boundary |
| --- | --- | --- |
| Strict C11 build and assertion suites | Passed with `-Wall -Wextra -Werror -pedantic` | Original raw/UWB behavior, four typed IMU profiles, HAL application example and explicit HAL stub |
| Bridge black-box tests | 9 passed | Byte-identical mixed raw/quaternion/Euler/barometer output, control frames skipped, non-finite values withheld, unknown payload isolation and malformed-length recovery |
| Known-length recovery matrix | Passed | Six known data/control functions, all 63 incorrect lengths in the 5-68 range per function, whole and fragmented input, immediate valid-frame recovery without padding |
| Float-value matrix | Passed | Every field in the three float profiles rejects quiet/signaling NaN and positive/negative infinity; finite fixtures preserve signed zero, negative values and finite extremes |
| Raw unit conversion | Passed | Original manufacturer sample, signed acceleration/gyro extremes, magnetic counts; inactive fields remain zero |
| IMU HAL lifecycle | Passed against stub | ISR deferral, interrupted IMU frame reset, Euler/quaternion recovery and UWB independence during an IMU error |
| AddressSanitizer and UndefinedBehaviorSanitizer | Both core/HAL suites passed | No reported findings in the exercised host cases |
| Existing showcase decoder compatibility | Passed in isolation | Mixed stream emitted exactly the two supplied raw acceleration samples and consumed the additional profiles without rejection; no API, UI or hardware acceptance claimed |
| Whitespace check | `git diff --check` passed | Source and documentation changes |

New attitude/barometer fixtures are synthetic byte sequences constructed independently from the documented layouts, not sensor recordings. Before the fixes, regression checks reproduced an empty bridge output after an in-range corrupted raw length and the loss of all three additional automatic telemetry profiles. The same checks pass after the update.

Commands used from this repository:

```sh
make test PYTHON=/Users/caysonyin/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3

cc -std=c11 -g -O1 -Wall -Wextra -Werror -pedantic \
  -fsanitize=address,undefined -fno-omit-frame-pointer -Iinclude \
  src/rf_telemetry.c tests/test_telemetry.c -lm \
  -o /tmp/refereelink-imu-docs/test_telemetry_sanitized
/tmp/refereelink-imu-docs/test_telemetry_sanitized

cc -std=c11 -g -O1 -Wall -Wextra -Werror -pedantic \
  -fsanitize=address,undefined -fno-omit-frame-pointer -Iinclude -Itests \
  '-DRF_HAL_HEADER="fake_hal.h"' \
  src/rf_telemetry.c src/rf_hal_adapter.c tests/test_hal_adapter.c \
  -o /tmp/refereelink-imu-docs/test_hal_sanitized
/tmp/refereelink-imu-docs/test_hal_sanitized
```

On this host, `sizeof(rf_imu_sample)` grew from 20 to 96 bytes and `sizeof(rf_sample)` from 24 to 100 bytes. Existing raw member offsets and values are preserved, but this is a C ABI change: every consumer must rebuild and check the sample kind before interpreting its fields. No new real-board build, firmware flash, remote deployment or hosted CI run was performed for this update.

## Remaining hardware acceptance

Not validated: board model, pinout, STM32 cross-compilation/linking, actual interrupt or DMA timing, firmware flashing, physical wiring/voltage, vendor RF ranging/accuracy, sensor installation axes, live full-field football motion or simultaneous camera+hardware timing. The deliverable is a portable software layer and the previously tested showcase API, with actual board integration explicitly pending arrival.
