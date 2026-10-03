# Integrating the portable gateway with an actual STM32 board

The adapter uses APIs confirmed in [ST's STM32F1 UART HAL driver](https://github.com/STMicroelectronics/stm32f1xx-hal-driver/blob/master/Src/stm32f1xx_hal_uart.c): interrupt-driven reception completes in `HAL_UART_RxCpltCallback`; errors enter `HAL_UART_ErrorCallback`; `HAL_UART_AbortReceive` is used during main-loop recovery. This does not certify behavior on a board or establish which STM32 family the user's gateway will use.

## 1. Board configuration

Select the MCU and pins from the received board's schematic. Configure **two different receive UART handles**, with 115200 baud, 8 data bits, no parity, one stop bit. Enable their interrupts in the actual project and retain its existing IRQ handlers. Connect each sensor TX to the selected MCU RX with common ground after confirming UART logic-level compatibility. Do not infer power wiring from UART labels or apply a voltage based on this generic example.

UWB data comes from anchor address `0x0001`, not the tag UART. Manufacturer tag default is `0x0005`; `rf_default_config()` selects ID 5. Three anchor distances correspond in order to `0x0001/0x0002/0x0003`. Anchor positions, common plane height and 2D solution stay in the showcase backend. There is no UWB hardware/radio setup code in this repository.

## 2. Add original source files

Compile `src/rf_telemetry.c`, `src/rf_hal_adapter.c` and optionally `examples/hal_application.c`, with `include/` in the include path and C11 enabled. Set `RF_HAL_HEADER` to the actual family umbrella header. For an F1 Cube project, the compiler definition would be:

```text
RF_HAL_HEADER="stm32f1xx_hal.h"
```

This example **does not select an MCU or pin assignment**. Supply the project's own CMSIS/HAL files; they are not bundled. The adapter requires lock-free C11 atomics and returns failure if the target/compiler cannot provide them. Do not remove that check to make an unsupported compiler build; use a reviewed platform-specific critical-section implementation instead.

## 3. Connect callbacks and main-loop processing

After the project's GPIO/clock/UART initialization, call:

```c
Application_TelemetryInit(actual_uwb_uart, actual_imu_uart,
                          your_frame_output_callback, your_transport_context);
```

Both UART handles and output callback are passed at runtime. Delegate from existing callbacks:

```c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart) {
    Application_UART_RxCplt(uart);
    /* Keep the application's other UART work here. */
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart) {
    Application_UART_Error(uart);
}
```

Call `Application_TelemetryPoll()` regularly from the main loop or one dedicated task. Each call consumes at most 64 bytes per channel. The ISR callback only copies one byte into its channel's SPSC ring and rearms reception. Parsing, output and abort/restart never occur in the receive ISR. Errors clear the partial frame and pending bytes after abort; interrupted frame tails cannot be forwarded as complete samples. Repeated HAL start/abort failures remain visible in diagnostics.

## 4. Route accepted raw frames

The output callback receives `(context, channel, frame, length, parsed_sample)`. `frame` and `parsed_sample` are borrowed for **the duration of the callback only**. Copy the bytes into your own bounded transmit queue before returning if using DMA, USB or asynchronous network output. Never save these pointers for later transmission. On queue-full, drop explicitly and expose your output-drop counter; do not block indefinitely in the processing loop.

Use a separate host stream/device for each channel and configure those actual host ports in the showcase's `serial.uwb` and `serial.imu` settings. Output bytes must remain the original 16-byte UWB packets or supported IMU packets: 23-byte raw, 21-byte quaternion, 17-byte Euler and 21-byte barometer. An IMU stream may interleave these profiles. Do not append debug strings to the same binary stream. A board with insufficient UARTs may provide an independently designed USB transport; this code does not configure USB descriptors or promise two CDC devices.

The IMU sample structure grew in the 2026-10-03 protocol update. Rebuild every consumer and inspect its sample kind before reading raw acceleration or any attitude/barometer payload; inactive fields contain zero and are not measurements. Callbacks that only forward the borrowed frame bytes need no profile-specific conversion. The adapter remains receive-only and performs no calibration, algorithm changes or sensor resets.

For `sample->channel == RF_IMU`, inspect `sample->value.imu.kind`:

| Kind | Valid sample fields |
| --- | --- |
| `RF_IMU_RAW` | `raw_xyz`, `acceleration_m_s2`, `gyro_raw_xyz`, `angular_velocity_rad_s`, `magnetometer_raw_xyz` |
| `RF_IMU_QUATERNION` | `quaternion_wxyz` |
| `RF_IMU_EULER` | `euler_rad` (roll, pitch, yaw) |
| `RF_IMU_BAROMETER` | `barometer.relative_height_m`, `barometer.temperature_c`, `barometer.pressure_pa`, `barometer.reference_pressure_pa` |

The decoder requires 32-bit IEEE 754 binary32 floats. Compile-time checks verify their size, radix, precision and exponent range; a native bit-layout check of `1.0f` and `-0.0f` must also pass before gateway initialization or float decoding. Non-finite float payloads are rejected with `RF_REJECT_VALUE`; malformed known telemetry lengths are rejected with `RF_REJECT_LENGTH`.

The working host `serial_bridge` example demonstrates the same callback contract using a binary stdout sink. Its source can be reviewed and tested without constructing a USB or network protocol.

## 5. Diagnostics and acceptance after arrival

Read `gateway.parser[channel].accepted_frames`, `rejected_frames`, `last_rejection`, `rf_gateway_overruns()`, `discarded_overflow_bytes[]`, `rf_hal_adapter_uart_errors()`, `receive_start_errors[]` and `restart_pending[]`. The board application should display/export these counters through its own diagnostics transport. UWB has no checksum; meaningful acceptance additionally requires known tag ID, stable sequence, valid range geometry and measured reference positions in the backend.

Host compilation and HAL stub checks prove logic and bounds only. Remaining board checks are: actual clock and baud accuracy, voltage compatibility, UART IRQ/DMA timing, buffer pressure at the device output rate, cable unplug/reconnect, tag/anchor firmware roles, RF loss/multipath, antenna calibration, physical same-plane layout, sensor axes and static gravity. None has been claimed as passed in this task.
