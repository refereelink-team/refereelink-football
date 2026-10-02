/* Compile this with your selected family HAL header and actual Cube project. */
#include "rf_hal_adapter.h"

static rf_hal_adapter telemetry;
static int initialized;

int Application_TelemetryInit(UART_HandleTypeDef *uwb_input,
                              UART_HandleTypeDef *imu_input,
                              rf_frame_sink frame_output, void *output_context) {
    initialized = rf_hal_adapter_init(&telemetry, uwb_input, imu_input,
                                      rf_default_config(), frame_output, output_context);
    return initialized;
}

/* Delegate from your existing HAL callbacks; this example does not replace them. */
void Application_UART_RxCplt(UART_HandleTypeDef *uart) {
    rf_hal_adapter_rx_complete(&telemetry, uart);
}
void Application_UART_Error(UART_HandleTypeDef *uart) {
    rf_hal_adapter_error(&telemetry, uart);
}
size_t Application_TelemetryPoll(void) {
    return initialized ? rf_hal_adapter_process(&telemetry, 64u) : 0u;
}
const rf_hal_adapter *Application_TelemetryDiagnostics(void) {
    return initialized ? &telemetry : 0;
}
