/* Application passes its actual Cube/HAL handles; this file defines no pins. */
#ifndef RF_HAL_ADAPTER_H
#define RF_HAL_ADAPTER_H
#include "rf_telemetry.h"
#ifndef RF_HAL_HEADER
#error "Define RF_HAL_HEADER to the actual STM32 family HAL umbrella header"
#endif
#include RF_HAL_HEADER

typedef struct {
    rf_gateway gateway;
    UART_HandleTypeDef *input[2];
    uint8_t receive_byte[2];
    atomic_uint_least8_t restart_pending[2];
    atomic_uint_least32_t uart_errors[2];
    atomic_uint_least32_t receive_start_errors[2];
} rf_hal_adapter;

/* The sink runs in process(), never in HAL ISR callbacks. */
int rf_hal_adapter_init(rf_hal_adapter *adapter,
                        UART_HandleTypeDef *uwb_uart, UART_HandleTypeDef *imu_uart,
                        rf_config config, rf_frame_sink sink, void *context);
/* Call from the application's HAL_UART_RxCpltCallback. Unrelated UARTs are ignored. */
void rf_hal_adapter_rx_complete(rf_hal_adapter *adapter, UART_HandleTypeDef *uart);
/* Call from HAL_UART_ErrorCallback; restart is deferred to main-loop process(). */
void rf_hal_adapter_error(rf_hal_adapter *adapter, UART_HandleTypeDef *uart);
size_t rf_hal_adapter_process(rf_hal_adapter *adapter, size_t budget_per_channel);
uint32_t rf_hal_adapter_uart_errors(const rf_hal_adapter *adapter, rf_channel channel);
#endif
