#include "rf_hal_adapter.h"
#include <string.h>

static int channel_for(const rf_hal_adapter *adapter, const UART_HandleTypeDef *uart) {
    if (!adapter || !uart) return -1;
    for (int i = 0; i < 2; ++i) if (adapter->input[i] == uart) return i;
    return -1;
}
static void start_receive(rf_hal_adapter *adapter, size_t channel) {
    if (HAL_UART_Receive_IT(adapter->input[channel], &adapter->receive_byte[channel], 1u) != HAL_OK) {
        atomic_fetch_add_explicit(&adapter->receive_start_errors[channel], 1u, memory_order_relaxed);
        atomic_store_explicit(&adapter->restart_pending[channel], 1u, memory_order_release);
    }
}
int rf_hal_adapter_init(rf_hal_adapter *adapter,
                        UART_HandleTypeDef *uwb_uart, UART_HandleTypeDef *imu_uart,
                        rf_config config, rf_frame_sink sink, void *context) {
    if (!adapter || !uwb_uart || !imu_uart || uwb_uart == imu_uart) return 0;
    memset(adapter, 0, sizeof(*adapter));
    adapter->input[RF_UWB] = uwb_uart;
    adapter->input[RF_IMU] = imu_uart;
    for (size_t i = 0; i < 2; ++i) {
        atomic_init(&adapter->restart_pending[i], 0u);
        atomic_init(&adapter->uart_errors[i], 0u);
        atomic_init(&adapter->receive_start_errors[i], 0u);
        if (!atomic_is_lock_free(&adapter->restart_pending[i]) ||
                !atomic_is_lock_free(&adapter->uart_errors[i]) ||
                !atomic_is_lock_free(&adapter->receive_start_errors[i])) return 0;
    }
    if (!rf_gateway_init(&adapter->gateway, config, sink, context)) return 0;
    start_receive(adapter, RF_UWB);
    start_receive(adapter, RF_IMU);
    return 1;
}
void rf_hal_adapter_rx_complete(rf_hal_adapter *adapter, UART_HandleTypeDef *uart) {
    int channel = channel_for(adapter, uart);
    if (channel < 0) return;
    if (atomic_load_explicit(&adapter->restart_pending[channel], memory_order_acquire)) return;
    rf_gateway_receive(&adapter->gateway, (rf_channel)channel, &adapter->receive_byte[channel], 1u);
    start_receive(adapter, (size_t)channel);
}
void rf_hal_adapter_error(rf_hal_adapter *adapter, UART_HandleTypeDef *uart) {
    int channel = channel_for(adapter, uart);
    if (channel < 0) return;
    uint_least32_t old = atomic_load_explicit(&adapter->uart_errors[channel], memory_order_relaxed);
    atomic_store_explicit(&adapter->uart_errors[channel], old + 1u, memory_order_release);
    atomic_store_explicit(&adapter->restart_pending[channel], 1u, memory_order_release);
}
size_t rf_hal_adapter_process(rf_hal_adapter *adapter, size_t budget_per_channel) {
    if (!adapter) return 0u;
    for (size_t channel = 0; channel < 2; ++channel) {
        if (atomic_exchange_explicit(&adapter->restart_pending[channel], 0u, memory_order_acq_rel)) {
            HAL_StatusTypeDef status = HAL_UART_AbortReceive(adapter->input[channel]);
            if (status != HAL_OK) {
                atomic_fetch_add_explicit(&adapter->receive_start_errors[channel], 1u, memory_order_relaxed);
                atomic_store_explicit(&adapter->restart_pending[channel], 1u, memory_order_release);
            }
            /* Flush after abort so a pre-abort interrupt cannot retain a partial frame. */
            rf_ring *ring = &adapter->gateway.input[channel];
            uint_least16_t head = atomic_load_explicit(&ring->head, memory_order_acquire);
            atomic_store_explicit(&ring->tail, head, memory_order_release);
            adapter->gateway.parser[channel].used = 0u;
            if (status == HAL_OK) {
                start_receive(adapter, channel);
            }
        }
    }
    return rf_gateway_process(&adapter->gateway, budget_per_channel);
}
uint32_t rf_hal_adapter_uart_errors(const rf_hal_adapter *adapter, rf_channel channel) {
    if (!adapter || (channel != RF_UWB && channel != RF_IMU)) return 0u;
    return atomic_load_explicit(&adapter->uart_errors[channel], memory_order_acquire);
}
