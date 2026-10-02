/* This stub test checks HAL integration logic, not STM32 interrupt/device behavior. */
#include "rf_hal_adapter.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t size) {
    assert(size == 1u);
    uart->receive_calls++;
    if (uart->fail_next_receive) { uart->fail_next_receive = 0; return HAL_ERROR; }
    uart->pending = data;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart) {
    uart->abort_calls++;
    uart->pending = NULL;
    if (uart->fail_next_abort) { uart->fail_next_abort = 0; return HAL_ERROR; }
    return HAL_OK;
}
static unsigned frames = 0;
static void sink(void *context, rf_channel channel, const uint8_t *frame,
                 size_t length, const rf_sample *sample) {
    (void)context; (void)frame; (void)sample;
    assert(channel == RF_UWB && length == 16u);
    frames++;
}
static const uint8_t raw[] = {'m','r',2,5,1,2,0xf4,1,0xf4,1,0x90,1,0xf4,1,13,10};
static void feed(rf_hal_adapter *adapter, UART_HandleTypeDef *uart, const uint8_t *data, size_t length) {
    for (size_t i = 0; i < length; ++i) {
        assert(uart->pending);
        *uart->pending = data[i];
        rf_hal_adapter_rx_complete(adapter, uart);
    }
}
int main(void) {
    rf_hal_adapter adapter;
    UART_HandleTypeDef uwb = {0}, imu = {0}, unrelated = {0};
    assert(!rf_hal_adapter_init(&adapter, &uwb, &uwb, rf_default_config(), sink, NULL));
    assert(rf_hal_adapter_init(&adapter, &uwb, &imu, rf_default_config(), sink, NULL));
    assert(uwb.receive_calls == 1u && imu.receive_calls == 1u);
    rf_hal_adapter_rx_complete(&adapter, &unrelated);
    assert(uwb.receive_calls == 1u);
    feed(&adapter, &uwb, raw, sizeof(raw));
    assert(frames == 0u); /* Parsing/output never runs in ISR. */
    assert(rf_hal_adapter_process(&adapter, 64u) == sizeof(raw) && frames == 1u);
    feed(&adapter, &uwb, raw, 5u);
    assert(rf_hal_adapter_process(&adapter, 64u) == 5u);
    assert(adapter.gateway.parser[RF_UWB].used == 5u);
    rf_hal_adapter_error(&adapter, &uwb);
    assert(uwb.abort_calls == 0u); /* Restart also deferred. */
    assert(rf_hal_adapter_uart_errors(&adapter, RF_UWB) == 1u);
    assert(rf_hal_adapter_process(&adapter, 64u) == 0u && uwb.abort_calls == 1u);
    assert(adapter.gateway.parser[RF_UWB].used == 0u);
    feed(&adapter, &uwb, raw + 5, sizeof(raw) - 5u);
    rf_hal_adapter_process(&adapter, 64u);
    assert(frames == 1u); /* The broken frame's tail is not forwarded. */
    feed(&adapter, &uwb, raw, sizeof(raw));
    rf_hal_adapter_process(&adapter, 64u);
    assert(frames == 2u);
    uwb.fail_next_receive = 1;
    feed(&adapter, &uwb, raw, 1u);
    assert(adapter.receive_start_errors[RF_UWB] == 1u);
    uwb.fail_next_abort = 1;
    rf_hal_adapter_process(&adapter, 64u);
    assert(atomic_load(&adapter.restart_pending[RF_UWB]) == 1u);
    rf_hal_adapter_process(&adapter, 64u);
    assert(atomic_load(&adapter.restart_pending[RF_UWB]) == 0u);
    puts("HAL adapter stub: deferred parsing, RX rearm, unrelated UART, errors and partial-frame reset: PASS");
    return 0;
}
