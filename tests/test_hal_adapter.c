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
static unsigned imu_frames = 0;
static rf_imu_sample last_imu;
static void sink(void *context, rf_channel channel, const uint8_t *frame,
                 size_t length, const rf_sample *sample) {
    (void)context; (void)frame;
    assert(channel == sample->channel);
    if (channel == RF_UWB) {
        assert(length == 16u);
        frames++;
    } else {
        assert(channel == RF_IMU);
        assert((sample->value.imu.kind == RF_IMU_EULER && length == 17u) ||
               (sample->value.imu.kind == RF_IMU_QUATERNION && length == 21u));
        last_imu = sample->value.imu;
        imu_frames++;
    }
}
static const uint8_t raw[] = {'m','r',2,5,1,2,0xf4,1,0xf4,1,0x90,1,0xf4,1,13,10};
static void feed(rf_hal_adapter *adapter, UART_HandleTypeDef *uart, const uint8_t *data, size_t length) {
    for (size_t i = 0; i < length; ++i) {
        assert(uart->pending);
        *uart->pending = data[i];
        rf_hal_adapter_rx_complete(adapter, uart);
    }
}
static void test_imu_recovery_keeps_uwb_independent(void) {
    /* Synthetic frames from the documented float32 layout, not device captures. */
    static const uint8_t euler[] = {
        0x7e,0x23,0x11,0x26,0,0,0,0x3f,0,0,0x80,0xbf,0,0,0xc0,0x3f,0x55
    };
    static const uint8_t quaternion[] = {
        0x7e,0x23,0x15,0x16,0,0,0x80,0x3f,0,0,0,0,0,0,0,0,0,0,0,0,0x8b
    };
    rf_hal_adapter adapter;
    UART_HandleTypeDef uwb = {0}, imu = {0};
    frames = 0;
    imu_frames = 0;
    assert(rf_hal_adapter_init(&adapter, &uwb, &imu, rf_default_config(), sink, NULL));
    feed(&adapter, &imu, euler, 5u);
    assert(imu_frames == 0u);
    assert(rf_hal_adapter_process(&adapter, 64u) == 5u);
    assert(adapter.gateway.parser[RF_IMU].used == 5u);
    feed(&adapter, &uwb, raw, sizeof(raw));
    rf_hal_adapter_error(&adapter, &imu);
    assert(imu.abort_calls == 0u && uwb.abort_calls == 0u);
    assert(rf_hal_adapter_process(&adapter, 64u) == sizeof(raw));
    assert(imu.abort_calls == 1u && uwb.abort_calls == 0u);
    assert(frames == 1u && imu_frames == 0u);
    assert(adapter.gateway.parser[RF_IMU].used == 0u);
    assert(rf_hal_adapter_uart_errors(&adapter, RF_IMU) == 1u);
    assert(rf_hal_adapter_uart_errors(&adapter, RF_UWB) == 0u);
    feed(&adapter, &imu, euler + 5u, sizeof(euler) - 5u);
    rf_hal_adapter_process(&adapter, 64u);
    assert(imu_frames == 0u);
    feed(&adapter, &imu, euler, sizeof(euler));
    assert(imu_frames == 0u); /* Still no parsing in the receive ISR. */
    assert(rf_hal_adapter_process(&adapter, 64u) == sizeof(euler));
    assert(imu_frames == 1u && last_imu.kind == RF_IMU_EULER);
    assert(last_imu.euler_rad[0] == 0.5f);
    assert(last_imu.euler_rad[1] == -1.0f);
    assert(last_imu.euler_rad[2] == 1.5f);
    assert(last_imu.acceleration_m_s2[0] == 0.0f);
    feed(&adapter, &imu, quaternion, sizeof(quaternion));
    assert(imu_frames == 1u);
    assert(rf_hal_adapter_process(&adapter, 64u) == sizeof(quaternion));
    assert(imu_frames == 2u && last_imu.kind == RF_IMU_QUATERNION);
    assert(last_imu.quaternion_wxyz[0] == 1.0f);
    assert(last_imu.euler_rad[0] == 0.0f);
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
    test_imu_recovery_keeps_uwb_independent();
    puts("HAL adapter stub: deferred parsing, RX rearm, unrelated UART, errors and partial-frame reset: PASS");
    return 0;
}
