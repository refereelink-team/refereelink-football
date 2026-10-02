#ifndef TEST_FAKE_HAL_H
#define TEST_FAKE_HAL_H
#include <stdint.h>
typedef enum { HAL_OK = 0, HAL_ERROR = 1 } HAL_StatusTypeDef;
typedef struct {
    uint8_t *pending;
    unsigned receive_calls;
    unsigned abort_calls;
    int fail_next_receive;
    int fail_next_abort;
} UART_HandleTypeDef;
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t size);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart);
#endif
