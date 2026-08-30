#ifndef CAN_DEMO_VOFA_H
#define CAN_DEMO_VOFA_H

#include "board_comm.h"

#include <stdint.h>

#define VOFA_COMMAND_MAX_LEN  64U
#define VOFA_TX_TIMEOUT_MS    15U

typedef struct {
    UART_HandleTypeDef *uart;
    uint8_t rx_byte;
    volatile char command_line[VOFA_COMMAND_MAX_LEN];
    volatile uint8_t command_length;
    volatile uint8_t command_ready;
    volatile uint8_t discard_line;
    volatile uint8_t communication_stop_pending;
    volatile uint32_t rx_overflow_count;
    volatile uint32_t command_error_count;
    volatile uint32_t command_ok_count;
    volatile uint32_t uart_error_count;
} Vofa_t;

extern Vofa_t vofa;

HAL_StatusTypeDef VOFA_Init(Vofa_t *instance, UART_HandleTypeDef *uart);
void VOFA_Process(Vofa_t *instance);
HAL_StatusTypeDef VOFA_SendTelemetry(Vofa_t *instance, uint32_t now_ms);

#endif /* CAN_DEMO_VOFA_H */
