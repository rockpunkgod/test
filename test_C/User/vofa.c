#include "vofa.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

Vofa_t vofa;

static uint8_t VOFA_ReadFloat(const char *text, float *value)
{
    char *end;

    if ((text == NULL) || (value == NULL)) {
        return 0U;
    }

    *value = strtof(text, &end);
    if ((end == text) || (!isfinite(*value))) {
        return 0U;
    }
    while ((*end == ' ') || (*end == '\t')) {
        ++end;
    }
    return (*end == '\0') ? 1U : 0U;
}

static uint8_t VOFA_ProcessCommand(char *line)
{
    float value;

    if (line == NULL) {
        return 0U;
    }

    /* 两块板烧录同一份固件，通过串口命令选择角色。 */
    if (strcmp(line, "ROLE:A") == 0) {
        /* 只需指定A板；A板随后会通过CAN自动分配B板。 */
        BoardComm_SetRole(BOARD_COMM_ROLE_A);
        return 1U;
    }
    if (strcmp(line, "ROLE:0") == 0) {
        BoardComm_SetRole(BOARD_COMM_ROLE_NONE);
        return 1U;
    }

    if (strcmp(line, "H") == 0) {
        return BoardComm_RequestHandshake();
    }
    if ((line[0] == 'D') && (line[1] == ':')) {
        if (VOFA_ReadFloat(&line[2], &value) != 0U) {
            return BoardComm_RequestDegreeToRadian(value);
        }
    }
    if ((line[0] == 'R') && (line[1] == ':')) {
        if (VOFA_ReadFloat(&line[2], &value) != 0U) {
            return BoardComm_RequestRadianToDegree(value);
        }
    }

    return 0U;
}

HAL_StatusTypeDef VOFA_Init(Vofa_t *instance, UART_HandleTypeDef *uart)
{
    if ((instance == NULL) || (uart == NULL)) {
        return HAL_ERROR;
    }

    memset(instance, 0, sizeof(*instance));
    instance->uart = uart;
    return HAL_UART_Receive_IT(instance->uart, &instance->rx_byte, 1U);
}

void VOFA_Process(Vofa_t *instance)
{
    char line[VOFA_COMMAND_MAX_LEN];
    uint32_t primask;
    uint8_t length = 0U;
    uint8_t command_ready;
    uint8_t communication_stop;

    if (instance == NULL) {
        return;
    }

    /* 中断只负责收字符，完整命令在任务中解析。 */
    primask = __get_PRIMASK();
    __disable_irq();
    communication_stop = instance->communication_stop_pending;
    instance->communication_stop_pending = 0U;
    command_ready = instance->command_ready;
    if (command_ready != 0U) {
        length = instance->command_length;
        memcpy(line, (const void *)instance->command_line, (size_t)length + 1U);
        instance->command_ready = 0U;
        instance->command_length = 0U;
    }
    if (primask == 0U) {
        __enable_irq();
    }

    /* 只有A板依赖上位机串口；B板串口未连接时不改变其CAN角色。 */
    if (communication_stop != 0U) {
        if (board_comm.role == BOARD_COMM_ROLE_A) {
            BoardComm_SetRole(BOARD_COMM_ROLE_NONE);
        }
        return;
    }

    if (command_ready != 0U) {
        if (VOFA_ProcessCommand(line) != 0U) {
            ++instance->command_ok_count;
        } else {
            ++instance->command_error_count;
        }
    }
}

HAL_StatusTypeDef VOFA_SendTelemetry(Vofa_t *instance, uint32_t now_ms)
{
    char buffer[192];
    int length;
    BoardComm_Telemetry_t telemetry = {0};

    if ((instance == NULL) || (instance->uart == NULL)) {
        return HAL_ERROR;
    }

    BoardComm_GetTelemetry(now_ms, &telemetry);

    /* B板数据先通过CAN交给A板，只有A板统一向上位机输出。 */
    if (telemetry.role != BOARD_COMM_ROLE_A) {
        return HAL_OK;
    }

    /* 保留原10个通道，并在末尾增加角色分配是否成功。 */
    length = snprintf(buffer,
                      sizeof(buffer),
                      "RM2027:%u,%u,%.6f,%.6f,%u,%u,%u,%lu,%lu,%lu,%u\r\n",
                      (unsigned int)telemetry.role,
                      (unsigned int)telemetry.last_operation,
                      (double)telemetry.input_value,
                      (double)telemetry.output_value,
                      (unsigned int)telemetry.operation_complete,
                      (unsigned int)telemetry.handshake_ok,
                      (unsigned int)telemetry.peer_online,
                      (unsigned long)telemetry.tx_count,
                      (unsigned long)telemetry.rx_count,
                      (unsigned long)(telemetry.error_count
                                      + instance->command_error_count
                                      + instance->rx_overflow_count
                                      + instance->uart_error_count),
                      (unsigned int)telemetry.role_assignment_ok);
    if ((length <= 0) || ((size_t)length >= sizeof(buffer))) {
        return HAL_ERROR;
    }

    return HAL_UART_Transmit(instance->uart,
                             (uint8_t *)buffer,
                             (uint16_t)length,
                             VOFA_TX_TIMEOUT_MS);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    if ((vofa.uart == NULL) || (uart != vofa.uart)) {
        return;
    }

    if (vofa.command_ready == 0U) {
        if (vofa.discard_line != 0U) {
            if (vofa.rx_byte == '\n') {
                vofa.discard_line = 0U;
            }
        } else if (vofa.rx_byte == '\n') {
            if (vofa.command_length > 0U) {
                vofa.command_line[vofa.command_length] = '\0';
                __DMB();
                vofa.command_ready = 1U;
            }
        } else if (vofa.rx_byte != '\r') {
            if (vofa.command_length < (VOFA_COMMAND_MAX_LEN - 1U)) {
                vofa.command_line[vofa.command_length] = (char)vofa.rx_byte;
                ++vofa.command_length;
            } else {
                vofa.command_length = 0U;
                vofa.discard_line = 1U;
                vofa.communication_stop_pending = 1U;
                ++vofa.rx_overflow_count;
            }
        }
    }

    if (HAL_UART_Receive_IT(vofa.uart, &vofa.rx_byte, 1U) != HAL_OK) {
        vofa.communication_stop_pending = 1U;
        ++vofa.uart_error_count;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if ((vofa.uart == NULL) || (uart != vofa.uart)) {
        return;
    }

    vofa.command_length = 0U;
    vofa.discard_line = 1U;
    vofa.communication_stop_pending = 1U;
    ++vofa.uart_error_count;
    (void)HAL_UART_Receive_IT(vofa.uart, &vofa.rx_byte, 1U);
}
