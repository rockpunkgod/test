#include "vofa.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

Vofa_t vofa;

static uint8_t VOFA_ProcessCommand(GM6020_t *gm6020, char *line)
{
    char *end;
    float value;

    if ((gm6020 == NULL) || (line == NULL) || (line[1] != ':')) {
        return 0U;
    }

    value = strtof(&line[2], &end);
    if ((end == &line[2]) || (!isfinite(value))) {
        return 0U;
    }
    while ((*end == ' ') || (*end == '\t')) {
        ++end;
    }
    if (*end != '\0') {
        return 0U;
    }

    switch (line[0]) {
        case 'M':
            if ((value == 0.0f) || (value == 1.0f) || (value == 2.0f)) {
                GM6020_SetMode(gm6020,
                               (GM6020_ControlMode_t)((uint8_t)value));
                return 1U;
            }
            break;

        case 'S':
            GM6020_SetTargetSpeed(gm6020, value);
            return 1U;

        case 'P':
            GM6020_SetTargetAngle(gm6020, value);
            return 1U;

        default:
            break;
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

void VOFA_Process(Vofa_t *instance, GM6020_t *gm6020)
{
    char line[VOFA_COMMAND_MAX_LEN];
    uint32_t primask;
    uint8_t length = 0U;
    uint8_t command_ready;
    uint8_t safety_stop;

    if ((instance == NULL) || (gm6020 == NULL)) {
        return;
    }

    /* Copy one complete command while the UART interrupt is briefly disabled. */
    primask = __get_PRIMASK();
    __disable_irq();
    safety_stop = instance->safety_stop_pending;
    instance->safety_stop_pending = 0U;
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

    if (safety_stop != 0U) {
        GM6020_SetMode(gm6020, GM6020_MODE_PROTECT);
        return;
    }

    if (command_ready != 0U) {
        if (VOFA_ProcessCommand(gm6020, line) != 0U) {
            ++instance->command_ok_count;
        } else {
            ++instance->command_error_count;
        }
    }
}

HAL_StatusTypeDef VOFA_SendTelemetry(Vofa_t *instance,
                                     const GM6020_t *gm6020,
                                     uint32_t now_ms)
{
    char buffer[160];
    int length;
    GM6020_Telemetry_t telemetry = {0};

    if ((instance == NULL) || (instance->uart == NULL) || (gm6020 == NULL)) {
        return HAL_ERROR;
    }

    GM6020_GetTelemetry(gm6020, now_ms, &telemetry);
    if (telemetry.feedback_online == 0U) {
        instance->display_speed_rpm = 0.0f;
        instance->display_speed_initialized = 0U;
    } else if (instance->display_speed_initialized == 0U) {
        instance->display_speed_rpm = telemetry.speed_rpm;
        instance->display_speed_initialized = 1U;
    } else {
        instance->display_speed_rpm +=
            VOFA_DISPLAY_SPEED_ALPHA
            * (telemetry.speed_rpm - instance->display_speed_rpm);
    }
    /* FireWater channels: target angle, angle, target speed, speed, mode,
     * control output and feedback online state. */
    length = snprintf(buffer,
                      sizeof(buffer),
                      "RM2027:%.3f,%.3f,%.3f,%.3f,%u,%.3f,%u\r\n",
                      (double)telemetry.target_angle_deg,
                      (double)telemetry.relative_angle_deg,
                      (double)telemetry.target_speed_rpm,
                      (double)instance->display_speed_rpm,
                      (unsigned int)telemetry.mode,
                      (double)telemetry.control_output,
                      (unsigned int)telemetry.feedback_online);
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
                vofa.safety_stop_pending = 1U;
                ++vofa.rx_overflow_count;
            }
        }
    }

    if (HAL_UART_Receive_IT(vofa.uart, &vofa.rx_byte, 1U) != HAL_OK) {
        vofa.safety_stop_pending = 1U;
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
    vofa.safety_stop_pending = 1U;
    ++vofa.uart_error_count;
    (void)HAL_UART_Receive_IT(vofa.uart, &vofa.rx_byte, 1U);
}
