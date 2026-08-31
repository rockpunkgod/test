#include "canio.h"
#include "vofa.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

Vofa_t vofa;

static uint8_t VOFA_ReadValue(char *text, float *value)
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
    if (*end != '\0') {
        return 0U;
    }

    return 1U;
}

static uint8_t VOFA_ProcessCommand(char *line)
{
    char *value_text;
    float value;

    if (line == NULL) {
        return 0U;
    }

    if ((line[0] == 'M') && (line[1] == ':')) {
        value_text = &line[2];
    } else if ((line[0] == 'Y') && (line[1] == 'S') && (line[2] == ':')) {
        value_text = &line[3];
    } else if ((line[0] == 'P') && (line[1] == 'S') && (line[2] == ':')) {
        value_text = &line[3];
    } else if ((line[0] == 'Y') && (line[1] == 'A') && (line[2] == ':')) {
        value_text = &line[3];
    } else if ((line[0] == 'P') && (line[1] == 'A') && (line[2] == ':')) {
        value_text = &line[3];
    } else {
        return 0U;
    }

    if (VOFA_ReadValue(value_text, &value) == 0U) {
        return 0U;
    }

    if (line[0] == 'M') {
        if ((value == 0.0f) || (value == 1.0f) || (value == 2.0f)) {
            Gimbal_SetMode((GM6020_ControlMode_t)((uint8_t)value));
            return 1U;
        }
        return 0U;
    }
    if ((line[0] == 'Y') && (line[1] == 'S')) {
        Gimbal_SetYawSpeed(value);
        return 1U;
    }
    if ((line[0] == 'P') && (line[1] == 'S')) {
        Gimbal_SetPitchSpeed(value);
        return 1U;
    }
    if ((line[0] == 'Y') && (line[1] == 'A')) {
        Gimbal_SetYawAngle(value);
        return 1U;
    }
    if ((line[0] == 'P') && (line[1] == 'A')) {
        Gimbal_SetPitchAngle(value);
        return 1U;
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
    uint8_t safety_stop;

    if (instance == NULL) {
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
        Gimbal_SetMode(GM6020_MODE_PROTECT);
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
    char buffer[240];
    int length;
    Gimbal_Telemetry_t telemetry = {0};

    if ((instance == NULL) || (instance->uart == NULL)) {
        return HAL_ERROR;
    }

    Gimbal_GetTelemetry(now_ms, &telemetry);

    /* FireWater: four Yaw channels, four Pitch channels, mode, two outputs
     * and two feedback-online flags. */
    length = snprintf(buffer,
                      sizeof(buffer),
                      "RM2027:%.2f,%.2f,%.2f,%.2f,"
                      "%.2f,%.2f,%.2f,%.2f,%u,%.2f,%.2f,%u,%u,%u\r\n",
                      (double)telemetry.yaw.target_angle_deg,
                      (double)telemetry.yaw.relative_angle_deg,
                      (double)telemetry.yaw.target_speed_rpm,
                      (double)telemetry.yaw.speed_rpm,
                      (double)telemetry.pitch.target_angle_deg,
                      (double)telemetry.pitch.relative_angle_deg,
                      (double)telemetry.pitch.target_speed_rpm,
                      (double)telemetry.pitch.speed_rpm,
                      (unsigned int)telemetry.mode,
                      (double)telemetry.yaw.control_output,
                      (double)telemetry.pitch.control_output,
                      (unsigned int)telemetry.yaw.feedback_online,
                      (unsigned int)telemetry.pitch.feedback_online,
                      (unsigned int)canio_available());
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
