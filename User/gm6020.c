#include "gm6020.h"

#include <stddef.h>
#include <string.h>

GM6020_t motor[3];

static float GM6020_Clamp(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

void GM6020_Init(GM6020_t *gm6020, uint8_t motor_id)
{
    if (gm6020 == NULL) {
        return;
    }

    memset(gm6020, 0, sizeof(*gm6020));
    gm6020->motor_id = motor_id;
    gm6020->target_angle_deg = GM6020_TARGET_ANGLE_DEG;

    /* Parameters intentionally remain zero until safe bench tuning begins. */
    PID_Init(&gm6020->angle_pid,
             0.0f,
             0.0f,
             0.0f,
             GM6020_ANGLE_PID_OUTPUT_LIMIT,
             GM6020_ANGLE_PID_OUTPUT_LIMIT);
    PID_Init(&gm6020->speed_pid,
             0.0f,
             0.0f,
             0.0f,
             GM6020_CONTROL_OUTPUT_LIMIT,
             GM6020_CONTROL_OUTPUT_LIMIT);
}

void GM6020_ParseFeedback(GM6020_t *gm6020, const CanMsg_t *msg)
{
    uint16_t encoder;
    int16_t speed_rpm;
    int16_t feedback_current;
    int32_t encoder_delta;

    if ((gm6020 == NULL) || (msg == NULL)) {
        return;
    }
    if ((msg->is_ext != 0U)
        || (msg->is_remote != 0U)
        || (msg->dlc != 8U)
        || (msg->id != GM6020_FEEDBACK_STD_ID)) {
        return;
    }

    encoder = (uint16_t)(((uint16_t)msg->data[0] << 8U) | msg->data[1]);
    speed_rpm = (int16_t)(((uint16_t)msg->data[2] << 8U) | msg->data[3]);
    feedback_current = (int16_t)(((uint16_t)msg->data[4] << 8U) | msg->data[5]);

    if (gm6020->has_feedback == 0U) {
        gm6020->zero_encoder = encoder;
        gm6020->last_encoder = encoder;
        gm6020->total_encoder_counts = 0;
        gm6020->relative_angle = 0.0f;
    } else {
        encoder_delta = (int32_t)encoder - (int32_t)gm6020->last_encoder;
        if (encoder_delta > GM6020_ENCODER_HALF_RANGE) {
            encoder_delta -= GM6020_ENCODER_RESOLUTION;
        } else if (encoder_delta < -GM6020_ENCODER_HALF_RANGE) {
            encoder_delta += GM6020_ENCODER_RESOLUTION;
        }

        gm6020->total_encoder_counts += encoder_delta;
        gm6020->relative_angle =
            ((float)gm6020->total_encoder_counts * 360.0f)
            / (float)GM6020_ENCODER_RESOLUTION;
        gm6020->last_encoder = encoder;
    }

    gm6020->encoder = encoder;
    gm6020->speed_rpm = speed_rpm;
    gm6020->feedback_current = feedback_current;
    gm6020->temperature = msg->data[6];
    gm6020->last_feedback_tick = HAL_GetTick();
    __DMB();
    gm6020->has_feedback = 1U;
}

int16_t GM6020_CalculateControl(GM6020_t *gm6020,
                                float target_angle_deg,
                                uint32_t now_ms)
{
    uint32_t primask;
    uint8_t has_feedback;
    uint32_t last_feedback_tick;
    float relative_angle;
    float speed_rpm;
    float control_output;

    if (gm6020 == NULL) {
        return 0;
    }

    /* Take a coherent snapshot of fields written by the CAN RX interrupt. */
    primask = __get_PRIMASK();
    __disable_irq();
    has_feedback = gm6020->has_feedback;
    last_feedback_tick = gm6020->last_feedback_tick;
    relative_angle = gm6020->relative_angle;
    speed_rpm = (float)gm6020->speed_rpm;
    if (primask == 0U) {
        __enable_irq();
    }

    gm6020->target_angle_deg = target_angle_deg;

    if ((has_feedback == 0U)
        || ((uint32_t)(now_ms - last_feedback_tick) > GM6020_FEEDBACK_TIMEOUT_MS)) {
        PID_Reset(&gm6020->angle_pid);
        PID_Reset(&gm6020->speed_pid);
        gm6020->target_speed_rpm = 0.0f;
        gm6020->control_output = 0.0f;
        return 0;
    }

    gm6020->target_speed_rpm = PID_Calculate(&gm6020->angle_pid,
                                              target_angle_deg,
                                              relative_angle,
                                              GM6020_CONTROL_PERIOD_S);
    control_output = PID_Calculate(&gm6020->speed_pid,
                                   gm6020->target_speed_rpm,
                                   speed_rpm,
                                   GM6020_CONTROL_PERIOD_S);
    control_output = GM6020_Clamp(control_output, GM6020_CONTROL_OUTPUT_LIMIT);
    gm6020->control_output = control_output;

    if (control_output >= 0.0f) {
        return (int16_t)(control_output + 0.5f);
    }
    return (int16_t)(control_output - 0.5f);
}

HAL_StatusTypeDef GM6020_SendMotor2Control(int16_t control_output)
{
    CanMsg_t msg = {0};
    uint16_t raw_output = (uint16_t)control_output;

    msg.id = GM6020_CONTROL_STD_ID;
    msg.dlc = 8U;
    msg.is_ext = 0U;
    msg.is_remote = 0U;
    msg.data[2] = (uint8_t)(raw_output >> 8U);
    msg.data[3] = (uint8_t)(raw_output & 0xFFU);

    return canio_send(&msg);
}
