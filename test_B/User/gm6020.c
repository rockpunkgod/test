#include "gm6020.h"

#include <math.h>
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

static float GM6020_ClampRange(float value, float minimum, float maximum)
{
    if (value > maximum) {
        return maximum;
    }
    if (value < minimum) {
        return minimum;
    }
    return value;
}

static float GM6020_WrapTo180(float angle_deg)
{
    angle_deg = fmodf(angle_deg + 180.0f, 360.0f);
    if (angle_deg < 0.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg - 180.0f;
}

void GM6020_Init(GM6020_t *gm6020, uint8_t motor_id)
{
    if (gm6020 == NULL) {
        return;
    }

    memset(gm6020, 0, sizeof(*gm6020));
    gm6020->motor_id = motor_id;
    gm6020->feedback_std_id = GM6020_FEEDBACK_BASE_STD_ID + motor_id;
    gm6020->command_mode = GM6020_MODE_PROTECT;
    gm6020->active_mode = GM6020_MODE_PROTECT;
    gm6020->command_angle_deg = GM6020_TARGET_ANGLE_DEG;
    gm6020->target_angle_deg = GM6020_TARGET_ANGLE_DEG;
    gm6020->max_speed_rpm = GM6020_SPEED_COMMAND_LIMIT_RPM;

    /* Parameters intentionally remain zero until safe bench tuning begins. */
    PID_Init(&gm6020->angle_pid,
             GM6020_ANGLE_KP,
             GM6020_ANGLE_KI,
             GM6020_ANGLE_KD,
             GM6020_ANGLE_PID_OUTPUT_LIMIT,
             GM6020_ANGLE_PID_OUTPUT_LIMIT);
    PID_Init(&gm6020->speed_pid,
             GM6020_SPEED_KP,
             GM6020_SPEED_KI,
             GM6020_SPEED_KD,
             GM6020_CONTROL_OUTPUT_LIMIT,
             GM6020_CONTROL_OUTPUT_LIMIT);
}

void GM6020_SetSoftwareLimits(GM6020_t *gm6020,
                              float min_angle_deg,
                              float max_angle_deg,
                              float max_speed_rpm)
{
    if ((gm6020 == NULL)
        || (!isfinite(min_angle_deg))
        || (!isfinite(max_angle_deg))
        || (!isfinite(max_speed_rpm))
        || (min_angle_deg >= max_angle_deg)
        || (max_speed_rpm <= 0.0f)) {
        return;
    }

    gm6020->min_angle_deg = min_angle_deg;
    gm6020->max_angle_deg = max_angle_deg;
    gm6020->max_speed_rpm = GM6020_Clamp(max_speed_rpm,
                                         GM6020_SPEED_COMMAND_LIMIT_RPM);
    gm6020->software_limit_enabled = 1U;

    GM6020_SetTargetAngle(gm6020, gm6020->command_angle_deg);
    GM6020_SetTargetSpeed(gm6020, gm6020->command_speed_rpm);
}

void GM6020_SetMode(GM6020_t *gm6020, GM6020_ControlMode_t mode)
{
    uint32_t primask;

    if ((gm6020 == NULL)
        || (mode < GM6020_MODE_PROTECT)
        || (mode > GM6020_MODE_POSITION)) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    gm6020->command_mode = mode;
    __DMB();
    if (primask == 0U) {
        __enable_irq();
    }
}

void GM6020_SetTargetSpeed(GM6020_t *gm6020, float target_speed_rpm)
{
    uint32_t primask;

    if ((gm6020 == NULL) || (!isfinite(target_speed_rpm))) {
        return;
    }

    target_speed_rpm = GM6020_Clamp(
        target_speed_rpm,
        (gm6020->software_limit_enabled != 0U)
            ? gm6020->max_speed_rpm
            : GM6020_SPEED_COMMAND_LIMIT_RPM);
    primask = __get_PRIMASK();
    __disable_irq();
    gm6020->command_speed_rpm = target_speed_rpm;
    __DMB();
    if (primask == 0U) {
        __enable_irq();
    }
}

void GM6020_SetTargetAngle(GM6020_t *gm6020, float target_angle_deg)
{
    uint32_t primask;

    if ((gm6020 == NULL) || (!isfinite(target_angle_deg))) {
        return;
    }

    if (gm6020->software_limit_enabled != 0U) {
        target_angle_deg = GM6020_ClampRange(target_angle_deg,
                                             gm6020->min_angle_deg,
                                             gm6020->max_angle_deg);
    } else {
        target_angle_deg = GM6020_WrapTo180(target_angle_deg);
    }
    primask = __get_PRIMASK();
    __disable_irq();
    gm6020->command_angle_deg = target_angle_deg;
    __DMB();
    if (primask == 0U) {
        __enable_irq();
    }
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
        || (msg->id != gm6020->feedback_std_id)) {
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

int16_t GM6020_CalculateControl(GM6020_t *gm6020, uint32_t now_ms)
{
    uint32_t primask;
    uint8_t has_feedback;
    uint32_t last_feedback_tick;
    GM6020_ControlMode_t command_mode;
    float command_angle_deg;
    float command_speed_rpm;
    float relative_angle;
    float speed_rpm;
    float control_output;
    float distance_to_limit;
    float braking_speed_limit;

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
    command_mode = gm6020->command_mode;
    command_angle_deg = gm6020->command_angle_deg;
    command_speed_rpm = gm6020->command_speed_rpm;
    if (primask == 0U) {
        __enable_irq();
    }

    if (command_mode != gm6020->active_mode) {
        PID_Reset(&gm6020->angle_pid);
        PID_Reset(&gm6020->speed_pid);
        gm6020->active_mode = command_mode;
    }

    if ((has_feedback == 0U)
        || ((uint32_t)(now_ms - last_feedback_tick) > GM6020_FEEDBACK_TIMEOUT_MS)) {
        PID_Reset(&gm6020->angle_pid);
        PID_Reset(&gm6020->speed_pid);
        gm6020->target_speed_rpm = 0.0f;
        gm6020->control_output = 0.0f;
        return 0;
    }

    if (command_mode == GM6020_MODE_PROTECT) {
        PID_Reset(&gm6020->angle_pid);
        PID_Reset(&gm6020->speed_pid);
        gm6020->target_angle_deg = relative_angle;
        gm6020->target_speed_rpm = 0.0f;
        gm6020->control_output = 0.0f;
        return 0;
    }

    if (command_mode == GM6020_MODE_SPEED) {
        gm6020->target_angle_deg = relative_angle;
        gm6020->target_speed_rpm = command_speed_rpm;
    } else if (gm6020->software_limit_enabled != 0U) {
        /* A mechanical axis cannot wrap through its software boundary. */
        gm6020->target_angle_deg = GM6020_ClampRange(command_angle_deg,
                                                     gm6020->min_angle_deg,
                                                     gm6020->max_angle_deg);
        gm6020->target_speed_rpm = PID_Calculate(&gm6020->angle_pid,
                                                  gm6020->target_angle_deg,
                                                  relative_angle,
                                                  GM6020_CONTROL_PERIOD_S);
    } else {
        /* Choose the equivalent target angle requiring at most 180 degrees. */
        gm6020->target_angle_deg = relative_angle
            + GM6020_WrapTo180(command_angle_deg - relative_angle);
        gm6020->target_speed_rpm = PID_Calculate(&gm6020->angle_pid,
                                                  gm6020->target_angle_deg,
                                                  relative_angle,
                                                  GM6020_CONTROL_PERIOD_S);
    }

    if (gm6020->software_limit_enabled != 0U) {
        gm6020->target_speed_rpm = GM6020_Clamp(gm6020->target_speed_rpm,
                                                gm6020->max_speed_rpm);

        /* Slow down before the boundary so speed mode does not brake too late. */
        if (gm6020->target_speed_rpm > 0.0f) {
            distance_to_limit = gm6020->max_angle_deg - relative_angle;
            if (distance_to_limit <= 0.0f) {
                gm6020->target_speed_rpm = 0.0f;
                PID_Reset(&gm6020->speed_pid);
            } else if (distance_to_limit < GM6020_LIMIT_BRAKE_ZONE_DEG) {
                braking_speed_limit = gm6020->max_speed_rpm
                    * distance_to_limit / GM6020_LIMIT_BRAKE_ZONE_DEG;
                if (gm6020->target_speed_rpm > braking_speed_limit) {
                    gm6020->target_speed_rpm = braking_speed_limit;
                    PID_Reset(&gm6020->speed_pid);
                }
            }
        } else if (gm6020->target_speed_rpm < 0.0f) {
            distance_to_limit = relative_angle - gm6020->min_angle_deg;
            if (distance_to_limit <= 0.0f) {
                gm6020->target_speed_rpm = 0.0f;
                PID_Reset(&gm6020->speed_pid);
            } else if (distance_to_limit < GM6020_LIMIT_BRAKE_ZONE_DEG) {
                braking_speed_limit = gm6020->max_speed_rpm
                    * distance_to_limit / GM6020_LIMIT_BRAKE_ZONE_DEG;
                if (gm6020->target_speed_rpm < -braking_speed_limit) {
                    gm6020->target_speed_rpm = -braking_speed_limit;
                    PID_Reset(&gm6020->speed_pid);
                }
            }
        }
    }

    control_output = PID_Calculate(&gm6020->speed_pid,
                                   gm6020->target_speed_rpm,
                                   speed_rpm,
                                   GM6020_CONTROL_PERIOD_S);
    control_output = GM6020_Clamp(control_output, GM6020_CONTROL_OUTPUT_LIMIT);

    /* Never apply torque farther outside an enabled software boundary. */
    if ((gm6020->software_limit_enabled != 0U)
        && (((relative_angle >= gm6020->max_angle_deg)
             && (control_output > 0.0f))
            || ((relative_angle <= gm6020->min_angle_deg)
                && (control_output < 0.0f)))) {
        PID_Reset(&gm6020->speed_pid);
        control_output = 0.0f;
    }

    gm6020->control_output = control_output;

    if (control_output >= 0.0f) {
        return (int16_t)(control_output + 0.5f);
    }
    return (int16_t)(control_output - 0.5f);
}

void GM6020_GetTelemetry(const GM6020_t *gm6020,
                         uint32_t now_ms,
                         GM6020_Telemetry_t *telemetry)
{
    uint32_t primask;
    uint8_t has_feedback;
    uint32_t last_feedback_tick;

    if ((gm6020 == NULL) || (telemetry == NULL)) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    has_feedback = gm6020->has_feedback;
    last_feedback_tick = gm6020->last_feedback_tick;
    telemetry->mode = gm6020->active_mode;
    telemetry->target_angle_deg = gm6020->target_angle_deg;
    telemetry->relative_angle_deg = gm6020->relative_angle;
    telemetry->target_speed_rpm = gm6020->target_speed_rpm;
    telemetry->speed_rpm = (float)gm6020->speed_rpm;
    telemetry->control_output = gm6020->control_output;
    if (primask == 0U) {
        __enable_irq();
    }

    telemetry->feedback_online =
        ((has_feedback != 0U)
         && ((uint32_t)(now_ms - last_feedback_tick)
             <= GM6020_FEEDBACK_TIMEOUT_MS)) ? 1U : 0U;
}

HAL_StatusTypeDef GM6020_SendMotor2Control(int16_t control_output)
{
    return GM6020_SendGimbalControl(0, control_output);
}

HAL_StatusTypeDef GM6020_SendGimbalControl(int16_t motor1_output,
                                           int16_t motor2_output)
{
    CanMsg_t msg = {0};
    uint16_t raw_motor1;
    uint16_t raw_motor2;

    if (motor1_output > (int16_t)GM6020_CONTROL_OUTPUT_LIMIT) {
        motor1_output = (int16_t)GM6020_CONTROL_OUTPUT_LIMIT;
    } else if (motor1_output < (int16_t)-GM6020_CONTROL_OUTPUT_LIMIT) {
        motor1_output = (int16_t)-GM6020_CONTROL_OUTPUT_LIMIT;
    }
    if (motor2_output > (int16_t)GM6020_CONTROL_OUTPUT_LIMIT) {
        motor2_output = (int16_t)GM6020_CONTROL_OUTPUT_LIMIT;
    } else if (motor2_output < (int16_t)-GM6020_CONTROL_OUTPUT_LIMIT) {
        motor2_output = (int16_t)-GM6020_CONTROL_OUTPUT_LIMIT;
    }
    raw_motor1 = (uint16_t)motor1_output;
    raw_motor2 = (uint16_t)motor2_output;

    msg.id = GM6020_CONTROL_STD_ID;
    msg.dlc = 8U;
    msg.is_ext = 0U;
    msg.is_remote = 0U;
    msg.data[0] = (uint8_t)(raw_motor1 >> 8U);
    msg.data[1] = (uint8_t)(raw_motor1 & 0xFFU);
    msg.data[2] = (uint8_t)(raw_motor2 >> 8U);
    msg.data[3] = (uint8_t)(raw_motor2 & 0xFFU);

    return canio_send(&msg);
}
