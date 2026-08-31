#include "pid.h"

#include <stddef.h>

static float PID_Clamp(float value, float limit)
{
    if (limit <= 0.0f) {
        return value;
    }
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

void PID_Init(PID_t *pid,
              float kp,
              float ki,
              float kd,
              float integral_limit,
              float output_limit)
{
    if (pid == NULL) {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
    PID_Reset(pid);
}

void PID_Reset(PID_t *pid)
{
    if (pid == NULL) {
        return;
    }

    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->output = 0.0f;
    pid->has_previous_error = 0U;
}

float PID_Calculate(PID_t *pid, float target, float feedback, float dt_s)
{
    float error;
    float derivative = 0.0f;

    if ((pid == NULL) || (dt_s <= 0.0f)) {
        return 0.0f;
    }

    error = target - feedback;

    if (pid->ki != 0.0f) {
        pid->integral += error * dt_s;
        pid->integral = PID_Clamp(pid->integral, pid->integral_limit);
    } else {
        pid->integral = 0.0f;
    }

    if (pid->has_previous_error != 0U) {
        derivative = (error - pid->previous_error) / dt_s;
    }

    pid->previous_error = error;
    pid->has_previous_error = 1U;
    pid->output = (pid->kp * error)
                + (pid->ki * pid->integral)
                + (pid->kd * derivative);
    pid->output = PID_Clamp(pid->output, pid->output_limit);

    return pid->output;
}
