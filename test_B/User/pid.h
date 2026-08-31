#ifndef CAN_DEMO_PID_H
#define CAN_DEMO_PID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float previous_error;
    float output;
    float integral_limit;
    float output_limit;
    uint8_t has_previous_error;
} PID_t;

void PID_Init(PID_t *pid,
              float kp,
              float ki,
              float kd,
              float integral_limit,
              float output_limit);
void PID_Reset(PID_t *pid);
float PID_Calculate(PID_t *pid, float target, float feedback, float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* CAN_DEMO_PID_H */
