#ifndef CAN_DEMO_GM6020_H
#define CAN_DEMO_GM6020_H

#include "canio.h"
#include "pid.h"

#include <stdint.h>

#define GM6020_ENCODER_RESOLUTION       8192
#define GM6020_ENCODER_HALF_RANGE       (GM6020_ENCODER_RESOLUTION / 2)
#define GM6020_FEEDBACK_STD_ID          0x206U
#define GM6020_CONTROL_STD_ID           0x1FE
#define GM6020_FEEDBACK_TIMEOUT_MS      100U
#define GM6020_TARGET_ANGLE_DEG         90.0f
#define GM6020_CONTROL_PERIOD_S         0.002f
#define GM6020_ANGLE_PID_OUTPUT_LIMIT   320.0f
#define GM6020_SPEED_COMMAND_LIMIT_RPM  200.0f
#define GM6020_CONTROL_OUTPUT_LIMIT     30000.0f

#define GM6020_ANGLE_KP                 0.0f
#define GM6020_ANGLE_KI                 0.0f
#define GM6020_ANGLE_KD                 0.0f
#define GM6020_SPEED_KP                 20.0f
#define GM6020_SPEED_KI                 0.5f
#define GM6020_SPEED_KD                 0.0f
typedef enum {
    GM6020_MODE_PROTECT = 0,
    GM6020_MODE_SPEED = 1,
    GM6020_MODE_POSITION = 2
} GM6020_ControlMode_t;

typedef struct {
    GM6020_ControlMode_t mode;
    float target_angle_deg;
    float relative_angle_deg;
    float target_speed_rpm;
    float speed_rpm;
    float control_output;
    uint8_t feedback_online;
} GM6020_Telemetry_t;

typedef struct {
    uint8_t motor_id;

    volatile uint8_t has_feedback;
    volatile uint16_t encoder;
    volatile uint16_t last_encoder;
    volatile uint16_t zero_encoder;
    volatile int16_t speed_rpm;
    volatile int16_t feedback_current;
    volatile uint8_t temperature;
    volatile int32_t total_encoder_counts;
    volatile float relative_angle;
    volatile uint32_t last_feedback_tick;

    volatile GM6020_ControlMode_t command_mode;
    volatile float command_angle_deg;
    volatile float command_speed_rpm;

    PID_t angle_pid;
    PID_t speed_pid;
    GM6020_ControlMode_t active_mode;
    float target_angle_deg;
    float target_speed_rpm;
    float control_output;
} GM6020_t;

extern GM6020_t motor[3];

void GM6020_Init(GM6020_t *gm6020, uint8_t motor_id);
void GM6020_ParseFeedback(GM6020_t *gm6020, const CanMsg_t *msg);
void GM6020_SetMode(GM6020_t *gm6020, GM6020_ControlMode_t mode);
void GM6020_SetTargetSpeed(GM6020_t *gm6020, float target_speed_rpm);
void GM6020_SetTargetAngle(GM6020_t *gm6020, float target_angle_deg);
int16_t GM6020_CalculateControl(GM6020_t *gm6020, uint32_t now_ms);
void GM6020_GetTelemetry(const GM6020_t *gm6020,
                         uint32_t now_ms,
                         GM6020_Telemetry_t *telemetry);
HAL_StatusTypeDef GM6020_SendMotor2Control(int16_t control_output);

#endif /* CAN_DEMO_GM6020_H */
