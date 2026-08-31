#ifndef CAN_DEMO_GIMBAL_H
#define CAN_DEMO_GIMBAL_H

#include "gm6020.h"

#include <stdint.h>

#define GIMBAL_YAW_MOTOR_ID          1U
#define GIMBAL_PITCH_MOTOR_ID        2U

#define GIMBAL_YAW_MIN_ANGLE_DEG    -100000000000.0f
#define GIMBAL_YAW_MAX_ANGLE_DEG     100000000000.0f
#define GIMBAL_YAW_MAX_SPEED_RPM    200.0f

#define GIMBAL_PITCH_MIN_ANGLE_DEG  0.0f
#define GIMBAL_PITCH_MAX_ANGLE_DEG   -100000000000.0f
#define GIMBAL_PITCH_MAX_SPEED_RPM   100000000000.0f

typedef struct {
    GM6020_ControlMode_t mode;
    GM6020_Telemetry_t yaw;
    GM6020_Telemetry_t pitch;
} Gimbal_Telemetry_t;

#ifdef __cplusplus

class Gimbal {
public:
    void Init(GM6020_t *yaw_motor, GM6020_t *pitch_motor);
    void ParseFeedback(const CanMsg_t *msg);
    void SetMode(GM6020_ControlMode_t mode);
    void SetYawSpeed(float speed_rpm);
    void SetPitchSpeed(float speed_rpm);
    void SetYawAngle(float angle_deg);
    void SetPitchAngle(float angle_deg);
    HAL_StatusTypeDef Control(uint32_t now_ms);
    void GetTelemetry(uint32_t now_ms, Gimbal_Telemetry_t *telemetry) const;

private:
    bool IsReady() const;

    GM6020_t *yaw_motor_;
    GM6020_t *pitch_motor_;
};

extern "C" {
#endif

void Gimbal_Init(void);
void Gimbal_ParseFeedback(const CanMsg_t *msg);
void Gimbal_SetMode(GM6020_ControlMode_t mode);
void Gimbal_SetYawSpeed(float speed_rpm);
void Gimbal_SetPitchSpeed(float speed_rpm);
void Gimbal_SetYawAngle(float angle_deg);
void Gimbal_SetPitchAngle(float angle_deg);
HAL_StatusTypeDef Gimbal_Control(uint32_t now_ms);
void Gimbal_GetTelemetry(uint32_t now_ms, Gimbal_Telemetry_t *telemetry);

#ifdef __cplusplus
}
#endif

#endif /* CAN_DEMO_GIMBAL_H */
