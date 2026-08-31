#include "gimbal.h"

#include <cstring>

static Gimbal g_gimbal;

bool Gimbal::IsReady() const
{
    return (yaw_motor_ != nullptr) && (pitch_motor_ != nullptr);
}

void Gimbal::Init(GM6020_t *yaw_motor, GM6020_t *pitch_motor)
{
    yaw_motor_ = yaw_motor;
    pitch_motor_ = pitch_motor;

    if (!IsReady()) {
        return;
    }

    GM6020_Init(yaw_motor_, GIMBAL_YAW_MOTOR_ID);
    GM6020_Init(pitch_motor_, GIMBAL_PITCH_MOTOR_ID);

    GM6020_SetSoftwareLimits(yaw_motor_,
                             GIMBAL_YAW_MIN_ANGLE_DEG,
                             GIMBAL_YAW_MAX_ANGLE_DEG,
                             GIMBAL_YAW_MAX_SPEED_RPM);
    GM6020_SetSoftwareLimits(pitch_motor_,
                             GIMBAL_PITCH_MIN_ANGLE_DEG,
                             GIMBAL_PITCH_MAX_ANGLE_DEG,
                             GIMBAL_PITCH_MAX_SPEED_RPM);

    GM6020_SetTargetAngle(yaw_motor_, 0.0f);
    GM6020_SetTargetAngle(pitch_motor_, 0.0f);
    GM6020_SetTargetSpeed(yaw_motor_, 0.0f);
    GM6020_SetTargetSpeed(pitch_motor_, 0.0f);
    SetMode(GM6020_MODE_PROTECT);
}

void Gimbal::ParseFeedback(const CanMsg_t *msg)
{
    if (!IsReady() || (msg == nullptr)) {
        return;
    }

    /* Each motor object checks its own feedback CAN ID. */
    GM6020_ParseFeedback(yaw_motor_, msg);
    GM6020_ParseFeedback(pitch_motor_, msg);
}

void Gimbal::SetMode(GM6020_ControlMode_t mode)
{
    uint32_t primask;

    if (!IsReady()) {
        return;
    }

    /* Keep both axes in the same mode even if a task switch is pending. */
    primask = __get_PRIMASK();
    __disable_irq();
    GM6020_SetMode(yaw_motor_, mode);
    GM6020_SetMode(pitch_motor_, mode);
    __DMB();
    if (primask == 0U) {
        __enable_irq();
    }
}

void Gimbal::SetYawSpeed(float speed_rpm)
{
    if (IsReady()) {
        GM6020_SetTargetSpeed(yaw_motor_, speed_rpm);
    }
}

void Gimbal::SetPitchSpeed(float speed_rpm)
{
    if (IsReady()) {
        GM6020_SetTargetSpeed(pitch_motor_, speed_rpm);
    }
}

void Gimbal::SetYawAngle(float angle_deg)
{
    if (IsReady()) {
        GM6020_SetTargetAngle(yaw_motor_, angle_deg);
    }
}

void Gimbal::SetPitchAngle(float angle_deg)
{
    if (IsReady()) {
        GM6020_SetTargetAngle(pitch_motor_, angle_deg);
    }
}

HAL_StatusTypeDef Gimbal::Control(uint32_t now_ms)
{
    int16_t yaw_output;
    int16_t pitch_output;

    if (!IsReady()) {
        return HAL_ERROR;
    }

    yaw_output = GM6020_CalculateControl(yaw_motor_, now_ms);
    pitch_output = GM6020_CalculateControl(pitch_motor_, now_ms);
    return GM6020_SendGimbalControl(yaw_output,pitch_output);
}

void Gimbal::GetTelemetry(uint32_t now_ms,
                          Gimbal_Telemetry_t *telemetry) const
{
    if (telemetry == nullptr) {
        return;
    }

    std::memset(telemetry, 0, sizeof(*telemetry));
    if (!IsReady()) {
        return;
    }

    GM6020_GetTelemetry(yaw_motor_, now_ms, &telemetry->yaw);
    GM6020_GetTelemetry(pitch_motor_, now_ms, &telemetry->pitch);
    telemetry->mode = telemetry->yaw.mode;
}

extern "C" void Gimbal_Init(void)
{
    g_gimbal.Init(&motor[GIMBAL_YAW_MOTOR_ID],
                  &motor[GIMBAL_PITCH_MOTOR_ID]);
}

extern "C" void Gimbal_ParseFeedback(const CanMsg_t *msg)
{
    g_gimbal.ParseFeedback(msg);
}

extern "C" void Gimbal_SetMode(GM6020_ControlMode_t mode)
{
    g_gimbal.SetMode(mode);
}

extern "C" void Gimbal_SetYawSpeed(float speed_rpm)
{
    g_gimbal.SetYawSpeed(speed_rpm);
}

extern "C" void Gimbal_SetPitchSpeed(float speed_rpm)
{
    g_gimbal.SetPitchSpeed(speed_rpm);
}

extern "C" void Gimbal_SetYawAngle(float angle_deg)
{
    g_gimbal.SetYawAngle(angle_deg);
}

extern "C" void Gimbal_SetPitchAngle(float angle_deg)
{
    g_gimbal.SetPitchAngle(angle_deg);
}

extern "C" HAL_StatusTypeDef Gimbal_Control(uint32_t now_ms)
{
    return g_gimbal.Control(now_ms);
}

extern "C" void Gimbal_GetTelemetry(uint32_t now_ms,
                                     Gimbal_Telemetry_t *telemetry)
{
    g_gimbal.GetTelemetry(now_ms, telemetry);
}
