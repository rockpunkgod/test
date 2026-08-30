#ifndef CAN_DEMO_GM6020_H                              /* 防止头文件被重复包含。 */
#define CAN_DEMO_GM6020_H                              /* 声明 GM6020 模块头文件保护宏。 */

#include "canio.h"                                    /* 使用 CAN 消息结构和发送接口。 */
#include "pid.h"                                      /* 使用位置环、速度环 PID 控制器。 */

#include <stdint.h>                                    /* 提供固定宽度整数类型。 */

#define GM6020_ENCODER_RESOLUTION       8192            /* 单圈编码器计数范围。 */
#define GM6020_ENCODER_HALF_RANGE       (GM6020_ENCODER_RESOLUTION / 2) /* 判断跨零点的半圈阈值。 */
#define GM6020_FEEDBACK_STD_ID          0x206U           /* 电机 2 的 CAN 反馈标准帧 ID。 */
#define GM6020_CONTROL_STD_ID           0x1FE            /* 发送 GM6020 电流指令的标准帧 ID。 */
#define GM6020_FEEDBACK_TIMEOUT_MS      100U             /* 超过此时间未反馈即判定离线。 */
#define GM6020_TARGET_ANGLE_DEG         90.0f            /* 上电后的默认位置目标。 */
#define GM6020_CONTROL_PERIOD_S         0.002f           /* 双环控制周期，单位秒。 */
#define GM6020_ANGLE_PID_OUTPUT_LIMIT   60.0f            /* 位置环输出的目标转速上限。 */
#define GM6020_SPEED_COMMAND_LIMIT_RPM  200.0f           /* 速度模式命令上限，单位 rpm。 */
#define GM6020_CONTROL_OUTPUT_LIMIT     10000.0f         /* 速度环输出的电流指令上限。 */

#define GM6020_ANGLE_KP                 20.0f            /* 位置环比例系数。 */
#define GM6020_ANGLE_KI                 0.0f             /* 位置环积分系数。 */
#define GM6020_ANGLE_KD                 0.0f             /* 位置环微分系数。 */
#define GM6020_SPEED_KP                 20.0f            /* 速度环比例系数。 */
#define GM6020_SPEED_KI                 5.0f             /* 速度环积分系数。 */
#define GM6020_SPEED_KD                 0.0f             /* 速度环微分系数。 */
typedef enum {
    GM6020_MODE_PROTECT = 0,                            /* 保护模式：输出为零。 */
    GM6020_MODE_SPEED = 1,                              /* 速度闭环模式。 */
    GM6020_MODE_POSITION = 2                            /* 位置外环 + 速度内环模式。 */
} GM6020_ControlMode_t;                                 /* 电机控制模式枚举。 */

typedef struct {
    GM6020_ControlMode_t mode;                          /* 当前生效的控制模式。 */
    float target_angle_deg;                            /* 当前目标角度，单位度。 */
    float relative_angle_deg;                          /* 当前累计相对角度，单位度。 */
    float target_speed_rpm;                            /* 当前目标转速，单位 rpm。 */
    float speed_rpm;                                   /* 当前反馈转速，单位 rpm。 */
    float control_output;                              /* 当前电流控制指令。 */
    uint8_t feedback_online;                           /* 1 表示反馈在线，0 表示离线。 */
} GM6020_Telemetry_t;                                  /* 提供给上位机的遥测快照。 */

typedef struct {
    uint8_t motor_id;                                  /* 软件侧电机编号。 */

    volatile uint8_t has_feedback;                     /* 是否收到过有效反馈。 */
    volatile uint16_t encoder;                         /* 当前单圈编码器值。 */
    volatile uint16_t last_encoder;                    /* 上一帧单圈编码器值。 */
    volatile uint16_t zero_encoder;                    /* 首帧反馈对应的软件零点。 */
    volatile int16_t speed_rpm;                        /* CAN 反馈转速，单位 rpm。 */
    volatile int16_t feedback_current;                 /* CAN 反馈转矩电流。 */
    volatile uint8_t temperature;                      /* CAN 反馈温度。 */
    volatile int32_t total_encoder_counts;             /* 跨圈累计编码器增量。 */
    volatile float relative_angle;                     /* 相对上电零点的累计角度。 */
    volatile uint32_t last_feedback_tick;              /* 最近有效反馈的系统毫秒计数。 */

    volatile GM6020_ControlMode_t command_mode;         /* 上位机请求的控制模式。 */
    volatile float command_angle_deg;                  /* 上位机请求的目标角度。 */
    volatile float command_speed_rpm;                  /* 上位机请求的目标转速。 */

    PID_t angle_pid;                                   /* 位置外环 PID 实例。 */
    PID_t speed_pid;                                   /* 速度内环 PID 实例。 */
    GM6020_ControlMode_t active_mode;                   /* 控制任务当前使用的模式。 */
    float target_angle_deg;                            /* 控制器实际采用的目标角度。 */
    float target_speed_rpm;                            /* 控制器实际采用的目标转速。 */
    float control_output;                              /* 最近一次计算的电流指令。 */
} GM6020_t;                                            /* 单台 GM6020 的完整运行状态。 */

extern GM6020_t motor[3];                              /* 全局电机实例表，本工程使用下标 1、2。 */

void GM6020_Init(GM6020_t *gm6020, uint8_t motor_id);  /* 初始化一台电机的软件状态。 */
void GM6020_ParseFeedback(GM6020_t *gm6020, const CanMsg_t *msg); /* 解析电机 2 的 CAN 反馈。 */
void GM6020_SetMode(GM6020_t *gm6020, GM6020_ControlMode_t mode); /* 原子更新控制模式命令。 */
void GM6020_SetTargetSpeed(GM6020_t *gm6020, float target_speed_rpm); /* 原子更新目标转速。 */
void GM6020_SetTargetAngle(GM6020_t *gm6020, float target_angle_deg); /* 原子更新目标角度。 */
int16_t GM6020_CalculateControl(GM6020_t *gm6020, uint32_t now_ms); /* 计算双环控制输出。 */
void GM6020_GetTelemetry(const GM6020_t *gm6020,       /* 读取目标电机。 */
                         uint32_t now_ms,               /* 当前系统毫秒计数。 */
                         GM6020_Telemetry_t *telemetry); /* 输出一致的遥测快照。 */
HAL_StatusTypeDef GM6020_SendMotor2Control(int16_t control_output); /* 发送电机 2 电流指令。 */

#endif /* CAN_DEMO_GM6020_H */                         /* 结束头文件保护。 */
