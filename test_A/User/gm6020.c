#include "gm6020.h"                                   /* 引入电机状态、参数和公开接口。 */

#include <math.h>                                      /* 提供 fmodf() 与 isfinite()。 */
#include <stddef.h>                                    /* 提供 NULL 空指针宏。 */
#include <string.h>                                    /* 提供 memset() 清零函数。 */

GM6020_t motor[3];                                     /* 保存电机实例，本工程控制 motor[2]。 */

static float GM6020_Clamp(float value, float limit)    /* 将控制量限制在对称区间内。 */
{
    if (value > limit) {                               /* 输入超过正向上限。 */
        return limit;                                  /* 返回正向上限。 */
    }
    if (value < -limit) {                              /* 输入低于负向下限。 */
        return -limit;                                 /* 返回负向下限。 */
    }
    return value;                                      /* 输入在允许范围内，原样返回。 */
}

static float GM6020_WrapTo180(float angle_deg)         /* 将角度归一化到 [-180°, 180°)。 */
{
    angle_deg = fmodf(angle_deg + 180.0f, 360.0f);     /* 先平移半圈，再对整圈取余。 */
    if (angle_deg < 0.0f) {                            /* C 语言负数取余结果可能为负。 */
        angle_deg += 360.0f;                           /* 将余数修正到 [0°, 360°)。 */
    }
    return angle_deg - 180.0f;                         /* 平移回目标区间。 */
}

void GM6020_Init(GM6020_t *gm6020, uint8_t motor_id)  /* 初始化一台 GM6020 的软件状态。 */
{
    if (gm6020 == NULL) {                              /* 防止解引用空指针。 */
        return;                                        /* 参数无效时不执行初始化。 */
    }

    memset(gm6020, 0, sizeof(*gm6020));                 /* 清零反馈、命令和控制器历史状态。 */
    gm6020->motor_id = motor_id;                        /* 记录软件侧电机编号。 */
    gm6020->command_mode = GM6020_MODE_PROTECT;         /* 上电默认请求保护模式。 */
    gm6020->active_mode = GM6020_MODE_PROTECT;          /* 当前模式同步设为保护模式。 */
    gm6020->command_angle_deg = GM6020_TARGET_ANGLE_DEG; /* 设置默认角度命令。 */
    gm6020->target_angle_deg = GM6020_TARGET_ANGLE_DEG; /* 设置控制器初始目标角度。 */

    /* 使用头文件中的台架参数初始化位置外环和速度内环。 */
    PID_Init(&gm6020->angle_pid,                        /* 选择位置外环 PID 实例。 */
             GM6020_ANGLE_KP,                          /* 传入位置环比例系数。 */
             GM6020_ANGLE_KI,                          /* 传入位置环积分系数。 */
             GM6020_ANGLE_KD,                          /* 传入位置环微分系数。 */
             GM6020_ANGLE_PID_OUTPUT_LIMIT,            /* 限制位置环积分。 */
             GM6020_ANGLE_PID_OUTPUT_LIMIT);           /* 限制位置环目标转速输出。 */
    PID_Init(&gm6020->speed_pid,                        /* 选择速度内环 PID 实例。 */
             GM6020_SPEED_KP,                          /* 传入速度环比例系数。 */
             GM6020_SPEED_KI,                          /* 传入速度环积分系数。 */
             GM6020_SPEED_KD,                          /* 传入速度环微分系数。 */
             GM6020_CONTROL_OUTPUT_LIMIT,              /* 限制速度环积分。 */
             GM6020_CONTROL_OUTPUT_LIMIT);             /* 限制最终电流指令。 */
}

void GM6020_SetMode(GM6020_t *gm6020, GM6020_ControlMode_t mode) /* 安全更新模式命令。 */
{
    uint32_t primask;                                  /* 保存进入临界区前的中断屏蔽状态。 */

    if ((gm6020 == NULL)                               /* 检查电机实例是否有效。 */
        || (mode < GM6020_MODE_PROTECT)                /* 拒绝低于枚举下界的模式。 */
        || (mode > GM6020_MODE_POSITION)) {            /* 拒绝高于枚举上界的模式。 */
        return;                                        /* 参数无效时保持原命令。 */
    }

    primask = __get_PRIMASK();                         /* 记录调用前是否已关闭中断。 */
    __disable_irq();                                   /* 进入临界区，避免任务和中断并发访问。 */
    gm6020->command_mode = mode;                       /* 写入新的模式命令。 */
    __DMB();                                           /* 确保写入在退出临界区前完成。 */
    if (primask == 0U) {                               /* 调用前中断原本处于开启状态。 */
        __enable_irq();                                /* 恢复中断响应。 */
    }
}

void GM6020_SetTargetSpeed(GM6020_t *gm6020, float target_speed_rpm) /* 安全更新速度命令。 */
{
    uint32_t primask;                                  /* 保存进入临界区前的中断状态。 */

    if ((gm6020 == NULL) || (!isfinite(target_speed_rpm))) { /* 检查指针及数值有限性。 */
        return;                                        /* 拒绝空指针、NaN 和无穷大。 */
    }

    target_speed_rpm = GM6020_Clamp(target_speed_rpm,  /* 对目标转速做对称限幅。 */
                                    GM6020_SPEED_COMMAND_LIMIT_RPM); /* 使用安全转速上限。 */
    primask = __get_PRIMASK();                         /* 记录调用前中断屏蔽状态。 */
    __disable_irq();                                   /* 进入临界区，保证浮点写入一致。 */
    gm6020->command_speed_rpm = target_speed_rpm;      /* 发布新的目标转速。 */
    __DMB();                                           /* 确保命令对控制任务可见。 */
    if (primask == 0U) {                               /* 调用前中断原本开启。 */
        __enable_irq();                                /* 恢复中断响应。 */
    }
}

void GM6020_SetTargetAngle(GM6020_t *gm6020, float target_angle_deg) /* 安全更新角度命令。 */
{
    uint32_t primask;                                  /* 保存进入临界区前的中断状态。 */

    if ((gm6020 == NULL) || (!isfinite(target_angle_deg))) { /* 检查指针及数值有限性。 */
        return;                                        /* 拒绝空指针、NaN 和无穷大。 */
    }

    target_angle_deg = GM6020_WrapTo180(target_angle_deg); /* 将目标角限制到标准角度区间。 */
    primask = __get_PRIMASK();                         /* 记录调用前中断屏蔽状态。 */
    __disable_irq();                                   /* 进入临界区，保证浮点写入一致。 */
    gm6020->command_angle_deg = target_angle_deg;      /* 发布新的目标角度。 */
    __DMB();                                           /* 确保命令对控制任务可见。 */
    if (primask == 0U) {                               /* 调用前中断原本开启。 */
        __enable_irq();                                /* 恢复中断响应。 */
    }
}

void GM6020_ParseFeedback(GM6020_t *gm6020, const CanMsg_t *msg) /* 解析并累计电机反馈。 */
{
    uint16_t encoder;                                  /* 本帧单圈编码器原始值。 */
    int16_t speed_rpm;                                 /* 本帧反馈转速。 */
    int16_t feedback_current;                          /* 本帧反馈转矩电流。 */
    int32_t encoder_delta;                             /* 相邻两帧的跨零修正后增量。 */

    if ((gm6020 == NULL) || (msg == NULL)) {           /* 检查电机实例和消息指针。 */
        return;                                        /* 参数无效时不更新状态。 */
    }
    if ((msg->is_ext != 0U)                            /* GM6020 反馈必须是标准帧。 */
        || (msg->is_remote != 0U)                      /* 反馈必须是数据帧。 */
        || (msg->dlc != 8U)                            /* 反馈数据长度必须为 8 字节。 */
        || (msg->id != GM6020_FEEDBACK_STD_ID)) {      /* 仅接受电机 2 的反馈 ID。 */
        return;                                        /* 忽略格式或来源不匹配的帧。 */
    }

    encoder = (uint16_t)(((uint16_t)msg->data[0] << 8U) | msg->data[1]); /* 合并大端编码器值。 */
    speed_rpm = (int16_t)(((uint16_t)msg->data[2] << 8U) | msg->data[3]); /* 合并大端有符号转速。 */
    feedback_current = (int16_t)(((uint16_t)msg->data[4] << 8U) | msg->data[5]); /* 合并反馈电流。 */

    if (gm6020->has_feedback == 0U) {                  /* 第一帧反馈用于建立软件零点。 */
        gm6020->zero_encoder = encoder;                /* 记录上电时的编码器位置。 */
        gm6020->last_encoder = encoder;                /* 初始化上一帧编码器值。 */
        gm6020->total_encoder_counts = 0;              /* 累计位移从零开始。 */
        gm6020->relative_angle = 0.0f;                 /* 相对角度从零度开始。 */
    } else {
        encoder_delta = (int32_t)encoder - (int32_t)gm6020->last_encoder; /* 计算单帧编码器差值。 */
        if (encoder_delta > GM6020_ENCODER_HALF_RANGE) { /* 正差超过半圈，说明反向跨过零点。 */
            encoder_delta -= GM6020_ENCODER_RESOLUTION; /* 修正为较小的负向位移。 */
        } else if (encoder_delta < -GM6020_ENCODER_HALF_RANGE) { /* 负差超过半圈，说明正向跨零。 */
            encoder_delta += GM6020_ENCODER_RESOLUTION; /* 修正为较小的正向位移。 */
        }

        gm6020->total_encoder_counts += encoder_delta; /* 将本帧位移累加到多圈计数。 */
        gm6020->relative_angle =                       /* 把累计计数换算为角度。 */
            ((float)gm6020->total_encoder_counts * 360.0f) /* 每整圈对应 360 度。 */
            / (float)GM6020_ENCODER_RESOLUTION;        /* 按每圈计数分辨率归一化。 */
        gm6020->last_encoder = encoder;                /* 保存本帧值供下次计算差分。 */
    }

    gm6020->encoder = encoder;                         /* 发布当前编码器值。 */
    gm6020->speed_rpm = speed_rpm;                     /* 发布当前反馈转速。 */
    gm6020->feedback_current = feedback_current;       /* 发布当前反馈电流。 */
    gm6020->temperature = msg->data[6];                /* 发布当前电机温度。 */
    gm6020->last_feedback_tick = HAL_GetTick();        /* 记录反馈到达时刻。 */
    __DMB();                                           /* 保证所有反馈字段先于有效标志可见。 */
    gm6020->has_feedback = 1U;                         /* 最后标记反馈数据已经有效。 */
}

int16_t GM6020_CalculateControl(GM6020_t *gm6020, uint32_t now_ms) /* 计算一次双环控制指令。 */
{
    uint32_t primask;                                  /* 保存进入临界区前的中断状态。 */
    uint8_t has_feedback;                              /* 反馈有效标志的本地快照。 */
    uint32_t last_feedback_tick;                       /* 最近反馈时刻的本地快照。 */
    GM6020_ControlMode_t command_mode;                  /* 模式命令的本地快照。 */
    float command_angle_deg;                           /* 角度命令的本地快照。 */
    float command_speed_rpm;                           /* 速度命令的本地快照。 */
    float relative_angle;                              /* 当前相对角度的本地快照。 */
    float speed_rpm;                                   /* 当前反馈转速的本地快照。 */
    float control_output;                              /* 本周期计算出的电流指令。 */

    if (gm6020 == NULL) {                              /* 防止解引用空指针。 */
        return 0;                                      /* 参数无效时输出安全零值。 */
    }

    /* 在短临界区内取得由 CAN 中断和通信任务共同更新的一致快照。 */
    primask = __get_PRIMASK();                         /* 记录调用前中断屏蔽状态。 */
    __disable_irq();                                   /* 暂停中断，避免读取一半更新的数据。 */
    has_feedback = gm6020->has_feedback;               /* 复制反馈有效标志。 */
    last_feedback_tick = gm6020->last_feedback_tick;   /* 复制最近反馈时刻。 */
    relative_angle = gm6020->relative_angle;           /* 复制当前累计角度。 */
    speed_rpm = (float)gm6020->speed_rpm;              /* 复制当前反馈转速并转为浮点。 */
    command_mode = gm6020->command_mode;               /* 复制最新模式命令。 */
    command_angle_deg = gm6020->command_angle_deg;     /* 复制最新角度命令。 */
    command_speed_rpm = gm6020->command_speed_rpm;     /* 复制最新速度命令。 */
    if (primask == 0U) {                               /* 调用前中断原本开启。 */
        __enable_irq();                                /* 完成快照后恢复中断。 */
    }

    if (command_mode != gm6020->active_mode) {         /* 检测控制模式切换。 */
        PID_Reset(&gm6020->angle_pid);                 /* 清除位置环历史，避免切换冲击。 */
        PID_Reset(&gm6020->speed_pid);                 /* 清除速度环历史，避免切换冲击。 */
        gm6020->active_mode = command_mode;            /* 使新模式正式生效。 */
    }

    if ((has_feedback == 0U)                           /* 尚未收到任何有效反馈。 */
        || ((uint32_t)(now_ms - last_feedback_tick) > GM6020_FEEDBACK_TIMEOUT_MS)) { /* 反馈已超时。 */
        PID_Reset(&gm6020->angle_pid);                 /* 清除可能失真的位置环状态。 */
        PID_Reset(&gm6020->speed_pid);                 /* 清除可能失真的速度环状态。 */
        gm6020->target_speed_rpm = 0.0f;               /* 将内部目标转速归零。 */
        gm6020->control_output = 0.0f;                 /* 将缓存控制输出归零。 */
        return 0;                                      /* 反馈异常时禁止电机出力。 */
    }

    if (command_mode == GM6020_MODE_PROTECT) {         /* 保护模式要求持续零输出。 */
        PID_Reset(&gm6020->angle_pid);                 /* 清除位置环积累。 */
        PID_Reset(&gm6020->speed_pid);                 /* 清除速度环积累。 */
        gm6020->target_angle_deg = relative_angle;     /* 将目标角跟随当前位置。 */
        gm6020->target_speed_rpm = 0.0f;               /* 将目标转速归零。 */
        gm6020->control_output = 0.0f;                 /* 将缓存控制输出归零。 */
        return 0;                                      /* 向电机发送零电流指令。 */
    }

    if (command_mode == GM6020_MODE_SPEED) {           /* 速度模式仅运行速度内环。 */
        gm6020->target_angle_deg = relative_angle;     /* 记录当前位置便于遥测显示。 */
        gm6020->target_speed_rpm = command_speed_rpm;  /* 直接采用上位机速度命令。 */
    } else {
        /* 位置模式选择与当前位置相差不超过 180° 的等效目标，避免绕远路。 */
        gm6020->target_angle_deg = relative_angle      /* 以当前累计角度为参考。 */
            + GM6020_WrapTo180(command_angle_deg - relative_angle); /* 加上最短方向角差。 */
        gm6020->target_speed_rpm = PID_Calculate(&gm6020->angle_pid, /* 位置外环生成目标转速。 */
                                                  gm6020->target_angle_deg, /* 位置环目标角度。 */
                                                  relative_angle,    /* 位置环反馈角度。 */
                                                  GM6020_CONTROL_PERIOD_S); /* 使用 2 ms 控制周期。 */
    }

    control_output = PID_Calculate(&gm6020->speed_pid, /* 速度内环生成电流指令。 */
                                   gm6020->target_speed_rpm, /* 速度环目标转速。 */
                                   speed_rpm,          /* 速度环反馈转速。 */
                                   GM6020_CONTROL_PERIOD_S); /* 使用 2 ms 控制周期。 */
    control_output = GM6020_Clamp(control_output, GM6020_CONTROL_OUTPUT_LIMIT); /* 再做执行器限幅。 */
    gm6020->control_output = control_output;           /* 保存输出供遥测任务读取。 */

    if (control_output >= 0.0f) {                      /* 正数需要加 0.5 后取整。 */
        return (int16_t)(control_output + 0.5f);       /* 四舍五入为正向 16 位指令。 */
    }
    return (int16_t)(control_output - 0.5f);           /* 四舍五入为负向 16 位指令。 */
}

void GM6020_GetTelemetry(const GM6020_t *gm6020,       /* 指定要读取的电机实例。 */
                         uint32_t now_ms,               /* 当前系统毫秒计数。 */
                         GM6020_Telemetry_t *telemetry) /* 输出遥测快照。 */
{
    uint32_t primask;                                  /* 保存进入临界区前的中断状态。 */
    uint8_t has_feedback;                              /* 反馈有效标志的本地快照。 */
    uint32_t last_feedback_tick;                       /* 最近反馈时刻的本地快照。 */

    if ((gm6020 == NULL) || (telemetry == NULL)) {     /* 校验输入与输出指针。 */
        return;                                        /* 参数无效时不写输出缓冲。 */
    }

    primask = __get_PRIMASK();                         /* 记录调用前中断屏蔽状态。 */
    __disable_irq();                                   /* 暂停中断以取得一致快照。 */
    has_feedback = gm6020->has_feedback;               /* 复制反馈有效标志。 */
    last_feedback_tick = gm6020->last_feedback_tick;   /* 复制最近反馈时刻。 */
    telemetry->mode = gm6020->active_mode;             /* 复制当前生效模式。 */
    telemetry->target_angle_deg = gm6020->target_angle_deg; /* 复制目标角度。 */
    telemetry->relative_angle_deg = gm6020->relative_angle; /* 复制反馈角度。 */
    telemetry->target_speed_rpm = gm6020->target_speed_rpm; /* 复制目标转速。 */
    telemetry->speed_rpm = (float)gm6020->speed_rpm;   /* 复制反馈转速并转为浮点。 */
    telemetry->control_output = gm6020->control_output; /* 复制电流控制输出。 */
    if (primask == 0U) {                               /* 调用前中断原本开启。 */
        __enable_irq();                                /* 快照完成后恢复中断。 */
    }

    telemetry->feedback_online =                       /* 根据有效标志和时效判断在线状态。 */
        ((has_feedback != 0U)                          /* 必须至少收到过一帧有效反馈。 */
         && ((uint32_t)(now_ms - last_feedback_tick)   /* 计算无符号毫秒时间差。 */
             <= GM6020_FEEDBACK_TIMEOUT_MS)) ? 1U : 0U; /* 未超时记为在线，否则离线。 */
}

HAL_StatusTypeDef GM6020_SendMotor2Control(int16_t control_output) /* 发送电机 2 的电流指令。 */
{
    CanMsg_t msg = {0};                                /* 创建并清零 8 字节 CAN 数据帧。 */
    uint16_t raw_output;                               /* 保存指令的 16 位原始补码。 */

    if (control_output > (int16_t)GM6020_CONTROL_OUTPUT_LIMIT) { /* 超过正向电流上限。 */
        control_output = (int16_t)GM6020_CONTROL_OUTPUT_LIMIT; /* 截断到正向上限。 */
    } else if (control_output < (int16_t)-GM6020_CONTROL_OUTPUT_LIMIT) { /* 低于负向下限。 */
        control_output = (int16_t)-GM6020_CONTROL_OUTPUT_LIMIT; /* 截断到负向下限。 */
    }
    raw_output = (uint16_t)control_output;              /* 保留有符号值的二进制补码位型。 */

    msg.id = GM6020_CONTROL_STD_ID;                     /* 使用 GM6020 控制帧标准 ID。 */
    msg.dlc = 8U;                                      /* 电机控制协议要求 8 字节数据。 */
    msg.is_ext = 0U;                                   /* 选择 11 位标准帧。 */
    msg.is_remote = 0U;                                /* 选择携带数据的数据帧。 */
    msg.data[2] = (uint8_t)(raw_output >> 8U);          /* 电机 2 指令高字节放在 data[2]。 */
    msg.data[3] = (uint8_t)(raw_output & 0xFFU);        /* 电机 2 指令低字节放在 data[3]。 */

    return canio_send(&msg);                           /* 提交 CAN 帧并返回 HAL 状态。 */
}
