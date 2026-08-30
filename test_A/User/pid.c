#include "pid.h"                                      /* 引入 PID 数据结构与公开接口。 */

#include <stddef.h>                                    /* 提供 NULL 空指针宏。 */

static float PID_Clamp(float value, float limit)       /* 将数值限制在对称区间 [-limit, limit]。 */
{
    if (limit <= 0.0f) {                               /* 非正限幅值表示不启用限幅。 */
        return value;                                  /* 原样返回输入值。 */
    }
    if (value > limit) {                               /* 输入超过正向上限。 */
        return limit;                                  /* 输出正向上限。 */
    }
    if (value < -limit) {                              /* 输入低于负向下限。 */
        return -limit;                                 /* 输出负向下限。 */
    }
    return value;                                      /* 输入在允许范围内，直接返回。 */
}

void PID_Init(PID_t *pid,                              /* 待初始化的 PID 实例。 */
              float kp,                                /* 比例系数。 */
              float ki,                                /* 积分系数。 */
              float kd,                                /* 微分系数。 */
              float integral_limit,                    /* 积分项绝对值上限。 */
              float output_limit)                      /* PID 输出绝对值上限。 */
{
    if (pid == NULL) {                                 /* 防止解引用空指针。 */
        return;                                        /* 参数无效时不执行初始化。 */
    }

    pid->kp = kp;                                      /* 保存比例系数。 */
    pid->ki = ki;                                      /* 保存积分系数。 */
    pid->kd = kd;                                      /* 保存微分系数。 */
    pid->integral_limit = integral_limit;              /* 保存积分限幅。 */
    pid->output_limit = output_limit;                  /* 保存输出限幅。 */
    PID_Reset(pid);                                    /* 清零历史状态，避免继承旧误差。 */
}

void PID_Reset(PID_t *pid)                             /* 复位 PID 的动态状态。 */
{
    if (pid == NULL) {                                 /* 防止解引用空指针。 */
        return;                                        /* 参数无效时直接返回。 */
    }

    pid->integral = 0.0f;                              /* 清零误差积分。 */
    pid->previous_error = 0.0f;                        /* 清零上一周期误差。 */
    pid->output = 0.0f;                                /* 清零缓存输出。 */
    pid->has_previous_error = 0U;                      /* 标记尚无可用于微分的历史误差。 */
}

float PID_Calculate(PID_t *pid, float target, float feedback, float dt_s) /* 计算一次离散 PID 输出。 */
{
    float error;                                       /* 当前目标与反馈的偏差。 */
    float derivative = 0.0f;                          /* 当前误差变化率，首次计算默认为零。 */

    if ((pid == NULL) || (dt_s <= 0.0f)) {             /* 校验实例指针和采样周期。 */
        return 0.0f;                                   /* 无法可靠计算时输出安全零值。 */
    }

    error = target - feedback;                        /* 按“目标减反馈”得到控制误差。 */

    if (pid->ki != 0.0f) {                             /* 仅在启用积分项时累加积分。 */
        pid->integral += error * dt_s;                 /* 用矩形法对误差进行离散积分。 */
        pid->integral = PID_Clamp(pid->integral, pid->integral_limit); /* 抑制积分饱和。 */
    } else {
        pid->integral = 0.0f;                          /* 关闭积分时同步清除历史积累。 */
    }

    if (pid->has_previous_error != 0U) {               /* 已有上一周期误差时才计算微分。 */
        derivative = (error - pid->previous_error) / dt_s; /* 用一阶差分估算误差变化率。 */
    }

    pid->previous_error = error;                       /* 保存本周期误差供下次计算。 */
    pid->has_previous_error = 1U;                      /* 标记微分历史数据已经有效。 */
    pid->output = (pid->kp * error)                    /* 计算比例项。 */
                + (pid->ki * pid->integral)            /* 叠加积分项。 */
                + (pid->kd * derivative);              /* 叠加微分项。 */
    pid->output = PID_Clamp(pid->output, pid->output_limit); /* 限制最终执行器指令。 */

    return pid->output;                                /* 返回本周期 PID 控制量。 */
}
