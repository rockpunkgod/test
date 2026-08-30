#ifndef CAN_DEMO_PID_H                                 /* 防止头文件被重复包含。 */
#define CAN_DEMO_PID_H                                 /* 声明 PID 模块头文件保护宏。 */

#include <stdint.h>                                    /* 提供固定宽度整数类型。 */

typedef struct {
    float kp;                                          /* 比例系数。 */
    float ki;                                          /* 积分系数。 */
    float kd;                                          /* 微分系数。 */
    float integral;                                    /* 已累计的误差积分。 */
    float previous_error;                              /* 上一控制周期的误差。 */
    float output;                                      /* 最近一次计算得到的输出。 */
    float integral_limit;                              /* 积分项绝对值上限。 */
    float output_limit;                                /* 最终输出绝对值上限。 */
    uint8_t has_previous_error;                        /* 上一周期误差是否有效。 */
} PID_t;                                               /* PID 控制器运行状态。 */

void PID_Init(PID_t *pid,                              /* 初始化目标 PID 实例。 */
              float kp,                                /* 比例系数。 */
              float ki,                                /* 积分系数。 */
              float kd,                                /* 微分系数。 */
              float integral_limit,                    /* 积分限幅。 */
              float output_limit);                     /* 输出限幅。 */
void PID_Reset(PID_t *pid);                            /* 清除 PID 动态状态。 */
float PID_Calculate(PID_t *pid, float target, float feedback, float dt_s); /* 计算一次 PID 输出。 */

#endif /* CAN_DEMO_PID_H */                            /* 结束头文件保护。 */
