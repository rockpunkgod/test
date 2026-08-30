#include "vofa.h"                                     /* 引入 VOFA 状态和公开接口。 */

#include <math.h>                                      /* 提供 isfinite() 数值检查。 */
#include <stdio.h>                                     /* 提供 snprintf() 格式化函数。 */
#include <stdlib.h>                                    /* 提供 strtof() 字符串转浮点函数。 */
#include <string.h>                                    /* 提供 memset() 与 memcpy()。 */

Vofa_t vofa;                                           /* 全局 VOFA 串口通信实例。 */

static uint8_t VOFA_ProcessCommand(GM6020_t *gm6020, char *line) /* 解析一条上位机文本命令。 */
{
    char *end;                                         /* 指向数值解析结束位置。 */
    float value;                                       /* 命令携带的浮点参数。 */

    if ((gm6020 == NULL) || (line == NULL) || (line[1] != ':')) { /* 校验指针及“字母:数值”格式。 */
        return 0U;                                     /* 格式无效，报告处理失败。 */
    }

    value = strtof(&line[2], &end);                    /* 从冒号后的字符解析浮点数。 */
    if ((end == &line[2]) || (!isfinite(value))) {     /* 拒绝无数字、NaN 和无穷大。 */
        return 0U;                                     /* 数值无效，报告处理失败。 */
    }
    while ((*end == ' ') || (*end == '\t')) {         /* 允许数值后存在空格或制表符。 */
        ++end;                                         /* 跳过尾部空白字符。 */
    }
    if (*end != '\0') {                               /* 检查数值后是否还有非法字符。 */
        return 0U;                                     /* 拒绝带多余内容的命令。 */
    }

    switch (line[0]) {                                 /* 按首字母选择命令类型。 */
        case 'M':                                      /* M:0/1/2 用于切换控制模式。 */
            if ((value == 0.0f) || (value == 1.0f) || (value == 2.0f)) { /* 仅接受有效枚举值。 */
                GM6020_SetMode(gm6020,                 /* 更新电机模式命令。 */
                               (GM6020_ControlMode_t)((uint8_t)value)); /* 浮点参数转为模式枚举。 */
                return 1U;                             /* 模式命令执行成功。 */
            }
            break;                                     /* 非法模式值进入统一失败返回。 */

        case 'S':                                      /* S:数值 用于设置目标转速。 */
            GM6020_SetTargetSpeed(gm6020, value);      /* 限幅并发布目标转速。 */
            return 1U;                                 /* 速度命令执行成功。 */

        case 'P':                                      /* P:数值 用于设置目标角度。 */
            GM6020_SetTargetAngle(gm6020, value);      /* 归一化并发布目标角度。 */
            return 1U;                                 /* 位置命令执行成功。 */

        default:                                       /* 未识别的命令字母。 */
            break;                                     /* 进入统一失败返回。 */
    }

    return 0U;                                         /* 报告命令未执行。 */
}

HAL_StatusTypeDef VOFA_Init(Vofa_t *instance, UART_HandleTypeDef *uart) /* 初始化 VOFA 通信。 */
{
    if ((instance == NULL) || (uart == NULL)) {        /* 校验实例和串口句柄。 */
        return HAL_ERROR;                              /* 参数无效时返回 HAL 错误。 */
    }

    memset(instance, 0, sizeof(*instance));             /* 清零接收缓冲、标志和统计计数。 */
    instance->uart = uart;                              /* 绑定用于 VOFA 通信的串口。 */
    return HAL_UART_Receive_IT(instance->uart, &instance->rx_byte, 1U); /* 启动首字节中断接收。 */
}

void VOFA_Process(Vofa_t *instance, GM6020_t *gm6020) /* 在任务上下文处理串口命令。 */
{
    char line[VOFA_COMMAND_MAX_LEN];                    /* 命令行的任务侧本地副本。 */
    uint32_t primask;                                  /* 保存进入临界区前的中断状态。 */
    uint8_t length = 0U;                               /* 待复制命令长度。 */
    uint8_t command_ready;                             /* 完整命令标志的本地快照。 */
    uint8_t safety_stop;                               /* 保护停机标志的本地快照。 */

    if ((instance == NULL) || (gm6020 == NULL)) {      /* 校验通信实例和电机实例。 */
        return;                                        /* 参数无效时不处理命令。 */
    }

    /* 在短临界区内复制一条完整命令，避免 UART 中断同时改写缓冲。 */
    primask = __get_PRIMASK();                         /* 记录调用前中断屏蔽状态。 */
    __disable_irq();                                   /* 暂停中断，取得一致的命令快照。 */
    safety_stop = instance->safety_stop_pending;       /* 复制待处理的保护停机请求。 */
    instance->safety_stop_pending = 0U;                /* 消费本次保护停机请求。 */
    command_ready = instance->command_ready;           /* 复制完整命令标志。 */
    if (command_ready != 0U) {                         /* 中断侧已有完整命令。 */
        length = instance->command_length;             /* 取得命令有效长度。 */
        memcpy(line, (const void *)instance->command_line, (size_t)length + 1U); /* 连同结尾零字符复制。 */
        instance->command_ready = 0U;                  /* 标记该命令已经被任务取走。 */
        instance->command_length = 0U;                 /* 为接收下一条命令重置长度。 */
    }
    if (primask == 0U) {                               /* 调用前中断原本开启。 */
        __enable_irq();                                /* 完成快照后恢复中断。 */
    }

    if (safety_stop != 0U) {                           /* UART 异常要求优先进入安全状态。 */
        GM6020_SetMode(gm6020, GM6020_MODE_PROTECT);   /* 将电机切换到零输出保护模式。 */
        return;                                        /* 本周期不再执行普通命令。 */
    }

    if (command_ready != 0U) {                         /* 有完整命令等待解析。 */
        if (VOFA_ProcessCommand(gm6020, line) != 0U) { /* 执行并检查解析结果。 */
            ++instance->command_ok_count;              /* 累计成功命令次数。 */
        } else {
            ++instance->command_error_count;           /* 累计格式或内容错误次数。 */
        }
    }
}

HAL_StatusTypeDef VOFA_SendTelemetry(Vofa_t *instance, /* 指定 VOFA 通信实例。 */
                                     const GM6020_t *gm6020, /* 指定遥测电机实例。 */
                                     uint32_t now_ms)  /* 格式化并发送一帧遥测。 */
{
    char buffer[160];                                  /* FireWater 文本帧发送缓冲。 */
    int length;                                        /* snprintf() 返回的文本长度。 */
    GM6020_Telemetry_t telemetry = {0};                /* 创建并清零电机遥测快照。 */

    if ((instance == NULL) || (instance->uart == NULL) || (gm6020 == NULL)) { /* 校验所需指针。 */
        return HAL_ERROR;                              /* 参数无效时不发送。 */
    }

    GM6020_GetTelemetry(gm6020, now_ms, &telemetry);   /* 取得一致的电机状态快照。 */
    if (telemetry.feedback_online == 0U) {             /* 反馈离线时不保留旧显示值。 */
        instance->display_speed_rpm = 0.0f;            /* 将显示转速清零。 */
        instance->display_speed_initialized = 0U;      /* 下次在线时重新初始化滤波器。 */
    } else if (instance->display_speed_initialized == 0U) { /* 在线后的第一帧有效转速。 */
        instance->display_speed_rpm = telemetry.speed_rpm; /* 用真实反馈初始化显示值。 */
        instance->display_speed_initialized = 1U;      /* 标记滤波器初值已经有效。 */
    } else {
        instance->display_speed_rpm +=                 /* 对显示转速执行一阶低通滤波。 */
            VOFA_DISPLAY_SPEED_ALPHA                   /* 新样本采用 0.2 的权重。 */
            * (telemetry.speed_rpm - instance->display_speed_rpm); /* 逐步逼近实际转速。 */
    }
    /* FireWater channels: target angle, angle, target speed, speed, mode,
     * control output and feedback online state. */
    length = snprintf(buffer,                           /* 将各遥测通道编码为一行文本。 */
                      sizeof(buffer),                   /* 限制写入不超过缓冲区容量。 */
                      "RM2027:%.3f,%.3f,%.3f,%.3f,%u,%.3f,%u\r\n", /* 定义 FireWater 帧格式。 */
                      (double)telemetry.target_angle_deg, /* 通道 1：目标角度。 */
                      (double)telemetry.relative_angle_deg, /* 通道 2：实际角度。 */
                      (double)telemetry.target_speed_rpm, /* 通道 3：目标转速。 */
                      (double)instance->display_speed_rpm, /* 通道 4：滤波后的实际转速。 */
                      (unsigned int)telemetry.mode,     /* 通道 5：当前控制模式。 */
                      (double)telemetry.control_output, /* 通道 6：电流控制输出。 */
                      (unsigned int)telemetry.feedback_online); /* 通道 7：反馈在线状态。 */
    if ((length <= 0) || ((size_t)length >= sizeof(buffer))) { /* 检查格式化失败或截断。 */
        return HAL_ERROR;                              /* 文本无效时不发送残缺帧。 */
    }

    return HAL_UART_Transmit(instance->uart,           /* 通过绑定串口阻塞发送。 */
                             (uint8_t *)buffer,         /* 发送已格式化的字节缓冲。 */
                             (uint16_t)length,          /* 只发送有效文本长度。 */
                             VOFA_TX_TIMEOUT_MS);      /* 应用 15 ms 发送超时。 */
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart) /* HAL 串口单字节接收完成回调。 */
{
    if ((vofa.uart == NULL) || (uart != vofa.uart)) {  /* 仅处理已绑定的 VOFA 串口。 */
        return;                                        /* 忽略其他串口的回调。 */
    }

    if (vofa.command_ready == 0U) {                    /* 上一条命令已被任务取走才接收新内容。 */
        if (vofa.discard_line != 0U) {                 /* 当前正在丢弃一条超长命令。 */
            if (vofa.rx_byte == '\n') {                /* 换行表示超长命令已经结束。 */
                vofa.discard_line = 0U;                /* 退出丢弃状态，等待下一条命令。 */
            }
        } else if (vofa.rx_byte == '\n') {             /* 正常命令以换行符结束。 */
            if (vofa.command_length > 0U) {            /* 忽略空命令行。 */
                vofa.command_line[vofa.command_length] = '\0'; /* 在文本末尾补字符串结束符。 */
                __DMB();                               /* 确保命令内容先于就绪标志可见。 */
                vofa.command_ready = 1U;               /* 通知任务有完整命令待处理。 */
            }
        } else if (vofa.rx_byte != '\r') {             /* 忽略 Windows 行尾中的回车符。 */
            if (vofa.command_length < (VOFA_COMMAND_MAX_LEN - 1U)) { /* 为结尾零字符预留空间。 */
                vofa.command_line[vofa.command_length] = (char)vofa.rx_byte; /* 追加当前接收字节。 */
                ++vofa.command_length;                 /* 更新命令有效长度。 */
            } else {
                vofa.command_length = 0U;              /* 缓冲溢出后丢弃已收内容。 */
                vofa.discard_line = 1U;                /* 丢弃后续字节直到换行。 */
                vofa.safety_stop_pending = 1U;         /* 请求任务切换到保护模式。 */
                ++vofa.rx_overflow_count;              /* 累计一次接收溢出。 */
            }
        }
    }

    if (HAL_UART_Receive_IT(vofa.uart, &vofa.rx_byte, 1U) != HAL_OK) { /* 重新挂起下一字节接收。 */
        vofa.safety_stop_pending = 1U;                 /* 重启失败时请求保护停机。 */
        ++vofa.uart_error_count;                       /* 累计一次 UART 错误。 */
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)  /* HAL 串口错误回调。 */
{
    if ((vofa.uart == NULL) || (uart != vofa.uart)) {  /* 仅处理已绑定的 VOFA 串口。 */
        return;                                        /* 忽略其他串口错误。 */
    }

    vofa.command_length = 0U;                          /* 丢弃发生错误时的残缺命令。 */
    vofa.discard_line = 1U;                            /* 丢弃当前行剩余字符直到换行。 */
    vofa.safety_stop_pending = 1U;                     /* 请求任务切换到保护模式。 */
    ++vofa.uart_error_count;                           /* 累计一次 UART 错误。 */
    (void)HAL_UART_Receive_IT(vofa.uart, &vofa.rx_byte, 1U); /* 尝试恢复单字节中断接收。 */
}
