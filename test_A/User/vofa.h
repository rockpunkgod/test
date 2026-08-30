#ifndef CAN_DEMO_VOFA_H                                /* 防止头文件被重复包含。 */
#define CAN_DEMO_VOFA_H                                /* 声明 VOFA 模块头文件保护宏。 */
#define VOFA_DISPLAY_SPEED_ALPHA  0.2f                 /* 遥测转速低通滤波权重。 */
#include "gm6020.h"                                   /* 使用电机控制与遥测数据类型。 */

#include <stdint.h>                                    /* 提供固定宽度整数类型。 */

#define VOFA_COMMAND_MAX_LEN  64U                      /* 单条上位机命令缓冲区长度。 */
#define VOFA_TX_TIMEOUT_MS    15U                      /* 串口遥测发送超时时间。 */

typedef struct {
    UART_HandleTypeDef *uart;                           /* 绑定的 HAL UART 句柄。 */
    uint8_t rx_byte;                                   /* 中断方式接收的单字节缓冲。 */
    float display_speed_rpm;                           /* 低通滤波后的显示转速。 */
    uint8_t display_speed_initialized;                 /* 显示转速滤波器是否已初始化。 */
    volatile char command_line[VOFA_COMMAND_MAX_LEN];  /* 中断写入的命令行缓冲。 */
    volatile uint8_t command_length;                   /* 当前命令行已接收长度。 */
    volatile uint8_t command_ready;                    /* 完整命令等待任务处理标志。 */
    volatile uint8_t discard_line;                     /* 超长命令丢弃至换行标志。 */
    volatile uint8_t safety_stop_pending;              /* 待执行保护停机标志。 */
    volatile uint32_t rx_overflow_count;               /* 接收命令溢出次数。 */
    volatile uint32_t command_error_count;             /* 格式错误命令累计次数。 */
    volatile uint32_t command_ok_count;                /* 成功执行命令累计次数。 */
    volatile uint32_t uart_error_count;                /* UART 异常累计次数。 */
} Vofa_t;                                              /* VOFA 串口通信运行状态。 */

extern Vofa_t vofa;                                    /* 全局 VOFA 通信实例。 */

HAL_StatusTypeDef VOFA_Init(Vofa_t *instance, UART_HandleTypeDef *uart); /* 初始化串口接收。 */
void VOFA_Process(Vofa_t *instance, GM6020_t *gm6020); /* 在任务上下文处理已接收命令。 */
HAL_StatusTypeDef VOFA_SendTelemetry(Vofa_t *instance, /* 指定通信实例。 */
                                     const GM6020_t *gm6020, /* 指定遥测电机。 */
                                     uint32_t now_ms); /* 格式化并发送一帧遥测数据。 */

#endif /* CAN_DEMO_VOFA_H */                           /* 结束头文件保护。 */
