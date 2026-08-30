/**
 * @file    canio.h
 * @brief   CAN 总线收发封装接口
 *          适用于 STM32F407 + HAL 库
 *          PB8=CAN1_RX, PB9=CAN1_TX, 波特率 1Mbps
 */

#ifndef CAN_DEMO_CANIO_H                               /* 防止头文件被重复包含。 */
#define CAN_DEMO_CANIO_H                               /* 声明 CAN 通信模块保护宏。 */

#include "main.h"                                     /* 使用 HAL CAN 类型和错误处理接口。 */
#include <stdint.h>                                    /* 提供固定宽度整数类型。 */

/* ---- CAN 消息结构体 ---- */
typedef struct {                                       /* 描述一帧标准或扩展 CAN 消息。 */
    uint32_t id;              /* CAN ID (标准帧 11bit 或扩展帧 29bit) */
    uint8_t  data[8];         /* 数据字段 (0-8 字节) */
    uint8_t  dlc;             /* 数据长度 (0-8) */
    uint8_t  is_ext;          /* 0=标准帧, 1=扩展帧 */
    uint8_t  is_remote;       /* 0=数据帧, 1=远程帧 */
} CanMsg_t;                                            /* CAN 收发的统一消息结构。 */

/* ---- 公开接口 ---- */

/**
 * @brief  初始化 CAN1：配置滤波器、启动外设
 * @note   调用前需先确保 HAL_Init() 和 MX_CAN1_Init() 已完成
 */
void canio_init(void);                                 /* 配置滤波器并启动 CAN1。 */

/**
 * @brief  向空闲邮箱提交一条 CAN 消息（非阻塞式）
 * @param  msg  指向待发送消息的指针
 * @retval HAL_OK / HAL_ERROR / HAL_TIMEOUT
 */
HAL_StatusTypeDef canio_send(const CanMsg_t *msg);     /* 非阻塞提交一帧 CAN 消息。 */

/**
 * @brief  判断接收 FIFO 中是否有新消息
 * @retval 1=有新消息, 0=无
 */
uint8_t canio_available(void);                         /* 查询模块缓冲中是否有新消息。 */

/**
 * @brief  从 FIFO 读取一条 CAN 消息（非阻塞）
 * @param  msg  指向存放消息的缓冲区
 * @retval HAL_OK=读取成功, HAL_ERROR=FIFO 为空
 */
HAL_StatusTypeDef canio_receive(CanMsg_t *msg);        /* 原子取出一帧已接收消息。 */

/**
 * @brief  CAN 接收中断回调（由 HAL_CAN_RxFifo0MsgPendingCallback 调用）
 * @note   用户可在此函数中处理接收到的消息，此函数在中断上下文中执行
 */
void canio_on_received(const CanMsg_t *msg);           /* 将有效反馈帧交给电机解析模块。 */

#endif /* CAN_DEMO_CANIO_H */                          /* 结束头文件保护。 */
