/**
 * @file    canio.h
 * @brief   CAN 总线收发封装接口
 *          适用于 STM32F407 + HAL 库
 *          PB8=CAN1_RX, PB9=CAN1_TX, 波特率 1Mbps
 */

#ifndef CAN_DEMO_CANIO_H
#define CAN_DEMO_CANIO_H

#include "main.h"
#include <stdint.h>

/* ---- CAN 消息结构体 ---- */
typedef struct {
    uint32_t id;              /* CAN ID (标准帧 11bit 或扩展帧 29bit) */
    uint8_t  data[8];         /* 数据字段 (0-8 字节) */
    uint8_t  dlc;             /* 数据长度 (0-8) */
    uint8_t  is_ext;          /* 0=标准帧, 1=扩展帧 */
    uint8_t  is_remote;       /* 0=数据帧, 1=远程帧 */
} CanMsg_t;

/* ---- 公开接口 ---- */

/**
 * @brief  初始化 CAN1：配置滤波器、启动外设
 * @note   调用前需先确保 HAL_Init() 和 MX_CAN1_Init() 已完成
 */
void canio_init(void);

/**
 * @brief  向空闲邮箱提交一条 CAN 消息（非阻塞式）
 * @param  msg  指向待发送消息的指针
 * @retval HAL_OK / HAL_ERROR / HAL_TIMEOUT
 */
HAL_StatusTypeDef canio_send(const CanMsg_t *msg);

#endif /* CAN_DEMO_CANIO_H */
