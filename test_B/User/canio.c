/**
 * @file    canio.c
 * @brief   CAN 总线收发实现
 *          PD0=CAN1_RX, PD1=CAN1_TX, 波特率 1Mbps (APB1=42MHz, Pre=3, BS1=9TQ, BS2=4TQ)
 */

#include "canio.h"
#include "gimbal.h"
#include <string.h>

/* ---- 外部变量：CubeMX 生成的 CAN1 句柄 ---- */
extern CAN_HandleTypeDef hcan1;

/* ---- 模块级变量 ---- */
static CanMsg_t g_rx_msg;                    /* 接收缓冲 */
static volatile uint8_t g_rx_flag = 0U;      /* ISR 与任务共享的新消息标志 */

/* ================================================================
 * 初始化
 * ================================================================ */
void canio_init(void)
{
    /* ---- 配置 CAN 滤波器 ---- */
    CAN_FilterTypeDef filter = {0};

    /* 滤波器组 0，分配给 FIFO0 */
    filter.FilterBank           = 0;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;    /* 掩码模式 */
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;    /* 32 位 */
    filter.FilterIdHigh         = 0x0000;                    /* 接收任意 ID */
    filter.FilterIdLow          = 0x0000;
    filter.FilterMaskIdHigh     = 0x0000;                    /* 掩码全 0 → 不过滤 */
    filter.FilterMaskIdLow      = 0x0000;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;             /* 绑定 FIFO0 */
    filter.FilterActivation     = ENABLE;
    filter.SlaveStartFilterBank = 14U;                        /* 仅 CAN1 使用 */

    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) {
        Error_Handler();
    }

    /* ---- 启动 CAN 外设 ---- */
    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        Error_Handler();
    }

    /* ---- 使能 FIFO0 接收中断（消息挂起通知） ---- */
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        Error_Handler();
    }
}

/* ================================================================
 * 发送
 * ================================================================ */
HAL_StatusTypeDef canio_send(const CanMsg_t *msg)
{
    if (msg == NULL || msg->dlc > 8) {
        return HAL_ERROR;
    }

    CAN_TxHeaderTypeDef tx_header = {0};

    /* 构造发送帧头 */
    if (msg->is_ext) {
        tx_header.IDE   = CAN_ID_EXT;
        tx_header.ExtId = msg->id & 0x1FFFFFFF;
    } else {
        tx_header.IDE   = CAN_ID_STD;
        tx_header.StdId = msg->id & 0x7FF;
    }

    tx_header.RTR    = msg->is_remote ? CAN_RTR_REMOTE : CAN_RTR_DATA;
    tx_header.DLC    = msg->dlc;
    tx_header.TransmitGlobalTime = DISABLE;

    /* 向当前空闲邮箱提交消息；HAL_CAN_AddTxMessage() 本身不等待发送完成。 */
    uint32_t tx_mailbox = 0;
    return HAL_CAN_AddTxMessage(&hcan1, &tx_header, msg->data, &tx_mailbox);
}

/* ================================================================
 * 接收（轮询方式）
 * ================================================================ */
uint8_t canio_available(void)
{
    return g_rx_flag;
}

HAL_StatusTypeDef canio_receive(CanMsg_t *msg)
{
    uint32_t primask;

    if (msg == NULL) return HAL_ERROR;

    /* 防止 CAN RX 中断在复制过程中改写共享缓冲区。 */
    primask = __get_PRIMASK();
    __disable_irq();

    if (g_rx_flag == 0) {
        if (primask == 0U) {
            __enable_irq();
        }
        return HAL_ERROR;   /* FIFO 空 */
    }

    /* 从模块缓冲复制到用户缓冲 */
    memcpy(msg, &g_rx_msg, sizeof(CanMsg_t));
    g_rx_flag = 0U;

    if (primask == 0U) {
        __enable_irq();
    }

    return HAL_OK;
}

/* ================================================================
 * HAL 中断回调：CAN RX FIFO0 消息挂起
 * ================================================================ */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance != CAN1) return;

    CAN_RxHeaderTypeDef rx_header = {0};
    uint8_t             rx_data[8] = {0};

    /* 从 FIFO0 读取一条消息 */
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
        return;
    }

    /* 填充接收结构体 */
    g_rx_msg.dlc     = rx_header.DLC;
    g_rx_msg.is_ext  = (rx_header.IDE == CAN_ID_EXT) ? 1 : 0;
    g_rx_msg.is_remote = (rx_header.RTR == CAN_RTR_REMOTE) ? 1 : 0;

    if (g_rx_msg.is_ext) {
        g_rx_msg.id = rx_header.ExtId;
    } else {
        g_rx_msg.id = rx_header.StdId;
    }

    memcpy(g_rx_msg.data, rx_data, g_rx_msg.dlc > 8 ? 8 : g_rx_msg.dlc);

    g_rx_flag = 1;  /* 标记有新消息 */

    /* 调用用户回调 */
    canio_on_received(&g_rx_msg);
}

/* ================================================================
 * 用户回调：把反馈交给Yaw/Pitch云台对象按CAN ID分流
 * ================================================================ */
void canio_on_received(const CanMsg_t *msg)
{
    Gimbal_ParseFeedback(msg);
}
