/**
 * @file    canio.c
 * @brief   CAN 总线收发实现
 *          PB8=CAN1_RX, PB9=CAN1_TX, 波特率 1Mbps (APB1=42MHz, Pre=3, BS1=9TQ, BS2=4TQ)
 */

#include "canio.h"
#include "gm6020.h"
#include <string.h>

extern CAN_HandleTypeDef hcan1;

static CanMsg_t g_rx_msg;
static volatile uint8_t g_rx_flag = 0U;  /* 由 CAN 接收中断更新。 */

void canio_init(void)
{
    CAN_FilterTypeDef filter = {0};

    /* 接收所有 CAN 帧，具体 ID 由电机反馈解析函数筛选。 */
    filter.FilterBank           = 0;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh         = 0x0000;
    filter.FilterIdLow          = 0x0000;
    filter.FilterMaskIdHigh     = 0x0000;
    filter.FilterMaskIdLow      = 0x0000;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation     = ENABLE;
    filter.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        Error_Handler();
    }
}

HAL_StatusTypeDef canio_send(const CanMsg_t *msg)
{
    if (msg == NULL || msg->dlc > 8) {
        return HAL_ERROR;
    }

    CAN_TxHeaderTypeDef tx_header = {0};

    if (msg->is_ext) {
        tx_header.IDE   = CAN_ID_EXT;
        tx_header.ExtId = msg->id & 0x1FFFFFFFU;
    } else {
        tx_header.IDE   = CAN_ID_STD;
        tx_header.StdId = msg->id & 0x7FF;
    }

    tx_header.RTR    = msg->is_remote ? CAN_RTR_REMOTE : CAN_RTR_DATA;
    tx_header.DLC    = msg->dlc;
    tx_header.TransmitGlobalTime = DISABLE;

    /* HAL 只把报文放入发送邮箱，此函数不会等待总线发送完成。 */
    uint32_t tx_mailbox = 0;
    return HAL_CAN_AddTxMessage(&hcan1, &tx_header, msg->data, &tx_mailbox);
}

uint8_t canio_available(void)
{
    return g_rx_flag;
}

HAL_StatusTypeDef canio_receive(CanMsg_t *msg)
{
    uint32_t primask;

    if (msg == NULL) return HAL_ERROR;

    /* 复制中断共享缓冲区时保持数据一致。 */
    primask = __get_PRIMASK();
    __disable_irq();

    if (g_rx_flag == 0) {
        if (primask == 0U) {
            __enable_irq();
        }
        return HAL_ERROR;
    }

    memcpy(msg, &g_rx_msg, sizeof(CanMsg_t));
    g_rx_flag = 0U;

    if (primask == 0U) {
        __enable_irq();
    }

    return HAL_OK;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance != CAN1) return;

    CAN_RxHeaderTypeDef rx_header = {0};
    uint8_t             rx_data[8] = {0};

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
        return;
    }

    g_rx_msg.dlc     = rx_header.DLC;
    g_rx_msg.is_ext  = (rx_header.IDE == CAN_ID_EXT) ? 1 : 0;
    g_rx_msg.is_remote = (rx_header.RTR == CAN_RTR_REMOTE) ? 1 : 0;

    if (g_rx_msg.is_ext) {
        g_rx_msg.id = rx_header.ExtId;
    } else {
        g_rx_msg.id = rx_header.StdId;
    }

    memcpy(g_rx_msg.data, rx_data, g_rx_msg.dlc > 8 ? 8 : g_rx_msg.dlc);

    g_rx_flag = 1;
    canio_on_received(&g_rx_msg);
}

void canio_on_received(const CanMsg_t *msg)
{
    /* A 题只控制电机 ID 2。 */
    GM6020_ParseFeedback(&motor[2], msg);
}
