/**
 * @file    canio.c
 * @brief   CAN 总线收发实现
 *          PB8=CAN1_RX, PB9=CAN1_TX, 波特率 1Mbps (APB1=42MHz, Pre=3, BS1=9TQ, BS2=4TQ)
 */

#include "canio.h"                                    /* 引入 CAN 消息类型与公开接口。 */
#include "gm6020.h"                                   /* 将接收反馈交给 GM6020 解析。 */
#include <string.h>                                    /* 提供 memcpy() 内存复制函数。 */

/* ---- 外部变量：CubeMX 生成的 CAN1 句柄 ---- */
extern CAN_HandleTypeDef hcan1;                        /* 引用 CubeMX 在 main.c 中创建的 CAN1 句柄。 */

/* ---- 模块级变量 ---- */
static CanMsg_t g_rx_msg;                    /* 接收缓冲 */
static volatile uint8_t g_rx_flag = 0U;      /* ISR 与任务共享的新消息标志 */

/* ================================================================
 * 初始化
 * ================================================================ */
void canio_init(void)                                  /* 配置滤波器、启动 CAN1 并开启接收中断。 */
{
    /* ---- 配置 CAN 滤波器 ---- */
    CAN_FilterTypeDef filter = {0};                    /* 创建并清零 HAL 滤波器配置。 */

    /* 滤波器组 0，分配给 FIFO0 */
    filter.FilterBank           = 0;                   /* 使用 CAN1 的第 0 号滤波器组。 */
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;    /* 掩码模式 */
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;    /* 32 位 */
    filter.FilterIdHigh         = 0x0000;                    /* 接收任意 ID */
    filter.FilterIdLow          = 0x0000;              /* 标识符低 16 位也设为零。 */
    filter.FilterMaskIdHigh     = 0x0000;                    /* 掩码全 0 → 不过滤 */
    filter.FilterMaskIdLow      = 0x0000;              /* 掩码低 16 位为零，全部不比较。 */
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;             /* 绑定 FIFO0 */
    filter.FilterActivation     = ENABLE;              /* 启用该滤波器配置。 */
    filter.SlaveStartFilterBank = 14;                        /* 仅 CAN1 使用 */

    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) { /* 将滤波器配置写入 CAN1。 */
        Error_Handler();                               /* 配置失败时进入统一错误处理。 */
    }

    /* ---- 启动 CAN 外设 ---- */
    if (HAL_CAN_Start(&hcan1) != HAL_OK) {             /* 将 CAN1 从初始化态切换到工作态。 */
        Error_Handler();                               /* 启动失败时进入统一错误处理。 */
    }

    /* ---- 使能 FIFO0 接收中断（消息挂起通知） ---- */
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) { /* 开启 FIFO0 新消息中断。 */
        Error_Handler();                               /* 中断启用失败时进入统一错误处理。 */
    }
}

/* ================================================================
 * 发送
 * ================================================================ */
HAL_StatusTypeDef canio_send(const CanMsg_t *msg)      /* 将统一消息结构转换为 HAL CAN 帧并提交。 */
{
    if (msg == NULL || msg->dlc > 8) {                 /* 校验消息指针和 CAN 最大数据长度。 */
        return HAL_ERROR;                              /* 参数无效时不访问发送邮箱。 */
    }

    CAN_TxHeaderTypeDef tx_header = {0};               /* 创建并清零 HAL 发送帧头。 */

    /* 构造发送帧头 */
    if (msg->is_ext) {                                 /* 根据消息标志选择扩展帧。 */
        tx_header.IDE   = CAN_ID_EXT;                  /* 使用 29 位扩展标识符。 */
        tx_header.ExtId = msg->id & 0x1FFFFFFFU;       /* 截取合法的 29 位扩展 ID。 */
    } else {
        tx_header.IDE   = CAN_ID_STD;                  /* 使用 11 位标准标识符。 */
        tx_header.StdId = msg->id & 0x7FF;             /* 截取合法的 11 位标准 ID。 */
    }

    tx_header.RTR    = msg->is_remote ? CAN_RTR_REMOTE : CAN_RTR_DATA; /* 选择远程帧或数据帧。 */
    tx_header.DLC    = msg->dlc;                       /* 写入有效数据字节数。 */
    tx_header.TransmitGlobalTime = DISABLE;            /* 不在帧中附加全局时间。 */

    /* 向当前空闲邮箱提交消息；HAL_CAN_AddTxMessage() 本身不等待发送完成。 */
    uint32_t tx_mailbox = 0;                           /* 接收 HAL 选择的发送邮箱编号。 */
    return HAL_CAN_AddTxMessage(&hcan1, &tx_header, msg->data, &tx_mailbox); /* 非阻塞提交发送请求。 */
}

/* ================================================================
 * 接收（轮询方式）
 * ================================================================ */
uint8_t canio_available(void)                          /* 查询软件接收缓冲是否有新帧。 */
{
    return g_rx_flag;                                  /* 返回由中断更新的新消息标志。 */
}

HAL_StatusTypeDef canio_receive(CanMsg_t *msg)         /* 原子复制一帧软件缓冲消息。 */
{
    uint32_t primask;                                  /* 保存进入临界区前的中断状态。 */

    if (msg == NULL) return HAL_ERROR;                 /* 输出指针无效时立即返回。 */

    /* 防止 CAN RX 中断在复制过程中改写共享缓冲区。 */
    primask = __get_PRIMASK();                         /* 记录调用前中断屏蔽状态。 */
    __disable_irq();                                   /* 暂停 CAN 中断，保护共享缓冲。 */

    if (g_rx_flag == 0) {                              /* 软件缓冲当前没有新消息。 */
        if (primask == 0U) {                           /* 调用前中断原本开启。 */
            __enable_irq();                            /* 返回前恢复中断。 */
        }
        return HAL_ERROR;   /* FIFO 空 */
    }

    /* 从模块缓冲复制到用户缓冲 */
    memcpy(msg, &g_rx_msg, sizeof(CanMsg_t));           /* 把共享消息复制到调用者缓冲。 */
    g_rx_flag = 0U;                                    /* 标记当前消息已被取走。 */

    if (primask == 0U) {                               /* 调用前中断原本开启。 */
        __enable_irq();                                /* 完成复制后恢复中断。 */
    }

    return HAL_OK;                                     /* 报告消息读取成功。 */
}

/* ================================================================
 * HAL 中断回调：CAN RX FIFO0 消息挂起
 * ================================================================ */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) /* HAL 的 FIFO0 新消息回调。 */
{
    if (hcan->Instance != CAN1) return;                 /* 忽略非 CAN1 外设触发的回调。 */

    CAN_RxHeaderTypeDef rx_header = {0};               /* 保存 HAL 返回的接收帧头。 */
    uint8_t             rx_data[8] = {0};              /* 保存最多 8 字节 CAN 数据。 */

    /* 从 FIFO0 读取一条消息 */
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) { /* 从硬件 FIFO 取出一帧。 */
        return;                                        /* 读取失败时不更新软件缓冲。 */
    }

    /* 填充接收结构体 */
    g_rx_msg.dlc     = rx_header.DLC;                   /* 保存本帧数据长度。 */
    g_rx_msg.is_ext  = (rx_header.IDE == CAN_ID_EXT) ? 1 : 0; /* 转换扩展帧标志。 */
    g_rx_msg.is_remote = (rx_header.RTR == CAN_RTR_REMOTE) ? 1 : 0; /* 转换远程帧标志。 */

    if (g_rx_msg.is_ext) {                             /* 扩展帧使用 29 位 ID 字段。 */
        g_rx_msg.id = rx_header.ExtId;                 /* 保存扩展标识符。 */
    } else {
        g_rx_msg.id = rx_header.StdId;                 /* 保存标准标识符。 */
    }

    memcpy(g_rx_msg.data, rx_data, g_rx_msg.dlc > 8 ? 8 : g_rx_msg.dlc); /* 复制合法长度的数据字段。 */

    g_rx_flag = 1;                                    /* 通知任务软件缓冲已有新消息。 */

    /* 调用用户回调 */
    canio_on_received(&g_rx_msg);                      /* 在中断上下文解析本工程电机反馈。 */
}

/* ================================================================
 * 用户回调：只解析本工程使用的 GM6020 电机 2 反馈
 * ================================================================ */
void canio_on_received(const CanMsg_t *msg)            /* 处理本工程关心的 CAN 消息。 */
{
    GM6020_ParseFeedback(&motor[2], msg);               /* 尝试按电机 2 协议解析反馈。 */
}
