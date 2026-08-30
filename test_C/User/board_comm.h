#ifndef CAN_DEMO_BOARD_COMM_H
#define CAN_DEMO_BOARD_COMM_H

#include "canio.h"

#include <stdint.h>

/* C题使用两个标准帧ID区分通信方向。 */
#define BOARD_COMM_CAN_ID_A_TO_B       0x301U
#define BOARD_COMM_CAN_ID_B_TO_A       0x302U
#define BOARD_COMM_PROTOCOL_VERSION    1U
#define BOARD_COMM_HEARTBEAT_PERIOD_MS 100U
#define BOARD_COMM_LINK_TIMEOUT_MS     300U
#define BOARD_COMM_ROLE_RETRY_MS       100U

typedef enum {
    BOARD_COMM_ROLE_NONE = 0,
    BOARD_COMM_ROLE_A = 1,
    BOARD_COMM_ROLE_B = 2
} BoardComm_Role_t;

typedef enum {
    BOARD_COMM_OPERATION_NONE = 0,
    BOARD_COMM_OPERATION_DEG_TO_RAD = 1,
    BOARD_COMM_OPERATION_RAD_TO_DEG = 2,
    BOARD_COMM_OPERATION_HANDSHAKE = 3
} BoardComm_Operation_t;

/*
 * 用结构体保存一块主板的全部通信状态，函数负责操作结构体。
 * 这就是本工程采用的C语言“面向对象”写法。
 */
typedef struct {
    volatile BoardComm_Role_t role;
    volatile BoardComm_Operation_t last_operation;
    volatile float input_value;
    volatile float output_value;
    volatile uint8_t operation_complete;
    volatile uint8_t handshake_ok;
    volatile uint8_t peer_seen;
    volatile uint8_t last_sequence;
    volatile uint32_t last_rx_tick;
    volatile uint32_t tx_count;
    volatile uint32_t rx_count;
    volatile uint32_t error_count;

    volatile uint8_t next_sequence;
    volatile uint8_t expected_sequence;
    volatile uint8_t tx_request_pending;
    volatile uint8_t tx_request_type;
    volatile float tx_request_value;

    volatile uint8_t rx_request_pending;
    volatile uint8_t rx_request_type;
    volatile uint8_t rx_request_sequence;
    volatile float rx_request_value;

    volatile uint8_t heartbeat_reply_pending;
    volatile uint8_t heartbeat_reply_sequence;
    volatile uint32_t next_heartbeat_tick;

    /* A板通过CAN给另一块未配置主板分配B角色。 */
    volatile uint8_t role_assignment_ok;
    volatile uint8_t role_assignment_reply_pending;
    volatile uint8_t role_assignment_sequence;
    volatile uint32_t next_role_assignment_tick;
} BoardComm_t;

typedef struct {
    BoardComm_Role_t role;
    BoardComm_Operation_t last_operation;
    float input_value;
    float output_value;
    uint8_t operation_complete;
    uint8_t handshake_ok;
    uint8_t peer_online;
    uint8_t last_sequence;
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t error_count;
    uint8_t role_assignment_ok;
} BoardComm_Telemetry_t;

extern BoardComm_t board_comm;

void BoardComm_Init(void);
void BoardComm_SetRole(BoardComm_Role_t role);
uint8_t BoardComm_RequestDegreeToRadian(float degree);
uint8_t BoardComm_RequestRadianToDegree(float radian);
uint8_t BoardComm_RequestHandshake(void);
void BoardComm_Process(uint32_t now_ms);
void BoardComm_OnCanReceived(const CanMsg_t *msg);
void BoardComm_GetTelemetry(uint32_t now_ms,
                            BoardComm_Telemetry_t *telemetry);

#endif /* CAN_DEMO_BOARD_COMM_H */
