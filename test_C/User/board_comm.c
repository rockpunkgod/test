#include "board_comm.h"

#include <math.h>
#include <string.h>

#define BOARD_COMM_PI 3.14159265358979323846f

typedef enum {
    BOARD_COMM_MSG_DEG_REQUEST = 1,
    BOARD_COMM_MSG_DEG_RESPONSE = 2,
    BOARD_COMM_MSG_RAD_REQUEST = 3,
    BOARD_COMM_MSG_RAD_RESPONSE = 4,
    BOARD_COMM_MSG_HELLO = 5,
    BOARD_COMM_MSG_HELLO_REPLY = 6,
    BOARD_COMM_MSG_HEARTBEAT = 7,
    BOARD_COMM_MSG_HEARTBEAT_REPLY = 8,
    BOARD_COMM_MSG_ASSIGN_B = 9,
    BOARD_COMM_MSG_ASSIGN_ACK = 10
} BoardComm_MessageType_t;

typedef union {
    float float_value;
    uint32_t uint_value;
} BoardComm_FloatBytes_t;

_Static_assert(sizeof(float) == 4U, "board protocol requires 32-bit float");

BoardComm_t board_comm;

static void BoardComm_WriteUint32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value >> 24);
    data[1] = (uint8_t)(value >> 16);
    data[2] = (uint8_t)(value >> 8);
    data[3] = (uint8_t)value;
}

static uint32_t BoardComm_ReadUint32(const uint8_t *data)
{
    return ((uint32_t)data[0] << 24)
         | ((uint32_t)data[1] << 16)
         | ((uint32_t)data[2] << 8)
         | (uint32_t)data[3];
}

static void BoardComm_WriteFloat(uint8_t *data, float value)
{
    BoardComm_FloatBytes_t bytes;

    bytes.float_value = value;
    BoardComm_WriteUint32(data, bytes.uint_value);
}

static float BoardComm_ReadFloat(const uint8_t *data)
{
    BoardComm_FloatBytes_t bytes;

    bytes.uint_value = BoardComm_ReadUint32(data);
    return bytes.float_value;
}

static HAL_StatusTypeDef BoardComm_Send(uint32_t can_id,
                                        BoardComm_MessageType_t type,
                                        uint8_t sequence,
                                        const uint8_t payload[4])
{
    CanMsg_t msg = {0};
    HAL_StatusTypeDef status;
    uint32_t primask;

    msg.id = can_id;
    msg.dlc = 8U;
    msg.is_ext = 0U;
    msg.is_remote = 0U;
    msg.data[0] = BOARD_COMM_PROTOCOL_VERSION;
    msg.data[1] = (uint8_t)type;
    msg.data[2] = sequence;
    msg.data[3] = 0U;
    if (payload != NULL) {
        memcpy(&msg.data[4], payload, 4U);
    }

    status = canio_send(&msg);

    primask = __get_PRIMASK();
    __disable_irq();
    if (status == HAL_OK) {
        ++board_comm.tx_count;
    } else {
        ++board_comm.error_count;
    }
    if (primask == 0U) {
        __enable_irq();
    }

    return status;
}

static uint8_t BoardComm_QueueRequest(BoardComm_MessageType_t type,
                                      BoardComm_Operation_t operation,
                                      float value)
{
    uint32_t primask;

    if (!isfinite(value)) {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (board_comm.role != BOARD_COMM_ROLE_A) {
        if (primask == 0U) {
            __enable_irq();
        }
        return 0U;
    }

    board_comm.tx_request_type = (uint8_t)type;
    board_comm.tx_request_value = value;
    board_comm.tx_request_pending = 1U;
    board_comm.last_operation = operation;
    board_comm.input_value = value;
    board_comm.output_value = 0.0f;
    board_comm.operation_complete = 0U;
    if (operation == BOARD_COMM_OPERATION_HANDSHAKE) {
        board_comm.handshake_ok = 0U;
    }
    if (primask == 0U) {
        __enable_irq();
    }

    return 1U;
}

void BoardComm_Init(void)
{
    memset(&board_comm, 0, sizeof(board_comm));
}

void BoardComm_SetRole(BoardComm_Role_t role)
{
    uint32_t primask;

    if ((role != BOARD_COMM_ROLE_NONE)
        && (role != BOARD_COMM_ROLE_A)) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    memset(&board_comm, 0, sizeof(board_comm));
    board_comm.role = role;
    board_comm.next_heartbeat_tick = HAL_GetTick();
    board_comm.next_role_assignment_tick = HAL_GetTick();
    if (primask == 0U) {
        __enable_irq();
    }
}

uint8_t BoardComm_RequestDegreeToRadian(float degree)
{
    return BoardComm_QueueRequest(BOARD_COMM_MSG_DEG_REQUEST,
                                  BOARD_COMM_OPERATION_DEG_TO_RAD,
                                  degree);
}

uint8_t BoardComm_RequestRadianToDegree(float radian)
{
    return BoardComm_QueueRequest(BOARD_COMM_MSG_RAD_REQUEST,
                                  BOARD_COMM_OPERATION_RAD_TO_DEG,
                                  radian);
}

uint8_t BoardComm_RequestHandshake(void)
{
    return BoardComm_QueueRequest(BOARD_COMM_MSG_HELLO,
                                  BOARD_COMM_OPERATION_HANDSHAKE,
                                  0.0f);
}

static void BoardComm_ProcessBoardA(uint32_t now_ms)
{
    uint8_t request_pending;
    uint8_t request_type = 0U;
    uint8_t sequence = 0U;
    float request_value = 0.0f;
    uint8_t payload[4] = {0};
    uint32_t primask;
    uint8_t role_assignment_ok;
    uint8_t peer_seen;
    uint32_t last_rx_tick;

    /* B板复位后会回到NONE；A板检测掉线后重新发角色分配。 */
    primask = __get_PRIMASK();
    __disable_irq();
    role_assignment_ok = board_comm.role_assignment_ok;
    peer_seen = board_comm.peer_seen;
    last_rx_tick = board_comm.last_rx_tick;
    if ((role_assignment_ok != 0U)
        && (peer_seen != 0U)
        && ((uint32_t)(now_ms - last_rx_tick) > BOARD_COMM_LINK_TIMEOUT_MS)) {
        board_comm.role_assignment_ok = 0U;
        board_comm.next_role_assignment_tick = now_ms;
        role_assignment_ok = 0U;
    }
    if (primask == 0U) {
        __enable_irq();
    }

    /* 未收到B板确认前，每100 ms发送一次角色分配帧。 */
    if ((role_assignment_ok == 0U)
        && ((int32_t)(now_ms - board_comm.next_role_assignment_tick) >= 0)) {
        sequence = board_comm.next_sequence++;
        payload[0] = 'S';
        payload[1] = 'E';
        payload[2] = 'T';
        payload[3] = 'B';
        if (BoardComm_Send(BOARD_COMM_CAN_ID_A_TO_B,
                           BOARD_COMM_MSG_ASSIGN_B,
                           sequence,
                           payload) == HAL_OK) {
            board_comm.role_assignment_sequence = sequence;
        }
        board_comm.next_role_assignment_tick = now_ms + BOARD_COMM_ROLE_RETRY_MS;
        memset(payload, 0, sizeof(payload));
    }

    primask = __get_PRIMASK();
    __disable_irq();
    request_pending = (board_comm.role_assignment_ok != 0U)
        ? board_comm.tx_request_pending : 0U;
    if (request_pending != 0U) {
        request_type = board_comm.tx_request_type;
        request_value = board_comm.tx_request_value;
        sequence = board_comm.next_sequence++;
        board_comm.expected_sequence = sequence;
        board_comm.last_sequence = sequence;
        board_comm.tx_request_pending = 0U;
    }
    if (primask == 0U) {
        __enable_irq();
    }

    if (request_pending != 0U) {
        if (request_type == (uint8_t)BOARD_COMM_MSG_HELLO) {
            payload[0] = 'H';
            payload[1] = 'I';
        } else {
            BoardComm_WriteFloat(payload, request_value);
        }

        if (BoardComm_Send(BOARD_COMM_CAN_ID_A_TO_B,
                           (BoardComm_MessageType_t)request_type,
                           sequence,
                           payload) != HAL_OK) {
            primask = __get_PRIMASK();
            __disable_irq();
            board_comm.tx_request_type = request_type;
            board_comm.tx_request_value = request_value;
            board_comm.tx_request_pending = 1U;
            if (primask == 0U) {
                __enable_irq();
            }
        }
    }

    if ((int32_t)(now_ms - board_comm.next_heartbeat_tick) >= 0) {
        sequence = board_comm.next_sequence++;
        BoardComm_WriteUint32(payload, now_ms);
        (void)BoardComm_Send(BOARD_COMM_CAN_ID_A_TO_B,
                             BOARD_COMM_MSG_HEARTBEAT,
                             sequence,
                             payload);
        board_comm.next_heartbeat_tick = now_ms + BOARD_COMM_HEARTBEAT_PERIOD_MS;
    }
}

static void BoardComm_ProcessBoardB(void)
{
    uint8_t request_pending;
    uint8_t request_type = 0U;
    uint8_t sequence = 0U;
    float input = 0.0f;
    float output = 0.0f;
    BoardComm_MessageType_t response_type = BOARD_COMM_MSG_HELLO_REPLY;
    uint8_t heartbeat_pending;
    uint8_t heartbeat_sequence;
    uint8_t role_reply_pending;
    uint8_t role_reply_sequence;
    uint8_t payload[4] = {0};
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    request_pending = board_comm.rx_request_pending;
    if (request_pending != 0U) {
        request_type = board_comm.rx_request_type;
        sequence = board_comm.rx_request_sequence;
        input = board_comm.rx_request_value;
        board_comm.rx_request_pending = 0U;
    }
    heartbeat_pending = board_comm.heartbeat_reply_pending;
    heartbeat_sequence = board_comm.heartbeat_reply_sequence;
    board_comm.heartbeat_reply_pending = 0U;
    role_reply_pending = board_comm.role_assignment_reply_pending;
    role_reply_sequence = board_comm.role_assignment_sequence;
    board_comm.role_assignment_reply_pending = 0U;
    if (primask == 0U) {
        __enable_irq();
    }

    if (request_pending != 0U) {
        if (request_type == (uint8_t)BOARD_COMM_MSG_DEG_REQUEST) {
            output = input * BOARD_COMM_PI / 180.0f;
            response_type = BOARD_COMM_MSG_DEG_RESPONSE;
            BoardComm_WriteFloat(payload, output);
            board_comm.last_operation = BOARD_COMM_OPERATION_DEG_TO_RAD;
        } else if (request_type == (uint8_t)BOARD_COMM_MSG_RAD_REQUEST) {
            output = input * 180.0f / BOARD_COMM_PI;
            response_type = BOARD_COMM_MSG_RAD_RESPONSE;
            BoardComm_WriteFloat(payload, output);
            board_comm.last_operation = BOARD_COMM_OPERATION_RAD_TO_DEG;
        } else {
            payload[0] = 'A';
            payload[1] = 'C';
            payload[2] = 'K';
            response_type = BOARD_COMM_MSG_HELLO_REPLY;
            board_comm.last_operation = BOARD_COMM_OPERATION_HANDSHAKE;
            board_comm.handshake_ok = 1U;
        }

        board_comm.input_value = input;
        board_comm.output_value = output;
        board_comm.last_sequence = sequence;
        if (BoardComm_Send(BOARD_COMM_CAN_ID_B_TO_A,
                           response_type,
                           sequence,
                           payload) == HAL_OK) {
            board_comm.operation_complete = 1U;
        }
    }

    if (role_reply_pending != 0U) {
        payload[0] = 'B';
        payload[1] = 'O';
        payload[2] = 'K';
        payload[3] = 0U;
        (void)BoardComm_Send(BOARD_COMM_CAN_ID_B_TO_A,
                             BOARD_COMM_MSG_ASSIGN_ACK,
                             role_reply_sequence,
                             payload);
    }

    if (heartbeat_pending != 0U) {
        memset(payload, 0, sizeof(payload));
        (void)BoardComm_Send(BOARD_COMM_CAN_ID_B_TO_A,
                             BOARD_COMM_MSG_HEARTBEAT_REPLY,
                             heartbeat_sequence,
                             payload);
    }
}

void BoardComm_Process(uint32_t now_ms)
{
    BoardComm_Role_t role = board_comm.role;

    if (role == BOARD_COMM_ROLE_A) {
        BoardComm_ProcessBoardA(now_ms);
    } else if (role == BOARD_COMM_ROLE_B) {
        BoardComm_ProcessBoardB();
    }
}

void BoardComm_OnCanReceived(const CanMsg_t *msg)
{
    BoardComm_Role_t role;
    BoardComm_MessageType_t type;
    float value;

    if ((msg == NULL) || (msg->is_ext != 0U) || (msg->is_remote != 0U)
        || (msg->dlc != 8U)
        || (msg->data[0] != BOARD_COMM_PROTOCOL_VERSION)) {
        return;
    }

    type = (BoardComm_MessageType_t)msg->data[1];
    role = board_comm.role;

    /* NONE状态只接受角色分配帧，避免误处理普通业务数据。 */
    if (role == BOARD_COMM_ROLE_NONE) {
        if ((msg->id == BOARD_COMM_CAN_ID_A_TO_B)
            && (type == BOARD_COMM_MSG_ASSIGN_B)
            && (msg->data[4] == 'S')
            && (msg->data[5] == 'E')
            && (msg->data[6] == 'T')
            && (msg->data[7] == 'B')) {
            board_comm.role = BOARD_COMM_ROLE_B;
            board_comm.role_assignment_ok = 1U;
            board_comm.role_assignment_sequence = msg->data[2];
            board_comm.role_assignment_reply_pending = 1U;
            board_comm.peer_seen = 1U;
            board_comm.last_rx_tick = HAL_GetTick();
            board_comm.last_sequence = msg->data[2];
            ++board_comm.rx_count;
        }
        return;
    }

    if (((role == BOARD_COMM_ROLE_A) && (msg->id != BOARD_COMM_CAN_ID_B_TO_A))
        || ((role == BOARD_COMM_ROLE_B) && (msg->id != BOARD_COMM_CAN_ID_A_TO_B))) {
        return;
    }

    board_comm.peer_seen = 1U;
    board_comm.last_rx_tick = HAL_GetTick();
    ++board_comm.rx_count;

    if (role == BOARD_COMM_ROLE_B) {
        if (type == BOARD_COMM_MSG_ASSIGN_B) {
            if ((msg->data[4] == 'S') && (msg->data[5] == 'E')
                && (msg->data[6] == 'T') && (msg->data[7] == 'B')) {
                board_comm.role_assignment_sequence = msg->data[2];
                board_comm.role_assignment_reply_pending = 1U;
                board_comm.role_assignment_ok = 1U;
            } else {
                ++board_comm.error_count;
            }
        } else if (type == BOARD_COMM_MSG_HEARTBEAT) {
            board_comm.heartbeat_reply_sequence = msg->data[2];
            board_comm.heartbeat_reply_pending = 1U;
        } else if ((type == BOARD_COMM_MSG_DEG_REQUEST)
                   || (type == BOARD_COMM_MSG_RAD_REQUEST)
                   || (type == BOARD_COMM_MSG_HELLO)) {
            if (type == BOARD_COMM_MSG_HELLO) {
                if ((msg->data[4] != 'H') || (msg->data[5] != 'I')) {
                    ++board_comm.error_count;
                    return;
                }
                value = 0.0f;
            } else {
                value = BoardComm_ReadFloat(&msg->data[4]);
                if (!isfinite(value)) {
                    ++board_comm.error_count;
                    return;
                }
            }
            if (board_comm.rx_request_pending != 0U) {
                /* 只保存一个业务请求；覆盖前记录一次拥塞错误。 */
                ++board_comm.error_count;
            }
            board_comm.rx_request_type = (uint8_t)type;
            board_comm.rx_request_sequence = msg->data[2];
            board_comm.rx_request_value = value;
            board_comm.rx_request_pending = 1U;
            board_comm.operation_complete = 0U;
            if (type == BOARD_COMM_MSG_HELLO) {
                board_comm.handshake_ok = 0U;
            }
        } else {
            ++board_comm.error_count;
        }
        return;
    }
    if (type == BOARD_COMM_MSG_ASSIGN_ACK) {
        if ((msg->data[2] == board_comm.role_assignment_sequence)
            && (msg->data[4] == 'B')
            && (msg->data[5] == 'O')
            && (msg->data[6] == 'K')) {
            board_comm.role_assignment_ok = 1U;
            board_comm.last_sequence = msg->data[2];
        } else {
            ++board_comm.error_count;
        }
        return;
    }

    if (type == BOARD_COMM_MSG_HEARTBEAT_REPLY) {
        return;
    }
    if (msg->data[2] != board_comm.expected_sequence) {
        ++board_comm.error_count;
        return;
    }

    if (((type == BOARD_COMM_MSG_DEG_RESPONSE)
         && (board_comm.last_operation != BOARD_COMM_OPERATION_DEG_TO_RAD))
        || ((type == BOARD_COMM_MSG_RAD_RESPONSE)
            && (board_comm.last_operation != BOARD_COMM_OPERATION_RAD_TO_DEG))
        || ((type == BOARD_COMM_MSG_HELLO_REPLY)
            && (board_comm.last_operation != BOARD_COMM_OPERATION_HANDSHAKE))) {
        ++board_comm.error_count;
        return;
    }

    if ((type == BOARD_COMM_MSG_DEG_RESPONSE)
        || (type == BOARD_COMM_MSG_RAD_RESPONSE)) {
        value = BoardComm_ReadFloat(&msg->data[4]);
        if (!isfinite(value)) {
            ++board_comm.error_count;
            return;
        }
        board_comm.output_value = value;
        board_comm.last_sequence = msg->data[2];
        board_comm.operation_complete = 1U;
    } else if (type == BOARD_COMM_MSG_HELLO_REPLY) {
        if ((msg->data[4] == 'A') && (msg->data[5] == 'C')
            && (msg->data[6] == 'K')) {
            board_comm.last_sequence = msg->data[2];
            board_comm.handshake_ok = 1U;
            board_comm.operation_complete = 1U;
        } else {
            ++board_comm.error_count;
        }
    } else {
        ++board_comm.error_count;
    }
}

void BoardComm_GetTelemetry(uint32_t now_ms,
                            BoardComm_Telemetry_t *telemetry)
{
    uint8_t peer_seen;
    uint32_t last_rx_tick;
    uint32_t primask;

    if (telemetry == NULL) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    telemetry->role = board_comm.role;
    telemetry->last_operation = board_comm.last_operation;
    telemetry->input_value = board_comm.input_value;
    telemetry->output_value = board_comm.output_value;
    telemetry->operation_complete = board_comm.operation_complete;
    telemetry->handshake_ok = board_comm.handshake_ok;
    telemetry->last_sequence = board_comm.last_sequence;
    telemetry->tx_count = board_comm.tx_count;
    telemetry->rx_count = board_comm.rx_count;
    telemetry->error_count = board_comm.error_count;
    telemetry->role_assignment_ok = board_comm.role_assignment_ok;
    peer_seen = board_comm.peer_seen;
    last_rx_tick = board_comm.last_rx_tick;
    if (primask == 0U) {
        __enable_irq();
    }

    telemetry->peer_online = ((peer_seen != 0U)
        && ((uint32_t)(now_ms - last_rx_tick) <= BOARD_COMM_LINK_TIMEOUT_MS))
        ? 1U : 0U;
}
