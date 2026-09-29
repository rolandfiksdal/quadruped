/* Runs the real CAN/joint modules against a fake HAL, without hardware. */
#include "robot_joints.h"
#include <string.h>

#define CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)

FDCAN_HandleTypeDef hfdcan1 = {.Instance = FDCAN1};
static uint32_t now_ms;
static uint32_t tx_count;
static uint8_t rx_pending;
static FDCAN_RxHeaderTypeDef rx_header;
static uint8_t rx_data[64];

uint32_t HAL_GetTick(void) { return now_ms; }
HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *h,
                                       const FDCAN_FilterTypeDef *f) { return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *h) { return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *h,
                                               uint32_t a, uint32_t b) { return HAL_OK; }
uint32_t HAL_FDCAN_GetRxFifoFillLevel(const FDCAN_HandleTypeDef *h,
                                    uint32_t fifo) { return rx_pending; }
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *h, uint32_t fifo,
                                        FDCAN_RxHeaderTypeDef *header, uint8_t *data)
{
    *header = rx_header;
    /* Exercise the production receive buffer with an FD-sized payload too. */
    memcpy(data, rx_data, rx_header.DataLength == FDCAN_DLC_BYTES_64 ? 64 : 8);
    rx_pending = 0;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *h,
                                              const FDCAN_TxHeaderTypeDef *header,
                                              const uint8_t *data)
{
    tx_count++;
    return HAL_OK;
}

static void deliver(void)
{
    rx_pending = 1;
    HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    RobStride_CAN_Process();
}

static void frame(uint32_t type)
{
    rx_header = (FDCAN_RxHeaderTypeDef){
        .Identifier = (type << 24) | (MOTOR_FL_HAA << 8) | ROBSTRIDE_HOST_ID,
        .IdType = FDCAN_EXTENDED_ID,
        .RxFrameType = FDCAN_DATA_FRAME,
        .FDFormat = FDCAN_CLASSIC_CAN,
        .DataLength = FDCAN_DLC_BYTES_8
    };
    deliver();
}

int run_tests(void)
{
    RobotJoints_Init();
    CHECK(RobStride_CAN_Init());
    const RobStride_Motor_t *motor = RobStride_GetMotor(MOTOR_FL_HAA);
    CHECK(RobotJoints_CheckHealth(0) == JOINT_INVALID_ID);
    CHECK(RobotJoints_CheckHealth(MOTOR_FL_HAA) == JOINT_OFFLINE);

    /* Discovery establishes online, never measurement validity. */
    now_ms = 0;
    frame(0x00);
    CHECK(motor->online && !motor->feedback_received);
    CHECK(RobotJoints_CheckHealth(MOTOR_FL_HAA) == JOINT_NO_FEEDBACK);
    CHECK(RobotJoints_Enable(MOTOR_FL_HAA) == JOINT_NO_FEEDBACK);
    CHECK(tx_count == 0);

    /* A real sample at tick zero must be accepted. */
    frame(0x02);
    CHECK(motor->feedback_received && motor->last_feedback_ms == 0);
    CHECK(motor->feedback_gap_max_ms == 0);
    CHECK(RobotJoints_CheckHealth(MOTOR_FL_HAA) == JOINT_OK);
    CHECK(RobotJoints_Enable(MOTOR_FL_HAA) == JOINT_OK);
    CHECK(tx_count == 1);

    now_ms = ROBOT_FEEDBACK_TIMEOUT_MS - 1;
    CHECK(RobotJoints_CheckHealth(MOTOR_FL_HAA) == JOINT_OK);
    now_ms++;
    CHECK(RobotJoints_CheckHealth(MOTOR_FL_HAA) == JOINT_STALE_FEEDBACK);
    frame(0x00);
    CHECK(motor->last_update_ms == now_ms && motor->last_feedback_ms == 0);
    CHECK(RobotJoints_CheckHealth(MOTOR_FL_HAA) == JOINT_STALE_FEEDBACK);
    CHECK(RobotJoints_Enable(MOTOR_FL_HAA) == JOINT_STALE_FEEDBACK);
    CHECK(RobotJoints_Command(MOTOR_FL_HAA, 0, 0, 0, 60, 5) == JOINT_STALE_FEEDBACK);
    CHECK(tx_count == 1);
    CHECK(RobotJoints_Stop(MOTOR_FL_HAA) == JOINT_OK);
    CHECK(tx_count == 2);

    /* Active reports restore freshness; track completed gaps, excluding discovery. */
    frame(0x18);
    CHECK(RobotJoints_CheckHealth(MOTOR_FL_HAA) == JOINT_OK);
    CHECK(motor->feedback_gap_last_ms == ROBOT_FEEDBACK_TIMEOUT_MS);
    now_ms += 20;
    frame(0x02);
    CHECK(motor->feedback_gap_last_ms == 20);
    CHECK(motor->feedback_gap_max_ms == ROBOT_FEEDBACK_TIMEOUT_MS);
    JointState_t state;
    now_ms += 7;
    CHECK(RobotJoints_GetState(MOTOR_FL_HAA, &state));
    CHECK(state.feedback_received && state.feedback_age_ms == 7);
    CHECK(state.feedback_gap_last_ms == 20);
    CHECK(state.feedback_gap_max_ms == ROBOT_FEEDBACK_TIMEOUT_MS);

    /* Each malformed frame must leave a stale timestamp untouched. */
    uint32_t saved = motor->last_feedback_ms;
    now_ms += ROBOT_FEEDBACK_TIMEOUT_MS;
    rx_header.DataLength = FDCAN_DLC_BYTES_7;
    deliver();
    CHECK(motor->last_feedback_ms == saved);
    rx_header.DataLength = FDCAN_DLC_BYTES_8;
    rx_header.RxFrameType = FDCAN_REMOTE_FRAME;
    deliver();
    CHECK(motor->last_feedback_ms == saved);
    rx_header.RxFrameType = FDCAN_DATA_FRAME;
    rx_header.IdType = FDCAN_STANDARD_ID;
    deliver();
    CHECK(motor->last_feedback_ms == saved);
    rx_header.IdType = FDCAN_EXTENDED_ID;
    rx_header.FDFormat = FDCAN_FD_CAN;
    rx_header.DataLength = FDCAN_DLC_BYTES_64;
    deliver();
    CHECK(motor->last_feedback_ms == saved);
    CHECK(RobotJoints_CheckHealth(MOTOR_FL_HAA) == JOINT_STALE_FEEDBACK);

    /* Reset clears validity; timeout and gap arithmetic survive tick wrap. */
    CHECK(RobStride_CAN_Init());
    CHECK(!motor->feedback_received && motor->feedback_gap_max_ms == 0);
    now_ms = UINT32_MAX - 99U;
    frame(0x02);
    now_ms = 399;
    CHECK(RobotJoints_CheckHealth(MOTOR_FL_HAA) == JOINT_OK);
    now_ms = 400;
    CHECK(RobotJoints_CheckHealth(MOTOR_FL_HAA) == JOINT_STALE_FEEDBACK);
    frame(0x02);
    CHECK(motor->feedback_gap_last_ms == 500);
    CHECK(RobotJoints_CheckHealth(MOTOR_FL_HAA) == JOINT_OK);
    rx_header.Identifier |= 1U << 16;
    deliver();
    CHECK(RobotJoints_CheckHealth(MOTOR_FL_HAA) == JOINT_FAULT);
    now_ms += 5001;
    RobStride_UpdateOnlineStatus();
    CHECK(RobotJoints_CheckHealth(MOTOR_FL_HAA) == JOINT_OFFLINE);
    return 0;
}
