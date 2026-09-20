#include "robstride_can.h"

extern FDCAN_HandleTypeDef hfdcan1;

/* Constants */

#define ROBSTRIDE_PI                3.14159265f
#define ROBSTRIDE_ONLINE_TIMEOUT_MS 5000

#define ROBSTRIDE_TYPE_DEVICE_ID    0x00
#define ROBSTRIDE_TYPE_COMMAND      0x01
#define ROBSTRIDE_TYPE_FEEDBACK     0x02
#define ROBSTRIDE_TYPE_ENABLE       0x03
#define ROBSTRIDE_TYPE_STOP         0x04
#define ROBSTRIDE_TYPE_ACTIVE_REPORT 0x18

/* Module state */

static RobStride_Motor_t motors[ROBSTRIDE_MOTOR_COUNT + 1];

static volatile uint8_t can_rx_pending = 0;

/* Private encoding / transport helpers */

static uint8_t RobStride_IsValidMotorId(MotorId_t id)
{
    return (id >= 1 && id <= ROBSTRIDE_MOTOR_COUNT);
}

static uint16_t RobStride_ReadU16BE(const uint8_t *data)
{
    return ((uint16_t)data[0] << 8) | data[1];
}

static uint64_t RobStride_ReadU64BE(const uint8_t *data)
{
    return
        ((uint64_t)data[0] << 56) |
        ((uint64_t)data[1] << 48) |
        ((uint64_t)data[2] << 40) |
        ((uint64_t)data[3] << 32) |
        ((uint64_t)data[4] << 24) |
        ((uint64_t)data[5] << 16) |
        ((uint64_t)data[6] << 8)  |
        ((uint64_t)data[7]);
}

static float RobStride_UIntToFloat(uint16_t value,
                                   float min_value,
                                   float max_value)
{
    return min_value +
           ((float)value / 65535.0f) *
           (max_value - min_value);
}

static uint16_t RobStride_FloatToUInt(float value,
                                      float min_value,
                                      float max_value)
{
    if (value < min_value)
    {
        value = min_value;
    }

    if (value > max_value)
    {
        value = max_value;
    }

    return (uint16_t)(
        (value - min_value) *
        65535.0f /
        (max_value - min_value)
    );
}

static HAL_StatusTypeDef RobStride_SendFrame(uint32_t identifier,
                                             const uint8_t data[8])
{
    FDCAN_TxHeaderTypeDef txHeader = {0};

    txHeader.Identifier = identifier;
    txHeader.IdType = FDCAN_EXTENDED_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.DataLength = FDCAN_DLC_BYTES_8;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker = 0;

    return HAL_FDCAN_AddMessageToTxFifoQ(
        &hfdcan1,
        &txHeader,
        (uint8_t *)data
    );
}

/* Private feedback decoding */

static void RobStride_HandleDeviceIdFrame(uint32_t identifier,
                                          const uint8_t data[8])
{
    uint8_t motor_id = (identifier >> 8) & 0xFF;
    uint8_t host_id = identifier & 0xFF;

    if (!RobStride_IsValidMotorId(motor_id))
    {
        return;
    }

    if (host_id != ROBSTRIDE_HOST_ID)
    {
        return;
    }

    RobStride_Motor_t *motor = &motors[motor_id];

    motor->mcu_uid = RobStride_ReadU64BE(data);
    motor->online = 1;
    motor->last_update_ms = HAL_GetTick();
}

static void RobStride_HandleFeedbackFrame(uint32_t identifier,
                                          const uint8_t data[8])
{
    uint8_t motor_id = (identifier >> 8) & 0xFF;

    if (!RobStride_IsValidMotorId(motor_id))
    {
        return;
    }

    RobStride_Motor_t *motor = &motors[motor_id];

    uint16_t raw_position =
        RobStride_ReadU16BE(&data[0]);

    uint16_t raw_velocity =
        RobStride_ReadU16BE(&data[2]);

    uint16_t raw_torque =
        RobStride_ReadU16BE(&data[4]);

    uint16_t raw_temperature =
        RobStride_ReadU16BE(&data[6]);

    motor->position_rad =
        RobStride_UIntToFloat(
            raw_position,
            -4.0f * ROBSTRIDE_PI,
             4.0f * ROBSTRIDE_PI
        );

    motor->velocity_rad_s =
        RobStride_UIntToFloat(
            raw_velocity,
            -50.0f,
             50.0f
        );

    motor->torque_nm =
        RobStride_UIntToFloat(
            raw_torque,
            -36.0f,
             36.0f
        );

    motor->temperature_c =
        raw_temperature / 10.0f;

    motor->fault_bits =
        (identifier >> 16) & 0x3F;

    motor->mode =
        (identifier >> 22) & 0x03;

    motor->online = 1;
    motor->last_update_ms = HAL_GetTick();
}

static void RobStride_HandleFrame(uint32_t identifier,
                                  const uint8_t data[8])
{
    uint8_t communication_type =
        (identifier >> 24) & 0x1F;

    switch (communication_type)
    {
        case ROBSTRIDE_TYPE_DEVICE_ID:
            RobStride_HandleDeviceIdFrame(identifier, data);
            break;

        case ROBSTRIDE_TYPE_FEEDBACK:
        case ROBSTRIDE_TYPE_ACTIVE_REPORT:
            RobStride_HandleFeedbackFrame(identifier, data);
            break;

        default:
            break;
    }
}

/* Initialization / processing */

uint8_t RobStride_CAN_Init(void)
{
    for (uint8_t id = 1;
         id <= ROBSTRIDE_MOTOR_COUNT;
         id++)
    {
        motors[id] = (RobStride_Motor_t){.id = (MotorId_t)id};
    }

    FDCAN_FilterTypeDef filter = {0};

    filter.IdType = FDCAN_EXTENDED_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;

    /*
     * Accept all extended frames for now.
     * We can make this stricter later.
     */
    filter.FilterID1 = 0x00000000;
    filter.FilterID2 = 0x00000000;

    if (HAL_FDCAN_ConfigFilter(
            &hfdcan1,
            &filter) != HAL_OK)
    {
        return 0;
    }

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        return 0;
    }

    if (HAL_FDCAN_ActivateNotification(
            &hfdcan1,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
            0) != HAL_OK)
    {
        return 0;
    }

    return 1;
}

uint8_t RobStride_CAN_CanSend(void)
{
    uint32_t free_slots = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1);

    if (free_slots == 0)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

void RobStride_CAN_Process(void)
{
    if (!can_rx_pending)
    {
        return;
    }

    /*
     * Clear first.
     *
     * If another frame arrives while we're processing,
     * the interrupt can set this back to 1.
     */
    can_rx_pending = 0;

    while (HAL_FDCAN_GetRxFifoFillLevel(
               &hfdcan1,
               FDCAN_RX_FIFO0) > 0)
    {
        FDCAN_RxHeaderTypeDef rxHeader;
        uint8_t data[8];

        if (HAL_FDCAN_GetRxMessage(
                &hfdcan1,
                FDCAN_RX_FIFO0,
                &rxHeader,
                data) != HAL_OK)
        {
            break;
        }

        RobStride_HandleFrame(
            rxHeader.Identifier,
            data);
    }
}

void RobStride_UpdateOnlineStatus(void)
{
    uint32_t now = HAL_GetTick();

    for (uint8_t id = 1;
         id <= ROBSTRIDE_MOTOR_COUNT;
         id++)
    {
        if (!motors[id].online)
        {
            continue;
        }

        if ((now - motors[id].last_update_ms) >
            ROBSTRIDE_ONLINE_TIMEOUT_MS)
        {
            motors[id].online = 0;
        }
    }
}

/* Motor state */

const RobStride_Motor_t *RobStride_GetMotor(MotorId_t motor)
{
    if (!RobStride_IsValidMotorId(motor))
    {
        return NULL;
    }

    return &motors[motor];
}

/* Discovery / active reporting */

HAL_StatusTypeDef RobStride_RequestDeviceID(MotorId_t motor)
{
    if (!RobStride_IsValidMotorId(motor))
    {
        return HAL_ERROR;
    }

    uint8_t data[8] = {0};

    uint32_t identifier =
        ((uint32_t)ROBSTRIDE_TYPE_DEVICE_ID << 24) |
        ((uint32_t)ROBSTRIDE_HOST_ID << 8) |
        ((uint32_t)motor);

    return RobStride_SendFrame(identifier, data);
}

void RobStride_RequestAllDeviceIDs(void)
{
    for (uint8_t id = 1; id <= ROBSTRIDE_MOTOR_COUNT; id++)
    {
        RobStride_RequestDeviceID((MotorId_t)id);

        HAL_Delay(2);

        RobStride_CAN_Process();
    }
}

HAL_StatusTypeDef RobStride_SetActiveReporting(
    MotorId_t motor,
    uint8_t enable)
{
    if (!RobStride_IsValidMotorId(motor))
    {
        return HAL_ERROR;
    }

    uint8_t data[8] =
    {
        0x01,
        0x02,
        0x03,
        0x04,
        0x05,
        0x06,
        enable ? 0x01 : 0x00,
        0x00
    };

    uint32_t identifier =
        ((uint32_t)ROBSTRIDE_TYPE_ACTIVE_REPORT << 24) |
        ((uint32_t)ROBSTRIDE_HOST_ID << 8) |
        ((uint32_t)motor);

    return RobStride_SendFrame(identifier, data);
}

void RobStride_SetAllActiveReporting(uint8_t enable)
{
    for (uint8_t id = 1; id <= ROBSTRIDE_MOTOR_COUNT; id++)
    {
        RobStride_SetActiveReporting(
            (MotorId_t)id,
            enable);

        HAL_Delay(2);

        RobStride_CAN_Process();
    }
}

/* Motor control */

HAL_StatusTypeDef RobStride_Enable(MotorId_t motor)
{
    if (!RobStride_IsValidMotorId(motor))
    {
        return HAL_ERROR;
    }

    uint8_t data[8] = {0};

    uint32_t identifier =
        ((uint32_t)ROBSTRIDE_TYPE_ENABLE << 24) |
        ((uint32_t)ROBSTRIDE_HOST_ID << 8) |
        ((uint32_t)motor);

    return RobStride_SendFrame(identifier, data);
}

HAL_StatusTypeDef RobStride_Stop(MotorId_t motor)
{
    if (!RobStride_IsValidMotorId(motor))
    {
        return HAL_ERROR;
    }

    uint8_t data[8] = {0};

    uint32_t identifier =
        ((uint32_t)ROBSTRIDE_TYPE_STOP << 24) |
        ((uint32_t)ROBSTRIDE_HOST_ID << 8) |
        ((uint32_t)motor);

    return RobStride_SendFrame(identifier, data);
}

HAL_StatusTypeDef RobStride_Command(
    MotorId_t motor,
    float position_rad,
    float velocity_rad_s,
    float torque_nm,
    float kp,
    float kd)
{
    if (!RobStride_IsValidMotorId(motor))
    {
        return HAL_ERROR;
    }

    uint16_t position_raw =
        RobStride_FloatToUInt(
            position_rad,
            -4.0f * ROBSTRIDE_PI,
             4.0f * ROBSTRIDE_PI
        );

    uint16_t velocity_raw =
        RobStride_FloatToUInt(
            velocity_rad_s,
            -50.0f,
             50.0f
        );

    uint16_t torque_raw =
        RobStride_FloatToUInt(
            torque_nm,
            -36.0f,
             36.0f
        );

    uint16_t kp_raw =
        RobStride_FloatToUInt(
            kp,
            0.0f,
            5000.0f
        );

    uint16_t kd_raw =
        RobStride_FloatToUInt(
            kd,
            0.0f,
            100.0f
        );

    uint8_t data[8];

    data[0] = (position_raw >> 8) & 0xFF;
    data[1] = position_raw & 0xFF;

    data[2] = (velocity_raw >> 8) & 0xFF;
    data[3] = velocity_raw & 0xFF;

    data[4] = (kp_raw >> 8) & 0xFF;
    data[5] = kp_raw & 0xFF;

    data[6] = (kd_raw >> 8) & 0xFF;
    data[7] = kd_raw & 0xFF;

    uint32_t identifier =
        ((uint32_t)ROBSTRIDE_TYPE_COMMAND << 24) |
        ((uint32_t)torque_raw << 8) |
        ((uint32_t)motor);

    return RobStride_SendFrame(identifier, data);
}

/* HAL interrupt callback */

void HAL_FDCAN_RxFifo0Callback(
    FDCAN_HandleTypeDef *hfdcan,
    uint32_t RxFifo0ITs)
{
    if (hfdcan->Instance != FDCAN1)
    {
        return;
    }

    if (RxFifo0ITs &
        FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
    {
        can_rx_pending = 1;
    }
}
