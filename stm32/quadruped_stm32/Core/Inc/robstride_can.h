#ifndef ROBSTRIDE_CAN_H
#define ROBSTRIDE_CAN_H

#include "main.h"
#include <stdint.h>

/* Constants */

#define ROBSTRIDE_HOST_ID       0xFE
#define ROBSTRIDE_MOTOR_COUNT   12

/* Types */

typedef enum
{
    MOTOR_FL_HAA = 1,
    MOTOR_FL_HFE = 2,
    MOTOR_FL_KFE = 3,

    MOTOR_FR_HAA = 4,
    MOTOR_FR_HFE = 5,
    MOTOR_FR_KFE = 6,

    MOTOR_RL_HAA = 7,
    MOTOR_RL_HFE = 8,
    MOTOR_RL_KFE = 9,

    MOTOR_RR_HAA = 10,
    MOTOR_RR_HFE = 11,
    MOTOR_RR_KFE = 12

} MotorId_t;

typedef struct
{
    MotorId_t id;

    uint8_t online;
    uint64_t mcu_uid;

    float position_rad;
    float velocity_rad_s;
    float torque_nm;
    float temperature_c;

    uint8_t fault_bits;
    uint8_t mode;

    uint32_t last_update_ms;

} RobStride_Motor_t;

/* Initialization / processing */

/* Returns 1 on successful CAN setup, 0 on failure. */
uint8_t RobStride_CAN_Init(void);

/* Drain pending feedback in the main loop; decoding does not run in the ISR. */
void RobStride_CAN_Process(void);

/* Call regularly to expire cached online flags. */
void RobStride_UpdateOnlineStatus(void);

/* Motor state */

/* Returns a read-only view of cached motor data, or NULL for an invalid ID. */
const RobStride_Motor_t *RobStride_GetMotor(MotorId_t motor);

/* Discovery / active reporting */

HAL_StatusTypeDef RobStride_RequestDeviceID(MotorId_t motor);

void RobStride_RequestAllDeviceIDs(void);

HAL_StatusTypeDef RobStride_SetActiveReporting(
    MotorId_t motor,
    uint8_t enable);

void RobStride_SetAllActiveReporting(uint8_t enable);

/* Motor control */

/* HAL_OK from control functions means queued for transmission, not acknowledged. */
HAL_StatusTypeDef RobStride_Enable(MotorId_t motor);

HAL_StatusTypeDef RobStride_Stop(MotorId_t motor);

/* Finite command values are clamped to the protocol encoding ranges. */
HAL_StatusTypeDef RobStride_Command(
    MotorId_t motor,
    float position_rad,
    float velocity_rad_s,
    float torque_nm,
    float kp,
    float kd
);

#endif /* ROBSTRIDE_CAN_H */
