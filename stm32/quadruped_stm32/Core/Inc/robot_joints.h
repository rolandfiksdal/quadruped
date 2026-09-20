#ifndef ROBOT_JOINTS_H
#define ROBOT_JOINTS_H

#include "robstride_can.h"
#include <stdint.h>

typedef struct
{
    float position_rad;
    float velocity_rad_s;
    float torque_nm;

    float temperature_c;

    uint8_t online;
    uint8_t fault_bits;

} JointState_t;

typedef enum
{
    JOINT_OK = 0,
    JOINT_INVALID_ID,
    JOINT_OFFLINE,
    JOINT_FAULT,
    JOINT_OUT_OF_RANGE,
    JOINT_CAN_ERROR

} JointResult_t;

/* Initialization / calibration */

/* Load calibration before reading or commanding joints. */
void RobotJoints_Init(void);

/* Joint state / target inspection */

/* Returns 1 when state was copied, even if offline; returns 0 on invalid input. */
uint8_t RobotJoints_GetState(
    MotorId_t joint,
    JointState_t *state);

/* Conversion only: no online/fault/limit checks. Invalid ID returns 0 rad. */
float RobotJoints_GetMotorTarget(
    MotorId_t joint,
    float joint_position_rad
);

/* Returns 1 with an accepted starting target, or 0 on invalid input/range. */
uint8_t RobotJoints_PrepareStartPosition(
    MotorId_t joint,
    float measured_position_rad,
    float *start_position_rad);

/* Joint control */

/* Check a finite position against command limits without sending anything. */
JointResult_t RobotJoints_ValidatePosition(MotorId_t joint, float position_rad);

/* JOINT_OK means the CAN request was queued, not acknowledged by the motor. */
JointResult_t RobotJoints_Enable(MotorId_t joint);

/* Attempts a stop even when the motor is offline or faulted. */
JointResult_t RobotJoints_Stop(MotorId_t joint);

JointResult_t RobotJoints_Command(
    MotorId_t joint,
    float position_rad,
    float velocity_rad_s,
    float torque_nm,
    float kp,
    float kd
);

#endif /* ROBOT_JOINTS_H */
