#include "robot_joints.h"
#include <stddef.h>

/* Constants */

#define DEG_TO_RAD 0.01745329252f
#define PI_F       3.14159265f
#define TWO_PI_F   6.28318531f

/* Private calibration data; array index is the motor/joint ID. */

typedef struct
{
    int8_t sign;

    float motor_reference_rad;
    float joint_reference_rad;

    float min_rad;
    float max_rad;

} JointConfig_t;

/* IDs start at 1; index 0 is unused. */
static JointConfig_t joint_config[ROBSTRIDE_MOTOR_COUNT + 1];

/* Private coordinate helpers */

static uint8_t RobotJoints_IsValid(MotorId_t joint)
{
    return joint >= 1 &&
           joint <= ROBSTRIDE_MOTOR_COUNT;
}

static float RobotJoints_WrapToPi(float angle)
{
    while (angle > PI_F)
    {
        angle -= TWO_PI_F;
    }

    while (angle < -PI_F)
    {
        angle += TWO_PI_F;
    }

    return angle;
}

static float RobotJoints_JointToMotorPosition(
    MotorId_t joint,
    float joint_position_rad)
{
    const JointConfig_t *config =
        &joint_config[joint];

    const RobStride_Motor_t *motor =
        RobStride_GetMotor(joint);

    float base_target =
        config->motor_reference_rad
        + config->sign *
          (joint_position_rad -
           config->joint_reference_rad);

    if (motor == NULL)
    {
        return base_target;
    }

    /*
     * Move the target onto the 2*pi-equivalent branch
     * closest to the motor's current reported position.
     */
    while (base_target - motor->position_rad > PI_F)
    {
        base_target -= TWO_PI_F;
    }

    while (base_target - motor->position_rad < -PI_F)
    {
        base_target += TWO_PI_F;
    }

    return base_target;
}

/* Initialization / calibration */

void RobotJoints_Init(void)
{
    /*
     * FL HAA
     * Motor zero = minimum abduction position.
     * Negative motor rotation = positive joint abduction.
     */
    joint_config[MOTOR_FL_HAA] = (JointConfig_t)
    {
        .sign = -1,
        .motor_reference_rad = 0.0f,
        .joint_reference_rad = -27.5f * DEG_TO_RAD,
        .min_rad = -27.5f * DEG_TO_RAD,
        .max_rad =  27.5f * DEG_TO_RAD
    };

    /*
     * FR HAA
     * Positive motor rotation = positive joint abduction.
     */
    joint_config[MOTOR_FR_HAA] = (JointConfig_t)
    {
        .sign = +1,
        .motor_reference_rad = 0.0f,
        .joint_reference_rad = -27.5f * DEG_TO_RAD,
        .min_rad = -27.5f * DEG_TO_RAD,
        .max_rad =  27.5f * DEG_TO_RAD
    };

    /*
     * RL HAA
     */
    joint_config[MOTOR_RL_HAA] = (JointConfig_t)
    {
        .sign = +1,
        .motor_reference_rad = 0.0f,
        .joint_reference_rad = -27.5f * DEG_TO_RAD,
        .min_rad = -27.5f * DEG_TO_RAD,
        .max_rad =  27.5f * DEG_TO_RAD
    };

    /*
     * RR HAA
     */
    joint_config[MOTOR_RR_HAA] = (JointConfig_t)
    {
        .sign = -1,
        .motor_reference_rad = 0.0f,
        .joint_reference_rad = -27.5f * DEG_TO_RAD,
        .min_rad = -27.5f * DEG_TO_RAD,
        .max_rad =  27.5f * DEG_TO_RAD
    };

    joint_config[MOTOR_FL_HFE] = (JointConfig_t)
    {
        .sign = +1,
        .motor_reference_rad = 6.590f,
        .joint_reference_rad = 0.0f,
        .min_rad = -1.360f,
        .max_rad = +0.620f
    };

    joint_config[MOTOR_FR_HFE] = (JointConfig_t)
    {
        .sign = -1,
        .motor_reference_rad = 5.368f,
        .joint_reference_rad = 0.0f,
        .min_rad = -1.360f,
        .max_rad = +0.620f
    };

    joint_config[MOTOR_RL_HFE] = (JointConfig_t)
    {
        .sign = +1,
        .motor_reference_rad = 3.520f,
        .joint_reference_rad = 0.0f,
        .min_rad = -1.360f,
        .max_rad = +0.620f
    };

    joint_config[MOTOR_RR_HFE] = (JointConfig_t)
    {
        .sign = -1,
        .motor_reference_rad = 5.238f,
        .joint_reference_rad = 0.0f,
        .min_rad = -1.360f,
        .max_rad = +0.620f
    };

    joint_config[MOTOR_FL_KFE] = (JointConfig_t)
    {
        .sign = +1,

        .motor_reference_rad = -0.177f,
        .joint_reference_rad = 134.82f * DEG_TO_RAD,

        .min_rad = 47.39f * DEG_TO_RAD,
        .max_rad = 134.82f * DEG_TO_RAD
    };

    joint_config[MOTOR_FR_KFE] = (JointConfig_t)
    {
        .sign = -1,

        .motor_reference_rad = +0.184f,
        .joint_reference_rad = 134.82f * DEG_TO_RAD,

        .min_rad = 48.25f * DEG_TO_RAD,
        .max_rad = 134.82f * DEG_TO_RAD
    };

    joint_config[MOTOR_RL_KFE] = (JointConfig_t)
    {
        .sign = +1,

        .motor_reference_rad = +4.000f,
        .joint_reference_rad = 134.82f * DEG_TO_RAD,

        .min_rad = 45.44f * DEG_TO_RAD,
        .max_rad = 134.82f * DEG_TO_RAD
    };

    joint_config[MOTOR_RR_KFE] = (JointConfig_t)
    {
        .sign = -1,

        .motor_reference_rad = +5.525f,
        .joint_reference_rad = 134.82f * DEG_TO_RAD,

        .min_rad = 47.90f * DEG_TO_RAD,
        .max_rad = 134.82f * DEG_TO_RAD
    };
}

/* Joint state / target inspection */

uint8_t RobotJoints_GetState(
    MotorId_t joint,
    JointState_t *state)
{
    if (!RobotJoints_IsValid(joint) || state == NULL)
    {
        return 0;
    }

    const RobStride_Motor_t *motor =
        RobStride_GetMotor(joint);

    if (motor == NULL)
    {
        return 0;
    }

    const JointConfig_t *config =
        &joint_config[joint];

    float motor_delta =
        RobotJoints_WrapToPi(
            motor->position_rad -
            config->motor_reference_rad
        );

    state->position_rad =
        config->sign * motor_delta
        + config->joint_reference_rad;

    state->velocity_rad_s =
        config->sign *
        motor->velocity_rad_s;

    state->torque_nm =
        config->sign *
        motor->torque_nm;

    state->temperature_c =
        motor->temperature_c;

    state->online =
        motor->online;

    state->fault_bits =
        motor->fault_bits;

    return 1;
}

float RobotJoints_GetMotorTarget(
    MotorId_t joint,
    float joint_position_rad)
{
    if (!RobotJoints_IsValid(joint))
    {
        return 0.0f;
    }

    return RobotJoints_JointToMotorPosition(
        joint,
        joint_position_rad);
}

/* Joint control */

JointResult_t RobotJoints_Enable(MotorId_t joint)
{
    if (!RobotJoints_IsValid(joint))
    {
        return JOINT_INVALID_ID;
    }

    const RobStride_Motor_t *motor =
        RobStride_GetMotor(joint);

    if (motor == NULL || !motor->online)
    {
        return JOINT_OFFLINE;
    }

    if (motor->fault_bits != 0)
    {
        return JOINT_FAULT;
    }

    if (RobStride_Enable(joint) != HAL_OK)
    {
        return JOINT_CAN_ERROR;
    }

    return JOINT_OK;
}

JointResult_t RobotJoints_Stop(MotorId_t joint)
{
    if (!RobotJoints_IsValid(joint))
    {
        return JOINT_INVALID_ID;
    }

    if (RobStride_Stop(joint) != HAL_OK)
    {
        return JOINT_CAN_ERROR;
    }

    return JOINT_OK;
}

JointResult_t RobotJoints_Command(
    MotorId_t joint,
    float position_rad,
    float velocity_rad_s,
    float torque_nm,
    float kp,
    float kd)
{
    if (!RobotJoints_IsValid(joint))
    {
        return JOINT_INVALID_ID;
    }

    const RobStride_Motor_t *motor =
        RobStride_GetMotor(joint);

    if (motor == NULL || !motor->online)
    {
        return JOINT_OFFLINE;
    }

    if (motor->fault_bits != 0)
    {
        return JOINT_FAULT;
    }

    const JointConfig_t *config =
        &joint_config[joint];

    /*
     * Never silently command outside the
     * calibrated mechanical range.
     */
    if (position_rad < config->min_rad ||
        position_rad > config->max_rad)
    {
        return JOINT_OUT_OF_RANGE;
    }

    float motor_position =
        RobotJoints_JointToMotorPosition(
            joint,
            position_rad);

    /*
     * Velocity and torque directions must
     * follow the same sign convention.
     */
    float motor_velocity =
        config->sign * velocity_rad_s;

    float motor_torque =
        config->sign * torque_nm;

    if (RobStride_Command(
            joint,
            motor_position,
            motor_velocity,
            motor_torque,
            kp,
            kd) != HAL_OK)
    {
        return JOINT_CAN_ERROR;
    }

    return JOINT_OK;
}
