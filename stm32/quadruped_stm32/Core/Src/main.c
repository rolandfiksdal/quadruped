/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bmi088.h"
#include "robstride_can.h"
#include "robot_joints.h"
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum
{
    POSE_DISABLED,
    POSE_STOPPING,
    POSE_TRANSITIONING,
    POSE_HOLDING
} PoseState_t;

/* Which pose is selected is separate from whether it is moving or holding. */
typedef enum
{
    TARGET_RETRACTED,
    TARGET_STANDING,
    TARGET_RETRACTED_BEFORE_REST,
    TARGET_RESTING
} PoseTarget_t;

typedef struct
{
    MotorId_t joint;
    const char *name;
    float target_rad;
} JointTarget_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define COMMAND_INTERVAL_MS 20U
#define COMMAND_BATCH_TIMEOUT_MS 50U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;
FDCAN_HandleTypeDef hfdcan1;

SPI_HandleTypeDef hspi3;

/* USER CODE BEGIN PV */
static const JointTarget_t standing_targets[] =
    {
        {MOTOR_FL_HAA, "FL_HAA", 0.0f},
        {MOTOR_FL_HFE, "FL_HFE", -0.7f},
        {MOTOR_FL_KFE, "FL_KFE", 1.2f},
        {MOTOR_FR_HAA, "FR_HAA", 0.0f},
        {MOTOR_FR_HFE, "FR_HFE", -0.7f},
        {MOTOR_FR_KFE, "FR_KFE", 1.2f},
        {MOTOR_RL_HAA, "RL_HAA", 0.0f},
        {MOTOR_RL_HFE, "RL_HFE", -0.7f},
        {MOTOR_RL_KFE, "RL_KFE", 1.2f},
        {MOTOR_RR_HAA, "RR_HAA", 0.0f},
        {MOTOR_RR_HFE, "RR_HFE", -0.7f},
        {MOTOR_RR_KFE, "RR_KFE", 1.2f}};

static const JointTarget_t retracted_targets[] =
    {
        {MOTOR_FL_HAA, "FL_HAA", 0.0f},
        {MOTOR_FL_HFE, "FL_HFE", -0.852f},
        {MOTOR_FL_KFE, "FL_KFE", 2.340f},
        {MOTOR_FR_HAA, "FR_HAA", 0.0f},
        {MOTOR_FR_HFE, "FR_HFE", -0.853f},
        {MOTOR_FR_KFE, "FR_KFE", 2.344f},
        {MOTOR_RL_HAA, "RL_HAA", 0.0f},
        {MOTOR_RL_HFE, "RL_HFE", -0.877f},
        {MOTOR_RL_KFE, "RL_KFE", 2.37f},
        {MOTOR_RR_HAA, "RR_HAA", 0.0f},
        {MOTOR_RR_HFE, "RR_HFE", -0.857f},
        {MOTOR_RR_KFE, "RR_KFE", 2.348f}};

static const JointTarget_t resting_targets[] =
    {
        {MOTOR_FL_HAA, "FL_HAA", -0.47f},
        {MOTOR_FL_HFE, "FL_HFE", -0.852f},
        {MOTOR_FL_KFE, "FL_KFE", 2.340f},
        {MOTOR_FR_HAA, "FR_HAA", -0.47f},
        {MOTOR_FR_HFE, "FR_HFE", -0.853f},
        {MOTOR_FR_KFE, "FR_KFE", 2.344f},
        {MOTOR_RL_HAA, "RL_HAA", -0.47f},
        {MOTOR_RL_HFE, "RL_HFE", -0.877f},
        {MOTOR_RL_KFE, "RL_KFE", 2.37f},
        {MOTOR_RR_HAA, "RR_HAA", -0.47f},
        {MOTOR_RR_HFE, "RR_HFE", -0.857f},
        {MOTOR_RR_KFE, "RR_KFE", 2.348f}};

#define POSE_JOINT_COUNT (sizeof(standing_targets) / sizeof(standing_targets[0]))

_Static_assert(POSE_JOINT_COUNT == ROBSTRIDE_MOTOR_COUNT, "Pose must cover all motors");
_Static_assert(sizeof(retracted_targets) == sizeof(standing_targets), "Pose sizes must match");
_Static_assert(sizeof(resting_targets) == sizeof(standing_targets), "Pose sizes must match");

static const JointTarget_t *active_targets = retracted_targets;
static PoseTarget_t selected_pose = TARGET_RETRACTED;
static const char *active_pose_name = "retracted";

float start_positions_rad[POSE_JOINT_COUNT];
uint32_t transition_start_ms;
uint32_t last_command_ms;
uint32_t last_batch_duration_ms = 0;
uint32_t max_batch_duration_ms = 0;
uint8_t next_joint_index = 0;
uint8_t batch_in_progress = 0;
uint32_t batch_start_ms;
float batch_progress = 0.0f;
uint32_t last_batch_gap_ms = 0;
uint32_t max_batch_gap_ms = 0;
uint8_t have_previous_batch = 0;
PoseState_t pose_state = POSE_DISABLED;

volatile uint8_t acc_data_ready = 0;
volatile uint8_t gyro_data_ready = 0;
volatile uint8_t user_button_pressed = 0;
volatile uint32_t last_button_irq = 0;

volatile uint32_t acc_irq_count = 0;
volatile uint32_t gyro_irq_count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Return true only when every stop request was queued successfully. */
static uint8_t StopAllJoints(void)
{
    uint8_t all_stopped = 1;

    for (uint8_t i = 0; i < POSE_JOINT_COUNT; i++)
    {
        JointResult_t result = RobotJoints_Stop(active_targets[i].joint);
        if (result != JOINT_OK)
        {
            printf("%s stop failed: %d\r\n", active_targets[i].name, result);
            all_stopped = 0;
        }
        HAL_Delay(2);
        RobStride_CAN_Process();
    }

    batch_in_progress = 0;
    if (all_stopped)
    {
        pose_state = POSE_DISABLED;
        selected_pose = TARGET_RETRACTED;
        active_targets = retracted_targets;
        active_pose_name = "retracted";
    }

    return all_stopped;
}


/* Validate the whole destination and capture measured starts before moving. */
static void StartPoseTransition(PoseTarget_t requested_pose)
{
    const JointTarget_t *targets;
    const char *name;

    switch (requested_pose)
    {
        case TARGET_RETRACTED:
            targets = retracted_targets;
            name = "retracted";
            break;
        case TARGET_STANDING:
            targets = standing_targets;
            name = "standing";
            break;
        case TARGET_RETRACTED_BEFORE_REST:
            targets = retracted_targets;
            name = "retracted before resting";
            break;
        case TARGET_RESTING:
            targets = resting_targets;
            name = "resting";
            break;
        default:
            return;
    }

    uint8_t targets_valid = 1;
    for (uint8_t i = 0; i < POSE_JOINT_COUNT; i++)
    {
        if (RobotJoints_ValidatePosition(targets[i].joint,
                                        targets[i].target_rad) != JOINT_OK)
        {
            printf("%s rejected: %s target=%+.5f rad outside command limits\r\n",
                   name, targets[i].name, targets[i].target_rad);
            targets_valid = 0;
        }
    }
    if (!targets_valid)
    {
        return;
    }

    RobStride_CAN_Process();
    RobStride_UpdateOnlineStatus();
    uint8_t ready = 1;
    for (uint8_t i = 0; i < POSE_JOINT_COUNT; i++)
    {
        JointState_t state;
        if (!RobotJoints_GetState(targets[i].joint, &state) ||
            !state.online || state.fault_bits != 0)
        {
            printf("%s not ready: offline or faulted\r\n", targets[i].name);
            ready = 0;
        }
        else if (!RobotJoints_PrepareStartPosition(targets[i].joint,
                                                  state.position_rad,
                                                  &start_positions_rad[i]))
        {
            printf("%s starting position rejected: %+.5f rad\r\n",
                   targets[i].name, state.position_rad);
            ready = 0;
        }
    }
    if (!ready)
    {
        if (pose_state != POSE_DISABLED)
        {
            batch_in_progress = 0;
            pose_state = POSE_STOPPING;
        }
        return;
    }

    /* Motors stay enabled between poses; only the first press enables them. */
    if (pose_state == POSE_DISABLED)
    {
        for (uint8_t i = 0; i < POSE_JOINT_COUNT; i++)
        {
            JointResult_t result = RobotJoints_Enable(targets[i].joint);
            if (result != JOINT_OK)
            {
                printf("%s enable failed: %d\r\n", targets[i].name, result);
                batch_in_progress = 0;
                pose_state = POSE_STOPPING;
                return;
            }
            HAL_Delay(2);
            RobStride_CAN_Process();
        }
    }

    active_targets = targets;
    selected_pose = requested_pose;
    active_pose_name = name;
    next_joint_index = 0;
    batch_in_progress = 0;
    batch_progress = 0.0f;
    have_previous_batch = 0;
    last_batch_gap_ms = 0;
    max_batch_gap_ms = 0;
    last_batch_duration_ms = 0;
    max_batch_duration_ms = 0;

    printf("Moving to %s over 2 seconds\r\n", name);
    transition_start_ms = HAL_GetTick();
    last_command_ms = transition_start_ms - COMMAND_INTERVAL_MS;
    pose_state = POSE_TRANSITIONING;
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_FDCAN1_Init();
    MX_SPI3_Init();
    /* USER CODE BEGIN 2 */

    /* USER CODE END 2 */

    /* Initialize led */
    BSP_LED_Init(LED_GREEN);

    /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
    BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

    /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
    BspCOMInit.BaudRate = 115200;
    BspCOMInit.WordLength = COM_WORDLENGTH_8B;
    BspCOMInit.StopBits = COM_STOPBITS_1;
    BspCOMInit.Parity = COM_PARITY_NONE;
    BspCOMInit.HwFlowCtl = COM_HWCONTROL_NONE;
    if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
    {
        Error_Handler();
    }

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    BMI088_Data_t imu = {0};

    if (!BMI088_Init())
    {
        printf("BMI088 init failed\r\n");
    }
    else
    {
        printf("BMI088 init success\r\n");
    }

    if (!RobStride_CAN_Init())
    {
        printf("CAN init failed\r\n");
    }
    else
    {
        printf("CAN init success\r\n");

        RobStride_RequestAllDeviceIDs();

        HAL_Delay(200);

        /*
         * responses are sitting in FIFO0,
         * so process them before inspecting states.
         */
        RobStride_CAN_Process();

        RobStride_SetAllActiveReporting(1);

        uint8_t online_count = 0;

        for (uint8_t id = 1; id <= ROBSTRIDE_MOTOR_COUNT; id++)
        {
            const RobStride_Motor_t *motor =
                RobStride_GetMotor((MotorId_t)id);

            if (motor != NULL && motor->online)
            {
                printf("Motor %2u: ONLINE\r\n", id);
                online_count++;
            }
            else
            {
                printf("Motor %2u: OFFLINE\r\n", id);
            }
        }

        printf("%u/%u motors online\r\n",
               online_count,
               ROBSTRIDE_MOTOR_COUNT);
    }

    RobotJoints_Init();

    /* Remain set after a failed stop so the next press retries stopping. */

    uint32_t last_joint_print = 0;

    while (1)
    {
        RobStride_CAN_Process();

        RobStride_UpdateOnlineStatus();

        if ((pose_state == POSE_DISABLED || pose_state == POSE_HOLDING) &&
            HAL_GetTick() - last_joint_print >= 800)
        {
            last_joint_print = HAL_GetTick();
            printf("Pose: %s, %s\r\n", active_pose_name,
                   pose_state == POSE_DISABLED ? "DISABLED (next target)" : "HOLDING");

            for (uint8_t id = 1; id <= ROBSTRIDE_MOTOR_COUNT; id++)
            {
                JointState_t state;

                if (RobotJoints_GetState((MotorId_t)id, &state))
                {
                    printf(
                        "Motor %2u: joint=%+.4f rad, torque=%+.3f Nm, temp=%.1f C, %s",
                        id,
                        state.position_rad,
                        state.torque_nm,
                        state.temperature_c,
                        state.online ? "ONLINE" : "OFFLINE");
                    for (uint8_t i = 0; i < POSE_JOINT_COUNT; i++)
                    {
                        if ((MotorId_t)id == active_targets[i].joint)
                        {
                            /* Signed remaining angle: target minus feedback. */
                            printf(
                                ", %s target=%+.4f rad, error=%+.4f rad",
                                active_targets[i].name,
                                active_targets[i].target_rad,
                                active_targets[i].target_rad - state.position_rad);
                            break;
                        }
                    }
                    printf("\r\n");
                }
                else
                {
                    printf("Motor %2u: state unavailable\r\n", id);
                }
                /* Service feedback between the blocking serial output lines. */
                RobStride_CAN_Process();
            }
            printf(
                "Batch duration: last=%lu max=%lu ms; "
                "start gap: last=%lu max=%lu ms\r\n",
                (unsigned long)last_batch_duration_ms,
                (unsigned long)max_batch_duration_ms,
                (unsigned long)last_batch_gap_ms,
                (unsigned long)max_batch_gap_ms
            );
        }

        if (user_button_pressed)
        {
            user_button_pressed = 0;

            if (pose_state == POSE_TRANSITIONING)
            {
                printf("Still moving to %s; press again after the ramp finishes\r\n",
                       active_pose_name);
            }
            else if (pose_state == POSE_STOPPING)
            {
                /* The stop handler below retries all stop requests. */
            }
            else if (pose_state == POSE_DISABLED)
            {
                StartPoseTransition(TARGET_RETRACTED);
            }
            else if (selected_pose == TARGET_RETRACTED)
            {
                StartPoseTransition(TARGET_STANDING);
            }
            else if (selected_pose == TARGET_STANDING)
            {
                StartPoseTransition(TARGET_RETRACTED_BEFORE_REST);
            }
            else if (selected_pose == TARGET_RETRACTED_BEFORE_REST)
            {
                StartPoseTransition(TARGET_RESTING);
            }
            else
            {
                batch_in_progress = 0;
                pose_state = POSE_STOPPING;
            }
        }
        if (pose_state == POSE_TRANSITIONING)
        {
            uint32_t now = HAL_GetTick();

            /* Start a new batch only when the previous one is finished and time is due. */
            if (!batch_in_progress &&
                now - last_command_ms >= COMMAND_INTERVAL_MS)
            {
                if (have_previous_batch)
                {
                    last_batch_gap_ms = now - last_command_ms;

                    if (last_batch_gap_ms > max_batch_gap_ms)
                    {
                        max_batch_gap_ms = last_batch_gap_ms;
                    }
                }

                have_previous_batch = 1;

                next_joint_index = 0;
                batch_start_ms = now;
                last_command_ms = now;

                uint32_t elapsed_ms = now - transition_start_ms;
                batch_progress = (float)elapsed_ms / 2000.0f;

                if (batch_progress > 1.0f)
                {
                    batch_progress = 1.0f;
                }

                batch_in_progress = 1;
            }

            if (batch_in_progress &&
                now - batch_start_ms >= COMMAND_BATCH_TIMEOUT_MS)
            {
                batch_in_progress = 0;
                pose_state = POSE_STOPPING;

                printf("Batch timeout: queued %u/%u commands\r\n",
                    (unsigned int)next_joint_index,
                    (unsigned int)POSE_JOINT_COUNT);
            }

            /* Attempt at most one command per main-loop pass. */
            if (batch_in_progress && RobStride_CAN_CanSend())
            {
                uint8_t i = next_joint_index;

                float s = batch_progress;
                float smooth_progress = s * s * (3.0f - 2.0f * s);

                float target =
                    start_positions_rad[i]
                    + (active_targets[i].target_rad - start_positions_rad[i])
                    * smooth_progress;

                JointResult_t result = RobotJoints_Command(
                    active_targets[i].joint,
                    target,
                    0.0f,
                    0.0f,
                    60.0f,
                    5.0f);

                if (result != JOINT_OK)
                {
                    printf("%s command failed: %d\r\n",
                        active_targets[i].name, result);

                    batch_in_progress = 0;
                    pose_state = POSE_STOPPING;
                }
                else
                {
                    next_joint_index++;

                    if (next_joint_index == POSE_JOINT_COUNT)
                    {
                        batch_in_progress = 0;

                        last_batch_duration_ms =
                            HAL_GetTick() - batch_start_ms;

                        if (last_batch_duration_ms > max_batch_duration_ms)
                        {
                            max_batch_duration_ms = last_batch_duration_ms;
                        }

                        if (batch_progress >= 1.0f)
                        {
                            pose_state = POSE_HOLDING;
                            printf("%s ramp finished; final targets queued\r\n",
                                   active_pose_name);
                        }
                    }
                }
            }
        }

        if (pose_state == POSE_STOPPING)
        {
            if (StopAllJoints())
            {
                printf("All 12 joint stop requests queued\r\n");
            }
        }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
    RCC_OscInitStruct.PLL.PLLN = 85;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief FDCAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_FDCAN1_Init(void)
{

    /* USER CODE BEGIN FDCAN1_Init 0 */

    /* USER CODE END FDCAN1_Init 0 */

    /* USER CODE BEGIN FDCAN1_Init 1 */

    /* USER CODE END FDCAN1_Init 1 */
    hfdcan1.Instance = FDCAN1;
    hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
    hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
    hfdcan1.Init.AutoRetransmission = ENABLE;
    hfdcan1.Init.TransmitPause = DISABLE;
    hfdcan1.Init.ProtocolException = DISABLE;
    hfdcan1.Init.NominalPrescaler = 10;
    hfdcan1.Init.NominalSyncJumpWidth = 1;
    hfdcan1.Init.NominalTimeSeg1 = 13;
    hfdcan1.Init.NominalTimeSeg2 = 3;
    hfdcan1.Init.DataPrescaler = 1;
    hfdcan1.Init.DataSyncJumpWidth = 1;
    hfdcan1.Init.DataTimeSeg1 = 1;
    hfdcan1.Init.DataTimeSeg2 = 1;
    hfdcan1.Init.StdFiltersNbr = 0;
    hfdcan1.Init.ExtFiltersNbr = 1;
    hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN FDCAN1_Init 2 */

    /* USER CODE END FDCAN1_Init 2 */
}

/**
 * @brief SPI3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI3_Init(void)
{

    /* USER CODE BEGIN SPI3_Init 0 */

    /* USER CODE END SPI3_Init 0 */

    /* USER CODE BEGIN SPI3_Init 1 */

    /* USER CODE END SPI3_Init 1 */
    /* SPI3 parameter configuration*/
    hspi3.Instance = SPI3;
    hspi3.Init.Mode = SPI_MODE_MASTER;
    hspi3.Init.Direction = SPI_DIRECTION_2LINES;
    hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi3.Init.NSS = SPI_NSS_SOFT;
    hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi3.Init.CRCPolynomial = 7;
    hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    if (HAL_SPI_Init(&hspi3) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI3_Init 2 */

    /* USER CODE END SPI3_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* USER CODE BEGIN MX_GPIO_Init_1 */

    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, ACC_CS_Pin | GYRO_CS_Pin, GPIO_PIN_SET);

    /*Configure GPIO pin : B1_BUTTON_Pin */
    GPIO_InitStruct.Pin = B1_BUTTON_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(B1_BUTTON_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : ACC_INT_Pin GYRO_INT_Pin */
    GPIO_InitStruct.Pin = ACC_INT_Pin | GYRO_INT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : ACC_CS_Pin GYRO_CS_Pin */
    GPIO_InitStruct.Pin = ACC_CS_Pin | GYRO_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);

    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    /* USER CODE BEGIN MX_GPIO_Init_2 */

    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == ACC_INT_Pin)
    {
        acc_data_ready = 1;
        acc_irq_count++;
    }

    if (GPIO_Pin == GYRO_INT_Pin)
    {
        gyro_data_ready = 1;
        gyro_irq_count++;
    }

    if (GPIO_Pin == B1_BUTTON_Pin)
    {
        uint32_t now = HAL_GetTick();

        if (now - last_button_irq > 50)
        {
            last_button_irq = now;
            user_button_pressed = 1;
        }
    }
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
