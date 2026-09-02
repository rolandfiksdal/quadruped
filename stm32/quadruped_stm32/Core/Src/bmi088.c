#include "bmi088.h"

extern SPI_HandleTypeDef hspi3;

static uint8_t BMI088_Accel_ReadReg(uint8_t reg)
{
    uint8_t tx[3] = {reg | 0x80, 0x00, 0x00};
    uint8_t rx[3] = {0};

    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_RESET);

    HAL_SPI_TransmitReceive(&hspi3, tx, rx, 3, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_SET);

    return rx[2];
}

static uint8_t BMI088_Gyro_ReadReg(uint8_t reg)
{
    uint8_t tx[2] = {reg | 0x80, 0x00};
    uint8_t rx[2] = {0};

    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET);

    HAL_SPI_TransmitReceive(&hspi3, tx, rx, 2, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);

    return rx[1];
}

static void BMI088_Accel_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {reg & 0x7F, value};

    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_RESET);

    HAL_SPI_Transmit(&hspi3, tx, 2, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_SET);
}

static void BMI088_Gyro_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {reg & 0x7F, value};

    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET);

    HAL_SPI_Transmit(&hspi3, tx, 2, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);
}

uint8_t BMI088_Init(void)
{
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);

    HAL_Delay(10);

    // First accel access switches it into SPI mode
    BMI088_Accel_ReadReg(0x00);
    HAL_Delay(1);

    uint8_t acc_id = BMI088_Accel_ReadReg(0x00);
    uint8_t gyro_id = BMI088_Gyro_ReadReg(0x00);

    if (acc_id != 0x1E || gyro_id != 0x0F)
    {
        return 0;
    }

    // Enable accelerometer
    BMI088_Accel_WriteReg(0x7D, 0x04);
    HAL_Delay(10);

    // Accel: 800 Hz, normal bandwidth
    BMI088_Accel_WriteReg(0x40, 0xAB);

    // Accel: ±12 g
    BMI088_Accel_WriteReg(0x41, 0x02);

    // Gyro: ±1000 deg/s
    BMI088_Gyro_WriteReg(0x0F, 0x01);

    // Gyro: 1000 Hz ODR, 116 Hz BW
    BMI088_Gyro_WriteReg(0x10, 0x02);

    // Accel INT1: push-pull, active high
    BMI088_Accel_WriteReg(0x53, 0x0A);

    // Map accel data-ready to INT1
    BMI088_Accel_WriteReg(0x58, 0x04);

    // Enable gyro data-ready interrupt
    BMI088_Gyro_WriteReg(0x15, 0x80);

    // Gyro INT3: push-pull, active high
    BMI088_Gyro_WriteReg(0x16, 0x01);

    // Map gyro data-ready to INT3
    BMI088_Gyro_WriteReg(0x18, 0x01);

    return 1;
}

void BMI088_ReadAccel(BMI088_Data_t *data)
{
    uint8_t tx[8] = {0};
    uint8_t rx[8] = {0};

    tx[0] = 0x12 | 0x80;

    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_RESET);

    HAL_SPI_TransmitReceive(&hspi3, tx, rx, 8, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_SET);

    int16_t x = (int16_t)((uint16_t)rx[3] << 8 | rx[2]);
    int16_t y = (int16_t)((uint16_t)rx[5] << 8 | rx[4]);
    int16_t z = (int16_t)((uint16_t)rx[7] << 8 | rx[6]);

    data->acc_x_g = x / 2730.0f;
    data->acc_y_g = y / 2730.0f;
    data->acc_z_g = z / 2730.0f;
}

void BMI088_ReadGyro(BMI088_Data_t *data)
{
    uint8_t tx[7] = {0};
    uint8_t rx[7] = {0};

    tx[0] = 0x02 | 0x80;

    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET);

    HAL_SPI_TransmitReceive(&hspi3, tx, rx, 7, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);

    int16_t x = (int16_t)((uint16_t)rx[2] << 8 | rx[1]);
    int16_t y = (int16_t)((uint16_t)rx[4] << 8 | rx[3]);
    int16_t z = (int16_t)((uint16_t)rx[6] << 8 | rx[5]);

    data->gyro_x_dps = x / 32.768f;
    data->gyro_y_dps = y / 32.768f;
    data->gyro_z_dps = z / 32.768f;
}
