#ifndef BMI088_H
#define BMI088_H

#include "main.h"
#include <stdint.h>

typedef struct
{
    float acc_x_g;
    float acc_y_g;
    float acc_z_g;

    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
} BMI088_Data_t;

uint8_t BMI088_Init(void);

void BMI088_ReadAccel(BMI088_Data_t *data);
void BMI088_ReadGyro(BMI088_Data_t *data);

#endif
