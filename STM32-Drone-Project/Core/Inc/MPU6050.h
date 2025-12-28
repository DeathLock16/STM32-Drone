#pragma once

#include "main.h"
#include <stdint.h>

#define MPU6050_ADDR              (0x68 << 1)

#define MPU6050_REG_WHO_AM_I      0x75
#define MPU6050_REG_PWR_MGMT_1    0x6B
#define MPU6050_REG_SMPLRT_DIV    0x19
#define MPU6050_REG_CONFIG        0x1A
#define MPU6050_REG_GYRO_CONFIG   0x1B
#define MPU6050_REG_ACCEL_CONFIG  0x1C
#define MPU6050_REG_ACCEL_XOUT_H  0x3B

typedef struct
{
    int16_t acc_x_raw;
    int16_t acc_y_raw;
    int16_t acc_z_raw;

    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;

    float acc_x;
    float acc_y;
    float acc_z;

    float gyro_x;
    float gyro_y;
    float gyro_z;

    float gyro_bias_x;
    float gyro_bias_y;
    float gyro_bias_z;

} MPU6050_t;

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef* hi2c);
HAL_StatusTypeDef MPU6050_ReadWhoAmI(I2C_HandleTypeDef* hi2c, uint8_t* who);
HAL_StatusTypeDef MPU6050_ReadRaw(I2C_HandleTypeDef* hi2c, MPU6050_t* imu);
void MPU6050_ComputeScaled(MPU6050_t* imu);
HAL_StatusTypeDef MPU6050_CalibrateGyro(I2C_HandleTypeDef* hi2c, MPU6050_t* imu, uint16_t samples);
