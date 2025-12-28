#include "MPU6050.h"

static HAL_StatusTypeDef writeReg(I2C_HandleTypeDef* hi2c, uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, HAL_MAX_DELAY);
}

static HAL_StatusTypeDef readReg(I2C_HandleTypeDef* hi2c, uint8_t reg, uint8_t* val)
{
    return HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT, val, 1, HAL_MAX_DELAY);
}

static HAL_StatusTypeDef readRegs(I2C_HandleTypeDef* hi2c, uint8_t reg, uint8_t* buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, len, HAL_MAX_DELAY);
}

HAL_StatusTypeDef MPU6050_ReadWhoAmI(I2C_HandleTypeDef* hi2c, uint8_t* who)
{
    return readReg(hi2c, MPU6050_REG_WHO_AM_I, who);
}

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef* hi2c)
{
    HAL_Delay(100);

    // Wake up, set clock source to PLL with X gyro (often more stable than internal)
    if (writeReg(hi2c, MPU6050_REG_PWR_MGMT_1, 0x01) != HAL_OK)
        return HAL_ERROR;

    HAL_Delay(10);

    // DLPF ~44Hz (CONFIG=3) – sensowne do drona/wizualizacji
    if (writeReg(hi2c, MPU6050_REG_CONFIG, 0x03) != HAL_OK)
        return HAL_ERROR;

    // Sample rate: 1kHz/(1+div) przy włączonym DLPF
    // 200Hz -> div=4
    if (writeReg(hi2c, MPU6050_REG_SMPLRT_DIV, 4) != HAL_OK)
        return HAL_ERROR;

    // Gyro ±250 dps
    if (writeReg(hi2c, MPU6050_REG_GYRO_CONFIG, 0x00) != HAL_OK)
        return HAL_ERROR;

    // Acc ±2g
    if (writeReg(hi2c, MPU6050_REG_ACCEL_CONFIG, 0x00) != HAL_OK)
        return HAL_ERROR;

    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_ReadRaw(I2C_HandleTypeDef* hi2c, MPU6050_t* imu)
{
    uint8_t buf[14];

    if (readRegs(hi2c, MPU6050_REG_ACCEL_XOUT_H, buf, 14) != HAL_OK)
        return HAL_ERROR;

    imu->acc_x_raw  = (int16_t)((buf[0]  << 8) | buf[1]);
    imu->acc_y_raw  = (int16_t)((buf[2]  << 8) | buf[3]);
    imu->acc_z_raw  = (int16_t)((buf[4]  << 8) | buf[5]);

    imu->gyro_x_raw = (int16_t)((buf[8]  << 8) | buf[9]);
    imu->gyro_y_raw = (int16_t)((buf[10] << 8) | buf[11]);
    imu->gyro_z_raw = (int16_t)((buf[12] << 8) | buf[13]);

    return HAL_OK;
}

void MPU6050_ComputeScaled(MPU6050_t* imu)
{
    // Acc ±2g => 16384 LSB/g
    imu->acc_x = imu->acc_x_raw / 16384.0f;
    imu->acc_y = imu->acc_y_raw / 16384.0f;
    imu->acc_z = imu->acc_z_raw / 16384.0f;

    // Gyro ±250 dps => 131 LSB/(deg/s)
    imu->gyro_x = (imu->gyro_x_raw - imu->gyro_bias_x) / 131.0f;
    imu->gyro_y = (imu->gyro_y_raw - imu->gyro_bias_y) / 131.0f;
    imu->gyro_z = (imu->gyro_z_raw - imu->gyro_bias_z) / 131.0f;
}

HAL_StatusTypeDef MPU6050_CalibrateGyro(I2C_HandleTypeDef* hi2c, MPU6050_t* imu, uint16_t samples)
{
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;

    imu->gyro_bias_x = 0.0f;
    imu->gyro_bias_y = 0.0f;
    imu->gyro_bias_z = 0.0f;

    // Ważne: podczas kalibracji moduł ma leżeć nieruchomo
    for (uint16_t i = 0; i < samples; i++)
    {
        if (MPU6050_ReadRaw(hi2c, imu) != HAL_OK)
            return HAL_ERROR;

        sum_x += (float)imu->gyro_x_raw;
        sum_y += (float)imu->gyro_y_raw;
        sum_z += (float)imu->gyro_z_raw;

        HAL_Delay(2);
    }

    imu->gyro_bias_x = sum_x / (float)samples;
    imu->gyro_bias_y = sum_y / (float)samples;
    imu->gyro_bias_z = sum_z / (float)samples;

    return HAL_OK;
}
