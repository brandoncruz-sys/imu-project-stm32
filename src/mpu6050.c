#include "mpu6050.h"

// Variables para almacenar cálculos
float accel_g[3];
float gyro_dps[3];

void MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t check;
    uint8_t data;

    // Usamos el puntero 'hi2c' en lugar de la variable global '&hi2c1'
    HAL_I2C_Mem_Read(hi2c, MPU_ADDR, 0x75, 1, &check, 1, 100);

    if (check == 0x68)
    {
        data = 0;
        HAL_I2C_Mem_Write(hi2c, MPU_ADDR, 0x6B, 1, &data, 1, 100); // Despertar
        
        data = 0;
        HAL_I2C_Mem_Write(hi2c, MPU_ADDR, 0x1B, 1, &data, 1, 100); // Gyro Config
        
        data = 0;
        HAL_I2C_Mem_Write(hi2c, MPU_ADDR, 0x1C, 1, &data, 1, 100); // Accel Config
    }
}

void MPU6050_Read_All(I2C_HandleTypeDef *hi2c, int16_t* accel, int16_t* gyro, int16_t* temp)
{
    uint8_t buffer[14];

    HAL_I2C_Mem_Read(hi2c, MPU_ADDR, 0x3B, 1, buffer, 14, 100);

    accel[0] = (buffer[0] << 8) | buffer[1];
    accel[1] = (buffer[2] << 8) | buffer[3];
    accel[2] = (buffer[4] << 8) | buffer[5];

    *temp = (buffer[6] << 8) | buffer[7];

    gyro[0] = (buffer[8] << 8) | buffer[9];
    gyro[1] = (buffer[10] << 8) | buffer[11];
    gyro[2] = (buffer[12] << 8) | buffer[13];
}

void Convert(int16_t* accel_raw, int16_t* gyro_raw)
{
    for (int i = 0; i < 3; i++) {
        accel_g[i] = accel_raw[i] / 16384.0f;
        gyro_dps[i] = gyro_raw[i] / 131.0f;
    }
}