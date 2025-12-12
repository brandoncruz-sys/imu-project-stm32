#include "mpu9250.h"

float accel_g_9250[3];
float gyro_dps_9250[3];

void MPU9250_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t check;
    uint8_t data;

    // 1. Preguntamos ¿Quién eres?
    HAL_I2C_Mem_Read(hi2c, MPU9250_ADDR, MPU9250_WHO_AM_I_REG, 1, &check, 1, 100);

    // 2. Verificamos si es un MPU-9250 (ID = 0x71)
    if (check == 0x71) 
    {
        // Configuración idéntica al 6050 para Accel/Gyro
        data = 0;
        HAL_I2C_Mem_Write(hi2c, MPU9250_ADDR, 0x6B, 1, &data, 1, 100); // Despertar
        data = 0; 
        HAL_I2C_Mem_Write(hi2c, MPU9250_ADDR, 0x1B, 1, &data, 1, 100); // Gyro Config
        data = 0;
        HAL_I2C_Mem_Write(hi2c, MPU9250_ADDR, 0x1C, 1, &data, 1, 100); // Accel Config
    }
}

void MPU9250_Read_All(I2C_HandleTypeDef *hi2c, int16_t* accel, int16_t* gyro, int16_t* temp)
{
    uint8_t buffer[14];

    // La lectura de registros es idéntica al 6050 (inician en 0x3B)
    HAL_I2C_Mem_Read(hi2c, MPU9250_ADDR, 0x3B, 1, buffer, 14, 100);

    accel[0] = (buffer[0] << 8) | buffer[1];
    accel[1] = (buffer[2] << 8) | buffer[3];
    accel[2] = (buffer[4] << 8) | buffer[5];

    *temp = (buffer[6] << 8) | buffer[7];

    gyro[0] = (buffer[8] << 8) | buffer[9];
    gyro[1] = (buffer[10] << 8) | buffer[11];
    gyro[2] = (buffer[12] << 8) | buffer[13];
}

void MPU9250_Convert(int16_t* accel_raw, int16_t* gyro_raw)
{
    for (int i = 0; i < 3; i++) {
        accel_g_9250[i] = accel_raw[i] / 16384.0f;
        gyro_dps_9250[i] = gyro_raw[i] / 131.0f;
    }
}