#ifndef MPU6050_H
#define MPU6050_H

#include "stm32l4xx_hal.h"

// Dirección I2C (ajustar si es necesario, 0xD0 es lo usual)
#define MPU_ADDR (0x68 << 1)

// Prototipos
void MPU6050_Init(I2C_HandleTypeDef *hi2c);
void MPU6050_Read_All(I2C_HandleTypeDef *hi2c, int16_t* accel, int16_t* gyro, int16_t* temp);

// IMPORTANTE: Esta es la nueva función que espera tu main (con floats)
void MPU6050_Convert(int16_t* accel_raw, int16_t* gyro_raw, float* accel_g, float* gyro_dps);

#endif
