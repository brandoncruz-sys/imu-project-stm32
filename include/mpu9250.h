#ifndef MPU9250_H
#define MPU9250_H

#include "stm32l4xx_hal.h"

// Dirección común (AD0 = 0 -> 0x68 << 1)
#define MPU9250_ADDR 0xD0 
// El ID del MPU9250 suele ser 0x71 (el del 6050 es 0x68)
#define MPU9250_WHO_AM_I_REG 0x75

// Variables externas para compartir datos
extern float accel_g_9250[3];
extern float gyro_dps_9250[3];

// Prototipos
void MPU9250_Init(I2C_HandleTypeDef *hi2c);
void MPU9250_Read_All(I2C_HandleTypeDef *hi2c, int16_t* accel, int16_t* gyro, int16_t* temp);
void MPU9250_Convert(int16_t* accel_raw, int16_t* gyro_raw);

#endif