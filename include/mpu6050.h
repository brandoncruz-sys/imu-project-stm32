#ifndef MPU6050_H
#define MPU6050_H

#include "stm32l4xx_hal.h" // Importante para reconocer los tipos de datos

// Dirección ajustada (0x68 << 1)
#define MPU_ADDR 0xD0 

// Variables globales para lectura (externas para que main.c las vea)
extern float accel_g[3];
extern float gyro_dps[3];

// Prototipos de funciones (Ahora piden el I2C como parámetro para ser flexibles)
void MPU6050_Init(I2C_HandleTypeDef *hi2c);
void MPU6050_Read_All(I2C_HandleTypeDef *hi2c, int16_t* accel, int16_t* gyro, int16_t* temp);
void Convert(int16_t* accel_raw, int16_t* gyro_raw);

#endif