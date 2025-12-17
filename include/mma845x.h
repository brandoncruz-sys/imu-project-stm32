#ifndef MMA845X_H
#define MMA845X_H

#include "stm32l4xx_hal.h"

// Direcciones posibles (shifted 1 bit a la izquierda para HAL)
// Si SA0=1 -> 0x1D << 1 = 0x3A
// Si SA0=0 -> 0x1C << 1 = 0x38
#define MMA845X_ADDR_DEFAULT (0x1D << 1)

// Registros importantes
#define MMA_REG_STATUS       0x00
#define MMA_REG_OUT_X_MSB    0x01
#define MMA_REG_WHO_AM_I     0x0D
#define MMA_REG_XYZ_DATA_CFG 0x0E
#define MMA_REG_CTRL_REG1    0x2A

// IDs de dispositivo (Who Am I)
#define MMA8451_ID  0x1A
#define MMA8452_ID  0x2A
#define MMA8453_ID  0x3A

// Variable externa para los datos en 'g'
extern float accel_g_mma[3];

// Prototipos
uint8_t MMA845x_Init(I2C_HandleTypeDef *hi2c, uint16_t DevAddress);
void MMA845x_Read_Accel(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, int16_t* accel_raw);
void MMA845x_Convert(int16_t* accel_raw);

#endif
