#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include "stm32l4xx_hal.h"

// Definiciones de PWM (1MHz clock -> 1 tick = 1us)
#define SERVO_MIN 500  // 0.5 ms
#define SERVO_MAX 2500 // 2.5 ms

// Prototipo
void Servo_SetAngle(TIM_HandleTypeDef *htim, uint32_t Channel, uint8_t angle);

#endif
