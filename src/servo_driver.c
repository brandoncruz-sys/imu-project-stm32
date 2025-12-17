#include "servo_driver.h"

void Servo_SetAngle(TIM_HandleTypeDef *htim, uint32_t Channel, uint8_t angle)
{
    if (angle > 180) angle = 180;
    
    // Mapeo: (Valor - InMin) * (OutMax - OutMin) / (InMax - InMin) + OutMin
    // Convierte 0-180 grados a 500-2500 pulsos
    uint32_t pulse = SERVO_MIN + ((SERVO_MAX - SERVO_MIN) * angle) / 180;
    
    __HAL_TIM_SET_COMPARE(htim, Channel, pulse);
}
