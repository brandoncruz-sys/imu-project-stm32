#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

// Incluimos la librería HAL para que todo el proyecto entienda qué chip es
#include "stm32l4xx_hal.h"

// Definición del LED de la placa (LD3 en Nucleo-L432KC suele ser PB3)
#define LD3_Pin GPIO_PIN_3
#define LD3_GPIO_Port GPIOB

// Prototipo de manejo de errores
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
