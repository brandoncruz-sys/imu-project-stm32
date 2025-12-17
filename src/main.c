#include "stm32l4xx_hal.h"
#include <stdio.h>
#include <string.h>

// --- SELECCIÓN DE SENSOR (Descomenta el que vayas a usar) ---
// #define USE_MPU6050
#define USE_MMA845X
// -----------------------------------------------------------

#ifdef USE_MPU6050
    #include "mpu6050.h"
    // Dirección por defecto del MPU6050
    #define SENSOR_ADDR (0xD0) 
#endif

#ifdef USE_MMA845X
    #include "mma845x.h"
    // Dirección por defecto del MMA845x (SA0=1 -> 0x1D<<1 = 0x3A)
    #define SENSOR_ADDR (0x1D << 1)
#endif

// Variables Globales de Hardware
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;

// Variables para datos crudos
int16_t accel_raw[3];
int16_t gyro_raw[3]; // Solo se usará en MPU6050
int16_t temp_raw;    // Solo se usará en MPU6050

// Prototipos Hardware
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
void Error_Handler(void);

// Redirección de printf
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_I2C1_Init();

    HAL_Delay(100);
    
    // --- INICIALIZACIÓN ---
    #ifdef USE_MPU6050
        printf("Iniciando MPU6050...\r\n");
        MPU6050_Init(&hi2c1, SENSOR_ADDR); // Si usaste mi versión actualizada con direccion variable
        // O usa MPU6050_Init(&hi2c1); si mantuviste la versión vieja
    #endif

    #ifdef USE_MMA845X
        printf("Iniciando MMA845x...\r\n");
        if (MMA845x_Init(&hi2c1, SENSOR_ADDR) != 0) {
            printf("Error: No se detecta el MMA845x\r\n");
        }
    #endif

    while (1)
    {
        // --- LECTURA ---
        
        #ifdef USE_MPU6050
            MPU6050_Read_All(&hi2c1, SENSOR_ADDR, accel_raw, gyro_raw, &temp_raw);
            // Convert(accel_raw, gyro_raw); // Si tienes la función de conversión activa
        #endif

        #ifdef USE_MMA845X
            MMA845x_Read_Accel(&hi2c1, SENSOR_ADDR, accel_raw);
            MMA845x_Convert(accel_raw); // Convierte a float en accel_g_mma
            
            // Ponemos el Gyro a 0 porque este sensor no tiene
            gyro_raw[0] = 0; gyro_raw[1] = 0; gyro_raw[2] = 0;
        #endif

        // --- IMPRESIÓN CSV ---
        // Formato: Ax, Ay, Az, Gx, Gy, Gz
        printf("%d,%d,%d,%d,%d,%d\r\n",
               accel_raw[0], accel_raw[1], accel_raw[2],
               gyro_raw[0], gyro_raw[1], gyro_raw[2]);

        HAL_Delay(50); // Muestreo a 20Hz aprox
    }
}

// --- TUS CONFIGURACIONES DE HARDWARE ---
// (Estas son vitales, no las toques)

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_RCCEx_EnableMSIPLLMode();
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00F12981;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}
