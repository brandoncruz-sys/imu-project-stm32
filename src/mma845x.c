#include "mma845x.h"

// Variable global para guardar datos convertidos
float accel_g_mma[3];

uint8_t MMA845x_Init(I2C_HandleTypeDef *hi2c, uint16_t DevAddress)
{
    uint8_t check;
    uint8_t data;

    // 1. Leer WHO_AM_I para verificar conexión
    HAL_I2C_Mem_Read(hi2c, DevAddress, MMA_REG_WHO_AM_I, 1, &check, 1, 100);

    // Verificamos si es alguno de la familia MMA845x (0x1A, 0x2A, o 0x3A)
    if (check == MMA8451_ID || check == MMA8452_ID || check == MMA8453_ID) 
    {
        // 2. Configurar rango a +/- 2g (Escribir 0x00 en XYZ_DATA_CFG)
        data = 0x00; 
        HAL_I2C_Mem_Write(hi2c, DevAddress, MMA_REG_XYZ_DATA_CFG, 1, &data, 1, 100);

        // 3. Poner en modo ACTIVE (Bit 0 de CTRL_REG1 = 1)
        // Data Rate 100Hz aprox (Bits DR=011 -> 0x18) | Active=1 -> 0x19
        data = 0x19; // 0001 1001
        HAL_I2C_Mem_Write(hi2c, DevAddress, MMA_REG_CTRL_REG1, 1, &data, 1, 100);
        
        return 0; // Éxito
    }
    return 1; // Fallo (ID incorrecto)
}

void MMA845x_Read_Accel(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, int16_t* accel_raw)
{
    uint8_t buffer[6];

    // Leer 6 bytes comenzando desde OUT_X_MSB (0x01)
    // El MMA autoincrementa el registro automáticamente
    HAL_I2C_Mem_Read(hi2c, DevAddress, MMA_REG_OUT_X_MSB, 1, buffer, 6, 100);

    // Los datos son 14/12 bits "Left Aligned" en un contenedor de 16 bits.
    // Esto significa que podemos leerlos como int16 directo.
    accel_raw[0] = (int16_t)((buffer[0] << 8) | buffer[1]); // X
    accel_raw[1] = (int16_t)((buffer[2] << 8) | buffer[3]); // Y
    accel_raw[2] = (int16_t)((buffer[4] << 8) | buffer[5]); // Z
}

void MMA845x_Convert(int16_t* accel_raw)
{
    // Para rango +/- 2g y datos left-aligned de 14 bits (MMA8451):
    // 1g = 4096 cuentas (aprox).
    // Si usas MMA8452 (12 bits), la resolución es menor pero la escala es similar.
    
    float sensitivity = 4096.0f; 

    for (int i = 0; i < 3; i++) {
        // Dividimos por 4 para ajustar el shift de 14 bits (o ajustar según modelo)
        // Usamos una aproximación estándar para visualización
        accel_g_mma[i] = (float)accel_raw[i] / sensitivity;
    }
}
