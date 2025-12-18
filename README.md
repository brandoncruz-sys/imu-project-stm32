# imu-project-stm32
El funcionamiento del sistema se divide en **tres etapas claras**.

En la *etapa física*, contamos con tres botones que configuran el entorno del sensor: Primero, el modo 'Reposo', donde caracterizamos el sistema estático. *Segundo*, el modo 'Vibración', donde inyectamos ruido mecánico externo. Y *tercero*, el modo 'Motor', donde un actuador genera movimientos suaves sumando ruido electromagnético.

Durante estos estados, los datos son enviados en tiempo real a una interfaz en Python mediante comunicación UART.

Finalmente, en la etapa de procesamiento, el software captura los **15 ensayos** y, utilizando la librería Matplotlib, procesa las señales. Es importante notar que el sistema trabaja por lotes: debemos esperar a que termine cada ciclo de captura para que el código genere y guarde automáticamente las gráficas en el ordenador. Al completar los tres modos, el sistema cierra el puerto serial y finaliza el experimento de forma segura.

# Control de Servo y Adquisición de Datos IMU con STM32

Este proyecto implementa un sistema embebido basado en **STM32 (Nucleo-L432KC)** capaz de controlar un servomotor mediante una **Máquina de Estados Finitos (FSM)** y adquirir datos de movimiento en tiempo real utilizando un sensor **MPU6050**.

El sistema está diseñado bajo una arquitectura modular, separando los controladores de hardware (Drivers) de la lógica de aplicación.

## Características Principales

* **Lectura de Sensor:** Adquisición de datos de Acelerómetro y Giroscopio (MPU6050) a 10Hz vía I2C.
* **Control de Actuador:** Generación de señal PWM precisa para control de posición de Servomotor (TIM1).
* **Máquina de Estados:** Sistema reactivo a interrupciones externas (Botones) para cambiar modos de operación.
* **Comunicación Serial:** Envío de telemetría en formato CSV (`Ax,Ay,Az,Gx,Gy,Gz`) vía UART a 115200 baudios.
* **Arquitectura Modular:** Código estructurado en drivers independientes (`mpu6050.c`, `servo_driver.c`).

## Hardware Requerido

* Placa de desarrollo: **STM32 Nucleo-L432KC**
* Sensor IMU: **MPU6050** (GY-521)
* Actuador: Micro Servomotor (**SG90** o similar)
* 3x Pulsadores (Push buttons)
* Protoboard y cables de conexión

## Diagrama de Conexiones (Pinout)

| Componente | Pin del Componente | Pin STM32 (Nucleo) | Función |
| :--- | :--- | :--- | :--- |
| **MPU6050** | VCC | 5V / 3.3V | Alimentación |
| | GND | GND | Tierra |
| | SCL | **PB6** (o D5) | I2C Clock |
| | SDA | **PB7** (o D4) | I2C Data |
| **Servo** | Señal (Naranja) | **PA8** (D9) | PWM (TIM1_CH1) |
| | VCC (Rojo) | 5V | Alimentación |
| | GND (Marrón) | GND | Tierra |
| **Botones** | Botón 1 | **PA3** (A2) | Estado: Reposo |
| | Botón 2 | **PA4** (A3) | Estado: Vibración |
| | Botón 3 | **PA5** (A4) | Estado: Movimiento Lento |

> **Nota:** Los botones deben configurarse con resistencias de Pull-down externas o confiar en la configuración interna si se especifica. En este firmware se utiliza interrupción por flanco de subida (`RISING`).

## Lógica de Control (Estados)

El sistema opera bajo tres estados exclusivos, controlados por interrupciones externas (EXTI):

1.  **REPOSO (Botón PA3):**
    * El servo se mantiene fijo en 90°.
2.  **VIBRACIÓN (Botón PA4):**
    * El servo oscila rápidamente entre 30° y 150° (simulando una alerta o vibración).
3.  **MOVIMIENTO LENTO (Botón PA5):**
    * El servo realiza un barrido suave y continuo entre 70° y 110°.

## Estructura del Proyecto

```text
IMU-PROJECT-STM32
├── include
│   ├── main.h          # Definiciones globales
│   ├── mpu6050.h       # Cabecera del driver del sensor
│   └── servo_driver.h  # Cabecera del driver del servo
├── src
│   ├── main.c          # Lógica principal y FSM
│   ├── mpu6050.c       # Implementación protocolo I2C MPU6050
│   └── servo_driver.c  # Implementación control PWM
├── platformio.ini      # Configuración del compilador y placa
└── README.md           # Documentación del proyecto
```
*:)*

