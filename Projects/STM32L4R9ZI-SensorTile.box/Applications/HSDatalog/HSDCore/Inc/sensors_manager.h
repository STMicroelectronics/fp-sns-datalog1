/**
  ******************************************************************************
  * @file    sensors_manager.h
  * @brief   Header for sensors_manager.c module.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SENSORS_MANAGER_H
#define __SENSORS_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"
#include "cmsis_os.h"

typedef enum
{
  SM_SENSOR_STATE_RUNNING,
  SM_SENSOR_STATE_SUSPENDING,
  SM_SENSOR_STATE_SUSPENDED,
  SM_SENSOR_STATE_INITIALIZING,
  SM_SENSOR_STATE_MLC_CONFIG,
} SM_Sensor_State_t;

typedef struct
{
  float ODR[4];
  float FS[4]; /* size = 4, in case of combo devices */
  uint8_t subSensorActive[4];
} SM_Init_Param_t;

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c3;

extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_spi1_rx;

extern DMA_HandleTypeDef hdma_spi3_tx;
extern DMA_HandleTypeDef hdma_spi3_rx;

extern DMA_HandleTypeDef hdma_i2c1_rx;
extern DMA_HandleTypeDef hdma_i2c1_tx;

extern DMA_HandleTypeDef hdma_i2c3_rx;
extern DMA_HandleTypeDef hdma_i2c3_tx;

extern TIM_HandleTypeDef htim5;

typedef struct
{
  uint8_t WhoAmI;
  uint8_t I2C_address;
  GPIO_TypeDef *GPIOx;
  uint16_t GPIO_Pin;
  osSemaphoreId *sem;
} sensor_handle_t;

typedef struct
{
  sensor_handle_t *sensorHandler;
  uint8_t isRead;
  uint8_t *dataPtr;
  uint8_t regAddr;
  uint16_t readSize;
} SM_Message_t;

/**SPI1 GPIO Configuration
 PE13     ------> SPI1_SCK
 PE14     ------> SPI1_MISO
 PE15     ------> SPI1_MOSI
 */
#define SPI1_SCK_PIN                            GPIO_PIN_13
#define SPI1_SCK_GPIO_PORT                      GPIOE
#define __SPI1_SCK_PIN_CLK_ENABLE()             __HAL_RCC_GPIOE_CLK_ENABLE()

#define SPI1_MISO_PIN                           GPIO_PIN_14
#define SPI1_MISO_GPIO_PORT                     GPIOE
#define __SPI1_MISO_PIN_CLK_ENABLE()            __HAL_RCC_GPIOB_CLK_ENABLE()

#define SPI1_MOSI_PIN                           GPIO_PIN_15
#define SPI1_MOSI_GPIO_PORT                     GPIOE
#define __SPI1_MOSI_PIN_CLK_ENABLE()            __HAL_RCC_GPIOB_CLK_ENABLE()

#define SPI1_RX_DMA_CHANNEL                     DMA1_Channel1
#define SPI1_RX_DMA_IRQn                        DMA1_Channel1_IRQn

#define SPI1_TX_DMA_CHANNEL                     DMA1_Channel2
#define SPI1_TX_DMA_IRQn                        DMA1_Channel2_IRQn

#define __SPI1_DMA_CLK_ENABLE()                __HAL_RCC_DMA1_CLK_ENABLE()

/**SPI3 GPIO Configuration
 PB3     ------> SPI3_SCK
 PB4     ------> SPI3_MISO
 PB5     ------> SPI3_MOSI
 */
#define SPI3_SCK_PIN                            GPIO_PIN_3
#define SPI3_SCK_GPIO_PORT                      GPIOB
#define __SPI3_SCK_PIN_CLK_ENABLE()             __HAL_RCC_GPIOB_CLK_ENABLE()

#define SPI3_MISO_PIN                           GPIO_PIN_4
#define SPI3_MISO_GPIO_PORT                     GPIOB
#define __SPI3_MISO_PIN_CLK_ENABLE()            __HAL_RCC_GPIOB_CLK_ENABLE()

#define SPI3_MOSI_PIN                           GPIO_PIN_5
#define SPI3_MOSI_GPIO_PORT                     GPIOB
#define __SPI3_MOSI_PIN_CLK_ENABLE()            __HAL_RCC_GPIOB_CLK_ENABLE()

#define SPI3_RX_DMA_CHANNEL                     DMA2_Channel1
#define SPI3_RX_DMA_IRQn                        DMA2_Channel1_IRQn

#define SPI3_TX_DMA_CHANNEL                     DMA2_Channel2
#define SPI3_TX_DMA_IRQn                        DMA2_Channel2_IRQn

#define __SPI3_DMA_CLK_ENABLE()                __HAL_RCC_DMA2_CLK_ENABLE()

#define I2C1_SCL_PIN                           GPIO_PIN_6
#define I2C1_SCL_GPIO_PORT                     GPIOB
#define __I2C1_SCL_PIN_CLK_ENABLE()            __HAL_RCC_GPIOB_CLK_ENABLE()

#define I2C1_SDA_PIN                           GPIO_PIN_7
#define I2C1_SDA_GPIO_PORT                     GPIOB
#define __I2C1_SDA_PIN_CLK_ENABLE()            __HAL_RCC_GPIOB_CLK_ENABLE()

#define I2C1_RX_DMA_CHANNEL                    DMA2_Channel3
#define I2C1_RX_DMA_IRQn                       DMA2_Channel3_IRQn

#define I2C1_TX_DMA_CHANNEL                    DMA2_Channel4
#define I2C1_TX_DMA_IRQn                       DMA2_Channel4_IRQn

#define __I2C1_DMA_CLK_ENABLE()                __HAL_RCC_DMA2_CLK_ENABLE()

#define I2C3_SCL_PIN                           GPIO_PIN_7
#define I2C3_SCL_GPIO_PORT                     GPIOG
#define __I2C3_SCL_PIN_CLK_ENABLE()            __HAL_RCC_GPIOG_CLK_ENABLE()

#define I2C3_SDA_PIN                           GPIO_PIN_8
#define I2C3_SDA_GPIO_PORT                     GPIOG
#define __I2C3_SDA_PIN_CLK_ENABLE()            __HAL_RCC_GPIOG_CLK_ENABLE()

#define I2C3_RX_DMA_CHANNEL                    DMA1_Channel3
#define I2C3_RX_DMA_IRQn                       DMA1_Channel3_IRQn

#define I2C3_TX_DMA_CHANNEL                    DMA1_Channel4
#define I2C3_TX_DMA_IRQn                       DMA1_Channel4_IRQn

#define __I2C3_DMA_CLK_ENABLE()                __HAL_RCC_DMA1_CLK_ENABLE()

void SM_Peripheral_Init(void);
void SM_OS_Init(void);
void SM_Error_Handler(void);
void SM_TIM_Start(void);
void SM_TIM_Stop(void);
double SM_GetTimeStamp(void);
double SM_GetTimeStamp_fromISR(void);

int32_t SM_SPI1_Read(void *handle, uint8_t reg, uint8_t *data, uint16_t len);
int32_t SM_SPI1_Write(void *handle, uint8_t reg, uint8_t *data, uint16_t len);
int32_t SM_SPI1_Read_Os(void *handle, uint8_t reg, uint8_t *data, uint16_t len);
int32_t SM_SPI1_Write_Os(void *handle, uint8_t reg, uint8_t *data, uint16_t len);

int32_t SM_SPI3_Read(void *handle, uint8_t reg, uint8_t *data, uint16_t len);
int32_t SM_SPI3_Write(void *handle, uint8_t reg, uint8_t *data, uint16_t len);
int32_t SM_SPI3_Read_Os(void *handle, uint8_t reg, uint8_t *data, uint16_t len);
int32_t SM_SPI3_Write_Os(void *handle, uint8_t reg, uint8_t *data, uint16_t len);

int32_t SM_I2C1_Read(void *handle, uint8_t reg, uint8_t *data, uint16_t len);
int32_t SM_I2C1_Write(void *handle, uint8_t reg, uint8_t *data, uint16_t len);
int32_t SM_I2C1_Read_Os(void *handle, uint8_t reg, uint8_t *data, uint16_t len);
int32_t SM_I2C1_Write_Os(void *handle, uint8_t reg, uint8_t *data, uint16_t len);

int32_t SM_I2C3_Read(void *handle, uint8_t reg, uint8_t *data, uint16_t len);
int32_t SM_I2C3_Write(void *handle, uint8_t reg, uint8_t *data, uint16_t len);
int32_t SM_I2C3_Read_Os(void *handle, uint8_t reg, uint8_t *data, uint16_t len);
int32_t SM_I2C3_Write_Os(void *handle, uint8_t reg, uint8_t *data, uint16_t len);

uint8_t SM_StopSensorAcquisition(void);
uint8_t SM_StartSensorThread(uint8_t sensorId);
uint8_t SM_StopSensorThread(uint8_t sensorId);

#ifdef __cplusplus
}
#endif

#endif /* __SENSORS_MANAGER_H */

