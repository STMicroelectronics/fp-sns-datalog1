/**
  ******************************************************************************
  * @file    sensors_manager.h
  * @author  SRA - MCD
  *
  *
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

extern DMA_HandleTypeDef hdma_sm_i2c_rx;
extern DMA_HandleTypeDef hdma_sm_i2c_tx;
extern I2C_HandleTypeDef hsm_i2c;
extern DMA_HandleTypeDef hdma_sm_spi_tx;
extern DMA_HandleTypeDef hdma_sm_spi_rx;
extern TIM_HandleTypeDef hsm_tim;


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



/**SPI3 GPIO Configuration
    PB4 (NJTRST)     ------> SPI3_MISO
    PB5     ------> SPI3_MOSI
    PB3 (JTDO/TRACESWO)     ------> SPI3_SCK
    */

#define SM_SPI_x                               SPI3
#define SM_SPIx_CLK_ENABLE()                   __HAL_RCC_SPI3_CLK_ENABLE()

#define SM_SPI_CLK_PIN                         GPIO_PIN_3
#define SM_SPI_CLK_GPIO_PORT                   GPIOB
#define SM_SPI_CLK_PIN_CLK_ENABLE()            __HAL_RCC_GPIOB_CLK_ENABLE()
#define SM_SPI_CLK_AF                          GPIO_AF6_SPI3

#define SM_SPI_MISO_PIN                         GPIO_PIN_4
#define SM_SPI_MISO_GPIO_PORT                   GPIOB
#define SM_SPI_MISO_PIN_CLK_ENABLE()            __HAL_RCC_GPIOB_CLK_ENABLE()
#define SM_SPI_MISO_AF                          GPIO_AF6_SPI3

#define SM_SPI_MOSI_PIN                         GPIO_PIN_5
#define SM_SPI_MOSI_GPIO_PORT                   GPIOB
#define SM_SPI_MOSI_PIN_CLK_ENABLE()            __HAL_RCC_GPIOB_CLK_ENABLE()
#define SM_SPI_MOSI_AF                          GPIO_AF6_SPI3

#define SM_SPI_RX_DMA_CHANNEL                   DMA1_Channel1
#define SM_SPI_RX_DMA_REQUEST                   DMA_REQUEST_SPI3_RX
#define SM_SPI_RX_DMA_IRQn                      DMA1_Channel1_IRQn

#define SM_SPI_TX_DMA_CHANNEL                   DMA1_Channel2
#define SM_SPI_TX_DMA_REQUEST                   DMA_REQUEST_SPI3_TX
#define SM_SPI_TX_DMA_IRQn                      DMA1_Channel2_IRQn

#define SM_SPIx_DMA_CLK_ENABLE()                __HAL_RCC_DMA1_CLK_ENABLE()

#define SM_I2C_x                               I2C2
#define SM_I2Cx_CLK_ENABLE()                   __HAL_RCC_I2C2_CLK_ENABLE()

#define SM_I2C_SCL_PIN                         GPIO_PIN_1
#define SM_I2C_SCL_GPIO_PORT                   GPIOF
#define SM_I2C_SCL_PIN_CLK_ENABLE()            __HAL_RCC_GPIOF_CLK_ENABLE()
#define SM_I2C_SCL_AF                          GPIO_AF4_I2C2

#define SM_I2C_SDA_PIN                         GPIO_PIN_0
#define SM_I2C_SDA_GPIO_PORT                   GPIOF
#define SM_I2C_SDA_PIN_CLK_ENABLE()            __HAL_RCC_GPIOF_CLK_ENABLE()
#define SM_I2C_SDA_AF                          GPIO_AF4_I2C2

#define SM_I2C_RX_DMA_CHANNEL                   DMA1_Channel3
#define SM_I2C_RX_DMA_REQUEST                   DMA_REQUEST_I2C2_RX
#define SM_I2C_RX_DMA_IRQn                      DMA1_Channel3_IRQn

#define SM_I2C_TX_DMA_CHANNEL                   DMA1_Channel4
#define SM_I2C_TX_DMA_REQUEST                   DMA_REQUEST_I2C2_TX
#define SM_I2C_TX_DMA_IRQn                      DMA1_Channel4_IRQn

#define SM_I2Cx_DMA_CLK_ENABLE()                __HAL_RCC_DMA1_CLK_ENABLE()

#define SM_TIMx                           TIM5
#define SM_TIMx_CLK_ENABLE()              __HAL_RCC_TIM5_CLK_ENABLE()

void SM_IT_Init(void);
void SM_Peripheral_Init(void);
void SM_OS_Init(void);
void SM_Error_Handler(void);
void SM_TIM_Init(void);
void SM_TIM_Start(void);
void SM_TIM_Stop(void);
double SM_GetTimeStamp(void);
double SM_GetTimeStamp_fromISR(void);

int32_t SM_SPI_Read(void *handle, uint8_t reg, uint8_t *data, uint16_t len);
int32_t SM_SPI_Write(void *handle, uint8_t reg, uint8_t *data, uint16_t len);
int32_t SM_SPI_Read_Os(void *handle, uint8_t reg, uint8_t *data, uint16_t len);
int32_t SM_SPI_Write_Os(void *handle, uint8_t reg, uint8_t *data, uint16_t len);

int32_t SM_I2C_Read(void *handle, uint8_t reg, uint8_t *data, uint16_t len);
int32_t SM_I2C_Write(void *handle, uint8_t reg, uint8_t *data, uint16_t len);
int32_t SM_I2C_Read_Os(void *handle, uint8_t reg, uint8_t *data, uint16_t len);
int32_t SM_I2C_Write_Os(void *handle, uint8_t reg, uint8_t *data, uint16_t len);

uint32_t SM_SubSensors_DataPartitioning(uint8_t *p, uint32_t nSamples, uint32_t sampleSize, uint8_t tag0_value);

uint8_t SM_StopSensorAcquisition(void);
uint8_t SM_StartSensorThread(uint8_t id);
uint8_t SM_StopSensorThread(uint8_t id);


#ifdef __cplusplus
}
#endif

#endif /* __SENSORS_MANAGER_H */


