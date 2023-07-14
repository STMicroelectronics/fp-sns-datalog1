/**
  ******************************************************************************
  * @file    lis2mdl_app.h
  * @brief   Header for lis2mdl_app.c module.
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
#ifndef __LIS2MDL_APP_H
#define __LIS2MDL_APP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include "sensors_manager.h"
#include "lis2mdl_reg.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define LIS2MDL_COM_SPI_3_WIRE            0
#define LIS2MDL_COM_SPI_4_WIRE            1
#define LIS2MDL_COM_I2C                   2

#define LIS2MDL_COM_MODE                  LIS2MDL_COM_SPI_4_WIRE

#define LIS2MDL_SPI_CS_Pin                GPIO_PIN_15
#define LIS2MDL_SPI_CS_GPIO_Port          GPIOA
#define LIS2MDL_SPI_CS_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()

/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern SM_Init_Param_t LIS2MDL_Init_Param;

/* Exported functions ------------------------------------------------------- */
void LIS2MDL_Peripheral_Init(void);
void LIS2MDL_OS_Init(void);
void LIS2MDL_Data_Ready(uint8_t subSensorId, uint8_t *buf, uint16_t size, double timeStamp);
void LIS2MDL_Set_ODR(float newODR);
void LIS2MDL_Set_FS(float newFS1, float newFS2);
void LIS2MDL_Start(void);
void LIS2MDL_Stop(void);
uint8_t LIS2MDL_Create_Sensor(const SM_Init_Param_t *pxParams);
uint8_t LIS2MDL_Get_Id(void);

#ifdef __cplusplus
}
#endif

#endif /* __LIS2MDL_APP_H */

