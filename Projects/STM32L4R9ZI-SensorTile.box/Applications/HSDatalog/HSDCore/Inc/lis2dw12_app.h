/**
  ******************************************************************************
  * @file    lis2dw12_app.h
  * @brief   Header for lis2dw12_app.c module.
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
#ifndef __LIS2DW12_APP_H
#define __LIS2DW12_APP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include "sensors_manager.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define LIS2DW12_SPI_CS_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOE_CLK_ENABLE()

#define LIS2DW12_SPI_CS_Pin                 GPIO_PIN_11
#define LIS2DW12_SPI_CS_GPIO_Port           GPIOE

#define LIS2DW12_INT1_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOC_CLK_ENABLE()

#define LIS2DW12_INT1_Pin                   GPIO_PIN_5
#define LIS2DW12_INT1_GPIO_Port             GPIOC
#define LIS2DW12_INT1_EXTI_LINE             EXTI_LINE_5
#define LIS2DW12_INT1_EXTI_IRQn             EXTI9_5_IRQn

#define LIS2DW12_INT2_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOD_CLK_ENABLE()

#define LIS2DW12_INT2_Pin                   GPIO_PIN_14
#define LIS2DW12_INT2_GPIO_Port             GPIOD
#define LIS2DW12_INT2_EXTI_LINE             EXTI_LINE_14
#define LIS2DW12_INT2_EXTI_IRQn             EXTI15_10_IRQn

/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern SM_Init_Param_t LIS2DW12_Init_Param;
extern EXTI_HandleTypeDef lis2dw12_exti;

/* Exported functions ------------------------------------------------------- */
void LIS2DW12_Peripheral_Init(void);
void LIS2DW12_OS_Init(void);
void LIS2DW12_Data_Ready(uint8_t subSensorId, uint8_t *buf, uint16_t size, double timeStamp);
void LIS2DW12_Set_ODR(float newODR);
void LIS2DW12_Set_FS(float newFS1, float newFS2);
void LIS2DW12_Start(void);
void LIS2DW12_Stop(void);
uint8_t LIS2DW12_Create_Sensor(const SM_Init_Param_t *pxParams);
uint8_t LIS2DW12_Get_Id(void);

#ifdef __cplusplus
}
#endif

#endif /* __LIS2DW12_APP_H */

