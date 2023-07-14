/**
  ******************************************************************************
  * @file    lis3dhh_app.h
  * @brief   Header for lis3dhh_app.c module.
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
#ifndef __LIS3DHH_APP_H
#define __LIS3DHH_APP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include "sensors_manager.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define LIS3DHH_SPI_CS_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOE_CLK_ENABLE()

#define LIS3DHH_SPI_CS_Pin                  GPIO_PIN_10
#define LIS3DHH_SPI_CS_GPIO_Port            GPIOE

#define LIS3DHH_INT1_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOC_CLK_ENABLE()

#define LIS3DHH_INT1_Pin                    GPIO_PIN_13
#define LIS3DHH_INT1_GPIO_Port              GPIOC
#define LIS3DHH_INT1_EXTI_LINE              EXTI_LINE_13
#define LIS3DHH_INT1_EXTI_IRQn              EXTI15_10_IRQn

#define LIS3DHH_INT2_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOE_CLK_ENABLE()

#define LIS3DHH_INT2_Pin                    GPIO_PIN_6
#define LIS3DHH_INT2_GPIO_Port              GPIOE
#define LIS3DHH_INT2_EXTI_LINE              EXTI_LINE_6
#define LIS3DHH_INT2_EXTI_IRQn              EXTI9_5_IRQn

/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern SM_Init_Param_t LIS3DHH_Init_Param;
extern EXTI_HandleTypeDef lis3dhh_exti;

/* Exported functions ------------------------------------------------------- */
void LIS3DHH_Peripheral_Init(void);
void LIS3DHH_OS_Init(void);
void LIS3DHH_Data_Ready(uint8_t subSensorId, uint8_t *buf, uint16_t size, double timeStamp);
void LIS3DHH_Set_ODR(float newODR);
void LIS3DHH_Set_FS(float newFS1, float newFS2);
void LIS3DHH_Start(void);
void LIS3DHH_Stop(void);
uint8_t LIS3DHH_Create_Sensor(const SM_Init_Param_t *pxParams);
uint8_t LIS3DHH_Get_Id(void);

#ifdef __cplusplus
}
#endif

#endif /* __LIS3DHH_APP_H */

