/**
  ******************************************************************************
  * @file    lps22hh_app.h
  * @brief   Header for lps22hh_app.c module.
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
#ifndef __LPS22HH_APP_H
#define __LPS22HH_APP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include "sensors_manager.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define LPS22HH_INT_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOD_CLK_ENABLE()

#define LPS22HH_INT_Pin                     GPIO_PIN_15
#define LPS22HH_INT_GPIO_Port               GPIOD
#define LPS22HH_INT_EXTI_LINE               EXTI_LINE_15
#define LPS22HH_INT_EXTI_IRQn               EXTI15_10_IRQn

/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern SM_Init_Param_t LPS22HH_Init_Param;
extern EXTI_HandleTypeDef lps22hh_exti;

/* Exported functions ------------------------------------------------------- */
void LPS22HH_Peripheral_Init(void);
void LPS22HH_OS_Init(void);
void LPS22HH_Data_Ready(uint8_t subSensorId, uint8_t *buf, uint16_t size, double timeStamp);
void LPS22HH_Start(void);
void LPS22HH_Stop(void);
uint8_t LPS22HH_Create_Sensor(const SM_Init_Param_t *pxParams);
uint8_t LPS22HH_Get_Id(void);

uint8_t LPS22HH_updateConfig(void);

#ifdef __cplusplus
}
#endif

#endif /* __LPS22HH_APP_H */

