/**
  ******************************************************************************
  * @file    iis2dh_app.h
  * @author  SRA - MCD
  *
  *
  * @brief   Header for iis2dh_app.c module.
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
#ifndef __IIS2DH_APP_H
#define __IIS2DH_APP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include "sensors_manager.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define IIS2DH_SPI_CS_Pin             GPIO_PIN_15
#define IIS2DH_SPI_CS_GPIO_Port       GPIOD

#define IIS2DH_INT2_Pin               GPIO_PIN_2
#define IIS2DH_INT2_GPIO_Port         GPIOA
#define IIS2DH_INT2_EXTI_IRQn         EXTI2_IRQn

/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern EXTI_HandleTypeDef iis2dh_exti;
extern osThreadId IIS2DH_Thread_Id;
extern SM_Init_Param_t IIS2DH_Init_Param;

/* Exported functions ------------------------------------------------------- */
void IIS2DH_Peripheral_Init(void);
void IIS2DH_OS_Init(void);
void IIS2DH_Data_Ready(uint8_t subSensorId, uint8_t *buf, uint16_t size, double timeStamp);
void IIS2DH_Set_ODR(float newODR);
void IIS2DH_Set_FS(float newFS1, float newFS2);
void IIS2DH_Start(void);
void IIS2DH_Stop(void);
uint8_t IIS2DH_Create_Sensor(const SM_Init_Param_t *pxParams);
uint8_t IIS2DH_Get_Id(void);

#ifdef __cplusplus
}
#endif

#endif /* __IIS2DH_APP_H */


