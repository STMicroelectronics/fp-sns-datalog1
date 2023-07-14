/**
  ******************************************************************************
  * @file    iis2mdc_app.h
  * @author  SRA - MCD
  *
  *
  * @brief   Header for iis2mdc_app.c module.
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
#ifndef __IIS2MDC_APP_H
#define __IIS2MDC_APP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include "sensors_manager.h"
#include "iis2mdc_reg.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define IIS2MDC_INT1_Pin GPIO_PIN_9
#define IIS2MDC_INT1_GPIO_Port GPIOF
#define IIS2MDC_INT1_EXTI_IRQn EXTI9_5_IRQn

#define FFT_LEN_MAG                            (uint32_t)(32)
#define N_AVERAGE_MAG                            (int)(1)
#define OVLP_MAG                                (float)(0.25)

/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern EXTI_HandleTypeDef iis2mdc_exti;
extern SM_Init_Param_t IIS2MDC_Init_Param;

/* Exported functions ------------------------------------------------------- */
void IIS2MDC_Peripheral_Init(void);
void IIS2MDC_OS_Init(void);
void IIS2MDC_Data_Ready(uint8_t subSensorId, uint8_t *buf, uint16_t size, double timeStamp);
void IIS2MDC_Set_ODR(float newODR);
void IIS2MDC_Set_FS(float newFS1, float newFS2);
void IIS2MDC_Start(void);
void IIS2MDC_Stop(void);
uint8_t IIS2MDC_Create_Sensor(const SM_Init_Param_t *pxParams);
uint8_t IIS2MDC_Get_Id(void);

#ifdef __cplusplus
}
#endif

#endif /* __IIS2MDC_APP_H */


