/**
  ******************************************************************************
  * @file    imp23absu_app.h
  * @author  SRA - MCD
  *
  *
  * @brief   Header for mp23abs1_app.c module.
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
#ifndef __IMP23ABSU_APP_H
#define __IMP23ABSU_APP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include "sensors_manager.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define IMP23ABSU_MAX_SAMPLING_FREQUENCY                 (uint32_t)(192000)
#define IMP23ABSU_MS                                 (uint32_t)(1)

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif /* M_PI */

/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_dfsdm1_flt1;
extern SM_Init_Param_t IMP23ABSU_Init_Param;

/* Exported functions ------------------------------------------------------- */
void IMP23ABSU_Peripheral_Init(void);
void IMP23ABSU_OS_Init(void);
void IMP23ABSU_Data_Ready(uint8_t subSensorId, uint8_t *buf, uint16_t size, double timeStamp);
void IMP23ABSU_Set_ODR(float newODR);
void IMP23ABSU_Set_FS(float newFS1, float newFS2);
void IMP23ABSU_Start(void);
void IMP23ABSU_Stop(void);
uint8_t IMP23ABSU_Create_Sensor(const SM_Init_Param_t *pxParams);
uint8_t IMP23ABSU_Get_Id(void);

void IMP23ABSU_updateConfig(void);

#ifdef __cplusplus
}
#endif

#endif /* __IMP23ABSU_APP_H */


