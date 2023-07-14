/**
  ******************************************************************************
  * @file    ism330dhcx_app.h
  * @author  SRA - MCD
  *
  *
  * @brief   Header
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
#ifndef __ISM330DHCX_H
#define __ISM330DHCX_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "sensors_manager.h"
#include "ism330dhcx_reg.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define ISM330DHCX_MAX_DRDY_PERIOD           (1.0)    /* seconds */

#ifndef ISM330DHCX_MAX_WTM_LEVEL
#define ISM330DHCX_MAX_WTM_LEVEL             (256)    /* samples */
#endif /* ISM330DHCX_MAX_WTM_LEVEL */

#define ISM330DHCX_MIN_WTM_LEVEL             (16)     /* samples */
#define ISM330DHCX_MAX_SAMPLES_PER_IT        (ISM330DHCX_MAX_WTM_LEVEL)

#define ISM330DHCX_SPI_CS_Pin                GPIO_PIN_13
#define ISM330DHCX_SPI_CS_GPIO_Port          GPIOF
#define ISM330DHCX_SPI_CS_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOF_CLK_ENABLE()

#define ISM330DHCX_INT1_Pin                  GPIO_PIN_8
#define ISM330DHCX_INT1_GPIO_Port            GPIOE
#define ISM330DHCX_INT1_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOE_CLK_ENABLE()
#define ISM330DHCX_INT1_EXTI_IRQn            EXTI9_5_IRQn
#define ISM330DHCX_INT1_EXTI_LINE            EXTI_LINE_8

#define ISM330DHCX_INT2_Pin                  GPIO_PIN_4
#define ISM330DHCX_INT2_GPIO_Port            GPIOF
#define ISM330DHCX_INT2_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOF_CLK_ENABLE()
#define ISM330DHCX_INT2_EXTI_IRQn            EXTI4_IRQn
#define ISM330DHCX_INT2_EXTI_LINE            EXTI_LINE_4

/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern EXTI_HandleTypeDef ism330dhcx_exti;
extern EXTI_HandleTypeDef mlc_exti;
extern SM_Init_Param_t ISM330DHCX_Init_Param;
extern sensor_handle_t ism330dhcx_hdl_instance;

/* Exported functions ------------------------------------------------------- */
void ISM330DHCX_Peripheral_Init(void);
void ISM330DHCX_OS_Init(void);
void ISM330DHCX_Data_Ready(uint8_t subSensorId, uint8_t *buf, uint16_t size, double timeStamp);
void ISM330DHCX_Set_ODR(float newODR);
void ISM330DHCX_Set_FS(float newFS1, float newFS2);
void ISM330DHCX_Start(void);
void ISM330DHCX_Stop(void);
uint8_t ISM330DHCX_Create_Sensor(const SM_Init_Param_t *pxParams);
uint8_t ISM330DHCX_Get_Id(void);

void ISM330DHCX_SetUCF(uint32_t mlcConfigSize, char *mlcConfigData);
int32_t ISM330DHCX_GetUCF_FromBuffer(char *buffer_in, uint32_t size_in, char *buffer_out, uint32_t size_out);

uint8_t ISM330DHCX_updateConfig(void);

#ifdef __cplusplus
}
#endif

#endif /* __ISM330DHCX_H */


