/**
  ******************************************************************************
  * @file    lsm6dsox_app.h
  * @brief   Header for lsm6dsox_app.c module.
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
#ifndef __LSM6DSOX_H
#define __LSM6DSOX_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "sensors_manager.h"
#include "lsm6dsox_reg.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define LSM6DSOX_MAX_DRDY_PERIOD           (1.0)    /* seconds */

#ifndef LSM6DSOX_MAX_WTM_LEVEL
#define LSM6DSOX_MAX_WTM_LEVEL             (256)    /* samples */
#endif /*LSM6DSOX_MAX_WTM_LEVEL */

#define LSM6DSOX_MIN_WTM_LEVEL             (16)     /* samples */
#define LSM6DSOX_MAX_SAMPLES_PER_IT        (LSM6DSOX_MAX_WTM_LEVEL)

#define LSM6DSOX_SPI_CS_Pin                GPIO_PIN_12
#define LSM6DSOX_SPI_CS_GPIO_Port          GPIOE
#define LSM6DSOX_SPI_CS_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOE_CLK_ENABLE()

/*
 * HW conflict between PWR button and LSM6DSOX INT1:
 * link to TIM2_IC CHANNEL3 to use the timer IC capture callback for the sensor
 */
/*
 #define LSM6DSOX_INT1_Pin                  GPIO_PIN_2
 #define LSM6DSOX_INT1_GPIO_Port            GPIOA
 #define LSM6DSOX_INT1_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOA_CLK_ENABLE()
 #define LSM6DSOX_INT1_EXTI_IRQn            EXTI2_IRQn
 #define LSM6DSOX_INT1_EXTI_LINE            EXTI_LINE_2
 */

#define LSM6DSOX_INT2_Pin                  GPIO_PIN_3
#define LSM6DSOX_INT2_GPIO_Port            GPIOE
#define LSM6DSOX_INT2_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOE_CLK_ENABLE()
#define LSM6DSOX_INT2_EXTI_IRQn            EXTI3_IRQn
#define LSM6DSOX_INT2_EXTI_LINE            EXTI_LINE_3

/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern EXTI_HandleTypeDef lsm6dsox_exti;
extern EXTI_HandleTypeDef mlc_exti;
extern SM_Init_Param_t LSM6DSOX_Init_Param;
extern sensor_handle_t lsm6dsox_hdl_instance;

/* Exported functions ------------------------------------------------------- */
void LSM6DSOX_Peripheral_Init(void);
void LSM6DSOX_OS_Init(void);
void LSM6DSOX_Data_Ready(uint8_t subSensorId, uint8_t *buf, uint16_t size, double timeStamp);
void LSM6DSOX_Set_ODR(float newODR);
void LSM6DSOX_Set_FS(float newFS1, float newFS2);
void LSM6DSOX_Start(void);
void LSM6DSOX_Stop(void);
uint8_t LSM6DSOX_Create_Sensor(const SM_Init_Param_t *pxParams);
uint8_t LSM6DSOX_Get_Id(void);

void LSM6DSOX_SetUCF(uint32_t mlcConfigSize, char *mlcConfigData);
int32_t LSM6DSOX_GetUCF_FromBuffer(char *buffer_in, uint32_t size_in, char *buffer_out, uint32_t size_out);

uint8_t LSM6DSOX_updateConfig(void);

#ifdef __cplusplus
}
#endif

#endif /* __LSM6DSOX_H */

