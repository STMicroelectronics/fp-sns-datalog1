/**
  ******************************************************************************
  * @file    SensorTile.box_conf.h
  * @brief   This file contains definitions for the components bus interfaces
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
#ifndef __SENSORTILE_BOX_CONF_H__
#define __SENSORTILE_BOX_CONF_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "SensorTile.box_bus.h"
#include "SensorTile.box_errno.h"

#define USE_AUDIO_IN                    1U
#define AUDIO_SAMPLING_FREQUENCY        16000

/* Battery Charger */

/* Define 1 to use already implemented callback; 0 to implement callback
 into an application file */

#define USE_BC_TIM_IRQ_CALLBACK         1U
#define USE_BC_CHG_IRQ_HANDLER          0U
#define USE_BC_IRQ_CALLBACK             0U

/* lis2dw12 */
#define BSP_LIS2DW12_CS_GPIO_CLK_ENABLE() __GPIOE_CLK_ENABLE()
#define BSP_LIS2DW12_CS_PORT GPIOE
#define BSP_LIS2DW12_CS_PIN GPIO_PIN_11

/* lis3dhh */
#define BSP_LIS3DHH_CS_GPIO_CLK_ENABLE() __GPIOE_CLK_ENABLE()
#define BSP_LIS3DHH_CS_PORT GPIOE
#define BSP_LIS3DHH_CS_PIN GPIO_PIN_10

#define BSP_LIS3DHH_INT1_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOE_CLK_ENABLE()
#define BSP_LIS3DHH_INT1_PORT               GPIOE
#define BSP_LIS3DHH_INT1_PIN                GPIO_PIN_6
#define BSP_LIS3DHH_INT1_EXTI_IRQn          EXTI9_5_IRQn
#define BSP_LIS3DHH_INT1_EXTI_IRQ_PP        1
#define BSP_LIS3DHH_INT1_EXTI_IRQ_SP        0
#define BSP_LIS3DHH_INT1_EXTI_IRQHandler    EXTI9_5_IRQHandler

#define BSP_LIS3DHH_INT2_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOC_CLK_ENABLE()
#define BSP_LIS3DHH_INT2_PORT               GPIOC
#define BSP_LIS3DHH_INT2_PIN                GPIO_PIN_13
#define BSP_LIS3DHH_INT2_EXTI_IRQn          EXTI15_10_IRQn
#define BSP_LIS3DHH_INT2_EXTI_IRQ_PP        1
#define BSP_LIS3DHH_INT2_EXTI_IRQ_SP        0
#define BSP_LIS3DHH_INT2_EXTI_IRQHandler    EXTI15_10_IRQHandler

/* LSM6DSOX */
#define BSP_LSM6DSOX_CS_GPIO_CLK_ENABLE() __GPIOE_CLK_ENABLE()
#define BSP_LSM6DSOX_CS_PORT GPIOE
#define BSP_LSM6DSOX_CS_PIN GPIO_PIN_12

#define BSP_LSM6DSOX_INT2_GPIO_CLK_ENABLE() __GPIOE_CLK_ENABLE()
#define BSP_LSM6DSOX_INT2_PORT GPIOE
#define BSP_LSM6DSOX_INT2_PIN GPIO_PIN_6
#define BSP_LSM6DSOX_INT2_EXTI_IRQn           EXTI9_5_IRQn

/* stts751 */
#define BSP_STTS751_INT_GPIO_CLK_ENABLE() __GPIOE_CLK_ENABLE()
#define BSP_STTS751_INT_PORT GPIOE
#define BSP_STTS751_INT_PIN GPIO_PIN_1
#define BSP_STTS751_INT_EXTI_IRQn           EXTI1_IRQn

#ifdef __cplusplus
}
#endif

#endif /* __SENSORTILE_BOX_CONF_H__*/

