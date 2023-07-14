/**
******************************************************************************
* @file    SensorTile.box_env_sensors_ex.h
* @author  SRA - Central Labs
* @version V1.3.5
* @date    10-Feb-2022
* @brief   This file contains definitions for the BSP Environment Sensors Extended interface
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
******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SENSORTILE_BOX_ENV_SENSORS_EX_H__
#define __SENSORTILE_BOX_ENV_SENSORS_EX_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "SensorTile.box_env_sensors.h"


int32_t BSP_ENV_SENSOR_Enable_DRDY_Interrupt(uint32_t Instance);
int32_t BSP_ENV_SENSOR_Set_PowerMode(uint32_t Instance, uint8_t powerMode);
int32_t BSP_ENV_SENSOR_Set_FilterMode(uint32_t Instance, uint8_t filterMode);
int32_t BSP_ENV_SENSOR_Set_One_Shot(uint32_t Instance);
int32_t BSP_ENV_SENSOR_Get_One_Shot_Status(uint32_t Instance, uint8_t *Status);
int32_t BSP_ENV_SENSOR_Get_DRDY_Status(uint32_t Instance, uint32_t Function, uint8_t *Status);
int32_t BSP_ENV_SENSOR_Read_Register(uint32_t Instance, uint8_t Reg, uint8_t *Data);
int32_t BSP_ENV_SENSOR_Write_Register(uint32_t Instance, uint8_t Reg, uint8_t Data);


#ifdef __cplusplus
}
#endif

#endif /* __SENSORTILE_BOX_ENV_SENSORS_EX_H__ */
