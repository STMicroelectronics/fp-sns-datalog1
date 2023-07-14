/**
  ******************************************************************************
  * @file    com_manager.h
  * @author  SRA - MCD
  *
  *
  * @brief   Header for com_manager.c file.
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
#ifndef __DEVICE_DESCRIPTION_H
#define __DEVICE_DESCRIPTION_H

/* Includes ------------------------------------------------------------------*/
#include "com_manager.h"
/* Exported types ------------------------------------------------------------*/

typedef struct
{
  char alias[HSD_DEVICE_ALIAS_LENGTH];
  char partNumber[HSD_DEVICE_PNUMBER_LENGTH];
  char URL[HSD_DEVICE_URL_LENGTH];
  char fwName[HSD_DEVICE_FW_NAME_LENGTH];
  char fwVersion[HSD_DEVICE_FW_VERSION_LENGTH];
  char model[HSD_DEVICE_MODEL_LENGTH];
  char bleMacAddress[HSD_BLE_MAC_ADDRESS_LENGTH];
} HSD_DeviceDescriptor_Init_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void set_device_description(HSD_DeviceDescriptor_Init_t *init);
void update_sensorStatus(COM_SensorStatus_t *oldSensorStatus, COM_SensorStatus_t *newSensorStatus, uint8_t sID);
void update_sensorStatus_from_USB(COM_SensorStatus_t *oldSensorStatus, COM_SensorStatus_t *newSensorStatus,
                                  uint8_t sID);
void update_samplesPerTimestamp(COM_Sensor_t *pSensor);
uint8_t update_sensors_config(void);

#endif /* __DEVICE_DESCRIPTION_H */

