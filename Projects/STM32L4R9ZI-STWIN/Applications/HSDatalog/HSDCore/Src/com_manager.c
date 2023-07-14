/**
  ******************************************************************************
  * @file    com_manager.c
  * @author  SRA - MCD
  *
  *
  * @brief   This file provides a set of functions to handle the COM structure
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

/* Includes ------------------------------------------------------------------*/
#include "HSDCore.h"
#include "com_manager.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"

volatile COM_Device_t COM_device = {{0, 0, 0, 0}};
volatile COM_AcquisitionDescriptor_t COM_acquisition_descriptor;
volatile uint8_t com_status = HS_DATALOG_IDLE;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

#define SATURAH(A,B) (((A)<=(B))?(A):(B))

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief Add Sensor to Db
  * @param None
  * @retval Sensor unique sID
  */
int32_t COM_AddSensor(void)
{
  uint32_t ii = COM_device.deviceDescriptor.nSensor;

  COM_device.sensors[ii] = calloc(1, sizeof(COM_Sensor_t));

  if (COM_device.sensors[ii] == NULL)
  {
    return -1;
  }

  COM_device.sensors[ii]->sensorDescriptor.id = ii;
  COM_device.deviceDescriptor.nSensor++;
  return COM_device.deviceDescriptor.nSensor - 1;
}

/**
  * @brief Get Device Struct
  * @param None
  * @retval whole device Structure
  */
COM_Device_t *COM_GetDevice(void)
{
  return (COM_Device_t *) &COM_device;
}

/**
  * @brief Get Device Descriptor
  * @param None
  * @retval Device Descriptor
  */
COM_DeviceDescriptor_t *COM_GetDeviceDescriptor(void)
{
  return (COM_DeviceDescriptor_t *) & (COM_device.deviceDescriptor);
}

/**
  * @brief Get Sensor
  * @param sID Sensor unique ID
  * @retval Sensor
  */
COM_Sensor_t *COM_GetSensor(uint8_t sID)
{
  return COM_device.sensors[sID];
}

/**
  * @brief Get Sensor Descriptor
  * @param sID Sensor unique ID
  * @retval Sensor Descriptor
  */
COM_SensorDescriptor_t *COM_GetSensorDescriptor(uint8_t sID)
{
  return &(COM_device.sensors[sID]->sensorDescriptor);
}

/**
  * @brief Get Sensor Status
  * @param sID Sensor unique ID
  * @retval Sensor Status
  */
COM_SensorStatus_t *COM_GetSensorStatus(uint8_t sID)
{
  return &(COM_device.sensors[sID]->sensorStatus);
}

/**
  * @brief Get Tag class List
  * @param None
  * @retval Tag class List
  */
COM_TagList_t *COM_GetTagList(void)
{
  return (COM_TagList_t *) & (COM_device.tagList);
}

/**
  * @brief Get Acquisition Struct
  * @param None
  * @retval whole device Structure
  */
COM_AcquisitionDescriptor_t *COM_GetAcquisitionDescriptor(void)
{
  return (COM_AcquisitionDescriptor_t *) &COM_acquisition_descriptor;
}

/**
  * @brief Set Sensor Config
  * @param name Acquisition name
  * @param description Acquisition description
  * @retval None
  */
void COM_SetAcquisitionDescriptor(char *name, char *description)
{
  uint32_t size;

  size = strlen(name);
  size = SATURAH(size, sizeof(COM_acquisition_descriptor.name) - 1);
  memcpy((char *) COM_acquisition_descriptor.name, name, size);
  COM_acquisition_descriptor.name[size] = '\0';

  size = strlen(description);
  size = SATURAH(size, sizeof(COM_acquisition_descriptor.description) - 1);
  memcpy((char *) COM_acquisition_descriptor.description, description, size);
  COM_acquisition_descriptor.description[size] = '\0';
}

/**
  * @brief Set Device Alias
  * @param alias
  * @retval None
  */
void COM_SetDeviceAlias(char *alias)
{
  uint32_t size;

  size = strlen(alias);
  size = SATURAH(size, sizeof(COM_device.deviceDescriptor.alias) - 1);
  memcpy((char *) COM_device.deviceDescriptor.alias, alias, size);
  COM_device.deviceDescriptor.alias[size] = '\0';
}

/**
  * @brief Set BLE MAC address
  * @param alias
  * @retval None
  */
void COM_SetBleMacAddress(uint8_t *bleMacAddress)
{
  uint32_t size;
  char mac_string[18];

  sprintf(mac_string, "%x:%x:%x:%x:%x:%x", bleMacAddress[5], bleMacAddress[4],
          bleMacAddress[3], bleMacAddress[2], bleMacAddress[1], bleMacAddress[0]);
  size = strlen(mac_string);
  size = SATURAH(size, sizeof(COM_device.deviceDescriptor.bleMacAddress) - 1);
  memcpy((char*) COM_device.deviceDescriptor.bleMacAddress, mac_string, size);
  COM_device.deviceDescriptor.bleMacAddress[size] = '\0';
}

/**
  * @brief Set Sensor Config
  * @param sID Sensor unique ID
  * @param source Input sensor status
  * @retval None
  */
void COM_SetSensorStatus(uint8_t sID, COM_SensorStatus_t *source)
{
  memcpy(&(COM_device.sensors[sID]->sensorStatus), source, sizeof(COM_SensorStatus_t));
}

/**
  * @brief Get Sensor Descriptor
  * @param sID Sensor unique ID
  * @param ssID SubSensor unique ID
  * @retval SubSensor Descriptor
  */
COM_SubSensorDescriptor_t *COM_GetSubSensorDescriptor(uint8_t sID, uint8_t ssID)
{
  return &(COM_device.sensors[sID]->sensorDescriptor.subSensorDescriptor[ssID]);
}

/**
  * @brief Get Sensor Status
  * @param sID Sensor unique ID
  * @param ssID SubSensor unique ID
  * @retval SubSensor Status
  */
COM_SubSensorStatus_t *COM_GetSubSensorStatus(uint8_t sID, uint8_t ssID)
{
  return &(COM_device.sensors[sID]->sensorStatus.subSensorStatus[ssID]);
}

/**
  * @brief Get SubSensor Context
  * @param sID Sensor unique ID
  * @param ssID SubSensor unique ID
  * @retval SubSensor Context
  */
COM_SubSensorContext_t *COM_GetSubSensorContext(uint8_t sID, uint8_t ssID)
{
  return &(COM_GetSubSensorStatus(sID, ssID)->context);
}

/**
  * @brief Reset SubSensor Context
  * @param sID Sensor unique ID
  * @param ssID SubSensor unique ID
  * @retval None
  */
void COM_ResetSubSensorContext(uint8_t sID, uint8_t ssID)
{
  COM_SubSensorContext_t *pSubSensorContext = COM_GetSubSensorContext(sID, ssID);

  pSubSensorContext->n_samples_acc = 0.0f;
  pSubSensorContext->old_time_stamp = 0.0;
  pSubSensorContext->first_dataReady = 1;
  pSubSensorContext->n_samples_to_timestamp = 0;
}

/**
  * @brief Generate and store the Acquisition UUID
  * @param None
  * @retval None
  */
void COM_GenerateAcquisitionUUID(void)
{
  char *pUUID_s = (char *) COM_acquisition_descriptor.UUIDAcquisition;

  uint32_t UUID[4];
  uint8_t *p8 = (uint8_t *) UUID;

  UUID[0] = (uint32_t) rand();
  UUID[1] = (uint32_t) rand();
  UUID[2] = (uint32_t) rand();
  UUID[3] = (uint32_t) rand();

  /*
   * UUID format
   * xxxxxxxx-xxxx-Mxxx-Nxxx-xxxxxxxxxxxx
   * M: The four bits of digit M are the UUID version --> 4 is for random number
   * N: 1 to 3 most significant bits of digit N represent UUID variant --> using Variant 1 (10x)
   * */
  p8[5] = 0x40 | (p8[5] & 0xf);
  p8[11] = 0x80 | (p8[11] & 0x3f); /* in variant 1, 3rd bit can be ignored */

  pUUID_s += sprintf(pUUID_s, "%08lx-", UUID[0]);
  pUUID_s += sprintf(pUUID_s, "%04lx-%04lx-", (UUID[1] >> 16) & 0xFFFF, UUID[1] & 0xFFFF);
  pUUID_s += sprintf(pUUID_s, "%04lx-%04lx", (UUID[2] >> 16) & 0xFFFF, UUID[2] & 0xFFFF);
  pUUID_s += sprintf(pUUID_s, "%08lx", UUID[3]);

  strcpy((char *) COM_device.UUIDAcquisition, (char *) COM_acquisition_descriptor.UUIDAcquisition);
}

/**
  * @brief Get Sensor Status
  * @param sID Sensor unique ID
  * @retval Number of subsensors
  */
uint8_t COM_GetSubSensorNumber(uint8_t sID)
{
  return COM_device.sensors[sID]->sensorDescriptor.nSubSensors;
}

uint8_t COM_IsFsLegal(float value, uint8_t sID, uint8_t ssID)
{
  uint16_t i = 0;
  uint8_t ret = 0;
  float *list = COM_device.sensors[sID]->sensorDescriptor.subSensorDescriptor[ssID].FS;
  while (list[i] != COM_END_OF_LIST_FLOAT)
  {
    if (list[i] == value)
    {
      ret = 1;
    }
    i++;
  }

  return ret;
}

uint32_t COM_GetnBytesPerSample(uint8_t sID, uint8_t ssID)
{
  COM_SubSensorDescriptor_t *pSubSensorDescriptor = COM_GetSubSensorDescriptor(sID, ssID);
  if (pSubSensorDescriptor->dataType == DATA_TYPE_FLOAT || pSubSensorDescriptor->dataType == DATA_TYPE_INT32
      || pSubSensorDescriptor->dataType == DATA_TYPE_UINT32)
  {
    return pSubSensorDescriptor->dimensions * 4;
  }
  else if (pSubSensorDescriptor->dataType == DATA_TYPE_UINT16 || pSubSensorDescriptor->dataType == DATA_TYPE_INT16)
  {
    return pSubSensorDescriptor->dimensions * 2;
  }
  else if (pSubSensorDescriptor->dataType == DATA_TYPE_UINT8 || pSubSensorDescriptor->dataType == DATA_TYPE_INT8)
  {
    return pSubSensorDescriptor->dimensions;
  }
  else
  {
    return 0;
  }
}

uint8_t COM_IsOdrLegal(float value, uint8_t sID, uint8_t ssID)
{
  uint16_t i = 0;
  uint8_t ret = 0;
  float *list = COM_device.sensors[sID]->sensorDescriptor.subSensorDescriptor[ssID].ODR;
  while (list[i] != COM_END_OF_LIST_FLOAT)
  {
    if (list[i] == value)
    {
      ret = 1;
    }
    i++;
  }

  return ret;
}

