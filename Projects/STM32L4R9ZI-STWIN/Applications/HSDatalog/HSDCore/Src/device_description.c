/**
  ******************************************************************************
  * @file    device_description.c
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
#include "device_description.h"

#include "imp23absu_app.h"
#include "imp34dt05_app.h"
#include "ism330dhcx_app.h"
#include "iis3dwb_app.h"
#include "iis2mdc_app.h"
#include "iis2dh_app.h"
#include "hts221_app.h"
#include "lps22hh_app.h"
#include "stts751_app.h"

#include "sensors_manager.h"
#include "stdlib.h"
#include "string.h"
#include "stdio.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern volatile uint8_t ISM330DHCX_params_loading;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* Unique ID is directly derived from STM32 UID and converted to string
string needs to be 25bytes 24+\0  */
static void get_unique_id(char *id)
{
  uint32_t stm32_UID[3];

  stm32_UID[0] = HAL_GetUIDw0();
  stm32_UID[1] = HAL_GetUIDw1();
  stm32_UID[2] = HAL_GetUIDw2();

  sprintf(id, "%08lX%08lX%08lX", stm32_UID[0], stm32_UID[1], stm32_UID[2]);
}



/**
  * @brief Set default device description
  * @param None
  * @retval None
  */
void set_device_description(HSD_DeviceDescriptor_Init_t *init)
{
  COM_Device_t *pDevice;
  COM_DeviceDescriptor_t *pDeviceDescriptor;

  pDevice = COM_GetDevice();
  char tmp[6] =
  {
    HSD_JSON_VERSION_MAJOR,
    '.',
    HSD_JSON_VERSION_MINOR,
    '.',
    HSD_JSON_VERSION_PATCH,
    '\0'
  };
  strcpy(pDevice->JSONVersion, tmp);

  pDeviceDescriptor = COM_GetDeviceDescriptor();
  get_unique_id(pDeviceDescriptor->serialNumber);

  strcpy(pDeviceDescriptor->alias, init->alias);
  strcpy(pDeviceDescriptor->partNumber, init->partNumber);
  strcpy(pDeviceDescriptor->URL, init->URL);
  strcpy(pDeviceDescriptor->fwName, init->fwName);
  strcpy(pDeviceDescriptor->fwVersion, init->fwVersion);
  strcpy(pDeviceDescriptor->model, init->model);

  strcpy(pDeviceDescriptor->dataFileExt, HSD_DATA_FILE_EXTENSION);
  strcpy(pDeviceDescriptor->dataFileFormat, HSD_DATA_FILE_FORMAT);
  strcpy(pDeviceDescriptor->bleMacAddress, init->bleMacAddress);
}


void update_sensorStatus(COM_SensorStatus_t *oldSensorStatus, COM_SensorStatus_t *newSensorStatus, uint8_t sID)
{
  /* Check differences between oldSensorStatus and newSensorStatus, act properly*/
  uint8_t update_combo_env = 0;

  /* subsensor: FS, ODR, is Active, channel number, samplePerTimestamp*/
  for (uint8_t i = 0; i < COM_GetSubSensorNumber(sID); i++)
  {
    if (oldSensorStatus->subSensorStatus[i].FS != newSensorStatus->subSensorStatus[i].FS)
    {
      oldSensorStatus->subSensorStatus[i].FS = newSensorStatus->subSensorStatus[i].FS;
      if (sID == ISM330DHCX_Get_Id())
      {
        ISM330DHCX_params_loading += 1;
      }
    }

    if (sID == HTS221_Get_Id() || sID == LPS22HH_Get_Id())
    {
      if (i == 0)
      {
        if (oldSensorStatus->subSensorStatus[i].ODR != newSensorStatus->subSensorStatus[i].ODR)
        {
          update_combo_env += 2;
        }
      }
      if (i == 1)
      {
        if (oldSensorStatus->subSensorStatus[i].ODR != newSensorStatus->subSensorStatus[i].ODR)
        {
          update_combo_env += 1;
        }

        if (update_combo_env > 1)
        {
          oldSensorStatus->subSensorStatus[0].ODR = newSensorStatus->subSensorStatus[0].ODR;
          oldSensorStatus->subSensorStatus[1].ODR = newSensorStatus->subSensorStatus[0].ODR;
        }
        else if (update_combo_env == 1)
        {
          oldSensorStatus->subSensorStatus[0].ODR = newSensorStatus->subSensorStatus[1].ODR;
          oldSensorStatus->subSensorStatus[1].ODR = newSensorStatus->subSensorStatus[1].ODR;
        }
        else
        {
          if (newSensorStatus->subSensorStatus[1].ODR != newSensorStatus->subSensorStatus[0].ODR)
          {
            oldSensorStatus->subSensorStatus[0].ODR = newSensorStatus->subSensorStatus[0].ODR;
            oldSensorStatus->subSensorStatus[1].ODR = newSensorStatus->subSensorStatus[0].ODR;
          }
        }
      }
    }
    else
    {
      if (oldSensorStatus->subSensorStatus[i].ODR != newSensorStatus->subSensorStatus[i].ODR)
      {
        oldSensorStatus->subSensorStatus[i].ODR = newSensorStatus->subSensorStatus[i].ODR;
        if (sID == ISM330DHCX_Get_Id())
        {
          ISM330DHCX_params_loading += 1;
        }
      }
    }

    if (oldSensorStatus->subSensorStatus[i].isActive != newSensorStatus->subSensorStatus[i].isActive)
    {
      oldSensorStatus->subSensorStatus[i].isActive = newSensorStatus->subSensorStatus[i].isActive;
      if (sID == ISM330DHCX_Get_Id())
      {
        ISM330DHCX_params_loading += 1;
      }
    }

    if (oldSensorStatus->subSensorStatus[i].comChannelNumber != newSensorStatus->subSensorStatus[i].comChannelNumber)
    {
      oldSensorStatus->subSensorStatus[i].comChannelNumber = newSensorStatus->subSensorStatus[i].comChannelNumber;
    }

    if (sID == ISM330DHCX_Get_Id() && i == 2)     /* MLC subsensor: samplesPerTimestamp must be 1 */
    {
      oldSensorStatus->subSensorStatus[i].samplesPerTimestamp = 1;
    }
    else
    {
      uint32_t idealSamplesPerTS = (uint32_t)(newSensorStatus->subSensorStatus[i].ODR);
      uint32_t newSamplesPerTS = (uint32_t)(newSensorStatus->subSensorStatus[i].samplesPerTimestamp);

      if (newSamplesPerTS > idealSamplesPerTS && newSamplesPerTS != 0)
      {
        if (idealSamplesPerTS > MAX_SPTS)
        {
          oldSensorStatus->subSensorStatus[i].samplesPerTimestamp = MAX_SPTS;
        }
        else
        {
          oldSensorStatus->subSensorStatus[i].samplesPerTimestamp = idealSamplesPerTS;
        }
      }
      else
      {
        oldSensorStatus->subSensorStatus[i].samplesPerTimestamp = newSamplesPerTS;
      }
    }

    if (oldSensorStatus->subSensorStatus[i].usbDataPacketSize != newSensorStatus->subSensorStatus[i].usbDataPacketSize)
    {
      oldSensorStatus->subSensorStatus[i].usbDataPacketSize = newSensorStatus->subSensorStatus[i].usbDataPacketSize;
    }

    if (oldSensorStatus->subSensorStatus[i].sdWriteBufferSize != newSensorStatus->subSensorStatus[i].sdWriteBufferSize)
    {
      oldSensorStatus->subSensorStatus[i].sdWriteBufferSize = newSensorStatus->subSensorStatus[i].sdWriteBufferSize;
    }

    if (oldSensorStatus->subSensorStatus[i].ucfLoaded != newSensorStatus->subSensorStatus[i].ucfLoaded)
    {
      oldSensorStatus->subSensorStatus[i].ucfLoaded = newSensorStatus->subSensorStatus[i].ucfLoaded;
    }

#if (HSD_USE_DUMMY_DATA == 1)
    oldSensorStatus->subSensorStatus[i].sensitivity = 1.0f;
#endif /* (HSD_USE_DUMMY_DATA == 1) */
  }

  /* Specific cases for Sensitivity */
#if (HSD_USE_DUMMY_DATA != 1)
  if (sID == IIS3DWB_Get_Id())
  {
    oldSensorStatus->subSensorStatus[0].sensitivity = 0.0000305f *  oldSensorStatus->subSensorStatus[0].FS;
  }
  else if (sID == IIS2DH_Get_Id())
  {
    if (oldSensorStatus->subSensorStatus[0].FS == 16.0f)
    {
      oldSensorStatus->subSensorStatus[0].sensitivity = 0.00075f;
    }
    else
    {
      oldSensorStatus->subSensorStatus[0].sensitivity = 0.00003125f * oldSensorStatus->subSensorStatus[0].FS;
    }
  }
  else if (sID == ISM330DHCX_Get_Id())
  {
    oldSensorStatus->subSensorStatus[0].sensitivity = 0.0000305f *  oldSensorStatus->subSensorStatus[0].FS;
    oldSensorStatus->subSensorStatus[1].sensitivity = 0.035f * oldSensorStatus->subSensorStatus[1].FS;
  }
#endif /* (HSD_USE_DUMMY_DATA != 1) */
}

void update_sensorStatus_from_USB(COM_SensorStatus_t *oldSensorStatus, COM_SensorStatus_t *newSensorStatus, uint8_t sID)
{
  /* Check differences between oldSensorStatus and newSensorStatus, act properly*/

  /* subsensor: FS, ODR, is Active, channel number, samplePerTimestamp*/
  for (uint8_t i = 0; i < COM_GetSubSensorNumber(sID); i++)
  {
    if (oldSensorStatus->subSensorStatus[i].FS != newSensorStatus->subSensorStatus[i].FS)
    {
      oldSensorStatus->subSensorStatus[i].FS = newSensorStatus->subSensorStatus[i].FS;
      if (sID == ISM330DHCX_Get_Id())
      {
        ISM330DHCX_params_loading += 1;
      }
    }

    if (sID == HTS221_Get_Id() || sID == LPS22HH_Get_Id())
    {
      oldSensorStatus->subSensorStatus[0].ODR = newSensorStatus->subSensorStatus[0].ODR;
      oldSensorStatus->subSensorStatus[1].ODR = newSensorStatus->subSensorStatus[0].ODR;
    }
    else
    {
      if (oldSensorStatus->subSensorStatus[i].ODR != newSensorStatus->subSensorStatus[i].ODR)
      {
        oldSensorStatus->subSensorStatus[i].ODR = newSensorStatus->subSensorStatus[i].ODR;
        if (sID == ISM330DHCX_Get_Id())
        {
          ISM330DHCX_params_loading += 1;
        }
      }
    }

    if (oldSensorStatus->subSensorStatus[i].isActive != newSensorStatus->subSensorStatus[i].isActive)
    {
      oldSensorStatus->subSensorStatus[i].isActive = newSensorStatus->subSensorStatus[i].isActive;
      if (sID == ISM330DHCX_Get_Id())
      {
        ISM330DHCX_params_loading += 1;
      }
    }

    if (oldSensorStatus->subSensorStatus[i].comChannelNumber != newSensorStatus->subSensorStatus[i].comChannelNumber)
    {
      oldSensorStatus->subSensorStatus[i].comChannelNumber = newSensorStatus->subSensorStatus[i].comChannelNumber;
    }

    if (sID == ISM330DHCX_Get_Id() && i == 2)     /* MLC subsensor: samplesPerTimestamp must be 1 */
    {
      oldSensorStatus->subSensorStatus[i].samplesPerTimestamp = 1;
    }
    else
    {
      uint32_t idealSamplesPerTS = (uint32_t)(newSensorStatus->subSensorStatus[i].ODR);
      uint32_t newSamplesPerTS = (uint32_t)(newSensorStatus->subSensorStatus[i].samplesPerTimestamp);

      if (newSamplesPerTS > idealSamplesPerTS && newSamplesPerTS != 0)
      {
        if (idealSamplesPerTS > MAX_SPTS)
        {
          oldSensorStatus->subSensorStatus[i].samplesPerTimestamp = MAX_SPTS;
        }
        else
        {
          oldSensorStatus->subSensorStatus[i].samplesPerTimestamp = idealSamplesPerTS;
        }
      }
      else
      {
        oldSensorStatus->subSensorStatus[i].samplesPerTimestamp = newSamplesPerTS;
      }
    }

    if (oldSensorStatus->subSensorStatus[i].usbDataPacketSize != newSensorStatus->subSensorStatus[i].usbDataPacketSize)
    {
      oldSensorStatus->subSensorStatus[i].usbDataPacketSize = newSensorStatus->subSensorStatus[i].usbDataPacketSize;
    }

    if (oldSensorStatus->subSensorStatus[i].sdWriteBufferSize != newSensorStatus->subSensorStatus[i].sdWriteBufferSize)
    {
      oldSensorStatus->subSensorStatus[i].sdWriteBufferSize = newSensorStatus->subSensorStatus[i].sdWriteBufferSize;
    }

    if (oldSensorStatus->subSensorStatus[i].ucfLoaded != newSensorStatus->subSensorStatus[i].ucfLoaded)
    {
      oldSensorStatus->subSensorStatus[i].ucfLoaded = newSensorStatus->subSensorStatus[i].ucfLoaded;
    }

#if (HSD_USE_DUMMY_DATA == 1)
    oldSensorStatus->subSensorStatus[i].sensitivity = 1.0f;
#endif /* (HSD_USE_DUMMY_DATA == 1) */
  }

  /* Specific cases for Sensitivity */
#if (HSD_USE_DUMMY_DATA != 1)
  if (sID == IIS3DWB_Get_Id())
  {
    oldSensorStatus->subSensorStatus[0].sensitivity = 0.0000305f *  oldSensorStatus->subSensorStatus[0].FS;
  }
  else if (sID == IIS2DH_Get_Id())
  {
    if (oldSensorStatus->subSensorStatus[0].FS == 16.0f)
    {
      oldSensorStatus->subSensorStatus[0].sensitivity = 0.00075f;
    }
    else
    {
      oldSensorStatus->subSensorStatus[0].sensitivity = 0.00003125f * oldSensorStatus->subSensorStatus[0].FS;
    }
  }
  else if (sID == ISM330DHCX_Get_Id())
  {
    oldSensorStatus->subSensorStatus[0].sensitivity = 0.0000305f *  oldSensorStatus->subSensorStatus[0].FS;
    oldSensorStatus->subSensorStatus[1].sensitivity = 0.035f * oldSensorStatus->subSensorStatus[1].FS;
  }
#endif /* (HSD_USE_DUMMY_DATA != 1) */
}

void update_samplesPerTimestamp(COM_Sensor_t *pSensor)
{
  for (uint8_t sID = 0; sID < pSensor->sensorDescriptor.nSubSensors; sID++)
  {
    if (pSensor->sensorDescriptor.id == ISM330DHCX_Get_Id()
        && sID == 2)     /* MLC subsensor: samplesPerTimestamp must be 1 */
    {
      pSensor->sensorStatus.subSensorStatus[sID].samplesPerTimestamp = 1;
    }
    else
    {
      uint32_t idealSamplesPerTS = (uint32_t)(pSensor->sensorStatus.subSensorStatus[sID].ODR);

      if (idealSamplesPerTS > MAX_SPTS)
      {
        pSensor->sensorStatus.subSensorStatus[sID].samplesPerTimestamp = MAX_SPTS;
      }
      else
      {
        pSensor->sensorStatus.subSensorStatus[sID].samplesPerTimestamp = idealSamplesPerTS;
      }
    }
  }
}

uint8_t update_sensors_config(void)
{
  COM_Sensor_t *pSensor;
  uint8_t ret = 0;

  pSensor = COM_GetSensor(IIS3DWB_Get_Id());
  IIS3DWB_Init_Param.ODR[0] = pSensor->sensorStatus.subSensorStatus[0].ODR;
  IIS3DWB_Init_Param.FS[0] = pSensor->sensorStatus.subSensorStatus[0].FS;
  IIS3DWB_Init_Param.subSensorActive[0] = pSensor->sensorStatus.subSensorStatus[0].isActive;

  ret += HTS221_updateConfig();

  pSensor = COM_GetSensor(IIS2DH_Get_Id());
  IIS2DH_Init_Param.ODR[0] = pSensor->sensorStatus.subSensorStatus[0].ODR;
  IIS2DH_Init_Param.FS[0] = pSensor->sensorStatus.subSensorStatus[0].FS;
  IIS2DH_Init_Param.subSensorActive[0] = pSensor->sensorStatus.subSensorStatus[0].isActive;

  pSensor = COM_GetSensor(IIS2MDC_Get_Id());
  IIS2MDC_Init_Param.ODR[0] = pSensor->sensorStatus.subSensorStatus[0].ODR;
  IIS2MDC_Init_Param.FS[0] = pSensor->sensorStatus.subSensorStatus[0].FS;
  IIS2MDC_Init_Param.subSensorActive[0] = pSensor->sensorStatus.subSensorStatus[0].isActive;

  IMP34DT05_updateConfig();

  ret += ISM330DHCX_updateConfig();

  ret += LPS22HH_updateConfig();

  IMP23ABSU_updateConfig();

  pSensor = COM_GetSensor(STTS751_Get_Id());
  STTS751_Init_Param.ODR[0] = pSensor->sensorStatus.subSensorStatus[0].ODR;
  STTS751_Init_Param.FS[0] = pSensor->sensorStatus.subSensorStatus[0].FS;
  STTS751_Init_Param.subSensorActive[0] = pSensor->sensorStatus.subSensorStatus[0].isActive;

  return ret;
}



