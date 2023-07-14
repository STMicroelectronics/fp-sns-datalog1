/**
  ******************************************************************************
  * @file    stts751_app.c
  * @author  SRA - MCD
  *
  *
  * @brief   This file provides a set of functions to handle stts751 sensor
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
#include "stts751_app.h"
#include "main.h"
#include "cmsis_os.h"
#include "stts751_reg.h"
#include "com_manager.h"
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define WRITE_BUFFER_SIZE_STTS751        (uint32_t)(256)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static int32_t s_nSTTS751_id = 0;

int16_t temperature;
static uint16_t taskDelay = 1000;
static volatile double TimeStamp_stts751;

SM_Init_Param_t STTS751_Init_Param;
SM_Sensor_State_t STTS751_Sensor_State = SM_SENSOR_STATE_INITIALIZING;

/* Semaphore used to wait on component interrupt */
/* Semaphore used to wait on BUS data read complete, managed by lower layer */
osSemaphoreId STTS751_ReadSem_id;
osSemaphoreDef(STTS751_ReadSem);

static sensor_handle_t stts751_hdl_instance = {0, STTS751_0xxxx_ADD_7K5, NULL, 0, &STTS751_ReadSem_id};
static stmdev_ctx_t stts751_ctx_instance = {SM_I2C_Write_Os, SM_I2C_Read_Os, NULL, &stts751_hdl_instance};

/* Private function prototypes -----------------------------------------------*/

osThreadId STTS751_Thread_Id;
static void STTS751_Thread(void const *argument);
static void STTS751_Sensor_Init(void);


/**
  * @brief STTS751 GPIO Initialization Function
  * @param None
  * @retval None
  */
void STTS751_Peripheral_Init(void)
{

}

/**
  * @brief STTS751 Threads Creation
  * @param None
  * @retval None
  */
void STTS751_OS_Init(void)
{
  STTS751_ReadSem_id = osSemaphoreCreate(osSemaphore(STTS751_ReadSem), 1);
  osSemaphoreWait(STTS751_ReadSem_id, osWaitForever);

  /* Thread 1 definition */
  osThreadDef(STTS751_RD_USR_THREAD, STTS751_Thread, HSD_STTS751_THREAD_PRIO, 1, configMINIMAL_STACK_SIZE);
  /* Start thread 1 */
  STTS751_Thread_Id = osThreadCreate(osThread(STTS751_RD_USR_THREAD), NULL);
}


/**
  * @brief  Get data raw from sensors to queue
  * @param  thread not used
  * @retval None
  */
static void STTS751_Thread(void const *argument)
{
  (void) argument;

#if (configUSE_APPLICATION_TASK_TAG == 1 && defined(HSD_TASK_STTS751_DEBUG_PIN))
  vTaskSetApplicationTaskTag(NULL, (TaskHookFunction_t)HSD_TASK_STTS751_DEBUG_PIN);
#endif /* configUSE_APPLICATION_TASK_TAG == 1 && defined(HSD_TASK_STTS751_DEBUG_PIN) */

#if (HSD_USE_DUMMY_DATA == 1)
  static uint16_t dummyDataCounter = 0;
#endif /* HSD_USE_DUMMY_DATA == 1 */

  /* Suspend thread */
  osThreadSuspend(STTS751_Thread_Id);

  for (;;)
  {
    if (STTS751_Sensor_State == SM_SENSOR_STATE_INITIALIZING)
    {
      STTS751_Sensor_Init();
      STTS751_Sensor_State = SM_SENSOR_STATE_RUNNING;
    }
    else if (STTS751_Sensor_State == SM_SENSOR_STATE_RUNNING)
    {
      vTaskDelay(taskDelay);

      if (STTS751_Sensor_State == SM_SENSOR_STATE_RUNNING) /* Change of state can happen while task blocked */
      {
        float temperature_celsius;

        TimeStamp_stts751 = SM_GetTimeStamp();

        stts751_temperature_raw_get(&stts751_ctx_instance, (int16_t *)&temperature);
        temperature_celsius = (float)temperature / 256.0f;

#if (HSD_USE_DUMMY_DATA == 1)
        temperature_celsius = (float)dummyDataCounter++;
#endif /* HSD_USE_DUMMY_DATA == 1 */
        STTS751_Data_Ready(0, (uint8_t *)&temperature_celsius, 4, TimeStamp_stts751);
      }
    }
    else if (STTS751_Sensor_State == SM_SENSOR_STATE_SUSPENDING)
    {
#if (HSD_USE_DUMMY_DATA == 1)
      dummyDataCounter = 0;
#endif /* HSD_USE_DUMMY_DATA == 1 */
      stts751_temp_data_rate_set(&stts751_ctx_instance,  STTS751_TEMP_ODR_OFF);
      STTS751_Sensor_State = SM_SENSOR_STATE_SUSPENDED;
      osThreadSuspend(STTS751_Thread_Id);
    }
  }
}


static void STTS751_Sensor_Init(void)
{
  stts751_id_t STTS751_Id;

  stts751_device_id_get(&stts751_ctx_instance, (stts751_id_t *)&STTS751_Id);
  stts751_temp_data_rate_set(&stts751_ctx_instance,  STTS751_TEMP_ODR_8Hz);
  stts751_resolution_set(&stts751_ctx_instance,  STTS751_12bit);

  if (STTS751_Init_Param.ODR[0] < 2.0f)
  {
    taskDelay = 1000;
  }
  else if (STTS751_Init_Param.ODR[0] < 3.0f)
  {
    taskDelay = 500;
  }
  else if (STTS751_Init_Param.ODR[0] < 5.0f)
  {
    taskDelay = 250;
  }
}

void STTS751_Set_State(SM_Sensor_State_t state)
{
  STTS751_Sensor_State = state;
}

void STTS751_Start(void)
{
  STTS751_Set_State(SM_SENSOR_STATE_INITIALIZING);
  osThreadResume(STTS751_Thread_Id);
}

void STTS751_Stop(void)
{
  STTS751_Set_State(SM_SENSOR_STATE_SUSPENDING);
}

__weak void STTS751_Data_Ready(uint8_t subSensorId, uint8_t *buf, uint16_t size, double timeStamp)
{

}


/**
  * @brief Get Sensor ID
  * @param None
  * @retval Sensor ID
  */
uint8_t STTS751_Get_Id(void)
{
  return s_nSTTS751_id;
}

/**
  * @brief STTS751 Sensor Initialization
  * @param None
  * @retval None
  */
uint8_t STTS751_Create_Sensor(const SM_Init_Param_t *pxParams)
{
  COM_Sensor_t *pSensor;

  s_nSTTS751_id = COM_AddSensor();

  if (s_nSTTS751_id == -1)
  {
    return 1; /* error */
  }

  pSensor = COM_GetSensor(s_nSTTS751_id);

  /* SENSOR DESCRIPTOR */
  strcpy(pSensor->sensorDescriptor.name, "STTS751");
  pSensor->sensorDescriptor.nSubSensors = 1;

  /* SUBSENSOR 0 DESCRIPTOR */
  pSensor->sensorDescriptor.subSensorDescriptor[0].id = 0;
  pSensor->sensorDescriptor.subSensorDescriptor[0].sensorType = COM_TYPE_TEMP;
  pSensor->sensorDescriptor.subSensorDescriptor[0].dimensions = 1;
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].dimensionsLabel[0], "tem");
  pSensor->sensorDescriptor.subSensorDescriptor[0].dataType = DATA_TYPE_FLOAT;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[0] = 1.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[1] = 2.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[2] = 4.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[3] = COM_END_OF_LIST_FLOAT;
  pSensor->sensorDescriptor.subSensorDescriptor[0].samplesPerTimestamp[0] = 0;
  pSensor->sensorDescriptor.subSensorDescriptor[0].samplesPerTimestamp[1] = 1000;
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].unit, "Celsius");
  pSensor->sensorDescriptor.subSensorDescriptor[0].FS[0] = 100.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].FS[1] = COM_END_OF_LIST_FLOAT;

  /* SUBSENSOR 0 STATUS */
  if (pxParams != NULL)
  {
    pSensor->sensorStatus.subSensorStatus[0].isActive = pxParams->subSensorActive[0];
    pSensor->sensorStatus.subSensorStatus[0].FS = pxParams->FS[0];
    pSensor->sensorStatus.subSensorStatus[0].ODR = pxParams->ODR[0];
  }
  else
  {
    pSensor->sensorStatus.subSensorStatus[0].isActive = 0;
    pSensor->sensorStatus.subSensorStatus[0].FS = 100.0f;
    pSensor->sensorStatus.subSensorStatus[0].ODR = 4.0f;
  }
  pSensor->sensorStatus.subSensorStatus[0].sensitivity = 1.0f;
  pSensor->sensorStatus.subSensorStatus[0].measuredODR = 0.0f;
  pSensor->sensorStatus.subSensorStatus[0].initialOffset = 0.0f;
  pSensor->sensorStatus.subSensorStatus[0].samplesPerTimestamp = 4;
  pSensor->sensorStatus.subSensorStatus[0].usbDataPacketSize = 16;
  pSensor->sensorStatus.subSensorStatus[0].sdWriteBufferSize = WRITE_BUFFER_SIZE_STTS751;
  pSensor->sensorStatus.subSensorStatus[0].comChannelNumber = -1;
  pSensor->sensorStatus.subSensorStatus[0].ucfLoaded = 0;

  STTS751_Init_Param.ODR[0] = pSensor->sensorStatus.subSensorStatus[0].ODR;
  STTS751_Init_Param.FS[0] = pSensor->sensorStatus.subSensorStatus[0].FS;
  STTS751_Init_Param.subSensorActive[0] = pSensor->sensorStatus.subSensorStatus[0].isActive;

  return 0;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */


