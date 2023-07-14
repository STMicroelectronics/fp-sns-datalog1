/**
  ******************************************************************************
  * @file    lps22hh_app.c
  * @author  SRA - MCD
  *
  *
  * @brief   This file provides a set of functions to handle lps22hh sensor
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
#include "lps22hh_app.h"
#include "main.h"
#include "com_manager.h"
#include "device_description.h"
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define WRITE_BUFFER_SIZE_LPS22HH_P       (uint32_t)(1024)
#define WRITE_BUFFER_SIZE_LPS22HH_T       (uint32_t)(1024)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static int32_t s_nLPS22HH_id = 0;

static uint8_t lps22hh_mem[256 * 5];
static float lps22hh_mem_temp_f[128 * 2];
static float lps22hh_mem_press_f[128 * 2];

static uint16_t taskDelay = 0;
static volatile double TimeStamp_lps22hh;

SM_Init_Param_t LPS22HH_Init_Param;
SM_Sensor_State_t LPS22HH_Sensor_State = SM_SENSOR_STATE_INITIALIZING;

/* Semaphore used to wait on BUS data read complete, managed by "sensors manager" */
static osSemaphoreId lps22hh_data_read_cmplt_sem_id;
static osSemaphoreDef(lps22hh_data_read_cmplt_sem);

static sensor_handle_t lps22hh_hdl_instance = {LPS22HH_ID, LPS22HH_I2C_ADD_H, NULL, 0, &lps22hh_data_read_cmplt_sem_id};
static stmdev_ctx_t lps22hh_ctx_instance = {SM_I2C_Write_Os, SM_I2C_Read_Os, NULL, &lps22hh_hdl_instance};

/* Private function prototypes -----------------------------------------------*/

osThreadId LPS22HH_Thread_Id;
static void LPS22HH_Thread(void const *argument);
static void LPS22HH_Sensor_Init(void);



/**
  * @brief GPIO Initialization Function, initialize CS and IT pins
  * @param None
  * @retval None
  */
void LPS22HH_Peripheral_Init(void)
{
  /*No additional initialization to be done here */
}

/**
  * @brief LPS22HH Threads Creation
  * @param None
  * @retval None
  */
void LPS22HH_OS_Init(void)
{
  /* Data read complete semaphore initialization */
  lps22hh_data_read_cmplt_sem_id = osSemaphoreCreate(osSemaphore(lps22hh_data_read_cmplt_sem), 1);
  osSemaphoreWait(lps22hh_data_read_cmplt_sem_id, osWaitForever);

  /* Thread definition: read data */
  osThreadDef(LPS22HH_Acquisition_Thread, LPS22HH_Thread, HSD_LPS22HH_THREAD_PRIO, 1, configMINIMAL_STACK_SIZE);
  /* Start thread 1 */
  LPS22HH_Thread_Id = osThreadCreate(osThread(LPS22HH_Acquisition_Thread), NULL);
}


static void LPS22HH_Thread(void const *argument)
{
  (void) argument;

#if (configUSE_APPLICATION_TASK_TAG == 1 && defined(HSD_TASK_LPS22HH_DEBUG_PIN))
  vTaskSetApplicationTaskTag(NULL, (TaskHookFunction_t)HSD_TASK_LPS22HH_DEBUG_PIN);
#endif /* configUSE_APPLICATION_TASK_TAG == 1 && defined(HSD_TASK_LPS22HH_DEBUG_PIN) */

#if (HSD_USE_DUMMY_DATA == 1)
  static uint16_t dummyDataCounter_press = 0;
  static uint16_t dummyDataCounter_temp = 0;
#endif /* HSD_USE_DUMMY_DATA == 1 */

  uint8_t fifo_level_LPS22HH = 0;

  /* Suspend thread */
  osThreadSuspend(LPS22HH_Thread_Id);

  for (;;)
  {
    if (LPS22HH_Sensor_State == SM_SENSOR_STATE_INITIALIZING)
    {
      LPS22HH_Sensor_Init();
      LPS22HH_Sensor_State = SM_SENSOR_STATE_RUNNING;
    }
    else if (LPS22HH_Sensor_State == SM_SENSOR_STATE_RUNNING)
    {
      vTaskDelay(taskDelay);

      if (LPS22HH_Sensor_State == SM_SENSOR_STATE_RUNNING) /* Change of state can happen while task blocked */
      {
        lps22hh_fifo_data_level_get(&lps22hh_ctx_instance, (uint8_t *)&fifo_level_LPS22HH) ;

        TimeStamp_lps22hh = SM_GetTimeStamp();

        lps22hh_read_reg(&lps22hh_ctx_instance, LPS22HH_FIFO_DATA_OUT_PRESS_XL, (uint8_t *) lps22hh_mem,
                         5 * fifo_level_LPS22HH);
        uint16_t i = 0;

        for (i = 0; i < fifo_level_LPS22HH; i++)
        {
          uint32_t press = (((uint32_t)lps22hh_mem[5 * i + 0]))
                           | (((uint32_t)lps22hh_mem[5 * i + 1]) << (8 * 1))
                           | (((uint32_t)lps22hh_mem[5 * i + 2]) << (8 * 2));

          /* convert the 2's complement 24 bit to 2's complement 32 bit */
          if (press & 0x00800000)
          {
            press |= 0xFF000000;
          }

          uint16_t temp = *((uint16_t *)(&lps22hh_mem[5 * i + 3]));

          if (LPS22HH_Init_Param.subSensorActive[0] && !LPS22HH_Init_Param.subSensorActive[1])
          {
            lps22hh_mem_press_f[i] = (float)press / 4096.0f; /* Pressure */
          }
          else if (!LPS22HH_Init_Param.subSensorActive[0] && LPS22HH_Init_Param.subSensorActive[1])
          {
            lps22hh_mem_temp_f[i] = (float)temp / 100.0f; /* Temperature */
          }
          else if (LPS22HH_Init_Param.subSensorActive[0] && LPS22HH_Init_Param.subSensorActive[1])
          {
            lps22hh_mem_press_f[i] = (float)press / 4096.0f; /* Pressure */
            lps22hh_mem_temp_f[i] = (float)temp / 100.0f; /* Temperature */
          }
        }

#if (HSD_USE_DUMMY_DATA == 1)
        for (i = 0; i < fifo_level_LPS22HH ; i++)
        {
          lps22hh_mem_press_f[i]  = (float)dummyDataCounter_press++;
          lps22hh_mem_temp_f[i] = (float)dummyDataCounter_temp++;
        }
#endif /* HSD_USE_DUMMY_DATA == 1 */

        if (LPS22HH_Init_Param.subSensorActive[0]) /* Press Active */
        {
          LPS22HH_Data_Ready(0, (uint8_t *)lps22hh_mem_press_f, 4 * fifo_level_LPS22HH,
                             TimeStamp_lps22hh); /*Todo check dimension / format...*/
        }
        if (LPS22HH_Init_Param.subSensorActive[1]) /* Temperature Active */
        {
          LPS22HH_Data_Ready(1, (uint8_t *)lps22hh_mem_temp_f, 4 * fifo_level_LPS22HH,
                             TimeStamp_lps22hh); /*Todo check dimension / format...*/
        }
      }
    }
    else if (LPS22HH_Sensor_State == SM_SENSOR_STATE_SUSPENDING)
    {
#if (HSD_USE_DUMMY_DATA == 1)
      dummyDataCounter_press = 0;
      dummyDataCounter_temp = 0;
#endif /* HSD_USE_DUMMY_DATA == 1 */
      lps22hh_data_rate_set(&lps22hh_ctx_instance, (lps22hh_odr_t)(LPS22HH_POWER_DOWN | 0x10));
      LPS22HH_Sensor_State = SM_SENSOR_STATE_SUSPENDED;
      osThreadSuspend(LPS22HH_Thread_Id);
    }
  }
}


uint8_t LPS22HH_updateConfig(void)
{
  uint8_t ret = 0;
  COM_Sensor_t *pSensor = COM_GetSensor(LPS22HH_Get_Id());

  if ((LPS22HH_Init_Param.ODR[0] != pSensor->sensorStatus.subSensorStatus[0].ODR)
      || (LPS22HH_Init_Param.ODR[1] != pSensor->sensorStatus.subSensorStatus[1].ODR))
  {
    ret = 1;
  }

  LPS22HH_Init_Param.ODR[0] = pSensor->sensorStatus.subSensorStatus[0].ODR;
  LPS22HH_Init_Param.FS[0] = pSensor->sensorStatus.subSensorStatus[0].FS;
  LPS22HH_Init_Param.subSensorActive[0] = pSensor->sensorStatus.subSensorStatus[0].isActive;
  LPS22HH_Init_Param.ODR[1] = pSensor->sensorStatus.subSensorStatus[1].ODR;
  LPS22HH_Init_Param.FS[1] = pSensor->sensorStatus.subSensorStatus[1].FS;
  LPS22HH_Init_Param.subSensorActive[1] = pSensor->sensorStatus.subSensorStatus[1].isActive;

  update_samplesPerTimestamp(pSensor);

  return ret;
}


static void LPS22HH_Sensor_Init(void)
{
  uint8_t reg0;
  float lps22hh_odr = 0.0f;

  /* Disable MIPI I3C(SM) interface */
  lps22hh_i3c_interface_set(&lps22hh_ctx_instance, LPS22HH_I3C_DISABLE);

  /* Power Down */
  lps22hh_device_id_get(&lps22hh_ctx_instance, (uint8_t *)&reg0);

  /* Power down the device, set Low Noise Enable (bit 5), clear One Shot (bit 4) */
  lps22hh_data_rate_set(&lps22hh_ctx_instance, (lps22hh_odr_t)(LPS22HH_POWER_DOWN | 0x10));

  /* Disable low-pass filter on LPS22HH pressure data */
  lps22hh_lp_bandwidth_set(&lps22hh_ctx_instance, LPS22HH_LPF_ODR_DIV_2);

  /* Set block data update mode */
  lps22hh_block_data_update_set(&lps22hh_ctx_instance, PROPERTY_ENABLE);

  /* Set autoincrement for multi-byte read/write */
  lps22hh_auto_increment_set(&lps22hh_ctx_instance, PROPERTY_ENABLE);

  lps22hh_reset_set(&lps22hh_ctx_instance, 1);

  /* Set fifo mode */
  lps22hh_fifo_mode_set(&lps22hh_ctx_instance, LPS22HH_STREAM_MODE);

  if (LPS22HH_Init_Param.subSensorActive[0] == 1)
  {
    lps22hh_odr = LPS22HH_Init_Param.ODR[0];
    LPS22HH_Init_Param.ODR[1] = LPS22HH_Init_Param.ODR[0];
  }
  else
  {
    lps22hh_odr = LPS22HH_Init_Param.ODR[1];
    LPS22HH_Init_Param.ODR[0] = LPS22HH_Init_Param.ODR[1];
  }

  if (lps22hh_odr < 2.0f)
  {
    lps22hh_data_rate_set(&lps22hh_ctx_instance, LPS22HH_1_Hz);
    taskDelay = 1000;
  }
  else if (lps22hh_odr < 11.0f)
  {
    lps22hh_data_rate_set(&lps22hh_ctx_instance, LPS22HH_10_Hz);
    taskDelay = 1000;
  }
  else if (lps22hh_odr < 26.0f)
  {
    lps22hh_data_rate_set(&lps22hh_ctx_instance, LPS22HH_25_Hz);
    taskDelay = 1000;
  }
  else if (lps22hh_odr < 51.0f)
  {
    lps22hh_data_rate_set(&lps22hh_ctx_instance, LPS22HH_50_Hz);
    taskDelay = 1000;
  }
  else if (lps22hh_odr < 76.0f)
  {
    lps22hh_data_rate_set(&lps22hh_ctx_instance, LPS22HH_75_Hz);
    taskDelay = 1000;
  }
  else if (lps22hh_odr < 101.0f)
  {
    lps22hh_data_rate_set(&lps22hh_ctx_instance, LPS22HH_100_Hz);
    taskDelay = 1000;
  }
  else if (lps22hh_odr < 201.0f)
  {
    lps22hh_data_rate_set(&lps22hh_ctx_instance, LPS22HH_200_Hz);
    taskDelay = 500;
  }
}

void LPS22HH_Set_State(SM_Sensor_State_t state)
{
  LPS22HH_Sensor_State = state;
}

void LPS22HH_Start(void)
{
  LPS22HH_Set_State(SM_SENSOR_STATE_INITIALIZING);
  osThreadResume(LPS22HH_Thread_Id);
}

void LPS22HH_Stop(void)
{
  LPS22HH_Set_State(SM_SENSOR_STATE_SUSPENDING);
}

__weak void LPS22HH_Data_Ready(uint8_t subSensorId, uint8_t *buf, uint16_t size, double timeStamp)
{

}

/**
  * @brief Get Sensor ID
  * @param None
  * @retval Sensor ID
  */
uint8_t LPS22HH_Get_Id(void)
{
  return s_nLPS22HH_id;
}

/**
  * @brief LPS22HH Sensor Initialization
  * @param None
  * @retval None
  */
uint8_t LPS22HH_Create_Sensor(const SM_Init_Param_t *pxParams)
{
  COM_Sensor_t *pSensor;

  s_nLPS22HH_id = COM_AddSensor();

  if (s_nLPS22HH_id == -1)
  {
    return 1; /* error */
  }

  pSensor = COM_GetSensor(s_nLPS22HH_id);

  /* SENSOR DESCRIPTOR */
  strcpy(pSensor->sensorDescriptor.name, "LPS22HH");
  pSensor->sensorDescriptor.nSubSensors = 2;

  /* SUBSENSOR 0 DESCRIPTOR */
  pSensor->sensorDescriptor.subSensorDescriptor[0].id = 0;
  pSensor->sensorDescriptor.subSensorDescriptor[0].sensorType = COM_TYPE_PRESS;
  pSensor->sensorDescriptor.subSensorDescriptor[0].dimensions = 1;
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].dimensionsLabel[0], "prs");
  pSensor->sensorDescriptor.subSensorDescriptor[0].dataType = DATA_TYPE_FLOAT;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[0] = 1.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[1] = 10.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[2] = 25.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[3] = 50.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[4] = 75.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[5] = 100.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[6] = 200.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[7] = COM_END_OF_LIST_FLOAT;
  pSensor->sensorDescriptor.subSensorDescriptor[0].samplesPerTimestamp[0] = 0;
  pSensor->sensorDescriptor.subSensorDescriptor[0].samplesPerTimestamp[1] = 1000;
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].unit, "hPa");
  pSensor->sensorDescriptor.subSensorDescriptor[0].FS[0] = 1260.0f;
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
    pSensor->sensorStatus.subSensorStatus[0].FS = 1260.0f;
    pSensor->sensorStatus.subSensorStatus[0].ODR = 200.0f;
  }
  pSensor->sensorStatus.subSensorStatus[0].sensitivity = 1.0f;
  pSensor->sensorStatus.subSensorStatus[0].measuredODR = 0.0f;
  pSensor->sensorStatus.subSensorStatus[0].initialOffset = 0.0f;
  pSensor->sensorStatus.subSensorStatus[0].samplesPerTimestamp = 200;
  pSensor->sensorStatus.subSensorStatus[0].usbDataPacketSize = 1600;
  pSensor->sensorStatus.subSensorStatus[0].sdWriteBufferSize = WRITE_BUFFER_SIZE_LPS22HH_P;
  pSensor->sensorStatus.subSensorStatus[0].comChannelNumber = -1;
  pSensor->sensorStatus.subSensorStatus[0].ucfLoaded = 0;

  /* SUBSENSOR 1 DESCRIPTOR */
  pSensor->sensorDescriptor.subSensorDescriptor[1].id = 1;
  pSensor->sensorDescriptor.subSensorDescriptor[1].sensorType = COM_TYPE_TEMP;
  pSensor->sensorDescriptor.subSensorDescriptor[1].dimensions = 1;
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[1].dimensionsLabel[0], "tem");
  pSensor->sensorDescriptor.subSensorDescriptor[1].dataType = DATA_TYPE_FLOAT;
  pSensor->sensorDescriptor.subSensorDescriptor[1].ODR[0] = 1.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].ODR[1] = 10.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].ODR[2] = 25.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].ODR[3] = 50.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].ODR[4] = 75.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].ODR[5] = 100.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].ODR[6] = 200.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].ODR[7] = COM_END_OF_LIST_FLOAT;
  pSensor->sensorDescriptor.subSensorDescriptor[1].samplesPerTimestamp[0] = 0;
  pSensor->sensorDescriptor.subSensorDescriptor[1].samplesPerTimestamp[1] = 1000;
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[1].unit, "Celsius");
  pSensor->sensorDescriptor.subSensorDescriptor[1].FS[0] = 85.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].FS[1] = COM_END_OF_LIST_FLOAT;

  /* SUBSENSOR 1 STATUS */
  if (pxParams != NULL)
  {
    pSensor->sensorStatus.subSensorStatus[1].isActive = pxParams->subSensorActive[1];
    pSensor->sensorStatus.subSensorStatus[1].FS = pxParams->FS[1];
    pSensor->sensorStatus.subSensorStatus[1].ODR = pSensor->sensorStatus.subSensorStatus[0].ODR;
  }
  else
  {
    pSensor->sensorStatus.subSensorStatus[1].isActive = 0;
    pSensor->sensorStatus.subSensorStatus[1].FS = 85.0f;
    pSensor->sensorStatus.subSensorStatus[1].ODR = pSensor->sensorStatus.subSensorStatus[0].ODR;
  }
  pSensor->sensorStatus.subSensorStatus[1].sensitivity = 1.0f;
  pSensor->sensorStatus.subSensorStatus[1].measuredODR = 0.0f;
  pSensor->sensorStatus.subSensorStatus[1].initialOffset = 0.0f;
  pSensor->sensorStatus.subSensorStatus[1].samplesPerTimestamp = 200;
  pSensor->sensorStatus.subSensorStatus[1].usbDataPacketSize = 1600;
  pSensor->sensorStatus.subSensorStatus[1].sdWriteBufferSize = WRITE_BUFFER_SIZE_LPS22HH_T;
  pSensor->sensorStatus.subSensorStatus[1].comChannelNumber = -1;
  pSensor->sensorStatus.subSensorStatus[1].ucfLoaded = 0;

  LPS22HH_Init_Param.ODR[0] = pSensor->sensorStatus.subSensorStatus[0].ODR;
  LPS22HH_Init_Param.ODR[1] = pSensor->sensorStatus.subSensorStatus[1].ODR;
  LPS22HH_Init_Param.FS[0] = pSensor->sensorStatus.subSensorStatus[0].FS;
  LPS22HH_Init_Param.FS[1] = pSensor->sensorStatus.subSensorStatus[1].FS;
  LPS22HH_Init_Param.subSensorActive[0] = pSensor->sensorStatus.subSensorStatus[0].isActive;
  LPS22HH_Init_Param.subSensorActive[1] = pSensor->sensorStatus.subSensorStatus[1].isActive;

  /**********/
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


