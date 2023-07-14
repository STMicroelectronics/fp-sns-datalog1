/**
  ******************************************************************************
  * @file    hts221_app.c
  * @author  SRA - MCD
  *
  *
  * @brief   This file provides a set of functions to handle hts221 sensor
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
#include "hts221_app.h"
#include "main.h"
#include "com_manager.h"
#include "device_description.h"

/* Private includes ----------------------------------------------------------*/
#define WRITE_BUFFER_SIZE_HTS221_H       (uint32_t)(256)
#define WRITE_BUFFER_SIZE_HTS221_T       (uint32_t)(256)

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
float x0_t = 0;
float y0_t = 0;
float x1_t = 0;
float y1_t = 0;
float x0_h = 0;
float y0_h = 0;
float x1_h = 0;
float y1_h = 0;

static volatile double TimeStamp_hts221;

SM_Init_Param_t HTS221_Init_Param;
SM_Sensor_State_t HTS221_Sensor_State = SM_SENSOR_STATE_INITIALIZING;

/* Semaphore used to wait on component interrupt */
static osSemaphoreId hts221_data_ready_sem_id;
static osSemaphoreDef(hts221_data_ready_sem);

/* Semaphore used to wait on BUS data read complete, managed by "sensors manager" */
static osSemaphoreId hts221_data_read_cmplt_sem_id;
static osSemaphoreDef(hts221_data_read_cmplt_sem);

static sensor_handle_t hts221_hdl_instance = {HTS221_ID, HTS221_I2C_ADDRESS, NULL, 0, &hts221_data_read_cmplt_sem_id};
static stmdev_ctx_t hts221_ctx_instance = {SM_I2C_Write_Os, SM_I2C_Read_Os, NULL, &hts221_hdl_instance};

/* Private function prototypes -----------------------------------------------*/

osThreadId HTS221_Thread_Id;
static void HTS221_Thread(void const *argument);

EXTI_HandleTypeDef hts221_exti;
static void HTS221_Int_Callback(void);
static void HTS221_Sensor_Init(void);



/**
  * @brief GPIO Initialization Function, initialize CS and IT pins
  * @param None
  * @retval None
  */
void HTS221_Peripheral_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  HTS221_INT_GPIO_ADDITIONAL();
  HTS221_INT_GPIO_CLK_ENABLE();
  /*Configure GPIO pins : STTS751_INT_Pin hts221_INT1_Pin */
  GPIO_InitStruct.Pin =  HTS221_INT_Pin ;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(HTS221_INT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(HTS221_INT_EXTI_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(HTS221_INT_EXTI_IRQn);
  HAL_EXTI_GetHandle(&hts221_exti, HTS221_INT_EXTI_LINE);
  HAL_EXTI_RegisterCallback(&hts221_exti,  HAL_EXTI_COMMON_CB_ID, HTS221_Int_Callback);
}

/**
  * @brief HTS221 Threads Creation
  * @param None
  * @retval None
  */
void HTS221_OS_Init(void)
{
  /* Data read complete semaphore initialization */
  hts221_data_read_cmplt_sem_id = osSemaphoreCreate(osSemaphore(hts221_data_read_cmplt_sem), 1);
  osSemaphoreWait(hts221_data_read_cmplt_sem_id, osWaitForever);

  /* Data ready interrupt semaphore initialization */
  hts221_data_ready_sem_id = osSemaphoreCreate(osSemaphore(hts221_data_ready_sem), 1);
  osSemaphoreWait(hts221_data_ready_sem_id,  osWaitForever);

  /* Thread definition: read data */
  osThreadDef(HTS221_Acquisition_Thread, HTS221_Thread, HSD_HTS221_THREAD_PRIO, 1, configMINIMAL_STACK_SIZE);
  /* Start thread */
  HTS221_Thread_Id = osThreadCreate(osThread(HTS221_Acquisition_Thread), NULL);
}



static void HTS221_Thread(void const *argument)
{
  (void) argument;

#if (configUSE_APPLICATION_TASK_TAG == 1 && defined(HSD_TASK_HTS221_DEBUG_PIN))
  vTaskSetApplicationTaskTag(NULL, (TaskHookFunction_t)HSD_TASK_HTS221_DEBUG_PIN);
#endif /* configUSE_APPLICATION_TASK_TAG == 1 && defined(HSD_TASK_HTS221_DEBUG_PIN) */

#if (HSD_USE_DUMMY_DATA == 1)
  static uint16_t dummyDataCounter_temp = 0;
  static uint16_t dummyDataCounter_hum = 0;
#endif /* HSD_USE_DUMMY_DATA == 1 */

  hts221_axis1bit16_t data_raw_humidity;
  hts221_axis1bit16_t data_raw_temperature;

  /* Suspend thread */
  osThreadSuspend(HTS221_Thread_Id);

  for (;;)
  {
    /* Read raw data (roughly) each 1 s */
    if (HTS221_Sensor_State == SM_SENSOR_STATE_INITIALIZING)
    {
      HTS221_Sensor_Init();
      HTS221_Sensor_State = SM_SENSOR_STATE_RUNNING;
      hts221_temperature_raw_get(&hts221_ctx_instance, &data_raw_temperature.i16bit);
      hts221_humidity_raw_get(&hts221_ctx_instance, &data_raw_humidity.i16bit);
    }
    else if (HTS221_Sensor_State == SM_SENSOR_STATE_RUNNING)
    {
      float dataOut[2];
      osSemaphoreWait(hts221_data_ready_sem_id,  osWaitForever);

      if (HTS221_Sensor_State == SM_SENSOR_STATE_RUNNING) /* Change of state can happen while task blocked */
      {
        hts221_temperature_raw_get(&hts221_ctx_instance, &data_raw_temperature.i16bit);
        /* Apply calibration */ /* To be optimized eventually */
        dataOut[0] = (((y1_t - y0_t) * (float)(data_raw_temperature.i16bit)) + ((x1_t * y0_t) - (x0_t * y1_t)))
                     / (x1_t - x0_t);

        hts221_humidity_raw_get(&hts221_ctx_instance, &data_raw_humidity.i16bit);
        /* Apply calibration */ /* To be optimized eventually */
        dataOut[1] = (((y1_h - y0_h) * (float)(data_raw_humidity.i16bit)) + ((x1_h * y0_h) - (x0_h * y1_h)))
                     / (x1_h - x0_h);

#if (HSD_USE_DUMMY_DATA == 1)
        dataOut[0] = (float)dummyDataCounter_temp++;
        dataOut[1] = (float)dummyDataCounter_hum++;
#endif /* HSD_USE_DUMMY_DATA == 1 */
        if (HTS221_Init_Param.subSensorActive[0]) /* Temperature */
        {
          HTS221_Data_Ready(0, (uint8_t *)dataOut, 4, TimeStamp_hts221);
        }
        if (HTS221_Init_Param.subSensorActive[1]) /* Humidity */
        {
          HTS221_Data_Ready(1, (uint8_t *)&dataOut[1], 4, TimeStamp_hts221);
        }
      }
    }
    else if (HTS221_Sensor_State == SM_SENSOR_STATE_SUSPENDING)
    {
#if (HSD_USE_DUMMY_DATA == 1)
      dummyDataCounter_temp = 0;
      dummyDataCounter_hum = 0;
#endif /* HSD_USE_DUMMY_DATA == 1 */
      hts221_power_on_set(&hts221_ctx_instance, PROPERTY_DISABLE);
      HTS221_Sensor_State = SM_SENSOR_STATE_SUSPENDED;
      osThreadSuspend(HTS221_Thread_Id);
    }
  }
}


uint8_t HTS221_updateConfig(void)
{
  uint8_t ret = 0;
  COM_Sensor_t *pSensor = COM_GetSensor(HTS221_Get_Id());

  if ((HTS221_Init_Param.ODR[0] != pSensor->sensorStatus.subSensorStatus[0].ODR)
      || (HTS221_Init_Param.ODR[1] != pSensor->sensorStatus.subSensorStatus[1].ODR))
  {
    ret = 1;
  }

  HTS221_Init_Param.ODR[0] = pSensor->sensorStatus.subSensorStatus[0].ODR;
  HTS221_Init_Param.FS[0] = pSensor->sensorStatus.subSensorStatus[0].FS;
  HTS221_Init_Param.subSensorActive[0] = pSensor->sensorStatus.subSensorStatus[0].isActive;
  HTS221_Init_Param.ODR[1] = pSensor->sensorStatus.subSensorStatus[1].ODR;
  HTS221_Init_Param.FS[1] = pSensor->sensorStatus.subSensorStatus[1].FS;
  HTS221_Init_Param.subSensorActive[1] = pSensor->sensorStatus.subSensorStatus[1].isActive;

  update_samplesPerTimestamp(pSensor);

  return ret;
}


static void HTS221_Sensor_Init(void)
{
  uint8_t reg0;
  float hts221_odr = 0.0f;
  lin_t lin_temp;
  lin_t lin_hum;

  hts221_device_id_get(&hts221_ctx_instance, (uint8_t *)&reg0);
  hts221_power_on_set(&hts221_ctx_instance, PROPERTY_DISABLE);
  hts221_boot_set(&hts221_ctx_instance, PROPERTY_ENABLE);

  /* Set BDU*/
  hts221_block_data_update_set(&hts221_ctx_instance, PROPERTY_ENABLE);
  /* Enable Interrupt */
  hts221_drdy_on_int_set(&hts221_ctx_instance, PROPERTY_ENABLE);

  /* Set Data Rate */
  if (HTS221_Init_Param.subSensorActive[1] == 1)
  {
    hts221_odr = HTS221_Init_Param.ODR[1];
    HTS221_Init_Param.ODR[0] = HTS221_Init_Param.ODR[1];
  }
  else
  {
    hts221_odr = HTS221_Init_Param.ODR[0];
    HTS221_Init_Param.ODR[1] = HTS221_Init_Param.ODR[0];
  }

  if (hts221_odr < 2.0f)
  {
    hts221_data_rate_set(&hts221_ctx_instance, HTS221_ODR_1Hz);
  }
  else if (hts221_odr < 8.0f)
  {
    hts221_data_rate_set(&hts221_ctx_instance, HTS221_ODR_7Hz);
  }
  else if (hts221_odr < 13.0f)
  {
    hts221_data_rate_set(&hts221_ctx_instance, HTS221_ODR_12Hz5);
  }

  /* Get calibration values (only first time) */
  hts221_temp_adc_point_0_get(&hts221_ctx_instance, &lin_temp.x0);
  x0_t = lin_temp.x0;

  hts221_temp_deg_point_0_get(&hts221_ctx_instance, &lin_temp.y0);
  y0_t = lin_temp.y0;

  hts221_temp_adc_point_1_get(&hts221_ctx_instance, &lin_temp.x1);
  x1_t = lin_temp.x1;

  hts221_temp_deg_point_1_get(&hts221_ctx_instance, &lin_temp.y1);
  y1_t = lin_temp.y1;

  hts221_hum_adc_point_0_get(&hts221_ctx_instance, &lin_hum.x0);
  x0_h = lin_hum.x0;

  hts221_hum_rh_point_0_get(&hts221_ctx_instance, &lin_hum.y0);
  y0_h = lin_hum.y0;

  hts221_hum_adc_point_1_get(&hts221_ctx_instance, &lin_hum.x1);
  x1_h = lin_hum.x1;

  hts221_hum_rh_point_1_get(&hts221_ctx_instance, &lin_hum.y1);
  y1_h = lin_hum.y1;

  /* Power Up */
  hts221_power_on_set(&hts221_ctx_instance, PROPERTY_ENABLE);
}

static void HTS221_Int_Callback(void)
{
  TimeStamp_hts221 = SM_GetTimeStamp_fromISR();
  osSemaphoreRelease(hts221_data_ready_sem_id);
}

void HTS221_Set_State(SM_Sensor_State_t state)
{
  HTS221_Sensor_State = state;
}

void HTS221_Start(void)
{
  HTS221_Set_State(SM_SENSOR_STATE_INITIALIZING);
  osThreadResume(HTS221_Thread_Id);
}

void HTS221_Stop(void)
{
  HTS221_Set_State(SM_SENSOR_STATE_SUSPENDING);
}

__weak void HTS221_Data_Ready(uint8_t subSensorId, uint8_t *buf, uint16_t size, double timeStamp)
{

}

static int32_t s_nHTS221_id = 0;

/**
  * @brief Get Sensor ID
  * @param None
  * @retval Sensor ID
  */
uint8_t HTS221_Get_Id(void)
{
  return s_nHTS221_id;
}

/**
  * @brief HTS221 Sensor Initialization
  * @param None
  * @retval None
  */
uint8_t HTS221_Create_Sensor(const SM_Init_Param_t *pxParams)
{
  COM_Sensor_t *pSensor;

  s_nHTS221_id = COM_AddSensor();

  if (s_nHTS221_id == -1)
  {
    return 1; /* error */
  }

  pSensor = COM_GetSensor(s_nHTS221_id);

  /* SENSOR DESCRIPTOR */
  strcpy(pSensor->sensorDescriptor.name, "HTS221");
  pSensor->sensorDescriptor.nSubSensors = 2;

  /* SUBSENSOR 0 DESCRIPTOR */
  pSensor->sensorDescriptor.subSensorDescriptor[0].id = 0;
  pSensor->sensorDescriptor.subSensorDescriptor[0].sensorType = COM_TYPE_TEMP;
  pSensor->sensorDescriptor.subSensorDescriptor[0].dimensions = 1;
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].dimensionsLabel[0], "tem");
  pSensor->sensorDescriptor.subSensorDescriptor[0].dataType = DATA_TYPE_FLOAT;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[0] = 1.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[1] = 7.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[2] = 12.5f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[3] = COM_END_OF_LIST_FLOAT;
  pSensor->sensorDescriptor.subSensorDescriptor[0].samplesPerTimestamp[0] = 0;
  pSensor->sensorDescriptor.subSensorDescriptor[0].samplesPerTimestamp[1] = 1000;
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].unit, "Celsius");
  pSensor->sensorDescriptor.subSensorDescriptor[0].FS[0] = 120.0f;
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
    pSensor->sensorStatus.subSensorStatus[0].FS = 120.0f;
    pSensor->sensorStatus.subSensorStatus[0].ODR = 12.5f;
  }
  pSensor->sensorStatus.subSensorStatus[0].sensitivity = 1.0f;
  pSensor->sensorStatus.subSensorStatus[0].measuredODR = 0.0f;
  pSensor->sensorStatus.subSensorStatus[0].initialOffset = 0.0f;
  pSensor->sensorStatus.subSensorStatus[0].samplesPerTimestamp = 12;
  pSensor->sensorStatus.subSensorStatus[0].usbDataPacketSize = 16;
  pSensor->sensorStatus.subSensorStatus[0].sdWriteBufferSize = WRITE_BUFFER_SIZE_HTS221_T;
  pSensor->sensorStatus.subSensorStatus[0].comChannelNumber = -1;
  pSensor->sensorStatus.subSensorStatus[0].ucfLoaded = 0;

  /* SUBSENSOR 1 DESCRIPTOR */
  pSensor->sensorDescriptor.subSensorDescriptor[1].id = 1;
  pSensor->sensorDescriptor.subSensorDescriptor[1].sensorType = COM_TYPE_HUM;
  pSensor->sensorDescriptor.subSensorDescriptor[1].dimensions = 1;
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[1].dimensionsLabel[0], "hum");
  pSensor->sensorDescriptor.subSensorDescriptor[1].dataType = DATA_TYPE_FLOAT;
  pSensor->sensorDescriptor.subSensorDescriptor[1].ODR[0] = 1.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].ODR[1] = 7.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].ODR[2] = 12.5f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].ODR[3] = COM_END_OF_LIST_FLOAT;
  pSensor->sensorDescriptor.subSensorDescriptor[1].samplesPerTimestamp[0] = 0;
  pSensor->sensorDescriptor.subSensorDescriptor[1].samplesPerTimestamp[1] = 1000;
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[1].unit, "%");
  pSensor->sensorDescriptor.subSensorDescriptor[1].FS[0] = 100.0f;
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
    pSensor->sensorStatus.subSensorStatus[1].FS = 100.0f;
    pSensor->sensorStatus.subSensorStatus[1].ODR = pSensor->sensorStatus.subSensorStatus[0].ODR;
  }
  pSensor->sensorStatus.subSensorStatus[1].sensitivity = 1.0f;
  pSensor->sensorStatus.subSensorStatus[1].measuredODR = 0.0f;
  pSensor->sensorStatus.subSensorStatus[1].initialOffset = 0.0f;
  pSensor->sensorStatus.subSensorStatus[1].samplesPerTimestamp = 12;
  pSensor->sensorStatus.subSensorStatus[1].usbDataPacketSize = 16;
  pSensor->sensorStatus.subSensorStatus[1].sdWriteBufferSize = WRITE_BUFFER_SIZE_HTS221_H;
  pSensor->sensorStatus.subSensorStatus[1].comChannelNumber = -1;
  pSensor->sensorStatus.subSensorStatus[1].ucfLoaded = 0;

  HTS221_Init_Param.ODR[0] = pSensor->sensorStatus.subSensorStatus[0].ODR;
  HTS221_Init_Param.ODR[1] = pSensor->sensorStatus.subSensorStatus[1].ODR;
  HTS221_Init_Param.FS[0] = pSensor->sensorStatus.subSensorStatus[0].FS;
  HTS221_Init_Param.FS[1] = pSensor->sensorStatus.subSensorStatus[1].FS;
  HTS221_Init_Param.subSensorActive[0] = pSensor->sensorStatus.subSensorStatus[0].isActive;
  HTS221_Init_Param.subSensorActive[1] = pSensor->sensorStatus.subSensorStatus[1].isActive;

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


