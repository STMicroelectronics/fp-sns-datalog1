/**
  ******************************************************************************
  * @file    iis2mdc_app.c
  * @author  SRA - MCD
  *
  *
  * @brief   This file provides a set of functions to handle iis3dwb sensor
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
#include "iis2mdc_app.h"
#include "main.h"
#include "com_manager.h"
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define WRITE_BUFFER_SIZE_IIS2MDC        (uint32_t)(512)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static int32_t s_nIIS2MDC_id = 0;

static uint8_t iis2mdc_mem[6];
static volatile double TimeStamp_iis2mdc;

SM_Init_Param_t IIS2MDC_Init_Param;
SM_Sensor_State_t IIS2MDC_Sensor_State = SM_SENSOR_STATE_INITIALIZING;

/* Semaphore used to wait on component interrupt */
static osSemaphoreId iis2mdc_data_ready_sem_id;
static osSemaphoreDef(iis2mdc_data_ready_sem);

/* Semaphore used to wait on BUS data read complete, managed by lower layer */
static osSemaphoreId iis2mdc_data_read_cmplt_sem_id;
static osSemaphoreDef(iis2mdc_data_read_cmplt_sem);

static sensor_handle_t iis2mdc_hdl_instance = {0, IIS2MDC_I2C_ADD, NULL, 0, &iis2mdc_data_read_cmplt_sem_id};
static stmdev_ctx_t iis2mdc_ctx_instance = {SM_I2C_Write_Os, SM_I2C_Read_Os, NULL, &iis2mdc_hdl_instance};

/* Private function prototypes -----------------------------------------------*/

osThreadId IIS2MDC_Thread_Id;
static void IIS2MDC_Thread(void const *argument);

EXTI_HandleTypeDef iis2mdc_exti;
static void IIS2MDC_Int_Callback(void);
static void IIS2MDC_Sensor_Init(void);



/**
  * @brief IIS3DWB GPIO Initialization Function
  * @param None
  * @retval None
  */
void IIS2MDC_Peripheral_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pins : STTS751_INT_Pin IIS2MDC_INT1_Pin */
  GPIO_InitStruct.Pin =  IIS2MDC_INT1_Pin ;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IIS2MDC_INT1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  HAL_EXTI_GetHandle(&iis2mdc_exti, EXTI_LINE_9);
  HAL_EXTI_RegisterCallback(&iis2mdc_exti,  HAL_EXTI_COMMON_CB_ID, IIS2MDC_Int_Callback);
}

/**
  * @brief IIS3DWB Threads Creation
  * @param None
  * @retval None
  */
void IIS2MDC_OS_Init(void)
{
  /* Data read complete semaphore initialization */
  iis2mdc_data_read_cmplt_sem_id = osSemaphoreCreate(osSemaphore(iis2mdc_data_read_cmplt_sem), 1);
  osSemaphoreWait(iis2mdc_data_read_cmplt_sem_id, osWaitForever);

  /* Data ready interrupt semaphore initialization */
  iis2mdc_data_ready_sem_id = osSemaphoreCreate(osSemaphore(iis2mdc_data_ready_sem), 1);
  osSemaphoreWait(iis2mdc_data_ready_sem_id,  osWaitForever);

  /* Thread definition: read data */
  osThreadDef(IIS2MDC_Acquisition_Thread, IIS2MDC_Thread, HSD_IIS2MDC_THREAD_PRIO, 1, configMINIMAL_STACK_SIZE);
  /* Start thread 1 */
  IIS2MDC_Thread_Id = osThreadCreate(osThread(IIS2MDC_Acquisition_Thread), NULL);
}


static void IIS2MDC_Thread(void const *argument)
{
  (void) argument;

#if (configUSE_APPLICATION_TASK_TAG == 1 && defined(HSD_TASK_IIS2MDC_DEBUG_PIN))
  vTaskSetApplicationTaskTag(NULL, (TaskHookFunction_t)HSD_TASK_IIS2MDC_DEBUG_PIN);
#endif /* configUSE_APPLICATION_TASK_TAG == 1 && defined(HSD_TASK_IIS2MDC_DEBUG_PIN) */

#if (HSD_USE_DUMMY_DATA == 1)
  static uint16_t dummyDataCounter = 0;
#endif /* HSD_USE_DUMMY_DATA == 1 */

  /* Suspend thread */
  osThreadSuspend(IIS2MDC_Thread_Id);

  for (;;)
  {
    if (IIS2MDC_Sensor_State == SM_SENSOR_STATE_INITIALIZING)
    {
      IIS2MDC_Sensor_Init();
      IIS2MDC_Sensor_State = SM_SENSOR_STATE_RUNNING;
    }
    else if (IIS2MDC_Sensor_State == SM_SENSOR_STATE_RUNNING)
    {
      osSemaphoreWait(iis2mdc_data_ready_sem_id,  osWaitForever);

      if (IIS2MDC_Sensor_State == SM_SENSOR_STATE_RUNNING) /* Change of state can happen while task blocked */
      {
        iis2mdc_magnetic_raw_get(&iis2mdc_ctx_instance, (int16_t *)iis2mdc_mem);

#if (HSD_USE_DUMMY_DATA == 1)
        int16_t *p16 = (int16_t *)iis2mdc_mem;

        *p16++ = dummyDataCounter++;
        *p16++ = dummyDataCounter++;
        *p16++ = dummyDataCounter++;
#endif /* HSD_USE_DUMMY_DATA == 1 */
        IIS2MDC_Data_Ready(0, (uint8_t *)iis2mdc_mem, 6, TimeStamp_iis2mdc);
      }
    }
    else if (IIS2MDC_Sensor_State == SM_SENSOR_STATE_SUSPENDING)
    {
#if (HSD_USE_DUMMY_DATA == 1)
      dummyDataCounter = 0;
#endif /* HSD_USE_DUMMY_DATA == 1 */
      iis2mdc_operating_mode_set(&iis2mdc_ctx_instance, IIS2MDC_POWER_DOWN);
      IIS2MDC_Sensor_State = SM_SENSOR_STATE_SUSPENDED;
      osThreadSuspend(IIS2MDC_Thread_Id);
    }
  }
}


static void IIS2MDC_Sensor_Init(void)
{
  uint8_t reg0 = 0;

  iis2mdc_device_id_get(&iis2mdc_ctx_instance, (uint8_t *)&reg0);
  iis2mdc_reset_set(&iis2mdc_ctx_instance, 1);
  iis2mdc_block_data_update_set(&iis2mdc_ctx_instance, 1);
  iis2mdc_self_test_set(&iis2mdc_ctx_instance, 0);
  iis2mdc_drdy_on_pin_set(&iis2mdc_ctx_instance, 1);

  /*Read first data to start it generation */
  int16_t buff[] = {0, 0, 0};
  iis2mdc_mag_user_offset_set(&iis2mdc_ctx_instance, buff);
  iis2mdc_operating_mode_set(&iis2mdc_ctx_instance, IIS2MDC_CONTINUOUS_MODE);

  if (IIS2MDC_Init_Param.ODR[0] < 11.0f)
  {
    iis2mdc_data_rate_set(&iis2mdc_ctx_instance, IIS2MDC_ODR_10Hz);
  }
  else if (IIS2MDC_Init_Param.ODR[0] < 21.0f)
  {
    iis2mdc_data_rate_set(&iis2mdc_ctx_instance, IIS2MDC_ODR_20Hz);
  }
  else if (IIS2MDC_Init_Param.ODR[0] < 51.0f)
  {
    iis2mdc_data_rate_set(&iis2mdc_ctx_instance, IIS2MDC_ODR_50Hz);
  }
  else if (IIS2MDC_Init_Param.ODR[0] < 101.0f)
  {
    iis2mdc_data_rate_set(&iis2mdc_ctx_instance, IIS2MDC_ODR_100Hz);
  }

  iis2mdc_magnetic_raw_get(&iis2mdc_ctx_instance, (int16_t *)iis2mdc_mem);
}

static void IIS2MDC_Int_Callback(void)
{
  TimeStamp_iis2mdc = SM_GetTimeStamp_fromISR();
  osSemaphoreRelease(iis2mdc_data_ready_sem_id);
}

void IIS2MDC_Set_State(SM_Sensor_State_t state)
{
  IIS2MDC_Sensor_State = state;
}

void IIS2MDC_Start(void)
{
  IIS2MDC_Set_State(SM_SENSOR_STATE_INITIALIZING);
  osThreadResume(IIS2MDC_Thread_Id);
}

void IIS2MDC_Stop(void)
{
  IIS2MDC_Set_State(SM_SENSOR_STATE_SUSPENDING);
}

__weak void IIS2MDC_Data_Ready(uint8_t subSensorId, uint8_t *buf, uint16_t size, double timeStamp)
{

}

/**
  * @brief Get Sensor ID
  * @param None
  * @retval Sensor ID
  */
uint8_t IIS2MDC_Get_Id(void)
{
  return s_nIIS2MDC_id;
}

/**
  * @brief IIS2MDC Sensor Initialization
  * @param None
  * @retval None
  */
uint8_t IIS2MDC_Create_Sensor(const SM_Init_Param_t *pxParams)
{
  COM_Sensor_t *pSensor;

  s_nIIS2MDC_id = COM_AddSensor();

  if (s_nIIS2MDC_id == -1)
  {
    return 1; /* error */
  }

  pSensor = COM_GetSensor(s_nIIS2MDC_id);

  /* SENSOR DESCRIPTOR */
  strcpy(pSensor->sensorDescriptor.name, "IIS2MDC");
  pSensor->sensorDescriptor.nSubSensors = 1;

  /* SUBSENSOR 0 DESCRIPTOR */
  pSensor->sensorDescriptor.subSensorDescriptor[0].id = 0;
  pSensor->sensorDescriptor.subSensorDescriptor[0].sensorType = COM_TYPE_MAG;
  pSensor->sensorDescriptor.subSensorDescriptor[0].dimensions = 3;
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].dimensionsLabel[0], "x");
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].dimensionsLabel[1], "y");
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].dimensionsLabel[2], "z");
  pSensor->sensorDescriptor.subSensorDescriptor[0].dataType = DATA_TYPE_INT16;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[0] = 10.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[1] = 20.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[2] = 50.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[3] = 100.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[4] = COM_END_OF_LIST_FLOAT;
  pSensor->sensorDescriptor.subSensorDescriptor[0].samplesPerTimestamp[0] = 0;
  pSensor->sensorDescriptor.subSensorDescriptor[0].samplesPerTimestamp[1] = 1000;
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].unit, "gauss");
  pSensor->sensorDescriptor.subSensorDescriptor[0].FS[0] = 50.0f;
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
    pSensor->sensorStatus.subSensorStatus[0].FS = 50.0f;
    pSensor->sensorStatus.subSensorStatus[0].ODR = 100.0f;
  }
  pSensor->sensorStatus.subSensorStatus[0].sensitivity = 0.0015f;
  pSensor->sensorStatus.subSensorStatus[0].measuredODR = 0.0f;
  pSensor->sensorStatus.subSensorStatus[0].initialOffset = 0.0f;
  pSensor->sensorStatus.subSensorStatus[0].samplesPerTimestamp = 100;
  pSensor->sensorStatus.subSensorStatus[0].usbDataPacketSize = 600;
  pSensor->sensorStatus.subSensorStatus[0].sdWriteBufferSize = WRITE_BUFFER_SIZE_IIS2MDC;
  pSensor->sensorStatus.subSensorStatus[0].comChannelNumber = -1;
  pSensor->sensorStatus.subSensorStatus[0].ucfLoaded = 0;

  IIS2MDC_Init_Param.ODR[0] = pSensor->sensorStatus.subSensorStatus[0].ODR;
  IIS2MDC_Init_Param.FS[0] = pSensor->sensorStatus.subSensorStatus[0].FS;
  IIS2MDC_Init_Param.subSensorActive[0] = pSensor->sensorStatus.subSensorStatus[0].isActive;

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


