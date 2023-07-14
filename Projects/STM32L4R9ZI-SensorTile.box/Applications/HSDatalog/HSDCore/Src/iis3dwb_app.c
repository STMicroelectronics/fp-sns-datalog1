/**
  ******************************************************************************
  * @file    iis3dwb_app.c
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
#include "iis3dwb_app.h"
#include "main.h"
#include "iis3dwb_reg.h"
#include "com_manager.h"
#include <string.h>

/* Private includes ----------------------------------------------------------*/
#define WRITE_BUFFER_SIZE_IIS3DWB        (uint32_t)(32768)

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static int32_t s_nIIS3DWB_id = 0;

static uint8_t iis3dwb_mem[IIS3DWB_MAX_SAMPLES_PER_IT * 7];
static volatile double TimeStamp_iis3dwb;
uint16_t iis3dwb_samples_per_it;

SM_Init_Param_t IIS3DWB_Init_Param;
SM_Sensor_State_t IIS3DWB_Sensor_State = SM_SENSOR_STATE_INITIALIZING;

/* Semaphore used to wait on component interrupt */
static osSemaphoreId iis3dwb_data_ready_sem_id;
static osSemaphoreDef(iis3dwb_data_ready_sem);

/* Semaphore used to wait on BUS data read complete, managed by lower layer */
static osSemaphoreId iis3dwb_data_read_cmplt_sem_id;
static osSemaphoreDef(iis3dwb_data_read_cmplt_sem);

sensor_handle_t iis3dwb_hdl_instance = {IIS3DWB_ID, 0, IIS3DWB_SPI_CS_GPIO_Port, IIS3DWB_SPI_CS_Pin,
                                        &iis3dwb_data_read_cmplt_sem_id
                                       };
stmdev_ctx_t iis3dwb_ctx_instance = {SM_SPI_Write_Os, SM_SPI_Read_Os, &iis3dwb_hdl_instance};

/* Private function prototypes -----------------------------------------------*/

osThreadId IIS3DWB_Thread_Id;
static void IIS3DWB_Thread(void const *argument);

EXTI_HandleTypeDef iis3dwb_exti;
static void IIS3DWB_Int_Callback(void);
static void IIS3DWB_Sensor_Init(void);



/**
  * @brief IIS3DWB GPIO Initialization Function
  * @param None
  * @retval None
  */
void IIS3DWB_Peripheral_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IIS3DWB_SPI_CS_GPIO_Port, IIS3DWB_SPI_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : IIS3DWB_SPI_CS_Pin */
  GPIO_InitStruct.Pin = IIS3DWB_SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(IIS3DWB_SPI_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STTS751_INT_Pin IIS3DWB_INT1_Pin */
  GPIO_InitStruct.Pin =  IIS3DWB_INT1_Pin ;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  HAL_EXTI_GetHandle(&iis3dwb_exti, EXTI_LINE_14);
  HAL_EXTI_RegisterCallback(&iis3dwb_exti,  HAL_EXTI_COMMON_CB_ID, IIS3DWB_Int_Callback);

}

/**
  * @brief IIS3DWB Threads Creation
  * @param None
  * @retval None
  */
void IIS3DWB_OS_Init(void)
{
  /* Data read complete semaphore initialization */
  iis3dwb_data_read_cmplt_sem_id = osSemaphoreCreate(osSemaphore(iis3dwb_data_read_cmplt_sem), 1);
  vQueueAddToRegistry(iis3dwb_data_read_cmplt_sem_id, "iis3dwb_data_read_cmplt_sem_id");

  osSemaphoreWait(iis3dwb_data_read_cmplt_sem_id, osWaitForever);

  /* Data ready interrupt semaphore initialization */
  iis3dwb_data_ready_sem_id = osSemaphoreCreate(osSemaphore(iis3dwb_data_ready_sem), 1);
  vQueueAddToRegistry(iis3dwb_data_ready_sem_id, "iis3dwb_data_ready_sem_id");

  osSemaphoreWait(iis3dwb_data_ready_sem_id,  osWaitForever);


  /* Thread definition: read data */
  osThreadDef(IIS3DWB_Acquisition_Thread, IIS3DWB_Thread, HSD_IIS3DWB_THREAD_PRIO, 1, configMINIMAL_STACK_SIZE);
  /* Start thread 1 */
  IIS3DWB_Thread_Id = osThreadCreate(osThread(IIS3DWB_Acquisition_Thread), NULL);

}


static void IIS3DWB_Thread(void const *argument)
{
  (void) argument;

#if (configUSE_APPLICATION_TASK_TAG == 1 && defined(HSD_TASK_IIS3DWB_DEBUG_PIN))
  vTaskSetApplicationTaskTag(NULL, (TaskHookFunction_t)HSD_TASK_IIS3DWB_DEBUG_PIN);
#endif /* configUSE_APPLICATION_TASK_TAG == 1 && defined(HSD_TASK_IIS3DWB_DEBUG_PIN) */

#if (HSD_USE_DUMMY_DATA == 1)
  static uint16_t dummyDataCounter = 0;
#endif /* HSD_USE_DUMMY_DATA == 1 */

  uint8_t reg[2];
  volatile uint16_t fifo_level = 0;

  /* Suspend thread */
  osThreadSuspend(IIS3DWB_Thread_Id);

  for (;;)
  {
    if (IIS3DWB_Sensor_State == SM_SENSOR_STATE_INITIALIZING)
    {
      IIS3DWB_Sensor_Init();
      IIS3DWB_Sensor_State = SM_SENSOR_STATE_RUNNING;
    }
    else if (IIS3DWB_Sensor_State == SM_SENSOR_STATE_RUNNING)
    {
      osSemaphoreWait(iis3dwb_data_ready_sem_id,  osWaitForever);

      if (IIS3DWB_Sensor_State == SM_SENSOR_STATE_RUNNING) /* Change of state can happen while task blocked */
      {
        /* Check FIFO_WTM_IA anf fifo level. We do not use PID in order to avoid reading one register twice */;
        iis3dwb_read_reg(&iis3dwb_ctx_instance, IIS3DWB_FIFO_STATUS1, reg, 2);

        fifo_level = ((reg[1] & 0x03) << 8) + reg[0];

        if ((reg[1]) & 0x80  && (fifo_level >= iis3dwb_samples_per_it))
        {
          uint16_t i = 0;
          iis3dwb_read_reg(&iis3dwb_ctx_instance, IIS3DWB_FIFO_DATA_OUT_TAG, (uint8_t *)iis3dwb_mem,
                           iis3dwb_samples_per_it * 7);

          /* Arrange Data */
#if (HSD_USE_DUMMY_DATA == 1)
          int16_t *p16 = (int16_t *)iis3dwb_mem;

          for (i = 0; i < iis3dwb_samples_per_it; i++)
          {
            *p16++ = dummyDataCounter++;
            *p16++ = dummyDataCounter++;
            *p16++ = dummyDataCounter++;
          }
#else
          int16_t *p16src = (int16_t *)iis3dwb_mem;
          int16_t *p16dest = (int16_t *)iis3dwb_mem;
          for (i = 0; i < iis3dwb_samples_per_it; i++)
          {
            p16src = (int16_t *) & ((uint8_t *)(p16src))[1];
            *p16dest++ = *p16src++;
            *p16dest++ = *p16src++;
            *p16dest++ = *p16src++;
          }
#endif /* HSD_USE_DUMMY_DATA == 1 */
          IIS3DWB_Data_Ready(0, (uint8_t *)iis3dwb_mem, iis3dwb_samples_per_it * 6, TimeStamp_iis3dwb);
        }
      }
    }
    else if (IIS3DWB_Sensor_State == SM_SENSOR_STATE_SUSPENDING)
    {
#if (HSD_USE_DUMMY_DATA == 1)
      dummyDataCounter = 0;
#endif /* HSD_USE_DUMMY_DATA == 1 */
      iis3dwb_fifo_xl_batch_set(&iis3dwb_ctx_instance, IIS3DWB_XL_NOT_BATCHED);
      IIS3DWB_Sensor_State = SM_SENSOR_STATE_SUSPENDED;
      osThreadSuspend(IIS3DWB_Thread_Id);
    }
  }
}



static void IIS3DWB_Sensor_Init(void)
{
  uint8_t reg0;
  uint16_t iis3dwb_wtm_level;

  iis3dwb_i2c_interface_set(&iis3dwb_ctx_instance, IIS3DWB_I2C_DISABLE);
  iis3dwb_device_id_get(&iis3dwb_ctx_instance, (uint8_t *)&reg0);
  iis3dwb_reset_set(&iis3dwb_ctx_instance, 1);
  iis3dwb_read_reg(&iis3dwb_ctx_instance, IIS3DWB_CTRL1_XL, (uint8_t *)&reg0, 1);
  reg0 |= 0xA0;
  iis3dwb_write_reg(&iis3dwb_ctx_instance, IIS3DWB_CTRL1_XL, (uint8_t *)&reg0, 1);

  /* Calculation of watermark and samples per int*/
  iis3dwb_wtm_level = ((uint16_t)IIS3DWB_Init_Param.ODR[0] * (uint16_t)IIS3DWB_MAX_DRDY_PERIOD);
  if (iis3dwb_wtm_level > IIS3DWB_MAX_WTM_LEVEL)
  {
    iis3dwb_wtm_level = IIS3DWB_MAX_WTM_LEVEL;
  }
  else if (iis3dwb_wtm_level < IIS3DWB_MIN_WTM_LEVEL)
  {
    iis3dwb_wtm_level = IIS3DWB_MIN_WTM_LEVEL;
  }

  iis3dwb_samples_per_it = iis3dwb_wtm_level;

  /*Set fifo in continuous / stream mode*/
  iis3dwb_fifo_mode_set(&iis3dwb_ctx_instance, IIS3DWB_STREAM_MODE);
  /*Set watermark*/
  iis3dwb_fifo_watermark_set(&iis3dwb_ctx_instance, iis3dwb_wtm_level);
  /*Data Ready pulse mode*/
  iis3dwb_data_ready_mode_set(&iis3dwb_ctx_instance, IIS3DWB_DRDY_PULSED);
  /*Set full scale*/
  if (IIS3DWB_Init_Param.FS[0] < 3.0f)
  {
    iis3dwb_xl_full_scale_set(&iis3dwb_ctx_instance, IIS3DWB_2g);
  }
  else if (IIS3DWB_Init_Param.FS[0] < 5.0f)
  {
    iis3dwb_xl_full_scale_set(&iis3dwb_ctx_instance, IIS3DWB_4g);
  }
  else if (IIS3DWB_Init_Param.FS[0] < 9.0f)
  {
    iis3dwb_xl_full_scale_set(&iis3dwb_ctx_instance, IIS3DWB_8g);
  }
  else if (IIS3DWB_Init_Param.FS[0] < 17.0f)
  {
    iis3dwb_xl_full_scale_set(&iis3dwb_ctx_instance, IIS3DWB_16g);
  }

  /*Set 2nd stage filter*/
  iis3dwb_xl_filt_path_on_out_set(&iis3dwb_ctx_instance, IIS3DWB_LP_6k3Hz);
  /* FIFO_WTM_IA routing on pin INT1 */
  iis3dwb_pin_int1_route_t pin_int1_route;
  *(uint8_t *)&(pin_int1_route) = 0;
  pin_int1_route.fifo_th = 1;
  iis3dwb_pin_int1_route_set(&iis3dwb_ctx_instance, &pin_int1_route);

  /*Enable writing to FIFO*/
  iis3dwb_fifo_xl_batch_set(&iis3dwb_ctx_instance, IIS3DWB_XL_BATCHED_AT_26k7Hz);

  HAL_NVIC_EnableIRQ(IIS3DWB_INT1_EXTI_IRQn);
}

/* Data Ready */
static void IIS3DWB_Int_Callback(void)
{
  TimeStamp_iis3dwb = SM_GetTimeStamp_fromISR();
  osSemaphoreRelease(iis3dwb_data_ready_sem_id);
}

void IIS3DWB_Set_State(SM_Sensor_State_t state)
{
  IIS3DWB_Sensor_State = state;
}

void IIS3DWB_Start(void)
{
  IIS3DWB_Set_State(SM_SENSOR_STATE_INITIALIZING);
  osThreadResume(IIS3DWB_Thread_Id);
}

void IIS3DWB_Stop(void)
{
  IIS3DWB_Set_State(SM_SENSOR_STATE_SUSPENDING);
}


__weak void IIS3DWB_Data_Ready(uint8_t subSensorId, uint8_t *buf, uint16_t size, double timeStamp)
{

}

/**
  * @brief Get Sensor ID
  * @param None
  * @retval Sensor ID
  */
uint8_t IIS3DWB_Get_Id(void)
{
  return s_nIIS3DWB_id;
}

/**
  * @brief IIS3DWB Sensor Initialization
  * @param None
  * @retval None
  */
uint8_t IIS3DWB_Create_Sensor(const SM_Init_Param_t *pxParams)
{
  COM_Sensor_t *pSensor;

  s_nIIS3DWB_id = COM_AddSensor();

  if (s_nIIS3DWB_id == -1)
  {
    return 1; /* error */
  }

  pSensor = COM_GetSensor(s_nIIS3DWB_id);

  /* SENSOR DESCRIPTOR */
  strcpy(pSensor->sensorDescriptor.name, "IIS3DWB");
  pSensor->sensorDescriptor.nSubSensors = 1;

  /* SUBSENSOR 0 DESCRIPTOR */
  pSensor->sensorDescriptor.subSensorDescriptor[0].id = 0;
  pSensor->sensorDescriptor.subSensorDescriptor[0].sensorType = COM_TYPE_ACC;
  pSensor->sensorDescriptor.subSensorDescriptor[0].dataType = DATA_TYPE_INT16;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[0] = 26667.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[1] = COM_END_OF_LIST_FLOAT;  /* Terminate list */
  pSensor->sensorDescriptor.subSensorDescriptor[0].samplesPerTimestamp[0] = 0;
  pSensor->sensorDescriptor.subSensorDescriptor[0].samplesPerTimestamp[1] = 1000;
  pSensor->sensorDescriptor.subSensorDescriptor[0].dimensions = 3;
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].dimensionsLabel[0], "x");
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].dimensionsLabel[1], "y");
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].dimensionsLabel[2], "z");
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].unit, "g");
  pSensor->sensorDescriptor.subSensorDescriptor[0].FS[0] = 2.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].FS[1] = 4.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].FS[2] = 8.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].FS[3] = 16.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].FS[4] = COM_END_OF_LIST_FLOAT;

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
    pSensor->sensorStatus.subSensorStatus[0].FS = 16.0f;
    pSensor->sensorStatus.subSensorStatus[0].ODR = 26667.0f;
  }
  pSensor->sensorStatus.subSensorStatus[0].sensitivity = 0.0000305f *  pSensor->sensorStatus.subSensorStatus[0].FS;
  pSensor->sensorStatus.subSensorStatus[0].measuredODR = 0.0f;
  pSensor->sensorStatus.subSensorStatus[0].initialOffset = 0.0f;
  pSensor->sensorStatus.subSensorStatus[0].samplesPerTimestamp = 1000;
  pSensor->sensorStatus.subSensorStatus[0].usbDataPacketSize = 3000;
  pSensor->sensorStatus.subSensorStatus[0].sdWriteBufferSize = WRITE_BUFFER_SIZE_IIS3DWB;
  pSensor->sensorStatus.subSensorStatus[0].comChannelNumber = -1;
  pSensor->sensorStatus.subSensorStatus[0].ucfLoaded = 0;

  IIS3DWB_Init_Param.ODR[0] = pSensor->sensorStatus.subSensorStatus[0].ODR;
  IIS3DWB_Init_Param.FS[0] = pSensor->sensorStatus.subSensorStatus[0].FS;
  IIS3DWB_Init_Param.subSensorActive[0] = pSensor->sensorStatus.subSensorStatus[0].isActive;

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


