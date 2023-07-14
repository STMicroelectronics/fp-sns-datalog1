/**
  ******************************************************************************
  * @file    lis3dhh_app.c
  * @brief   This file provides a set of functions to handle lis3dhh
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
#include "lis3dhh_app.h"
#include "main.h"
#include "lis3dhh_reg.h"
#include "com_manager.h"
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define WRITE_BUFFER_SIZE_LIS3DHH       ( ( uint32_t ) 32768 )

#define WTM_LEVEL                          ( 16 )

#define SAMPLES_PER_IT                     ( WTM_LEVEL )

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static int32_t s_nLIS3DHH_id = -1;

static volatile double TimeStamp_lis3dhh;

static uint8_t lis3dhh_mem[SAMPLES_PER_IT * 6];

SM_Init_Param_t LIS3DHH_Init_Param;
SM_Sensor_State_t LIS3DHH_Sensor_State = SM_SENSOR_STATE_INITIALIZING;

EXTI_HandleTypeDef lis3dhh_exti;

/* Semaphore used to wait on component interrupt */
static osSemaphoreId lis3dhh_drdy_sem_id;
static osSemaphoreDef(lis3dhh_drdy_sem);

/* Semaphore used to wait on BUS data read complete, managed by lower layer */
static osSemaphoreId lis3dhh_read_cplt_sem_id;
static osSemaphoreDef(lis3dhh_read_cplt_sem);

static sensor_handle_t lis3dhh_hdl_instance =
{
  LIS3DHH_ID,
  0,
  LIS3DHH_SPI_CS_GPIO_Port,
  LIS3DHH_SPI_CS_Pin,
  &lis3dhh_read_cplt_sem_id
};
static stmdev_ctx_t lis3dhh_ctx_instance =
{
  SM_SPI1_Write_Os,
  SM_SPI1_Read_Os,
  NULL,
  &lis3dhh_hdl_instance
};

/* Private function prototypes -----------------------------------------------*/
osThreadId LIS3DHH_Thread_Id;
static void LIS3DHH_Thread(void const *argument);

static void LIS3DHH_Int_Callback(void);
static void LIS3DHH_Sensor_Init(void);

/**
  * @brief LIS3DHH GPIO Initialization Function
  * @param None
  * @retval None
  */
void LIS3DHH_Peripheral_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct =
  {
    0
  };

  /* GPIO Ports Clock Enable */
  LIS3DHH_SPI_CS_GPIO_CLK_ENABLE();
  LIS3DHH_INT2_GPIO_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LIS3DHH_SPI_CS_GPIO_Port, LIS3DHH_SPI_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LIS3DHH_SPI_CS_Pin */
  GPIO_InitStruct.Pin = LIS3DHH_SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LIS3DHH_SPI_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LIS3DHH_INT2_Pin */
  GPIO_InitStruct.Pin = LIS3DHH_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LIS3DHH_INT2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(LIS3DHH_INT2_EXTI_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(LIS3DHH_INT2_EXTI_IRQn);

  HAL_EXTI_GetHandle(&lis3dhh_exti, LIS3DHH_INT2_EXTI_LINE);
  HAL_EXTI_RegisterCallback(&lis3dhh_exti, HAL_EXTI_COMMON_CB_ID, LIS3DHH_Int_Callback);
}

/**
  * @brief LIS3DHH Threads Creation
  * @param None
  * @retval None
  */
void LIS3DHH_OS_Init(void)
{
  /* Data read complete semaphore initialization */
  lis3dhh_read_cplt_sem_id = osSemaphoreCreate(osSemaphore(lis3dhh_read_cplt_sem), 1);
  osSemaphoreWait(lis3dhh_read_cplt_sem_id, osWaitForever);

  /* Data ready interrupt semaphore initialization */
  lis3dhh_drdy_sem_id = osSemaphoreCreate(osSemaphore(lis3dhh_drdy_sem), 1);
  osSemaphoreWait(lis3dhh_drdy_sem_id, osWaitForever);

  /* Thread definition: read data */
  osThreadDef(LIS3DHH_Acquisition_Thread, LIS3DHH_Thread, HSD_LIS3DHH_THREAD_PRIO, 1, configMINIMAL_STACK_SIZE);
  /* Start thread */
  LIS3DHH_Thread_Id = osThreadCreate(osThread(LIS3DHH_Acquisition_Thread), NULL);
}

/**
  * @brief LIS3DHH Thread
  * @param None
  * @retval None
  */
static void LIS3DHH_Thread(void const *argument)
{
  (void) argument;

#if (HSD_USE_DUMMY_DATA == 1)
  static uint16_t dummyDataCounter = 0;
#endif /* (HSD_USE_DUMMY_DATA == 1) */

  /* Suspend thread */
  osThreadSuspend(LIS3DHH_Thread_Id);

  for (;;)
  {
    if (LIS3DHH_Sensor_State == SM_SENSOR_STATE_INITIALIZING)
    {
      LIS3DHH_Sensor_Init();
      LIS3DHH_Sensor_State = SM_SENSOR_STATE_RUNNING;
    }
    else if (LIS3DHH_Sensor_State == SM_SENSOR_STATE_RUNNING)
    {
      osSemaphoreWait(lis3dhh_drdy_sem_id, osWaitForever);

      if (LIS3DHH_Sensor_State == SM_SENSOR_STATE_RUNNING) /* Change of state can happen while task blocked */
      {
        lis3dhh_fifo_src_t fifo_src_reg;

        /* Check FIFO_WTM_IA anf fifo level */
        lis3dhh_fifo_status_get(&lis3dhh_ctx_instance, &fifo_src_reg);

        if ((fifo_src_reg.fth != 0) && (fifo_src_reg.fss >= SAMPLES_PER_IT))
        {
          lis3dhh_read_reg(&lis3dhh_ctx_instance, LIS3DHH_OUT_X_L_XL, (uint8_t *) lis3dhh_mem, SAMPLES_PER_IT * 6);

#if (HSD_USE_DUMMY_DATA == 1)
          uint16_t i = 0;
          int16_t *p16 = (int16_t *) lis3dhh_mem;

          for (i = 0; i < SAMPLES_PER_IT; i++)
          {
            *p16++ = dummyDataCounter++;
            *p16++ = dummyDataCounter++;
            *p16++ = dummyDataCounter++;
          }
#endif /* (HSD_USE_DUMMY_DATA == 1) */

          LIS3DHH_Data_Ready(0, (uint8_t *) lis3dhh_mem, SAMPLES_PER_IT * 6, TimeStamp_lis3dhh);
        }
      }
    }
    else if (LIS3DHH_Sensor_State == SM_SENSOR_STATE_SUSPENDING)
    {
#if (HSD_USE_DUMMY_DATA == 1)
      dummyDataCounter = 0;
#endif /* (HSD_USE_DUMMY_DATA == 1) */

      lis3dhh_data_rate_set(&lis3dhh_ctx_instance, LIS3DHH_POWER_DOWN);
      LIS3DHH_Sensor_State = SM_SENSOR_STATE_SUSPENDED;
      osThreadSuspend(LIS3DHH_Thread_Id);
    }
  }
}

static void LIS3DHH_Sensor_Init(void)
{
  uint8_t reg0;

  lis3dhh_reset_set(&lis3dhh_ctx_instance, PROPERTY_ENABLE);

  lis3dhh_device_id_get(&lis3dhh_ctx_instance, &reg0);

  lis3dhh_fifo_block_spi_hs_set(&lis3dhh_ctx_instance, PROPERTY_ENABLE);

  /* Enable Auto-Increment */
  lis3dhh_auto_add_inc_set(&lis3dhh_ctx_instance, PROPERTY_ENABLE);

  /* Enable BDU */
  lis3dhh_block_data_update_set(&lis3dhh_ctx_instance, PROPERTY_ENABLE);

  /* Disable FIFO to start */
  lis3dhh_fifo_mode_set(&lis3dhh_ctx_instance, LIS3DHH_BYPASS_MODE);

  /* Power down sensor */
  lis3dhh_data_rate_set(&lis3dhh_ctx_instance, LIS3DHH_POWER_DOWN);

  /* Set fifo in continuous / stream mode */
  lis3dhh_fifo_mode_set(&lis3dhh_ctx_instance, LIS3DHH_DYNAMIC_STREAM_MODE);

  /* Set watermark */
  lis3dhh_fifo_watermark_set(&lis3dhh_ctx_instance, WTM_LEVEL);

  /* FIFO_WTM routing on pin INT2 */
  lis3dhh_fifo_threshold_on_int2_set(&lis3dhh_ctx_instance, PROPERTY_ENABLE);

  /* Enable writing to FIFO */
  lis3dhh_fifo_set(&lis3dhh_ctx_instance, PROPERTY_ENABLE);

  /* Enable sensor */
  lis3dhh_data_rate_set(&lis3dhh_ctx_instance, LIS3DHH_1kHz1);
}

/* Data Ready */
static void LIS3DHH_Int_Callback(void)
{
  TimeStamp_lis3dhh = SM_GetTimeStamp_fromISR();
  osSemaphoreRelease(lis3dhh_drdy_sem_id);
}

void LIS3DHH_Set_State(SM_Sensor_State_t state)
{
  LIS3DHH_Sensor_State = state;
}

void LIS3DHH_Start(void)
{
  LIS3DHH_Set_State(SM_SENSOR_STATE_INITIALIZING);
  osThreadResume(LIS3DHH_Thread_Id);
}

void LIS3DHH_Stop(void)
{
  LIS3DHH_Set_State(SM_SENSOR_STATE_SUSPENDING);
}

__weak void LIS3DHH_Data_Ready(uint8_t subSensorId, uint8_t *buf, uint16_t size, double timeStamp)
{

}

/**
  * @brief Get Sensor ID
  * @param None
  * @retval Sensor ID
  */
uint8_t LIS3DHH_Get_Id(void)
{
  return s_nLIS3DHH_id;
}

/**
  * @brief LIS3DHH Sensor Initialization
  * @param None
  * @retval None
  */
uint8_t LIS3DHH_Create_Sensor(const SM_Init_Param_t *pxParams)
{
  COM_Sensor_t *pSensor;

  s_nLIS3DHH_id = COM_AddSensor();

  if (s_nLIS3DHH_id == -1)
  {
    return 1; /* error */
  }

  pSensor = COM_GetSensor(s_nLIS3DHH_id);

  /* SENSOR DESCRIPTOR */
  strcpy(pSensor->sensorDescriptor.name, "LIS3DHH");
  pSensor->sensorDescriptor.nSubSensors = 1;

  /* SUBSENSOR 0 DESCRIPTOR */
  pSensor->sensorDescriptor.subSensorDescriptor[0].id = 0;
  pSensor->sensorDescriptor.subSensorDescriptor[0].sensorType = COM_TYPE_ACC;
  pSensor->sensorDescriptor.subSensorDescriptor[0].dimensions = 3;
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].dimensionsLabel[0], "x");
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].dimensionsLabel[1], "y");
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].dimensionsLabel[2], "z");
  pSensor->sensorDescriptor.subSensorDescriptor[0].dataType = DATA_TYPE_INT16;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[0] = 1100.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[1] = COM_END_OF_LIST_FLOAT;
  pSensor->sensorDescriptor.subSensorDescriptor[0].samplesPerTimestamp[0] = 0;
  pSensor->sensorDescriptor.subSensorDescriptor[0].samplesPerTimestamp[1] = 1000;
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].unit, "g");
  pSensor->sensorDescriptor.subSensorDescriptor[0].FS[0] = 2.5f;
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
    pSensor->sensorStatus.subSensorStatus[0].FS = 2.5f;
    pSensor->sensorStatus.subSensorStatus[0].ODR = 1100.0f;
  }

  pSensor->sensorStatus.subSensorStatus[0].sensitivity = 0.000076f;
  pSensor->sensorStatus.subSensorStatus[0].measuredODR = 0.0f;
  pSensor->sensorStatus.subSensorStatus[0].initialOffset = 0.0f;
  pSensor->sensorStatus.subSensorStatus[0].samplesPerTimestamp = 1000;
  pSensor->sensorStatus.subSensorStatus[0].usbDataPacketSize = 3000;
  pSensor->sensorStatus.subSensorStatus[0].sdWriteBufferSize = WRITE_BUFFER_SIZE_LIS3DHH;
  pSensor->sensorStatus.subSensorStatus[0].comChannelNumber = -1;
  pSensor->sensorStatus.subSensorStatus[0].ucfLoaded = 0;

  LIS3DHH_Init_Param.ODR[0] = pSensor->sensorStatus.subSensorStatus[0].ODR;
  LIS3DHH_Init_Param.FS[0] = pSensor->sensorStatus.subSensorStatus[0].FS;
  LIS3DHH_Init_Param.subSensorActive[0] = pSensor->sensorStatus.subSensorStatus[0].isActive;

  return 0;
}

