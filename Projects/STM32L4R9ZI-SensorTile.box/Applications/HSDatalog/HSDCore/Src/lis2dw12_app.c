/**
  ******************************************************************************
  * @file    lis2dw12_app.c
  * @brief   This file provides a set of functions to handle lis2dw12
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
#include "lis2dw12_app.h"
#include "main.h"
#include "lis2dw12_reg.h"
#include "com_manager.h"
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define WRITE_BUFFER_SIZE_LIS2DW12         ( ( uint32_t ) 8192 )

#define WTM_LEVEL                          ( 16 )

#define SAMPLES_PER_IT                     ( WTM_LEVEL )

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static int32_t s_nLIS2DW12_id = -1;

static volatile double TimeStamp_lis2dw12;

static int8_t lis2dw12_mem[SAMPLES_PER_IT * 6];

SM_Init_Param_t LIS2DW12_Init_Param;
SM_Sensor_State_t LIS2DW12_Sensor_State = SM_SENSOR_STATE_INITIALIZING;

EXTI_HandleTypeDef lis2dw12_exti;

/* Semaphore used to wait on component interrupt */
static osSemaphoreId lis2dw12_drdy_sem_id;
static osSemaphoreDef(lis2dw12_drdy_sem);

/* Semaphore used to wait on BUS data read complete, managed by lower layer */
static osSemaphoreId lis2dw12_read_cplt_sem_id;
static osSemaphoreDef(lis2dw12_read_cplt_sem);

static sensor_handle_t lis2dw12_hdl_instance =
{
  LIS2DW12_ID,
  0,
  LIS2DW12_SPI_CS_GPIO_Port,
  LIS2DW12_SPI_CS_Pin,
  &lis2dw12_read_cplt_sem_id
};
static stmdev_ctx_t lis2dw12_ctx_instance =
{
  SM_SPI1_Write_Os,
  SM_SPI1_Read_Os,
  NULL,
  &lis2dw12_hdl_instance
};

/* Private function prototypes -----------------------------------------------*/
osThreadId LIS2DW12_Thread_Id;
static void LIS2DW12_Thread(void const *argument);

static void LIS2DW12_Int_Callback(void);
static void LIS2DW12_Sensor_Init(void);

/**
  * @brief LIS2DW12 GPIO Initialization Function
  * @param None
  * @retval None
  */
void LIS2DW12_Peripheral_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct =
  {
    0
  };

  /* GPIO Ports Clock Enable */
  LIS2DW12_SPI_CS_GPIO_CLK_ENABLE();
  LIS2DW12_INT2_GPIO_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LIS2DW12_SPI_CS_GPIO_Port, LIS2DW12_SPI_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LIS2DW12_SPI_CS_Pin */
  GPIO_InitStruct.Pin = LIS2DW12_SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LIS2DW12_SPI_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LIS2DW12_INT2_Pin */
  GPIO_InitStruct.Pin = LIS2DW12_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LIS2DW12_INT2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(LIS2DW12_INT2_EXTI_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(LIS2DW12_INT2_EXTI_IRQn);

  HAL_EXTI_GetHandle(&lis2dw12_exti, LIS2DW12_INT2_EXTI_LINE);
  HAL_EXTI_RegisterCallback(&lis2dw12_exti, HAL_EXTI_COMMON_CB_ID, LIS2DW12_Int_Callback);
}

/**
  * @brief LIS2DW12 Thread Creation
  * @param None
  * @retval None
  */
void LIS2DW12_OS_Init(void)
{
  /* Data read complete semaphore initialization */
  lis2dw12_read_cplt_sem_id = osSemaphoreCreate(osSemaphore(lis2dw12_read_cplt_sem), 1);
  osSemaphoreWait(lis2dw12_read_cplt_sem_id, osWaitForever);

  /* Data ready interrupt semaphore initialization */
  lis2dw12_drdy_sem_id = osSemaphoreCreate(osSemaphore(lis2dw12_drdy_sem), 1);
  osSemaphoreWait(lis2dw12_drdy_sem_id, osWaitForever);

  /* Thread definition: read data */
  osThreadDef(LIS2DW12_Aquisition_Thread, LIS2DW12_Thread, HSD_LIS2DW12_THREAD_PRIO, 1, configMINIMAL_STACK_SIZE);
  /* Start thread */
  LIS2DW12_Thread_Id = osThreadCreate(osThread(LIS2DW12_Aquisition_Thread), NULL);
}

/**
  * @brief LIS2DW12 Thread
  * @param None
  * @retval None
  */
static void LIS2DW12_Thread(void const *argument)
{
  (void) argument;

#if (HSD_USE_DUMMY_DATA == 1)
  static uint16_t dummyDataCounter = 0;
#endif /* (HSD_USE_DUMMY_DATA == 1) */

  /* Suspend thread */
  osThreadSuspend(LIS2DW12_Thread_Id);

  for (;;)
  {
    if (LIS2DW12_Sensor_State == SM_SENSOR_STATE_INITIALIZING)
    {
      LIS2DW12_Sensor_Init();
      LIS2DW12_Sensor_State = SM_SENSOR_STATE_RUNNING;
    }
    else if (LIS2DW12_Sensor_State == SM_SENSOR_STATE_RUNNING)
    {
      osSemaphoreWait(lis2dw12_drdy_sem_id, osWaitForever);

      if (LIS2DW12_Sensor_State == SM_SENSOR_STATE_RUNNING) /* Change of state can happen while task blocked */
      {
        uint8_t wtmFlag = 0;
        uint8_t wtmLevel = 0;

        /* Check FIFO_WTM_IA anf fifo level */
        lis2dw12_fifo_wtm_flag_get(&lis2dw12_ctx_instance, &wtmFlag);
        lis2dw12_fifo_data_level_get(&lis2dw12_ctx_instance, &wtmLevel);

        if ((wtmFlag != 0) && (wtmLevel >= SAMPLES_PER_IT))
        {
          lis2dw12_read_reg(&lis2dw12_ctx_instance, LIS2DW12_OUT_X_L, (uint8_t *) lis2dw12_mem, SAMPLES_PER_IT * 6);

          uint8_t i = 0;
          int16_t *p16_src = (int16_t *) lis2dw12_mem;
          int16_t *p16_dest = (int16_t *) lis2dw12_mem;

          for (i = 0; i < SAMPLES_PER_IT; i++)
          {
            *p16_dest++ = *p16_src++ / 4;
            *p16_dest++ = *p16_src++ / 4;
            *p16_dest++ = *p16_src++ / 4;
          }
#if (HSD_USE_DUMMY_DATA == 1)
          uint16_t ii = 0;
          int16_t *p16 = (int16_t *) lis2dw12_mem;

          for (ii = 0; ii < SAMPLES_PER_IT; ii++)
          {
            *p16++ = dummyDataCounter++;
            *p16++ = dummyDataCounter++;
            *p16++ = dummyDataCounter++;
          }
#endif /* (HSD_USE_DUMMY_DATA == 1) */

          LIS2DW12_Data_Ready(0, (uint8_t *) lis2dw12_mem, SAMPLES_PER_IT * 6, TimeStamp_lis2dw12);
        }
      }
    }
    else if (LIS2DW12_Sensor_State == SM_SENSOR_STATE_SUSPENDING)
    {
#if (HSD_USE_DUMMY_DATA == 1)
      dummyDataCounter = 0;
#endif /* (HSD_USE_DUMMY_DATA == 1) */

      lis2dw12_data_rate_set(&lis2dw12_ctx_instance, LIS2DW12_XL_ODR_OFF);
      LIS2DW12_Sensor_State = SM_SENSOR_STATE_SUSPENDED;
      osThreadSuspend(LIS2DW12_Thread_Id);
    }
  }
}

static void LIS2DW12_Sensor_Init(void)
{
  uint8_t reg0;

  lis2dw12_reset_set(&lis2dw12_ctx_instance, PROPERTY_ENABLE);

  /* Enable 4-wire SPI mode */
  lis2dw12_spi_mode_set(&lis2dw12_ctx_instance, LIS2DW12_SPI_4_WIRE);

  /* Disable I2C mode */
  lis2dw12_i2c_interface_set(&lis2dw12_ctx_instance, LIS2DW12_I2C_DISABLE);

  lis2dw12_device_id_get(&lis2dw12_ctx_instance, &reg0);
  lis2dw12_boot_set(&lis2dw12_ctx_instance, PROPERTY_ENABLE);

  /* Enable Auto-Increment */
  lis2dw12_auto_increment_set(&lis2dw12_ctx_instance, PROPERTY_ENABLE);

  /* Enable BDU */
  lis2dw12_block_data_update_set(&lis2dw12_ctx_instance, PROPERTY_ENABLE);

  /* Disable FIFO to start */
  lis2dw12_fifo_mode_set(&lis2dw12_ctx_instance, LIS2DW12_BYPASS_MODE);

  /* Power down sensor */
  lis2dw12_data_rate_set(&lis2dw12_ctx_instance, LIS2DW12_XL_ODR_OFF);

  /* Set fifo in continuous / stream mode */
  lis2dw12_fifo_mode_set(&lis2dw12_ctx_instance, LIS2DW12_STREAM_MODE);

  /* Set watermark */
  lis2dw12_fifo_watermark_set(&lis2dw12_ctx_instance, WTM_LEVEL);

  /* FIFO_WTM routing on pin INT2 */
  lis2dw12_ctrl5_int2_pad_ctrl_t reg5 =
  {
    0
  };
  reg5.int2_fth = 1;
  lis2dw12_pin_int2_route_set(&lis2dw12_ctx_instance, &reg5);

  lis2dw12_ctrl_reg7_t reg7 =
  {
    0
  };
  reg7.interrupts_enable = 1;
  lis2dw12_write_reg(&lis2dw12_ctx_instance, LIS2DW12_CTRL_REG7, (uint8_t *) &reg7, 1);

  /* Full scale selection. */
  if (LIS2DW12_Init_Param.FS[0] < 3.0f)
  {
    lis2dw12_full_scale_set(&lis2dw12_ctx_instance, LIS2DW12_2g);
  }
  else if (LIS2DW12_Init_Param.FS[0] < 5.0f)
  {
    lis2dw12_full_scale_set(&lis2dw12_ctx_instance, LIS2DW12_4g);
  }
  else if (LIS2DW12_Init_Param.FS[0] < 9.0f)
  {
    lis2dw12_full_scale_set(&lis2dw12_ctx_instance, LIS2DW12_8g);
  }
  else if (LIS2DW12_Init_Param.FS[0] < 17.0f)
  {
    lis2dw12_full_scale_set(&lis2dw12_ctx_instance, LIS2DW12_16g);
  }

  if (LIS2DW12_Init_Param.ODR[0] < 2.0f)
  {
    lis2dw12_power_mode_set(&lis2dw12_ctx_instance, LIS2DW12_CONT_LOW_PWR_LOW_NOISE_4);
    lis2dw12_data_rate_set(&lis2dw12_ctx_instance, LIS2DW12_XL_ODR_1Hz6_LP_ONLY);
  }
  else if (LIS2DW12_Init_Param.ODR[0] < 13.0f)
  {
    lis2dw12_power_mode_set(&lis2dw12_ctx_instance, LIS2DW12_HIGH_PERFORMANCE_LOW_NOISE);
    lis2dw12_data_rate_set(&lis2dw12_ctx_instance, LIS2DW12_XL_ODR_12Hz5);
  }
  else if (LIS2DW12_Init_Param.ODR[0] < 26.0f)
  {
    lis2dw12_power_mode_set(&lis2dw12_ctx_instance, LIS2DW12_HIGH_PERFORMANCE_LOW_NOISE);
    lis2dw12_data_rate_set(&lis2dw12_ctx_instance, LIS2DW12_XL_ODR_25Hz);
  }
  else if (LIS2DW12_Init_Param.ODR[0] < 51.0f)
  {
    lis2dw12_power_mode_set(&lis2dw12_ctx_instance, LIS2DW12_HIGH_PERFORMANCE_LOW_NOISE);
    lis2dw12_data_rate_set(&lis2dw12_ctx_instance, LIS2DW12_XL_ODR_50Hz);
  }
  else if (LIS2DW12_Init_Param.ODR[0] < 101.0f)
  {
    lis2dw12_power_mode_set(&lis2dw12_ctx_instance, LIS2DW12_HIGH_PERFORMANCE_LOW_NOISE);
    lis2dw12_data_rate_set(&lis2dw12_ctx_instance, LIS2DW12_XL_ODR_100Hz);
  }
  else if (LIS2DW12_Init_Param.ODR[0] < 201.0f)
  {
    lis2dw12_power_mode_set(&lis2dw12_ctx_instance, LIS2DW12_HIGH_PERFORMANCE_LOW_NOISE);
    lis2dw12_data_rate_set(&lis2dw12_ctx_instance, LIS2DW12_XL_ODR_200Hz);
  }
  else if (LIS2DW12_Init_Param.ODR[0] < 401.0f)
  {
    lis2dw12_power_mode_set(&lis2dw12_ctx_instance, LIS2DW12_HIGH_PERFORMANCE_LOW_NOISE);
    lis2dw12_data_rate_set(&lis2dw12_ctx_instance, LIS2DW12_XL_ODR_400Hz);
  }
  else if (LIS2DW12_Init_Param.ODR[0] < 801.0f)
  {
    lis2dw12_power_mode_set(&lis2dw12_ctx_instance, LIS2DW12_HIGH_PERFORMANCE_LOW_NOISE);
    lis2dw12_data_rate_set(&lis2dw12_ctx_instance, LIS2DW12_XL_ODR_800Hz);
  }
  else if (LIS2DW12_Init_Param.ODR[0] < 1601.0f)
  {
    lis2dw12_power_mode_set(&lis2dw12_ctx_instance, LIS2DW12_HIGH_PERFORMANCE_LOW_NOISE);
    lis2dw12_data_rate_set(&lis2dw12_ctx_instance, LIS2DW12_XL_ODR_1k6Hz);
  }
}

/* Data Ready */
static void LIS2DW12_Int_Callback(void)
{
  TimeStamp_lis2dw12 = SM_GetTimeStamp_fromISR();
  osSemaphoreRelease(lis2dw12_drdy_sem_id);
}

void LIS2DW12_Set_State(SM_Sensor_State_t state)
{
  LIS2DW12_Sensor_State = state;
}

void LIS2DW12_Start(void)
{
  LIS2DW12_Set_State(SM_SENSOR_STATE_INITIALIZING);
  osThreadResume(LIS2DW12_Thread_Id);
}

void LIS2DW12_Stop(void)
{
  LIS2DW12_Set_State(SM_SENSOR_STATE_SUSPENDING);
}

__weak void LIS2DW12_Data_Ready(uint8_t subSensorId, uint8_t *buf, uint16_t size, double timeStamp)
{

}

/**
  * @brief Get Sensor ID
  * @param None
  * @retval Sensor ID
  */
uint8_t LIS2DW12_Get_Id(void)
{
  return s_nLIS2DW12_id;
}

/**
  * @brief LIS2DW12 Sensor Initialization
  * @param None
  * @retval None
  */
uint8_t LIS2DW12_Create_Sensor(const SM_Init_Param_t *pxParams)
{
  COM_Sensor_t *pSensor;

  s_nLIS2DW12_id = COM_AddSensor();

  if (s_nLIS2DW12_id == -1)
  {
    return 1; /* error */
  }

  pSensor = COM_GetSensor(s_nLIS2DW12_id);

  /* SENSOR DESCRIPTOR */
  strcpy(pSensor->sensorDescriptor.name, "LIS2DW12");
  pSensor->sensorDescriptor.nSubSensors = 1;

  /* SUBSENSOR 0 DESCRIPTOR */
  pSensor->sensorDescriptor.subSensorDescriptor[0].id = 0;
  pSensor->sensorDescriptor.subSensorDescriptor[0].sensorType = COM_TYPE_ACC;
  pSensor->sensorDescriptor.subSensorDescriptor[0].dimensions = 3;
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].dimensionsLabel[0], "x");
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].dimensionsLabel[1], "y");
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].dimensionsLabel[2], "z");
  pSensor->sensorDescriptor.subSensorDescriptor[0].dataType = DATA_TYPE_INT16;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[0] = 1.6f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[1] = 12.5f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[2] = 25.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[3] = 50.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[4] = 100.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[5] = 200.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[6] = 400.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[7] = 800.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[8] = 1600.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[9] = COM_END_OF_LIST_FLOAT;
  pSensor->sensorDescriptor.subSensorDescriptor[0].samplesPerTimestamp[0] = 0;
  pSensor->sensorDescriptor.subSensorDescriptor[0].samplesPerTimestamp[1] = 1000;
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
    pSensor->sensorStatus.subSensorStatus[0].ODR = 1600.0f;
  }

  pSensor->sensorStatus.subSensorStatus[0].sensitivity = 0.000122f * pSensor->sensorStatus.subSensorStatus[0].FS;
  pSensor->sensorStatus.subSensorStatus[0].measuredODR = 0.0f;
  pSensor->sensorStatus.subSensorStatus[0].initialOffset = 0.0f;
  pSensor->sensorStatus.subSensorStatus[0].samplesPerTimestamp = 800;
  pSensor->sensorStatus.subSensorStatus[0].usbDataPacketSize = 2400;
  pSensor->sensorStatus.subSensorStatus[0].sdWriteBufferSize = WRITE_BUFFER_SIZE_LIS2DW12;
  pSensor->sensorStatus.subSensorStatus[0].comChannelNumber = -1;
  pSensor->sensorStatus.subSensorStatus[0].ucfLoaded = 0;

  LIS2DW12_Init_Param.ODR[0] = pSensor->sensorStatus.subSensorStatus[0].ODR;
  LIS2DW12_Init_Param.FS[0] = pSensor->sensorStatus.subSensorStatus[0].FS;
  LIS2DW12_Init_Param.subSensorActive[0] = pSensor->sensorStatus.subSensorStatus[0].isActive;

  return 0;
}

