/**
  ******************************************************************************
  * @file    lis2mdl_app.c
  * @brief   This file provides a set of functions to handle lis2mdl
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
#include "lis2mdl_app.h"
#include "main.h"
#include "lis2mdl_reg.h"
#include "com_manager.h"
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define WRITE_BUFFER_SIZE_LIS2MDL        (uint32_t)(512)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static int32_t s_nLIS2MDL_id = -1;

static volatile double TimeStamp_lis2mdl;

static uint8_t lis2mdl_mem[6];

SM_Init_Param_t LIS2MDL_Init_Param;
SM_Sensor_State_t LIS2MDL_Sensor_State = SM_SENSOR_STATE_INITIALIZING;

static uint16_t taskDelay = 1000;

/* Semaphore used to wait on BUS data read complete, managed by lower layer */
static osSemaphoreId lis2mdl_read_cplt_sem_id;
static osSemaphoreDef(lis2mdl_read_cplt_sem);

static sensor_handle_t lis2mdl_hdl_instance =
{
  0,
  0,
  LIS2MDL_SPI_CS_GPIO_Port,
  LIS2MDL_SPI_CS_Pin,
  &lis2mdl_read_cplt_sem_id
};
static stmdev_ctx_t lis2mdl_ctx_instance =
{
  SM_SPI3_Write_Os,
  SM_SPI3_Read_Os,
  NULL,
  &lis2mdl_hdl_instance
};

/* Private function prototypes -----------------------------------------------*/
osThreadId LIS2MDL_Thread_Id;
static void LIS2MDL_Thread(void const *argument);

static void LIS2MDL_Sensor_Init(void);

/**
  * @brief LIS2MDL GPIO Initialization Function
  * @param None
  * @retval None
  */
void LIS2MDL_Peripheral_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct =
  {
    0
  };

  /* GPIO Ports Clock Enable */
  LIS2MDL_SPI_CS_GPIO_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LIS2MDL_SPI_CS_GPIO_Port, LIS2MDL_SPI_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LIS2MDL_SPI_CS_Pin */
  GPIO_InitStruct.Pin = LIS2MDL_SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LIS2MDL_SPI_CS_GPIO_Port, &GPIO_InitStruct);
}

/**
  * @brief LIS2MDL Thread Creation
  * @param None
  * @retval None
  */
void LIS2MDL_OS_Init(void)
{
  /* Data read complete semaphore initialization */
  lis2mdl_read_cplt_sem_id = osSemaphoreCreate(osSemaphore(lis2mdl_read_cplt_sem), 1);
  osSemaphoreWait(lis2mdl_read_cplt_sem_id, osWaitForever);

  /* Thread definition: read data */
  osThreadDef(LIS2MDL_Acquisition_Thread, LIS2MDL_Thread, HSD_LIS2MDL_THREAD_PRIO, 1, configMINIMAL_STACK_SIZE);
  /* Start thread */
  LIS2MDL_Thread_Id = osThreadCreate(osThread(LIS2MDL_Acquisition_Thread), NULL);
}

/**
  * @brief LIS2MDL Thread
  * @param None
  * @retval None
  */
static void LIS2MDL_Thread(void const *argument)
{
  (void) argument;

#if (HSD_USE_DUMMY_DATA == 1)
  static uint16_t dummyDataCounter = 0;
#endif /* (HSD_USE_DUMMY_DATA == 1) */

  /* Suspend thread */
  osThreadSuspend(LIS2MDL_Thread_Id);

  for (;;)
  {
    if (LIS2MDL_Sensor_State == SM_SENSOR_STATE_INITIALIZING)
    {
      LIS2MDL_Sensor_Init();
      LIS2MDL_Sensor_State = SM_SENSOR_STATE_RUNNING;
    }
    else if (LIS2MDL_Sensor_State == SM_SENSOR_STATE_RUNNING)
    {
      vTaskDelay(taskDelay);

      if (LIS2MDL_Sensor_State == SM_SENSOR_STATE_RUNNING) /* Change of state can happen while task blocked */
      {
        TimeStamp_lis2mdl = SM_GetTimeStamp();

        lis2mdl_magnetic_raw_get(&lis2mdl_ctx_instance, (int16_t *) lis2mdl_mem);

#if (HSD_USE_DUMMY_DATA == 1)
        int16_t *p16 = (int16_t *) lis2mdl_mem;

        *p16++ = dummyDataCounter++;
        *p16++ = dummyDataCounter++;
        *p16++ = dummyDataCounter++;
#endif /* (HSD_USE_DUMMY_DATA == 1) */
        LIS2MDL_Data_Ready(0, (uint8_t *) lis2mdl_mem, 6, TimeStamp_lis2mdl);
      }
    }
    else if (LIS2MDL_Sensor_State == SM_SENSOR_STATE_SUSPENDING)
    {
#if (HSD_USE_DUMMY_DATA == 1)
      dummyDataCounter = 0;
#endif /* (HSD_USE_DUMMY_DATA == 1) */

      lis2mdl_operating_mode_set(&lis2mdl_ctx_instance, LIS2MDL_POWER_DOWN);
      LIS2MDL_Sensor_State = SM_SENSOR_STATE_SUSPENDED;
      osThreadSuspend(LIS2MDL_Thread_Id);
    }
  }
}

static void LIS2MDL_Sensor_Init(void)
{
  uint8_t reg0;

  if (LIS2MDL_COM_MODE == LIS2MDL_COM_SPI_4_WIRE)
  {
    lis2mdl_cfg_reg_c_t reg =
    {
      0
    };
    reg._4wspi = 1;

    lis2mdl_write_reg(&lis2mdl_ctx_instance, LIS2MDL_CFG_REG_C, (uint8_t *) &reg, 1);
  }

  lis2mdl_i2c_interface_set(&lis2mdl_ctx_instance, LIS2MDL_I2C_DISABLE);

  lis2mdl_device_id_get(&lis2mdl_ctx_instance, &reg0);

  /* Enable BDU */
  lis2mdl_block_data_update_set(&lis2mdl_ctx_instance, PROPERTY_ENABLE);

  /* Self Test disable */
  lis2mdl_self_test_set(&lis2mdl_ctx_instance, PROPERTY_DISABLE);

  if (LIS2MDL_Init_Param.ODR[0] < 11.0f)
  {
    taskDelay = 100;
    lis2mdl_data_rate_set(&lis2mdl_ctx_instance, LIS2MDL_ODR_10Hz);
  }
  else if (LIS2MDL_Init_Param.ODR[0] < 21.0f)
  {
    taskDelay = 50;
    lis2mdl_data_rate_set(&lis2mdl_ctx_instance, LIS2MDL_ODR_20Hz);
  }
  else if (LIS2MDL_Init_Param.ODR[0] < 51.0f)
  {
    taskDelay = 20;
    lis2mdl_data_rate_set(&lis2mdl_ctx_instance, LIS2MDL_ODR_50Hz);
  }
  else if (LIS2MDL_Init_Param.ODR[0] < 101.0f)
  {
    taskDelay = 10;
    lis2mdl_data_rate_set(&lis2mdl_ctx_instance, LIS2MDL_ODR_100Hz);
  }

  lis2mdl_operating_mode_set(&lis2mdl_ctx_instance, LIS2MDL_CONTINUOUS_MODE);
}

void LIS2MDL_Set_State(SM_Sensor_State_t state)
{
  LIS2MDL_Sensor_State = state;
}

void LIS2MDL_Start(void)
{
  LIS2MDL_Set_State(SM_SENSOR_STATE_INITIALIZING);
  osThreadResume(LIS2MDL_Thread_Id);
}

void LIS2MDL_Stop(void)
{
  LIS2MDL_Set_State(SM_SENSOR_STATE_SUSPENDING);
}

__weak void LIS2MDL_Data_Ready(uint8_t subSensorId, uint8_t *buf, uint16_t size, double timeStamp)
{

}

/**
  * @brief Get Sensor ID
  * @param None
  * @retval Sensor ID
  */
uint8_t LIS2MDL_Get_Id(void)
{
  return s_nLIS2MDL_id;
}

/**
  * @brief LIS2MDL Sensor Initialization
  * @param None
  * @retval None
  */
uint8_t LIS2MDL_Create_Sensor(const SM_Init_Param_t *pxParams)
{
  COM_Sensor_t *pSensor;

  s_nLIS2MDL_id = COM_AddSensor();

  if (s_nLIS2MDL_id == -1)
  {
    return 1; /* error */
  }

  pSensor = COM_GetSensor(s_nLIS2MDL_id);

  /* SENSOR DESCRIPTOR */
  strcpy(pSensor->sensorDescriptor.name, "LIS2MDL");
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
  pSensor->sensorStatus.subSensorStatus[0].sdWriteBufferSize = WRITE_BUFFER_SIZE_LIS2MDL;
  pSensor->sensorStatus.subSensorStatus[0].comChannelNumber = -1;
  pSensor->sensorStatus.subSensorStatus[0].ucfLoaded = 0;

  LIS2MDL_Init_Param.ODR[0] = pSensor->sensorStatus.subSensorStatus[0].ODR;
  LIS2MDL_Init_Param.FS[0] = pSensor->sensorStatus.subSensorStatus[0].FS;
  LIS2MDL_Init_Param.subSensorActive[0] = pSensor->sensorStatus.subSensorStatus[0].isActive;

  return 0;
}

