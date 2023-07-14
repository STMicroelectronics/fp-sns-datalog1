/**
  ******************************************************************************
  * @file    iis2dh_app.c
  * @author  SRA - MCD
  *
  *
  * @brief   This file provides a set of functions to handle iis2dh
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
#include "iis2dh_app.h"
#include "main.h"
#include "sensors_manager.h"
#include "iis2dh_reg.h"
#include "com_manager.h"
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define WRITE_BUFFER_SIZE_IIS2DH         (uint32_t)(8192)
#define DEFAULT_TASK_DELAY  (uint16_t)(1000)

#define HTONS(A)  ((((uint16_t)(A) & 0xff00) >> 8) | \
                   (((uint16_t)(A) & 0x00ff) << 8))

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static int32_t s_nIIS2DH_id = 0;

/* taskDelay is set so that the number of samples in FIFO shouldn't be more than 16
   so the buffer is 16*2 to have some margin */
static uint8_t iis2dh_mem[32 * 7];

static uint16_t taskDelay = DEFAULT_TASK_DELAY;
static volatile double TimeStamp_iis2dh;
volatile uint8_t testWTM = 0;

SM_Init_Param_t IIS2DH_Init_Param;
SM_Sensor_State_t IIS2DH_Sensor_State = SM_SENSOR_STATE_INITIALIZING;

/* Semaphore used to wait on component interrupt */
osSemaphoreId  iis2dh_DreadySem_id;
osSemaphoreDef(iis2dh_DreadySem);

/* Semaphore used to wait on BUS data read complete, managed by lower layer */
osSemaphoreId  iis2dhReadSem_id;
osSemaphoreDef(iis2dhReadSem);

static sensor_handle_t iis2dh_hdl_instance = {IIS2DH_ID, 0, IIS2DH_SPI_CS_GPIO_Port, IIS2DH_SPI_CS_Pin,
                                              &iis2dhReadSem_id
                                             };
static stmdev_ctx_t iis2dh_ctx_instance = {SM_SPI_Write_Os, SM_SPI_Read_Os, & iis2dh_hdl_instance};

/* Private function prototypes -----------------------------------------------*/

osThreadId IIS2DH_Thread_Id;
static void IIS2DH_Thread(void const *argument);

EXTI_HandleTypeDef iis2dh_exti;
static void IIS2DH_Int_Callback(void);
static void IIS2DH_Sensor_Init(void);



/**
  * @brief IIS3DWB GPIO Initialization Function
  * @param None
  * @retval None
  */
void IIS2DH_Peripheral_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IIS2DH_SPI_CS_GPIO_Port, IIS2DH_SPI_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : IIS3DWB_SPI_CS_Pin */
  GPIO_InitStruct.Pin = IIS2DH_SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(IIS2DH_SPI_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STTS751_INT_Pin IIS3DWB_INT1_Pin */
  GPIO_InitStruct.Pin =  IIS2DH_INT2_Pin ;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IIS2DH_INT2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_EXTI_GetHandle(& iis2dh_exti, EXTI_LINE_2);
  HAL_EXTI_RegisterCallback(& iis2dh_exti,  HAL_EXTI_COMMON_CB_ID, IIS2DH_Int_Callback);

}


void IIS2DH_OS_Init(void)
{
  /* Data read complete semaphore initialization */
  iis2dhReadSem_id = osSemaphoreCreate(osSemaphore(iis2dhReadSem), 1);
  osSemaphoreWait(iis2dhReadSem_id, osWaitForever);

  /* Data ready interrupt semaphore initialization */
  iis2dh_DreadySem_id = osSemaphoreCreate(osSemaphore(iis2dh_DreadySem), 1);
  osSemaphoreWait(iis2dh_DreadySem_id,  osWaitForever);

  /* Thread 1 definition */
  osThreadDef(IIS2DH_RD_USR_THREAD, IIS2DH_Thread, HSD_IIS2DH_THREAD_PRIO, 1, configMINIMAL_STACK_SIZE);
  /* Start thread 1 */
  IIS2DH_Thread_Id = osThreadCreate(osThread(IIS2DH_RD_USR_THREAD), NULL);

}



static void IIS2DH_Thread(void const *argument)
{
  (void) argument;

#if (configUSE_APPLICATION_TASK_TAG == 1 && defined(TASK_IIS2DH_DEBUG_PIN))
  vTaskSetApplicationTaskTag(NULL, (TaskHookFunction_t)TASK_IIS2DH_DEBUG_PIN);
#endif /* configUSE_APPLICATION_TASK_TAG == 1 && defined(TASK_IIS2DH_DEBUG_PIN) */

#if (HSD_USE_DUMMY_DATA == 1)
  static uint16_t dummyDataCounter = 0;
#endif /* HSD_USE_DUMMY_DATA == 1 */

  /* Suspend thread */
  osThreadSuspend(IIS2DH_Thread_Id);

  for (;;)
  {
    if (IIS2DH_Sensor_State == SM_SENSOR_STATE_INITIALIZING)
    {
      IIS2DH_Sensor_Init();
      IIS2DH_Sensor_State = SM_SENSOR_STATE_RUNNING;
    }
    else if (IIS2DH_Sensor_State == SM_SENSOR_STATE_RUNNING)
    {
      vTaskDelay(taskDelay);

      if (IIS2DH_Sensor_State == SM_SENSOR_STATE_RUNNING) /* Change of state can happen while task blocked */
      {
        iis2dh_fifo_data_level_get(&iis2dh_ctx_instance, (uint8_t *)&testWTM);

        /* This needs to be after iis2dh_fifo_data_level_get to be
            closer to the real acquisition of the last item in FIFO */
        TimeStamp_iis2dh = SM_GetTimeStamp();

        iis2dh_read_reg(& iis2dh_ctx_instance, IIS2DH_OUT_X_L, (uint8_t *) iis2dh_mem, testWTM * 6);

#if (HSD_USE_DUMMY_DATA == 1)
        uint16_t i = 0;
        int16_t *p16 = (int16_t *)iis2dh_mem;

        for (i = 0; i < testWTM * 3 ; i++)
        {
          *p16++ = dummyDataCounter++;
        }
#endif /* HSD_USE_DUMMY_DATA == 1 */

        IIS2DH_Data_Ready(0, (uint8_t *)iis2dh_mem, testWTM * 6, TimeStamp_iis2dh);
      }
    }
    else if (IIS2DH_Sensor_State == SM_SENSOR_STATE_SUSPENDING)
    {
#if (HSD_USE_DUMMY_DATA == 1)
      dummyDataCounter = 0;
#endif /* HSD_USE_DUMMY_DATA == 1 */
      iis2dh_data_rate_set(&iis2dh_ctx_instance, IIS2DH_POWER_DOWN);
      IIS2DH_Sensor_State = SM_SENSOR_STATE_SUSPENDED;
      osThreadSuspend(IIS2DH_Thread_Id);
    }
  }
}


static void IIS2DH_Sensor_Init(void)
{
  uint8_t reg0;

  iis2dh_boot_set(&iis2dh_ctx_instance, PROPERTY_ENABLE);
  iis2dh_spi_mode_set(&iis2dh_ctx_instance, IIS2DH_SPI_4_WIRE);

  iis2dh_device_id_get(&iis2dh_ctx_instance, &reg0);
  iis2dh_ctrl_reg1_t ctrl_reg1;
  iis2dh_read_reg(&iis2dh_ctx_instance, IIS2DH_CTRL_REG1, (uint8_t *)&ctrl_reg1, 1);

  ctrl_reg1.xen = 0;
  ctrl_reg1.yen = 0;
  ctrl_reg1.zen = 0;
  iis2dh_write_reg(&iis2dh_ctx_instance, IIS2DH_CTRL_REG1, (uint8_t *)&ctrl_reg1, 1);

  /* Output data rate selection - power down. */
  iis2dh_data_rate_set(&iis2dh_ctx_instance, IIS2DH_POWER_DOWN);
  /* ODisable Temperature Measurement */
  iis2dh_temperature_meas_set(&iis2dh_ctx_instance, IIS2DH_TEMP_DISABLE);
  /* Enable BDU */
  iis2dh_block_data_update_set(&iis2dh_ctx_instance, PROPERTY_ENABLE);

  /* Full scale selection. */
  if (IIS2DH_Init_Param.FS[0] < 3.0f)
  {
    iis2dh_full_scale_set(&iis2dh_ctx_instance, IIS2DH_2g);
  }
  else if (IIS2DH_Init_Param.FS[0] < 5.0f)
  {
    iis2dh_full_scale_set(&iis2dh_ctx_instance, IIS2DH_4g);
  }
  else if (IIS2DH_Init_Param.FS[0] < 9.0f)
  {
    iis2dh_full_scale_set(&iis2dh_ctx_instance, IIS2DH_8g);
  }
  else if (IIS2DH_Init_Param.FS[0] < 17.0f)
  {
    iis2dh_full_scale_set(&iis2dh_ctx_instance, IIS2DH_16g);
  }

  /* Power mode selection */
  iis2dh_fifo_set(& iis2dh_ctx_instance, 1);

  iis2dh_operating_mode_set(&iis2dh_ctx_instance, IIS2DH_HR_12bit);

  /* Big/Little Endian data selection configuration */
  iis2dh_data_format_set(&iis2dh_ctx_instance, IIS2DH_MSB_AT_LOW_ADD);
  iis2dh_read_reg(&iis2dh_ctx_instance, IIS2DH_CTRL_REG1, (uint8_t *)&ctrl_reg1, 1);
  ctrl_reg1.xen = 1;
  ctrl_reg1.yen = 1;
  ctrl_reg1.zen = 1;
  iis2dh_write_reg(&iis2dh_ctx_instance, IIS2DH_CTRL_REG1, (uint8_t *)&ctrl_reg1, 1);
  iis2dh_fifo_mode_set(&iis2dh_ctx_instance, IIS2DH_DYNAMIC_STREAM_MODE);

  /* taskDelay is set so that the number of samples in FIFO shouldn't be more than 16 */
  if (IIS2DH_Init_Param.ODR[0] < 2.0f)
  {
    iis2dh_data_rate_set(&iis2dh_ctx_instance, IIS2DH_ODR_1Hz);
    taskDelay = 1000;
  }
  else if (IIS2DH_Init_Param.ODR[0] < 11.0f)
  {
    iis2dh_data_rate_set(&iis2dh_ctx_instance, IIS2DH_ODR_10Hz);
    taskDelay = 1000;
  }
  else if (IIS2DH_Init_Param.ODR[0] < 26.0f)
  {
    iis2dh_data_rate_set(&iis2dh_ctx_instance, IIS2DH_ODR_25Hz);
    taskDelay = 640;
  }
  else if (IIS2DH_Init_Param.ODR[0] < 51.0f)
  {
    iis2dh_data_rate_set(&iis2dh_ctx_instance, IIS2DH_ODR_50Hz);
    taskDelay = 320;
  }
  else if (IIS2DH_Init_Param.ODR[0] < 101.0f)
  {
    iis2dh_data_rate_set(&iis2dh_ctx_instance, IIS2DH_ODR_100Hz);
    taskDelay = 160;
  }
  else if (IIS2DH_Init_Param.ODR[0] < 201.0f)
  {
    iis2dh_data_rate_set(&iis2dh_ctx_instance, IIS2DH_ODR_200Hz);
    taskDelay = 80;
  }
  else if (IIS2DH_Init_Param.ODR[0] < 401.0f)
  {
    iis2dh_data_rate_set(&iis2dh_ctx_instance, IIS2DH_ODR_400Hz);
    taskDelay = 40;
  }
  else if (IIS2DH_Init_Param.ODR[0] < 1345.0f)
  {
    iis2dh_data_rate_set(&iis2dh_ctx_instance, IIS2DH_ODR_5kHz376_LP_1kHz344_NM_HP);
    taskDelay = 12;
  }

  HAL_NVIC_EnableIRQ(IIS2DH_INT2_EXTI_IRQn);
}

static void IIS2DH_Int_Callback(void)
{
  osSemaphoreRelease(iis2dh_DreadySem_id);
}

void IIS2DH_Set_State(SM_Sensor_State_t state)
{
  IIS2DH_Sensor_State = state;
}

void IIS2DH_Start(void)
{
  IIS2DH_Set_State(SM_SENSOR_STATE_INITIALIZING);
  osThreadResume(IIS2DH_Thread_Id);
}

void IIS2DH_Stop(void)
{
  IIS2DH_Set_State(SM_SENSOR_STATE_SUSPENDING);
}

__weak void IIS2DH_Data_Ready(uint8_t subSensorId, uint8_t *buf, uint16_t size, double timeStamp)
{

}

/**
  * @brief Get Sensor ID
  * @param None
  * @retval Sensor ID
  */
uint8_t IIS2DH_Get_Id(void)
{
  return s_nIIS2DH_id;
}

/**
  * @brief IIS2DH Sensor Initialization
  * @param None
  * @retval None
  */
uint8_t IIS2DH_Create_Sensor(const SM_Init_Param_t *pxParams)
{
  COM_Sensor_t *pSensor;

  s_nIIS2DH_id = COM_AddSensor();

  if (s_nIIS2DH_id == -1)
  {
    return 1; /* error */
  }

  pSensor = COM_GetSensor(s_nIIS2DH_id);

  /* SENSOR DESCRIPTOR */
  strcpy(pSensor->sensorDescriptor.name, "IIS2DH");
  pSensor->sensorDescriptor.nSubSensors = 1;

  /* SUBSENSOR 0 DESCRIPTOR */
  pSensor->sensorDescriptor.subSensorDescriptor[0].id = 0;
  pSensor->sensorDescriptor.subSensorDescriptor[0].sensorType = COM_TYPE_ACC;
  pSensor->sensorDescriptor.subSensorDescriptor[0].dimensions = 3;
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].dimensionsLabel[0], "x");
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].dimensionsLabel[1], "y");
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].dimensionsLabel[2], "z");
  pSensor->sensorDescriptor.subSensorDescriptor[0].dataType = DATA_TYPE_INT16;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[0] = 1.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[1] = 10.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[2] = 25.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[3] = 50.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[4] = 100.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[5] = 200.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[6] = 400.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[7] = 1344.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[8] = COM_END_OF_LIST_FLOAT;
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
    pSensor->sensorStatus.subSensorStatus[0].ODR = 1344.0f;
  }
  if (pSensor->sensorStatus.subSensorStatus[0].FS == 16.0f)
  {
    pSensor->sensorStatus.subSensorStatus[0].sensitivity = 0.00075f;
  }
  else
  {
    pSensor->sensorStatus.subSensorStatus[0].sensitivity = 0.00003125f *  pSensor->sensorStatus.subSensorStatus[0].FS;
  }
  pSensor->sensorStatus.subSensorStatus[0].measuredODR = 0.0f;
  pSensor->sensorStatus.subSensorStatus[0].initialOffset = 0.0f;
  pSensor->sensorStatus.subSensorStatus[0].samplesPerTimestamp = 1000;
  pSensor->sensorStatus.subSensorStatus[0].usbDataPacketSize = 2400;
  pSensor->sensorStatus.subSensorStatus[0].sdWriteBufferSize = WRITE_BUFFER_SIZE_IIS2DH;
  pSensor->sensorStatus.subSensorStatus[0].comChannelNumber = -1;
  pSensor->sensorStatus.subSensorStatus[0].ucfLoaded = 0;

  IIS2DH_Init_Param.ODR[0] = pSensor->sensorStatus.subSensorStatus[0].ODR;
  IIS2DH_Init_Param.FS[0] = pSensor->sensorStatus.subSensorStatus[0].FS;
  IIS2DH_Init_Param.subSensorActive[0] = pSensor->sensorStatus.subSensorStatus[0].isActive;

  /**********/

  return 0;
}


