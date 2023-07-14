/**
  ******************************************************************************
  * @file    lsm6dsox_app.c
  * @brief   This file provides a set of functions to handle lsm6dsox
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
#include "lsm6dsox_app.h"
#include "main.h"
#include "cmsis_os.h"
#include "sensors_manager.h"
#include "com_manager.h"
#include "device_description.h"
#include "lsm6dsox_reg.h"
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define WRITE_BUFFER_SIZE_LSM6DSOX_A   (uint32_t)(16384)
#define WRITE_BUFFER_SIZE_LSM6DSOX_G   (uint32_t)(16384)
#define WRITE_BUFFER_SIZE_LSM6DSOX_MLC (uint32_t)(1024)

#define LSM6DSOX_SAMPLE_SIZE  (7)
#define LSM6DSOX_TAG_ACC      (0x02)

#define MLC_CONFIG              (0x00000001)
#define DHCX_INIT               (0x00000010)
#define DHCX_FIFO               (0x00000011)
#define DHCX_MLC                (0x00000100)
#define DHCX_SUSPEND            (0x00000101)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static int32_t s_nLSM6DSOX_id = -1;

static volatile double TimeStamp_lsm6dsox;
static uint8_t lsm6dsox_mem[LSM6DSOX_MAX_SAMPLES_PER_IT * 7];
static uint8_t lsm6dsox_mem_app[LSM6DSOX_MAX_SAMPLES_PER_IT / 2 * 6]; /*without Tag*/
uint16_t lsm6dsox_samples_per_it;

#if (HSD_USE_DUMMY_DATA == 1)
static int16_t dummyDataCounter_acc = 0;
static int16_t dummyDataCounter_gyro = 0;
#endif /* (HSD_USE_DUMMY_DATA == 1) */

static volatile double TimeStamp_mlc;
static uint8_t mlc_mem[8];
lsm6dsox_mlc_status_mainpage_t mlc_status;

volatile uint8_t LSM6DSOX_params_loading = 0;
volatile uint8_t UCF_loading = 0;

static volatile uint32_t UCFSize;
static volatile char *UCFData;

SM_Init_Param_t LSM6DSOX_Init_Param;
SM_Sensor_State_t LSM6DSOX_Sensor_State = SM_SENSOR_STATE_INITIALIZING;
volatile static uint32_t MLC_Data_Ready = 0;

/* Semaphore used to wait on component interrupt */
osSemaphoreId lsm6dsox_DreadySem_id;
osSemaphoreDef(lsm6dsox_DreadySem);

/* Semaphore used to wait on BUS data read complete, managed by lower layer */
osSemaphoreId lsm6dsoxReadSem_id;
osSemaphoreDef(lsm6dsoxReadSem);

osMessageQId lsm6dsoxThreadQueue_id;
osMessageQDef(lsm6dsoxThreadQueue, 100, int);

sensor_handle_t lsm6dsox_hdl_instance =
{
  0,
  0,
  LSM6DSOX_SPI_CS_GPIO_Port,
  LSM6DSOX_SPI_CS_Pin,
  &lsm6dsoxReadSem_id
};
stmdev_ctx_t lsm6dsox_ctx_instance =
{
  SM_SPI1_Write_Os,
  SM_SPI1_Read_Os,
  NULL,
  &lsm6dsox_hdl_instance
};

EXTI_HandleTypeDef mlc_exti;
EXTI_HandleTypeDef lsm6dsox_exti;
TIM_HandleTypeDef htim2;

/* Private function prototypes -----------------------------------------------*/
osThreadId LSM6DSOX_Thread_Id;
static void LSM6DSOX_Thread(void const *argument);
static void HSD_MLC_Int_Config(void);
static void LSM6DSOX_Program_MLC(uint32_t size, char *buffer);
static void LSM6DSOX_Init(void);
static void LSM6DSOX_Read_MLC(void);
static void LSM6DSOX_Read_Data(void);
static void LSM6DSOX_Read_Data_From_FIFO(void);
static void LSM6DSOX_Suspend(void);
#if (HSD_USE_DUMMY_DATA == 1)
static void LSM6DSOX_CreateDummyData(void);
#endif /* (HSD_USE_DUMMY_DATA == 1) */

static void LSM6DSOX_updateFromUCF(void);
static void LSM6DSOX_XL_ODR_From_UCF(void);
static void LSM6DSOX_XL_FS_From_UCF(void);
static void LSM6DSOX_GY_ODR_From_UCF(void);
static void LSM6DSOX_GY_FS_From_UCF(void);

static void LSM6DSOX_Sensor_Init(void);
static void LSM6DSOX_MLC_Int_Callback(void);

void LSM6DSOX_sendCMD(uint32_t cmd);
static uint8_t LSM6DSOX_updateSensorStatus(void);

static void MX_TIM2_Init(void);

/**
  * @brief LIS3DHH GPIO Initialization Function
  * @param None
  * @retval None
  */
void LSM6DSOX_Peripheral_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct =
  {
    0
  };

  /* GPIO Ports Clock Enable */
  LSM6DSOX_SPI_CS_GPIO_CLK_ENABLE();
  LSM6DSOX_INT2_GPIO_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LSM6DSOX_SPI_CS_GPIO_Port, LSM6DSOX_SPI_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LIS3DHH_SPI_CS_Pin */
  GPIO_InitStruct.Pin = LSM6DSOX_SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LSM6DSOX_SPI_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LIS3DHH_INT1_Pin */
  GPIO_InitStruct.Pin = LSM6DSOX_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LSM6DSOX_INT2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(LSM6DSOX_INT2_EXTI_IRQn, 5, 0);

  HAL_EXTI_GetHandle(&mlc_exti, LSM6DSOX_INT2_EXTI_LINE);
  HAL_EXTI_RegisterCallback(&mlc_exti, HAL_EXTI_COMMON_CB_ID, LSM6DSOX_MLC_Int_Callback);

  /*
   * HW conflict between PWR button and LSM6DSOX INT1:
   * link to TIM2_IC CHANNEL3 to use the timer IC capture callback for the sensor
   */
  MX_TIM2_Init();
}

void LSM6DSOX_OS_Init(void)
{
  /* Data read complete semaphore initialization */
  lsm6dsoxReadSem_id = osSemaphoreCreate(osSemaphore(lsm6dsoxReadSem), 1);
  osSemaphoreWait(lsm6dsoxReadSem_id, osWaitForever);

  /* Data ready interrupt semaphore initialization */
  lsm6dsox_DreadySem_id = osSemaphoreCreate(osSemaphore(lsm6dsox_DreadySem), 1);
  osSemaphoreWait(lsm6dsox_DreadySem_id, osWaitForever);

  lsm6dsoxThreadQueue_id = osMessageCreate(osMessageQ(lsm6dsoxThreadQueue), NULL);
  vQueueAddToRegistry(lsm6dsoxThreadQueue_id, "lsm6dsoxThreadQueue_id");

  /* Thread 1 definition */
  osThreadDef(ISM330_RD_USR_THREAD, LSM6DSOX_Thread, HSD_LSM6DSOX_THREAD_PRIO, 1, 8000 / 4);
  /* Start thread 1 */
  LSM6DSOX_Thread_Id = osThreadCreate(osThread(ISM330_RD_USR_THREAD), NULL);
}

static void LSM6DSOX_Thread(void const *argument)
{
  (void) argument;
  osEvent evt;

#if (configUSE_APPLICATION_TASK_TAG == 1 && defined(HSD_TASK_LSM6DSOX_DEBUG_PIN))
  vTaskSetApplicationTaskTag(NULL, (TaskHookFunction_t)HSD_TASK_LSM6DSOX_DEBUG_PIN);
#endif /* (configUSE_APPLICATION_TASK_TAG == 1 && defined(HSD_TASK_LSM6DSOX_DEBUG_PIN)) */

  lsm6dsox_reset_set(&lsm6dsox_ctx_instance, 1);

  for (;;)
  {
    evt = osMessageGet(lsm6dsoxThreadQueue_id, osWaitForever);

    if (evt.status == osEventMessage)
    {
      switch (evt.value.v)
      {
        case MLC_CONFIG:
        {
          LSM6DSOX_Program_MLC((uint32_t) UCFSize, (char *) UCFData);
          HSD_free((void *) UCFData);
          LSM6DSOX_Sensor_State = SM_SENSOR_STATE_INITIALIZING;
          break;
        }
        case DHCX_INIT:
        {
          LSM6DSOX_Init();
          LSM6DSOX_Sensor_State = SM_SENSOR_STATE_RUNNING;
          break;
        }
        case DHCX_FIFO:
        {
          if (LSM6DSOX_Sensor_State == SM_SENSOR_STATE_RUNNING)
          {
            LSM6DSOX_Read_Data();
          }
          break;
        }
        case DHCX_MLC:
        {
          if (LSM6DSOX_Sensor_State == SM_SENSOR_STATE_RUNNING)
          {
            LSM6DSOX_Read_MLC();
          }
          break;
        }
        case DHCX_SUSPEND:
        {
          LSM6DSOX_Suspend();
          LSM6DSOX_Sensor_State = SM_SENSOR_STATE_SUSPENDED;
          break;
        }
        default:
        {
          break;
        }
      }
    }
  }
}

static void LSM6DSOX_Init(void)
{
  COM_SubSensorStatus_t *pSubSensorStatus = COM_GetSubSensorStatus(LSM6DSOX_Get_Id(), 2);
  if (LSM6DSOX_Init_Param.subSensorActive[2] == 0 || pSubSensorStatus->ucfLoaded == 0)
  {
    lsm6dsox_reset_set(&lsm6dsox_ctx_instance, 1);
  }
  LSM6DSOX_Sensor_Init();
  if (LSM6DSOX_Init_Param.subSensorActive[2]) /* MLC isActive */
  {
    /* Route int pins for MLC */
    HSD_MLC_Int_Config();
  }
  lsm6dsox_fifo_mode_set(&lsm6dsox_ctx_instance, LSM6DSOX_STREAM_MODE);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
}

static void LSM6DSOX_Read_Data(void)
{
  uint8_t reg[2];

  /* Check FIFO_WTM_IA anf fifo level. We do not use PID in order to avoid reading one register twice */
  lsm6dsox_read_reg(&lsm6dsox_ctx_instance, LSM6DSOX_FIFO_STATUS1, reg, 2);
  uint16_t fifo_level = ((reg[1] & 0x03) << 8) + reg[0];

  if ((reg[1]) & 0x80 && (fifo_level >= lsm6dsox_samples_per_it))
  {
    LSM6DSOX_Read_Data_From_FIFO();
  }
}

static void LSM6DSOX_Read_Data_From_FIFO(void)
{
  uint16_t i = 0;
  /* Read sensor data from FIFO */
  lsm6dsox_read_reg(&lsm6dsox_ctx_instance, LSM6DSOX_FIFO_DATA_OUT_TAG, (uint8_t *) lsm6dsox_mem,
                    lsm6dsox_samples_per_it * 7);

#if (HSD_USE_DUMMY_DATA == 1)
  LSM6DSOX_CreateDummyData();
#endif /* (HSD_USE_DUMMY_DATA == 1) */

  if (LSM6DSOX_Init_Param.subSensorActive[0] && LSM6DSOX_Init_Param.subSensorActive[1]) /* Read both ACC and GYRO */
  {
    uint32_t ODR_Acc = (uint32_t) COM_GetSubSensorStatus(LSM6DSOX_Get_Id(), 0)->ODR;
    uint32_t ODR_Gyro = (uint32_t) COM_GetSubSensorStatus(LSM6DSOX_Get_Id(), 1)->ODR;
    uint32_t gyroSamplesCount = 0;
    uint32_t accSamplesCount = 0;

    int16_t *p16src = (int16_t *) lsm6dsox_mem;
    int16_t *pAcc;
    int16_t *pGyro;

    if (ODR_Acc > ODR_Gyro) /* Acc is faster than Gyro */
    {
      pAcc = (int16_t *) lsm6dsox_mem;
      pGyro = (int16_t *) lsm6dsox_mem_app;
    }
    else
    {
      pAcc = (int16_t *) lsm6dsox_mem_app;
      pGyro = (int16_t *) lsm6dsox_mem;
    }

    uint8_t *pTag = (uint8_t *) p16src;

    for (i = 0; i < lsm6dsox_samples_per_it; i++)
    {
      if (((*pTag) >> 3) == LSM6DSOX_TAG_ACC)
      {
        p16src = (int16_t *)(pTag + 1);
        *pAcc++ = *p16src++;
        *pAcc++ = *p16src++;
        *pAcc++ = *p16src++;
        accSamplesCount++;
      }
      else
      {
        p16src = (int16_t *)(pTag + 1);
        *pGyro++ = *p16src++;
        *pGyro++ = *p16src++;
        *pGyro++ = *p16src++;
        gyroSamplesCount++;
      }
      pTag += 7;
    }
    if (ODR_Acc > ODR_Gyro) /* Acc is faster than Gyro */
    {
      LSM6DSOX_Data_Ready(0, (uint8_t *) lsm6dsox_mem, accSamplesCount * 6, TimeStamp_lsm6dsox);
      LSM6DSOX_Data_Ready(1, (uint8_t *) lsm6dsox_mem_app, gyroSamplesCount * 6, TimeStamp_lsm6dsox);
    }
    else
    {
      LSM6DSOX_Data_Ready(0, (uint8_t *) lsm6dsox_mem_app, accSamplesCount * 6, TimeStamp_lsm6dsox);
      LSM6DSOX_Data_Ready(1, (uint8_t *) lsm6dsox_mem, gyroSamplesCount * 6, TimeStamp_lsm6dsox);
    }
  }
  else /* 1 subsensor active only --> simply drop TAGS */
  {
    int16_t *p16src = (int16_t *) lsm6dsox_mem;
    int16_t *p16dest = (int16_t *) lsm6dsox_mem;
    for (i = 0; i < lsm6dsox_samples_per_it; i++)
    {
      p16src = (int16_t *) & ((uint8_t *)(p16src))[1];
      *p16dest++ = *p16src++;
      *p16dest++ = *p16src++;
      *p16dest++ = *p16src++;
    }
    if (LSM6DSOX_Init_Param.subSensorActive[0]) /* Acc only */
    {
      LSM6DSOX_Data_Ready(0, (uint8_t *) lsm6dsox_mem, lsm6dsox_samples_per_it * 6, TimeStamp_lsm6dsox);
    }
    else if (LSM6DSOX_Init_Param.subSensorActive[1]) /* Gyro only */
    {
      LSM6DSOX_Data_Ready(1, (uint8_t *) lsm6dsox_mem, lsm6dsox_samples_per_it * 6, TimeStamp_lsm6dsox);
    }
  }
}

#if (HSD_USE_DUMMY_DATA == 1)
static void LSM6DSOX_CreateDummyData(void)
{
  uint16_t i = 0;
  int16_t *p16 = (int16_t *)lsm6dsox_mem;

  for (i = 0; i < lsm6dsox_samples_per_it; i++)
  {
    p16 = (int16_t *)(&lsm6dsox_mem[i * 7] + 1);
    if ((lsm6dsox_mem[i * 7] >> 3) == LSM6DSOX_TAG_ACC)
    {
      *p16++ = dummyDataCounter_acc++;
      *p16++ = dummyDataCounter_acc++;
      *p16++ = dummyDataCounter_acc++;
    }
    else
    {
      *p16++ = dummyDataCounter_gyro++;
      *p16++ = dummyDataCounter_gyro++;
      *p16++ = dummyDataCounter_gyro++;
    }
  }
}
#endif /* (HSD_USE_DUMMY_DATA == 1) */

static void LSM6DSOX_Read_MLC(void)
{
  if (LSM6DSOX_Init_Param.subSensorActive[2] && MLC_Data_Ready == 1)
  {
    uint32_t ii;
    for (ii = 0; ii < 8; ii++)
    {
      lsm6dsox_mem_bank_set(&lsm6dsox_ctx_instance, LSM6DSOX_EMBEDDED_FUNC_BANK);
      lsm6dsox_read_reg(&lsm6dsox_ctx_instance, LSM6DSOX_MLC0_SRC + ii, (uint8_t *) &mlc_mem[ii], 1);
      lsm6dsox_mem_bank_set(&lsm6dsox_ctx_instance, LSM6DSOX_USER_BANK);
    }
    lsm6dsox_mlc_status_get(&lsm6dsox_ctx_instance, &mlc_status);
    LSM6DSOX_Data_Ready(2, (uint8_t *) mlc_mem, 8, TimeStamp_mlc);
    MLC_Data_Ready = 0;
  }
}

static void LSM6DSOX_Suspend(void)
{
#if (HSD_USE_DUMMY_DATA == 1)
  dummyDataCounter_acc = 0;
  dummyDataCounter_gyro = 0;
#endif /* (HSD_USE_DUMMY_DATA == 1) */
  lsm6dsox_fifo_gy_batch_set(&lsm6dsox_ctx_instance, LSM6DSOX_GY_NOT_BATCHED);
  lsm6dsox_fifo_xl_batch_set(&lsm6dsox_ctx_instance, LSM6DSOX_XL_NOT_BATCHED);
}

void LSM6DSOX_sendCMD(uint32_t cmd)
{
  if (osMessagePut(lsm6dsoxThreadQueue_id, cmd, 0) != osOK)
  {
    while (1);
  }
}

static uint8_t LSM6DSOX_updateSensorStatus(void)
{
  COM_SubSensorStatus_t *pSubSensorStatus = COM_GetSubSensorStatus(LSM6DSOX_Get_Id(), 2);
  uint8_t ret = 0;

  if (pSubSensorStatus->ucfLoaded == 1)
  {
    if ((LSM6DSOX_params_loading != 0) && (pSubSensorStatus->isActive == LSM6DSOX_Init_Param.subSensorActive[2]))
    {
      pSubSensorStatus->ucfLoaded = 0;
      pSubSensorStatus->isActive = 0;
      ret += 1;
    }
    else
    {
      if (UCF_loading == 1)
      {
        LSM6DSOX_updateFromUCF();
        ret += 1;
      }
    }
  }
  COM_Sensor_t *pSensor;
  pSensor = COM_GetSensor(LSM6DSOX_Get_Id());

  LSM6DSOX_Init_Param.ODR[0] = pSensor->sensorStatus.subSensorStatus[0].ODR;
  LSM6DSOX_Init_Param.ODR[1] = pSensor->sensorStatus.subSensorStatus[1].ODR;
  LSM6DSOX_Init_Param.ODR[2] = pSensor->sensorStatus.subSensorStatus[2].ODR;
  LSM6DSOX_Init_Param.FS[0] = pSensor->sensorStatus.subSensorStatus[0].FS;
  LSM6DSOX_Init_Param.FS[1] = pSensor->sensorStatus.subSensorStatus[1].FS;
  LSM6DSOX_Init_Param.subSensorActive[0] = pSensor->sensorStatus.subSensorStatus[0].isActive;
  LSM6DSOX_Init_Param.subSensorActive[1] = pSensor->sensorStatus.subSensorStatus[1].isActive;
  LSM6DSOX_Init_Param.subSensorActive[2] = pSensor->sensorStatus.subSensorStatus[2].isActive;
  update_samplesPerTimestamp(pSensor);

  LSM6DSOX_params_loading = 0;
  return ret;
}

static void LSM6DSOX_updateFromUCF(void)
{
  LSM6DSOX_XL_ODR_From_UCF();
  LSM6DSOX_XL_FS_From_UCF();
  LSM6DSOX_GY_ODR_From_UCF();
  LSM6DSOX_GY_FS_From_UCF();

  COM_SubSensorStatus_t *pSubSensorStatus = COM_GetSubSensorStatus(LSM6DSOX_Get_Id(), 0);
  pSubSensorStatus->ODR = LSM6DSOX_Init_Param.ODR[0];
  pSubSensorStatus->FS = LSM6DSOX_Init_Param.FS[0];
  pSubSensorStatus->isActive = LSM6DSOX_Init_Param.subSensorActive[0];

  pSubSensorStatus = COM_GetSubSensorStatus(LSM6DSOX_Get_Id(), 1);
  pSubSensorStatus->ODR = LSM6DSOX_Init_Param.ODR[1];
  pSubSensorStatus->FS = LSM6DSOX_Init_Param.FS[1];
  pSubSensorStatus->isActive = LSM6DSOX_Init_Param.subSensorActive[1];

  LSM6DSOX_params_loading = 0;
  UCF_loading = 0;
}

static void LSM6DSOX_XL_ODR_From_UCF(void)
{
  lsm6dsox_odr_xl_t lsm6dsox_odr_xl = LSM6DSOX_XL_ODR_OFF;
  COM_SubSensorStatus_t *pSubSensorStatus = COM_GetSubSensorStatus(LSM6DSOX_Get_Id(), 0);

  LSM6DSOX_Init_Param.subSensorActive[0] = 1;
  pSubSensorStatus->isActive = 1;

  lsm6dsox_xl_data_rate_get(&lsm6dsox_ctx_instance, &lsm6dsox_odr_xl);
  float Odr = 12.5f;

  switch (lsm6dsox_odr_xl)
  {
    case LSM6DSOX_XL_ODR_OFF:
    {
      LSM6DSOX_Init_Param.subSensorActive[0] = 0;
      COM_SubSensorStatus_t *pSubSensorStatus = COM_GetSubSensorStatus(LSM6DSOX_Get_Id(), 0);
      pSubSensorStatus->isActive = 0;
    }
    break;

    case LSM6DSOX_XL_ODR_12Hz5:
      Odr = 12.5f;
      break;

    case LSM6DSOX_XL_ODR_26Hz:
      Odr = 26.0f;
      break;

    case LSM6DSOX_XL_ODR_52Hz:
      Odr = 52.0f;
      break;

    case LSM6DSOX_XL_ODR_104Hz:
      Odr = 104.0f;
      break;

    case LSM6DSOX_XL_ODR_208Hz:
      Odr = 208.0f;
      break;

    case LSM6DSOX_XL_ODR_417Hz:
      Odr = 416.0f;
      break;

    case LSM6DSOX_XL_ODR_833Hz:
      Odr = 833.0f;
      break;

    case LSM6DSOX_XL_ODR_1667Hz:
      Odr = 1666.0f;
      break;

    case LSM6DSOX_XL_ODR_3333Hz:
      Odr = 3332.0f;
      break;

    case LSM6DSOX_XL_ODR_6667Hz:
      Odr = 6667.0f;
      break;

    default:
      break;
  }
  LSM6DSOX_Init_Param.ODR[0] = Odr;
}

static void LSM6DSOX_XL_FS_From_UCF(void)
{
  lsm6dsox_fs_xl_t fs_xl;
  lsm6dsox_xl_full_scale_get(&lsm6dsox_ctx_instance, &fs_xl);
  float FullScale = 0.0;

  switch (fs_xl)
  {
    case LSM6DSOX_2g:
      FullScale = 2.0;
      break;

    case LSM6DSOX_4g:
      FullScale = 4.0;
      break;

    case LSM6DSOX_8g:
      FullScale = 8.0;
      break;

    case LSM6DSOX_16g:
      FullScale = 16.0;
      break;

    default:
      break;
  }
  LSM6DSOX_Init_Param.FS[0] = FullScale;
}

static void LSM6DSOX_GY_ODR_From_UCF(void)
{
  lsm6dsox_odr_g_t lsm6dsox_odr_g = LSM6DSOX_GY_ODR_OFF;
  COM_SubSensorStatus_t *pSubSensorStatus = COM_GetSubSensorStatus(LSM6DSOX_Get_Id(), 1);

  LSM6DSOX_Init_Param.subSensorActive[1] = 1;
  pSubSensorStatus->isActive = 1;

  lsm6dsox_gy_data_rate_get(&lsm6dsox_ctx_instance, &lsm6dsox_odr_g);
  float Odr = 12.5f;

  switch (lsm6dsox_odr_g)
  {
    case LSM6DSOX_GY_ODR_OFF:
    {
      LSM6DSOX_Init_Param.subSensorActive[1] = 0;
      COM_SubSensorStatus_t *pSubSensorStatus = COM_GetSubSensorStatus(LSM6DSOX_Get_Id(), 1);
      pSubSensorStatus->isActive = 0;
    }
    break;

    case LSM6DSOX_GY_ODR_12Hz5:
      Odr = 12.5f;
      break;

    case LSM6DSOX_GY_ODR_26Hz:
      Odr = 26.0f;
      break;

    case LSM6DSOX_GY_ODR_52Hz:
      Odr = 52.0f;
      break;

    case LSM6DSOX_GY_ODR_104Hz:
      Odr = 104.0f;
      break;

    case LSM6DSOX_GY_ODR_208Hz:
      Odr = 208.0f;
      break;

    case LSM6DSOX_GY_ODR_417Hz:
      Odr = 416.0f;
      break;

    case LSM6DSOX_GY_ODR_833Hz:
      Odr = 833.0f;
      break;

    case LSM6DSOX_GY_ODR_1667Hz:
      Odr = 1666.0f;
      break;

    case LSM6DSOX_GY_ODR_3333Hz:
      Odr = 3332.0f;
      break;

    case LSM6DSOX_GY_ODR_6667Hz:
      Odr = 6667.0f;
      break;

    default:
      break;
  }
  LSM6DSOX_Init_Param.ODR[1] = Odr;
}

static void LSM6DSOX_GY_FS_From_UCF(void)
{
  lsm6dsox_fs_g_t fs_g;
  lsm6dsox_gy_full_scale_get(&lsm6dsox_ctx_instance, &fs_g);
  float FullScale = 0.0;

  switch (fs_g)
  {
    case LSM6DSOX_125dps:
      FullScale = 125;
      break;

    case LSM6DSOX_250dps:
      FullScale = 250;
      break;

    case LSM6DSOX_500dps:
      FullScale = 500;
      break;

    case LSM6DSOX_1000dps:
      FullScale = 1000;
      break;

    case LSM6DSOX_2000dps:
      FullScale = 2000;
      break;

    default:
      break;
  }

  LSM6DSOX_Init_Param.FS[1] = FullScale;
}

uint8_t LSM6DSOX_updateConfig(void)
{
  COM_SubSensorStatus_t *pSubSensorStatus = COM_GetSubSensorStatus(LSM6DSOX_Get_Id(), 2);
  if (pSubSensorStatus->ucfLoaded == 0
      && pSubSensorStatus->isActive == 1) /* If you enter here, MLC isActive is true but no UCF are available */
  {
    /* So, MLC can't be active --> subSensorStatus is forced not Active */
    pSubSensorStatus->isActive = 0;
  }

  return LSM6DSOX_updateSensorStatus();
}

static void LSM6DSOX_Sensor_Init(void)
{
  uint8_t reg0;
  uint16_t lsm6dsox_wtm_level = 0;
  uint16_t lsm6dsox_wtm_level_acc;
  uint16_t lsm6dsox_wtm_level_gyro;
  lsm6dsox_odr_xl_t lsm6dsox_odr_xl = LSM6DSOX_XL_ODR_OFF;
  lsm6dsox_bdr_xl_t lsm6dsox_bdr_xl = LSM6DSOX_XL_NOT_BATCHED;
  lsm6dsox_odr_g_t lsm6dsox_odr_g = LSM6DSOX_GY_ODR_OFF;
  lsm6dsox_bdr_gy_t lsm6dsox_bdr_gy = LSM6DSOX_GY_NOT_BATCHED;

  lsm6dsox_i2c_interface_set(&lsm6dsox_ctx_instance, LSM6DSOX_I2C_DISABLE);
  lsm6dsox_device_id_get(&lsm6dsox_ctx_instance, (uint8_t *) &reg0);

  /* AXL FS */
  if (LSM6DSOX_Init_Param.FS[0] < 3.0f)
  {
    lsm6dsox_xl_full_scale_set(&lsm6dsox_ctx_instance, LSM6DSOX_2g);
  }
  else if (LSM6DSOX_Init_Param.FS[0] < 5.0f)
  {
    lsm6dsox_xl_full_scale_set(&lsm6dsox_ctx_instance, LSM6DSOX_4g);
  }
  else if (LSM6DSOX_Init_Param.FS[0] < 9.0f)
  {
    lsm6dsox_xl_full_scale_set(&lsm6dsox_ctx_instance, LSM6DSOX_8g);
  }
  else if (LSM6DSOX_Init_Param.FS[0] < 17.0f)
  {
    lsm6dsox_xl_full_scale_set(&lsm6dsox_ctx_instance, LSM6DSOX_16g);
  }

  /* GYRO FS */
  if (LSM6DSOX_Init_Param.FS[1] < 126.0f)
  {
    lsm6dsox_gy_full_scale_set(&lsm6dsox_ctx_instance, LSM6DSOX_125dps);
  }
  else if (LSM6DSOX_Init_Param.FS[1] < 251.0f)
  {
    lsm6dsox_gy_full_scale_set(&lsm6dsox_ctx_instance, LSM6DSOX_250dps);
  }
  else if (LSM6DSOX_Init_Param.FS[1] < 501.0f)
  {
    lsm6dsox_gy_full_scale_set(&lsm6dsox_ctx_instance, LSM6DSOX_500dps);
  }
  else if (LSM6DSOX_Init_Param.FS[1] < 1001.0f)
  {
    lsm6dsox_gy_full_scale_set(&lsm6dsox_ctx_instance, LSM6DSOX_1000dps);
  }
  else if (LSM6DSOX_Init_Param.FS[1] < 2001.0f)
  {
    lsm6dsox_gy_full_scale_set(&lsm6dsox_ctx_instance, LSM6DSOX_2000dps);
  }

  if (LSM6DSOX_Init_Param.ODR[0] < 13.0f)
  {
    lsm6dsox_odr_xl = LSM6DSOX_XL_ODR_12Hz5;
    lsm6dsox_bdr_xl = LSM6DSOX_XL_BATCHED_AT_12Hz5;
  }
  else if (LSM6DSOX_Init_Param.ODR[0] < 27.0f)
  {
    lsm6dsox_odr_xl = LSM6DSOX_XL_ODR_26Hz;
    lsm6dsox_bdr_xl = LSM6DSOX_XL_BATCHED_AT_26Hz;
  }
  else if (LSM6DSOX_Init_Param.ODR[0] < 53.0f)
  {
    lsm6dsox_odr_xl = LSM6DSOX_XL_ODR_52Hz;
    lsm6dsox_bdr_xl = LSM6DSOX_XL_BATCHED_AT_52Hz;
  }
  else if (LSM6DSOX_Init_Param.ODR[0] < 105.0f)
  {
    lsm6dsox_odr_xl = LSM6DSOX_XL_ODR_104Hz;
    lsm6dsox_bdr_xl = LSM6DSOX_XL_BATCHED_AT_104Hz;
  }
  else if (LSM6DSOX_Init_Param.ODR[0] < 209.0f)
  {
    lsm6dsox_odr_xl = LSM6DSOX_XL_ODR_208Hz;
    lsm6dsox_bdr_xl = LSM6DSOX_XL_BATCHED_AT_208Hz;
  }
  else if (LSM6DSOX_Init_Param.ODR[0] < 417.0f)
  {
    lsm6dsox_odr_xl = LSM6DSOX_XL_ODR_417Hz;
    lsm6dsox_bdr_xl = LSM6DSOX_XL_BATCHED_AT_417Hz;
  }
  else if (LSM6DSOX_Init_Param.ODR[0] < 834.0f)
  {
    lsm6dsox_odr_xl = LSM6DSOX_XL_ODR_833Hz;
    lsm6dsox_bdr_xl = LSM6DSOX_XL_BATCHED_AT_833Hz;
  }
  else if (LSM6DSOX_Init_Param.ODR[0] < 1667.0f)
  {
    lsm6dsox_odr_xl = LSM6DSOX_XL_ODR_1667Hz;
    lsm6dsox_bdr_xl = LSM6DSOX_XL_BATCHED_AT_1667Hz;
  }
  else if (LSM6DSOX_Init_Param.ODR[0] < 3333.0f)
  {
    lsm6dsox_odr_xl = LSM6DSOX_XL_ODR_3333Hz;
    lsm6dsox_bdr_xl = LSM6DSOX_XL_BATCHED_AT_3333Hz;
  }
  else if (LSM6DSOX_Init_Param.ODR[0] < 6668.0f)
  {
    lsm6dsox_odr_xl = LSM6DSOX_XL_ODR_6667Hz;
    lsm6dsox_bdr_xl = LSM6DSOX_XL_BATCHED_AT_6667Hz;
  }

  if (LSM6DSOX_Init_Param.ODR[1] < 13.0f)
  {
    lsm6dsox_odr_g = LSM6DSOX_GY_ODR_12Hz5;
    lsm6dsox_bdr_gy = LSM6DSOX_GY_BATCHED_AT_12Hz5;
  }
  else if (LSM6DSOX_Init_Param.ODR[1] < 27.0f)
  {
    lsm6dsox_odr_g = LSM6DSOX_GY_ODR_26Hz;
    lsm6dsox_bdr_gy = LSM6DSOX_GY_BATCHED_AT_26Hz;
  }
  else if (LSM6DSOX_Init_Param.ODR[1] < 53.0f)
  {
    lsm6dsox_odr_g = LSM6DSOX_GY_ODR_52Hz;
    lsm6dsox_bdr_gy = LSM6DSOX_GY_BATCHED_AT_52Hz;
  }
  else if (LSM6DSOX_Init_Param.ODR[1] < 105.0f)
  {
    lsm6dsox_odr_g = LSM6DSOX_GY_ODR_104Hz;
    lsm6dsox_bdr_gy = LSM6DSOX_GY_BATCHED_AT_104Hz;
  }
  else if (LSM6DSOX_Init_Param.ODR[1] < 209.0f)
  {
    lsm6dsox_odr_g = LSM6DSOX_GY_ODR_208Hz;
    lsm6dsox_bdr_gy = LSM6DSOX_GY_BATCHED_AT_208Hz;
  }
  else if (LSM6DSOX_Init_Param.ODR[1] < 417.0f)
  {
    lsm6dsox_odr_g = LSM6DSOX_GY_ODR_417Hz;
    lsm6dsox_bdr_gy = LSM6DSOX_GY_BATCHED_AT_417Hz;
  }
  else if (LSM6DSOX_Init_Param.ODR[1] < 834.0f)
  {
    lsm6dsox_odr_g = LSM6DSOX_GY_ODR_833Hz;
    lsm6dsox_bdr_gy = LSM6DSOX_GY_BATCHED_AT_833Hz;
  }
  else if (LSM6DSOX_Init_Param.ODR[1] < 1667.0f)
  {
    lsm6dsox_odr_g = LSM6DSOX_GY_ODR_1667Hz;
    lsm6dsox_bdr_gy = LSM6DSOX_GY_BATCHED_AT_1667Hz;
  }
  else if (LSM6DSOX_Init_Param.ODR[1] < 3333.0f)
  {
    lsm6dsox_odr_g = LSM6DSOX_GY_ODR_3333Hz;
    lsm6dsox_bdr_gy = LSM6DSOX_GY_BATCHED_AT_3333Hz;
  }
  else if (LSM6DSOX_Init_Param.ODR[1] < 6668.0f)
  {
    lsm6dsox_odr_g = LSM6DSOX_GY_ODR_6667Hz;
    lsm6dsox_bdr_gy = LSM6DSOX_GY_BATCHED_AT_6667Hz;
  }

  if (LSM6DSOX_Init_Param.subSensorActive[0])
  {
    lsm6dsox_xl_data_rate_set(&lsm6dsox_ctx_instance, lsm6dsox_odr_xl);
    lsm6dsox_fifo_xl_batch_set(&lsm6dsox_ctx_instance, lsm6dsox_bdr_xl);
  }
  else
  {
    lsm6dsox_xl_data_rate_set(&lsm6dsox_ctx_instance, LSM6DSOX_XL_ODR_OFF);
    lsm6dsox_fifo_xl_batch_set(&lsm6dsox_ctx_instance, LSM6DSOX_XL_NOT_BATCHED);
  }

  if (LSM6DSOX_Init_Param.subSensorActive[1])
  {
    lsm6dsox_gy_data_rate_set(&lsm6dsox_ctx_instance, lsm6dsox_odr_g);
    lsm6dsox_fifo_gy_batch_set(&lsm6dsox_ctx_instance, lsm6dsox_bdr_gy);
  }
  else
  {
    lsm6dsox_gy_data_rate_set(&lsm6dsox_ctx_instance, LSM6DSOX_GY_ODR_OFF);
    lsm6dsox_fifo_gy_batch_set(&lsm6dsox_ctx_instance, LSM6DSOX_GY_NOT_BATCHED);
  }

  /* Calculation of watermark and samples per int*/
  lsm6dsox_wtm_level_acc = ((uint16_t) LSM6DSOX_Init_Param.ODR[0] * (uint16_t) LSM6DSOX_MAX_DRDY_PERIOD);
  lsm6dsox_wtm_level_gyro = ((uint16_t) LSM6DSOX_Init_Param.ODR[1] * (uint16_t) LSM6DSOX_MAX_DRDY_PERIOD);

  if (LSM6DSOX_Init_Param.subSensorActive[0] && LSM6DSOX_Init_Param.subSensorActive[1]) /* Both subSensor is active */
  {
    if (lsm6dsox_wtm_level_acc > lsm6dsox_wtm_level_gyro)
    {
      lsm6dsox_wtm_level = lsm6dsox_wtm_level_acc;
    }
    else
    {
      lsm6dsox_wtm_level = lsm6dsox_wtm_level_gyro;
    }
  }
  else /* Only one subSensor is active */
  {
    if (LSM6DSOX_Init_Param.subSensorActive[0])
    {
      lsm6dsox_wtm_level = lsm6dsox_wtm_level_acc;
    }
    else
    {
      lsm6dsox_wtm_level = lsm6dsox_wtm_level_gyro;
    }
  }

  if (lsm6dsox_wtm_level > LSM6DSOX_MAX_WTM_LEVEL)
  {
    lsm6dsox_wtm_level = LSM6DSOX_MAX_WTM_LEVEL;
  }
  else if (lsm6dsox_wtm_level < LSM6DSOX_MIN_WTM_LEVEL)
  {
    lsm6dsox_wtm_level = LSM6DSOX_MIN_WTM_LEVEL;
  }
  lsm6dsox_samples_per_it = lsm6dsox_wtm_level;

  /* Setup int for FIFO */
  lsm6dsox_fifo_watermark_set(&lsm6dsox_ctx_instance, lsm6dsox_wtm_level);

  lsm6dsox_pin_int1_route_t int1_route =
  {
    0
  };
  int1_route.fifo_th = 1;
  lsm6dsox_pin_int1_route_set(&lsm6dsox_ctx_instance, int1_route);
}

static void HSD_MLC_Int_Config(void)
{
  lsm6dsox_pin_int1_route_t pin_int1_route;
  lsm6dsox_pin_int2_route_t pin_int2_route;

  lsm6dsox_pin_int1_route_get(&lsm6dsox_ctx_instance, &pin_int1_route);
  lsm6dsox_pin_int2_route_get(&lsm6dsox_ctx_instance, NULL, &pin_int2_route);

  if (pin_int1_route.mlc1 == 1)
  {
    pin_int1_route.mlc1 = 0;
    lsm6dsox_pin_int1_route_set(&lsm6dsox_ctx_instance, pin_int1_route);
  }

  if (pin_int2_route.mlc1 == 0)
  {
    pin_int2_route.mlc1 = 1;
    lsm6dsox_pin_int2_route_set(&lsm6dsox_ctx_instance, NULL, pin_int2_route);
    HAL_NVIC_EnableIRQ(LSM6DSOX_INT2_EXTI_IRQn);
  }
}

static void LSM6DSOX_Program_MLC(uint32_t size, char *buffer)
{
  char ucf_reg[3];
  char ucf_data[3];
  long reg;
  long data;
  uint32_t i;

  lsm6dsox_reset_set(&lsm6dsox_ctx_instance, 1);

  lsm6dsox_mem_bank_set(&lsm6dsox_ctx_instance, LSM6DSOX_EMBEDDED_FUNC_BANK);

  for (i = 0; i < size / 4; i++)
  {
    ucf_reg[0] = buffer[4 * i];
    ucf_reg[1] = buffer[4 * i + 1];
    ucf_reg[2] = '\0';
    ucf_data[0] = buffer[4 * i + 2];
    ucf_data[1] = buffer[4 * i + 3];
    ucf_data[2] = '\0';

    reg = strtol(ucf_reg, NULL, 16);
    data = strtol(ucf_data, NULL, 16);

    lsm6dsox_write_reg(&lsm6dsox_ctx_instance, (uint8_t) reg, (uint8_t *) &data, 1);
  }

  lsm6dsox_emb_sens_t val =
  {
    0
  };
  val.mlc = 1;
  lsm6dsox_embedded_sens_set(&lsm6dsox_ctx_instance, &val);

  lsm6dsox_mem_bank_set(&lsm6dsox_ctx_instance, LSM6DSOX_USER_BANK);

  COM_SubSensorStatus_t *pSubSensorStatus = COM_GetSubSensorStatus(LSM6DSOX_Get_Id(), 2);
  pSubSensorStatus->ucfLoaded = 1;
  pSubSensorStatus->isActive = 1;
  UCF_loading = 1;
  LSM6DSOX_updateConfig();
}

/**
  * @brief LIS3DHH Get a buffer UCF file content
  * @param buffer_in: input buffer with compressed UCF
  * @param size_in: in_buffer size
  * @param buffer_out: output buffer of at least 9*(in_size/4) bytes
  * @param size_out: out_buffer size
  * @retval 0 --> OK, 1 --> Error
  */
int32_t LSM6DSOX_GetUCF_FromBuffer(char *buffer_in, uint32_t size_in, char *buffer_out, uint32_t size_out)
{
  uint8_t *p;
  uint32_t i;

  if (size_out < 9 * (size_in / 4))
  {
    return -1;
  }

  p = (uint8_t *) buffer_out;

  for (i = 0; i < size_in / 4; i++)
  {
    *p++ = 'A';
    *p++ = 'c';
    *p++ = ' ';
    *p++ = buffer_in[4 * i];
    *p++ = buffer_in[4 * i + 1];
    *p++ = ' ';
    *p++ = buffer_in[4 * i + 2];
    *p++ = buffer_in[4 * i + 3];
    *p++ = '\n';
  }
  return 0;
}

static void LSM6DSOX_MLC_Int_Callback(void)
{
  TimeStamp_mlc = SM_GetTimeStamp_fromISR();
  MLC_Data_Ready = 1;
  LSM6DSOX_sendCMD(DHCX_MLC);
}

/*
 * HW conflict between PWR button and LSM6DSOX INT1:
 * link to TIM2_IC CHANNEL3 to use the timer IC capture callback for the sensor
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);
  TimeStamp_lsm6dsox = SM_GetTimeStamp_fromISR();
  LSM6DSOX_sendCMD(DHCX_FIFO);
}

void LSM6DSOX_Set_State(SM_Sensor_State_t state)
{
  LSM6DSOX_Sensor_State = state;
}

void LSM6DSOX_Start(void)
{
  LSM6DSOX_Set_State(SM_SENSOR_STATE_INITIALIZING);
  LSM6DSOX_sendCMD(DHCX_INIT);
}

void LSM6DSOX_SetUCF(uint32_t mlcConfigSize, char *mlcConfigData)
{
  UCFSize = mlcConfigSize;
  UCFData = mlcConfigData;
  LSM6DSOX_Set_State(SM_SENSOR_STATE_MLC_CONFIG);
  LSM6DSOX_sendCMD(MLC_CONFIG);
}

void LSM6DSOX_Stop(void)
{
  LSM6DSOX_Set_State(SM_SENSOR_STATE_SUSPENDING);
  LSM6DSOX_sendCMD(DHCX_SUSPEND);
}

__weak void LSM6DSOX_Data_Ready(uint8_t subSensorId, uint8_t *buf, uint16_t size, double timeStamp)
{

}

/**
  * @brief Get Sensor ID
  * @param None
  * @retval Sensor ID
  */
uint8_t LSM6DSOX_Get_Id(void)
{
  return s_nLSM6DSOX_id;
}

/**
  * @brief LSM6DSOX Sensor Initialization
  * @param None
  * @retval None
  */
uint8_t LSM6DSOX_Create_Sensor(const SM_Init_Param_t *pxParams)
{
  COM_Sensor_t *pSensor;

  s_nLSM6DSOX_id = COM_AddSensor();

  if (s_nLSM6DSOX_id == -1)
  {
    return 1; /* error */
  }

  pSensor = COM_GetSensor(s_nLSM6DSOX_id);

  /* SENSOR DESCRIPTOR */
  strcpy(pSensor->sensorDescriptor.name, "LSM6DSOX");
  pSensor->sensorDescriptor.nSubSensors = 3;

  /* SUBSENSOR 0 DESCRIPTOR */
  pSensor->sensorDescriptor.subSensorDescriptor[0].id = 0;
  pSensor->sensorDescriptor.subSensorDescriptor[0].sensorType = COM_TYPE_ACC;
  pSensor->sensorDescriptor.subSensorDescriptor[0].dimensions = 3;
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].dimensionsLabel[0], "x");
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].dimensionsLabel[1], "y");
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].dimensionsLabel[2], "z");
  pSensor->sensorDescriptor.subSensorDescriptor[0].dataType = DATA_TYPE_INT16;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[0] = 12.5f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[1] = 26.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[2] = 52.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[3] = 104.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[4] = 208.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[5] = 416.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[6] = 833.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[7] = 1666.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[8] = 3332.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[9] = 6667.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[10] = COM_END_OF_LIST_FLOAT;
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
    pSensor->sensorStatus.subSensorStatus[0].ODR = 6667.0f;
  }
  pSensor->sensorStatus.subSensorStatus[0].sensitivity = 0.0000305f * pSensor->sensorStatus.subSensorStatus[0].FS;
  pSensor->sensorStatus.subSensorStatus[0].measuredODR = 0.0f;
  pSensor->sensorStatus.subSensorStatus[0].initialOffset = 0.0f;
  pSensor->sensorStatus.subSensorStatus[0].samplesPerTimestamp = 1000;
  pSensor->sensorStatus.subSensorStatus[0].usbDataPacketSize = 2048;
  pSensor->sensorStatus.subSensorStatus[0].sdWriteBufferSize = WRITE_BUFFER_SIZE_LSM6DSOX_A;
  pSensor->sensorStatus.subSensorStatus[0].comChannelNumber = -1;
  pSensor->sensorStatus.subSensorStatus[0].ucfLoaded = 0;

  /* SUBSENSOR 1 DESCRIPTOR */
  pSensor->sensorDescriptor.subSensorDescriptor[1].id = 1;
  pSensor->sensorDescriptor.subSensorDescriptor[1].sensorType = COM_TYPE_GYRO;
  pSensor->sensorDescriptor.subSensorDescriptor[1].dimensions = 3;
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[1].dimensionsLabel[0], "x");
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[1].dimensionsLabel[1], "y");
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[1].dimensionsLabel[2], "z");
  pSensor->sensorDescriptor.subSensorDescriptor[1].dataType = DATA_TYPE_INT16;
  pSensor->sensorDescriptor.subSensorDescriptor[1].ODR[0] = 12.5f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].ODR[1] = 26.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].ODR[2] = 52.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].ODR[3] = 104.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].ODR[4] = 208.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].ODR[5] = 416.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].ODR[6] = 833.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].ODR[7] = 1666.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].ODR[8] = 3332.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].ODR[9] = 6667.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].ODR[10] = COM_END_OF_LIST_FLOAT;
  pSensor->sensorDescriptor.subSensorDescriptor[1].samplesPerTimestamp[0] = 0;
  pSensor->sensorDescriptor.subSensorDescriptor[1].samplesPerTimestamp[1] = 1000;
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[1].unit, "mdps");
  pSensor->sensorDescriptor.subSensorDescriptor[1].FS[0] = 125.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].FS[1] = 250.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].FS[2] = 500.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].FS[3] = 1000.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].FS[4] = 2000.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].FS[5] = COM_END_OF_LIST_FLOAT;

  /* SUBSENSOR 1 STATUS */
  if (pxParams != NULL)
  {
    pSensor->sensorStatus.subSensorStatus[1].isActive = pxParams->subSensorActive[1];
    pSensor->sensorStatus.subSensorStatus[1].FS = pxParams->FS[1];
    pSensor->sensorStatus.subSensorStatus[1].ODR = pxParams->ODR[1];
  }
  else
  {
    pSensor->sensorStatus.subSensorStatus[1].isActive = 0;
    pSensor->sensorStatus.subSensorStatus[1].FS = 2000.0f;
    pSensor->sensorStatus.subSensorStatus[1].ODR = 6667.0f;
  }
  pSensor->sensorStatus.subSensorStatus[1].sensitivity = 0.035f * pSensor->sensorStatus.subSensorStatus[1].FS;
  pSensor->sensorStatus.subSensorStatus[1].measuredODR = 0.0f;
  pSensor->sensorStatus.subSensorStatus[1].initialOffset = 0.0f;
  pSensor->sensorStatus.subSensorStatus[1].samplesPerTimestamp = 1000;
  pSensor->sensorStatus.subSensorStatus[1].usbDataPacketSize = 2048;
  pSensor->sensorStatus.subSensorStatus[1].sdWriteBufferSize = WRITE_BUFFER_SIZE_LSM6DSOX_G;
  pSensor->sensorStatus.subSensorStatus[1].comChannelNumber = -1;
  pSensor->sensorStatus.subSensorStatus[1].ucfLoaded = 0;

  /* SUBSENSOR 2 DESCRIPTOR */
  pSensor->sensorDescriptor.subSensorDescriptor[2].id = 2;
  pSensor->sensorDescriptor.subSensorDescriptor[2].sensorType = COM_TYPE_MLC;
  pSensor->sensorDescriptor.subSensorDescriptor[2].dimensions = 8;
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[2].dimensionsLabel[0], "1");
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[2].dimensionsLabel[1], "2");
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[2].dimensionsLabel[2], "3");
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[2].dimensionsLabel[3], "4");
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[2].dimensionsLabel[4], "5");
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[2].dimensionsLabel[5], "6");
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[2].dimensionsLabel[6], "7");
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[2].dimensionsLabel[7], "8");
  pSensor->sensorDescriptor.subSensorDescriptor[2].dataType = DATA_TYPE_INT8;
  pSensor->sensorDescriptor.subSensorDescriptor[2].samplesPerTimestamp[0] = 0;
  pSensor->sensorDescriptor.subSensorDescriptor[2].samplesPerTimestamp[1] = 1000;
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[2].unit, "out");

  /* SUBSENSOR 2 STATUS */
  if (pxParams != NULL)
  {
    pSensor->sensorStatus.subSensorStatus[2].isActive = pxParams->subSensorActive[2];
    pSensor->sensorStatus.subSensorStatus[2].FS = pxParams->FS[2];
    pSensor->sensorStatus.subSensorStatus[2].ODR = pxParams->ODR[2];
  }
  else
  {
    pSensor->sensorStatus.subSensorStatus[2].isActive = 0;
    pSensor->sensorStatus.subSensorStatus[2].FS = 0.0f;
    pSensor->sensorStatus.subSensorStatus[2].ODR = 0.0f;
  }
  pSensor->sensorStatus.subSensorStatus[2].sensitivity = 1.0f;
  pSensor->sensorStatus.subSensorStatus[2].measuredODR = 0.0f;
  pSensor->sensorStatus.subSensorStatus[2].initialOffset = 0.0f;
  pSensor->sensorStatus.subSensorStatus[2].samplesPerTimestamp = 1;
  pSensor->sensorStatus.subSensorStatus[2].usbDataPacketSize = 16;
  pSensor->sensorStatus.subSensorStatus[2].sdWriteBufferSize = WRITE_BUFFER_SIZE_LSM6DSOX_MLC;
  pSensor->sensorStatus.subSensorStatus[2].comChannelNumber = -1;
  pSensor->sensorStatus.subSensorStatus[2].ucfLoaded = 0;

  LSM6DSOX_Init_Param.ODR[0] = pSensor->sensorStatus.subSensorStatus[0].ODR;
  LSM6DSOX_Init_Param.ODR[1] = pSensor->sensorStatus.subSensorStatus[1].ODR;
  LSM6DSOX_Init_Param.ODR[2] = pSensor->sensorStatus.subSensorStatus[2].ODR;
  LSM6DSOX_Init_Param.FS[0] = pSensor->sensorStatus.subSensorStatus[0].FS;
  LSM6DSOX_Init_Param.FS[1] = pSensor->sensorStatus.subSensorStatus[1].FS;
  LSM6DSOX_Init_Param.FS[2] = pSensor->sensorStatus.subSensorStatus[2].FS;
  LSM6DSOX_Init_Param.subSensorActive[0] = pSensor->sensorStatus.subSensorStatus[0].isActive;
  LSM6DSOX_Init_Param.subSensorActive[1] = pSensor->sensorStatus.subSensorStatus[1].isActive;
  LSM6DSOX_Init_Param.subSensorActive[2] = pSensor->sensorStatus.subSensorStatus[2].isActive;

  /**********/
  return 0;
}

/*
 * HW conflict between PWR button and LSM6DSOX INT1:
 * link to TIM2_IC CHANNEL3 to use the timer IC capture callback for the sensor
 */

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig =
  {
    0
  };
  TIM_MasterConfigTypeDef sMasterConfig =
  {
    0
  };
  TIM_IC_InitTypeDef sConfigIC =
  {
    0
  };

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 119;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    SM_Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    SM_Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    SM_Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    SM_Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    SM_Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM_Base MSP Initialization
  * This function configures the hardware resources used in this example
  * @param htim_base: TIM_Base handle pointer
  * @retval None
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim_base)
{
  GPIO_InitTypeDef GPIO_InitStruct =
  {
    0
  };
  if (htim_base->Instance == TIM2)
  {
    /* USER CODE BEGIN TIM2_MspInit 0 */

    /* USER CODE END TIM2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM2 GPIO Configuration
     PA2     ------> TIM2_CH3
     */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* TIM2 interrupt Init */
    HAL_NVIC_SetPriority(TIM2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    /* USER CODE BEGIN TIM2_MspInit 1 */

    /* USER CODE END TIM2_MspInit 1 */
  }
}

/**
  * @brief TIM_Base MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param htim_base: TIM_Base handle pointer
  * @retval None
  */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim_base)
{
  if (htim_base->Instance == TIM2)
  {
    /* USER CODE BEGIN TIM2_MspDeInit 0 */

    /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();

    /**TIM2 GPIO Configuration
     PA2     ------> TIM2_CH3
     */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2);

    /* TIM2 interrupt DeInit */
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
    /* USER CODE BEGIN TIM2_MspDeInit 1 */

    /* USER CODE END TIM2_MspDeInit 1 */
  }
}

