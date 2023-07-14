/**
  ******************************************************************************
  * @file    ism330dhcx_app.c
  * @author  SRA - MCD
  *
  *
  * @brief   This file provides a set of functions to handle ism330dhcx sensor
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
#include "ism330dhcx_app.h"
#include "main.h"
#include "cmsis_os.h"
#include "sensors_manager.h"
#include "com_manager.h"
#include "device_description.h"
#include "ism330dhcx_reg.h"
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define WRITE_BUFFER_SIZE_ISM330DHCX_A   (uint32_t)(16384)
#define WRITE_BUFFER_SIZE_ISM330DHCX_G   (uint32_t)(16384)
#define WRITE_BUFFER_SIZE_ISM330DHCX_MLC (uint32_t)(1024)

#define ISM330DHCX_SAMPLE_SIZE  (7)
#define ISM330DHCX_TAG_ACC      (0x02)

#define MLC_CONFIG              (0x00000001)
#define DHCX_INIT               (0x00000010)
#define DHCX_FIFO               (0x00000011)
#define DHCX_MLC                (0x00000100)
#define DHCX_SUSPEND            (0x00000101)


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static int32_t s_nISM330DHCX_id = 0;

static volatile double TimeStamp_ism330dhcx;
static uint8_t ism330dhcx_mem[ISM330DHCX_MAX_SAMPLES_PER_IT * 7];
static uint8_t ism330dhcx_mem_app[ISM330DHCX_MAX_SAMPLES_PER_IT / 2 * 6]; /*without Tag*/
uint16_t ism330dhcx_samples_per_it;

#if (HSD_USE_DUMMY_DATA == 1)
static int16_t dummyDataCounter_acc = 0;
static int16_t dummyDataCounter_gyro = 0;
#endif /* HSD_USE_DUMMY_DATA == 1 */

static volatile double TimeStamp_mlc;
static uint8_t mlc_mem[8];
ism330dhcx_mlc_status_mainpage_t mlc_status;

volatile uint8_t ISM330DHCX_params_loading = 0;
volatile uint8_t UCF_loading = 0;

static volatile uint32_t UCFSize;
static volatile char *UCFData;

SM_Init_Param_t ISM330DHCX_Init_Param;
SM_Sensor_State_t ISM330DHCX_Sensor_State = SM_SENSOR_STATE_INITIALIZING;
volatile static uint32_t MLC_Data_Ready = 0;

/* Semaphore used to wait on component interrupt */
osSemaphoreId ism330dhcx_DreadySem_id;
osSemaphoreDef(ism330dhcx_DreadySem);

/* Semaphore used to wait on BUS data read complete, managed by lower layer */
osSemaphoreId ism330dhcxReadSem_id;
osSemaphoreDef(ism330dhcxReadSem);

osMessageQId ism330dhcxThreadQueue_id;
osMessageQDef(ism330dhcxThreadQueue, 100, int);

sensor_handle_t ism330dhcx_hdl_instance = {0, 0, ISM330DHCX_SPI_CS_GPIO_Port, ISM330DHCX_SPI_CS_Pin,
                                           &ism330dhcxReadSem_id
                                          };
stmdev_ctx_t ism330dhcx_ctx_instance = {SM_SPI_Write_Os, SM_SPI_Read_Os, &ism330dhcx_hdl_instance};

EXTI_HandleTypeDef mlc_exti;
EXTI_HandleTypeDef ism330dhcx_exti;

/* Private function prototypes -----------------------------------------------*/
osThreadId ISM330DHCX_Thread_Id;
static void ISM330DHCX_Thread(void const *argument);
static void HSD_MLC_Int_Config(void);
static void ISM330DHCX_Program_MLC(uint32_t size, char *buffer);
static void ISM330DHCX_Init(void);
static void ISM330DHCX_Read_MLC(void);
static void ISM330DHCX_Read_Data(void);
static void ISM330DHCX_Read_Data_From_FIFO(void);
static void ISM330DHCX_Suspend(void);
#if (HSD_USE_DUMMY_DATA == 1)
static void ISM330DHCX_CreateDummyData(void);
#endif /* HSD_USE_DUMMY_DATA == 1 */

static void ISM330DHCX_updateFromUCF(void);
static void ISM330DHCX_XL_ODR_From_UCF(void);
static void ISM330DHCX_XL_FS_From_UCF(void);
static void ISM330DHCX_GY_ODR_From_UCF(void);
static void ISM330DHCX_GY_FS_From_UCF(void);

static void ISM330DHCX_Int_Callback(void);
static void ISM330DHCX_Sensor_Init(void);
static void ISM330DHCX_MLC_Int_Callback(void);

void ISM330DHCX_sendCMD(uint32_t cmd);
static uint8_t ISM330DHCX_updateSensorStatus(void);


/**
  * @brief IIS3DWB GPIO Initialization Function
  * @param None
  * @retval None
  */
void ISM330DHCX_Peripheral_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  ISM330DHCX_SPI_CS_GPIO_CLK_ENABLE();
  ISM330DHCX_INT1_GPIO_CLK_ENABLE();
  ISM330DHCX_INT2_GPIO_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ISM330DHCX_SPI_CS_GPIO_Port, ISM330DHCX_SPI_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : IIS3DWB_SPI_CS_Pin */
  GPIO_InitStruct.Pin = ISM330DHCX_SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(ISM330DHCX_SPI_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STTS751_INT_Pin IIS3DWB_INT1_Pin */
  GPIO_InitStruct.Pin =  ISM330DHCX_INT1_Pin ;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ISM330DHCX_INT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STTS751_INT_Pin IIS3DWB_INT1_Pin */
  GPIO_InitStruct.Pin =  ISM330DHCX_INT2_Pin ;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ISM330DHCX_INT2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(ISM330DHCX_INT1_EXTI_IRQn, 5, 0);

  HAL_EXTI_GetHandle(&ism330dhcx_exti, ISM330DHCX_INT1_EXTI_LINE);
  HAL_EXTI_RegisterCallback(&ism330dhcx_exti,  HAL_EXTI_COMMON_CB_ID, ISM330DHCX_Int_Callback);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(ISM330DHCX_INT2_EXTI_IRQn, 5, 0);

  HAL_EXTI_GetHandle(&mlc_exti, ISM330DHCX_INT2_EXTI_LINE);
  HAL_EXTI_RegisterCallback(&mlc_exti,  HAL_EXTI_COMMON_CB_ID, ISM330DHCX_MLC_Int_Callback);
}


void ISM330DHCX_OS_Init(void)
{
  /* Data read complete semaphore initialization */
  ism330dhcxReadSem_id = osSemaphoreCreate(osSemaphore(ism330dhcxReadSem), 1);
  osSemaphoreWait(ism330dhcxReadSem_id, osWaitForever);

  /* Data ready interrupt semaphore initialization */
  ism330dhcx_DreadySem_id = osSemaphoreCreate(osSemaphore(ism330dhcx_DreadySem), 1);
  osSemaphoreWait(ism330dhcx_DreadySem_id,  osWaitForever);

  ism330dhcxThreadQueue_id = osMessageCreate(osMessageQ(ism330dhcxThreadQueue), NULL);
  vQueueAddToRegistry(ism330dhcxThreadQueue_id, "ism330dhcxThreadQueue_id");

  /* Thread 1 definition */
  osThreadDef(ISM330_RD_USR_THREAD, ISM330DHCX_Thread, HSD_ISM330DHCX_THREAD_PRIO, 1, 8000 / 4);
  /* Start thread 1 */
  ISM330DHCX_Thread_Id = osThreadCreate(osThread(ISM330_RD_USR_THREAD), NULL);
}


static void ISM330DHCX_Thread(void const *argument)
{
  (void) argument;
  osEvent evt;

#if (configUSE_APPLICATION_TASK_TAG == 1 && defined(HSD_TASK_ISM330DHCX_DEBUG_PIN))
  vTaskSetApplicationTaskTag(NULL, (TaskHookFunction_t)HSD_TASK_ISM330DHCX_DEBUG_PIN);
#endif /* configUSE_APPLICATION_TASK_TAG == 1 && defined(HSD_TASK_ISM330DHCX_DEBUG_PIN) */

  ism330dhcx_reset_set(&ism330dhcx_ctx_instance, 1);

  for (;;)
  {
    evt = osMessageGet(ism330dhcxThreadQueue_id, osWaitForever);

    if (evt.status == osEventMessage)
    {
      switch (evt.value.v)
      {
        case MLC_CONFIG:
        {
          ISM330DHCX_Program_MLC((uint32_t)UCFSize, (char *)UCFData);
          HSD_free((void *)UCFData);
          ISM330DHCX_Sensor_State = SM_SENSOR_STATE_INITIALIZING;
          break;
        }
        case DHCX_INIT:
        {
          ISM330DHCX_Init();
          ISM330DHCX_Sensor_State = SM_SENSOR_STATE_RUNNING;
          break;
        }
        case DHCX_FIFO:
        {
          if (ISM330DHCX_Sensor_State == SM_SENSOR_STATE_RUNNING)
          {
            ISM330DHCX_Read_Data();
          }
          break;
        }
        case DHCX_MLC:
        {
          if (ISM330DHCX_Sensor_State == SM_SENSOR_STATE_RUNNING)
          {
            ISM330DHCX_Read_MLC();
          }
          break;
        }
        case DHCX_SUSPEND:
        {
          ISM330DHCX_Suspend();
          ISM330DHCX_Sensor_State = SM_SENSOR_STATE_SUSPENDED;
          break;
        }
        default :
        {
          break;
        }
      }
    }
  }
}

static void ISM330DHCX_Init(void)
{
  COM_SubSensorStatus_t *pSubSensorStatus = COM_GetSubSensorStatus(ISM330DHCX_Get_Id(), 2);
  if (ISM330DHCX_Init_Param.subSensorActive[2] == 0 || pSubSensorStatus->ucfLoaded == 0)
  {
    ism330dhcx_reset_set(&ism330dhcx_ctx_instance, 1);
  }
  ISM330DHCX_Sensor_Init();
  if (ISM330DHCX_Init_Param.subSensorActive[2]) /* MLC isActive */
  {
    /* Route int pins for MLC */
    HSD_MLC_Int_Config();
  }
  ism330dhcx_fifo_mode_set(&ism330dhcx_ctx_instance, ISM330DHCX_STREAM_MODE);
  HAL_NVIC_EnableIRQ(ISM330DHCX_INT1_EXTI_IRQn);
}

static void ISM330DHCX_Read_Data(void)
{
  uint8_t reg[2];

  /* Check FIFO_WTM_IA anf fifo level. We do not use PID in order to avoid reading one register twice */
  ism330dhcx_read_reg(&ism330dhcx_ctx_instance, ISM330DHCX_FIFO_STATUS1, reg, 2);
  uint16_t fifo_level = ((reg[1] & 0x03) << 8) + reg[0];

  if ((reg[1]) & 0x80  && (fifo_level >= ism330dhcx_samples_per_it))
  {
    ISM330DHCX_Read_Data_From_FIFO();
  }
}

static void ISM330DHCX_Read_Data_From_FIFO(void)
{
  uint16_t i = 0;
  /* Read sensor data from FIFO */
  ism330dhcx_read_reg(&ism330dhcx_ctx_instance, ISM330DHCX_FIFO_DATA_OUT_TAG, (uint8_t *)ism330dhcx_mem,
                      ism330dhcx_samples_per_it * 7);

#if (HSD_USE_DUMMY_DATA == 1)
  ISM330DHCX_CreateDummyData();
#endif /* HSD_USE_DUMMY_DATA == 1 */

  if (ISM330DHCX_Init_Param.subSensorActive[0]
      && ISM330DHCX_Init_Param.subSensorActive[1])     /* Read both ACC and GYRO */
  {
    uint32_t ODR_Acc  = (uint32_t)COM_GetSubSensorStatus(ISM330DHCX_Get_Id(), 0)->ODR;
    uint32_t ODR_Gyro = (uint32_t)COM_GetSubSensorStatus(ISM330DHCX_Get_Id(), 1)->ODR;
    uint32_t gyroSamplesCount = 0;
    uint32_t accSamplesCount = 0;

    int16_t *p16src = (int16_t *)ism330dhcx_mem;
    int16_t *pAcc;
    int16_t *pGyro;

    if (ODR_Acc > ODR_Gyro) /* Acc is faster than Gyro */
    {
      pAcc  = (int16_t *)ism330dhcx_mem;
      pGyro = (int16_t *)ism330dhcx_mem_app;
    }
    else
    {
      pAcc  = (int16_t *)ism330dhcx_mem_app;
      pGyro = (int16_t *)ism330dhcx_mem;
    }

    uint8_t *pTag = (uint8_t *)p16src;

    for (i = 0; i < ism330dhcx_samples_per_it; i++)
    {
      if (((*pTag) >> 3) == ISM330DHCX_TAG_ACC)
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
      ISM330DHCX_Data_Ready(0, (uint8_t *)ism330dhcx_mem, accSamplesCount * 6, TimeStamp_ism330dhcx);
      ISM330DHCX_Data_Ready(1, (uint8_t *)ism330dhcx_mem_app, gyroSamplesCount * 6, TimeStamp_ism330dhcx);
    }
    else
    {
      ISM330DHCX_Data_Ready(0, (uint8_t *)ism330dhcx_mem_app, accSamplesCount * 6, TimeStamp_ism330dhcx);
      ISM330DHCX_Data_Ready(1, (uint8_t *)ism330dhcx_mem, gyroSamplesCount * 6, TimeStamp_ism330dhcx);
    }
  }
  else /* 1 subsensor active only --> simply drop TAGS */
  {
    int16_t *p16src = (int16_t *)ism330dhcx_mem;
    int16_t *p16dest = (int16_t *)ism330dhcx_mem;
    for (i = 0; i < ism330dhcx_samples_per_it; i++)
    {
      p16src = (int16_t *) & ((uint8_t *)(p16src))[1];
      *p16dest++ = *p16src++;
      *p16dest++ = *p16src++;
      *p16dest++ = *p16src++;
    }
    if (ISM330DHCX_Init_Param.subSensorActive[0]) /* Acc only */
    {
      ISM330DHCX_Data_Ready(0, (uint8_t *)ism330dhcx_mem, ism330dhcx_samples_per_it * 6, TimeStamp_ism330dhcx);
    }
    else if (ISM330DHCX_Init_Param.subSensorActive[1]) /* Gyro only */
    {
      ISM330DHCX_Data_Ready(1, (uint8_t *)ism330dhcx_mem, ism330dhcx_samples_per_it * 6, TimeStamp_ism330dhcx);
    }
  }
}

#if (HSD_USE_DUMMY_DATA == 1)
static void ISM330DHCX_CreateDummyData(void)
{
  uint16_t i = 0;
  int16_t *p16 = (int16_t *)ism330dhcx_mem;

  for (i = 0; i < ism330dhcx_samples_per_it; i++)
  {
    p16 = (int16_t *)(&ism330dhcx_mem[i * 7] + 1);
    if ((ism330dhcx_mem[i * 7] >> 3) == ISM330DHCX_TAG_ACC)
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
#endif /* HSD_USE_DUMMY_DATA == 1 */

static void ISM330DHCX_Read_MLC(void)
{
  if (ISM330DHCX_Init_Param.subSensorActive[2] && MLC_Data_Ready == 1)
  {
    uint32_t ii;
    for (ii = 0; ii < 8; ii++)
    {
      ism330dhcx_mem_bank_set(&ism330dhcx_ctx_instance, ISM330DHCX_EMBEDDED_FUNC_BANK);
      ism330dhcx_read_reg(&ism330dhcx_ctx_instance, ISM330DHCX_MLC0_SRC + ii, (uint8_t *)&mlc_mem[ii], 1);
      ism330dhcx_mem_bank_set(&ism330dhcx_ctx_instance, ISM330DHCX_USER_BANK);
    }
    ism330dhcx_mlc_status_get(&ism330dhcx_ctx_instance, &mlc_status);
    ISM330DHCX_Data_Ready(2, (uint8_t *)mlc_mem, 8, TimeStamp_mlc);
    MLC_Data_Ready = 0;
  }
}

static void ISM330DHCX_Suspend(void)
{
#if (HSD_USE_DUMMY_DATA == 1)
  dummyDataCounter_acc = 0;
  dummyDataCounter_gyro = 0;
#endif /* HSD_USE_DUMMY_DATA == 1 */
  ism330dhcx_fifo_gy_batch_set(&ism330dhcx_ctx_instance, ISM330DHCX_GY_NOT_BATCHED);
  ism330dhcx_fifo_xl_batch_set(&ism330dhcx_ctx_instance, ISM330DHCX_XL_NOT_BATCHED);
}


void ISM330DHCX_sendCMD(uint32_t cmd)
{
  if (osMessagePut(ism330dhcxThreadQueue_id, cmd, 0) != osOK)
  {
    while (1);
  }
}


static uint8_t ISM330DHCX_updateSensorStatus(void)
{
  COM_SubSensorStatus_t *pSubSensorStatus = COM_GetSubSensorStatus(ISM330DHCX_Get_Id(), 2);
  uint8_t ret = 0;

  if (pSubSensorStatus->ucfLoaded == 1)
  {
    if ((ISM330DHCX_params_loading != 0) && (pSubSensorStatus->isActive == ISM330DHCX_Init_Param.subSensorActive[2]))
    {
      pSubSensorStatus->ucfLoaded = 0;
      pSubSensorStatus->isActive = 0;
      ret += 1;
    }
    else
    {
      if (UCF_loading == 1)
      {
        ISM330DHCX_updateFromUCF();
        ret += 1;
      }
    }
  }
  COM_Sensor_t *pSensor;
  pSensor = COM_GetSensor(ISM330DHCX_Get_Id());

  ISM330DHCX_Init_Param.ODR[0] = pSensor->sensorStatus.subSensorStatus[0].ODR;
  ISM330DHCX_Init_Param.ODR[1] = pSensor->sensorStatus.subSensorStatus[1].ODR;
  ISM330DHCX_Init_Param.ODR[2] = pSensor->sensorStatus.subSensorStatus[2].ODR;
  ISM330DHCX_Init_Param.FS[0] = pSensor->sensorStatus.subSensorStatus[0].FS;
  ISM330DHCX_Init_Param.FS[1] = pSensor->sensorStatus.subSensorStatus[1].FS;
  ISM330DHCX_Init_Param.subSensorActive[0] = pSensor->sensorStatus.subSensorStatus[0].isActive;
  ISM330DHCX_Init_Param.subSensorActive[1] = pSensor->sensorStatus.subSensorStatus[1].isActive;
  ISM330DHCX_Init_Param.subSensorActive[2] = pSensor->sensorStatus.subSensorStatus[2].isActive;
  update_samplesPerTimestamp(pSensor);

  ISM330DHCX_params_loading = 0;
  return ret;
}

static void ISM330DHCX_updateFromUCF(void)
{
  ISM330DHCX_XL_ODR_From_UCF();
  ISM330DHCX_XL_FS_From_UCF();
  ISM330DHCX_GY_ODR_From_UCF();
  ISM330DHCX_GY_FS_From_UCF();

  COM_SubSensorStatus_t *pSubSensorStatus = COM_GetSubSensorStatus(ISM330DHCX_Get_Id(), 0);
  pSubSensorStatus->ODR = ISM330DHCX_Init_Param.ODR[0];
  pSubSensorStatus->FS = ISM330DHCX_Init_Param.FS[0];
  pSubSensorStatus->isActive = ISM330DHCX_Init_Param.subSensorActive[0];

  pSubSensorStatus = COM_GetSubSensorStatus(ISM330DHCX_Get_Id(), 1);
  pSubSensorStatus->ODR = ISM330DHCX_Init_Param.ODR[1];
  pSubSensorStatus->FS = ISM330DHCX_Init_Param.FS[1];
  pSubSensorStatus->isActive = ISM330DHCX_Init_Param.subSensorActive[1];

  ISM330DHCX_params_loading = 0;
  UCF_loading = 0;
}

static void ISM330DHCX_XL_ODR_From_UCF(void)
{
  ism330dhcx_odr_xl_t ism330dhcx_odr_xl = ISM330DHCX_XL_ODR_OFF;
  COM_SubSensorStatus_t *pSubSensorStatus = COM_GetSubSensorStatus(ISM330DHCX_Get_Id(), 0);

  ISM330DHCX_Init_Param.subSensorActive[0] = 1;
  pSubSensorStatus->isActive = 1;

  ism330dhcx_xl_data_rate_get(&ism330dhcx_ctx_instance, &ism330dhcx_odr_xl);
  float Odr = 12.5f;

  switch (ism330dhcx_odr_xl)
  {
    case ISM330DHCX_XL_ODR_OFF:
    {
      ISM330DHCX_Init_Param.subSensorActive[0] = 0;
      COM_SubSensorStatus_t *pSubSensorStatus = COM_GetSubSensorStatus(ISM330DHCX_Get_Id(), 0);
      pSubSensorStatus->isActive = 0;
    }
    break;

    case ISM330DHCX_XL_ODR_12Hz5:
      Odr = 12.5f;
      break;

    case ISM330DHCX_XL_ODR_26Hz:
      Odr = 26.0f;
      break;

    case ISM330DHCX_XL_ODR_52Hz:
      Odr = 52.0f;
      break;

    case ISM330DHCX_XL_ODR_104Hz:
      Odr = 104.0f;
      break;

    case ISM330DHCX_XL_ODR_208Hz:
      Odr = 208.0f;
      break;

    case ISM330DHCX_XL_ODR_416Hz:
      Odr = 416.0f;
      break;

    case ISM330DHCX_XL_ODR_833Hz:
      Odr = 833.0f;
      break;

    case ISM330DHCX_XL_ODR_1666Hz:
      Odr = 1666.0f;
      break;

    case ISM330DHCX_XL_ODR_3332Hz:
      Odr = 3332.0f;
      break;

    case ISM330DHCX_XL_ODR_6667Hz:
      Odr = 6667.0f;
      break;

    default:
      break;
  }
  ISM330DHCX_Init_Param.ODR[0] = Odr;
}

static void ISM330DHCX_XL_FS_From_UCF(void)
{
  ism330dhcx_fs_xl_t fs_xl;
  ism330dhcx_xl_full_scale_get(&ism330dhcx_ctx_instance, &fs_xl);
  float FullScale = 0.0;

  switch (fs_xl)
  {
    case ISM330DHCX_2g:
      FullScale =  2.0;
      break;

    case ISM330DHCX_4g:
      FullScale =  4.0;
      break;

    case ISM330DHCX_8g:
      FullScale =  8.0;
      break;

    case ISM330DHCX_16g:
      FullScale = 16.0;
      break;

    default:
      break;
  }
  ISM330DHCX_Init_Param.FS[0] = FullScale;
}

static void ISM330DHCX_GY_ODR_From_UCF(void)
{
  ism330dhcx_odr_g_t ism330dhcx_odr_g = ISM330DHCX_GY_ODR_OFF;
  COM_SubSensorStatus_t *pSubSensorStatus = COM_GetSubSensorStatus(ISM330DHCX_Get_Id(), 1);

  ISM330DHCX_Init_Param.subSensorActive[1] = 1;
  pSubSensorStatus->isActive = 1;

  ism330dhcx_gy_data_rate_get(&ism330dhcx_ctx_instance, &ism330dhcx_odr_g);
  float Odr = 12.5f;

  switch (ism330dhcx_odr_g)
  {
    case ISM330DHCX_GY_ODR_OFF:
    {
      ISM330DHCX_Init_Param.subSensorActive[1] = 0;
      COM_SubSensorStatus_t *pSubSensorStatus = COM_GetSubSensorStatus(ISM330DHCX_Get_Id(), 1);
      pSubSensorStatus->isActive = 0;
    }
    break;

    case ISM330DHCX_GY_ODR_12Hz5:
      Odr = 12.5f;
      break;

    case ISM330DHCX_GY_ODR_26Hz:
      Odr = 26.0f;
      break;

    case ISM330DHCX_GY_ODR_52Hz:
      Odr = 52.0f;
      break;

    case ISM330DHCX_GY_ODR_104Hz:
      Odr = 104.0f;
      break;

    case ISM330DHCX_GY_ODR_208Hz:
      Odr = 208.0f;
      break;

    case ISM330DHCX_GY_ODR_416Hz:
      Odr = 416.0f;
      break;

    case ISM330DHCX_GY_ODR_833Hz:
      Odr = 833.0f;
      break;

    case ISM330DHCX_GY_ODR_1666Hz:
      Odr = 1666.0f;
      break;

    case ISM330DHCX_GY_ODR_3332Hz:
      Odr = 3332.0f;
      break;

    case ISM330DHCX_GY_ODR_6667Hz:
      Odr = 6667.0f;
      break;

    default:
      break;
  }
  ISM330DHCX_Init_Param.ODR[1] = Odr;
}

static void ISM330DHCX_GY_FS_From_UCF(void)
{
  ism330dhcx_fs_g_t fs_g;
  ism330dhcx_gy_full_scale_get(&ism330dhcx_ctx_instance, &fs_g);
  float FullScale = 0.0;

  switch (fs_g)
  {
    case ISM330DHCX_125dps:
      FullScale =  125;
      break;

    case ISM330DHCX_250dps:
      FullScale =  250;
      break;

    case ISM330DHCX_500dps:
      FullScale =  500;
      break;

    case ISM330DHCX_1000dps:
      FullScale = 1000;
      break;

    case ISM330DHCX_2000dps:
      FullScale = 2000;
      break;

    case ISM330DHCX_4000dps:
      FullScale = 4000;
      break;

    default:
      break;
  }

  ISM330DHCX_Init_Param.FS[1] = FullScale;
}


uint8_t ISM330DHCX_updateConfig(void)
{
  COM_SubSensorStatus_t *pSubSensorStatus = COM_GetSubSensorStatus(ISM330DHCX_Get_Id(), 2);
  if (pSubSensorStatus->ucfLoaded == 0
      && pSubSensorStatus->isActive == 1)      /* If you enter here, MLC isActive is true but no UCF are available */
  {
    /* So, MLC can't be active --> subSensorStatus is forced not Active */
    pSubSensorStatus->isActive = 0;
  }

  return ISM330DHCX_updateSensorStatus();
}


static void ISM330DHCX_Sensor_Init(void)
{
  uint8_t reg0;
  uint16_t ism330dhcx_wtm_level = 0;
  uint16_t ism330dhcx_wtm_level_acc;
  uint16_t ism330dhcx_wtm_level_gyro;
  ism330dhcx_odr_xl_t ism330dhcx_odr_xl = ISM330DHCX_XL_ODR_OFF;
  ism330dhcx_bdr_xl_t ism330dhcx_bdr_xl = ISM330DHCX_XL_NOT_BATCHED;
  ism330dhcx_odr_g_t ism330dhcx_odr_g = ISM330DHCX_GY_ODR_OFF;
  ism330dhcx_bdr_gy_t ism330dhcx_bdr_gy = ISM330DHCX_GY_NOT_BATCHED;

  ism330dhcx_i2c_interface_set(&ism330dhcx_ctx_instance, ISM330DHCX_I2C_DISABLE);
  ism330dhcx_device_id_get(&ism330dhcx_ctx_instance, (uint8_t *)&reg0);

  /* AXL FS */
  if (ISM330DHCX_Init_Param.FS[0] < 3.0f)
  {
    ism330dhcx_xl_full_scale_set(&ism330dhcx_ctx_instance, ISM330DHCX_2g);
  }
  else if (ISM330DHCX_Init_Param.FS[0] < 5.0f)
  {
    ism330dhcx_xl_full_scale_set(&ism330dhcx_ctx_instance, ISM330DHCX_4g);
  }
  else if (ISM330DHCX_Init_Param.FS[0] < 9.0f)
  {
    ism330dhcx_xl_full_scale_set(&ism330dhcx_ctx_instance, ISM330DHCX_8g);
  }
  else if (ISM330DHCX_Init_Param.FS[0] < 17.0f)
  {
    ism330dhcx_xl_full_scale_set(&ism330dhcx_ctx_instance, ISM330DHCX_16g);
  }

  /* GYRO FS */
  if (ISM330DHCX_Init_Param.FS[1] < 126.0f)
  {
    ism330dhcx_gy_full_scale_set(&ism330dhcx_ctx_instance, ISM330DHCX_125dps);
  }
  else if (ISM330DHCX_Init_Param.FS[1] < 251.0f)
  {
    ism330dhcx_gy_full_scale_set(&ism330dhcx_ctx_instance, ISM330DHCX_250dps);
  }
  else if (ISM330DHCX_Init_Param.FS[1] < 501.0f)
  {
    ism330dhcx_gy_full_scale_set(&ism330dhcx_ctx_instance, ISM330DHCX_500dps);
  }
  else if (ISM330DHCX_Init_Param.FS[1] < 1001.0f)
  {
    ism330dhcx_gy_full_scale_set(&ism330dhcx_ctx_instance, ISM330DHCX_1000dps);
  }
  else if (ISM330DHCX_Init_Param.FS[1] < 2001.0f)
  {
    ism330dhcx_gy_full_scale_set(&ism330dhcx_ctx_instance, ISM330DHCX_2000dps);
  }
  else if (ISM330DHCX_Init_Param.FS[1] < 4001.0f)
  {
    ism330dhcx_gy_full_scale_set(&ism330dhcx_ctx_instance, ISM330DHCX_4000dps);
  }


  if (ISM330DHCX_Init_Param.ODR[0] < 13.0f)
  {
    ism330dhcx_odr_xl = ISM330DHCX_XL_ODR_12Hz5;
    ism330dhcx_bdr_xl = ISM330DHCX_XL_BATCHED_AT_12Hz5;
  }
  else if (ISM330DHCX_Init_Param.ODR[0] < 27.0f)
  {
    ism330dhcx_odr_xl = ISM330DHCX_XL_ODR_26Hz;
    ism330dhcx_bdr_xl = ISM330DHCX_XL_BATCHED_AT_26Hz;
  }
  else if (ISM330DHCX_Init_Param.ODR[0] < 53.0f)
  {
    ism330dhcx_odr_xl = ISM330DHCX_XL_ODR_52Hz;
    ism330dhcx_bdr_xl = ISM330DHCX_XL_BATCHED_AT_52Hz;
  }
  else if (ISM330DHCX_Init_Param.ODR[0] < 105.0f)
  {
    ism330dhcx_odr_xl = ISM330DHCX_XL_ODR_104Hz;
    ism330dhcx_bdr_xl = ISM330DHCX_XL_BATCHED_AT_104Hz;
  }
  else if (ISM330DHCX_Init_Param.ODR[0] < 209.0f)
  {
    ism330dhcx_odr_xl = ISM330DHCX_XL_ODR_208Hz;
    ism330dhcx_bdr_xl = ISM330DHCX_XL_BATCHED_AT_208Hz;
  }
  else if (ISM330DHCX_Init_Param.ODR[0] < 417.0f)
  {
    ism330dhcx_odr_xl = ISM330DHCX_XL_ODR_416Hz;
    ism330dhcx_bdr_xl = ISM330DHCX_XL_BATCHED_AT_417Hz;
  }
  else if (ISM330DHCX_Init_Param.ODR[0] < 834.0f)
  {
    ism330dhcx_odr_xl = ISM330DHCX_XL_ODR_833Hz;
    ism330dhcx_bdr_xl = ISM330DHCX_XL_BATCHED_AT_833Hz;
  }
  else if (ISM330DHCX_Init_Param.ODR[0] < 1667.0f)
  {
    ism330dhcx_odr_xl = ISM330DHCX_XL_ODR_1666Hz;
    ism330dhcx_bdr_xl = ISM330DHCX_XL_BATCHED_AT_1667Hz;
  }
  else if (ISM330DHCX_Init_Param.ODR[0] < 3333.0f)
  {
    ism330dhcx_odr_xl = ISM330DHCX_XL_ODR_3332Hz;
    ism330dhcx_bdr_xl = ISM330DHCX_XL_BATCHED_AT_3333Hz;
  }
  else if (ISM330DHCX_Init_Param.ODR[0] < 6668.0f)
  {
    ism330dhcx_odr_xl = ISM330DHCX_XL_ODR_6667Hz;
    ism330dhcx_bdr_xl = ISM330DHCX_XL_BATCHED_AT_6667Hz;
  }

  if (ISM330DHCX_Init_Param.ODR[1] < 13.0f)
  {
    ism330dhcx_odr_g = ISM330DHCX_GY_ODR_12Hz5;
    ism330dhcx_bdr_gy = ISM330DHCX_GY_BATCHED_AT_12Hz5;
  }
  else if (ISM330DHCX_Init_Param.ODR[1] < 27.0f)
  {
    ism330dhcx_odr_g = ISM330DHCX_GY_ODR_26Hz;
    ism330dhcx_bdr_gy = ISM330DHCX_GY_BATCHED_AT_26Hz;
  }
  else if (ISM330DHCX_Init_Param.ODR[1] < 53.0f)
  {
    ism330dhcx_odr_g = ISM330DHCX_GY_ODR_52Hz;
    ism330dhcx_bdr_gy = ISM330DHCX_GY_BATCHED_AT_52Hz;
  }
  else if (ISM330DHCX_Init_Param.ODR[1] < 105.0f)
  {
    ism330dhcx_odr_g = ISM330DHCX_GY_ODR_104Hz;
    ism330dhcx_bdr_gy = ISM330DHCX_GY_BATCHED_AT_104Hz;
  }
  else if (ISM330DHCX_Init_Param.ODR[1] < 209.0f)
  {
    ism330dhcx_odr_g = ISM330DHCX_GY_ODR_208Hz;
    ism330dhcx_bdr_gy = ISM330DHCX_GY_BATCHED_AT_208Hz;
  }
  else if (ISM330DHCX_Init_Param.ODR[1] < 417.0f)
  {
    ism330dhcx_odr_g = ISM330DHCX_GY_ODR_416Hz;
    ism330dhcx_bdr_gy = ISM330DHCX_GY_BATCHED_AT_417Hz;
  }
  else if (ISM330DHCX_Init_Param.ODR[1] < 834.0f)
  {
    ism330dhcx_odr_g = ISM330DHCX_GY_ODR_833Hz;
    ism330dhcx_bdr_gy = ISM330DHCX_GY_BATCHED_AT_833Hz;
  }
  else if (ISM330DHCX_Init_Param.ODR[1] < 1667.0f)
  {
    ism330dhcx_odr_g = ISM330DHCX_GY_ODR_1666Hz;
    ism330dhcx_bdr_gy = ISM330DHCX_GY_BATCHED_AT_1667Hz;
  }
  else if (ISM330DHCX_Init_Param.ODR[1] < 3333.0f)
  {
    ism330dhcx_odr_g = ISM330DHCX_GY_ODR_3332Hz;
    ism330dhcx_bdr_gy = ISM330DHCX_GY_BATCHED_AT_3333Hz;
  }
  else if (ISM330DHCX_Init_Param.ODR[1] < 6668.0f)
  {
    ism330dhcx_odr_g = ISM330DHCX_GY_ODR_6667Hz;
    ism330dhcx_bdr_gy = ISM330DHCX_GY_BATCHED_AT_6667Hz;
  }

  if (ISM330DHCX_Init_Param.subSensorActive[0])
  {
    ism330dhcx_xl_data_rate_set(&ism330dhcx_ctx_instance, ism330dhcx_odr_xl);
    ism330dhcx_fifo_xl_batch_set(&ism330dhcx_ctx_instance, ism330dhcx_bdr_xl);
  }
  else
  {
    ism330dhcx_xl_data_rate_set(&ism330dhcx_ctx_instance, ISM330DHCX_XL_ODR_OFF);
    ism330dhcx_fifo_xl_batch_set(&ism330dhcx_ctx_instance, ISM330DHCX_XL_NOT_BATCHED);
  }

  if (ISM330DHCX_Init_Param.subSensorActive[1])
  {
    ism330dhcx_gy_data_rate_set(&ism330dhcx_ctx_instance, ism330dhcx_odr_g);
    ism330dhcx_fifo_gy_batch_set(&ism330dhcx_ctx_instance, ism330dhcx_bdr_gy);
  }
  else
  {
    ism330dhcx_gy_data_rate_set(&ism330dhcx_ctx_instance, ISM330DHCX_GY_ODR_OFF);
    ism330dhcx_fifo_gy_batch_set(&ism330dhcx_ctx_instance, ISM330DHCX_GY_NOT_BATCHED);
  }

  /* Calculation of watermark and samples per int*/
  ism330dhcx_wtm_level_acc = ((uint16_t)ISM330DHCX_Init_Param.ODR[0] * (uint16_t)ISM330DHCX_MAX_DRDY_PERIOD);
  ism330dhcx_wtm_level_gyro = ((uint16_t)ISM330DHCX_Init_Param.ODR[1] * (uint16_t)ISM330DHCX_MAX_DRDY_PERIOD);

  if (ISM330DHCX_Init_Param.subSensorActive[0]
      && ISM330DHCX_Init_Param.subSensorActive[1])     /* Both subSensor is active */
  {
    if (ism330dhcx_wtm_level_acc > ism330dhcx_wtm_level_gyro)
    {
      ism330dhcx_wtm_level = ism330dhcx_wtm_level_acc;
    }
    else
    {
      ism330dhcx_wtm_level = ism330dhcx_wtm_level_gyro;
    }
  }
  else  /* Only one subSensor is active */
  {
    if (ISM330DHCX_Init_Param.subSensorActive[0])
    {
      ism330dhcx_wtm_level = ism330dhcx_wtm_level_acc;
    }
    else
    {
      ism330dhcx_wtm_level = ism330dhcx_wtm_level_gyro;
    }
  }

  if (ism330dhcx_wtm_level > ISM330DHCX_MAX_WTM_LEVEL)
  {
    ism330dhcx_wtm_level = ISM330DHCX_MAX_WTM_LEVEL;
  }
  else if (ism330dhcx_wtm_level < ISM330DHCX_MIN_WTM_LEVEL)
  {
    ism330dhcx_wtm_level = ISM330DHCX_MIN_WTM_LEVEL;
  }
  ism330dhcx_samples_per_it = ism330dhcx_wtm_level;

  /* Setup int for FIFO */
  ism330dhcx_fifo_watermark_set(&ism330dhcx_ctx_instance, ism330dhcx_wtm_level);

  ism330dhcx_pin_int1_route_t int1_route = {0};
  int1_route.int1_ctrl.int1_fifo_th = 1;
  ism330dhcx_pin_int1_route_set(&ism330dhcx_ctx_instance, &int1_route);
}

static void HSD_MLC_Int_Config(void)
{
  ism330dhcx_pin_int1_route_t pin_int1_route;
  ism330dhcx_pin_int2_route_t pin_int2_route;

  ism330dhcx_pin_int1_route_get(&ism330dhcx_ctx_instance, &pin_int1_route);
  ism330dhcx_pin_int2_route_get(&ism330dhcx_ctx_instance, &pin_int2_route);

  if (pin_int1_route.mlc_int1.int1_mlc1 == 1 || pin_int1_route.md1_cfg.int1_emb_func == 1)
  {
    pin_int1_route.mlc_int1.int1_mlc1 = 0;
    pin_int1_route.md1_cfg.int1_emb_func = 0;
    ism330dhcx_pin_int1_route_set(&ism330dhcx_ctx_instance, &pin_int1_route);
  }

  if (pin_int2_route.mlc_int2.int2_mlc1 == 0 || pin_int2_route.md2_cfg.int2_emb_func == 0)
  {
    pin_int2_route.mlc_int2.int2_mlc1 = 1;
    pin_int2_route.md2_cfg.int2_emb_func = 1;
    ism330dhcx_pin_int2_route_set(&ism330dhcx_ctx_instance, &pin_int2_route);
    HAL_NVIC_EnableIRQ(ISM330DHCX_INT2_EXTI_IRQn);
  }
}


static void ISM330DHCX_Program_MLC(uint32_t size, char *buffer)
{
  char ucf_reg[3];
  char ucf_data[3];
  long reg;
  long data;
  uint32_t i;

  ism330dhcx_reset_set(&ism330dhcx_ctx_instance, 1);

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

    ism330dhcx_write_reg(&ism330dhcx_ctx_instance, (uint8_t)reg, (uint8_t *)&data, 1);
  }

  COM_SubSensorStatus_t *pSubSensorStatus = COM_GetSubSensorStatus(ISM330DHCX_Get_Id(), 2);
  pSubSensorStatus->ucfLoaded = 1;
  pSubSensorStatus->isActive = 1;
  UCF_loading = 1;
  ISM330DHCX_updateConfig();
}


/**
  * @brief IIS3DWB Get a buffer UCF file content
  * @param buffer_in: input buffer with compressed UCF
  * @param size_in: in_buffer size
  * @param buffer_out: output buffer of at least 9*(in_size/4) bytes
  * @param size_out: out_buffer size
  * @retval 0 --> OK, 1 --> Error
  */
int32_t ISM330DHCX_GetUCF_FromBuffer(char *buffer_in, uint32_t size_in, char *buffer_out, uint32_t size_out)
{
  uint8_t *p;
  uint32_t i;

  if (size_out < 9 * (size_in / 4))
  {
    return -1;
  }

  p = (uint8_t *)buffer_out;

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

static void ISM330DHCX_MLC_Int_Callback(void)
{
  TimeStamp_mlc = SM_GetTimeStamp_fromISR();
  MLC_Data_Ready = 1;
  ISM330DHCX_sendCMD(DHCX_MLC);
}

static void ISM330DHCX_Int_Callback(void)
{
  TimeStamp_ism330dhcx = SM_GetTimeStamp_fromISR();
  ISM330DHCX_sendCMD(DHCX_FIFO);
}

void ISM330DHCX_Set_State(SM_Sensor_State_t state)
{
  ISM330DHCX_Sensor_State = state;
}

void ISM330DHCX_Start(void)
{
  ISM330DHCX_Set_State(SM_SENSOR_STATE_INITIALIZING);
  ISM330DHCX_sendCMD(DHCX_INIT);
}

void ISM330DHCX_SetUCF(uint32_t mlcConfigSize, char *mlcConfigData)
{
  UCFSize = mlcConfigSize;
  UCFData = mlcConfigData;
  ISM330DHCX_Set_State(SM_SENSOR_STATE_MLC_CONFIG);
  ISM330DHCX_sendCMD(MLC_CONFIG);
}

void ISM330DHCX_Stop(void)
{
  ISM330DHCX_Set_State(SM_SENSOR_STATE_SUSPENDING);
  ISM330DHCX_sendCMD(DHCX_SUSPEND);
}

__weak void ISM330DHCX_Data_Ready(uint8_t subSensorId, uint8_t *buf, uint16_t size, double timeStamp)
{

}

/**
  * @brief Get Sensor ID
  * @param None
  * @retval Sensor ID
  */
uint8_t ISM330DHCX_Get_Id(void)
{
  return s_nISM330DHCX_id;
}

/**
  * @brief ISM330DHCX Sensor Initialization
  * @param None
  * @retval None
  */
uint8_t ISM330DHCX_Create_Sensor(const SM_Init_Param_t *pxParams)
{
  COM_Sensor_t *pSensor;

  s_nISM330DHCX_id = COM_AddSensor();

  if (s_nISM330DHCX_id == -1)
  {
    return 1; /* error */
  }

  pSensor = COM_GetSensor(s_nISM330DHCX_id);

  /* SENSOR DESCRIPTOR */
  strcpy(pSensor->sensorDescriptor.name, "ISM330DHCX");
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
  pSensor->sensorStatus.subSensorStatus[0].sdWriteBufferSize = WRITE_BUFFER_SIZE_ISM330DHCX_A;
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
  pSensor->sensorDescriptor.subSensorDescriptor[1].FS[5] = 4000.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[1].FS[6] = COM_END_OF_LIST_FLOAT;

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
    pSensor->sensorStatus.subSensorStatus[1].FS = 4000.0f;
    pSensor->sensorStatus.subSensorStatus[1].ODR = 6667.0f;
  }
  pSensor->sensorStatus.subSensorStatus[1].sensitivity = 0.035f * pSensor->sensorStatus.subSensorStatus[1].FS;
  pSensor->sensorStatus.subSensorStatus[1].measuredODR = 0.0f;
  pSensor->sensorStatus.subSensorStatus[1].initialOffset = 0.0f;
  pSensor->sensorStatus.subSensorStatus[1].samplesPerTimestamp = 1000;
  pSensor->sensorStatus.subSensorStatus[1].usbDataPacketSize = 2048;
  pSensor->sensorStatus.subSensorStatus[1].sdWriteBufferSize = WRITE_BUFFER_SIZE_ISM330DHCX_G;
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
  pSensor->sensorStatus.subSensorStatus[2].sdWriteBufferSize = WRITE_BUFFER_SIZE_ISM330DHCX_MLC;
  pSensor->sensorStatus.subSensorStatus[2].comChannelNumber = -1;
  pSensor->sensorStatus.subSensorStatus[2].ucfLoaded = 0;

  ISM330DHCX_Init_Param.ODR[0] = pSensor->sensorStatus.subSensorStatus[0].ODR;
  ISM330DHCX_Init_Param.ODR[1] = pSensor->sensorStatus.subSensorStatus[1].ODR;
  ISM330DHCX_Init_Param.ODR[2] = pSensor->sensorStatus.subSensorStatus[2].ODR;
  ISM330DHCX_Init_Param.FS[0] = pSensor->sensorStatus.subSensorStatus[0].FS;
  ISM330DHCX_Init_Param.FS[1] = pSensor->sensorStatus.subSensorStatus[1].FS;
  ISM330DHCX_Init_Param.FS[2] = pSensor->sensorStatus.subSensorStatus[2].FS;
  ISM330DHCX_Init_Param.subSensorActive[0] = pSensor->sensorStatus.subSensorStatus[0].isActive;
  ISM330DHCX_Init_Param.subSensorActive[1] = pSensor->sensorStatus.subSensorStatus[1].isActive;
  ISM330DHCX_Init_Param.subSensorActive[2] = pSensor->sensorStatus.subSensorStatus[2].isActive;

  /**********/
  return 0;
}


