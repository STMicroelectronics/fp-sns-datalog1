/**
  ******************************************************************************
  * @file    imp34dt05_app.c
  * @author  SRA - MCD
  *
  *
  * @brief   This file provides a set of functions to handle imp34dt05 microphone
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
#include "imp34dt05_app.h"
#include "main.h"
#include "com_manager.h"
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define WRITE_BUFFER_SIZE_IMP34DT05      (uint32_t)(32768)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static int32_t s_nIMP34DT05_id = 0;

SM_Init_Param_t IMP34DT05_Init_Param;
SM_Sensor_State_t IMP34DT05_Sensor_State = SM_SENSOR_STATE_INITIALIZING;

/* Semaphore used to wait on component interrupt */
osMessageQId dmicDreadyQueue_id;
static osMessageQDef(dmicdreadyqueue, 1, int);


static uint32_t dmic_mem[((IMP34DT05_MAX_SAMPLING_FREQUENCY / 1000) * IMP34DT05_MS * 2)];

uint16_t newDataIdxDMic;
uint16_t oldDataLenDMic;
uint16_t newDataLenDMic;

osThreadId IMP34DT05_Thread_Id;

DFSDM_Filter_HandleTypeDef hdfsdm1_filter0;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel5;
DMA_HandleTypeDef hdma_dfsdm1_flt0;

static volatile uint32_t tim_value = 0, tim_value_old = 0, period = 0;
static volatile uint64_t ts_imp34dt05 = 0;
static volatile uint32_t periodCounter = 0;
static volatile double TimeStamp_imp34dt05a;

/* Private function prototypes -----------------------------------------------*/
void DFSDM_Filter_0_HalfComplete_Callback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
void DFSDM_Filter_0_Complete_Callback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
void IMP34DT05_DFSDM_FilterMspInit(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);

static void IMP34DT05_Thread(void const *argument);

static void IMP34DT05_DFSDM_Init(void);
static void IMP34DT05_DFSDM_DeInit(void);

static void Error_Handler(void);
static void Error_Handler(void)
{
  while (1);
}

/**
  * @brief IIS3DWB GPIO Initialization Function
  * @param None
  * @retval None
  */
void IMP34DT05_Peripheral_Init(void)
{
  IMP34DT05_DFSDM_Init();
}


/**
  * @brief IIS3DWB Threads Creation
  * @param None
  * @retval None
  */
void IMP34DT05_OS_Init(void)
{
  /* Data read complete semaphore initialization */
  dmicDreadyQueue_id = osMessageCreate(osMessageQ(dmicdreadyqueue), NULL);
  /* Thread definition: read data */
  osThreadDef(IMP34DT05_Acquisition_Thread, IMP34DT05_Thread, HSD_IMP34DT05_THREAD_PRIO, 1, configMINIMAL_STACK_SIZE);
  /* Start thread */
  IMP34DT05_Thread_Id = osThreadCreate(osThread(IMP34DT05_Acquisition_Thread), NULL);
}


static void IMP34DT05_Thread(void const *argument)
{
  (void) argument;

#if (configUSE_APPLICATION_TASK_TAG == 1 && defined(HSD_TASK_IMP34DT05_DEBUG_PIN))
  vTaskSetApplicationTaskTag(NULL, (TaskHookFunction_t)HSD_TASK_IMP34DT05_DEBUG_PIN);
#endif /* configUSE_APPLICATION_TASK_TAG == 1 && defined(HSD_TASK_IMP34DT05_DEBUG_PIN) */

#if (HSD_USE_DUMMY_DATA == 1)
  static uint16_t dummyDataCounter = 0;
#endif /* HSD_USE_DUMMY_DATA == 1 */

  osEvent evt;

  /* Suspend thread */
  osThreadSuspend(IMP34DT05_Thread_Id);

  for (;;)
  {
    if (IMP34DT05_Sensor_State == SM_SENSOR_STATE_INITIALIZING)
    {
      ts_imp34dt05 = 0;
      tim_value_old = 0;

      HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, (int32_t *)dmic_mem,
                                       ((uint32_t)IMP34DT05_Init_Param.ODR[0] / 1000) * IMP34DT05_MS * 2);
      IMP34DT05_Sensor_State = SM_SENSOR_STATE_RUNNING;
    }
    else if (IMP34DT05_Sensor_State == SM_SENSOR_STATE_RUNNING)
    {
      evt = osMessageGet(dmicDreadyQueue_id, osWaitForever);

      if (IMP34DT05_Sensor_State == SM_SENSOR_STATE_RUNNING) /* Change of state can happen while task blocked */
      {
        uint16_t idx = 0;
        void *data_ptr = evt.value.p;  /* void since it is independent from data format */

#if (HSD_USE_DUMMY_DATA != 1)
        static int32_t oldIn = 0;
        static int32_t oldOut = 0;
        int32_t *p32 = (int32_t *)data_ptr;
        int16_t *p16 = (int16_t *)data_ptr;
#else
        uint16_t *p16 = (uint16_t *)data_ptr;
#endif /* HSD_USE_DUMMY_DATA != 1 */

        for (idx = 0; idx < ((IMP34DT05_Init_Param.ODR[0] / 1000) * IMP34DT05_MS) ; idx++)
        {
#if (HSD_USE_DUMMY_DATA == 1)
          *p16++ = dummyDataCounter++;
#else
          *p16++ = oldOut = (0xFC * (oldOut + ((*p32) >> 11) - oldIn)) / 0xFF;
          oldIn = (*p32 ++) >> 11;
#endif /* HSD_USE_DUMMY_DATA == 1 */
        }

        IMP34DT05_Data_Ready(0, (uint8_t *)data_ptr, ((uint32_t)IMP34DT05_Init_Param.ODR[0] / 1000) * IMP34DT05_MS * 2,
                             TimeStamp_imp34dt05a);
      }
    }
    else if (IMP34DT05_Sensor_State == SM_SENSOR_STATE_SUSPENDING)
    {
#if (HSD_USE_DUMMY_DATA == 1)
      dummyDataCounter = 0;
#endif /* HSD_USE_DUMMY_DATA == 1 */
      HAL_DFSDM_FilterRegularStop_DMA(&hdfsdm1_filter0);
      IMP34DT05_Sensor_State = SM_SENSOR_STATE_SUSPENDED;
      osThreadSuspend(IMP34DT05_Thread_Id);
    }
  }
}


static void IMP34DT05_DFSDM_Init(void)
{
  IMP34DT05_DFSDM_FilterMspInit(&hdfsdm1_filter0);

  __HAL_DFSDM_CHANNEL_RESET_HANDLE_STATE(&hdfsdm1_channel5);
  hdfsdm1_channel5.Instance = DFSDM1_Channel5;
  hdfsdm1_channel5.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel5.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_AUDIO;
  hdfsdm1_channel5.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel5.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel5.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel5.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel5.Init.Awd.Oversampling = 10;
  hdfsdm1_channel5.Init.Offset = 0;
  hdfsdm1_channel5.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel5.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_FALLING;

  switch ((uint32_t)IMP34DT05_Init_Param.ODR[0])
  {
    case 8000 :
      hdfsdm1_channel5.Init.OutputClock.Divider = 4;
      hdfsdm1_channel5.Init.RightBitShift = 0x0E;
      break;
    case 16000 :
      hdfsdm1_channel5.Init.OutputClock.Divider = 4;
      hdfsdm1_channel5.Init.RightBitShift = 0x0A;
      break;
    case 32000 :
      hdfsdm1_channel5.Init.OutputClock.Divider = 4;
      hdfsdm1_channel5.Init.RightBitShift = 0x0D;
      break;
    case 48000 :
      hdfsdm1_channel5.Init.OutputClock.Divider = 4;
      hdfsdm1_channel5.Init.RightBitShift = 0x0A;
      break;
    default:
      hdfsdm1_channel5.Init.OutputClock.Divider = 4;
      hdfsdm1_channel5.Init.RightBitShift = 0x0A;
      break;
  }

  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel5) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_DFSDM_FILTER_RESET_HANDLE_STATE(&hdfsdm1_filter0);
  hdfsdm1_filter0.Instance = DFSDM1_Filter0;
  hdfsdm1_filter0.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter0.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter0.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm1_filter0.Init.InjectedParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter0.Init.InjectedParam.ScanMode = DISABLE;
  hdfsdm1_filter0.Init.InjectedParam.DmaMode = DISABLE;
  hdfsdm1_filter0.Init.InjectedParam.ExtTrigger     = DFSDM_FILTER_EXT_TRIG_TIM1_TRGO;
  hdfsdm1_filter0.Init.InjectedParam.ExtTriggerEdge = DFSDM_FILTER_EXT_TRIG_RISING_EDGE;
  hdfsdm1_filter0.Init.FilterParam.IntOversampling = 1;

  switch ((uint32_t)IMP34DT05_Init_Param.ODR[0])
  {
    case 8000 :
      hdfsdm1_filter0.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC4_ORDER;
      hdfsdm1_filter0.Init.FilterParam.Oversampling = 384;
      break;
    case 16000 :
      hdfsdm1_filter0.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC4_ORDER;
      hdfsdm1_filter0.Init.FilterParam.Oversampling = 192;
      break;
    case 32000 :
      hdfsdm1_filter0.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC5_ORDER;
      hdfsdm1_filter0.Init.FilterParam.Oversampling = 96;
      break;
    case 48000 :
      hdfsdm1_filter0.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC5_ORDER;
      hdfsdm1_filter0.Init.FilterParam.Oversampling = 64;
      break;
    default:
      hdfsdm1_filter0.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC5_ORDER;
      hdfsdm1_filter0.Init.FilterParam.Oversampling = 64;
      break;
  }

  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter0) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter0, DFSDM_CHANNEL_5, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_DFSDM_Filter_RegisterCallback(&hdfsdm1_filter0, HAL_DFSDM_FILTER_REGCONV_HALFCOMPLETE_CB_ID,
                                    DFSDM_Filter_0_HalfComplete_Callback);
  HAL_DFSDM_Filter_RegisterCallback(&hdfsdm1_filter0, HAL_DFSDM_FILTER_REGCONV_COMPLETE_CB_ID,
                                    DFSDM_Filter_0_Complete_Callback);
}

static void IMP34DT05_DFSDM_DeInit(void)
{
  if (HAL_DFSDM_FilterDeInit(&hdfsdm1_filter0) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_filter0.Instance = NULL;

  if (HAL_DFSDM_ChannelDeInit(&hdfsdm1_channel5) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_channel5.Instance = NULL;
}

void IMP34DT05_DFSDM_FilterMspInit(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  /* Enable DFSDM clock */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DFSDM_CLK_ENABLE();
  /* Enable the DMA clock */
  __HAL_RCC_DMA1_CLK_ENABLE();

  hdma_dfsdm1_flt0.Instance = IMP34DT05_DFSDM_RX_DMA_CHANNEL;
  hdma_dfsdm1_flt0.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_dfsdm1_flt0.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_dfsdm1_flt0.Init.MemInc = DMA_MINC_ENABLE;
  hdma_dfsdm1_flt0.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_dfsdm1_flt0.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_dfsdm1_flt0.Init.Mode = DMA_CIRCULAR;
  hdma_dfsdm1_flt0.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_dfsdm1_flt0.Init.Request = IMP34DT05_DFSDM_RX_DMA_REQUEST;

  /* Several peripheral DMA handle pointers point to the same DMA handle.
  Be aware that there is only one channel to perform all the requested DMAs. */
  __HAL_LINKDMA(hdfsdm_filter, hdmaReg, hdma_dfsdm1_flt0);
  /* Reset DMA handle state */
  __HAL_DMA_RESET_HANDLE_STATE(&hdma_dfsdm1_flt0);
  /* Configure the DMA Channel */
  if (HAL_DMA_Init(&hdma_dfsdm1_flt0) != HAL_OK)
  {
    Error_Handler();
  }
  /* DMA IRQ Channel configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);


  GPIO_InitTypeDef GPIO_InitStruct = {0};

  IMP34DT05_DFSDM_CLK_ENABLE();
  IMP34DT05_DFSDM_PDM_PIN_CLK_ENABLE();
  IMP34DT05_DFSDM_CLK_PIN_CLK_ENABLE();

  /**DFSDM1 GPIO Configuration
  PB6     ------> DFSDM1_DATIN5
  PE9     ------> DFSDM1_CKOUT
  */
  GPIO_InitStruct.Pin = IMP34DT05_DFSDM_PDM_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_DFSDM1;
  HAL_GPIO_Init(IMP34DT05_DFSDM_PDM_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = IMP34DT05_DFSDM_CLK_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_DFSDM1;
  HAL_GPIO_Init(IMP34DT05_DFSDM_CLK_GPIO_PORT, &GPIO_InitStruct);
}


uint32_t tickCounter = 0;
uint32_t callbackCounter = 0;

void DFSDM_Filter_0_HalfComplete_Callback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  tickCounter = HAL_GetTick();
  callbackCounter++;

  TimeStamp_imp34dt05a = SM_GetTimeStamp_fromISR();

  osMessagePut(dmicDreadyQueue_id, (uint32_t)(dmic_mem), osWaitForever);
}

void DFSDM_Filter_0_Complete_Callback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  tickCounter = HAL_GetTick();
  callbackCounter++;

  TimeStamp_imp34dt05a = SM_GetTimeStamp_fromISR();

  osMessagePut(dmicDreadyQueue_id, (uint32_t)(&dmic_mem[(((uint32_t)IMP34DT05_Init_Param.ODR[0]
                                                          / 1000) * IMP34DT05_MS)]), osWaitForever);
}


void IMP34DT05_updateConfig(void)
{
  COM_Sensor_t *pSensor = COM_GetSensor(IMP34DT05_Get_Id());

  if (pSensor->sensorStatus.subSensorStatus[0].ODR != IMP34DT05_Init_Param.ODR[0])
  {
    IMP34DT05_Init_Param.ODR[0] = pSensor->sensorStatus.subSensorStatus[0].ODR;
    IMP34DT05_DFSDM_DeInit();
    IMP34DT05_DFSDM_Init();
  }

  IMP34DT05_Init_Param.FS[0] = pSensor->sensorStatus.subSensorStatus[0].FS;
  IMP34DT05_Init_Param.subSensorActive[0] = pSensor->sensorStatus.subSensorStatus[0].isActive;
}


void IMP34DT05_Set_State(SM_Sensor_State_t state)
{
  IMP34DT05_Sensor_State = state;
}

void IMP34DT05_Start(void)
{
  IMP34DT05_Set_State(SM_SENSOR_STATE_INITIALIZING);
  osThreadResume(IMP34DT05_Thread_Id);
}

void IMP34DT05_Stop(void)
{
  IMP34DT05_Set_State(SM_SENSOR_STATE_SUSPENDING);
}

__weak void IMP34DT05_Data_Ready(uint8_t subSensorId, uint8_t *buf, uint16_t size, double timeStamp)
{

}

/**
  * @brief Get Sensor ID
  * @param None
  * @retval Sensor ID
  */
uint8_t IMP34DT05_Get_Id(void)
{
  return s_nIMP34DT05_id;
}

/**
  * @brief IMP34DT05 Sensor Initialization
  * @param None
  * @retval None
  */
uint8_t IMP34DT05_Create_Sensor(const SM_Init_Param_t *pxParams)
{
  COM_Sensor_t *pSensor;

  s_nIMP34DT05_id = COM_AddSensor();

  if (s_nIMP34DT05_id == -1)
  {
    return 1; /* error */
  }

  pSensor = COM_GetSensor(s_nIMP34DT05_id);

  /* SENSOR DESCRIPTOR */
  strcpy(pSensor->sensorDescriptor.name, "IMP34DT05");
  pSensor->sensorDescriptor.nSubSensors = 1;

  /* SUBSENSOR 0 DESCRIPTOR */
  pSensor->sensorDescriptor.subSensorDescriptor[0].id = 0;
  pSensor->sensorDescriptor.subSensorDescriptor[0].sensorType = COM_TYPE_MIC;
  pSensor->sensorDescriptor.subSensorDescriptor[0].dimensions = 1;
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].dimensionsLabel[0], "aud");
  pSensor->sensorDescriptor.subSensorDescriptor[0].dataType = DATA_TYPE_INT16;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[0] = 8000.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[1] = 16000.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[2] = 32000.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[3] = 48000.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[4] = COM_END_OF_LIST_FLOAT;
  pSensor->sensorDescriptor.subSensorDescriptor[0].samplesPerTimestamp[0] = 0;
  pSensor->sensorDescriptor.subSensorDescriptor[0].samplesPerTimestamp[1] = 1000;
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].unit, "Waveform");
  pSensor->sensorDescriptor.subSensorDescriptor[0].FS[0] = 122.5f;
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
    pSensor->sensorStatus.subSensorStatus[0].FS = 122.5f;
    pSensor->sensorStatus.subSensorStatus[0].ODR = 48000.0f;
  }
  pSensor->sensorStatus.subSensorStatus[0].sensitivity = 1.0f;
  pSensor->sensorStatus.subSensorStatus[0].measuredODR = 0.0f;
  pSensor->sensorStatus.subSensorStatus[0].initialOffset = 0.0f;
  pSensor->sensorStatus.subSensorStatus[0].samplesPerTimestamp = 1000;
  pSensor->sensorStatus.subSensorStatus[0].usbDataPacketSize = 4096;
  pSensor->sensorStatus.subSensorStatus[0].sdWriteBufferSize = WRITE_BUFFER_SIZE_IMP34DT05;
  pSensor->sensorStatus.subSensorStatus[0].comChannelNumber = -1;
  pSensor->sensorStatus.subSensorStatus[0].ucfLoaded = 0;

  IMP34DT05_Init_Param.ODR[0] = pSensor->sensorStatus.subSensorStatus[0].ODR;
  IMP34DT05_Init_Param.FS[0] = pSensor->sensorStatus.subSensorStatus[0].FS;
  IMP34DT05_Init_Param.subSensorActive[0] = pSensor->sensorStatus.subSensorStatus[0].isActive;

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


