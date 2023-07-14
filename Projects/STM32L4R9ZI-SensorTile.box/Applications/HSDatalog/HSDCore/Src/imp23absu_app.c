/**
  ******************************************************************************
  * @file    imp23absu_app.c
  * @author  SRA - MCD
  *
  *
  * @brief   This file provides a set of functions to handle imp23absu microphone
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
#include "imp23absu_app.h"
#include "main.h"
#include "sensors_manager.h"
#include "com_manager.h"
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define WRITE_BUFFER_SIZE_IMP23ABSU       (uint32_t)(32768 * 3)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static int32_t s_nIMP23ABSU_id = 0;

SM_Init_Param_t IMP23ABSU_Init_Param;
SM_Sensor_State_t IMP23ABSU_Sensor_State = SM_SENSOR_STATE_INITIALIZING;
static uint32_t amic_mem[((IMP23ABSU_MAX_SAMPLING_FREQUENCY / 1000) * IMP23ABSU_MS * 2)];

DFSDM_Filter_HandleTypeDef hdfsdm1_filter1;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel0;
DMA_HandleTypeDef hdma_dfsdm1_flt1;

osThreadId IMP23ABSU_Thread_Id;

osMessageQId amicDreadyQueue_id;
osMessageQDef(amicdreadyqueue, 1, int);

static volatile uint32_t tim_value = 0, tim_value_old = 0, period = 0;
static volatile uint64_t ts_imp23absu;
static volatile double TimeStamp_imp23absu;
/* Private function prototypes -----------------------------------------------*/

static void IMP23ABSU_ADC_Init(void);
static void IMP23ABSU_DFSDM_Init(void);
static void IMP23ABSU_DFSDM_DeInit(void);
void IMP23ABSU_DFSDM_FilterMspInit(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
void IMP23ABSU_ADC_MspInit(ADC_HandleTypeDef *hadc);

void DFSDM_Filter_1_HalfComplete_Callback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
void DFSDM_Filter_1_Complete_Callback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);

static void IMP23ABSU_Thread(void const *argument);

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
void IMP23ABSU_Peripheral_Init(void)
{
  IMP23ABSU_DFSDM_Init();
  IMP23ABSU_ADC_Init();
}


void IMP23ABSU_OS_Init(void)
{
  /* Thread definition */
  osThreadDef(IMP23ABSU_RD_USR_THREAD, IMP23ABSU_Thread, HSD_IMP23ABSU_THREAD_PRIO, 1, configMINIMAL_STACK_SIZE);
  /* Start thread */
  IMP23ABSU_Thread_Id = osThreadCreate(osThread(IMP23ABSU_RD_USR_THREAD), NULL);

  amicDreadyQueue_id = osMessageCreate(osMessageQ(amicdreadyqueue), NULL);
}


static void IMP23ABSU_Thread(void const *argument)
{
  (void) argument;

#if (configUSE_APPLICATION_TASK_TAG == 1 && defined(HSD_TASK_IMP23ABSU_DEBUG_PIN))
  vTaskSetApplicationTaskTag(NULL, (TaskHookFunction_t)HSD_TASK_IMP23ABSU_DEBUG_PIN);
#endif /* configUSE_APPLICATION_TASK_TAG == 1 && defined(HSD_TASK_IMP23ABSU_DEBUG_PIN) */

#if (HSD_USE_DUMMY_DATA == 1)
  static uint16_t dummyDataCounter = 0;
#endif /* HSD_USE_DUMMY_DATA == 1 */

  osEvent evt;

  /* Suspend thread */
  osThreadSuspend(IMP23ABSU_Thread_Id);

  for (;;)
  {
    if (IMP23ABSU_Sensor_State == SM_SENSOR_STATE_INITIALIZING)
    {
      tim_value_old = 0;
      HAL_ADC_Start(&ADC1_Handle);

      HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter1, (int32_t *)amic_mem,
                                       ((uint32_t)IMP23ABSU_Init_Param.ODR[0] / 1000) * IMP23ABSU_MS * 2);
      IMP23ABSU_Sensor_State = SM_SENSOR_STATE_RUNNING;
    }
    else if (IMP23ABSU_Sensor_State == SM_SENSOR_STATE_RUNNING)
    {
      evt = osMessageGet(amicDreadyQueue_id, osWaitForever);

      if (IMP23ABSU_Sensor_State == SM_SENSOR_STATE_RUNNING) /* Change of state can happen while task blocked */
      {
        void *data_ptr = evt.value.p;  /* void since it is independent from data format*/
        /* Do something */
        uint16_t idx = 0;

#if (HSD_USE_DUMMY_DATA != 1)
        static int32_t oldIn = 0;
        static int32_t oldOut = 0;
        int32_t *p32 = (int32_t *)data_ptr;
#endif /* HSD_USE_DUMMY_DATA != 1 */
        int16_t *p16 = (int16_t *)data_ptr;

        for (idx = 0; idx < (((uint32_t)IMP23ABSU_Init_Param.ODR[0] / 1000) * IMP23ABSU_MS) ; idx++)
        {
#if (HSD_USE_DUMMY_DATA == 1)
          *p16++ = dummyDataCounter++;
#else
          *p16++ = oldOut = (0xFC * (oldOut + ((*p32) >> 12) - oldIn)) / 0xFF;
          oldIn = (*p32 ++) >> 12;
#endif /* HSD_USE_DUMMY_DATA == 1 */
        }

        IMP23ABSU_Data_Ready(0, (uint8_t *)data_ptr, ((uint32_t)IMP23ABSU_Init_Param.ODR[0] / 1000) * IMP23ABSU_MS * 2,
                             TimeStamp_imp23absu);
      }
    }
    else if (IMP23ABSU_Sensor_State == SM_SENSOR_STATE_SUSPENDING)
    {
#if (HSD_USE_DUMMY_DATA == 1)
      dummyDataCounter = 0;
#endif /* HSD_USE_DUMMY_DATA == 1 */
      HAL_ADC_Stop(&ADC1_Handle);
      IMP23ABSU_Sensor_State = SM_SENSOR_STATE_SUSPENDED;
      ts_imp23absu = 0;
      osThreadSuspend(IMP23ABSU_Thread_Id);
    }
  }
}


/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void IMP23ABSU_ADC_Init(void)
{
  HAL_ADC_RegisterCallback(&ADC1_Handle, HAL_ADC_MSPINIT_CB_ID, IMP23ABSU_ADC_MspInit);

  ADC_ChannelConfTypeDef sConfig = {0};

  /* Enable ADC1 if not already initialized by Battery Charger */
  (void)BSP_ADC1_Initialization(ADC1_FOR_AUDIO);

  /**Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_1;
  sConfig.Offset = 0x800;
  if (HAL_ADC_ConfigChannel(&ADC1_Handle, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}


static void IMP23ABSU_DFSDM_Init(void)
{
  IMP23ABSU_DFSDM_FilterMspInit(&hdfsdm1_filter1);

  __HAL_DFSDM_CHANNEL_RESET_HANDLE_STATE(&hdfsdm1_channel0);
  hdfsdm1_channel0.Instance = DFSDM1_Channel0;
  hdfsdm1_channel0.Init.OutputClock.Activation = DISABLE;
  hdfsdm1_channel0.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_AUDIO;
  hdfsdm1_channel0.Init.OutputClock.Divider = 1;
  hdfsdm1_channel0.Init.Input.Multiplexer = DFSDM_CHANNEL_ADC_OUTPUT;
  hdfsdm1_channel0.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel0.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel0.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel0.Init.Awd.Oversampling = 10;
  hdfsdm1_channel0.Init.Offset = 0x00;
  hdfsdm1_channel0.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel0.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_FALLING;

  switch ((uint32_t)IMP23ABSU_Init_Param.ODR[0])
  {
    case 8000 :
      hdfsdm1_channel0.Init.RightBitShift = 0x09;
      break;
    case 16000 :
      hdfsdm1_channel0.Init.RightBitShift = 0x07;
      break;
    case 32000 :
      hdfsdm1_channel0.Init.RightBitShift = 0x05;
      break;
    case 48000 :
      hdfsdm1_channel0.Init.RightBitShift = 0x0A;
      break;
    case 96000 :
      hdfsdm1_channel0.Init.RightBitShift = 0x07;
      break;
    case 192000 :
      hdfsdm1_channel0.Init.RightBitShift = 0x08;
      break;
    default :
      hdfsdm1_channel0.Init.RightBitShift = 0x08;
      break;
  }

  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel0) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_DFSDM_FILTER_RESET_HANDLE_STATE(&hdfsdm1_filter1);
  hdfsdm1_filter1.Instance = DFSDM1_Filter1;
  hdfsdm1_filter1.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter1.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter1.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm1_filter1.Init.InjectedParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter1.Init.InjectedParam.ScanMode = DISABLE;
  hdfsdm1_filter1.Init.InjectedParam.DmaMode = DISABLE;
  hdfsdm1_filter1.Init.InjectedParam.ExtTrigger = DFSDM_FILTER_EXT_TRIG_TIM1_TRGO;
  hdfsdm1_filter1.Init.InjectedParam.ExtTriggerEdge = DFSDM_FILTER_EXT_TRIG_RISING_EDGE;
  hdfsdm1_filter1.Init.FilterParam.IntOversampling = 1;

  switch ((uint32_t)IMP23ABSU_Init_Param.ODR[0])
  {
    case 8000 :
      hdfsdm1_filter1.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC2_ORDER;
      hdfsdm1_filter1.Init.FilterParam.Oversampling = 384;
      break;
    case 16000 :
      hdfsdm1_filter1.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC2_ORDER;
      hdfsdm1_filter1.Init.FilterParam.Oversampling = 192;
      break;
    case 32000 :
      hdfsdm1_filter1.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC2_ORDER;
      hdfsdm1_filter1.Init.FilterParam.Oversampling = 96;
      break;
    case 48000 :
      hdfsdm1_filter1.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC3_ORDER;
      hdfsdm1_filter1.Init.FilterParam.Oversampling = 64;
      break;
    case 96000 :
      hdfsdm1_filter1.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC3_ORDER;
      hdfsdm1_filter1.Init.FilterParam.Oversampling = 32;
      break;
    case 192000 :
      hdfsdm1_filter1.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC4_ORDER;
      hdfsdm1_filter1.Init.FilterParam.Oversampling = 16;
      break;
    default :
      hdfsdm1_filter1.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC4_ORDER;
      hdfsdm1_filter1.Init.FilterParam.Oversampling = 16;
      break;
  }

  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter1, DFSDM_CHANNEL_0, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_DFSDM_Filter_RegisterCallback(&hdfsdm1_filter1, HAL_DFSDM_FILTER_REGCONV_HALFCOMPLETE_CB_ID,
                                    DFSDM_Filter_1_HalfComplete_Callback);
  HAL_DFSDM_Filter_RegisterCallback(&hdfsdm1_filter1, HAL_DFSDM_FILTER_REGCONV_COMPLETE_CB_ID,
                                    DFSDM_Filter_1_Complete_Callback);
}


static void IMP23ABSU_DFSDM_DeInit(void)
{
  if (HAL_DFSDM_FilterDeInit(&hdfsdm1_filter1) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_filter1.Instance = NULL;

  if (HAL_DFSDM_ChannelDeInit(&hdfsdm1_channel0) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_channel0.Instance = NULL;
}


void IMP23ABSU_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (hadc->Instance == ADC1)
  {
    /* Peripheral clock enable */
    __HAL_RCC_ADC_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PC0     ------> ADC1_IN1
    PC1     ------> ADC1_IN2
      */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  }
}


void IMP23ABSU_DFSDM_FilterMspInit(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  /* Peripheral clock enable */
  __HAL_RCC_DFSDM1_CLK_ENABLE();
  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  hdma_dfsdm1_flt1.Instance = DMA1_Channel6;
  hdma_dfsdm1_flt1.Init.Request = DMA_REQUEST_DFSDM1_FLT1;
  hdma_dfsdm1_flt1.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_dfsdm1_flt1.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_dfsdm1_flt1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_dfsdm1_flt1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_dfsdm1_flt1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_dfsdm1_flt1.Init.Mode = DMA_CIRCULAR;
  hdma_dfsdm1_flt1.Init.Priority = DMA_PRIORITY_HIGH;

  /* Several peripheral DMA handle pointers point to the same DMA handle.
  Be aware that there is only one channel to perform all the requested DMAs. */
  __HAL_LINKDMA(hdfsdm_filter, hdmaReg, hdma_dfsdm1_flt1);
  /* Reset DMA handle state */
  __HAL_DMA_RESET_HANDLE_STATE(&hdma_dfsdm1_flt1);
  /* Configure the DMA Channel */
  if (HAL_DMA_Init(&hdma_dfsdm1_flt1) != HAL_OK)
  {
    Error_Handler();
  }

  /* DMA1_Channel_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
}


/**
  * @brief  Regular conversion complete callback.
  * @note   This function performs an HP filter in order to remove DC offset and arranges PCM data
  *         following the standard PCM format.
  * @param  hdfsdm_filter : DFSDM filter handle.
  * @retval None
  */
void DFSDM_Filter_1_Complete_Callback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  TimeStamp_imp23absu = SM_GetTimeStamp_fromISR();

  if (hdfsdm_filter == &hdfsdm1_filter1) /* Analog Mic */
  {
    osMessagePut(amicDreadyQueue_id, (uint32_t)(&amic_mem[(((uint32_t)IMP23ABSU_Init_Param.ODR[0]
                                                            / 1000) * IMP23ABSU_MS)]), osWaitForever);
  }
}

/**
  * @brief  Half regular conversion complete callback.
  * @param  hdfsdm_filter : DFSDM filter handle.
  * @retval None
  */
void DFSDM_Filter_1_HalfComplete_Callback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  TimeStamp_imp23absu = SM_GetTimeStamp_fromISR();

  if (hdfsdm_filter == &hdfsdm1_filter1) /* Analog Mic */
  {
    osMessagePut(amicDreadyQueue_id, (uint32_t)(amic_mem), osWaitForever);
  }
}


void IMP23ABSU_updateConfig(void)
{
  COM_Sensor_t *pSensor = COM_GetSensor(IMP23ABSU_Get_Id());

  if (pSensor->sensorStatus.subSensorStatus[0].ODR != IMP23ABSU_Init_Param.ODR[0])
  {
    IMP23ABSU_Init_Param.ODR[0] = pSensor->sensorStatus.subSensorStatus[0].ODR;
    IMP23ABSU_DFSDM_DeInit();
    IMP23ABSU_DFSDM_Init();
  }

  IMP23ABSU_Init_Param.FS[0] = pSensor->sensorStatus.subSensorStatus[0].FS;
  IMP23ABSU_Init_Param.subSensorActive[0] = pSensor->sensorStatus.subSensorStatus[0].isActive;
}


void IMP23ABSU_StartAcquisition(void)
{
  HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter1, (int32_t *)amic_mem,
                                   ((uint32_t)IMP23ABSU_Init_Param.ODR[0] / 1000) * IMP23ABSU_MS * 2);
}

void IMP23ABSU_StopAcquisition(void)
{
  HAL_DFSDM_FilterRegularStop_DMA(&hdfsdm1_filter1);
}

void IMP23ABSU_PauseAcquisition(void)
{
  HAL_DFSDM_FilterRegularStop_DMA(&hdfsdm1_filter1);
}

void IMP23ABSU_ResumeAcquisition(void)
{
  HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter1, (int32_t *)amic_mem,
                                   ((uint32_t)IMP23ABSU_Init_Param.ODR[0] / 1000) * IMP23ABSU_MS * 2);
}

void IMP23ABSU_Set_State(SM_Sensor_State_t state)
{
  IMP23ABSU_Sensor_State = state;
}

void IMP23ABSU_Start(void)
{
  IMP23ABSU_Set_State(SM_SENSOR_STATE_INITIALIZING);
  osThreadResume(IMP23ABSU_Thread_Id);
}

void IMP23ABSU_Stop(void)
{
  IMP23ABSU_Set_State(SM_SENSOR_STATE_SUSPENDING);
}

__weak void IMP23ABSU_Data_Ready(uint8_t subSensorId, uint8_t *buf, uint16_t size, double timeStamp)
{

}

/**
  * @brief Get Sensor ID
  * @param None
  * @retval Sensor ID
  */
uint8_t IMP23ABSU_Get_Id(void)
{
  return s_nIMP23ABSU_id;
}

/**
  * @brief IMP23ABSU Sensor Initialization
  * @param None
  * @retval None
  */
uint8_t IMP23ABSU_Create_Sensor(const SM_Init_Param_t *pxParams)
{
  COM_Sensor_t *pSensor;

  s_nIMP23ABSU_id = COM_AddSensor();

  if (s_nIMP23ABSU_id == -1)
  {
    return 1; /* error */
  }

  pSensor = COM_GetSensor(s_nIMP23ABSU_id);

  /* SENSOR DESCRIPTOR */
  strcpy(pSensor->sensorDescriptor.name, "IMP23ABSU");
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
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[4] = 96000.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[5] = 192000.0f;
  pSensor->sensorDescriptor.subSensorDescriptor[0].ODR[6] = COM_END_OF_LIST_FLOAT;
  pSensor->sensorDescriptor.subSensorDescriptor[0].samplesPerTimestamp[0] = 0;
  pSensor->sensorDescriptor.subSensorDescriptor[0].samplesPerTimestamp[1] = 1000;
  strcpy(pSensor->sensorDescriptor.subSensorDescriptor[0].unit, "Waveform");
  pSensor->sensorDescriptor.subSensorDescriptor[0].FS[0] = 130.0f;
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
    pSensor->sensorStatus.subSensorStatus[0].FS = 130.0f;
    pSensor->sensorStatus.subSensorStatus[0].ODR = 192000.0f;
  }
  pSensor->sensorStatus.subSensorStatus[0].sensitivity = 1.0f;
  pSensor->sensorStatus.subSensorStatus[0].measuredODR = 0.0f;
  pSensor->sensorStatus.subSensorStatus[0].initialOffset = 0.0f;
  pSensor->sensorStatus.subSensorStatus[0].samplesPerTimestamp = 1000;
  pSensor->sensorStatus.subSensorStatus[0].usbDataPacketSize = 4096;
  pSensor->sensorStatus.subSensorStatus[0].sdWriteBufferSize = WRITE_BUFFER_SIZE_IMP23ABSU;
  pSensor->sensorStatus.subSensorStatus[0].comChannelNumber = -1;
  pSensor->sensorStatus.subSensorStatus[0].ucfLoaded = 0;

  IMP23ABSU_Init_Param.ODR[0] = pSensor->sensorStatus.subSensorStatus[0].ODR;
  IMP23ABSU_Init_Param.FS[0] = pSensor->sensorStatus.subSensorStatus[0].FS;
  IMP23ABSU_Init_Param.subSensorActive[0] = pSensor->sensorStatus.subSensorStatus[0].isActive;

  return 0;
}


