/**
  ******************************************************************************
  * @file    sensors_manager.c
  * @brief   This file provides a set of functions to handle the sensors
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
#include "sensors_manager.h"
#include "main.h"
#include "com_manager.h"
#include "HSD_tags.h"

#include "mp23abs1_app.h"
#include "lsm6dsox_app.h"
#include "lis3dhh_app.h"
#include "lis2mdl_app.h"
#include "lis2dw12_app.h"
#include "hts221_app.h"
#include "lps22hh_app.h"
#include "stts751_app.h"

/* Private includes ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/

typedef struct SM_ThreadParameters_t
{
  osSemaphoreId *comThreadSem_id;
  osMessageQId *comReqQueue_id;
  osPoolId *comPool_id;
  void *hcom;
} SM_ThreadParameters_t;

/* Private define ------------------------------------------------------------*/
#define SM_I2C_TIMEOUT              ( 1000 )
#define SM_SPI_TIMEOUT              ( 1000 )
#define SM_TS_UPDATE_PERIOD_S     (10)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi1_rx;

DMA_HandleTypeDef hdma_spi3_tx;
DMA_HandleTypeDef hdma_spi3_rx;

DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

DMA_HandleTypeDef hdma_i2c3_rx;
DMA_HandleTypeDef hdma_i2c3_tx;

TIM_HandleTypeDef htim5;

osSemaphoreId spi1ThreadSem_id;
osSemaphoreDef(spi1ThreadSem);

osSemaphoreId spi3ThreadSem_id;
osSemaphoreDef(spi3ThreadSem);

osSemaphoreId i2c1ThreadSem_id;
osSemaphoreDef(i2c1ThreadSem);

osSemaphoreId i2c3ThreadSem_id;
osSemaphoreDef(i2c3ThreadSem);

osMessageQId spi1ReqQueue_id;
osMessageQDef(spi1reqqueue, 10, int);

osMessageQId spi3ReqQueue_id;
osMessageQDef(spi3reqqueue, 10, int);

osMessageQId i2c1ReqQueue_id;
osMessageQDef(i2c1reqqueue, 10, int);

osMessageQId i2c3ReqQueue_id;
osMessageQDef(i2c3reqqueue, 10, int);

osPoolId spi1Pool_id;
osPoolDef(spi1Pool, 100, SM_Message_t);

osPoolId spi3Pool_id;
osPoolDef(spi3Pool, 100, SM_Message_t);

osPoolId i2c1Pool_id;
osPoolDef(i2c1Pool, 100, SM_Message_t);

osPoolId i2c3Pool_id;
osPoolDef(i2c3Pool, 100, SM_Message_t);

SM_ThreadParameters_t i2c1ThreadParams;
SM_ThreadParameters_t i2c3ThreadParams;
SM_ThreadParameters_t spi1ThreadParams;
SM_ThreadParameters_t spi3ThreadParams;

osThreadId spi1ThreadId;
osThreadId spi3ThreadId;

osThreadId i2c1ThreadId;
osThreadId i2c3ThreadId;

uint32_t reg_after_release = 0;
static int32_t errors = 0;

static volatile uint64_t SM_TimeStamp = 0; /* Sensor Manager global timestamp */

/* Private function prototypes -----------------------------------------------*/
static void SM_DMA_Init(void);

static void SM_I2C1_Init(void);
static void SM_I2C3_Init(void);

static void SM_I2C_MspInit(I2C_HandleTypeDef *hi2c);
static void SM_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);
static void SM_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c);
static void SM_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);

static void SM_SPI1_Init(void);
static void SM_SPI3_Init(void);

static void SM_SPI_MspInit(SPI_HandleTypeDef *hspi);
static void SM_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);

static void SM_TIM_Init(void);
static void SM_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

static void i2c_Thread(void const *argument);
static void spi_Thread(void const *argument);

#if( LIS2MDL_COM_MODE == LIS2MDL_COM_SPI_3_WIRE )

static void spi3_Thread(void const *argument);

static void SM_SPI_3_WIRE_READ(SPI_HandleTypeDef *hspi, uint16_t Reg, uint8_t *pData, uint16_t Size);
static void SM_SPI_3_WIRE_WRITE(SPI_HandleTypeDef *hspi, uint16_t Reg, uint8_t *pData, uint16_t Size);

#endif /* ( LIS2MDL_COM_MODE == LIS2MDL_COM_SPI_3_WIRE ) */

/******************************************************************************/

/**
  * @brief  Sensor manager peripherals Initialization Function
  * @param  None
  * @retval None
  * @note   None
  */
void SM_Peripheral_Init(void)
{
  SM_DMA_Init();

  SM_I2C1_Init();
  SM_I2C3_Init();

  SM_SPI1_Init();
  SM_SPI3_Init();

  SM_TIM_Init();
}

/**
  * @brief  Sensor manager SPI1 Initialization Function
  * @param None
  * @retval None
  * @note   Sets callbacks to the MSP
  */
static void SM_SPI1_Init(void)
{
  /* SPI3 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; /*SPI running @ 10 MHz *//*stwin*/
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;

  /* Register MSP Callback */
  HAL_SPI_RegisterCallback(&hspi1, HAL_SPI_MSPINIT_CB_ID, SM_SPI_MspInit);

  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    SM_Error_Handler();
  }

  /* Register SPI DMA complete Callback */
  HAL_SPI_RegisterCallback(&hspi1, HAL_SPI_TX_RX_COMPLETE_CB_ID, SM_SPI_TxRxCpltCallback);
}

#if( LIS2MDL_COM_MODE == LIS2MDL_COM_SPI_4_WIRE )

/**
  * @brief  Sensor manager SPI3 Initialization Function
  * @param  None
  * @retval None
  * @note   Sets callbacks to the MSP
  */
static void SM_SPI3_Init(void)
{
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; /*SPI running @ 10 MHz */
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;

  /* Register MSP Callback */
  HAL_SPI_RegisterCallback(&hspi3, HAL_SPI_MSPINIT_CB_ID, SM_SPI_MspInit);

  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    SM_Error_Handler();
  }

  /* Register SPI DMA complete Callback */
  HAL_SPI_RegisterCallback(&hspi3, HAL_SPI_TX_RX_COMPLETE_CB_ID, SM_SPI_TxRxCpltCallback);
}

#elif( LIS2MDL_COM_MODE == LIS2MDL_COM_SPI_3_WIRE )

/**
  * @brief  Sensor manager SPI3 Initialization Function
  * @param  None
  * @retval None
  * @note   Sets callbacks to the MSP
  */
static void SM_SPI3_Init(void)
{
  /* SPI3 parameter configuration*/
  hspi3.Instance               = SPI3;
  hspi3.Init.Mode              = SPI_MODE_MASTER;
  hspi3.Init.Direction         = SPI_DIRECTION_1LINE;
  hspi3.Init.DataSize          = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity       = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase          = SPI_PHASE_1EDGE;
  hspi3.Init.NSS               = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; /*SPI running @ 10 MHz */ /*stwin*/
  hspi3.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode            = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial     = 7;
  hspi3.Init.CRCLength         = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;

  /* Register MSP Callback */
  HAL_SPI_RegisterCallback(&hspi3, HAL_SPI_MSPINIT_CB_ID, SM_SPI_MspInit);

  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    SM_Error_Handler();
  }

  /* Register SPI DMA complete Callback */
  HAL_SPI_RegisterCallback(&hspi3, HAL_SPI_TX_RX_COMPLETE_CB_ID, SM_SPI_TxRxCpltCallback);

  SPI_1LINE_TX(&hspi3);
  __HAL_SPI_ENABLE(&hspi3);
}

#endif /* ( LIS2MDL_COM_MODE == LIS2MDL_COM_SPI_4_WIRE ) */

/**
  * @brief  Sensor Manager I2C1 Initialization Function
  * @param  None
  * @retval None
  * @note   Sets callbacks to the MSP
  */
static void SM_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00B03FDB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  /* Register MSP Callback */
  HAL_I2C_RegisterCallback(&hi2c1, HAL_I2C_MSPINIT_CB_ID, SM_I2C_MspInit);

  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    SM_Error_Handler();
  }
  /** Configure Analog filter */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    SM_Error_Handler();
  }
  /** Configure Digital filter */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    SM_Error_Handler();
  }

  HAL_I2C_RegisterCallback(&hi2c1, HAL_I2C_MEM_RX_COMPLETE_CB_ID, SM_I2C_MemRxCpltCallback);
  HAL_I2C_RegisterCallback(&hi2c1, HAL_I2C_MEM_TX_COMPLETE_CB_ID, SM_I2C_MemTxCpltCallback);
  HAL_I2C_RegisterCallback(&hi2c1, HAL_I2C_ERROR_CB_ID, SM_I2C_ErrorCallback);
}

/**
  * @brief  Sensor Manager I2C3 Initialization Function
  * @param  None
  * @retval None
  * @note   Sets callbacks to the MSP
  */
static void SM_I2C3_Init(void)
{
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00B03FDB;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  /* Register MSP Callback */
  HAL_I2C_RegisterCallback(&hi2c3, HAL_I2C_MSPINIT_CB_ID, SM_I2C_MspInit);

  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    SM_Error_Handler();
  }
  /** Configure Analog filter */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    SM_Error_Handler();
  }
  /** Configure Digital filter */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    SM_Error_Handler();
  }

  HAL_I2C_RegisterCallback(&hi2c3, HAL_I2C_MEM_RX_COMPLETE_CB_ID, SM_I2C_MemRxCpltCallback);
  HAL_I2C_RegisterCallback(&hi2c3, HAL_I2C_MEM_TX_COMPLETE_CB_ID, SM_I2C_MemTxCpltCallback);
  HAL_I2C_RegisterCallback(&hi2c3, HAL_I2C_ERROR_CB_ID, SM_I2C_ErrorCallback);
}

void SM_Error_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief  SPI thread: it waits on the SPI request queue, performs SPI transactions in non blocking mode and unlocks
  *         the thread which made the request at the end of the read.
  * @param  pvParams : required resources for the thread.
  * @retval None
  */
static void spi_Thread(void const *argument)
{
  SM_ThreadParameters_t *pvParams = (SM_ThreadParameters_t *) argument;

  osEvent evt;

  for (;;)
  {
    evt = osMessageGet(*(pvParams->comReqQueue_id), osWaitForever);

    SM_Message_t *msg = evt.value.p;

    /**
      * SPI read is controlled by asserting the MSB of the register address.
      * This logic can/should be pulled into the individual sensor driver as it
      * is device-dependent.
      *
      **/
    if (msg->isRead != 0)
    {
      msg->regAddr |= 0x80;
    }

    HAL_GPIO_WritePin(((sensor_handle_t *) msg->sensorHandler)->GPIOx,
                      ((sensor_handle_t *) msg->sensorHandler)->GPIO_Pin, GPIO_PIN_RESET);

    HAL_SPI_Transmit((SPI_HandleTypeDef *) pvParams->hcom, &msg->regAddr, 1, 1000);
    HAL_SPI_TransmitReceive_DMA((SPI_HandleTypeDef *) pvParams->hcom, msg->dataPtr, msg->dataPtr, msg->readSize);

    osSemaphoreWait(*(pvParams->comThreadSem_id), osWaitForever);

    HAL_GPIO_WritePin(((sensor_handle_t *) msg->sensorHandler)->GPIOx,
                      ((sensor_handle_t *) msg->sensorHandler)->GPIO_Pin, GPIO_PIN_SET);

    osSemaphoreId *sem = ((sensor_handle_t *) msg->sensorHandler)->sem;
    osPoolFree(*(pvParams->comPool_id), msg);
    osSemaphoreRelease(*sem);
  }
}

#if( LIS2MDL_COM_MODE == LIS2MDL_COM_SPI_3_WIRE )

/**
  * @brief  SPI thread: it waits on the SPI request queue, performs SPI transactions in non blocking mode and unlocks
  *         the thread which made the request at the end of the read.
  * @param  pvParams : required resources for the thread.
  * @retval None
  */
static void spi3_Thread(void const *argument)
{
  SM_ThreadParameters_t *pvParams = (SM_ThreadParameters_t *) argument;

  osEvent evt;

  for (; ;)
  {
    evt = osMessageGet(*(pvParams->comReqQueue_id), osWaitForever);

    SM_Message_t *msg = evt.value.p;

    HAL_GPIO_WritePin(((sensor_handle_t *) msg->sensorHandler)->GPIOx,
                      ((sensor_handle_t *) msg->sensorHandler)->GPIO_Pin,
                      GPIO_PIN_RESET);

    if (msg->isRead != 0)
    {
      /**
        * SPI read is controlled by asserting the MSB of the register address.
        * This logic can/should be pulled into the individual sensor driver as
        * it is device-dependent.
        *
        **/
      SM_SPI_3_WIRE_READ((SPI_HandleTypeDef *) pvParams->hcom,
                         (msg->regAddr | 0x80),
                         msg->dataPtr,
                         msg->readSize);
    }
    else
    {
      SM_SPI_3_WIRE_WRITE((SPI_HandleTypeDef *) pvParams->hcom,
                          msg->regAddr,
                          msg->dataPtr,
                          msg->readSize);
    }

    HAL_GPIO_WritePin(((sensor_handle_t *) msg->sensorHandler)->GPIOx,
                      ((sensor_handle_t *) msg->sensorHandler)->GPIO_Pin,
                      GPIO_PIN_SET);

    osSemaphoreId *sem = ((sensor_handle_t *) msg->sensorHandler)->sem;
    osPoolFree(*(pvParams->comPool_id), msg);
    osSemaphoreRelease(*sem);
  }
}

#endif /* ( LIS2MDL_COM_MODE == LIS2MDL_COM_SPI_3_WIRE ) */

/**
  * @brief  I2C thread: it waits on the I2C request queue, performs I2C transactions in non blocking mode and
  *         unlocks the thread which made the request at the end of the read.
  * @param  pvParams : required resources for the thread.
  * @retval None
  */
static void i2c_Thread(void const *argument)
{
  SM_ThreadParameters_t *pvParams = (SM_ThreadParameters_t *) argument;

  osEvent evt;

  for (;;)
  {
    evt = osMessageGet(*(pvParams->comReqQueue_id), osWaitForever);
    uint8_t autoInc = 0;

    SM_Message_t *msg = evt.value.p;

    /**
      * HTS221 Auto-Increment is controlled by asserting the MSB of the register
      * address. This logic can/should be pulled into the individual sensor
      * driver as it is device-dependent.
      *
      **/
    if ((msg->sensorHandler->WhoAmI == 0xBCU) && (msg->readSize > 1))
    {
      autoInc = 0x80;
    }

    if (msg->isRead)
    {
      HAL_I2C_Mem_Read_DMA((I2C_HandleTypeDef *) pvParams->hcom, ((sensor_handle_t *) msg->sensorHandler)->I2C_address,
                           (uint16_t)(msg->regAddr | autoInc),
                           I2C_MEMADD_SIZE_8BIT,
                           msg->dataPtr, msg->readSize);
    }
    else
    {
      HAL_I2C_Mem_Write_DMA((I2C_HandleTypeDef *) pvParams->hcom, ((sensor_handle_t *) msg->sensorHandler)->I2C_address,
                            (uint16_t)(msg->regAddr | autoInc),
                            I2C_MEMADD_SIZE_8BIT,
                            msg->dataPtr, msg->readSize);
    }

    osSemaphoreWait(*(pvParams->comThreadSem_id), osWaitForever);

    osSemaphoreId *sem = ((sensor_handle_t *) msg->sensorHandler)->sem;
    osPoolFree(*(pvParams->comPool_id), msg);
    osSemaphoreRelease(*sem);
  }
}

/**
  * @brief  SPI read function: it adds a request on the SPI read queue (which will be handled by the SPI thread)
  * @param  handle : handle to a sensor context
  *         reg    : register of sensor to read
  *         data   : pointer to the read destination memory
  *         len    : number of bytes to read
  * @note   When the function is used and linked to the sensor context, all the calls made by the
  *         PID driver will result in a call to this function. If this is the case, be sure to make
  *         all the calls to the PID driver functions from a freeRTOS thread
  * @retval None
  */
int32_t SM_SPI1_Read_Os(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
  SM_Message_t *msg;

  msg = osPoolAlloc(spi1Pool_id);

  msg->isRead = 1;
  msg->sensorHandler = handle;
  msg->regAddr = reg;
  msg->readSize = len;
  msg->dataPtr = data;

  osMessagePut(spi1ReqQueue_id, (uint32_t) msg, osWaitForever);

  osSemaphoreWait(*(((sensor_handle_t *) handle)->sem), osWaitForever);

  return 0;
}

/**
  * @brief  SPI write function: it adds a request on the SPI write queue (which will be handled by the SPI thread)
  * @param  handle : handle to a sensor context
  *         reg    : register of sensor to read
  *         data   : pointer to the read destination memory
  *         len    : number of bytes to read
  * @note   When the function is used and linked to the sensor context, all the calls made by the
  *         PID driver will result in a call to this function. If this is the case, be sure to make
  *         all the calls to the PID driver functions from a freeRTOS thread
  * @retval None
  */
int32_t SM_SPI1_Write_Os(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
  SM_Message_t *msg;

  msg = osPoolAlloc(spi1Pool_id);

  msg->isRead = 0;
  msg->sensorHandler = handle;
  msg->regAddr = reg;
  msg->readSize = len;
  msg->dataPtr = data;

  osMessagePut(spi1ReqQueue_id, (uint32_t) msg, osWaitForever);

  osSemaphoreWait(*(((sensor_handle_t *) handle)->sem), osWaitForever);

  return 0;
}
/**
  * @brief  SPI read function: it adds a request on the SPI read queue (which will be handled by the SPI read thread)
  * @param  argument not used
  * @note   When the function is used and linked to the sensor context, all the calls made by the
  *         PID driver will result in a call to this function. If this is the case, be sure to make
  *         all the calls to the PID driver functions from a freeRTOS thread
  * @retval None
  */
int32_t SM_SPI3_Read_Os(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
  SM_Message_t *msg;

  msg = osPoolAlloc(spi3Pool_id);

  msg->isRead = 1;
  msg->sensorHandler = handle;
  msg->regAddr = reg;
  msg->readSize = len;
  msg->dataPtr = data;

  osMessagePut(spi3ReqQueue_id, (uint32_t) msg, osWaitForever);

  osSemaphoreWait(*(((sensor_handle_t *) handle)->sem), osWaitForever);

  return 0;
}

int32_t SM_SPI3_Write_Os(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
  SM_Message_t *msg;

  msg = osPoolAlloc(spi3Pool_id);

  msg->isRead = 0;
  msg->sensorHandler = handle;
  msg->regAddr = reg;
  msg->readSize = len;
  msg->dataPtr = data;

  osMessagePut(spi3ReqQueue_id, (uint32_t) msg, osWaitForever);

  osSemaphoreWait(*(((sensor_handle_t *) handle)->sem), osWaitForever);

  return 0;
}

/**
  * @brief  I2C read function: it adds a request on the I2C read queue (which will be handled by the I2C read thread)
  * @param  argument not used
  * @retval None
  * @note   When the function is used and linked to the sensor context, all the calls made by the
  *         PID driver will result in a call to this function. If this is the case, be sure to make
  *         all the calls to the PID driver functions from a freeRTOS thread
  */
int32_t SM_I2C1_Read_Os(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
  SM_Message_t *msg;

  msg = osPoolAlloc(i2c1Pool_id);

  msg->isRead = 1;
  msg->sensorHandler = handle;
  msg->regAddr = reg;
  msg->readSize = len;
  msg->dataPtr = data;

  osMessagePut(i2c1ReqQueue_id, (uint32_t) msg, osWaitForever);

  osSemaphoreWait(*(((sensor_handle_t *) handle)->sem), osWaitForever);

  return 0;
}

int32_t SM_I2C1_Write_Os(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
  SM_Message_t *msg;

  msg = osPoolAlloc(i2c1Pool_id);

  msg->isRead = 0;
  msg->sensorHandler = handle;
  msg->regAddr = reg;
  msg->readSize = len;
  msg->dataPtr = data;

  osMessagePut(i2c1ReqQueue_id, (uint32_t) msg, osWaitForever);

  osSemaphoreWait(*(((sensor_handle_t *) handle)->sem), osWaitForever);

  return 0;
}

/**
  * @brief  I2C read function: it adds a request on the I2C read queue (which will be handled by the I2C read thread)
  * @param  argument not used
  * @retval None
  * @note   When the function is used and linked to the sensor context, all the calls made by the
  *         PID driver will result in a call to this function. If this is the case, be sure to make
  *         all the calls to the PID driver functions from a freeRTOS thread
  */
int32_t SM_I2C3_Read_Os(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
  SM_Message_t *msg;

  msg = osPoolAlloc(i2c3Pool_id);

  msg->isRead = 1;
  msg->sensorHandler = handle;
  msg->regAddr = reg;
  msg->readSize = len;
  msg->dataPtr = data;

  osMessagePut(i2c3ReqQueue_id, (uint32_t) msg, osWaitForever);

  osSemaphoreWait(*(((sensor_handle_t *) handle)->sem), osWaitForever);

  return 0;
}

int32_t SM_I2C3_Write_Os(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
  SM_Message_t *msg;

  msg = osPoolAlloc(i2c3Pool_id);

  msg->isRead = 0;
  msg->sensorHandler = handle;
  msg->regAddr = reg;
  msg->readSize = len;
  msg->dataPtr = data;

  osMessagePut(i2c3ReqQueue_id, (uint32_t) msg, osWaitForever);

  osSemaphoreWait(*(((sensor_handle_t *) handle)->sem), osWaitForever);

  return 0;
}

/**
  * @brief Sensor Manager Blocking SPI read function
  * @param None
  * @retval None
  * @note This function can be liked to the sensor PID if freeRTOS is not used
  */
int32_t SM_SPI1_Read(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
  taskENTER_CRITICAL();

  reg = reg | 0x80;

  HAL_GPIO_WritePin(((sensor_handle_t *) handle)->GPIOx, ((sensor_handle_t *) handle)->GPIO_Pin, GPIO_PIN_RESET);

  HAL_SPI_Transmit(&hspi1, &reg, 1, SM_SPI_TIMEOUT);
  HAL_SPI_Receive(&hspi1, data, len, SM_SPI_TIMEOUT);

  HAL_GPIO_WritePin(((sensor_handle_t *) handle)->GPIOx, ((sensor_handle_t *) handle)->GPIO_Pin, GPIO_PIN_SET);

  taskEXIT_CRITICAL();

  return 0;
}

/**
  * @brief Sensor Manager Blocking SPI write function
  * @param None
  * @retval None
  */
int32_t SM_SPI1_Write(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
  taskENTER_CRITICAL();

  HAL_GPIO_WritePin(((sensor_handle_t *) handle)->GPIOx, ((sensor_handle_t *) handle)->GPIO_Pin, GPIO_PIN_RESET);

  HAL_SPI_Transmit(&hspi1, &reg, 1, SM_SPI_TIMEOUT);
  HAL_SPI_Transmit(&hspi1, data, len, SM_SPI_TIMEOUT);

  HAL_GPIO_WritePin(((sensor_handle_t *) handle)->GPIOx, ((sensor_handle_t *) handle)->GPIO_Pin, GPIO_PIN_SET);

  taskEXIT_CRITICAL();
  return 0;
}

#if( LIS2MDL_COM_MODE == LIS2MDL_COM_SPI_4_WIRE )

int32_t SM_SPI3_Write(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
  taskENTER_CRITICAL();

  HAL_GPIO_WritePin(((sensor_handle_t *) handle)->GPIOx, ((sensor_handle_t *) handle)->GPIO_Pin, GPIO_PIN_RESET);

  HAL_SPI_Transmit(&hspi3, &reg, 1, SM_SPI_TIMEOUT);
  HAL_SPI_Transmit(&hspi3, data, len, SM_SPI_TIMEOUT);

  HAL_GPIO_WritePin(((sensor_handle_t *) handle)->GPIOx, ((sensor_handle_t *) handle)->GPIO_Pin, GPIO_PIN_SET);

  taskEXIT_CRITICAL();

  return 0;
}

/**
  * @brief Sensor Manager Blocking SPI read function
  * @param None
  * @retval None
  * @note This function can be liked to the sensor PID if freeRTOS is not used
  */
int32_t SM_SPI3_Read(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
  taskENTER_CRITICAL();

  /**
    * SPI read is controlled by asserting the MSB of the register address. This
    * logic can/should be pulled into the individual sensor driver as it is
    * device-dependent.
    *
    **/
  reg = reg | 0x80;

  HAL_GPIO_WritePin(((sensor_handle_t *) handle)->GPIOx, ((sensor_handle_t *) handle)->GPIO_Pin, GPIO_PIN_RESET);

  HAL_SPI_Transmit(&hspi3, &reg, 1, SM_SPI_TIMEOUT);
  HAL_SPI_Receive(&hspi3, data, len, SM_SPI_TIMEOUT);

  HAL_GPIO_WritePin(((sensor_handle_t *) handle)->GPIOx, ((sensor_handle_t *) handle)->GPIO_Pin, GPIO_PIN_SET);

  taskEXIT_CRITICAL();

  return 0;
}

#elif( LIS2MDL_COM_MODE == LIS2MDL_COM_SPI_3_WIRE )

int32_t SM_SPI3_Write(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
  taskENTER_CRITICAL();

  HAL_GPIO_WritePin(((sensor_handle_t *) handle)->GPIOx,
                    ((sensor_handle_t *) handle)->GPIO_Pin,
                    GPIO_PIN_RESET);

  SM_SPI_3_WIRE_WRITE(&hspi3, uint16_t reg, data, len);

  HAL_GPIO_WritePin(((sensor_handle_t *) handle)->GPIOx,
                    ((sensor_handle_t *) handle)->GPIO_Pin,
                    GPIO_PIN_SET);

  taskEXIT_CRITICAL();

  return 0;
}

/**
  * @brief Sensor Manager Blocking SPI read function
  * @param None
  * @retval None
  * @note This function can be liked to the sensor PID if freeRTOS is not used
  */
int32_t SM_SPI3_Read(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
  reg = reg | 0x80;

  taskENTER_CRITICAL();

  HAL_GPIO_WritePin(((sensor_handle_t *) handle)->GPIOx,
                    ((sensor_handle_t *) handle)->GPIO_Pin,
                    GPIO_PIN_RESET);

  SM_SPI_3_WIRE_READ(&hspi3, reg, data, len);

  HAL_GPIO_WritePin(((sensor_handle_t *) handle)->GPIOx,
                    ((sensor_handle_t *) handle)->GPIO_Pin,
                    GPIO_PIN_SET);

  taskEXIT_CRITICAL();

  return 0;
}

/**
  * @brief  This function reads multiple bytes on SPI 3-wire.
  * @param  xSpiHandle: SPI Handler.
  * @param  val: value.
  * @param  nBytesToRead: number of bytes to read.
  * @retval None
  */
void SPI_3_WIRE_READ_nBYTES(SPI_HandleTypeDef *xSpiHandle, uint8_t *val, uint16_t nBytesToRead)
{
  /* Interrupts should be disabled during this operation */
  taskENTER_CRITICAL();
  __HAL_SPI_ENABLE(xSpiHandle);

  /* Transfer loop */
  while (nBytesToRead > 1U)
  {
    /* Check the RXNE flag */
    if (xSpiHandle->Instance->SR & SPI_FLAG_RXNE)
    {
      /* read the received data */
      *val = *(__IO uint8_t *) &xSpiHandle->Instance->DR;
      val += sizeof(uint8_t);
      nBytesToRead--;
    }
  }
  /* In master RX mode the clock is automatically generated on the SPI enable.
  So to guarantee the clock generation for only one data, the clock must be
  disabled after the first bit and before the latest bit of the last Byte received */
  /* __DSB instruction are inserted to guarantee that clock is Disabled in the right timeframe */

  __DSB();
  __DSB();
  __HAL_SPI_DISABLE(xSpiHandle);

  taskEXIT_CRITICAL();

  while ((xSpiHandle->Instance->SR & SPI_FLAG_RXNE) != SPI_FLAG_RXNE);
  /* read the received data */
  *val = *(__IO uint8_t *) &xSpiHandle->Instance->DR;
  while ((xSpiHandle->Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY);
}

/**
  * @brief  This function send a command through SPI bus.
  * @param  command: command id.
  * @param  uint8_t val: value.
  * @retval None
  */
void SPI_3_WIRE_READ_BYTE(SPI_HandleTypeDef *xSpiHandle, uint8_t *val)
{
  /* In master RX mode the clock is automatically generated on the SPI enable.
  So to guarantee the clock generation for only one data, the clock must be
  disabled after the first bit and before the latest bit */
  /* Interrupts should be disabled during this operation */

  taskENTER_CRITICAL();
  __HAL_SPI_ENABLE(xSpiHandle);
  __asm("dsb\n");
  __asm("dsb\n");
  __HAL_SPI_DISABLE(xSpiHandle);
  taskEXIT_CRITICAL();

  while ((xSpiHandle->Instance->SR & SPI_FLAG_RXNE) != SPI_FLAG_RXNE);
  /* read the received data */
  *val = *(__IO uint8_t *) &xSpiHandle->Instance->DR;
  while ((xSpiHandle->Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY);
}

/**
  * @brief  This function send a command through SPI bus.
  * @param  command : command id.
  * @param  val : value.
  * @retval None
  */
static void SPI_3_WIRE_WRITE_CMD(SPI_HandleTypeDef *xSpiHandle, uint8_t val)
{
  /* check TXE flag */
  while ((xSpiHandle->Instance->SR & SPI_FLAG_TXE) != SPI_FLAG_TXE);

  /* Write the data */
  *((__IO uint8_t *) &xSpiHandle->Instance->DR) = val;

  /* Wait BSY flag */
  while ((xSpiHandle->Instance->SR & SPI_FLAG_FTLVL) != SPI_FTLVL_EMPTY);
  while ((xSpiHandle->Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY);
}

static void SM_SPI_3_WIRE_READ(SPI_HandleTypeDef *hspi, uint16_t Reg, uint8_t *pData, uint16_t Size)
{
  uint8_t dataReg = (uint8_t) Reg;

  /* CS Enable */
  SPI_3_WIRE_WRITE_CMD(hspi, (dataReg | 0x80));
  __HAL_SPI_DISABLE(hspi);
  SPI_1LINE_RX(hspi);

  if (len > 1)
  {
    SPI_3_WIRE_READ_nBYTES(hspi, pData, Size);
  }
  else
  {
    SPI_3_WIRE_READ_BYTE(hspi, pdata);
  }

  /* CS Disable */
  SPI_1LINE_TX(hspi);
  __HAL_SPI_ENABLE(hspi);
}

static void SM_SPI_3_WIRE_WRITE(SPI_HandleTypeDef *hspi, uint16_t Reg, uint8_t *pData, uint16_t Size)
{
  uint8_t dataReg = (uint8_t) Reg;

  HAL_SPI_Transmit(hspi, &dataReg, 1, SM_SPI_TIMEOUT);
  HAL_SPI_Transmit(hspi, pData, Size, SM_SPI_TIMEOUT);
}

#endif /* ( LIS2MDL_COM_MODE == LIS2MDL_COM_SPI_4_WIRE ) */

/**
  * @brief Sensor Manager Blocking I2C read function
  * @param None
  * @retval None
  * @note This function can be liked to the sensor PID if freeRTOS is not used
  */
int32_t SM_I2C1_Read(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
  /**
    * HTS221 Auto-Increment is controlled by asserting the MSB of the register
    * address. This logic can/should be pulled into the individual sensor driver
    * as it is device-dependent.
    *
    **/
  uint8_t autoInc = 0x00;

  if ((((sensor_handle_t *) handle)->WhoAmI == 0xBCU) && (len > 1))
  {
    autoInc = 0x80;
  }

  taskENTER_CRITICAL();

  HAL_I2C_Mem_Read(&hi2c1, ((sensor_handle_t *) handle)->I2C_address, (uint16_t)(reg | autoInc),
                   I2C_MEMADD_SIZE_8BIT,
                   data, len,
                   SM_I2C_TIMEOUT);

  taskEXIT_CRITICAL();

  return 0;
}

/**
  * @brief Sensor Manager Blocking I2C write function
  * @param None
  * @retval None
  */
int32_t SM_I2C1_Write(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
  /**
    * HTS221 Auto-Increment is controlled by asserting the MSB of the register
    * address. This logic can/should be pulled into the individual sensor driver
    * as it is device-dependent.
    *
    **/
  uint8_t autoInc = 0x00;

  if ((((sensor_handle_t *) handle)->WhoAmI == 0xBCU) && (len > 1)) /*enable HTS221 auto increment*/
  {
    autoInc = 0x80;
  }

  taskENTER_CRITICAL();

  HAL_I2C_Mem_Write(&hi2c1, ((sensor_handle_t *) handle)->I2C_address, (uint16_t)(reg | autoInc),
                    I2C_MEMADD_SIZE_8BIT,
                    data, len,
                    SM_I2C_TIMEOUT);

  taskEXIT_CRITICAL();

  return 0;
}

/**
  * @brief Sensor Manager Blocking I2C read function
  * @param None
  * @retval None
  * @note This function can be liked to the sensor PID if freeRTOS is not used
  */
int32_t SM_I2C3_Read(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
  taskENTER_CRITICAL();

  HAL_I2C_Mem_Read(&hi2c3, ((sensor_handle_t *) handle)->I2C_address, (uint16_t) reg,
                   I2C_MEMADD_SIZE_8BIT,
                   data, len,
                   SM_I2C_TIMEOUT);

  taskEXIT_CRITICAL();

  return 0;
}

/**
  * @brief Sensor Manager Blocking I2C write function
  * @param None
  * @retval None
  */
int32_t SM_I2C3_Write(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
  taskENTER_CRITICAL();

  HAL_I2C_Mem_Write(&hi2c3, ((sensor_handle_t *) handle)->I2C_address, (uint16_t) reg,
                    I2C_MEMADD_SIZE_8BIT,
                    data, len,
                    SM_I2C_TIMEOUT);

  taskEXIT_CRITICAL();

  return 0;
}

/**
  * @brief  Sensor manager OS functionalities initialization: for each BUS (I2C
  *         and SPI) it initializes a queue to collect read/write request, a
  *         thread (blocking on the queue) to handle read/write requests and a
  *         semaphore used to wait for DMA transfer complete
  * @param  None
  * @retval None
  */
void SM_OS_Init(void)
{
  /* Bus read semaphores */
  spi1ThreadSem_id = osSemaphoreCreate(osSemaphore(spi1ThreadSem), 1);
  spi3ThreadSem_id = osSemaphoreCreate(osSemaphore(spi3ThreadSem), 1);
  i2c1ThreadSem_id = osSemaphoreCreate(osSemaphore(i2c1ThreadSem), 1);
  i2c3ThreadSem_id = osSemaphoreCreate(osSemaphore(i2c3ThreadSem), 1);

  osSemaphoreWait(spi1ThreadSem_id, osWaitForever);
  osSemaphoreWait(spi3ThreadSem_id, osWaitForever);
  osSemaphoreWait(i2c1ThreadSem_id, osWaitForever);
  osSemaphoreWait(i2c3ThreadSem_id, osWaitForever);

  /* Bus read functions memory pools */
  spi1Pool_id = osPoolCreate(osPool(spi1Pool));
  spi3Pool_id = osPoolCreate(osPool(spi3Pool));
  i2c1Pool_id = osPoolCreate(osPool(i2c1Pool));
  i2c3Pool_id = osPoolCreate(osPool(i2c3Pool));

  /* Bus read queues */
  spi1ReqQueue_id = osMessageCreate(osMessageQ(spi1reqqueue), NULL);
  spi3ReqQueue_id = osMessageCreate(osMessageQ(spi3reqqueue), NULL);
  i2c1ReqQueue_id = osMessageCreate(osMessageQ(i2c1reqqueue), NULL);
  i2c3ReqQueue_id = osMessageCreate(osMessageQ(i2c3reqqueue), NULL);

  i2c1ThreadParams.comThreadSem_id = &i2c1ThreadSem_id;
  i2c1ThreadParams.comReqQueue_id = &i2c1ReqQueue_id;
  i2c1ThreadParams.comPool_id = &i2c1Pool_id;
  i2c1ThreadParams.hcom = (void *) &hi2c1;

  osThreadDef(I2C1_THREAD, i2c_Thread, HSD_I2C1_RD_THREAD_PRIO, 1, 300 / 4);
  i2c1ThreadId = osThreadCreate(osThread(I2C1_THREAD), (void *) &i2c1ThreadParams);

  i2c3ThreadParams.comThreadSem_id = &i2c3ThreadSem_id;
  i2c3ThreadParams.comReqQueue_id = &i2c3ReqQueue_id;
  i2c3ThreadParams.comPool_id = &i2c3Pool_id;
  i2c3ThreadParams.hcom = (void *) &hi2c3;

  osThreadDef(I2C3_THREAD, i2c_Thread, HSD_I2C3_RD_THREAD_PRIO, 1, 300 / 4);
  i2c3ThreadId = osThreadCreate(osThread(I2C3_THREAD), (void *) &i2c3ThreadParams);

  spi1ThreadParams.comThreadSem_id = &spi1ThreadSem_id;
  spi1ThreadParams.comReqQueue_id = &spi1ReqQueue_id;
  spi1ThreadParams.comPool_id = &spi1Pool_id;
  spi1ThreadParams.hcom = (void *) &hspi1;

  osThreadDef(SPI1_THREAD, spi_Thread, HSD_SPI1_RD_THREAD_PRIO, 1, 300 / 4);
  spi1ThreadId = osThreadCreate(osThread(SPI1_THREAD), (void *) &spi1ThreadParams);

  spi3ThreadParams.comThreadSem_id = &spi3ThreadSem_id;
  spi3ThreadParams.comReqQueue_id = &spi3ReqQueue_id;
  spi3ThreadParams.comPool_id = &spi3Pool_id;
  spi3ThreadParams.hcom = (void *) &hspi3;

#if( LIS2MDL_COM_MODE == LIS2MDL_COM_SPI_4_WIRE )
  osThreadDef(SPI3_THREAD, spi_Thread, HSD_SPI3_RD_THREAD_PRIO, 1, 300 / 4);
#elif( LIS2MDL_COM_MODE == LIS2MDL_COM_SPI_3_WIRE )
  osThreadDef(SPI3_THREAD, spi3_Thread, HSD_SPI3_RD_THREAD_PRIO, 1, 300 / 4);
#endif /* ( LIS2MDL_COM_MODE == LIS2MDL_COM_SPI_4_WIRE ) */

  spi3ThreadId = osThreadCreate(osThread(SPI3_THREAD), (void *) &spi3ThreadParams);
}

/**
  * Enable DMA controller clock
  */
static void SM_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();

  __SPI1_DMA_CLK_ENABLE();
  __SPI3_DMA_CLK_ENABLE();

  __I2C1_DMA_CLK_ENABLE();
  __I2C3_DMA_CLK_ENABLE();

  /* SP1I_RX_DMA_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SPI1_RX_DMA_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(SPI1_RX_DMA_IRQn);

  /* SPI1_TX_DMA_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SPI1_TX_DMA_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(SPI1_TX_DMA_IRQn);

  /* SPI3_RX_DMA_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SPI3_RX_DMA_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(SPI3_RX_DMA_IRQn);

  /* SPI3_TX_DMA_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SPI3_TX_DMA_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(SPI3_TX_DMA_IRQn);

  /* I2C1_RX_DMA_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C1_RX_DMA_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(I2C1_RX_DMA_IRQn);

  /* I2C1_TX_DMA_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C1_TX_DMA_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(I2C1_TX_DMA_IRQn);

  /* I2C3_RX_DMA_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C3_RX_DMA_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(I2C3_RX_DMA_IRQn);

  /* I2C3_TX_DMA_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C3_TX_DMA_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(I2C3_TX_DMA_IRQn);
}

/**
  * @brief  SPI MSP Initialization
  *         This function configures the SPI hardware resources used
  * @param  hspi: SPI handle pointer
  * @retval None
  */
static void SM_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct =
  {
    0
  };

  if (hspi->Instance == SPI1)
  {
    __HAL_RCC_SPI1_CLK_ENABLE();

    __SPI1_SCK_PIN_CLK_ENABLE();
    __SPI1_MISO_PIN_CLK_ENABLE();
    __SPI1_MOSI_PIN_CLK_ENABLE();

    GPIO_InitStruct.Pin = SPI1_SCK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;

    HAL_GPIO_Init(SPI1_SCK_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SPI1_MISO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;

    HAL_GPIO_Init(SPI1_MISO_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SPI1_MOSI_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;

    HAL_GPIO_Init(SPI1_MOSI_GPIO_PORT, &GPIO_InitStruct);

    /* SPI DMA Init */
    /* SPI_RX Init */
    hdma_spi1_rx.Instance = SPI1_RX_DMA_CHANNEL;
    hdma_spi1_rx.Init.Request = DMA_REQUEST_SPI1_RX;
    hdma_spi1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi1_rx.Init.Mode = DMA_NORMAL;
    hdma_spi1_rx.Init.Priority = DMA_PRIORITY_LOW;

    if (HAL_DMA_Init(&hdma_spi1_rx) != HAL_OK)
    {
      SM_Error_Handler();
    }

    __HAL_LINKDMA(hspi, hdmarx, hdma_spi1_rx);

    /* SPI_TX Init */
    hdma_spi1_tx.Instance = SPI1_TX_DMA_CHANNEL;
    hdma_spi1_tx.Init.Request = DMA_REQUEST_SPI1_TX;
    hdma_spi1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi1_tx.Init.Mode = DMA_NORMAL;
    hdma_spi1_tx.Init.Priority = DMA_PRIORITY_LOW;

    if (HAL_DMA_Init(&hdma_spi1_tx) != HAL_OK)
    {
      SM_Error_Handler();
    }

    __HAL_LINKDMA(hspi, hdmatx, hdma_spi1_tx);
  }
  else if (hspi->Instance == SPI3)
  {
    __HAL_RCC_SPI3_CLK_ENABLE();

    __SPI3_SCK_PIN_CLK_ENABLE();
    __SPI3_MISO_PIN_CLK_ENABLE();
    __SPI3_MOSI_PIN_CLK_ENABLE();

    GPIO_InitStruct.Pin = SPI3_SCK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;

    HAL_GPIO_Init(SPI3_SCK_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SPI3_MISO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;

    HAL_GPIO_Init(SPI3_MISO_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SPI3_MOSI_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;

    HAL_GPIO_Init(SPI3_MOSI_GPIO_PORT, &GPIO_InitStruct);

    /* SPI DMA Init */
    /* SPI_RX Init */
    hdma_spi3_rx.Instance = SPI3_RX_DMA_CHANNEL;
    hdma_spi3_rx.Init.Request = DMA_REQUEST_SPI3_RX;
    hdma_spi3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi3_rx.Init.Mode = DMA_NORMAL;
    hdma_spi3_rx.Init.Priority = DMA_PRIORITY_LOW;

    if (HAL_DMA_Init(&hdma_spi3_rx) != HAL_OK)
    {
      SM_Error_Handler();
    }

    __HAL_LINKDMA(hspi, hdmarx, hdma_spi3_rx);

    /* SPI_TX Init */
    hdma_spi3_tx.Instance = SPI3_TX_DMA_CHANNEL;
    hdma_spi3_tx.Init.Request = DMA_REQUEST_SPI3_TX;
    hdma_spi3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi3_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi3_tx.Init.Mode = DMA_NORMAL;
    hdma_spi3_tx.Init.Priority = DMA_PRIORITY_LOW;

    if (HAL_DMA_Init(&hdma_spi3_tx) != HAL_OK)
    {
      SM_Error_Handler();
    }

    __HAL_LINKDMA(hspi, hdmatx, hdma_spi3_tx);
  }
}

static void SM_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI1)
  {
    osSemaphoreRelease(spi1ThreadSem_id);
  }
  else if (hspi->Instance == SPI3)
  {
    osSemaphoreRelease(spi3ThreadSem_id);
  }
}

/**
  * @brief I2C MSP Initialization
  *         This function configures the I2C hardware resources used
  * @param hi2c: I2C handle pointer
  * @retval None
  */
static void SM_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
  GPIO_InitTypeDef GPIO_InitStruct =
  {
    0
  };

  if (hi2c->Instance == I2C1)
  {
    __HAL_RCC_I2C1_CLK_ENABLE();

    __I2C1_SCL_PIN_CLK_ENABLE();
    __I2C1_SDA_PIN_CLK_ENABLE();

    GPIO_InitStruct.Pin = I2C1_SCL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(I2C1_SCL_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = I2C1_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(I2C1_SDA_GPIO_PORT, &GPIO_InitStruct);

    /* I2C DMA Init */
    /* I2C_RX Init */
    hdma_i2c1_rx.Instance = I2C1_RX_DMA_CHANNEL;
    hdma_i2c1_rx.Init.Request = DMA_REQUEST_I2C1_RX;
    hdma_i2c1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_i2c1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c1_rx.Init.Mode = DMA_NORMAL;
    hdma_i2c1_rx.Init.Priority = DMA_PRIORITY_HIGH;

    if (HAL_DMA_Init(&hdma_i2c1_rx) != HAL_OK)
    {
      SM_Error_Handler();
    }

    __HAL_LINKDMA(hi2c, hdmarx, hdma_i2c1_rx);

    /* I2C_TX Init */
    hdma_i2c1_tx.Instance = I2C1_TX_DMA_CHANNEL;
    hdma_i2c1_tx.Init.Request = DMA_REQUEST_I2C1_TX;
    hdma_i2c1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_i2c1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c1_tx.Init.Mode = DMA_NORMAL;
    hdma_i2c1_tx.Init.Priority = DMA_PRIORITY_HIGH;

    if (HAL_DMA_Init(&hdma_i2c1_tx) != HAL_OK)
    {
      SM_Error_Handler();
    }

    __HAL_LINKDMA(hi2c, hdmatx, hdma_i2c1_tx);

    /* I2C interrupt Init */
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
  }
  else if (hi2c->Instance == I2C3)
  {
    __HAL_RCC_I2C3_CLK_ENABLE();

    __I2C3_SCL_PIN_CLK_ENABLE();
    __I2C3_SDA_PIN_CLK_ENABLE();

    GPIO_InitStruct.Pin = I2C3_SCL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
    HAL_GPIO_Init(I2C3_SCL_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = I2C3_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
    HAL_GPIO_Init(I2C3_SDA_GPIO_PORT, &GPIO_InitStruct);

    /* I2C DMA Init */
    /* I2C_RX Init */
    hdma_i2c3_rx.Instance = I2C3_RX_DMA_CHANNEL;
    hdma_i2c3_rx.Init.Request = DMA_REQUEST_I2C3_RX;
    hdma_i2c3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_i2c3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c3_rx.Init.Mode = DMA_NORMAL;
    hdma_i2c3_rx.Init.Priority = DMA_PRIORITY_HIGH;

    if (HAL_DMA_Init(&hdma_i2c3_rx) != HAL_OK)
    {
      SM_Error_Handler();
    }

    __HAL_LINKDMA(hi2c, hdmarx, hdma_i2c3_rx);

    /* I2C_TX Init */
    hdma_i2c3_tx.Instance = I2C3_TX_DMA_CHANNEL;
    hdma_i2c3_tx.Init.Request = DMA_REQUEST_I2C3_TX;
    hdma_i2c3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_i2c3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c3_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c3_tx.Init.Mode = DMA_NORMAL;
    hdma_i2c3_tx.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_i2c3_tx) != HAL_OK)
    {
      SM_Error_Handler();
    }

    __HAL_LINKDMA(hi2c, hdmatx, hdma_i2c3_tx);

    /* I2C interrupt Init */
    HAL_NVIC_SetPriority(I2C3_EV_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(I2C3_EV_IRQn);
    HAL_NVIC_SetPriority(I2C3_ER_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(I2C3_ER_IRQn);
  }
}

void SM_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C1)
  {
    osSemaphoreRelease(i2c1ThreadSem_id);
    reg_after_release = hi2c1.Instance->CR1 & 0x00000040;
  }
  else if (hi2c->Instance == I2C3)
  {
    osSemaphoreRelease(i2c3ThreadSem_id);
    reg_after_release = hi2c3.Instance->CR1 & 0x00000040;
  }
}

void SM_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C1)
  {
    osSemaphoreRelease(i2c1ThreadSem_id);
  }
  else if (hi2c->Instance == I2C3)
  {
    osSemaphoreRelease(i2c3ThreadSem_id);
  }
}

void SM_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C1)
  {
    errors++;
  }
  else if (hi2c->Instance == I2C3)
  {
    errors++;
  }
}

/******************************************************************************/
/* Sensor Manager Timer-related Functions                                     */
/******************************************************************************/

/**
  * @brief  Retrieve the current time stamp from outside of an interrupt context
  * @param  None
  * @retval Time stamp
  */
double SM_GetTimeStamp(void)
{
  uint32_t tim_counter;
  uint64_t timestamp;
  double timestamp_seconds;

  /* -------- Start critical section -----------*/
  taskENTER_CRITICAL();

  tim_counter = htim5.Instance->CNT;

  if (__HAL_TIM_GET_FLAG(&htim5, TIM_FLAG_UPDATE)) /* Update Event happened while already in critical section */
  {
    /* Evaluate if the timer was read before or after the "UPDATE" event */
    if (tim_counter < ((SystemCoreClock * SM_TS_UPDATE_PERIOD_S) / 2)) /* After */
    {
      timestamp = SM_TimeStamp + tim_counter + (SystemCoreClock * SM_TS_UPDATE_PERIOD_S);
    }
    else /* Before */
    {
      timestamp = SM_TimeStamp + tim_counter;
    }
  }
  else
  {
    /* No Update Event, just sum the timer value to the global TimeStamp */
    timestamp = SM_TimeStamp + tim_counter;
  }

  timestamp_seconds = ((double) timestamp / (double) SystemCoreClock);

  taskEXIT_CRITICAL();
  /* -------- End critical section -----------*/
  return timestamp_seconds;
}

/**
  * @brief  Retrieve the current time stamp from within an interrupt context
  * @param  None
  * @retval Time stamp
  */
double SM_GetTimeStamp_fromISR(void)
{
  uint32_t tim_counter;
  uint64_t timestamp;
  double timestamp_seconds;
  uint32_t isr_mask;

  /* -------- Start critical section -----------*/
  isr_mask = taskENTER_CRITICAL_FROM_ISR();

  tim_counter = htim5.Instance->CNT;

  if (__HAL_TIM_GET_FLAG(&htim5, TIM_FLAG_UPDATE)) /* Update Event happened while already in critical section */
  {
    /* Evaluate if the timer was read before or after the "UPDATE" event */
    if (tim_counter < ((SystemCoreClock * SM_TS_UPDATE_PERIOD_S) / 2)) /* After */
    {
      timestamp = SM_TimeStamp + tim_counter + (SystemCoreClock * SM_TS_UPDATE_PERIOD_S);
    }
    else /* Before */
    {
      timestamp = SM_TimeStamp + tim_counter;
    }
  }
  else
  {
    /* No Update Event, just sum the timer value to the global TimeStamp */
    timestamp = SM_TimeStamp + tim_counter;
  }

  timestamp_seconds = ((double) timestamp / (double) SystemCoreClock);

  taskEXIT_CRITICAL_FROM_ISR(isr_mask);
  /* -------- End critical section -----------*/

  return timestamp_seconds;
}

/**
  * @brief  Sensor manager TIM5 Initialization Function
  * @param  None
  * @retval None
  */
static void SM_TIM_Init(void)
{
  __HAL_RCC_TIM5_CLK_ENABLE();

  /* Set TIMx instance */
  htim5.Instance = TIM5;

  /* Initialize TIMx peripheral as follows:
   + Period = TimerClock * SM_TS_UPDATE_PERIOD_S - 1
   + Prescaler = 0
   + ClockDivision = 0
   + Counter direction = Up
   */
  htim5.Init.Period = (SystemCoreClock * SM_TS_UPDATE_PERIOD_S) - 1; /* 10s */
  htim5.Init.Prescaler = 0;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.RepetitionCounter = 0;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    /* Initialization Error */
  }

  HAL_TIM_RegisterCallback(&htim5, HAL_TIM_PERIOD_ELAPSED_CB_ID, SM_TIM_PeriodElapsedCallback);

  /* TIM5 interrupt Init */
  HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM5_IRQn);
}

static void SM_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  static uint8_t first = 1;

  if (first) /* first interrupt is generated at Tim_Start --> ignore */
  {
    first = 0;
  }
  else
  {
    SM_TimeStamp += (SystemCoreClock * SM_TS_UPDATE_PERIOD_S);
  }
}

void SM_TIM_Start(void)
{
  /* Start the TIM Base generation */
  if (HAL_TIM_Base_Start_IT(&htim5) != HAL_OK)
  {
    /* Starting Error */
  }
}

void SM_TIM_Stop(void)
{
  /* Stop the TIM Base generation */
  if (HAL_TIM_Base_Stop(&htim5) != HAL_OK)
  {
    /* Stopping Error */
  }

  __HAL_TIM_SET_COUNTER(&htim5, 0);
  SM_TimeStamp = 0;
}

/**
  * @brief  Stop all active sensor threads, reset first_dataReady, Stop SM Timer and Tags Timer
  * @param  None
  * @retval 0: no error
  */
uint8_t SM_StopSensorAcquisition(void)
{
  uint32_t sensorId = 0;
  uint32_t subSensorId = 0;
  COM_DeviceDescriptor_t *pDeviceDescriptor;
  COM_SensorDescriptor_t *pSensorDescriptor;
  COM_SubSensorStatus_t *pSubSensorStatus;

  pDeviceDescriptor = COM_GetDeviceDescriptor();
  HSD_TAGS_timer_stop();

  for (sensorId = 0; sensorId < pDeviceDescriptor->nSensor; sensorId++)
  {
    pSensorDescriptor = COM_GetSensorDescriptor(sensorId);

    for (subSensorId = 0; subSensorId < pSensorDescriptor->nSubSensors; subSensorId++)
    {
      pSubSensorStatus = COM_GetSubSensorStatus(sensorId, subSensorId);

      if (pSubSensorStatus->isActive == 1)
      {
        SM_StopSensorThread(sensorId);
        COM_GetSubSensorContext(sensorId, subSensorId)->first_dataReady = 0;
      }
    }
  }

  /* Stop Sensor Manager Timer */
  SM_TIM_Stop();

  return 0;
}

/**
  * @brief  Stop sensor thread
  * @param  sId: Sensor id
  * @retval 0: no error
  */
uint8_t SM_StartSensorThread(uint8_t sensorId)
{
  if (sensorId == LIS3DHH_Get_Id())
  {
    LIS3DHH_Start();
  }
  else if (sensorId == HTS221_Get_Id())
  {
    HTS221_Start();
  }
  else if (sensorId == LIS2DW12_Get_Id())
  {
    LIS2DW12_Start();
  }
  else if (sensorId == LIS2MDL_Get_Id())
  {
    LIS2MDL_Start();
  }
  else if (sensorId == MP23ABS1_Get_Id())
  {
    MP23ABS1_Start();
  }
  else if (sensorId == LSM6DSOX_Get_Id())
  {
    LSM6DSOX_Start();
  }
  else if (sensorId == LPS22HH_Get_Id())
  {
    LPS22HH_Start();
  }
  else if (sensorId == STTS751_Get_Id())
  {
    STTS751_Start();
  }
  return 0;
}

/**
  * @brief  Stop sensor thread
  * @param  sId: Sensor id
  * @retval 0: no error
  */
uint8_t SM_StopSensorThread(uint8_t sensorId)
{
  if (sensorId == LIS3DHH_Get_Id())
  {
    LIS3DHH_Stop();
  }
  else if (sensorId == HTS221_Get_Id())
  {
    HTS221_Stop();
  }
  else if (sensorId == LIS2DW12_Get_Id())
  {
    LIS2DW12_Stop();
  }
  else if (sensorId == LIS2MDL_Get_Id())
  {
    LIS2MDL_Stop();
  }
  else if (sensorId == MP23ABS1_Get_Id())
  {
    MP23ABS1_Stop();
  }
  else if (sensorId == LSM6DSOX_Get_Id())
  {
    LSM6DSOX_Stop();
  }
  else if (sensorId == LPS22HH_Get_Id())
  {
    LPS22HH_Stop();
  }
  else if (sensorId == STTS751_Get_Id())
  {
    STTS751_Stop();
  }
  return 0;
}

