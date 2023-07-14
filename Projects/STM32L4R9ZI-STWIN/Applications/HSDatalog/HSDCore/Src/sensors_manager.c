/**
  ******************************************************************************
  * @file    sensors_manager.c
  * @author  SRA - MCD
  *
  *
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
#include "iis2dh_reg.h"
#include "com_manager.h"
#include "HSD_tags.h"

#include "imp23absu_app.h"
#include "imp34dt05_app.h"
#include "ism330dhcx_app.h"
#include "iis3dwb_app.h"
#include "iis2mdc_app.h"
#include "iis2dh_app.h"
#include "hts221_app.h"
#include "lps22hh_app.h"
#include "stts751_app.h"

osSemaphoreId spiThreadSem_id;
osSemaphoreDef(spiThreadSem);

osSemaphoreId i2cThreadSem_id;
osSemaphoreDef(i2cThreadSem);

/* Private includes ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define SM_TS_UPDATE_PERIOD_S     (10)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hsm_spi;
I2C_HandleTypeDef hsm_i2c;
DMA_HandleTypeDef hdma_sm_i2c_rx;
DMA_HandleTypeDef hdma_sm_i2c_tx;
DMA_HandleTypeDef hdma_sm_spi_tx;
DMA_HandleTypeDef hdma_sm_spi_rx;
TIM_HandleTypeDef hsm_tim;

/* Private function prototypes -----------------------------------------------*/
static void SM_DMA_Init(void);
static void SM_I2C_Init(void);
static void SM_I2C_MspInit(I2C_HandleTypeDef *hi2c);
static void SM_I2C_MemRxCpltCallback(I2C_HandleTypeDef *I2cHandle);
static void SM_I2C_MemTxCpltCallback(I2C_HandleTypeDef *I2cHandle);
static void SM_I2C_ErrorCallback(I2C_HandleTypeDef *I2cHandle);
static void SM_SPI_Init(void);
static void SM_SPI_MspInit(SPI_HandleTypeDef *hspi);
static void SM_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
void SM_TIM_Init(void);

osMessageQId spiReqQueue_id;
osMessageQDef(spireqqueue, 10, int);

osMessageQId i2cReqQueue_id;
osMessageQDef(i2creqqueue, 10, int);

static void spi_Thread(void const *argument);
osThreadId spiThreadId;

static void i2c_Thread(void const *argument);
osThreadId i2cThreadId;

osPoolId spiPool_id;
osPoolDef(spiPool, 100, SM_Message_t);

osPoolId i2cPool_id;
osPoolDef(i2cPool, 100, SM_Message_t);

uint32_t reg_after_release = 0;
static int32_t errors = 0;

static volatile uint64_t SM_TimeStamp = 0;   /* Sensor Manager global timestamp */

/**
  * @brief Sensor manager SPI Initialization Function
  * @param None
  * @retval None
  * @note callbacks to the MSP
  */
static void SM_SPI_Init(void)
{
  /* SPI3 parameter configuration*/
  hsm_spi.Instance = SM_SPI_x;
  hsm_spi.Init.Mode = SPI_MODE_MASTER;
  hsm_spi.Init.Direction = SPI_DIRECTION_2LINES;
  hsm_spi.Init.DataSize = SPI_DATASIZE_8BIT;
  hsm_spi.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hsm_spi.Init.CLKPhase = SPI_PHASE_2EDGE;
  hsm_spi.Init.NSS = SPI_NSS_SOFT;
  hsm_spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; /*SPI running @ 10 MHz */ /*stwin*/
  hsm_spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hsm_spi.Init.TIMode = SPI_TIMODE_DISABLE;
  hsm_spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hsm_spi.Init.CRCPolynomial = 7;
  hsm_spi.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hsm_spi.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;

  /* Register MSP Callback */
  HAL_SPI_RegisterCallback(&hsm_spi, HAL_SPI_MSPINIT_CB_ID, SM_SPI_MspInit);

  if (HAL_SPI_Init(&hsm_spi) != HAL_OK)
  {
    SM_Error_Handler();
  }

  /* Register SPI DMA complete Callback */
  HAL_SPI_RegisterCallback(&hsm_spi, HAL_SPI_TX_RX_COMPLETE_CB_ID, SM_SPI_TxRxCpltCallback);
}

void SM_Error_Handler(void)
{
  while (1)
  {}
}

/**
  * @brief Sensor manager OS functionalities initialization: for each BUS (I2C and SPI) it
  *        initializes a queue to collect read request, a thread (blocking on the queue) to handle
  *        read requests and a semaphore used to wait for DMA transfer complete
  * @param None
  * @retval None
  */
void SM_OS_Init(void)
{
  /* Bus read semaphores */
  spiThreadSem_id = osSemaphoreCreate(osSemaphore(spiThreadSem), 1);
  osSemaphoreWait(spiThreadSem_id, osWaitForever);
  i2cThreadSem_id = osSemaphoreCreate(osSemaphore(i2cThreadSem), 1);
  osSemaphoreWait(i2cThreadSem_id, osWaitForever);

  /* Bus read functions memory pools */
  spiPool_id = osPoolCreate(osPool(spiPool));
  i2cPool_id = osPoolCreate(osPool(i2cPool));

  /* Bus read queues */
  spiReqQueue_id = osMessageCreate(osMessageQ(spireqqueue), NULL);
  i2cReqQueue_id = osMessageCreate(osMessageQ(i2creqqueue), NULL);

  vQueueAddToRegistry(spiReqQueue_id, "spiReqQueue_id");

  /* SPI read Thread*/
  osThreadDef(SPI_THREAD, spi_Thread, HSD_SPI_RD_THREAD_PRIO, 1, 300 / 4);
  /* SPI read Thread*/
  osThreadDef(I2C_THREAD, i2c_Thread, HSD_I2C_RD_THREAD_PRIO, 1, 300 / 4);

  /* Start SPI read Thread */
  spiThreadId = osThreadCreate(osThread(SPI_THREAD), NULL);
  /* Start I2C read Thread */
  i2cThreadId = osThreadCreate(osThread(I2C_THREAD), NULL);
}

int32_t SM_SPI_Write(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
  taskENTER_CRITICAL();

  if (((sensor_handle_t *)handle)->WhoAmI == IIS2DH_ID && len > 1)
  {
    reg = reg | 0x40;
  }

  HAL_GPIO_WritePin(((sensor_handle_t *)handle)->GPIOx, ((sensor_handle_t *)handle)->GPIO_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hsm_spi, &reg, 1, 1000);
  HAL_SPI_Transmit(&hsm_spi, data, len, 1000);
  HAL_GPIO_WritePin(((sensor_handle_t *)handle)->GPIOx, ((sensor_handle_t *)handle)->GPIO_Pin, GPIO_PIN_SET);

  taskEXIT_CRITICAL();
  return 0;
}

/**
  * @brief Sensor Manager Blocking SPI read function
  * @param None
  * @retval None
  * @note This function can be liked to the sensor PID if freeRTOS is not used
  */
int32_t SM_SPI_Read(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
  uint8_t autoInc = 0x00;

  if (((sensor_handle_t *)handle)->WhoAmI == IIS2DH_ID && len > 1)
  {
    autoInc = 0x40;
  }

  reg = reg | 0x80 | autoInc;

  HAL_GPIO_WritePin(((sensor_handle_t *)handle)->GPIOx, ((sensor_handle_t *)handle)->GPIO_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hsm_spi, &reg, 1, 1000);
  HAL_SPI_Receive(&hsm_spi, data, len, 1000);
  HAL_GPIO_WritePin(((sensor_handle_t *)handle)->GPIOx, ((sensor_handle_t *)handle)->GPIO_Pin, GPIO_PIN_SET);
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
int32_t SM_SPI_Read_Os(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
  uint8_t autoInc = 0x00;
  SM_Message_t *msg;

  msg = osPoolAlloc(spiPool_id);

  if (((sensor_handle_t *)handle)->WhoAmI == IIS2DH_ID && len > 1)
  {
    autoInc = 0x40;
  }

  msg->sensorHandler = handle;
  msg->regAddr = reg | 0x80 | autoInc;
  msg->readSize = len;
  msg->dataPtr = data;

  osMessagePut(spiReqQueue_id, (uint32_t)(msg), osWaitForever);
  osSemaphoreWait(*(((sensor_handle_t *)handle)->sem), osWaitForever);

  return 0;
}

int32_t SM_SPI_Write_Os(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
  uint8_t autoInc = 0x00;

  if (((sensor_handle_t *)handle)->WhoAmI == IIS2DH_ID && len > 1)
  {
    autoInc = 0x40;
  }

  SM_Message_t *msg;
  msg = osPoolAlloc(spiPool_id);
  msg->sensorHandler = handle;
  msg->regAddr = reg | autoInc ;
  msg->readSize = len;
  msg->dataPtr = data;

  osMessagePut(spiReqQueue_id, (uint32_t)(msg), osWaitForever);
  osSemaphoreWait(*(((sensor_handle_t *)handle)->sem), osWaitForever);

  return 0;
}

/**
  * @brief  SPI thread: it waits on the SPI request queue, performs SPI transactions in non blocking mode and unlocks
  *         the thread which made the request at the end of the read.
  * @param  argument not used
  * @retval None
  */
static void spi_Thread(void const *argument)
{
  (void)argument;

#if (configUSE_APPLICATION_TASK_TAG == 1 && defined(HSD_TASK_SM_SPI_DEBUG_PIN))
  vTaskSetApplicationTaskTag(NULL, (TaskHookFunction_t)HSD_TASK_SM_SPI_DEBUG_PIN);
#endif /* configUSE_APPLICATION_TASK_TAG == 1 && defined(HSD_TASK_SM_SPI_DEBUG_PIN) */

  osEvent evt;
  for (;;)
  {
    evt = osMessageGet(spiReqQueue_id, osWaitForever);

    SM_Message_t *msg = evt.value.p;

    HAL_GPIO_WritePin(((sensor_handle_t *)msg->sensorHandler)->GPIOx, ((sensor_handle_t *)msg->sensorHandler)->GPIO_Pin,
                      GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hsm_spi, &msg->regAddr, 1, 1000);
    HAL_SPI_TransmitReceive_DMA(&hsm_spi, msg->dataPtr, msg->dataPtr, msg->readSize);

    osSemaphoreWait(spiThreadSem_id, osWaitForever);

    HAL_GPIO_WritePin(((sensor_handle_t *)msg->sensorHandler)->GPIOx, ((sensor_handle_t *)msg->sensorHandler)->GPIO_Pin,
                      GPIO_PIN_SET);

    osSemaphoreId *sem = ((sensor_handle_t *)msg->sensorHandler)->sem;
    osPoolFree(spiPool_id, msg);
    osSemaphoreRelease(*sem);
  }
}

static void SM_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  osSemaphoreRelease(spiThreadSem_id);
}

/**
  * @brief Sensor Manager I2C Initialization Function
  * @param None
  * @retval None
  */
static void SM_I2C_Init(void)
{
  hsm_i2c.Instance = SM_I2C_x;
  hsm_i2c.Init.Timing = 0x00B03FDB;
  hsm_i2c.Init.OwnAddress1 = 0;
  hsm_i2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hsm_i2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hsm_i2c.Init.OwnAddress2 = 0;
  hsm_i2c.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hsm_i2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hsm_i2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  /* Register MSP Callback */
  HAL_I2C_RegisterCallback(&hsm_i2c, HAL_I2C_MSPINIT_CB_ID, SM_I2C_MspInit);

  if (HAL_I2C_Init(&hsm_i2c) != HAL_OK)
  {
    SM_Error_Handler();
  }
  /** Configure Analog filter */
  if (HAL_I2CEx_ConfigAnalogFilter(&hsm_i2c, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    SM_Error_Handler();
  }
  /** Configure Digital filter */
  if (HAL_I2CEx_ConfigDigitalFilter(&hsm_i2c, 0) != HAL_OK)
  {
    SM_Error_Handler();
  }

  HAL_I2C_RegisterCallback(&hsm_i2c, HAL_I2C_MEM_RX_COMPLETE_CB_ID, SM_I2C_MemRxCpltCallback);
  HAL_I2C_RegisterCallback(&hsm_i2c, HAL_I2C_MEM_TX_COMPLETE_CB_ID, SM_I2C_MemTxCpltCallback);
  HAL_I2C_RegisterCallback(&hsm_i2c, HAL_I2C_ERROR_CB_ID, SM_I2C_ErrorCallback);
}


/**
  * @brief Sensor Manager Blocking I2C read function
  * @param None
  * @retval None
  * @note This function can be liked to the sensor PID if freeRTOS is not used
  */
int32_t SM_I2C_Read(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
  uint8_t autoInc = 0x00;

  if (((sensor_handle_t *)handle)->WhoAmI == 0xBCU && len > 1) /*enable HTS221 auto increment*/
  {
    autoInc = 0x80;
  }
  HAL_I2C_Mem_Read(&hsm_i2c, ((sensor_handle_t *)handle)->I2C_address, (uint16_t)reg | autoInc, I2C_MEMADD_SIZE_8BIT,
                   data, len, 1000);
  return 0;
}

/**
  * @brief Sensor Manager Blocking I2C write function
  * @param None
  * @retval None
  */
int32_t SM_I2C_Write(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
  taskENTER_CRITICAL();
  uint8_t autoInc = 0x00;

  if (((sensor_handle_t *)handle)->WhoAmI == 0xBCU && len > 1) /*enable HTS221 auto increment*/
  {
    autoInc = 0x80;
  }

  HAL_I2C_Mem_Write(&hsm_i2c, ((sensor_handle_t *)handle)->I2C_address, (uint16_t)reg | autoInc, I2C_MEMADD_SIZE_8BIT,
                    data, len, 1000);
  taskEXIT_CRITICAL();

  return 0;
}

/**
  * @brief  I2C thread: it waits on the I2C request queue, performs I2C transactions in non blocking mode and
  *         unlocks the thread which made the request at the end of the read.
  * @param  argument not used
  * @retval None
  */
static void i2c_Thread(void const *argument)
{
  (void)argument;

#if (configUSE_APPLICATION_TASK_TAG == 1 && defined(HSD_TASK_SM_I2C_DEBUG_PIN))
  vTaskSetApplicationTaskTag(NULL, (TaskHookFunction_t)HSD_TASK_SM_I2C_DEBUG_PIN);
#endif /* configUSE_APPLICATION_TASK_TAG == 1 && defined(HSD_TASK_SM_I2C_DEBUG_PIN) */

  osEvent evt;
  for (;;)
  {
    evt = osMessageGet(i2cReqQueue_id, osWaitForever);
    uint8_t autoInc = 0;

    SM_Message_t *msg = evt.value.p;
    if (msg->sensorHandler->WhoAmI == 0xBCU && msg->readSize > 1)
    {
      autoInc = 0x80;
    }

    if (msg->isRead)
    {
      HAL_I2C_Mem_Read_DMA(&hsm_i2c, ((sensor_handle_t *)msg->sensorHandler)->I2C_address,
                           (uint16_t)msg->regAddr | autoInc, I2C_MEMADD_SIZE_8BIT,
                           msg->dataPtr, msg->readSize);
    }
    else
    {
      HAL_I2C_Mem_Write_DMA(&hsm_i2c, ((sensor_handle_t *)msg->sensorHandler)->I2C_address,
                            (uint16_t)msg->regAddr | autoInc, I2C_MEMADD_SIZE_8BIT,
                            msg->dataPtr, msg->readSize);
    }
    osSemaphoreWait(i2cThreadSem_id, osWaitForever);

    osSemaphoreId *sem = ((sensor_handle_t *)msg->sensorHandler)->sem;
    osPoolFree(i2cPool_id, msg);

    osSemaphoreRelease(*sem);
  }
}

/**
  * @brief  I2C read function: it adds a request on the I2C read queue (which will be handled by the I2C read thread)
  * @param  argument not used
  * @retval None
  * @note   When the function is used and linked to the sensor context, all the calls made by the
  *         PID driver will result in a call to this function. If this is the case, be sure to make
  *         all the calls to the PID driver functions from a freeRTOS thread
  */
int32_t SM_I2C_Read_Os(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
  SM_Message_t *msg;
  msg = osPoolAlloc(i2cPool_id);
  msg->isRead = 1;
  msg->sensorHandler = handle;
  msg->regAddr = reg ;
  msg->readSize = len;
  msg->dataPtr = data;

  osMessagePut(i2cReqQueue_id, (uint32_t)(msg), osWaitForever);

  osSemaphoreWait(*(((sensor_handle_t *)handle)->sem), osWaitForever);
  return 0;
}

int32_t SM_I2C_Write_Os(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
  SM_Message_t *msg;
  msg = osPoolAlloc(i2cPool_id);
  msg->isRead = 0;
  msg->sensorHandler = handle;
  msg->regAddr = reg ;
  msg->readSize = len;
  msg->dataPtr = data;

  osMessagePut(i2cReqQueue_id, (uint32_t)(msg), osWaitForever);

  osSemaphoreWait(*(((sensor_handle_t *)handle)->sem), osWaitForever);
  return 0;
}

void SM_I2C_MemRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
  osSemaphoreRelease(i2cThreadSem_id);
  reg_after_release =  hsm_i2c.Instance->CR1 & 0x00000040;
}

void SM_I2C_MemTxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
  osSemaphoreRelease(i2cThreadSem_id);
}


void SM_I2C_ErrorCallback(I2C_HandleTypeDef *I2cHandle)
{
  errors++;
}

void SM_Peripheral_Init(void)
{
  SM_DMA_Init();
  SM_I2C_Init();
  SM_SPI_Init();
  SM_TIM_Init();
}


/**
  * Enable DMA controller clock
  */
static void SM_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  SM_SPIx_DMA_CLK_ENABLE();
  SM_I2Cx_DMA_CLK_ENABLE();

  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SM_I2C_TX_DMA_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(SM_I2C_TX_DMA_IRQn);

  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SM_I2C_RX_DMA_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(SM_I2C_RX_DMA_IRQn);

  /* SM_SPI_RX_DMA_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SM_SPI_RX_DMA_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(SM_SPI_RX_DMA_IRQn);

  /* SM_SPI_TX_DMA_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SM_SPI_TX_DMA_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(SM_SPI_TX_DMA_IRQn);
}

static void SM_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  SM_SPI_CLK_PIN_CLK_ENABLE();
  SM_SPI_MISO_PIN_CLK_ENABLE();
  SM_SPI_MOSI_PIN_CLK_ENABLE();

  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  GPIO_InitStruct.Alternate = SM_SPI_CLK_AF;
  GPIO_InitStruct.Pin = SM_SPI_CLK_PIN;
  HAL_GPIO_Init(SM_SPI_CLK_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Alternate = SM_SPI_MISO_AF;
  GPIO_InitStruct.Pin = SM_SPI_MISO_PIN;
  HAL_GPIO_Init(SM_SPI_MISO_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Alternate = SM_SPI_MOSI_AF;
  GPIO_InitStruct.Pin = SM_SPI_MOSI_PIN;
  HAL_GPIO_Init(SM_SPI_MOSI_GPIO_PORT, &GPIO_InitStruct);

  SM_SPIx_CLK_ENABLE();

  /* SPI DMA Init */
  /* SPI_RX Init */
  hdma_sm_spi_rx.Instance = SM_SPI_RX_DMA_CHANNEL;
  hdma_sm_spi_rx.Init.Request = SM_SPI_RX_DMA_REQUEST;
  hdma_sm_spi_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_sm_spi_rx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_sm_spi_rx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_sm_spi_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_sm_spi_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_sm_spi_rx.Init.Mode = DMA_NORMAL;
  hdma_sm_spi_rx.Init.Priority = DMA_PRIORITY_LOW;
  if (HAL_DMA_Init(&hdma_sm_spi_rx) != HAL_OK)
  {
    SM_Error_Handler();
  }

  __HAL_LINKDMA(hspi, hdmarx, hdma_sm_spi_rx);

  /* SPI_TX Init */
  hdma_sm_spi_tx.Instance = SM_SPI_TX_DMA_CHANNEL;
  hdma_sm_spi_tx.Init.Request = SM_SPI_TX_DMA_REQUEST;
  hdma_sm_spi_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_sm_spi_tx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_sm_spi_tx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_sm_spi_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_sm_spi_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_sm_spi_tx.Init.Mode = DMA_NORMAL;
  hdma_sm_spi_tx.Init.Priority = DMA_PRIORITY_LOW;
  if (HAL_DMA_Init(&hdma_sm_spi_tx) != HAL_OK)
  {
    SM_Error_Handler();
  }

  __HAL_LINKDMA(hspi, hdmatx, hdma_sm_spi_tx);
}

/**
  * @brief I2C MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hi2c: I2C handle pointer
  * @retval None
  */
static void SM_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  SM_I2C_SCL_PIN_CLK_ENABLE();
  SM_I2C_SDA_PIN_CLK_ENABLE();

  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  GPIO_InitStruct.Pin = SM_I2C_SCL_PIN;
  GPIO_InitStruct.Alternate = SM_I2C_SCL_AF;
  HAL_GPIO_Init(SM_I2C_SCL_GPIO_PORT, &GPIO_InitStruct);


  GPIO_InitStruct.Pin = SM_I2C_SDA_PIN;
  GPIO_InitStruct.Alternate = SM_I2C_SDA_AF;
  HAL_GPIO_Init(SM_I2C_SDA_GPIO_PORT, &GPIO_InitStruct);

  /* Peripheral clock enable */
  SM_I2Cx_CLK_ENABLE();

  /* I2C DMA Init */
  /* I2C_RX Init */
  hdma_sm_i2c_rx.Instance = SM_I2C_RX_DMA_CHANNEL;
  hdma_sm_i2c_rx.Init.Request = SM_I2C_RX_DMA_REQUEST;
  hdma_sm_i2c_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_sm_i2c_rx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_sm_i2c_rx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_sm_i2c_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_sm_i2c_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_sm_i2c_rx.Init.Mode = DMA_NORMAL;
  hdma_sm_i2c_rx.Init.Priority = DMA_PRIORITY_HIGH;
  if (HAL_DMA_Init(&hdma_sm_i2c_rx) != HAL_OK)
  {
    SM_Error_Handler();
  }

  __HAL_LINKDMA(hi2c, hdmarx, hdma_sm_i2c_rx);

  /* I2C_TX Init */
  hdma_sm_i2c_tx.Instance = SM_I2C_TX_DMA_CHANNEL;
  hdma_sm_i2c_tx.Init.Request = SM_I2C_TX_DMA_REQUEST;
  hdma_sm_i2c_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_sm_i2c_tx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_sm_i2c_tx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_sm_i2c_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_sm_i2c_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_sm_i2c_tx.Init.Mode = DMA_NORMAL;
  hdma_sm_i2c_tx.Init.Priority = DMA_PRIORITY_HIGH;
  if (HAL_DMA_Init(&hdma_sm_i2c_tx) != HAL_OK)
  {
    SM_Error_Handler();
  }

  __HAL_LINKDMA(hi2c, hdmatx, hdma_sm_i2c_tx);

  /* I2C2 interrupt Init */
  HAL_NVIC_SetPriority(I2C2_EV_IRQn, 4, 0); /*defines*/
  HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);
  HAL_NVIC_SetPriority(I2C2_ER_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);
}

double SM_GetTimeStamp(void)
{
  uint32_t tim_counter;
  uint64_t timestamp;
  double timestamp_seconds;

  /* -------- Start critical section -----------*/
  taskENTER_CRITICAL();

  tim_counter = hsm_tim.Instance->CNT;
  if (__HAL_TIM_GET_FLAG(&hsm_tim, TIM_FLAG_UPDATE)) /* Update Event happened while already in critical section */
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

  timestamp_seconds = ((double)timestamp / (double)SystemCoreClock);

  /* ---------- */

  taskEXIT_CRITICAL();
  /* -------- End critical section -----------*/
  return timestamp_seconds;
}

double SM_GetTimeStamp_fromISR(void)
{
  uint32_t tim_counter;
  uint64_t timestamp;
  double timestamp_seconds;
  uint32_t isr_mask;

  /* -------- Start critical section -----------*/
  isr_mask = taskENTER_CRITICAL_FROM_ISR();

  tim_counter = hsm_tim.Instance->CNT;
  if (__HAL_TIM_GET_FLAG(&hsm_tim, TIM_FLAG_UPDATE)) /* Update Event happened while already in critical section */
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

  timestamp_seconds = ((double)timestamp / (double)SystemCoreClock);

  taskEXIT_CRITICAL_FROM_ISR(isr_mask);
  /* -------- End critical section -----------*/

  return timestamp_seconds;
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



void SM_TIM_Init(void)
{
  SM_TIMx_CLK_ENABLE();
  /* Set TIMx instance */
  hsm_tim.Instance = SM_TIMx;

  /* Initialize TIMx peripheral as follows:
  + Period = TimerClock * SM_TS_UPDATE_PERIOD_S - 1
  + Prescaler = 0
  + ClockDivision = 0
  + Counter direction = Up
  */
  hsm_tim.Init.Period            = (SystemCoreClock * SM_TS_UPDATE_PERIOD_S) - 1; /* 10s */
  hsm_tim.Init.Prescaler         = 0;
  hsm_tim.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  hsm_tim.Init.CounterMode       = TIM_COUNTERMODE_UP;
  hsm_tim.Init.RepetitionCounter = 0;
  hsm_tim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  if (HAL_TIM_Base_Init(&hsm_tim) != HAL_OK)
  {
    /* Initialization Error */
  }

  HAL_TIM_RegisterCallback(&hsm_tim, HAL_TIM_PERIOD_ELAPSED_CB_ID, SM_TIM_PeriodElapsedCallback);

  /* TIM5 interrupt Init */
  HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM5_IRQn);
}

void SM_TIM_Start(void)
{
  /*##-2- Start the TIM Base generation ####################*/
  if (HAL_TIM_Base_Start_IT(&hsm_tim) != HAL_OK)
  {
    /* Starting Error */
  }
}

void SM_TIM_Stop(void)
{
  /*##-2- Start the TIM Base generation ####################*/
  if (HAL_TIM_Base_Stop(&hsm_tim) != HAL_OK)
  {
    /* Starting Error */
  }
  hsm_tim.Instance->CNT = 0;
  SM_TimeStamp = 0;
}


/**
  * @brief  Stop all active sensor threads, reset first_dataReady, Stop SM Timer and Tags Timer
  * @param  None
  * @retval 0: no error
  */
uint8_t SM_StopSensorAcquisition(void)
{
  uint32_t sID = 0;
  uint32_t ssID = 0;
  COM_DeviceDescriptor_t *pDeviceDescriptor;
  COM_SensorDescriptor_t *pSensorDescriptor;
  COM_SubSensorStatus_t *pSubSensorStatus;

  pDeviceDescriptor = COM_GetDeviceDescriptor();
  HSD_TAGS_timer_stop();

  for (sID = 0; sID < pDeviceDescriptor->nSensor; sID++)
  {
    pSensorDescriptor = COM_GetSensorDescriptor(sID);

    for (ssID = 0; ssID < pSensorDescriptor->nSubSensors; ssID++)
    {
      pSubSensorStatus = COM_GetSubSensorStatus(sID, ssID);

      if (pSubSensorStatus->isActive == 1)
      {
        SM_StopSensorThread(sID);
        COM_GetSubSensorContext(sID, ssID)->first_dataReady = 0;
      }
    }
  }

  /* Stop Sensor Manager Timer */
  SM_TIM_Stop();

  return 0;
}

/**
  * @brief  Start sensor thread
  * @param  id: Sensor id
  * @retval 0: no error
  */
uint8_t SM_StartSensorThread(uint8_t sId)
{
  if (sId == IIS3DWB_Get_Id())
  {
    IIS3DWB_Start();
  }
  else if (sId == HTS221_Get_Id())
  {
    HTS221_Start();
  }
  else if (sId == IIS2DH_Get_Id())
  {
    IIS2DH_Start();
  }
  else if (sId == IIS2MDC_Get_Id())
  {
    IIS2MDC_Start();
  }
  else if (sId == IMP34DT05_Get_Id())
  {
    IMP34DT05_Start();
  }
  else if (sId == IMP23ABSU_Get_Id())
  {
    IMP23ABSU_Start();
  }
  else if (sId == ISM330DHCX_Get_Id())
  {
    ISM330DHCX_Start();
  }
  else if (sId == LPS22HH_Get_Id())
  {
    LPS22HH_Start();
  }
  else if (sId == STTS751_Get_Id())
  {
    STTS751_Start();
  }
  return 0;
}


/**
  * @brief  Stop sensor thread
  * @param  id: Sensor id
  * @retval 0: no error
  */
uint8_t SM_StopSensorThread(uint8_t sId)
{
  if (sId == IIS3DWB_Get_Id())
  {
    IIS3DWB_Stop();
  }
  else if (sId == HTS221_Get_Id())
  {
    HTS221_Stop();
  }
  else if (sId == IIS2DH_Get_Id())
  {
    IIS2DH_Stop();
  }
  else if (sId == IIS2MDC_Get_Id())
  {
    IIS2MDC_Stop();
  }
  else if (sId == IMP34DT05_Get_Id())
  {
    IMP34DT05_Stop();
  }
  else if (sId == IMP23ABSU_Get_Id())
  {
    IMP23ABSU_Stop();
  }
  else if (sId == ISM330DHCX_Get_Id())
  {
    ISM330DHCX_Stop();
  }
  else if (sId == LPS22HH_Get_Id())
  {
    LPS22HH_Stop();
  }
  else if (sId == STTS751_Get_Id())
  {
    STTS751_Stop();
  }
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


