/**
  ******************************************************************************
  * @file    ble_comm_manager.c
  * @author  SRA
  *
  *
  * @brief   This file provides a set of functions to handle ble communication
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
#include "ble_comm_manager.h"
#include "ble_config_service.h"
#include <stdio.h>
#include "main.h"
#include "hci_tl_interface.h"
#include "hci_tl.h"
#include "hci.h"
#include "ble_status.h"
#include "bluenrg1_types.h"
#include "sensors_manager.h"
#include "com_manager.h"
#include "HSDCore.h"
#include "HSD_json.h"
#include "ble_comm_transfer_protocol.h"
#include "bluenrg1_gatt_aci.h"
#include "cpu_utils.h"
#include "ism330dhcx_app.h"
#include "OTA.h"
#include "STWIN_bc.h"
#include "STWIN_sd.h"

/* Private includes ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

#ifndef MIN
#define MIN(a,b)            ((a) < (b) )? (a) : (b)
#endif /* MIN */

/* Private variables ---------------------------------------------------------*/
extern uint8_t connected;
extern uint16_t configServiceHandle;
extern uint16_t configTxCharHandle;
extern uint16_t MLCCharHandle;
extern uint16_t service_handle;
extern uint16_t dev_name_char_handle;
extern uint16_t consoleServiceHandle;
extern uint16_t consoleTermCharHandle;

extern uint16_t MaxBLECharLen;
extern uint16_t MaxBLECharLen_FFOTA;

extern uint8_t ble_init;
SPI_HandleTypeDef hble_cm_spi;

extern uint8_t SD_Logging_Active;

extern volatile char BoardName[8];
extern volatile uint32_t OTA_crc;

extern ism330dhcx_mlc_status_mainpage_t mlc_status;

/* Private function prototypes -----------------------------------------------*/

static uint32_t BLE_CM_ConfigConsole_SendBuffer(uint8_t *buffer, uint32_t len);
static uint32_t BLE_CM_ConfigConsole_BuildResponse(uint32_t comRequest, char **pSerializedJson,
                                                   int32_t *serializedJsonSize);

static uint32_t BLE_CM_DebugConsole_SendBuffer(uint8_t *buffer, uint32_t len);
static uint32_t BLE_CM_DebugConsole_BuildResponse(uint32_t comRequest);

static uint32_t BLE_CM_DebugConsole_SendInfo(void);

static void sendMLCtoBLE(void);
static tBleStatus MLC_Update(uint8_t *mlc_out, uint8_t *mlc_status_mainpage);


static void BLE_CM_SPI_MspInit(SPI_HandleTypeDef *hspi);


static void ble_user_evt_proc_Thread(void const *argument);
osThreadId bleUserEvtProcThreadId;

static void ble_send_Thread(void const *argument);
osThreadId bleSendThreadId;

static void ble_init_Thread(void const *argument);
osThreadId bleInitThreadId;

void bleSendPerformanceStatusCallback(void const *argument);

#if (HSD_BLE_STATUS_TIMER_ENABLE == 1)
osTimerId bleSendPerformanceStatusTim_id;
osTimerDef(bleSendPerformanceStatusTim, bleSendPerformanceStatusCallback);
#endif /* (HSD_BLE_STATUS_TIMER_ENABLE == 1) */

void bleAdvUpdaterTim_Callback(void const *argument);

osTimerId bleAdvUpdaterTim_id;
osTimerDef(bleAdvUpdaterTim, bleAdvUpdaterTim_Callback);

osSemaphoreId bleInitThreadSem_id;
osSemaphoreDef(bleInitThreadSem);

osSemaphoreId bleUserEvtProcThreadSem_id;
osSemaphoreDef(bleUserEvtProcThreadSem);

osMessageQId bleSendThreadQueue_id;
osMessageQDef(bleSendThreadQueue, 1, uint32_t);

osSemaphoreId bleCongestionSem_id;
osSemaphoreDef(bleCongestionSem);

/**
  * @brief Sensor manager SPI Initialization Function
  * @param None
  * @retval None
  * @note callbacks to the MSP
  */
void BLE_CM_SPI_Init(void)
{
  /* SPI3 parameter configuration*/
  hble_cm_spi.Instance = BLE_CM_SPI_x;
  hble_cm_spi.Init.Mode = SPI_MODE_MASTER;
  hble_cm_spi.Init.Direction = SPI_DIRECTION_2LINES;
  hble_cm_spi.Init.DataSize = SPI_DATASIZE_8BIT;
  hble_cm_spi.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hble_cm_spi.Init.CLKPhase = SPI_PHASE_2EDGE;
  hble_cm_spi.Init.NSS = SPI_NSS_SOFT;
  hble_cm_spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hble_cm_spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hble_cm_spi.Init.TIMode = SPI_TIMODE_DISABLE;
  hble_cm_spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hble_cm_spi.Init.CRCPolynomial = 7;
  hble_cm_spi.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hble_cm_spi.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;

  /* Register MSP Callback */
  HAL_SPI_RegisterCallback(&hble_cm_spi, HAL_SPI_MSPINIT_CB_ID, BLE_CM_SPI_MspInit);

  if (HAL_SPI_Init(&hble_cm_spi) != HAL_OK)
  {
    SM_Error_Handler();
  }
}

void BLE_CM_SPI_DeInit(void)
{
  HAL_GPIO_DeInit(BLE_CM_SPI_EXTI_PORT, BLE_CM_SPI_EXTI_PIN);
  HAL_GPIO_DeInit(BLE_CM_SPI_CS_PORT, BLE_CM_SPI_CS_PIN);
  HAL_GPIO_DeInit(BLE_CM_RST_PORT, BLE_CM_RST_PIN);
}

void BLE_CM_SPI_Reset(void)
{
  /* Deselect CS PIN for BlueNRG to avoid spurious commands */
  HAL_GPIO_WritePin(BLE_CM_SPI_CS_PORT, BLE_CM_SPI_CS_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BLE_CM_RST_PORT, BLE_CM_RST_PIN, GPIO_PIN_RESET);
  HAL_Delay(5);
  HAL_GPIO_WritePin(BLE_CM_RST_PORT, BLE_CM_RST_PIN, GPIO_PIN_SET);
  HAL_Delay(5);
}


/**
  * @brief Sensor manager OS functionalities initialization: for each BUS (I2C and SPI) it
  *        initializes a queue to collect read request, a thread (blocking on the queue) to handle
  *        read requests and a semaphore used to wait for DMA transfer complete
  * @param None
  * @retval None
  */
void BLE_CM_OS_Init(void)
{
  /* Bus read semaphores */
  bleInitThreadSem_id = osSemaphoreCreate(osSemaphore(bleInitThreadSem), 1);
  osSemaphoreWait(bleInitThreadSem_id, osWaitForever);

  bleUserEvtProcThreadSem_id = osSemaphoreCreate(osSemaphore(bleUserEvtProcThreadSem), 1);
  osSemaphoreWait(bleUserEvtProcThreadSem_id, osWaitForever);

  bleSendThreadQueue_id = osMessageCreate(osMessageQ(bleSendThreadQueue), NULL);
  vQueueAddToRegistry(bleSendThreadQueue_id, "bleSendThreadQueue_id");

  bleCongestionSem_id = osSemaphoreCreate(osSemaphore(bleCongestionSem), 1);
  osSemaphoreWait(bleCongestionSem_id, osWaitForever);

#if (HSD_BLE_STATUS_TIMER_ENABLE == 1)
  /* BLE Performance Status send Timer*/
  bleSendPerformanceStatusTim_id = osTimerCreate(osTimer(bleSendPerformanceStatusTim), osTimerPeriodic, NULL);
#endif /* (HSD_BLE_STATUS_TIMER_ENABLE == 1) */

  /* BLE Performance Status send Timer*/
  bleAdvUpdaterTim_id = osTimerCreate(osTimer(bleAdvUpdaterTim), osTimerPeriodic, NULL);

  /* BLE init Thread*/
  osThreadDef(BLE_INIT_THREAD, ble_init_Thread, BLE_INIT_THREAD_PRIO, 1, 1300 / 4);

  /* BLE init read Thread */
  bleInitThreadId = osThreadCreate(osThread(BLE_INIT_THREAD), NULL);

  /* BLE send Thread*/
  osThreadDef(BLE_SEND_THREAD, ble_send_Thread, BLE_SEND_THREAD_PRIO, 1, 2048 / 4);

  /* BLE send Thread */
  bleSendThreadId = osThreadCreate(osThread(BLE_SEND_THREAD), NULL);

  /* Task to process user events*/
  osThreadDef(BLE_USER_EVT_PROC_THREAD, ble_user_evt_proc_Thread, BLE_USER_EVT_PROC_THREAD_PRIO, 1, 2400 / 4);

  /* Start User Events process Thread */
  bleUserEvtProcThreadId = osThreadCreate(osThread(BLE_USER_EVT_PROC_THREAD), NULL);
}


/**
  * @brief  Send and Receive data to/from SPI BUS (Full duplex)
  * @param  pData: Data
  * @param  len: Length of data in byte
  * @retval BSP status
  */
int32_t BLE_CM_SPI_SendRecv(uint8_t *pTxData, uint8_t *pRxData, uint16_t len)
{
  int32_t ret = -1;

  if (HAL_SPI_TransmitReceive(&hble_cm_spi, pTxData, pRxData, len, 1000) == HAL_OK)
  {
    ret = (int32_t)len;
  }
  return ret;
}


int32_t BLE_CM_SPI_Write(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
  taskENTER_CRITICAL();

  taskEXIT_CRITICAL();
  return 0;
}

/**
  * @brief Sensor Manager Blocking SPI read function
  * @param None
  * @retval None
  * @note This function can be liked to the sensor PID if freeRTOS is not used
  */
int32_t BLE_CM_SPI_Read(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
  return 0;
}


/* Performance Status Callback function */
void bleSendPerformanceStatusCallback(void const *argument)
{
  osMessagePut(bleSendThreadQueue_id, COM_REQUEST_STATUS_PERFORMANCE, 0);
}

/**
  * @brief  BLE receiver thread
  * @param  argument not used
  * @retval None
  */
static void ble_init_Thread(void const *argument)
{
  (void)argument;

#if (configUSE_APPLICATION_TASK_TAG == 1 && defined(TASK_BLE_INIT_DEBUG_PIN))
  vTaskSetApplicationTaskTag(NULL, (TaskHookFunction_t)TASK_BLE_INIT_DEBUG_PIN);
#endif /* (configUSE_APPLICATION_TASK_TAG == 1 && defined(TASK_BLE_INIT_DEBUG_PIN)) */

  for (;;)
  {
    if (ble_init == 1)
    {
      osSemaphoreWait(bleInitThreadSem_id, osWaitForever);
    }
    ble_interface_init();
  }
}

/**
  * @brief  BLE User Events process thread
  * @param  argument not used
  * @retval None
  */
static void ble_user_evt_proc_Thread(void const *argument)
{
  (void)argument;

#if (configUSE_APPLICATION_TASK_TAG == 1 && defined(TASK_BLE_USER_EVT_PROC_DEBUG_PIN))
  vTaskSetApplicationTaskTag(NULL, (TaskHookFunction_t)TASK_BLE_USER_EVT_PROC_DEBUG_PIN);
#endif /* (configUSE_APPLICATION_TASK_TAG == 1 && defined(TASK_BLE_USER_EVT_PROC_DEBUG_PIN)) */

  for (;;)
  {
    osSemaphoreWait(bleUserEvtProcThreadSem_id, osWaitForever);

    if (ble_init)
    {
      hci_user_evt_proc();
    }
  }
}

void aci_gatt_tx_pool_available_event(uint16_t Connection_Handle, uint16_t Available_Buffers)
{
  osSemaphoreRelease(bleCongestionSem_id);
}



/**
  * @brief  BLE send thread
  * @param  argument not used
  * @retval None
  */
static void ble_send_Thread(void const *argument)
{
  (void)argument;

#if (configUSE_APPLICATION_TASK_TAG == 1 && defined(TASK_BLE_SEND_DEBUG_PIN))
  vTaskSetApplicationTaskTag(NULL, (TaskHookFunction_t)TASK_BLE_SEND_DEBUG_PIN);
#endif /* (configUSE_APPLICATION_TASK_TAG == 1 && defined(TASK_BLE_SEND_DEBUG_PIN)) */

  osEvent evt;

  for (;;)
  {
    evt = osMessageGet(bleSendThreadQueue_id, osWaitForever);  /* wait for message */

    if (connected == TRUE)
    {
      if (evt.status == osEventMessage)
      {
        if ((evt.value.v & BLE_COMMAND_MASK) == BLE_COMMAND_HSD_PROTOCOL)
        {
          uint32_t bufferSize = 0;
          int32_t serializedJsonSize = 0;
          uint32_t wTP_size = 0;
          char *pSerializedJson = NULL;
          uint32_t subCommand = evt.value.v & BLE_SUB_COMMAND_MASK;

          BLE_CM_ConfigConsole_BuildResponse(subCommand, &pSerializedJson, &serializedJsonSize);

          if (serializedJsonSize % (MaxBLECharLen - 1) == 0)
          {
            wTP_size = (serializedJsonSize / (MaxBLECharLen - 1)) + serializedJsonSize;
          }
          else
          {
            wTP_size = (serializedJsonSize / (MaxBLECharLen - 1)) + 1 + serializedJsonSize;
          }

          uint8_t *pBuffer = HSD_malloc(sizeof(uint8_t) * wTP_size);
          if (pBuffer == NULL)
          {
            HSD_PRINTF("Mem alloc error [%ld]: %d@%s\r\n", wTP_size, __LINE__, __FILE__);
          }
          else
          {
            HSD_PRINTF("Mem alloc ok [%ld]: %d@%s\r\n", wTP_size, __LINE__, __FILE__);
          }

          bufferSize = BLECommand_TP_Encapsulate(pBuffer, (uint8_t *) pSerializedJson, (uint16_t) serializedJsonSize);

          BLE_CM_ConfigConsole_SendBuffer(pBuffer, bufferSize);

          HSD_free(pSerializedJson);
          HSD_free(pBuffer);

        }
        else if ((evt.value.v & BLE_COMMAND_MASK) == BLE_COMMAND_DEBUG_CONSOLE)
        {
          uint32_t subCommand = evt.value.v & BLE_SUB_COMMAND_MASK;

          BLE_CM_DebugConsole_BuildResponse(subCommand);
        }
        else if ((evt.value.v & BLE_COMMAND_MASK) == BLE_COMMAND_MLC)
        {
          sendMLCtoBLE();
        }
      }
    }
  }
}



static uint32_t BLE_CM_DebugConsole_BuildResponse(uint32_t comRequest)
{
  switch (comRequest)
  {
    case BLE_SUB_CMD_BOARD_NAME:
    {
      aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0, strlen((char *) BoardName), (uint8_t *) BoardName);
      break;
    }
    case BLE_SUB_CMD_FW_VERSION:
    {
      uint8_t buffer[50];
      uint32_t size;
      size = sprintf((char *) buffer, "%s_%s_%c.%c.%c\r\n", "L4R9", "HSD",
                     HSD_VERSION_MAJOR,
                     HSD_VERSION_MINOR, HSD_VERSION_PATCH);
      BLE_CM_DebugConsole_SendBuffer(buffer, size);
      break;
    }
    case BLE_SUB_CMD_INFO:
    {
      BLE_CM_DebugConsole_SendInfo();
      break;
    }
    case BLE_SUB_CMD_FOTA_START:
    {
      uint32_t crc = OTA_crc;
      BLE_CM_DebugConsole_SendBuffer((uint8_t *) &crc, sizeof(crc));
      break;
    }
    case BLE_SUB_CMD_FOTA_COMPLETED:
    {
      int8_t response = 1;
      aci_gatt_update_char_value(consoleServiceHandle, consoleTermCharHandle, 0, 1, (uint8_t *)&response);
      HAL_Delay(100);
      EnableDisableDualBoot();
      break;
    }
    case BLE_SUB_CMD_FOTA_ERROR:
    {
      int8_t response = -1;
      aci_gatt_update_char_value(consoleServiceHandle, consoleTermCharHandle, 0, 1, (uint8_t *)&response);
      break;
    }
    default:
    {
      break;
    }
  }
  return 0;
}


static uint32_t BLE_CM_DebugConsole_SendInfo(void)
{
  uint8_t buffer[150];
  uint32_t size;
  size = sprintf((char *) buffer, "\r\nSTMicroelectronics %s:\n"
                 "\tVersion %c.%c.%c\n"
                 "\tSTM32L4R9ZI-STWIN board"
                 "\n",
                 "FP-SNS-DATALOG1", HSD_VERSION_MAJOR, HSD_VERSION_MINOR, HSD_VERSION_PATCH);

  BLE_CM_DebugConsole_SendBuffer(buffer, size);

  size = sprintf((char *) buffer, "\t(HAL %ld.%ld.%ld_%ld)\n"
                 "\tCompiled %s %s"
#if defined (__IAR_SYSTEMS_ICC__)
                 " (IAR)\n",
#elif defined (__CC_ARM)
                 " (KEIL)\n",
#elif defined (__GNUC__)
                 " (STM32CubeIDE)\n",
#endif /* IDE */
                 HAL_GetHalVersion() >> 24,
                 (HAL_GetHalVersion() >> 16) & 0xFF, (HAL_GetHalVersion() >> 8) & 0xFF, HAL_GetHalVersion() & 0xFF,
                 __DATE__,
                 __TIME__);
  BLE_CM_DebugConsole_SendBuffer(buffer, size);

  size = sprintf((char *) buffer, "Current Bank =%d\r\n", CurrentActiveBank);

  BLE_CM_DebugConsole_SendBuffer(buffer, size);

  return 0;
}


static uint32_t BLE_CM_DebugConsole_SendBuffer(uint8_t *buffer, uint32_t len)
{
  uint32_t j = 0;
  uint32_t ret = 0;
  uint32_t chunk;
  /* Data are sent as notifications*/
  while (j < len)
  {
    chunk = MIN(MaxBLECharLen_FFOTA, len - j);
    ret = aci_gatt_update_char_value(consoleServiceHandle, consoleTermCharHandle, 0, chunk, (uint8_t *) &buffer[j]);
    if (ret == BLE_STATUS_INSUFFICIENT_RESOURCES)
    {
      osSemaphoreWait(bleCongestionSem_id, osWaitForever);
    }
    else if (ret == BLE_STATUS_SUCCESS)
    {
      j += chunk;
    }
  }
  return ret;
}



static uint32_t BLE_CM_ConfigConsole_BuildResponse(uint32_t comRequest, char **pSerializedJson, int32_t *size)
{
  int32_t serializedJsonSize = 0;

  switch (comRequest)
  {
    case COM_REQUEST_DEVICE :
    {
      if (AMTIsStarted())
      {
        serializedJsonSize = HSD_JSON_serialize_FWStatus_Logging(pSerializedJson, BSP_SD_IsDetected(), 1);
      }
      else if (SD_Logging_Active == 0)
      {
        COM_Device_t *pDevice = COM_GetDevice();
        serializedJsonSize = HSD_JSON_serialize_Device(pDevice, pSerializedJson, SHORT_JSON);
      }
      else
      {
        serializedJsonSize = HSD_JSON_serialize_FWStatus_Logging(pSerializedJson, BSP_SD_IsDetected(), SD_Logging_Active);
      }
      break;
    }
    case COM_REQUEST_DEVICEREFRESH :
    {
      COM_Device_t *pDevice = COM_GetDevice();
      serializedJsonSize = HSD_JSON_serialize_Device(pDevice, pSerializedJson, SHORT_JSON);
      break;
    }
    case COM_REQUEST_DEVICE_INFO :
    {
      COM_DeviceDescriptor_t *pDeviceDescriptor = COM_GetDeviceDescriptor();
      serializedJsonSize = HSD_JSON_serialize_DeviceInfo(pDeviceDescriptor, pSerializedJson);
      break;
    }
    case COM_REQUEST_DESCRIPTOR :
    {
      break;
    }
    case COM_REQUEST_STATUS :
    {
      break;
    }
    case COM_REQUEST_REGISTER :
    {
      break;
    }
    case COM_REQUEST_STATUS_PERFORMANCE :
    {
      uint32_t mV = 0;
      uint32_t level = 0;
      stbc02_State_TypeDef BC_State;
      uint16_t cpu_usage;

      BSP_BC_GetVoltageAndLevel(&mV, &level);
      BSP_BC_GetState(&BC_State);
      cpu_usage = osGetCPUUsage();
      serializedJsonSize = HSD_JSON_serialize_FWStatus_Performance(pSerializedJson, (char *)&BC_State.Name, mV, level, cpu_usage);
      break;
    }
    case COM_REQUEST_STATUS_LOGGING :
    {
      if (AMTIsStarted())
      {
        serializedJsonSize = HSD_JSON_serialize_FWStatus_Logging(pSerializedJson, BSP_SD_IsDetected(), 1);
      }
      else
      {
        serializedJsonSize = HSD_JSON_serialize_FWStatus_Logging(pSerializedJson, BSP_SD_IsDetected(), SD_Logging_Active);
      }
      break;
    }
    case COM_REQUEST_STATUS_NETWORK :
    {
      serializedJsonSize = HSD_JSON_serialize_FWStatus_Network(pSerializedJson, "ssid", "password", "ip");
      break;
    }
    case COM_REQUEST_TAG_CONFIG :
    {
      COM_TagList_t *pTagList = COM_GetTagList();
      serializedJsonSize = HSD_JSON_serialize_TagList(pTagList, pSerializedJson, SHORT_JSON);
      break;
    }
    default :
    {
      if ((comRequest & COM_REQUEST_SENSORREFRESH) == COM_REQUEST_SENSORREFRESH)
      {
        uint8_t mask = comRequest & COM_REQUEST_SENSORREFRESH;
        uint8_t sensorId = comRequest - mask;
        COM_SensorStatus_t *pSensorStatus = COM_GetSensorStatus(sensorId);
        serializedJsonSize = HSD_JSON_serialize_RefreshSensorStatus(sensorId, pSensorStatus, pSerializedJson);
      }
      break;
    }
  }
  *size = serializedJsonSize;
  return 0;
}


static uint32_t BLE_CM_ConfigConsole_SendBuffer(uint8_t *buffer, uint32_t len)
{
  uint32_t j = 0;
  uint32_t ret = 0;
  uint32_t chunk;
  /* Data are sent as notifications*/
  while (j < len)
  {
    chunk = MIN(MaxBLECharLen, len - j);
    ret = aci_gatt_update_char_value(configServiceHandle, configTxCharHandle, 0, chunk, (uint8_t *) &buffer[j]);
    if (ret == BLE_STATUS_INSUFFICIENT_RESOURCES)
    {
      osSemaphoreWait(bleCongestionSem_id, osWaitForever);
    }
    else if (ret == BLE_STATUS_SUCCESS)
    {
      j += chunk;
    }
  }
  return ret;
}



void sendMLCtoBLE(void)
{
  uint8_t mlcOut[8];

  GetMLCOut(mlcOut);
  MLC_Update(mlcOut, (uint8_t *)&mlc_status);
}


#define STORE_LE_16(buf, val)    ( ((buf)[0] =  (uint8_t) (val)    ) , \
                                   ((buf)[1] =  (uint8_t) (val>>8) ) )

/**
  * @brief  Update Machine Learning Core output registers
  * @param  uint8_t *mlc_out pointers to 8 MLC register
  * @param  uint8_t *mlc_status_mainpage pointer to the MLC status bits from 1 to 8
  * @retval tBleStatus Status
  */
tBleStatus MLC_Update(uint8_t *mlc_out, uint8_t *mlc_status_mainpage)
{
  tBleStatus ret;
  uint8_t buff[2 + 8 + 1];

  /* TimeStamp */
  STORE_LE_16(buff, (HAL_GetTick() >> 3));
  /* MLC outputs registers */
  memcpy(buff + 2, mlc_out, 8 * sizeof(uint8_t));
  /* Status bit for MLC from 1 to 8   */
  buff[10] = *mlc_status_mainpage;

  ret = aci_gatt_update_char_value(configServiceHandle, MLCCharHandle, 0, 2 + 8 + 1, buff);

  return ret;
}

static void BLE_CM_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct =
  {
    0
  };

  BLE_CM_SPI_CLK_PIN_CLK_ENABLE();
  BLE_CM_SPI_MISO_PIN_CLK_ENABLE();
  BLE_CM_SPI_MOSI_PIN_CLK_ENABLE();

  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  GPIO_InitStruct.Alternate = BLE_CM_SPI_CLK_AF;
  GPIO_InitStruct.Pin = BLE_CM_SPI_CLK_PIN;
  HAL_GPIO_Init(BLE_CM_SPI_CLK_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Alternate = BLE_CM_SPI_MISO_AF;
  GPIO_InitStruct.Pin = BLE_CM_SPI_MISO_PIN;
  HAL_GPIO_Init(BLE_CM_SPI_MISO_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Alternate = BLE_CM_SPI_MOSI_AF;
  GPIO_InitStruct.Pin = BLE_CM_SPI_MOSI_PIN;
  HAL_GPIO_Init(BLE_CM_SPI_MOSI_GPIO_PORT, &GPIO_InitStruct);

  BLE_CM_SPIx_CLK_ENABLE();

  /* Enable GPIO Ports Clock */
  __GPIOA_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
  __GPIOG_CLK_ENABLE();

  /* Enable SPI clock */
  __SPI2_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWREx_EnableVddUSB();
  HAL_PWREx_EnableVddIO2();

  /*Configure EXTI Line */
  GPIO_InitStruct.Pin = BLE_CM_SPI_EXTI_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BLE_CM_SPI_EXTI_PORT, &GPIO_InitStruct);

  /* Register event irq handler */
  HAL_NVIC_SetPriority(BLE_CM_SPI_EXTI_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(BLE_CM_SPI_EXTI_IRQn);

  /*Configure CS & RESET Line */
  GPIO_InitStruct.Pin = BLE_CM_RST_PIN ;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BLE_CM_RST_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = BLE_CM_SPI_CS_PIN ;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BLE_CM_SPI_CS_PORT, &GPIO_InitStruct);
}

/**
  * @brief  Enable SPI IRQ.
  * @param  None
  * @retval None
  */
void BLE_CM_SPI_Enable_IRQ(void)
{
  HAL_NVIC_EnableIRQ(BLE_CM_SPI_EXTI_IRQn);
}

/**
  * @brief  Disable SPI IRQ.
  * @param  None
  * @retval None
  */
void BLE_CM_SPI_Disable_IRQ(void)
{
  HAL_NVIC_DisableIRQ(BLE_CM_SPI_EXTI_IRQn);
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


