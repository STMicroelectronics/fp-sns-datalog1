/**
  ******************************************************************************
  * @file    usbd_wcid_interface.c
  * @brief   Source file for USBD WCID interface
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
#include "usbd_wcid_interface.h"
#include "main.h"
#include "com_manager.h"
#include "HSD_json.h"
#include "HSDCore.h"
#include "OTA.h"

#include "lsm6dsox_app.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

uint8_t *USB_RxBuffer = NULL;
uint8_t *TxBuffer[N_CHANNELS_MAX];
extern USBD_HandleTypeDef USBD_Device;
extern volatile uint8_t HSD_ResetUSB;

extern osTimerId bleAdvUpdaterTim_id;

/* Private function prototypes -----------------------------------------------*/
static int8_t WCID_STREAMING_Itf_Init(void);
static int8_t WCID_STREAMING_Itf_DeInit(void);
static int8_t WCID_STREAMING_Itf_Control(uint8_t isHostToDevice, uint8_t cmd, uint16_t wValue, uint16_t wIndex,
                                         uint8_t *pbuf, uint16_t length);
static int8_t WCID_STREAMING_Itf_Receive(uint8_t *pbuf, uint32_t Len);
void WCID_CalculateUsbWriteBufferSize(COM_SubSensorStatus_t *pSubSensorStatus, uint32_t nBytesPerSample);

static void WCID_Error_Handler(void);

USBD_WCID_STREAMING_ItfTypeDef USBD_WCID_STREAMING_fops =
{
  WCID_STREAMING_Itf_Init,
  WCID_STREAMING_Itf_DeInit,
  WCID_STREAMING_Itf_Control,
  WCID_STREAMING_Itf_Receive
};

/* Private functions ---------------------------------------------------------*/

static uint32_t WCID_STREAMING_Itf_StartStreaming(void);
static uint32_t WCID_STREAMING_Itf_StopStreaming(void);
static uint32_t WCID_STREAMING_Itf_SerializeRequest(COM_Command_t command, char **serialized_json, uint16_t *size);
static uint32_t WCID_STREAMING_Itf_ParseSetRequest(COM_Command_t request, char *serialized_json, uint16_t size);

/**
  * @brief  WCID_STREAMING_Itf_Init
  *         Initializes the WCID media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t WCID_STREAMING_Itf_Init(void)
{
  USB_RxBuffer = HSD_calloc(512, sizeof(uint8_t));
  if (USB_RxBuffer == NULL)
  {
    HSD_PRINTF("Mem alloc error [%d]: %d@%s\r\n", 512, __LINE__, __FILE__);
    return (USBD_FAIL);
  }
  else
  {
    HSD_PRINTF("Mem alloc ok [%d]: %d@%s\r\n", 512, __LINE__, __FILE__);
  }

  USBD_WCID_STREAMING_SetRxDataBuffer(&USBD_Device, (uint8_t *) USB_RxBuffer);
  return (USBD_OK);
}

/**
  * @brief  WCID_STREAMING_Itf_DeInit
  *         DeInitializes the WCID media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t WCID_STREAMING_Itf_DeInit(void)
{
  if (USB_RxBuffer != NULL)
  {
    HSD_free(USB_RxBuffer);
    USB_RxBuffer = NULL;
  }

  return (USBD_OK);
}

/**
  * @brief  WCID_STREAMING_Itf_Control
  *         Manage the WCID class requests
  * @param  isHostToDevice: 1 if the direction o the request is from Host to Device, 0 otherwise
  * @param  cmd: Command code
  * @param  wValue: not used
  * @param  wIndex: not used
  * @param  pBuf: Data Buffer, input for Host-To-Device and output for Device-To-Host
  * @param  length: Buffer size (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t WCID_STREAMING_Itf_Control(uint8_t isHostToDevice, uint8_t cmd, uint16_t wValue, uint16_t wIndex,
                                         uint8_t *pbuf, uint16_t length)
{
  if (com_status != HS_DATALOG_IDLE && com_status != HS_DATALOG_USB_STARTED)
  {
    return USBD_FAIL;
  }

  static uint16_t USB_packet_size = 0;
  static uint16_t counter = 0;
  static char *serialized = 0;
  static char *p = 0;
  static COM_Command_t outCommand;

  /* State for internal Ctrl Endpoint state machine */
  static uint8_t state = USBD_WCID_WAITING_FOR_SIZE;

  if (isHostToDevice)
  {
    switch (state)
    {
      case USBD_WCID_WAITING_FOR_SIZE :

        if (cmd != CMD_SIZE_SET) /* discard if it isn't a size set command */
        {
          return -1;
        }

        USB_packet_size = *(uint16_t *) pbuf;
        serialized = HSD_malloc(USB_packet_size); /* Allocate the buffer to receive next command */
        if (serialized == NULL)
        {
          HSD_PRINTF("Mem alloc error [%d]: %d@%s\r\n", USB_packet_size, __LINE__, __FILE__);
        }
        else
        {
          HSD_PRINTF("Mem alloc ok [%d]: %d@%s\r\n", USB_packet_size, __LINE__, __FILE__);
        }

        p = serialized;
        state = USB_WCID_WAITING_FOR_DATA;
        counter = USB_packet_size;

        break;
      case USB_WCID_WAITING_FOR_DATA :
      {
        uint32_t i = 0;

        if (cmd != CMD_DATA_SET)
        {
          return -1;  /* error */
        }

        for (i = 0; i < length; i++)
        {
          *p++ = pbuf[i];
          counter--;
        }

        if (counter == 0) /* The complete message has been received */
        {
          HSD_JSON_parse_Command((char *) serialized, &outCommand);
          state = USBD_WCID_WAITING_FOR_SIZE_REQUEST;

          if (outCommand.command == COM_COMMAND_SET)
          {
            WCID_STREAMING_Itf_ParseSetRequest(outCommand, serialized, USB_packet_size);

            state = USBD_WCID_WAITING_FOR_SIZE;
          }
          else if (outCommand.command == COM_COMMAND_START)
          {
            COM_AcquisitionDescriptor_t *pAcquisitionDescriptor;
            pAcquisitionDescriptor = COM_GetAcquisitionDescriptor();
            HSD_JSON_parse_StartTime((char *) serialized, pAcquisitionDescriptor);

            WCID_STREAMING_Itf_StartStreaming();

            HSD_JSON_free(serialized);
            state = USBD_WCID_WAITING_FOR_SIZE;
          }
          else if (outCommand.command == COM_COMMAND_STOP)
          {
            COM_AcquisitionDescriptor_t *pAcquisitionDescriptor;
            pAcquisitionDescriptor = COM_GetAcquisitionDescriptor();
            HSD_JSON_parse_EndTime((char *) serialized, pAcquisitionDescriptor);

            WCID_STREAMING_Itf_StopStreaming();

            HSD_JSON_free(serialized);
            state = USBD_WCID_WAITING_FOR_SIZE;
          }
          else if (outCommand.command == COM_COMMAND_SWITCH)
          {
            /* change flash bank */
            EnableDisableDualBoot();
          }
        }
        break;
      }
    }
  }
  else /* Device to host */
  {
    switch (state)
    {
      case USBD_WCID_WAITING_FOR_SIZE_REQUEST : /* Host needs size */
      {
        if (cmd != CMD_SIZE_GET)
        {
          return -1;  /* error*/
        }

        HSD_JSON_free(serialized);

        WCID_STREAMING_Itf_SerializeRequest(outCommand, &serialized, &USB_packet_size);

        *(uint16_t *) pbuf = USB_packet_size;
        p = serialized;

        state = USBD_WCID_WAITING_FOR_DATA_REQUEST;
        counter = USB_packet_size;
        break;
      }
      case USBD_WCID_WAITING_FOR_DATA_REQUEST :
      {
        uint32_t i = 0;

        if (cmd != CMD_DATA_GET)
        {
          return -1;  /* error*/
        }

        for (i = 0; i < length; i++)
        {
          pbuf[i] = *p++;
          counter--;
        }

        if (counter == 0) /* The complete message has been received */
        {
          HSD_JSON_free(serialized);
          serialized = NULL;
          state = USBD_WCID_WAITING_FOR_SIZE;
        }
        break;
      }
    }
  }

  return (USBD_OK);
}

/**
  * @brief  WCID_STREAMING_Itf_Receive
  *         Data received over USB OUT endpoint are sent over WCID interface
  *         through this function.
  * @param  Buf: Buffer of data to be transmitted
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t WCID_STREAMING_Itf_Receive(uint8_t *Buf, uint32_t Len)
{
  return (USBD_OK);
}

/* Private Functions definition ------------------------------------- */

/**
  * @brief  WCID_STREAMING_Itf_StartStreaming
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static uint32_t WCID_STREAMING_Itf_StartStreaming(void)
{
  COM_SubSensorStatus_t *pSubSensorStatus;
  COM_DeviceDescriptor_t *pDeviceDescriptor;
  COM_SensorDescriptor_t *pSensorDescriptor;
  uint32_t sID = 0;
  uint32_t ssID = 0;
  uint32_t nBytesPerSample;

  com_status = HS_DATALOG_USB_STARTED;
  pDeviceDescriptor = COM_GetDeviceDescriptor();
  COM_GenerateAcquisitionUUID();
  SM_TIM_Start();

  osTimerStop(bleAdvUpdaterTim_id);

  uint8_t sensorIsActive;
  for (sID = 0; sID < pDeviceDescriptor->nSensor; sID++)
  {
    sensorIsActive = 0;
    pSensorDescriptor = COM_GetSensorDescriptor(sID);

    for (ssID = 0; ssID < pSensorDescriptor->nSubSensors; ssID++)
    {
      pSubSensorStatus = COM_GetSubSensorStatus(sID, ssID);

      if (pSubSensorStatus->comChannelNumber != -1 && pSubSensorStatus->isActive)
      {
        nBytesPerSample = COM_GetnBytesPerSample(sID, ssID);
        WCID_CalculateUsbWriteBufferSize(pSubSensorStatus, nBytesPerSample);

        sensorIsActive = 1;
        TxBuffer[pSubSensorStatus->comChannelNumber] = NULL;
        TxBuffer[pSubSensorStatus->comChannelNumber] = HSD_calloc((pSubSensorStatus->usbDataPacketSize * 2 + 2),
                                                                  sizeof(uint8_t));

        if (TxBuffer[pSubSensorStatus->comChannelNumber] == NULL)
        {
          HSD_PRINTF("Mem alloc error [%d]: %d@%s\r\n", (pSubSensorStatus->usbDataPacketSize * 2 + 2), __LINE__,
                     __FILE__);
          /* Error */
          WCID_Error_Handler();
        }
        else
        {
          HSD_PRINTF("Mem alloc ok [%d]: %d@%s\r\n", (pSubSensorStatus->usbDataPacketSize * 2 + 2), __LINE__,
                     __FILE__);
        }

        USBD_WCID_STREAMING_SetTxDataBuffer(&USBD_Device, pSubSensorStatus->comChannelNumber,
                                            TxBuffer[pSubSensorStatus->comChannelNumber],
                                            pSubSensorStatus->usbDataPacketSize);
        USBD_WCID_STREAMING_CleanTxDataBuffer(&USBD_Device, pSubSensorStatus->comChannelNumber);

        COM_GetSubSensorContext(sID, ssID)->first_dataReady = 1;
      }
    }
    if (sensorIsActive)
    {
      SM_StartSensorThread(sID);
    }
  }
  USBD_WCID_STREAMING_StartStreaming(&USBD_Device);
  HSD_TAGS_timer_start();

  return USBD_OK;
}

/**
  * @brief  WCID_STREAMING_Itf_StartStreaming
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static uint32_t WCID_STREAMING_Itf_StopStreaming(void)
{
  uint32_t i;
  StopExecutionPhases();

  for (i = 0; i < N_CHANNELS_MAX; i++)
  {
    if (TxBuffer[i] != NULL)
    {
      HSD_free(TxBuffer[i]);
      TxBuffer[i] = NULL;
    }
  }

  osTimerStart(bleAdvUpdaterTim_id, 3000);

  return USBD_OK;
}

/**
  * @brief  WCID_STREAMING_Itf_StartStreaming
  * @param command (input)
  * @param *serialized_json serialized json (output)
  * @param *size size of the serialized json (output)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static uint32_t WCID_STREAMING_Itf_SerializeRequest(COM_Command_t command, char **serialized_json, uint16_t *size)
{
  COM_Device_t *pDevice;
  COM_DeviceDescriptor_t *pDeviceDescriptor;
  COM_SensorDescriptor_t *pSensorDescriptor;
  COM_SubSensorDescriptor_t *pSubSensorDescriptor;
  COM_SensorStatus_t *pSensorStatus;
  COM_SubSensorStatus_t *pSubSensorStatus;
  COM_AcquisitionDescriptor_t *pAcquisitionDescriptor;

  switch (command.request)
  {
    case COM_REQUEST_DEVICE :
    {
      pDevice = COM_GetDevice();
      *size = HSD_JSON_serialize_Device(pDevice, serialized_json, SHORT_JSON);
      break;
    }
    case COM_REQUEST_DEVICE_INFO :
    {
      pDeviceDescriptor = COM_GetDeviceDescriptor();
      *size = HSD_JSON_serialize_DeviceInfo(pDeviceDescriptor, serialized_json);
      break;
    }
    case COM_REQUEST_ACQ_INFO :
    {
      pAcquisitionDescriptor = COM_GetAcquisitionDescriptor();
      *size = HSD_JSON_serialize_Acquisition(pAcquisitionDescriptor, serialized_json, PRETTY_JSON);
      break;
    }
    case COM_REQUEST_DESCRIPTOR :
    {
      if (command.subSensorId < 0) /* Request is for Sensor, since subSensor was not present in the Json */
      {
        pSensorDescriptor = COM_GetSensorDescriptor(command.sensorId);
        *size = HSD_JSON_serialize_SensorDescriptor(pSensorDescriptor, serialized_json);
      }
      else
      {
        pSubSensorDescriptor = COM_GetSubSensorDescriptor(command.sensorId, command.subSensorId);
        *size = HSD_JSON_serialize_SubSensorDescriptor(pSubSensorDescriptor, serialized_json);
      }
      break;
    }
    case COM_REQUEST_STATUS :
    {
      if (command.subSensorId < 0) /* Request is for Sensor, since subSensor was not present in the Json */
      {
        pSensorStatus = COM_GetSensorStatus(command.sensorId);
        *size = HSD_JSON_serialize_SensorStatus(command.sensorId, pSensorStatus, serialized_json);
      }
      else
      {
        pSubSensorStatus = COM_GetSubSensorStatus(command.sensorId, command.subSensorId);
        *size = HSD_JSON_serialize_SubSensorStatus(pSubSensorStatus, serialized_json);
      }
      break;
    }
    case COM_REQUEST_TAG_CONFIG :
    {
      COM_TagList_t *tagConfig = COM_GetTagList();
      *size = HSD_JSON_serialize_TagList(tagConfig, serialized_json, PRETTY_JSON);
      break;
    }
  }

  return USBD_OK;
}


/**
  * @brief  WCID_STREAMING_Itf_Parse_SetRequests
  * @param  command: Command
  * @param  len: buffer len
  * @param  serialized_json: json buffer
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static uint32_t WCID_STREAMING_Itf_ParseSetRequest(COM_Command_t command, char *serialized_json, uint16_t size)
{
  switch (command.request)
  {
    case COM_REQUEST_DEVICE_INFO :
    {
      /* SET device alias */
      char alias[HSD_DEVICE_ALIAS_LENGTH];
      HSD_JSON_parse_SetDeviceAliasCommand((char *) serialized_json, alias, HSD_DEVICE_ALIAS_LENGTH);
      HSD_JSON_free(serialized_json);
      COM_SetDeviceAlias(alias);
      break;
    }
    case COM_REQUEST_SW_TAG :
    {
      /* SET SW Tag (Enable/Disable) */
      uint8_t id;
      HSD_Tags_Enable_t enable;
      double timestamp = SM_GetTimeStamp_fromISR();
      HSD_JSON_parse_EnableTagCommand((char *) serialized_json, &id, &enable);
      HSD_JSON_free(serialized_json);
      HSD_TAGS_add_tag(HSD_TAGS_Type_Sw, id, enable, timestamp);
      break;
    }
    case COM_REQUEST_SW_TAG_LABEL :
    {
      /* SET SW Tag Label */
      uint8_t id;
      char label[HSD_TAGS_LABEL_LENGTH];
      COM_Device_t *device = COM_GetDevice();
      HSD_JSON_parse_UpdateTagLabelCommand((char *) serialized_json, &id, label, HSD_TAGS_LABEL_LENGTH);
      HSD_JSON_free(serialized_json);
      HSD_TAGS_set_tag_label(device, HSD_TAGS_Type_Sw, id, label);
      break;
    }
    case COM_REQUEST_HW_TAG_LABEL :
    {
      /* SET HW Tag Label */
      uint8_t id;
      char label[HSD_TAGS_LABEL_LENGTH];
      COM_Device_t *device = COM_GetDevice();
      HSD_JSON_parse_UpdateTagLabelCommand((char *) serialized_json, &id, label, HSD_TAGS_LABEL_LENGTH);
      HSD_JSON_free(serialized_json);
      HSD_TAGS_set_tag_label(device, HSD_TAGS_Type_Hw, id, label);
      break;
    }
    case COM_REQUEST_HW_TAG :
    {
      /* SET HW Tag (Enable/Disable) */
      uint8_t id;
      HSD_Tags_Enable_t enable;
      COM_Device_t *device = COM_GetDevice();
      HSD_JSON_parse_EnableTagCommand((char *) serialized_json, &id, &enable);
      HSD_JSON_free(serialized_json);
      HSD_TAGS_set_tag_enabled(device, id, enable);
      break;
    }
    case COM_REQUEST_ACQ_INFO :
    {
      /* SET Acquisition Name and Description */
      char name[HSD_ACQ_NAME_LENGTH];
      char notes[HSD_ACQ_NOTES_LENGTH];
      HSD_JSON_parse_AcqInfoCommand((char *) serialized_json, name, HSD_ACQ_NAME_LENGTH, notes, HSD_ACQ_NOTES_LENGTH);
      HSD_JSON_free(serialized_json);
      COM_SetAcquisitionDescriptor(name, notes);
      break;
    }
    case COM_REQUEST_MLC_CONFIG :
    {
      /* Extract loaded ucf size and data */
      uint32_t mlcConfigSize;
      char *mlcConfigData;
      mlcConfigData = (char *) USBD_malloc(size);
      if (mlcConfigData == NULL)
      {
        HSD_PRINTF("Mem alloc error [%d]: %d@%s\r\n", size, __LINE__, __FILE__);
      }
      else
      {
        HSD_PRINTF("Mem alloc ok [%d]: %d@%s\r\n", size, __LINE__, __FILE__);
      }
      HSD_JSON_parse_MlcConfigCommand((char *) serialized_json, &mlcConfigSize, mlcConfigData, size);
      HSD_JSON_free(serialized_json);
      LSM6DSOX_SetUCF(mlcConfigSize, mlcConfigData);
      break;
    }
    default:
    {
      COM_Sensor_t tmpSensor;
      COM_SensorStatus_t *pSensorStatus;

      pSensorStatus = COM_GetSensorStatus(command.sensorId);
      memcpy(&tmpSensor.sensorStatus, pSensorStatus, sizeof(COM_SensorStatus_t));
      HSD_JSON_parse_Status((char *) serialized_json, &tmpSensor.sensorStatus);
      HSD_JSON_free(serialized_json);
      update_sensorStatus_from_USB(pSensorStatus, &tmpSensor.sensorStatus, command.sensorId);

      /* Update the sensor-specific config structure */
      update_sensors_config();
    }
  }
  return USBD_OK;
}

/**
  * @brief  This function is executed in case of error occurrence
  * @param  None
  * @retval None
  */
static void WCID_Error_Handler(void)
{
  com_status = HS_DATALOG_IDLE;
  while (1)
  {
    HAL_Delay(100);
    BSP_LED_On(LED_GREEN);
    BSP_LED_On(LED_RED);
    HAL_Delay(300);
    BSP_LED_Off(LED_GREEN);
    BSP_LED_Off(LED_RED);
  }
}


/**
  * @brief  Calculate UsbWriteBufferSize for each active subSensor.
  *       UsbWriteBufferSize is proportional to subSensor effective baud rate.
  * @param
  * @retval 1: no error
  */
void WCID_CalculateUsbWriteBufferSize(COM_SubSensorStatus_t *pSubSensorStatus, uint32_t nBytesPerSample)
{
  uint32_t bufferSize; /* Amount of data written on SD card for each fwrite */

  if (pSubSensorStatus->ODR != 0)
  {
    /* 500ms of sensor data; when there's a timestamp packets will be sent fastly */
    bufferSize = (uint32_t)(pSubSensorStatus->ODR * nBytesPerSample * 0.5f);

    if (bufferSize > 4096)
    {
      bufferSize = 4096; /* set a limit to avoid buffer to big */
    }
    else if (bufferSize < (uint32_t)(nBytesPerSample + 8))
    {
      /* In case usb_dps is a very low value, verify the setup to send at least 1 sensor data + timestamp */
      bufferSize = (uint32_t)(nBytesPerSample + 8);
    }
  }
  else /* ODR = 0 is used for not sampled subSensors (i.e.: virtual sensors, MLC) */
  {
    bufferSize = nBytesPerSample; /* Buffer size = length of the subSensor output */
    pSubSensorStatus->samplesPerTimestamp = 1; /* write a timestamp for each sample */
  }

  /* check samplesPerTimestamp is valid (set SDM_MAX_WRITE_TIME seconds of data or clip to 1000) */
  if (pSubSensorStatus->samplesPerTimestamp != 0)
  {
    uint32_t samplesPerTS = (uint32_t)(pSubSensorStatus->ODR);
    if (samplesPerTS > MAX_SPTS)
    {
      pSubSensorStatus->samplesPerTimestamp = MAX_SPTS;
    }
    else
    {
      pSubSensorStatus->samplesPerTimestamp = samplesPerTS;
    }
  }
  pSubSensorStatus->usbDataPacketSize = bufferSize;
}


