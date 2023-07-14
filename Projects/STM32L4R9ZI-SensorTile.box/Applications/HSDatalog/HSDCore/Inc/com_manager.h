/**
  ******************************************************************************
  * @file    com_manager.h
  * @author  SRA - MCD
  *
  *
  * @brief   Header for com_manager.c file.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __COM_MANAGER_H
#define __COM_MANAGER_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "ff.h"

/* Package Version only numbers 0->9 */
#define HSD_JSON_VERSION_MAJOR '1'
#define HSD_JSON_VERSION_MINOR '2'
#define HSD_JSON_VERSION_PATCH '0'

/* Version of acquisition file format */
#define HSD_DATA_FILE_EXTENSION       ".dat"
#define HSD_DATA_FILE_FORMAT          "HSD_1.0.0"

#define COM_MAX_SENSORS 10

#define COM_TYPE_ACC    1
#define COM_TYPE_MAG    2
#define COM_TYPE_GYRO   3
#define COM_TYPE_TEMP   4
#define COM_TYPE_PRESS  5
#define COM_TYPE_HUM    6
#define COM_TYPE_MIC    7
#define COM_TYPE_MLC    8

#define N_MAX_DIM_LABELS                    8U
#define DIM_LABELS_LENGTH                   3U

#define N_MAX_SENSOR_COMBO                  4U
#define N_MAX_SUPPORTED_ODR                 16U
#define N_MAX_SUPPORTED_FS                  16U

#define HSD_DEVICE_ALIAS_LENGTH             16U
#define HSD_DEVICE_PNUMBER_LENGTH           17U
#define HSD_DEVICE_URL_LENGTH               32U
#define HSD_DEVICE_FW_NAME_LENGTH           32U
#define HSD_DEVICE_MODEL_LENGTH             17U
#define HSD_DEVICE_FW_VERSION_LENGTH        8U
#define HSD_DEVICE_DATA_FILE_EXT_LENGTH     8U
#define HSD_DEVICE_DATA_FILE_FORMAT_LENGTH  16U
#define HSD_BLE_MAC_ADDRESS_LENGTH          18U

#define HSD_JSON_VERSION_LENGTH             8U

#define HSD_TAGS_PINDESC_LENGTH             8U
#define HSD_TAGS_LABEL_LENGTH               21U
#define HSD_ACQ_NAME_LENGTH                 15U
#define HSD_ACQ_NOTES_LENGTH                100U
#define HSD_TAGS_MAX_SW_CLASSES             5U
#define HSD_TAGS_MAX_HW_CLASSES             5U
#define HSD_TAGS_MAX_PER_ACQUISITION        100U

#define COM_END_OF_LIST_INT -1
#define COM_END_OF_LIST_FLOAT -1.0f

#define COM_LIST_SEPARATOR_INT -2
#define COM_LIST_SEPARATOR_FLOAT -2.0f

/* Commands */
#define CMD_DEVICE                  (uint8_t)(0x00)
#define CMD_DEVICE_HANDLER          (uint8_t)(0x01)
#define CMD_SENSOR_HANDLER          (uint8_t)(0x02)
#define CMD_SENSOR_CONFIG           (uint8_t)(0x03)
#define CMD_SENSOR_STATUS           (uint8_t)(0x04)
#define CMD_SENSOR_UTILITY          (uint8_t)(0x05)
#define CMD_DEVICE_LEN            (uint8_t)(0x06)
#define CMD_DEVICE_HANDLER_LEN    (uint8_t)(0x07)
#define CMD_SENSOR_HANDLER_LEN    (uint8_t)(0x08)
#define CMD_SENSOR_CONFIG_LEN     (uint8_t)(0x09)
#define CMD_SENSOR_STATUS_LEN     (uint8_t)(0x0A)
#define CMD_SENSOR_UTILITY_LEN    (uint8_t)(0x0B)

#define CMD_START_ACQUISITION    (uint8_t)(0x83)
#define CMD_STOP_ACQUISITION     (uint8_t)(0x84)
#define CMD_SERIAL_CMD           (uint8_t)(0x85)
#define CMD_RESET                (uint8_t)(0x87)

#define DATA_TYPE_UINT8     (uint8_t)(0x00)
#define DATA_TYPE_INT8      (uint8_t)(0x01)
#define DATA_TYPE_UINT16    (uint8_t)(0x02)
#define DATA_TYPE_INT16     (uint8_t)(0x03)
#define DATA_TYPE_UINT32    (uint8_t)(0x04)
#define DATA_TYPE_INT32     (uint8_t)(0x05)
#define DATA_TYPE_FLOAT     (uint8_t)(0x06)

/* To be used in COM_Command_t */
#define COM_COMMAND_SET           (uint8_t)(0x00)
#define COM_COMMAND_GET           (uint8_t)(0x01)
#define COM_COMMAND_START         (uint8_t)(0x02)
#define COM_COMMAND_STOP          (uint8_t)(0x03)
#define BLE_COMMAND_SAVE          (uint8_t)(0x04)
#define COM_COMMAND_SWITCH        (uint8_t)(0x05)

#define COM_REQUEST_DEVICE              (uint8_t)(0x00)
#define COM_REQUEST_DEVICE_INFO         (uint8_t)(0x01)
#define COM_REQUEST_DESCRIPTOR          (uint8_t)(0x02)
#define COM_REQUEST_STATUS              (uint8_t)(0x03)
#define COM_REQUEST_REGISTER            (uint8_t)(0x04)
#define COM_REQUEST_STATUS_NETWORK      (uint8_t)(0x05)
#define COM_REQUEST_STATUS_PERFORMANCE  (uint8_t)(0x06)
#define COM_REQUEST_STATUS_LOGGING      (uint8_t)(0x07)
#define COM_REQUEST_DEVICEREFRESH       (uint8_t)(0x08)
#define COM_REQUEST_SW_TAG              (uint8_t)(0x09)
#define COM_REQUEST_HW_TAG              (uint8_t)(0x0A)
#define COM_REQUEST_SW_TAG_LABEL        (uint8_t)(0x0B)
#define COM_REQUEST_HW_TAG_LABEL        (uint8_t)(0x0C)
#define COM_REQUEST_ACQ_INFO            (uint8_t)(0x0D)
#define COM_REQUEST_TAG_CONFIG          (uint8_t)(0x0E)
#define COM_REQUEST_MLC_CONFIG          (uint8_t)(0x0F)
#define COM_REQUEST_SENSORREFRESH       (uint8_t)(0x10)

#define CMD_TYPE_NETWORK                (uint8_t)(0x00)
#define CMD_TYPE_PERFORMANCE            (uint8_t)(0x01)
#define CMD_TYPE_LOGSTATUS              (uint8_t)(0x02)
/* #define CMD_TYPE_LOGSTATUS_CHANGING     (uint8_t)(0x03) */

#define COM_COMMAND_ERROR         -1

/* Context is only used in the firmware, it's not written into DeviceConfiG.json */
typedef struct
{
  float n_samples_acc;
  double old_time_stamp;
  uint16_t n_samples_to_timestamp;
  uint8_t first_dataReady;
  uint8_t *sd_write_buffer;
  uint32_t sd_write_buffer_idx;
  FIL file_handler;
} COM_SubSensorContext_t;

typedef struct
{
  uint8_t isActive;
  float ODR;
  float measuredODR;
  float initialOffset;
  uint16_t samplesPerTimestamp;
  float FS;
  float sensitivity;
  uint16_t usbDataPacketSize;
  uint32_t sdWriteBufferSize;
  uint32_t wifiDataPacketSize;
  int16_t comChannelNumber;
  uint8_t ucfLoaded;
  COM_SubSensorContext_t context;
} COM_SubSensorStatus_t;

typedef struct
{
  uint8_t id;
  uint8_t sensorType;
  uint8_t dimensions;
  char dimensionsLabel[N_MAX_DIM_LABELS][DIM_LABELS_LENGTH + 1];
  char unit[16];
  uint8_t dataType;
  float FS[N_MAX_SUPPORTED_FS];
  float ODR[N_MAX_SUPPORTED_ODR];
  uint16_t samplesPerTimestamp[2];
} COM_SubSensorDescriptor_t;

typedef struct
{
  COM_SubSensorStatus_t subSensorStatus[N_MAX_SENSOR_COMBO];
} COM_SensorStatus_t;

typedef struct
{
  uint8_t id;
  char name[16];
  uint8_t nSubSensors;
  COM_SubSensorDescriptor_t subSensorDescriptor[N_MAX_SENSOR_COMBO];
} COM_SensorDescriptor_t;

typedef struct
{
  COM_SensorDescriptor_t sensorDescriptor;
  COM_SensorStatus_t sensorStatus;
} COM_Sensor_t;

typedef struct
{
  char serialNumber[25];
  char alias[HSD_DEVICE_ALIAS_LENGTH];
  char partNumber[HSD_DEVICE_PNUMBER_LENGTH];
  char URL[HSD_DEVICE_URL_LENGTH];
  char fwName[HSD_DEVICE_FW_NAME_LENGTH];
  char fwVersion[HSD_DEVICE_FW_VERSION_LENGTH];
  char model[HSD_DEVICE_MODEL_LENGTH];
  char dataFileExt[HSD_DEVICE_DATA_FILE_EXT_LENGTH];
  char dataFileFormat[HSD_DEVICE_DATA_FILE_FORMAT_LENGTH];
  uint32_t nSensor;
  char bleMacAddress[HSD_BLE_MAC_ADDRESS_LENGTH];
} COM_DeviceDescriptor_t;

typedef struct
{
  char pinDesc[HSD_TAGS_PINDESC_LENGTH];
  char label[HSD_TAGS_LABEL_LENGTH];
  uint8_t enabled;
} COM_HwTag_t;

typedef struct
{
  char HSD_SwTagClasses[HSD_TAGS_MAX_SW_CLASSES][HSD_TAGS_LABEL_LENGTH];
  COM_HwTag_t HwTag[HSD_TAGS_MAX_HW_CLASSES];
} COM_TagList_t;

typedef struct
{
  char UUIDAcquisition[37]; /* UUID: 8-4-4-4-12 = 36char + \0 */
  char JSONVersion[HSD_JSON_VERSION_LENGTH];
  COM_DeviceDescriptor_t deviceDescriptor;
  COM_Sensor_t *sensors[COM_MAX_SENSORS];
  COM_TagList_t tagList;
} COM_Device_t;

typedef struct
{
  int8_t command;
  int8_t request;
  int8_t sensorId;
  int8_t subSensorId;
} COM_Command_t;

typedef struct
{
  char name[HSD_ACQ_NAME_LENGTH];
  char description[HSD_ACQ_NOTES_LENGTH];
  char UUIDAcquisition[37]; /* UUID: 8-4-4-4-12 = 36char + \0 */
  char start_time[25]; /* date/time format + \0 */
  char end_time[25]; /* 2014-01-01T23:28:56.782Z */
} COM_AcquisitionDescriptor_t;

#define HS_DATALOG_IDLE          (uint8_t)(0x00)
#define HS_DATALOG_USB_STARTED   (uint8_t)(0x01)
#define HS_DATALOG_SD_STARTED    (uint8_t)(0x02)

extern volatile uint8_t com_status;

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

int32_t COM_AddSensor(void);

COM_Device_t *COM_GetDevice(void);
COM_DeviceDescriptor_t *COM_GetDeviceDescriptor(void);
COM_Sensor_t *COM_GetSensor(uint8_t sID);
COM_SensorDescriptor_t *COM_GetSensorDescriptor(uint8_t sID);
COM_SensorStatus_t *COM_GetSensorStatus(uint8_t sID);
COM_SubSensorDescriptor_t *COM_GetSubSensorDescriptor(uint8_t sID, uint8_t ssID);
COM_SubSensorStatus_t *COM_GetSubSensorStatus(uint8_t sID, uint8_t ssID);
COM_SubSensorContext_t *COM_GetSubSensorContext(uint8_t sID, uint8_t ssID);
COM_TagList_t *COM_GetTagList(void);
COM_AcquisitionDescriptor_t *COM_GetAcquisitionDescriptor(void);

uint8_t COM_GetSubSensorNumber(uint8_t sID);

void COM_SetAcquisitionDescriptor(char *name, char *description);
void COM_SetDeviceAlias(char *alias);
void COM_SetBleMacAddress(uint8_t *bleMacAddress);
void COM_ResetSubSensorContext(uint8_t sID, uint8_t ssID);
void COM_GenerateAcquisitionUUID(void);

uint32_t COM_GetnBytesPerSample(uint8_t sID, uint8_t ssID);
uint8_t COM_IsFsLegal(float value, uint8_t sID, uint8_t ssID);
uint8_t COM_IsOdrLegal(float value, uint8_t sID, uint8_t ssID);

#endif /* __COM_MANAGER_H */

