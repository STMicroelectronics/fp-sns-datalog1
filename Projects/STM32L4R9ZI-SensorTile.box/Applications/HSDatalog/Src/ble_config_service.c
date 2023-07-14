/**
  ******************************************************************************
  * @file    config_service.c
  * @brief   Add bluetooth services using vendor specific profiles.
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

#include <stdio.h>
#include "main.h"
#include "bluenrg1_l2cap_aci.h"
#include "bluenrg1_gatt_aci.h"
#include "bluenrg1_gap_aci.h"
#include "hci.h"
#include "bluenrg1_hci_le.h"
#include "bluenrg1_hal_aci.h"
#include "ble_config_service.h"
#include "ble_comm_transfer_protocol.h"
#include "com_manager.h"
#include "HSD_tags.h"
#include "HSD_json.h"
#include "HSDCore.h"
#include "lsm6dsox_app.h"
#include "sdcard_manager.h"
#include "SensorTile.box_sd.h"
#include "SensorTile.box_bc.h"
#include "OTA.h"

/* Defines -------------------------------------------------------------------*/

/* Magic Number for understanding if there is a valid BoarName saved in Flash */
#define BLE_FLASH_MAGIC_NUM        (0xF0CACC1A)
/* Position where store the BoardName and it's magic Number (last page in Flash)  */
#define BLE_FLASH_BASE_ADDRESS     (FLASH_BASE + 0x000FF000)
#define BLE_FLASH_BASE_ADDRESS_B2  (FLASH_BASE + 0x001FF000)

/* Exported variables ---------------------------------------------------------*/
uint8_t connected = FALSE;

/* Imported Variables -------------------------------------------------------------*/
extern osMessageQId bleSendThreadQueue_id;
extern osSemaphoreId bleInitThreadSem_id;
#if (HSD_BLE_STATUS_TIMER_ENABLE == 1)
extern osTimerId bleSendPerformanceStatusTim_id;
#endif /* (HSD_BLE_STATUS_TIMER_ENABLE == 1) */

extern osTimerId bleAdvUpdaterTim_id;

extern uint8_t SD_Logging_Active;
extern uint8_t SD_present;

uint32_t ConnectionBleStatus;

uint8_t bdaddr[6];

/* Private variables ------------------------------------------------------------*/
uint16_t configServiceHandle;
uint16_t configTxCharHandle;
uint16_t MLCCharHandle;

uint16_t consoleServiceHandle;
uint16_t consoleTermCharHandle;
uint16_t consoleStdErrCharHandle;

uint16_t service_handle;
uint16_t dev_name_char_handle;
uint16_t appearance_char_handle;

static uint16_t connection_handle = 0;
static uint32_t OTA_remaining_size = 0;
static uint8_t BLE_OTA_Ongoing = 0;
tBleStatus status_ble;

/* MaxBLECharLen represent the maximum BLE message length */
#if (ENABLE_MTU_EXCHANGE == 1)
uint16_t MaxBLECharLen = 20;
uint16_t MaxBLECharLen_FFOTA = MAX_ATT_MTU;
#else
/* When MTU Exchange is disabled, BLE nessage length is fixed to 20 bytes */
uint16_t MaxBLECharLen = 20;
uint16_t MaxBLECharLen_FFOTA = 20;
#endif /* (ENABLE_MTU_EXCHANGE == 1) */

Service_UUID_t service_config_uuid;
Char_UUID_t char_tx_uuid;
Char_UUID_t char_mlc_uuid;

Service_UUID_t service_console_uuid;
Char_UUID_t term_char_uuid;
Char_UUID_t stderr_char_uuid;

COM_Command_t outCommand;
COM_Sensor_t tempSensor;
COM_SensorStatus_t *myStatus;

/* Console Service */
static const uint8_t console_service_uuid[16] = {0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0xb4, 0x9a, 0xe1, 0x11, 0x0E,
                                                 0x00, 0x00, 0x00, 0x00, 0x00};
static const uint8_t console_term_char_uuid[16] = {0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac, 0xe1, 0x11, 0x0E,
                                                   0x00, 0x01, 0x00, 0x00, 0x00};
static const uint8_t console_stderr_char_uuid[16] = {0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac, 0xe1, 0x11, 0x0E,
                                                     0x00, 0x02, 0x00, 0x00, 0x00};

/* STWINConfig Service UUID */
static const uint8_t config_service_uuid[16] = {0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0xb4, 0x9a, 0xe1, 0x11, 0x01, 0x00,
                                                0x00, 0x00, 0x00, 0x00};
static const uint8_t config_tx_char_uuid[16] = {0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac, 0xe1, 0x11, 0x02, 0x00,
                                                0x11, 0x00, 0x00, 0x00};
static const uint8_t config_mlc_char_uuid[16] = {0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x36, 0xac, 0xe1, 0x11, 0x02, 
                                                 0x00, 0x0F, 0x00, 0x00, 0x00};


uint8_t *hs_command_buffer;

uint8_t ble_init = 0;
volatile char BoardName[8] = {NAME_HSD, 0};
volatile uint32_t OTA_crc;

/* Private functions ------------------------------------------------------------*/
static uint32_t BLE_ConfigConsole_Command_Parsing(uint8_t *att_data, uint32_t len);
static uint32_t BLE_ConfigConsole_SetRequests_Parsing(int8_t request, uint32_t len);
static uint32_t BLE_DebugConsole_CommandParsing(uint8_t *att_data, uint32_t len);
static uint32_t BLE_OTA_upgrade_start(uint8_t *payload);
static int32_t BLE_OTA_upgrade_chunk(uint8_t *buffer, uint32_t len);

void APP_UserEvtRx(void *pData);
tBleStatus Init_BlueNRG_Stack(void);
tBleStatus Add_Config_Service(void);
tBleStatus Add_Console_Service(void);
void setConnectable(void);

static uint32_t GetPage(uint32_t Addr);
static uint32_t GetBank(uint32_t Addr);

uint32_t GetBoardNameFromFlash(void);
uint32_t SaveBoardNameToFlash(uint32_t address);
uint32_t SetBoardName(char *boardName, uint8_t length);

/* Private define ------------------------------------------------------------*/

/**
  * @brief  Gets the page of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The page of a given address
  */
static uint32_t GetPage(uint32_t Addr)
{
  uint32_t page = 0;

  if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
  {
    /* Bank 1 */
    page = (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
  }
  else
  {
    /* Bank 2 */
    page = (Addr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
  }

  return page;
}

/**
  * @brief  Gets the bank of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The bank of a given address
  */
static uint32_t GetBank(uint32_t Addr)
{
  uint32_t bank = 0;

  if (READ_BIT(SYSCFG->MEMRMP, SYSCFG_MEMRMP_FB_MODE) == 0)
  {
    /* No Bank swap */
    if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
    {
      bank = FLASH_BANK_1;
    }
    else
    {
      bank = FLASH_BANK_2;
    }
  }
  else
  {
    /* Bank swap */
    if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
    {
      bank = FLASH_BANK_2;
    }
    else
    {
      bank = FLASH_BANK_1;
    }
  }

  return bank;
}

/* BLE Advertisement option bytes Callback function */
void bleAdvUpdaterTim_Callback(void const *argument)
{
  setConnectable();
}

void ble_interface_init(void)
{
  if (ble_init != 1)
  {
    BLE_CM_SPI_Reset();

    Init_BlueNRG_Stack();

    Add_Config_Service();

    Add_Console_Service();

    ble_init = 1;
  }
  osTimerStart(bleAdvUpdaterTim_id, 3000);

}

/** @brief Initialize the BlueNRG Stack
  * @param None
  * @retval None
  */
tBleStatus Init_BlueNRG_Stack(void)
{
  tBleStatus ret;
  uint8_t data_len_out;

  /* Initialize the BlueNRG HCI */
  hci_init(APP_UserEvtRx, NULL);

  HAL_Delay(100);

  ret = aci_gatt_init();

  if (ret != BLE_STATUS_SUCCESS)
  {
    return BLE_STATUS_ERROR;
  }

  ret = aci_gap_init(GAP_PERIPHERAL_ROLE, 0, strlen((char *) BoardName), &service_handle, &dev_name_char_handle,
                                                    &appearance_char_handle);

  if (ret != BLE_STATUS_SUCCESS)
  {
    return BLE_STATUS_ERROR;
  }

  GetBoardNameFromFlash();

  /* we will let the BLE chip to use its Random MAC address */
#define CONFIG_DATA_RANDOM_ADDRESS          (0x80) /**< Stored static random address. Read-only. */
  ret = aci_hal_read_config_data(CONFIG_DATA_RANDOM_ADDRESS, &data_len_out, bdaddr);
  if (ret != BLE_STATUS_SUCCESS)
  {
    return BLE_STATUS_ERROR;
  }
  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
                                  CONFIG_DATA_PUBADDR_LEN,
                                  bdaddr);
  if (ret != BLE_STATUS_SUCCESS)
  {
    return BLE_STATUS_ERROR;
  }

  ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0, strlen((char *) BoardName),
                                   (uint8_t *) BoardName);

  if (ret != BLE_STATUS_SUCCESS)
  {
    return BLE_STATUS_ERROR;
  }

  ret = aci_gap_set_authentication_requirement(BONDING,
                                               MITM_PROTECTION_REQUIRED,
                                               SC_IS_SUPPORTED,
                                               KEYPRESS_IS_NOT_SUPPORTED,
                                               7, 16,
                                               USE_FIXED_PIN_FOR_PAIRING,
                                               123456, 0x00);

  if (ret != BLE_STATUS_SUCCESS)
  {
    return BLE_STATUS_ERROR;
  }

  /* Set output power level */
  aci_hal_set_tx_power_level(1, 4);

  hci_le_write_suggested_default_data_length(220, 2120);

  return BLE_STATUS_SUCCESS;
}

/**
  * @brief  Add the Config service using a vendor specific profile
  * @param  None
  * @retval tBleStatus Status
  */
tBleStatus Add_Config_Service(void)
{
  tBleStatus ret;

  BLUENRG_memcpy(&service_config_uuid.Service_UUID_128, config_service_uuid, 16);
  ret = aci_gatt_add_service(UUID_TYPE_128, &service_config_uuid, PRIMARY_SERVICE, 1 + 6, &configServiceHandle);

  if (ret != BLE_STATUS_SUCCESS)
  {
    return BLE_STATUS_ERROR;
  }

  BLUENRG_memcpy(&char_tx_uuid.Char_UUID_128, config_tx_char_uuid, 16);
  ret = aci_gatt_add_char(configServiceHandle, UUID_TYPE_128, &char_tx_uuid, MaxBLECharLen,
                          CHAR_PROP_NOTIFY | CHAR_PROP_WRITE_WITHOUT_RESP,
                          ATTR_PERMISSION_NONE, GATT_NOTIFY_ATTRIBUTE_WRITE, 16, 1, &configTxCharHandle);

  if (ret != BLE_STATUS_SUCCESS)
  {
    return BLE_STATUS_ERROR;
  }

  BLUENRG_memcpy(&char_mlc_uuid.Char_UUID_128, config_mlc_char_uuid, 16);
  /* 2 byte timestamp, 8 MLC registers output, 1 MCL output state */
  ret = aci_gatt_add_char(configServiceHandle, UUID_TYPE_128, &char_mlc_uuid, 2 + 8 + 1,
                          CHAR_PROP_NOTIFY,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                          16, 0, &MLCCharHandle);

  if (ret != BLE_STATUS_SUCCESS)
  {
    return BLE_STATUS_ERROR;
  }

  return BLE_STATUS_SUCCESS;
}

/**
  * @brief  Add the Console service using a vendor specific profile
  * @param  None
  * @retval tBleStatus Status
  */
tBleStatus Add_Console_Service(void)
{
  tBleStatus ret;

  BLUENRG_memcpy(&service_console_uuid.Service_UUID_128, console_service_uuid, 16);
  ret = aci_gatt_add_service(UUID_TYPE_128, &service_console_uuid, PRIMARY_SERVICE, 1 + 6, &consoleServiceHandle);

  if (ret != BLE_STATUS_SUCCESS)
  {
    return BLE_STATUS_ERROR;
  }

  BLUENRG_memcpy(&term_char_uuid.Char_UUID_128, console_term_char_uuid, 16);
  ret = aci_gatt_add_char(consoleServiceHandle, UUID_TYPE_128, &term_char_uuid, MaxBLECharLen_FFOTA,
                          CHAR_PROP_NOTIFY | CHAR_PROP_WRITE_WITHOUT_RESP | CHAR_PROP_WRITE | CHAR_PROP_READ,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_ATTRIBUTE_WRITE | GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                          16, 1, &consoleTermCharHandle);

  if (ret != BLE_STATUS_SUCCESS)
  {
    return BLE_STATUS_ERROR;
  }

  BLUENRG_memcpy(&stderr_char_uuid.Char_UUID_128, console_stderr_char_uuid, 16);
  ret = aci_gatt_add_char(consoleServiceHandle, UUID_TYPE_128, &stderr_char_uuid, MaxBLECharLen_FFOTA,
                          CHAR_PROP_NOTIFY | CHAR_PROP_READ,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                          16, 1, &consoleStdErrCharHandle);
  if (ret != BLE_STATUS_SUCCESS)
  {
    return BLE_STATUS_ERROR;
  }

  return BLE_STATUS_SUCCESS;
}

/**
  * @brief Puts the device in connectable mode.
  * @param None
  * @retval None
  */

void setConnectable(void)
{
  char local_name[8];
  local_name[0] = AD_TYPE_COMPLETE_LOCAL_NAME;
  memcpy(&local_name[1], (char *) BoardName, 7);

  /* First adv. Option byte */
  BSP_BC_CmdSend(BATMS_ON); /* Enable the Battery voltage reading */
  uint32_t mV = 0;
  uint32_t level = 0;
  stbc02_State_TypeDef BC_State;
  BSP_BC_GetVoltageAndLevel(&mV, &level);
  BSP_BC_GetState(&BC_State);
  BSP_BC_CmdSend(BATMS_OFF); /* Disable the Battery voltage reading */
  uint8_t battery_low_thresh = 10;

  /*Second adv. Option byte */
  uint8_t alarm = 0;
  uint8_t icon = 0;

  uint8_t lowMemory = 0; /*Check SD free size only if an acquisition is not ongoing (avoid dirtying the acquisition)*/
  if (!SD_Logging_Active)
  {
    lowMemory = SDM_CheckLowMemory();
  }

  if (BC_State.Id == 0 || BC_State.Id == 9) /* id == 9 --> unplugged battery id selection known bug (BatteryCharger) */
  {
    icon = 0; /* USB Plugged, No Battery */
    alarm = 2;
  }
  else if (BC_State.Id > 0 && BC_State.Id < 3)
  {
    icon = 2; /* Battery, no USB Plugged */
    if (level < battery_low_thresh) /* Low Battery */
    {
      alarm = icon = 4;
    }
  }
  else
  {
    icon = alarm = 1; /* Battery, USB Plugged */
  }
  if (!BSP_SD_IsDetected()) /* No SD Card Detected */
  {
    alarm = icon = 5;
  }
  if (lowMemory == 1) /* Low Memory */
  {
    alarm = icon = 6;
  }
  if (SD_Logging_Active) /* Logging Ongoing */
  {
    alarm = icon = 3;
  }

  uint8_t manuf_data[25] =
  {
    8,
    0x09,
    BoardName[0],
    BoardName[1],
    BoardName[2],
    BoardName[3],
    BoardName[4],
    BoardName[5],
    BoardName[6], /* Complete Name */
    15,
    0xFF,
    0x30,
    0x00, /*[length][propetary adv.][STMicroelectronics Manufacturer ID]*/
    0x02, /* BlueSTSDK version*/
    0x06, /* Board_id STEVAL-MKSBOX1V1*/
    0x0E, /* FW_id FP-SNS-DATALOG1*/
    (uint8_t) level, /* FW_data F1 */
    alarm, /* FW_data F2 */
    icon, /* FW_data F3 */
    0x00, /* BLE MAC start */
    0x00,
    0x00,
    0x00,
    0x00,
    0x00, /* BLE MAC stop */
  };

  /* BLE MAC */
  manuf_data[19] = bdaddr[5];
  manuf_data[20] = bdaddr[4];
  manuf_data[21] = bdaddr[3];
  manuf_data[22] = bdaddr[2];
  manuf_data[23] = bdaddr[1];
  manuf_data[24] = bdaddr[0];

  /* disable scan response */
  status_ble = hci_le_set_scan_response_data(0, NULL);
  status_ble = aci_gap_set_discoverable(ADV_IND, 0, 0,
                                        STATIC_RANDOM_ADDR,
                                        NO_WHITE_LIST_USE, sizeof(local_name), (uint8_t *) local_name, 0, NULL, 0, 0);

  /* Send Advertising data */
  status_ble = aci_gap_update_adv_data(25, manuf_data);

  /* Save BLE MAC address in DeviceInfo */
  COM_SetBleMacAddress(bdaddr);
}

/**
  * @brief  This function is called when there is a change on the gatt attribute
  * With this function it's possible to understand if one application
  * is subscribed or not to the one service
  * @param uint16_t att_handle Handle of the attribute
  * @param uint8_t *att_data attribute data
  * @param uint8_t data_length length of the data
  * @retval None
  */
void Attribute_Modified_CB(uint16_t attr_handle, uint8_t *att_data, uint8_t data_length)
{
  if (attr_handle == consoleTermCharHandle + 1)
  {
    BLE_DebugConsole_CommandParsing(att_data, data_length);
  }
  if (BLE_OTA_Ongoing) /* if OTA is ongoing ignore other messages */
  {
    return;
  }

  if (attr_handle == configTxCharHandle + 1)
  {
    BLE_ConfigConsole_Command_Parsing(att_data, data_length);
  }
  else if (attr_handle == configTxCharHandle +
           2) /* Enabling the generation of the Battery voltage only if it's necessary */
  {
    if (att_data[0] == 01)
    {
      BSP_BC_CmdSend(BATMS_ON);
#if (HSD_BLE_STATUS_TIMER_ENABLE == 1)
      osTimerStart(bleSendPerformanceStatusTim_id, 2000);
#endif /* (HSD_BLE_STATUS_TIMER_ENABLE == 1) */
    }
    else if (att_data[0] == 0)
    {
      BSP_BC_CmdSend(BATMS_OFF);
#if (HSD_BLE_STATUS_TIMER_ENABLE == 1)
      osTimerStop(bleSendPerformanceStatusTim_id);
#endif /* (HSD_BLE_STATUS_TIMER_ENABLE == 1) */
    }
  }
  else if (attr_handle == MLCCharHandle + 2)
  {
    if (att_data[0] == 1)
    {
      osMessagePut(bleSendThreadQueue_id, BLE_COMMAND_MLC, 0);
    }
  }
}

/**
  * @brief  Parse commands received on Config Console
  * @param uint8_t *att_data attribute data
  * @param uint8_t len length of the data
  * @retval 0 if ok
  */
static uint32_t BLE_ConfigConsole_Command_Parsing(uint8_t *att_data, uint32_t len)
{
  uint32_t HSCommandBufLen = BLECommand_TP_Parse(&hs_command_buffer, att_data, len);

  if (HSCommandBufLen > 0)
  {
    HSD_JSON_parse_Command((char *) hs_command_buffer, &outCommand);
    if (outCommand.command == COM_COMMAND_SET)
    {
      BLE_ConfigConsole_SetRequests_Parsing(outCommand.request, HSCommandBufLen);
    }
    else if (outCommand.command == COM_COMMAND_GET)
    {
      HSD_JSON_free(hs_command_buffer);
      osMessagePut(bleSendThreadQueue_id, (uint32_t) outCommand.request, 0);
    }
    else if ((outCommand.command == COM_COMMAND_START) && (SD_Logging_Active == 0))
    {
      COM_AcquisitionDescriptor_t *pAcquisitionDescriptor;
      pAcquisitionDescriptor = COM_GetAcquisitionDescriptor();
      HSD_JSON_parse_StartTime((char *) hs_command_buffer, pAcquisitionDescriptor);

      HSD_JSON_free(hs_command_buffer);

      if (StartStop_AutoMode() == SYS_NO_ERROR_CODE)
      {
        /* Enter here if AutoMode configuration exists and is valid */
        return 0;
      }
      else if (BSP_SD_IsDetected() == SD_PRESENT) /* No valid AutoMode configuration, usual procedure */
      {
        if (osMessagePut(sdThreadQueue_id, SDM_START_STOP, 0) != osOK)
        {
          return 1;
        }
      }
      else
      {
        osMessagePut(bleSendThreadQueue_id, COM_REQUEST_STATUS_LOGGING, 0);
      }
    }
    else if (((outCommand.command == COM_COMMAND_STOP) && (SD_Logging_Active == 1)) || AMTIsStarted())
    {
      COM_AcquisitionDescriptor_t *pAcquisitionDescriptor;
      pAcquisitionDescriptor = COM_GetAcquisitionDescriptor();
      HSD_JSON_parse_EndTime((char *) hs_command_buffer, pAcquisitionDescriptor);

      HSD_JSON_free(hs_command_buffer);

      if (AMTIsStarted())
      {
        StartStop_AutoMode();
      }
      else
      {
        StopExecutionPhases();
      }
    }
    else if (outCommand.command == BLE_COMMAND_SAVE)
    {
      HSD_JSON_free(hs_command_buffer);
      SDM_UpdateDeviceConfig();
    }
    else if (outCommand.command == COM_COMMAND_SWITCH)
    {
      /* change flash bank */
      EnableDisableDualBoot();
    }
  }
  return 0;
}

/**
  * @brief  Parse SET requests received on Config Console
  * @param  int8_t request
  * @param  uint32_t len length of the data
  * @retval 0 if ok
  */
static uint32_t BLE_ConfigConsole_SetRequests_Parsing(int8_t request, uint32_t len)
{
  switch (request)
  {
    case COM_REQUEST_DEVICE_INFO :
    {
      /* SET device alias */
      char alias[HSD_DEVICE_ALIAS_LENGTH];
      HSD_JSON_parse_SetDeviceAliasCommand((char *) hs_command_buffer, alias, HSD_DEVICE_ALIAS_LENGTH);
      HSD_JSON_free(hs_command_buffer);
      COM_SetDeviceAlias(alias);
      osMessagePut(bleSendThreadQueue_id, COM_REQUEST_DEVICE_INFO, 0);
      break;
    }
    case COM_REQUEST_SW_TAG :
    {
      /* SET SW Tag (Enable/Disable) */
      uint8_t id;
      HSD_Tags_Enable_t enable;
      double timestamp = SM_GetTimeStamp();
      HSD_JSON_parse_EnableTagCommand((char *) hs_command_buffer, &id, &enable);
      HSD_JSON_free(hs_command_buffer);
      HSD_TAGS_add_tag(HSD_TAGS_Type_Sw, id, enable, timestamp);
      break;
    }
    case COM_REQUEST_SW_TAG_LABEL :
    {
      /* SET SW Tag Label */
      uint8_t id;
      char label[HSD_TAGS_LABEL_LENGTH];
      COM_Device_t *device = COM_GetDevice();
      HSD_JSON_parse_UpdateTagLabelCommand((char *) hs_command_buffer, &id, label, HSD_TAGS_LABEL_LENGTH);
      HSD_JSON_free(hs_command_buffer);
      HSD_TAGS_set_tag_label(device, HSD_TAGS_Type_Sw, id, label);
      break;
    }
    case COM_REQUEST_HW_TAG :
    {
      /* SET HW Tag (Enable/Disable) */
      uint8_t id;
      HSD_Tags_Enable_t enable;
      COM_Device_t *device = COM_GetDevice();
      HSD_JSON_parse_EnableTagCommand((char *) hs_command_buffer, &id, &enable);
      HSD_JSON_free(hs_command_buffer);
      HSD_TAGS_set_tag_enabled(device, id, enable);
      break;
    }
    case COM_REQUEST_HW_TAG_LABEL :
    {
      /* SET HW Tag Label */
      uint8_t id;
      char label[HSD_TAGS_LABEL_LENGTH];
      COM_Device_t *device = COM_GetDevice();
      HSD_JSON_parse_UpdateTagLabelCommand((char *) hs_command_buffer, &id, label, HSD_TAGS_LABEL_LENGTH);
      HSD_JSON_free(hs_command_buffer);
      HSD_TAGS_set_tag_label(device, HSD_TAGS_Type_Hw, id, label);
      break;
    }
    case COM_REQUEST_STATUS_NETWORK :
    {
      HSD_JSON_free(hs_command_buffer);
      break;
    }
    case COM_REQUEST_ACQ_INFO :
    {
      /* SET Acquisition Name and Description */
      char name[HSD_ACQ_NAME_LENGTH];
      char notes[HSD_ACQ_NOTES_LENGTH];
      HSD_JSON_parse_AcqInfoCommand((char *) hs_command_buffer, name, HSD_ACQ_NAME_LENGTH, notes, 
	                                HSD_ACQ_NOTES_LENGTH);
      HSD_JSON_free(hs_command_buffer);
      COM_SetAcquisitionDescriptor(name, notes);
      break;
    }
    case COM_REQUEST_MLC_CONFIG :
    {
      /* Extract loaded ucf size and data */
      uint32_t mlcConfigSize;
      char *mlcConfigData;

      mlcConfigData = (char *) HSD_malloc(len);
      if (mlcConfigData == NULL)
      {
        HSD_PRINTF("Mem alloc error [%d]: %d@%s\r\n", len, __LINE__, __FILE__);
      }
      else
      {
        HSD_PRINTF("Mem alloc ok [%d]: %d@%s\r\n", len, __LINE__, __FILE__);
      }

      HSD_JSON_parse_MlcConfigCommand((char *) hs_command_buffer, &mlcConfigSize, mlcConfigData, len);
      HSD_JSON_free(hs_command_buffer);

      uint32_t ucfFileSize = 9 * (mlcConfigSize / 4);
      char *ucfFileBuffer = HSD_malloc(ucfFileSize);
      if (ucfFileBuffer == NULL)
      {
        HSD_PRINTF("Mem alloc error [%ld]: %d@%s\r\n", ucfFileSize, __LINE__, __FILE__);
      }
      else
      {
        HSD_PRINTF("Mem alloc ok [%ld]: %d@%s\r\n", ucfFileSize, __LINE__, __FILE__);
      }

      LSM6DSOX_GetUCF_FromBuffer(mlcConfigData, mlcConfigSize, ucfFileBuffer, ucfFileSize);
      LSM6DSOX_SetUCF(mlcConfigSize, mlcConfigData);

      g_prgUcfFileBuffer = ucfFileBuffer;
      g_prgUcfFileSize = ucfFileSize;

      if (osMessagePut(sdThreadQueue_id, SDM_WRITE_UCF_TO_ROOT, 0) != osOK)
      {
        SM_Error_Handler();
      }
      if (osMessagePut(bleSendThreadQueue_id, COM_REQUEST_SENSORREFRESH | LSM6DSOX_Get_Id(), 0) != osOK)
      {
        SM_Error_Handler();
      }
      break;
    }
    default:
    {
      myStatus = COM_GetSensorStatus(outCommand.sensorId);
      memcpy(&tempSensor.sensorStatus, myStatus, sizeof(COM_SensorStatus_t));
      HSD_JSON_parse_Status((char *) hs_command_buffer, &tempSensor.sensorStatus);
      HSD_JSON_free(hs_command_buffer);
      update_sensorStatus(myStatus, &tempSensor.sensorStatus, outCommand.sensorId);

      /* Update the sensor-specific config structure */
      uint8_t deviceRefresh = update_sensors_config();
      if (deviceRefresh != 0)
      {
        osMessagePut(bleSendThreadQueue_id, COM_REQUEST_SENSORREFRESH | outCommand.sensorId, 0);
      }
    }
  }
  return 0;
}

/**
  * @brief  Parse commands received on Config Console
  * @param  att_data attribute data
  * @param  len length of the data
  * @retval 0 if ok
  */
static uint32_t BLE_DebugConsole_CommandParsing(uint8_t *att_data, uint32_t len)
{
  if (BLE_OTA_Ongoing == 1)
  {
    BLE_OTA_upgrade_chunk(att_data, len);
  }
  else if (!strncmp("setName ", (char *)(att_data), 8))
  {
    uint8_t nameLen = len - 8;
    nameLen = (nameLen > 7) ? 7 : nameLen;

    SetBoardName((char *) &att_data[8], nameLen);
    SaveBoardNameToFlash(BLE_FLASH_BASE_ADDRESS);

    /* Add the response request to the BLE_Send_Thread queue */
    if (osMessagePut(bleSendThreadQueue_id, BLE_COMMAND_DEBUG_CONSOLE | BLE_SUB_CMD_BOARD_NAME, 0) != osOK)
    {
      SM_Error_Handler();
    }
  }
  else if (!strncmp("versionFw", (char *)(att_data), 9))
  {
    if (osMessagePut(bleSendThreadQueue_id, BLE_COMMAND_DEBUG_CONSOLE | BLE_SUB_CMD_FW_VERSION, 0) != osOK)
    {
      SM_Error_Handler();
    }
  }
  else if (strncmp("upgradeFw", (char *)(att_data), 9) == 0)
  {
    BLE_OTA_upgrade_start(&att_data[9]);

    if (osMessagePut(bleSendThreadQueue_id, BLE_COMMAND_DEBUG_CONSOLE | BLE_SUB_CMD_FOTA_START, 0) != osOK)
    {
      SM_Error_Handler();
    }
  }
  else if (!strncmp("info", (char *)(att_data), 4))
  {
    if (osMessagePut(bleSendThreadQueue_id, BLE_COMMAND_DEBUG_CONSOLE | BLE_SUB_CMD_INFO, 0) != osOK)
    {
      SM_Error_Handler();
    }
  }

  return 0;
}

/**
  * @brief  Write a chunk of the new FW to the flash and notify the app in case of error or if the procedure is 
  *         complete
  * @param  buffer data
  * @param  len length of the data
  * @retval 0 if ok, -1 in case of errors
  */
static int32_t BLE_OTA_upgrade_chunk(uint8_t *buffer, uint32_t len)
{
  int8_t ret = UpdateFWBlueMS(&OTA_remaining_size, buffer, len, 1);

  if (ret == 1)
  {
    if (osMessagePut(bleSendThreadQueue_id, BLE_COMMAND_DEBUG_CONSOLE | BLE_SUB_CMD_FOTA_COMPLETED, 0) != osOK)
    {
      SM_Error_Handler();
    }
    BLE_OTA_Ongoing = 0;
    EnableDisableDualBoot(); /* swap STM32 banks */
  }
  else if (ret == -1)
  {
    if (osMessagePut(bleSendThreadQueue_id, BLE_COMMAND_DEBUG_CONSOLE | BLE_SUB_CMD_FOTA_ERROR, 0) != osOK)
    {
      SM_Error_Handler();
    }
    BLE_OTA_Ongoing = 0;
  }

  return ret;
}

/**
  * @brief Start the FW upgrade procedure
  * @param payload
  * @retval 0 if ok
  */
static uint32_t BLE_OTA_upgrade_start(uint8_t *payload)
{
  uint8_t *p8 = (uint8_t *) &OTA_remaining_size;

  p8[0] = payload[0];
  p8[1] = payload[1];
  p8[2] = payload[2];
  p8[3] = payload[3];

  p8 = (uint8_t *) &OTA_crc;
  /* Check the Maximum Possible OTA size */
  if (OTA_remaining_size > OTA_MAX_PROG_SIZE)
  {
    p8[0] = payload[4];
    p8[1] = (payload[5] != 0) ? 0 : 1;/* In order to be sure to have a wrong CRC */
    p8[2] = payload[6];
    p8[3] = payload[7];
  }
  else
  {
    p8[0] = payload[4];
    p8[1] = payload[5];
    p8[2] = payload[6];
    p8[3] = payload[7];

    /* Reset the Flash */
    StartUpdateFWBlueMS(OTA_remaining_size, OTA_crc);

    /* If the Board name is stored in flash, read it and save to the other bank */
    if (GetBoardNameFromFlash() == SYS_NO_ERROR_CODE)
    {
      SaveBoardNameToFlash(BLE_FLASH_BASE_ADDRESS_B2);
    }

    BLE_OTA_Ongoing = 1;

    /* Reduce the connection interval */
    {
      tBleStatus ret = aci_l2cap_connection_parameter_update_req(connection_handle, 10 /* interval_min*/,
                                                                 10 /* interval_max */, 0 /* slave_latency */,
                                                                 400 /*timeout_multiplier*/);
      if (ret != BLE_STATUS_SUCCESS)
      {
        return 1;
      }
    }
  }
  return 0;
}

void APP_UserEvtRx(void *pData)
{
  uint32_t i;

  hci_spi_pckt *hci_pckt = (hci_spi_pckt *) pData;

  if (hci_pckt->type == HCI_EVENT_PKT)
  {
    hci_event_pckt *event_pckt = (hci_event_pckt *) hci_pckt->data;

    if (event_pckt->evt == EVT_LE_META_EVENT)
    {
      evt_le_meta_event *evt = (void *) event_pckt->data;

      for (i = 0; i < (sizeof(hci_le_meta_events_table) / sizeof(hci_le_meta_events_table_type)); i++)
      {
        if (evt->subevent == hci_le_meta_events_table[i].evt_code)
        {
          hci_le_meta_events_table[i].process((void *) evt->data);
        }
      }
    }
    else if (event_pckt->evt == EVT_VENDOR)
    {
      evt_blue_aci *blue_evt = (void *) event_pckt->data;

      for (i = 0; i < (sizeof(hci_vendor_specific_events_table) / sizeof(hci_vendor_specific_events_table_type)); i++)
      {
        if (blue_evt->ecode == hci_vendor_specific_events_table[i].evt_code)
        {
          hci_vendor_specific_events_table[i].process((void *) blue_evt->data);
        }
      }
    }
    else
    {
      for (i = 0; i < (sizeof(hci_events_table) / sizeof(hci_events_table_type)); i++)
      {
        if (event_pckt->evt == hci_events_table[i].evt_code)
        {
          hci_events_table[i].process((void *) event_pckt->data);
        }
      }
    }
  }
}

uint32_t SetBoardName(char *boardName, uint8_t length)
{
  length = (length <= sizeof(BoardName)) ? length : sizeof(BoardName);

  memcpy((void *) &BoardName[0], boardName, length);

  return SYS_NO_ERROR_CODE;
}

uint32_t SaveBoardNameToFlash(uint32_t address)
{
  /* Save the BoardName in Flash */
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t SectorError = 0;
  uint64_t ValueToWrite;

  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.Banks = GetBank(address);
  EraseInitStruct.Page = GetPage(address);
  EraseInitStruct.NbPages = 1;

  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

  /* Clear OPTVERR bit set on virgin samples */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
  /* Clear PEMPTY bit set (as the code is executed from Flash which is not empty) */
  if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_PEMPTY) != 0)
  {
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PEMPTY);
  }

  if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
  {
    return SYS_BASE_ERROR_CODE;
  }

  ValueToWrite = ((uint64_t) BLE_FLASH_MAGIC_NUM);
  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, ValueToWrite) != HAL_OK)
  {
    return SYS_BASE_ERROR_CODE;
  }
  else
  {
    ValueToWrite = (((uint64_t) BoardName[4]) << (32 + 24)) | (((uint64_t) BoardName[5]) << (32 + 16)) 
	               | (((uint64_t) BoardName[6]) << (32 + 8)) | (((uint64_t) BoardName[0]) << (0 + 24)) 
				   | (((uint64_t) BoardName[1]) << (0 + 16)) | (((uint64_t) BoardName[2]) << (0 + 8))
                   | (((uint64_t) BoardName[3]));
    address += 8;
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, ValueToWrite) != HAL_OK)
    {
      return SYS_BASE_ERROR_CODE;
    }
  }
  /* Lock the Flash to disable the flash control register access (recommended
   to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();

  return SYS_NO_ERROR_CODE;
}

uint32_t GetBoardNameFromFlash(void)
{
  uint32_t SourceAddress = BLE_FLASH_BASE_ADDRESS;

  /* Check if there is a valid BoardName in Flash */
  if ((*(uint32_t *) SourceAddress) == BLE_FLASH_MAGIC_NUM)
  {
    /* Read the Board Name from Flash */
    SourceAddress += 8;
    uint32_t Data;

    Data = *(uint32_t *) SourceAddress;
    BoardName[0] = (Data >> 24) & 0xFF;
    BoardName[1] = (Data >> 16) & 0xFF;
    BoardName[2] = (Data >> 8) & 0xFF;
    BoardName[3] = (Data) & 0xFF;

    SourceAddress += 4;
    Data = *(uint32_t *) SourceAddress;
    BoardName[4] = (Data >> 24) & 0xFF;
    BoardName[5] = (Data >> 16) & 0xFF;
    BoardName[6] = (Data >> 8) & 0xFF;
    return SYS_NO_ERROR_CODE;
  }
  return SYS_BASE_ERROR_CODE;
}

/* ***************** BlueNRG-1 Stack Callbacks ********************************/

/*******************************************************************************
  * Function Name  : hci_le_connection_complete_event.
  * Description    : This event indicates that a new connection has been created.
  * Input          : See file bluenrg1_events.h
  * Output         : See file bluenrg1_events.h
  * Return         : See file bluenrg1_events.h
  *******************************************************************************/
void hci_le_connection_complete_event(uint8_t Status, uint16_t Connection_Handle, uint8_t Role,
                                      uint8_t Peer_Address_Type, uint8_t Peer_Address[6],
                                      uint16_t Conn_Interval, uint16_t Conn_Latency, uint16_t Supervision_Timeout,
									  uint8_t Master_Clock_Accuracy)
{
  connected = TRUE;
  connection_handle = Connection_Handle;

  osTimerStop(bleAdvUpdaterTim_id);

  ConnectionBleStatus = 0;
#if (ENABLE_MTU_EXCHANGE == 1)
  aci_gatt_exchange_config(connection_handle);
#endif /* (ENABLE_MTU_EXCHANGE == 1) */

  HAL_Delay(200); /*Workaround for iOS*/
  aci_l2cap_connection_parameter_update_req(connection_handle, 8, 17, 0, 400);

}/* end hci_le_connection_complete_event() */

/*******************************************************************************
  * Function Name  : hci_disconnection_complete_event.
  * Description    : This event occurs when a connection is terminated.
  * Input          : See file bluenrg1_events.h
  * Output         : See file bluenrg1_events.h
  * Return         : See file bluenrg1_events.h
  *******************************************************************************/
void hci_disconnection_complete_event(uint8_t Status, uint16_t Connection_Handle, uint8_t Reason)
{
  connected = FALSE;

#if (HSD_BLE_STATUS_TIMER_ENABLE == 1)
  osTimerStop(bleSendPerformanceStatusTim_id);
#endif /* (HSD_BLE_STATUS_TIMER_ENABLE == 1) */

  /* Make the device connectable again. */
  osSemaphoreRelease(bleInitThreadSem_id);
  ConnectionBleStatus = 0;
}/* end hci_disconnection_complete_event() */

void aci_gatt_attribute_modified_event(uint16_t Connection_Handle, uint16_t Attr_Handle, uint16_t Offset,
                                       uint16_t Attr_Data_Length, uint8_t Attr_Data[])
{
  Attribute_Modified_CB(Attr_Handle, Attr_Data, Attr_Data_Length);
}

void aci_att_exchange_mtu_resp_event(uint16_t Connection_Handle, uint16_t Server_RX_MTU)
{
  if ((Server_RX_MTU - 3) < MaxBLECharLen_FFOTA)
  {
    MaxBLECharLen_FFOTA = Server_RX_MTU - 3;
  }
  HSD_PRINTF("aci_att_exchange_mtu_resp_event Server_RX_MTU=%d\r\n", Server_RX_MTU);
}

void aci_gatt_indication_event(uint16_t Connection_Handle, uint16_t Attribute_Handle, uint8_t Attribute_Value_Length,
                               uint8_t Attribute_Value[])
{
  tBleStatus RetStatus;

  /* This callback should be called when we connect the board also to something
    * that could work also like server mode.
    * In our case we don't need to do nothing when we receive this indication,
    * except it's confirmation
    */
#if HSD_PRINTF_DEBUG
  HSD_PRINTF("aci_gatt_indication_event:\r\n");
  HSD_PRINTF("\tConnection_Handle=0x%x\r\n", Connection_Handle);
  HSD_PRINTF("\tAttribute_Handle=0x%x\r\n", Attribute_Handle);

  if (Attribute_Value_Length == 4)
  {
    /* Should be the range of Handles */
    uint16_t StartHandle = (((uint16_t) Attribute_Value[1]) << 8) | Attribute_Value[0];
    uint16_t StopHandle  = (((uint16_t) Attribute_Value[3]) << 8) | Attribute_Value[2];
    HSD_PRINTF("\tFrom Handles =0x%x to 0x%x\r\n", StartHandle, StopHandle);
  }

  HSD_PRINTF("Nothing to do except send confirmation\r\n");
#endif /* HSD_PRINTF_DEBUG */

  RetStatus = aci_gatt_confirm_indication(Connection_Handle);
  if (RetStatus != BLE_STATUS_SUCCESS)
  {
    HSD_PRINTF("aci_gatt_confirm_indicationt failed %d\r\n", RetStatus);
  }
  else
  {
    HSD_PRINTF("aci_gatt_confirm_indication Done\r\n");
  }
}

