/**
  ******************************************************************************
  * @file    sdcard_manager.c
  * @author  SRA
  *
  *
  * @brief   This file provides a set of functions to handle SDcard communication
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
#include "sdcard_manager.h"
#include "main.h"
#include "com_manager.h"
#include "cmsis_os.h"
#include "string.h"
#include "stdio.h"

#include "imp23absu_app.h"
#include "imp34dt05_app.h"
#include "ism330dhcx_app.h"
#include "iis3dwb_app.h"
#include "iis2mdc_app.h"
#include "iis2dh_app.h"
#include "hts221_app.h"
#include "lps22hh_app.h"
#include "stts751_app.h"

#include "HSD_json.h"
#include "HSDCore.h"
#include "AutoMode.h"

/* FatFs includes component */
#include "ff_gen_drv.h"
#include "sd_diskio.h"
#include "STWIN_bc.h"

/* Private includes ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define LOG_DIR_PREFIX    "STWIN_"

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

FATFS SDFatFs;  /* File system object for SD card logical drive */
FIL FileConfigHandler;
FIL FileLogError;
char SDPath[4]; /* SD card logical drive path */
DWORD memoryAvailable;
uint32_t SDTick = 0;

SD_HandleTypeDef hsd1;

uint8_t SD_Logging_Active = 0;
uint8_t SD_present = 0;
uint8_t init_SD_peripheral = 0;
uint8_t ConfigFromSD = 0;

char *g_prgUcfFileBuffer;
uint32_t g_prgUcfFileSize;

uint32_t t_click = 0;

volatile uint8_t BatteryLow = 0;

float activeBaudRate;
float activeSubSensors;

/**
  * Specifies the duration in ms of the next phase (datalog).
  */
static TickType_t s_nTimerPeriodMS = 0;

/**
  * Specifies a software timer used to stop the current datalog phase.
  */
static TimerHandle_t s_xStopTimer = NULL;

/**
  * Specifies the callback to notify the upper layer the end of a datalog execution phase.
  */
static SDMTaskStopEPCallback s_pfStopEPCallback = NULL;

osSemaphoreId sdioSem_id;
osSemaphoreDef(sdioSem);

osMessageQId sdThreadQueue_id;
osMessageQDef(sdThreadQueue, 100, int);

extern osTimerId bleAdvUpdaterTim_id;
extern osMessageQId bleSendThreadQueue_id;

extern volatile uint8_t UCF_loading;
extern volatile uint8_t ISM330DHCX_params_loading;
static stmdev_ctx_t MLC_ctx_instance = {SM_SPI_Write_Os, SM_SPI_Read_Os, NULL, &ism330dhcx_hdl_instance};

/* Private function prototypes -----------------------------------------------*/

osThreadId SDM_Thread_Id;
static void SDM_Thread(void const *argument);

static void SDM_Error_Handler(void);

static void Enable_Sensors(void);
static void Activate_Sensor(uint32_t id);

static void SDM_Boot(void);
static void SDM_DataReady(osEvent evt);
static void SDM_StartStopAcquisition(void);
static void SDM_StartAcquisition(void);
static void SDM_StopAcquisition(void);
static uint8_t checkRootFolder(void);
static uint8_t checkConfigJson(FILINFO fno);
static uint8_t checkConfigUcf(FILINFO fno);
static uint32_t readUCFfromSD(char *MLC_string);

uint8_t SDM_Memory_Init(void);
uint8_t SDM_Memory_Deinit(void);
void SDM_CalculateSdWriteBufferSize(COM_SubSensorStatus_t *pSubSensorStatus, uint32_t nBytesPerSample);
uint32_t SDM_GetLastDirNumber(void);

static uint32_t SDM_SaveData(void);
static uint32_t SDM_SaveDeviceConfig(char *dir_name);
static uint32_t SDM_SaveAcquisitionInfo(char *dir_name);
static uint32_t SDM_SaveUCF(char *dir_name);
static void switchOff_LowBatteryLowMemory(void);

/**
  * Function called when the Stop timer expires.
  *
  * @param xTimer [IN] specifies an handle to the expired timer.
  */
static void SDM_TimerStopCallbackFunction(TimerHandle_t xTimer);

/**
  * Initialize SD Card and file system.
  */
static inline void SDM_StartSDOperation(void);

/**
  * Deinitialize SD Card and file system.
  */
static inline void SDM_EndSDOperation(void);

/**
  * Split the file name in Name and Extension. Name and extension are separated by a'.'.
  * To save memory the function return
  * a pointer to the first character of the file name and
  * a pointer to the first character of the file extension
  *
  * @param pcFilename [IN] full name name
  * @param ppcName [OUT] pointer to the first character of the file name
  * @param ppcExtensio [OUT] pointer to the first character of the file extension
  * @return length of the file name if success, or -1 otherwise
  */
static int16_t SDM_SplitFileNameAndExt(char *pcFilename, char **ppcName, char **ppcExtension);



/*----------------------------------------------------------------------------*/
/**
  * @brief  Default enabled sensors
  * @param  None
  * @retval None
  */
void Enable_Sensors(void)
{
  /* Comment or uncomment each of the following lines
  * to chose which sensor you want to log.         */

  Activate_Sensor(HTS221_Get_Id());
  Activate_Sensor(IIS3DWB_Get_Id());
  Activate_Sensor(IIS2DH_Get_Id());
  Activate_Sensor(IIS2MDC_Get_Id());
  Activate_Sensor(IMP34DT05_Get_Id());
  Activate_Sensor(ISM330DHCX_Get_Id());
  Activate_Sensor(LPS22HH_Get_Id());
  Activate_Sensor(IMP23ABSU_Get_Id());
  Activate_Sensor(STTS751_Get_Id());
}


/**
  * @brief  Activate the sensor
  * @param  id: sensor id
  * @retval None
  */
void Activate_Sensor(uint32_t id)
{
  COM_SensorStatus_t *pSensorStatus = COM_GetSensorStatus(id);
  COM_SensorDescriptor_t *pSensorDescriptor = COM_GetSensorDescriptor(id);
  uint8_t i = 0;

  if (id == ISM330DHCX_Get_Id())  /* MLC subsensor should never start by default */
  {
    for (i = 0; i < 2; i++)
    {
      pSensorStatus->subSensorStatus[i].isActive = 1;
    }
    ISM330DHCX_params_loading += 1;
  }
  else
  {
    for (i = 0; i < pSensorDescriptor->nSubSensors; i++)
    {
      pSensorStatus->subSensorStatus[i].isActive = 1;
    }
  }
}


uint8_t SDM_CheckLowMemory(void)
{
  if (BSP_SD_IsDetected() == SD_PRESENT)
  {
    if (com_status == HS_DATALOG_IDLE)
    {
      SDM_StartSDOperation();
    }

    FATFS *SD;
    DWORD free_clusters, free_sectors, total_sectors;
    FRESULT fres = f_getfree(SDPath, &free_clusters, &SD);
    if (fres != FR_OK)
    {
      while (1);
    }

    if (com_status == HS_DATALOG_IDLE)
    {
      SDM_EndSDOperation();
    }

    total_sectors = (SD->n_fatent - 2) * SD->csize;
    free_sectors = free_clusters * SD->csize;

    if (free_sectors < 0.1 * total_sectors)
    {
      return 1;
    }
    else
    {
      return 0;
    }
  }
  return 0;
}


/*----------------------------------------------------------------------------*/
/**
  * @brief  SD card main thread
  * @retval None
  */
static void SDM_Thread(void const *argument)
{
  (void)argument;
  osEvent evt;

  SDM_Boot();

#if (configUSE_APPLICATION_TASK_TAG == 1 && defined(TASK_SDM_DEBUG_PIN))
  vTaskSetApplicationTaskTag(NULL, (TaskHookFunction_t)TASK_SDM_DEBUG_PIN);
#endif /* (configUSE_APPLICATION_TASK_TAG == 1 && defined(TASK_SDM_DEBUG_PIN)) */
  for (;;)
  {
    BSP_LED_Off(LED_GREEN);

    /* If the battery is too low close the file and turn off the system */
    if (BatteryLow == 1)
    {
      switchOff_LowBatteryLowMemory();
    }
    evt = osMessageGet(sdThreadQueue_id, osWaitForever);  /* wait for message */

    if (com_status == HS_DATALOG_IDLE || com_status == HS_DATALOG_SD_STARTED)
    {
      BSP_LED_On(LED_GREEN);

      if (evt.status == osEventMessage)                 /* check the received message */
      {
        if (evt.value.v == SDM_START_STOP)              /* start/stop acquisition command */
        {
          SDM_StartStopAcquisition();
        }
        if (evt.value.v == SDM_WRITE_UCF_TO_ROOT)       /* write ucf to sd card command */
        {
          SDM_WriteUCF(g_prgUcfFileBuffer, g_prgUcfFileSize);
          HSD_free(g_prgUcfFileBuffer);
        }
        else if (evt.value.v & SDM_DATA_READY_MASK)     /* transfer data to sd card command */
        {
          SDM_DataReady(evt);
        }
        else
        {

        }
      }

      if (HAL_GetTick() - SDTick > 60000)               /* Check SD card memory available every minute */
      {
        SDTick = HAL_GetTick();

        if (SDM_CheckLowMemory() == 1)
        {
          switchOff_LowBatteryLowMemory();
        }
      }
    }
  }
}


/**
  * @brief  Check if SD Card is inserted and search for possible config files
  * @param  None
  * @retval None
  */
static void SDM_Boot(void)
{
  if (BSP_SD_IsDetected())
  {
    SDM_StartSDOperation();

    ConfigFromSD = checkRootFolder();

    if (ConfigFromSD < SDM_MLC_CONFIG)
    {
      Enable_Sensors();
      update_sensors_config();
    }

    SDM_EndSDOperation();
  }
}

/**
  * @brief  Handle SDM_START_STOP task message
  * @param  None
  * @retval None
  */
static void SDM_StartStopAcquisition(void)
{
  if (SD_Logging_Active == 0)
  {
    SDM_StartAcquisition();
  }
  else if (SD_Logging_Active == 1)
  {
    SDM_StopAcquisition();

    if (s_pfStopEPCallback != NULL)     /* notify the upper layer */
    {
      s_pfStopEPCallback();
    }
  }
}

static void SDM_StartAcquisition(void)
{
  com_status = HS_DATALOG_SD_STARTED;
  SM_TIM_Start();

  osTimerStop(bleAdvUpdaterTim_id);

  COM_GenerateAcquisitionUUID();

  if (BSP_SD_IsDetected())
  {
    SDM_StartSDOperation();
    SD_present = 1;
    if (SDM_InitFiles() == 0)
    {
      SD_Logging_Active = 1;
      BSP_LED_Off(LED_ORANGE);
    }
    HSD_TAGS_timer_start();

    if (s_nTimerPeriodMS)
    {
      /* start the software stop timer */
      xTimerChangePeriod(s_xStopTimer, pdMS_TO_TICKS(s_nTimerPeriodMS), pdMS_TO_TICKS(200));
    }
  }
  else
  {
    SD_present = 0;
  }
}

static void SDM_StopAcquisition(void)
{
  if (SDM_CloseFiles() == 0)
  {
    SD_Logging_Active = 0;
#if (HSD_BLE_ENABLE == 1)
    osMessagePut(bleSendThreadQueue_id, COM_REQUEST_DEVICEREFRESH, 0);
#endif /* (HSD_BLE_ENABLE == 1) */
  }

  SDM_EndSDOperation();
  com_status = HS_DATALOG_IDLE;
  osTimerStart(bleAdvUpdaterTim_id, 3000);
}

/**
  * @brief  Handle SDM_DATA_READY_MASK task message
  * @param  None
  * @retval None
  */
static void SDM_DataReady(osEvent evt)
{
  COM_SubSensorStatus_t *pSubSensorStatus;
  COM_SubSensorContext_t *pSubSensorContext;
  uint32_t buf_size;
  uint8_t sID = (uint8_t)(evt.value.v & SDM_SENSOR_ID_MASK);
  uint8_t ssID = (uint8_t)((evt.value.v & SDM_SUBSENSOR_ID_MASK) >> 8);

  pSubSensorStatus = COM_GetSubSensorStatus(sID, ssID);
  pSubSensorContext = COM_GetSubSensorContext(sID, ssID);

  buf_size = pSubSensorStatus->sdWriteBufferSize;

  if (evt.value.v & SDM_DATA_FIRST_HALF_MASK) /* Data available on first half of the circular buffer */
  {
    SDM_WriteBuffer(sID, ssID, pSubSensorContext->sd_write_buffer, buf_size);
  }
  else /* Data available on second half of the circular buffer */
  {
    SDM_WriteBuffer(sID, ssID, (uint8_t *)(pSubSensorContext->sd_write_buffer + buf_size), buf_size);
  }
}

/**
  * @brief  Check if a custom configuration JSON or UCF is available in the root folder of the SD Card
  * @param  None
  * @retval 1 if there is a Config JSON, else 0
  */
uint8_t checkRootFolder(void)
{
  DIR dir;
  static FILINFO fno;
  static FILINFO fno_json;
  static FILINFO fno_ucf;
  uint8_t ret = 0;

  (void)f_opendir(&dir, "/");                   /* Open the root directory */

  for (;;)
  {
    (void)f_readdir(&dir, &fno);                /* Read files in root folder */
    if (fno.fname[0] == 0)
    {
      break;
    }

    if (fno.fattrib & AM_ARC)                   /* It is a file. */
    {
      char *pcName = NULL;
      char *pcExt = NULL;
      int16_t nNameLength = SDM_SplitFileNameAndExt(&fno.fname[0], &pcName, &pcExt);
      if (nNameLength != -1)
      {
        if (strncmp(pcExt, "json", 4) == 0)     /* file name has a 'json' extension */
        {
          fno_json = fno;
        }
        else if (strncmp(pcExt, "ucf", 3) == 0) /* file name has a 'ucf' extension*/
        {
          fno_ucf = fno;
        }
      }
    }

    if (fno_json.fname != NULL)                         /* First load JSON configuration (if available) */
    {
      ret += checkConfigJson(fno_json);
    }
    if (fno_ucf.fname != NULL)                          /* Then load UCF configuration (if available) */
    {
      ret += checkConfigUcf(fno_ucf);
    }
  }

  f_closedir(&dir);
  return ret;
}


/**
  * @brief  Open and parse the configuration JSON and update the device model
  * @param  FILINFO fno
  * @retval SDM_JSON_CONFIG if JSON is opened and parsed, else SDM_DEFAULT_CONFIG
  */
uint8_t checkConfigJson(FILINFO fno)
{
  uint8_t ret = SDM_DEFAULT_CONFIG;
  FIL FileConfigJSON;
  int32_t checkDeviceConfig = strncmp(fno.fname, "DeviceConfig.json", 17); /* Check if JSON name is DeviceConfig.json */

  if (checkDeviceConfig == 0) /* 0 -> Valid file name -> Load JSON */
  {
    if (f_open(&FileConfigJSON, fno.fname, FA_OPEN_EXISTING | FA_READ) == FR_OK) /* Open JSON file */
    {
      char *config_JSON_string = NULL;
      int32_t sizeFile;
      UINT br;
      sizeFile = f_size(&FileConfigJSON) + 1;

      config_JSON_string = HSD_malloc(sizeFile);
      if (config_JSON_string == NULL)
      {
        HSD_PRINTF("Mem alloc error [%d]: %d@%s\r\n", sizeFile, __LINE__, __FILE__);
      }
      else
      {
        HSD_PRINTF("Mem alloc ok [%d]: %d@%s\r\n", sizeFile, __LINE__, __FILE__);
      }

      f_read(&FileConfigJSON, config_JSON_string, sizeFile, &br); /* Read the file */
      SDM_ReadJSON(config_JSON_string); /* Parse and update */
      HSD_JSON_free(config_JSON_string);
      config_JSON_string = NULL;
      f_close(&FileConfigJSON);
      ret = SDM_JSON_CONFIG;
    }
  }
  else
  {
    /* Check if JSON name is AutoMode.json */
    checkDeviceConfig = strncmp(fno.fname, AMC_CFG_FILE_NAME, strlen(AMC_CFG_FILE_NAME));

    if (checkDeviceConfig == 0)
    {
      /* valid file name. Load and try to parse it. */
      if (f_open(&FileConfigJSON, fno.fname, FA_OPEN_EXISTING | FA_READ) == FR_OK)
      {
        char *pcCfgJson = NULL;
        int32_t nFileSize = 0;
        UINT nByteRead = 0;

        nFileSize = f_size(&FileConfigJSON) + 1;

        pcCfgJson = HSD_malloc(nFileSize);
        if (pcCfgJson == NULL)
        {
          HSD_PRINTF("Mem alloc error [%ld]: %d@%s\r\n", nFileSize, __LINE__, __FILE__);
        }
        else
        {
          HSD_PRINTF("Mem alloc ok [%ld]: %d@%s\r\n", nFileSize, __LINE__, __FILE__);
        }

        /* read the file */
        f_read(&FileConfigJSON, pcCfgJson, nFileSize, &nByteRead);
        AMLoadCfgFromString(pcCfgJson);
        HSD_JSON_free(pcCfgJson);
        pcCfgJson = NULL;
        f_close(&FileConfigJSON);
        ret = SDM_JSON_CONFIG;
      }
    }
  }
  return ret;
}


/**
  * @brief  Open and read the UCF file and load the MLC configuration
  * @param  FILINFO fno
  * @retval SDM_MLC_CONFIG if MLC id configurated, else SDM_DEFAULT_CONFIG
  */
uint8_t checkConfigUcf(FILINFO fno)
{
  uint8_t ret = SDM_DEFAULT_CONFIG;
  FIL FileConfigMLC;
  if (f_open(&FileConfigMLC, fno.fname, FA_OPEN_EXISTING | FA_READ) == FR_OK) /* Open UCF file */
  {
    char *config_MLC_string = NULL;
    int32_t sizeFile;
    UINT br;
    sizeFile = f_size(&FileConfigMLC) + 1;

    config_MLC_string = HSD_malloc(sizeFile);
    if (config_MLC_string == NULL)
    {
      HSD_PRINTF("Mem alloc error [%d]: %d@%s\r\n", sizeFile, __LINE__, __FILE__);
    }
    else
    {
      HSD_PRINTF("Mem alloc ok [%d]: %d@%s\r\n", sizeFile, __LINE__, __FILE__);
    }

    f_read(&FileConfigMLC, config_MLC_string, sizeFile, &br);   /* Read the file */
    readUCFfromSD(config_MLC_string);                         /* Load MLC configuration */
    HSD_free(config_MLC_string);
    config_MLC_string = NULL;
    f_close(&FileConfigMLC);
    ret = SDM_MLC_CONFIG;
  }
  return ret;
}


/**
  * @brief  When battery or memory are low, save the last available data and switch off the board
  * @param  None
  * @retval None
  */
void switchOff_LowBatteryLowMemory(void)
{
  SM_TIM_Stop();
  if (SDM_CloseFiles() == 0)
  {
    SD_Logging_Active = 0;
  }

  SDM_EndSDOperation();
  BSP_BC_CmdSend(SHIPPING_MODE_ON);
}


/**
  * @brief  PWR PVD interrupt callback
  * @param  None
  * @retval None
  */
void HAL_PWR_PVDCallback(void)
{
  BatteryLow = 1;
}


/**
  * @brief  SD Card Manager memory initialization. Performs the dynamic allocation for
  *         the sd_write_buffer associated to each active subsensor.
  * @param
  * @retval 1: no error
  */
uint8_t SDM_Memory_Init(void)
{
  COM_DeviceDescriptor_t *pDeviceDescriptor = COM_GetDeviceDescriptor();
  COM_SensorDescriptor_t *pSensorDescriptor;
  COM_SubSensorStatus_t *pSubSensorStatus;
  COM_SubSensorContext_t *pSubSensorContext;
  uint32_t sID;
  uint32_t ssID;
  uint32_t nBytesPerSample;

  for (sID = 0; sID < pDeviceDescriptor->nSensor; sID++)
  {
    pSensorDescriptor = COM_GetSensorDescriptor(sID);

    for (ssID = 0; ssID < pSensorDescriptor->nSubSensors; ssID++)
    {
      pSubSensorStatus = COM_GetSubSensorStatus(sID, ssID);
      pSubSensorContext = COM_GetSubSensorContext(sID, ssID);
      if (pSubSensorStatus->isActive)
      {
        nBytesPerSample = COM_GetnBytesPerSample(sID, ssID);
        SDM_CalculateSdWriteBufferSize(pSubSensorStatus, nBytesPerSample);
        pSubSensorContext->sd_write_buffer = HSD_malloc(pSubSensorStatus->sdWriteBufferSize * 2);
        if (pSubSensorContext->sd_write_buffer == NULL)
        {
          HSD_PRINTF("Mem alloc error [%ld]: %d@%s\r\n", pSubSensorStatus->sdWriteBufferSize * 2, __LINE__, __FILE__);
          SDM_Error_Handler();
        }
        else
        {
          HSD_PRINTF("Mem alloc ok [%ld]: %d@%s\r\n", pSubSensorStatus->sdWriteBufferSize * 2, __LINE__, __FILE__);
        }
      }
      else
      {
        pSubSensorContext->sd_write_buffer = 0;
      }
    }
  }

  return 1;
}


/**
  * @brief  Calculate SdWriteBufferSize for each active subSensor.
  *       SdWriteBufferSize is proportional to subSensor effective baud rate.
  * @param
  * @retval 1: no error
  */
void SDM_CalculateSdWriteBufferSize(COM_SubSensorStatus_t *pSubSensorStatus, uint32_t nBytesPerSample)
{
  uint32_t bufferSize;  /* Amount of data written on SD card for each fwrite */
  /* fixedRAM = total miminum data used by the active subSensors. Each subSensor should use at least SDM_MIN_BUFFER_SIZE bytes */
  float fixedRAM = SDM_MIN_BUFFER_SIZE * activeSubSensors;
  /* variableRAM = data space available. Can be assigned to each subSensor in base of its effective baudrate */
  float variableRAM = SDM_BUFFER_RAM_USAGE - fixedRAM;

  if (pSubSensorStatus->ODR != 0)       /* Setup bufferSize in base of the subSensor ODR and the RAM available */
  {
    /* nBytesPerSecond = # bytes used by the subSensor for each second */
    float nBytesPerSecond = pSubSensorStatus->ODR * nBytesPerSample;
    /* activeBaudRate = sum of nBytesPerSecond for all active subSensors */
    /* bufferSize = weighted average of the RAM available in base of effective baud rate */
    bufferSize = (uint32_t)((nBytesPerSecond * (variableRAM / activeBaudRate)) / 2.0f);
    /* bufferSize = mod(bufferSize,SDM_MIN_BUFFER_SIZE) */
    bufferSize = bufferSize - (bufferSize % SDM_MIN_BUFFER_SIZE) + SDM_MIN_BUFFER_SIZE;

    /* Data are written on SD card when half buffer is available */
    /* enter here if bufferSize > SDM_MAX_WRITE_TIME seconds of data */
    /* so to avoid not to writing for more than SDM_MAX_WRITE_TIME seconds */
    if (bufferSize > SDM_MAX_WRITE_TIME * pSubSensorStatus->ODR * nBytesPerSample)
    {
      /* clip bufferSize to SDM_MAX_WRITE_TIME seconds of data */
      bufferSize = SDM_MAX_WRITE_TIME * (uint32_t)(pSubSensorStatus->ODR * nBytesPerSample);

      /* check samplesPerTimestamp is valid (set SDM_MAX_WRITE_TIME seconds of data or clip to 1000) */
      if (pSubSensorStatus->samplesPerTimestamp != 0)
      {
        uint32_t samplesPerTS = (uint32_t)(SDM_MAX_WRITE_TIME * pSubSensorStatus->ODR);
        if (samplesPerTS > MAX_SPTS)
        {
          pSubSensorStatus->samplesPerTimestamp = MAX_SPTS;
        }
        else
        {
          pSubSensorStatus->samplesPerTimestamp = samplesPerTS;
        }
      }
    }
  }
  else  /* ODR = 0 is used for not sampled subSensors (i.e.: virtual sensors, MLC) */
  {
    bufferSize = nBytesPerSample;  /* Buffer size = length of the subSensor output */
    pSubSensorStatus->samplesPerTimestamp = 1;  /* write a timestamp for each sample */
  }

  pSubSensorStatus->sdWriteBufferSize = bufferSize;
}

/**
  * @brief  SD Card Manager memory De-initialization.
  * @param
  * @retval 1: no error
  */
uint8_t SDM_Memory_Deinit(void)
{
  COM_DeviceDescriptor_t *pDeviceDescriptor = COM_GetDeviceDescriptor();
  COM_SensorDescriptor_t *pSensorDescriptor;
  COM_SubSensorStatus_t *pSubSensorStatus;
  COM_SubSensorContext_t *pSubSensorContext;
  uint32_t sID;
  uint32_t ssID;

  for (sID = 0; sID < pDeviceDescriptor->nSensor; sID++)
  {
    pSensorDescriptor = COM_GetSensorDescriptor(sID);

    for (ssID = 0; ssID < pSensorDescriptor->nSubSensors; ssID++)
    {
      pSubSensorStatus = COM_GetSubSensorStatus(sID, ssID);
      pSubSensorContext = COM_GetSubSensorContext(sID, ssID);
      if (pSubSensorStatus->isActive && pSubSensorContext->sd_write_buffer != 0)
      {
        HSD_free(pSubSensorContext->sd_write_buffer);
        pSubSensorContext->sd_write_buffer = NULL;
      }
    }
  }

  return 1;
}


/**
  * @brief  SDM_Peripheral_Init
  * @param  None
  * @retval None
  */
void SDM_Peripheral_Init(void)
{
  BSP_SD_Detect_Init();
}


/**
  * @brief  Initialize SD Card Manager thread and queue
  * @param  None
  * @retval None
  */
void SDM_OS_Init(void)
{
  sdioSem_id = osSemaphoreCreate(osSemaphore(sdioSem), 1);
  osSemaphoreWait(sdioSem_id, osWaitForever);

  sdThreadQueue_id = osMessageCreate(osMessageQ(sdThreadQueue), NULL);
  vQueueAddToRegistry(sdThreadQueue_id, "sdThreadQueue_id");

  /* Thread definition: read data */
  osThreadDef(SDManager_Thread, SDM_Thread, SD_THREAD_PRIO, 1, 4096 / 4);

  /*  create the software timer: one-shot timer. The period is changed in the START command execution. */
  s_xStopTimer = xTimerCreate("SDMTim", 1, pdFALSE, NULL, SDM_TimerStopCallbackFunction);
  if (s_xStopTimer == NULL)
  {
    SDM_Error_Handler();
  }

  /* Start thread 1 */
  SDM_Thread_Id = osThreadCreate(osThread(SDManager_Thread), NULL);
}


/**
  * @brief  Initialize SD Card and file system
  * @param  None
  * @retval None
  */
void SDM_SD_Init(void)
{
  if (FATFS_LinkDriver(&SD_Driver, SDPath) == 0)
  {
    /* Register the file system object to the FatFs module */
    if (f_mount(&SDFatFs, (TCHAR const *)SDPath, 0) != FR_OK)
    {
      SDM_Error_Handler();
    }
  }
}


/**
  * @brief  Deinitialize SD Card and file system
  * @param  None
  * @retval None
  */
void SDM_SD_DeInit(void)
{
  if (FATFS_UnLinkDriver(SDPath) == 0)
  {
    /* Register the file system object to the FatFs module */
    if (f_mount(NULL, (TCHAR const *)SDPath, 0) != FR_OK)
    {
      SDM_Error_Handler();
    }
  }
}


/**
  * @brief  Open error file
  * @param  name: name of the log file
  * @retval 1 for f_write error, else 0
  */
uint8_t SDM_OpenLogErrorFile(const char *name)
{
  uint32_t byteswritten;

  if (f_open(&FileLogError, (char const *)name, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
  {
    return 1;
  }
  if (f_write(&FileLogError, "WARNING: possible data loss at samples [seconds]: ", 50, (void *)&byteswritten) != FR_OK)
  {
    return 1;
  }
  return 0;
}


/**
  * @brief  PWR PVD interrupt callback
  * @param  None
  * @retval 1 for f_write error, else 0
  */
uint8_t SDM_OpenDatFile(uint8_t sID, uint8_t ssID, const char *sensorName)
{
  char file_name[54];

  FIL *p = &(COM_GetSubSensorContext(sID, ssID)->file_handler);
  sprintf(file_name, "%s%s", sensorName, ".dat");

  if (f_open(p, (const char *)file_name, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
  {
    return 1;
  }
  return 0;
}


uint8_t SDM_CloseFile(uint8_t sID, uint8_t ssID)
{
  FIL *p = &(COM_GetSubSensorContext(sID, ssID)->file_handler);
  return f_close(p);
}


/**
  * @brief  Scan SD Card file system to find the latest directory number that includes to the LOG_DIR_PREFIX
  * @param  None
  * @retval
  */
uint32_t SDM_GetLastDirNumber(void)
{
  FRESULT fr;     /* Return value */
  DIR dj;         /* Directory search object */
  FILINFO fno;    /* File information */
  int32_t dir_n = 0;
  int32_t tmp;
  char dir_name[sizeof(LOG_DIR_PREFIX) + 6] = LOG_DIR_PREFIX;

  dir_name[sizeof(LOG_DIR_PREFIX) - 1] = '*'; /* wildcard */
  dir_name[sizeof(LOG_DIR_PREFIX)] = 0;

  fr = f_findfirst(&dj, &fno, "", dir_name);  /* Start to search for matching directories */
  if (fno.fname[0])
  {
    tmp = strtol(&fno.fname[sizeof(LOG_DIR_PREFIX)], NULL, 10);
    if (dir_n < tmp)
    {
      dir_n = tmp;
    }
  }

  /* Repeat while an item is found */
  while (fr == FR_OK && fno.fname[0])
  {
    fr = f_findnext(&dj, &fno);   /* Search for next item */
    if (fno.fname[0])
    {
      tmp = strtol(&fno.fname[sizeof(LOG_DIR_PREFIX)], NULL, 10);
      if (tmp > dir_n)
      {
        dir_n = tmp;
      }
    }
  }

  f_closedir(&dj);
  return (uint32_t)dir_n;
}


/**
  * @brief  Open one file for each subsensor to store raw data and a JSON file with the device configuration
  * @param  None
  * @retval 1 for f_write error, else 0
  */
uint8_t SDM_InitFiles(void)
{
  COM_DeviceDescriptor_t *pDeviceDescriptor;
  COM_SensorDescriptor_t *pSensorDescriptor;

  uint8_t sensorIsActive;
  uint32_t sID = 0;
  uint32_t ssID = 0;
  uint32_t dir_n = 0;
  char dir_name[sizeof(LOG_DIR_PREFIX) + 6];
  char file_name[50];

  activeBaudRate = 0;
  activeSubSensors = 0;
  pDeviceDescriptor = COM_GetDeviceDescriptor();
  dir_n = SDM_GetLastDirNumber();
  dir_n++;

  sprintf(dir_name, "%s%05ld", LOG_DIR_PREFIX, dir_n);

  FRESULT test = f_mkdir(dir_name);
  if (test != FR_OK)
  {
    return 1;
  }

  for (sID = 0; sID < pDeviceDescriptor->nSensor; sID++)
  {
    pSensorDescriptor = COM_GetSensorDescriptor(sID);

    for (ssID = 0; ssID < pSensorDescriptor->nSubSensors; ssID++)
    {
      if (COM_GetSubSensorStatus(sID, ssID)->isActive)
      {
        char subSensorName[6];
        switch (pSensorDescriptor->subSensorDescriptor[ssID].sensorType)
        {
          case COM_TYPE_ACC:
            sprintf(subSensorName, "ACC");
            break;
          case COM_TYPE_MAG:
            sprintf(subSensorName, "MAG");
            break;
          case COM_TYPE_GYRO:
            sprintf(subSensorName, "GYRO");
            break;
          case COM_TYPE_TEMP:
            sprintf(subSensorName, "TEMP");
            break;
          case COM_TYPE_PRESS:
            sprintf(subSensorName, "PRESS");
            break;
          case COM_TYPE_HUM:
            sprintf(subSensorName, "HUM");
            break;
          case COM_TYPE_MIC:
            sprintf(subSensorName, "MIC");
            break;
          case COM_TYPE_MLC:
            sprintf(subSensorName, "MLC");
            break;
          default:
            sprintf(subSensorName, "NA");
            break;
        }
        sprintf(file_name, "%s/%s_%s", dir_name, pSensorDescriptor->name, subSensorName);
        if (SDM_OpenDatFile(sID, ssID, file_name) != 0)
        {
          return 1;
        }
        activeBaudRate += COM_GetSubSensorStatus(sID, ssID)->ODR * COM_GetnBytesPerSample(sID, ssID);
        activeSubSensors += 1;
      }
    }
  }

  SDM_Memory_Init();

  for (sID = 0; sID < pDeviceDescriptor->nSensor; sID++)
  {
    sensorIsActive = 0;
    pSensorDescriptor = COM_GetSensorDescriptor(sID);

    for (ssID = 0; ssID < pSensorDescriptor->nSubSensors; ssID++)
    {
      COM_ResetSubSensorContext(sID, ssID);
      sensorIsActive |= COM_GetSubSensorStatus(sID, ssID)->isActive;
    }
    /* Sensor is Active if at least one of the SubSensor is active */
    if (sensorIsActive)
    {
      SM_StartSensorThread(sID);
    }
  }
  return 0;
}

uint8_t SDM_UpdateDeviceConfig(void)
{
  FIL fil;        /* File object */
  FRESULT fr;     /* FatFs return code */
  char *JSON_string = NULL;
  uint32_t byteswritten;
  uint32_t size;

  SDM_StartSDOperation();

  fr = f_open(&fil, "DeviceConfig.json", FA_OPEN_ALWAYS | FA_WRITE);
  if (fr != FR_OK)
  {
    return 1;
  }

  size = SDM_CreateJSON(&JSON_string);
  fr = f_write(&fil, (uint8_t *)JSON_string, size - 1, (void *)&byteswritten);
  if (fr != FR_OK)
  {
    return 1;
  }

  fr = f_close(&fil);
  if (fr != FR_OK)
  {
    return 1;
  }

  SDM_EndSDOperation();

  HSD_JSON_free(JSON_string);
  JSON_string = NULL;
  return fr;
}


/**
  * @brief  Close all files
  * @param  None
  * @retval 1 for f_write error, else 0
  */
uint8_t SDM_CloseFiles(void)
{
  char dir_name[sizeof(LOG_DIR_PREFIX) + 6];
  uint32_t dir_n = 0;

  /* Put all the sensors in "SUSPENDED" mode, write and close all data files */
  if (SDM_SaveData())
  {
    return 1;
  }

  dir_n = SDM_GetLastDirNumber();
  sprintf(dir_name, "%s%05ld", LOG_DIR_PREFIX, dir_n);

  /* Write DeviceConfig.json */
  if (SDM_SaveDeviceConfig(dir_name))
  {
    return 1;
  }
  /* Write Data Tags */
  if (SDM_SaveAcquisitionInfo(dir_name))
  {
    return 1;
  }
  /* Copy UCF file in the Acquisition folder */
  if (SDM_SaveUCF(dir_name))
  {
    return 1;
  }

  return 0;
}

static uint32_t SDM_SaveData(void)
{
  COM_DeviceDescriptor_t *pDeviceDescriptor = COM_GetDeviceDescriptor();
  COM_SensorDescriptor_t *pSensorDescriptor;
  uint32_t i = 0;
  uint32_t n = 0;

  /* Put all the sensors in "SUSPENDED" mode */
  for (i = 0; i < pDeviceDescriptor->nSensor; i++)
  {
    pSensorDescriptor = COM_GetSensorDescriptor(i);

    for (n = 0; n < pSensorDescriptor->nSubSensors; n++)
    {
      if (COM_GetSubSensorStatus(i, n)->isActive)
      {
        SDM_Flush_Buffer(i, n);
        if (SDM_CloseFile(i, n) != 0)
        {
          return 1;
        }
      }
    }
  }

  /* Deallocate here SD buffers to have enough memory for next section */
  SDM_Memory_Deinit();
  return 0;
}

static uint32_t SDM_SaveDeviceConfig(char *dir_name)
{
  char *JSON_string = NULL;
  uint32_t byteswritten;
  uint32_t size;
  char file_name[50];

  sprintf(file_name, "%s/DeviceConfig.json", dir_name);

  if (f_open(&FileConfigHandler, (char const *)file_name, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
  {
    return 1;
  }
  size = SDM_CreateJSON(&JSON_string);
  if (f_write(&FileConfigHandler, (uint8_t *)JSON_string, size - 1, (void *)&byteswritten) != FR_OK)
  {
    return 1;
  }
  if (f_close(&FileConfigHandler) != FR_OK)
  {
    return 1;
  }

  HSD_JSON_free(JSON_string);
  JSON_string = NULL;
  return 0;
}

static uint32_t SDM_SaveAcquisitionInfo(char *dir_name)
{
  char *JSON_string = NULL;
  uint32_t byteswritten;
  uint32_t size;
  char file_name[50];

  sprintf(file_name, "%s/AcquisitionInfo.json", dir_name);

  if (f_open(&FileConfigHandler, (char const *)file_name, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
  {
    return 1;
  }
  size = SDM_CreateAcquisitionJSON(&JSON_string);
  if (f_write(&FileConfigHandler, (uint8_t *)JSON_string, size - 1, (void *)&byteswritten) != FR_OK)
  {
    return 1;
  }
  if (f_close(&FileConfigHandler) != FR_OK)
  {
    return 1;
  }

  HSD_JSON_free(JSON_string);
  JSON_string = NULL;
  return 0;
}

static uint32_t SDM_SaveUCF(char *dir_name)
{
  char file_name[258];
  FILINFO fno;    /* File information */
  DIR dir;
  uint8_t isMLC = 7;

  (void)f_opendir(&dir, "/");                   /* Open the root directory */

  for (;;)
  {
    (void)f_readdir(&dir, &fno);                /* Read files in root folder */
    if (fno.fname[0] == 0)
    {
      break;
    }

    if (fno.fattrib & AM_ARC)                   /* It is a file. */
    {
      char *pch = NULL;
      char local_name[256];
      strncpy(local_name, &fno.fname[0], 256);  /* Copy file name */
      pch = strtok(local_name, " .-,_\r\n");    /* Exclude separators from the search */

      while (pch != NULL)                       /* Check files until the end of file list */
      {
        isMLC = strncmp(pch, "ucf", 3);
        if (isMLC == 0)                         /* file name has a 'ucf' extension*/
        {
          FIL FileSourceConfigMLC;
          FIL FileDestConfigMLC;
          BYTE buffer[50];                      /* File copy buffer */
          UINT br, bw;                          /* File read/write count */

          size_t nBuffSize = sizeof(file_name);
          int32_t ret = snprintf(file_name, nBuffSize, "%s", dir_name);
          if ((ret > 0) && (ret + 1 < nBuffSize))
          {
            ret = snprintf(&file_name[ret], nBuffSize - ret, "/%s", fno.fname);
          }
          if (ret < 0)
          {
            /* file name has been truncated do something */
          }
          if (f_open(&FileSourceConfigMLC, fno.fname, FA_OPEN_EXISTING | FA_READ) == FR_OK) /* Open UCF file */
          {
            f_open(&FileDestConfigMLC, file_name, FA_WRITE | FA_CREATE_ALWAYS);
            /* Copy source to destination */
            for (;;)
            {
              f_read(&FileSourceConfigMLC, buffer, sizeof buffer, &br);  /* Read a chunk of data from the source file */
              if (br == 0)
              {
                break;
              } /* error or eof */
              f_write(&FileDestConfigMLC, buffer, br, &bw);            /* Write it to the destination file */
              if (bw < br)
              {
                break;
              } /* error or disk full */
            }
            f_close(&FileSourceConfigMLC);
            f_close(&FileDestConfigMLC);
          }
        }
        pch = strtok(NULL, " .-,_\r\n");
      }
    }
  }
  f_closedir(&dir);
  return 0;
}

/**
  * @brief
  * @param
  * @retval
  */
uint8_t SDM_WriteConfigBuffer(uint8_t *buffer, uint32_t size)
{
  uint32_t byteswritten;
  FRESULT returnWrite;

  returnWrite = f_write(&FileConfigHandler, buffer, size, (void *)&byteswritten);
  if (returnWrite != FR_OK)
  {
    return 0;
  }
  return 1;
}


/**
  * @brief  Write data buffer to SD card
  * @param  None
  * @retval 1 for f_write error, else 0
  */
uint8_t SDM_WriteBuffer(uint8_t sID, uint8_t ssID, uint8_t *buffer, uint32_t size)
{
  uint32_t byteswritten;
  FIL *p = &(COM_GetSubSensorContext(sID, ssID)->file_handler);

  if (f_write(p, buffer, size, (void *)&byteswritten) != FR_OK)
  {
    return 1;
  }
  return 0;
}


/**
  * @brief  Write down all the data available
  * @param  id: sensor id
  * @retval 1 for f_write error, else 0
  */
/* Write remaining data to file */
uint8_t SDM_Flush_Buffer(uint8_t sID, uint8_t ssID)
{
  uint8_t ret = 1;
  uint32_t bufSize = COM_GetSubSensorStatus(sID, ssID)->sdWriteBufferSize;
  COM_SubSensorContext_t *pSubSensorContext = COM_GetSubSensorContext(sID, ssID);

  if (pSubSensorContext->sd_write_buffer_idx > 0 && pSubSensorContext->sd_write_buffer_idx < (bufSize - 1))
  {
    /* flush from the beginning */
    ret = SDM_WriteBuffer(sID, ssID, pSubSensorContext->sd_write_buffer, pSubSensorContext->sd_write_buffer_idx + 1);
  }
  else if (pSubSensorContext->sd_write_buffer_idx > (bufSize - 1)
           && pSubSensorContext->sd_write_buffer_idx < (bufSize * 2 - 1))
  {
    /* flush from half buffer */
    ret =  SDM_WriteBuffer(sID, ssID, (uint8_t *)(pSubSensorContext->sd_write_buffer + bufSize), pSubSensorContext->sd_write_buffer_idx + 1 - bufSize);
  }

  pSubSensorContext->sd_write_buffer_idx = 0;
  return ret;
}


/**
  * @brief  Fill SD buffer with new data
  * @param  id: sensor id
  * @param  src: pointer to data buffer
  * @param  srcSize: buffer size
  * @retval 0: ok
  */
uint8_t SDM_Fill_Buffer(uint8_t sID, uint8_t ssID, uint8_t *src, uint16_t srcSize)
{
  uint8_t *dst;
  uint32_t dstP = 0;
  uint32_t srcP = 0;
  uint32_t dstSize;
  uint32_t bufSize;
  COM_SubSensorContext_t *pSubSensorContext = COM_GetSubSensorContext(sID, ssID);

  bufSize = COM_GetSubSensorStatus(sID, ssID)->sdWriteBufferSize;
  dstSize = bufSize * 2;

  dst = pSubSensorContext->sd_write_buffer;
  dstP = pSubSensorContext->sd_write_buffer_idx;

  /* byte per byte copy to SD buffer, automatic wrap */
  while (srcP < srcSize)
  {
    dst[dstP] = src[srcP];
    dstP++;
    srcP++;
    if (dstP >= dstSize)
    {
      dstP = 0;
    }
  }
  if (pSubSensorContext->sd_write_buffer_idx < (dstSize / 2) && dstP >= (dstSize / 2)) /* first half full */
  {
    /* unlock write task */
    if (osMessagePut(sdThreadQueue_id, sID | ssID << 8 | SDM_DATA_READY_MASK | SDM_DATA_FIRST_HALF_MASK, 0) != osOK)
    {
      SDM_Error_Handler();
    }
    /* check for buffer consistency */
  }
  else if (dstP < pSubSensorContext->sd_write_buffer_idx) /* second half full */
  {
    if (osMessagePut(sdThreadQueue_id, sID | ssID << 8 | SDM_DATA_READY_MASK | SDM_DATA_SECOND_HALF_MASK, 0) != osOK)
    {
      SDM_Error_Handler();
    }
  }

  pSubSensorContext->sd_write_buffer_idx = dstP;
  return 0;
}


/**
  * @brief  Read and parse Json string and update device model
  * @param  serialized_string: pointer to Json string
  * @retval 0: ok
  */
uint32_t SDM_ReadJSON(char *serialized_string)
{
  static COM_Device_t JSON_device;
  COM_Device_t *local_device;
  uint8_t ii;
  uint32_t size;

  local_device = COM_GetDevice();
  size = sizeof(COM_Device_t);

  memcpy(&JSON_device, local_device, size);
  HSD_JSON_parse_Device(serialized_string, &JSON_device);

  for (ii = 0; ii < JSON_device.deviceDescriptor.nSensor; ii++)
  {
    update_sensorStatus(&local_device->sensors[ii]->sensorStatus, &JSON_device.sensors[ii]->sensorStatus, ii);
  }

  update_sensors_config();
  update_tagList(local_device, &JSON_device);
  return 0;
}


/**
  * @brief  Serialize Json string from device model
  * @param  serialized_string: double pointer to json string
  * @retval size of the string
  */
uint32_t SDM_CreateJSON(char **serialized_string)
{
  COM_Device_t *device;
  uint32_t size;

  device = COM_GetDevice();
  size = HSD_JSON_serialize_Device(device, serialized_string, PRETTY_JSON);

  return size;
}


/**
  * @brief
  * @param
  * @retval size of the string
  */
uint32_t SDM_CreateAcquisitionJSON(char **serialized_string)
{
  COM_AcquisitionDescriptor_t *acquisition;
  uint32_t size;

  acquisition = COM_GetAcquisitionDescriptor();
  size = HSD_JSON_serialize_Acquisition(acquisition, serialized_string, PRETTY_JSON);

  return size;
}



uint32_t readUCFfromSD(char *MLC_string)
{
  int32_t ucf_reg;
  int32_t ucf_data;
  char *pch = NULL;

  pch = strtok(MLC_string, " -,_\r\n");
  while (pch != NULL)
  {
    if (strncmp(pch, "Ac", 2) == 0)
    {
      pch = strtok(NULL, " -,_\r\n");
      ucf_reg = strtol(pch, NULL, 16);
      pch = strtok(NULL, " -,_\r\n");
      ucf_data = strtol(pch, NULL, 16);
      ism330dhcx_write_reg(&MLC_ctx_instance, (uint8_t)ucf_reg, (uint8_t *)&ucf_data, 1);
    }
    pch = strtok(NULL, " -,_\r\n");
  }

  COM_SubSensorStatus_t *pSubSensorStatus = COM_GetSubSensorStatus(ISM330DHCX_Get_Id(), 2);
  pSubSensorStatus->ucfLoaded = 1;
  pSubSensorStatus->isActive = 1;
  UCF_loading = 1;
  ISM330DHCX_updateConfig();

  return 0;
}


void SDM_WriteUCF(char *ucfData, uint32_t ucfSize)
{
  uint32_t byteswritten;
  FIL FileConfigMLC;

  if (BSP_SD_IsDetected())
  {
    SDM_StartSDOperation();

    f_open(&FileConfigMLC, "ISM330DHCX_MLC.ucf", FA_CREATE_ALWAYS | FA_WRITE);
    f_write(&FileConfigMLC, (uint8_t *)ucfData, ucfSize, (void *)&byteswritten);
    f_close(&FileConfigMLC);
    SDM_EndSDOperation();

  }
}

static inline void SDM_StartSDOperation(void)
{
  if (init_SD_peripheral != 1)
  {
    SDM_SD_Init();
    init_SD_peripheral = 1;
  }
}

static inline void SDM_EndSDOperation(void)
{
  if (init_SD_peripheral != 0)
  {
    SDM_SD_DeInit();
    init_SD_peripheral = 0;
  }
}

static void SDM_TimerStopCallbackFunction(TimerHandle_t xTimer)
{
  StopExecutionPhases();
}

static int16_t SDM_SplitFileNameAndExt(char *pcFilename, char **ppcName, char **ppcExtension)
{
  int16_t nLenght = strlen(pcFilename);
  for (int16_t i = nLenght - 1; i >= 0; --i)
  {
    if (pcFilename[i] == '.')
    {
      nLenght = i;
      *ppcName = &pcFilename[0];
      *ppcExtension = &pcFilename[i + 1];
      break;
    }
  }
  return nLenght;
}

uint8_t SDM_SetExecutionContext(TickType_t nStopTimerPeriodMS)
{
  s_nTimerPeriodMS = nStopTimerPeriodMS;
  return 1;
}

void SDM_SetStopEPCallback(SDMTaskStopEPCallback pfCallback)
{
  s_pfStopEPCallback = pfCallback;
}


/**
  * @brief  This function is executed in case of error occurrence
  * @param  None
  * @retval None
  */
static void SDM_Error_Handler(void)
{
  com_status = HS_DATALOG_IDLE;
  while (1)
  {
    HAL_Delay(200);
    BSP_LED_On(LED_GREEN);
    BSP_LED_On(LED_ORANGE);
    HAL_Delay(600);
    BSP_LED_Off(LED_GREEN);
    BSP_LED_Off(LED_ORANGE);
  }
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


