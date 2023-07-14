/**
  ******************************************************************************
  * @file    sdcard_manager.h
  * @author  SRA
  *
  *
  * @brief   Header for sdcard_manager.c module
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
#ifndef __SDCARD_MANAGER_H
#define __SDCARD_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os.h"

#define SDM_CMD_MASK                (0x00008000)
#define SDM_DATA_READY_MASK         (0x00004000)
#define SDM_DATA_FIRST_HALF_MASK    (0x00002000)
#define SDM_DATA_SECOND_HALF_MASK   (0x00000000)
#define SDM_SENSOR_ID_MASK          (0x000000FF)
#define SDM_SUBSENSOR_ID_MASK       (0x00000700)

#define SDM_START_STOP              (0x00000001|SDM_CMD_MASK)
#define SDM_WRITE_UCF_TO_ROOT       (0x00000002|SDM_CMD_MASK)

#define SDM_MAX_WRITE_TIME      2
#define SDM_BUFFER_RAM_USAGE    400000
#define SDM_MIN_BUFFER_SIZE     1024

#define SDM_DEFAULT_CONFIG      (uint8_t)(0x00)
#define SDM_MLC_CONFIG          (uint8_t)(0x01)
#define SDM_JSON_CONFIG         (uint8_t)(0x10)

extern osMessageQId sdThreadQueue_id;

extern char *g_prgUcfFileBuffer;
extern uint32_t g_prgUcfFileSize;

/**
  * Create a type name for the callback function. This callback is called when an execution phase is stopped.
  */
typedef void (*SDMTaskStopEPCallback)(void);

void SDM_OS_Init(void);
void SDM_Peripheral_Init(void);
void SDM_SD_Init(void);
void SDM_SD_DeInit(void);
uint8_t SDM_Memory_Init(void);

uint8_t SDM_InitFiles(void);
uint8_t SDM_CloseFiles(void);
uint8_t SDM_UpdateDeviceConfig(void);
uint8_t SDM_OpenLogErrorFile(const char *name);
uint8_t SDM_OpenDatFile(uint8_t sID, uint8_t ssID, const char *sensorName);
uint8_t SDM_CloseFile(uint8_t sID, uint8_t ssID);
uint8_t SDM_WriteBuffer(uint8_t sID, uint8_t ssID, uint8_t *buffer, uint32_t size);
uint8_t SDM_WriteConfigBuffer(uint8_t *buffer, uint32_t size);
uint8_t SDM_Flush_Buffer(uint8_t sID, uint8_t ssID);
uint8_t SDM_Fill_Buffer(uint8_t sID, uint8_t ssID, uint8_t *src, uint16_t srcSize);

uint32_t SDM_CreateJSON(char **serialized_string);
uint32_t SDM_ReadJSON(char *serialized_string);
uint32_t SDM_CreateAcquisitionJSON(char **serialized_string);

void SDM_WriteUCF(char *ucfData, uint32_t ucfSize);

uint8_t SDM_CheckLowMemory(void);

/**
  * Modifies one of the execution context. It copies the value of the parameters passed in the
  * execution context variable to the task execution context.
  *
  * @param nStopTimerPeriodMS [IN] specifies the duration in ms of the next datalog execution phase.
  *                           Zero means infinite period.
  * @return 0 if success, an error code otherwise.
  */
uint8_t SDM_SetExecutionContext(TickType_t nStopTimerPeriodMS);

/**
  * Set the stop execution phase callback. The callback is called when an execution phase ends.
  *
  * @param pfCallback [IN] specifies a callback. Pass a NULL pointer to disable the callback.
  */
void SDM_SetStopEPCallback(SDMTaskStopEPCallback pfCallback);

#ifdef __cplusplus
}
#endif

#endif /* __SDCARD_MANAGER_H */

