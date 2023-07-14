/**
  ******************************************************************************
  * @file    main.h
  * @brief   This file contains all the function prototypes for the main.c
  *          file.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_conf.h"
#include "systypes.h"

#include "apperror.h"
#include "SensorTile.box.h"

#ifndef M_PI
#define M_PI   3.14159265358979323846264338327950288
#endif /* M_PI */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

#define HSD_BLE_ENABLE                  1
#define HSD_BLE_STATUS_TIMER_ENABLE     1

/*
 * SD task priority should be lower than data acquisition tasks
 */
#define SD_THREAD_PRIO                  osPriorityNormal

/*
 * BLE threads are not critical and they shouldn't interfere with the data acquisition,
 * so they have a lower priority than the others.
 * BLE_USER_EVT_PROC_THREAD priority must be lower than BLE_SEND_THREAD priority.
 * Otherwise the EVT_PROC thread intecepts a message that should instead be received
 * by the BLE_SEND thread.
 */
#define BLE_USER_EVT_PROC_THREAD_PRIO   osPriorityLow
#define BLE_INIT_THREAD_PRIO            osPriorityBelowNormal
#define BLE_SEND_THREAD_PRIO            osPriorityBelowNormal

/* Exported macro ------------------------------------------------------------*/

/* Package Version only numbers 0->9 */
#define HSD_VERSION_MAJOR '1'
#define HSD_VERSION_MINOR '5'
#define HSD_VERSION_PATCH '0'

/* Print messages for debugging memory allocation */
#define HSD_PRINTF_DEBUG 0

#if HSD_PRINTF_DEBUG
/**
  * User can change here printf with a custom implementation.
  * For example:
  * #define HSD_PRINTF(...)   MY_PRINTF(__VA_ARGS__)
  */
#include <stdio.h>
#define HSD_PRINTF(...)                printf(__VA_ARGS__)
#else
#define HSD_PRINTF(...)
#endif /* HSD_PRINTF_DEBUG */

/* Exported variables ------------------------------------------------------- */
/* Exported prototypes ------------------------------------------------------- */
uint32_t StopExecutionPhases(void);

void *malloc_debug(size_t size);
void *calloc_debug(size_t num, size_t size);
void free_debug(void *mem);

void *gcc_malloc_debug(size_t size);
void *gcc_calloc_debug(size_t num, size_t size);
void gcc_free_debug(void *mem);

void *malloc_critical(size_t size);
void *calloc_critical(size_t num, size_t size);
void free_critical(void *mem);

void AMTAbortAutoMode(void);
boolean_t AMTIsStarted(void);
boolean_t AMTIsEnded(void);

void SetMLCOut(uint8_t *mlcInBuffer, uint8_t *mlcOutBuffer);
void GetMLCOut(uint8_t *mlcOutBuffer);

sys_error_code_t StartStop_AutoMode(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

