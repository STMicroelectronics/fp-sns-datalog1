/**
  ******************************************************************************
  * @file    HSDCore.h
  * @author  SRA - MCD
  *
  *
  * @brief   HSDCore global configuration file.
  *
  * This file contain the main configuration values for the module.
  * The application can provide its own value in the HSDConfig.h file.
  *
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
#ifndef HSDCORE_INC_HSDCORE_H_
#define HSDCORE_INC_HSDCORE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "HSDCoreConfig.h"

/* Task priorities */
/*******************/

#ifndef HSD_I2C_RD_THREAD_PRIO
#define HSD_I2C_RD_THREAD_PRIO                      osPriorityAboveNormal
#endif /* HSD_I2C_RD_THREAD_PRIO */

#ifndef HSD_SPI_RD_THREAD_PRIO
#define HSD_SPI_RD_THREAD_PRIO                      osPriorityAboveNormal
#endif /* HSD_SPI_RD_THREAD_PRIO */

#ifndef HSD_IIS3DWB_THREAD_PRIO
#define HSD_IIS3DWB_THREAD_PRIO                     osPriorityAboveNormal
#endif /* HSD_IIS3DWB_THREAD_PRIO */

#ifndef HSD_IIS2MDC_THREAD_PRIO
#define HSD_IIS2MDC_THREAD_PRIO                     osPriorityAboveNormal
#endif /* HSD_IIS2MDC_THREAD_PRIO */

#ifndef HSD_IIS2DH_THREAD_PRIO
#define HSD_IIS2DH_THREAD_PRIO                      osPriorityAboveNormal
#endif /* HSD_IIS2DH_THREAD_PRIO */

#ifndef HSD_STTS751_THREAD_PRIO
#define HSD_STTS751_THREAD_PRIO                     osPriorityAboveNormal
#endif /* HSD_STTS751_THREAD_PRIO */

#ifndef HSD_LPS22HH_THREAD_PRIO
#define HSD_LPS22HH_THREAD_PRIO                     osPriorityAboveNormal
#endif /* HSD_LPS22HH_THREAD_PRIO */

#ifndef HSD_HTS221_THREAD_PRIO
#define HSD_HTS221_THREAD_PRIO                      osPriorityAboveNormal
#endif /* HSD_HTS221_THREAD_PRIO */

#ifndef HSD_IMP34DT05_THREAD_PRIO
#define HSD_IMP34DT05_THREAD_PRIO                   osPriorityAboveNormal
#endif /* HSD_IMP34DT05_THREAD_PRIO */

#ifndef HSD_IMP23ABSU_THREAD_PRIO
#define HSD_IMP23ABSU_THREAD_PRIO                    osPriorityAboveNormal
#endif /* HSD_IMP23ABSU_THREAD_PRIO */

#ifndef HSD_ISM330DHCX_THREAD_PRIO
#define HSD_ISM330DHCX_THREAD_PRIO                  osPriorityAboveNormal
#endif /* HSD_ISM330DHCX_THREAD_PRIO */

/*
 * Each time a task is executing the corresponding pin is SET otherwise is RESET
 * Pins
 * Set configUSE_APPLICATION_TASK_TAG to 1 in FreeRTOSConfig.h to enable the Task debugging mode.
 */
#ifndef HSD_TASK_DEBUG_PINS_ENABLE
#define HSD_TASK_DEBUG_PINS_ENABLE    0
#endif /* HSD_TASK_DEBUG_PINS_ENABLE */

/*
 * HSD_USE_DUMMY_DATA, if enabled, replaces real sensor data with a 2 bytes idependend counter
 * for each sensor. Useful to debug the complete application and verify that data are stored or
 * streamed correctly.
 * Sensitivity parameter is forced to to 1.
 * Timestamp is not replaced, if you want to disable it, set samplesPerTS parameter to 0.
 */
#ifndef HSD_USE_DUMMY_DATA
#define HSD_USE_DUMMY_DATA                           0
#endif /* HSD_USE_DUMMY_DATA */

#ifndef HSD_malloc
#define HSD_malloc                                 malloc
#endif /* HSD_malloc */

#ifndef HSD_calloc
#define HSD_calloc                                 calloc
#endif /* HSD_calloc */

#ifndef HSD_free
#define HSD_free                                    free
#endif /* HSD_free */

#ifndef HSD_memset
#define HSD_memset                                  memset
#endif /* HSD_memset */

#ifndef HSD_memcpy
#define HSD_memcpy                                  memcpy
#endif /* HSD_memcpy */

#ifdef __cplusplus
}
#endif

#endif /* HSDCORE_INC_HSDCORE_H_ */
