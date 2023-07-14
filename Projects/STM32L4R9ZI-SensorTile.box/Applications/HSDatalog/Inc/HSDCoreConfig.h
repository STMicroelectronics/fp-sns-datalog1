/**
  ******************************************************************************
  * @file    HSDCoreConfig.h
  * @brief
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

#ifndef INC_HSDCORECONFIG_H_
#define INC_HSDCORECONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

#define HSD_MIN(a, b)             (((a) < (b)) ? (a) : (b))

#define MAX_SPTS 1000

/*
 * HSD_USE_DUMMY_DATA, if enabled, replaces real sensor data with a 2 bytes independent counter
 * for each sensor. Useful to debug the complete application and verify that data are stored or
 * streamed correctly.
 * Sensitivity parameter is forced to to 1.
 * Timestamp is not replaced, if you want to disable it, set samplesPerTS parameter to 0.
 */
#define HSD_USE_DUMMY_DATA       0

/*
 The watermark defines the level of the sensor queue that triggers the IRQ.
 LSM6DSOX_MAX_WTM_LEVEL is used to compute the the watermark.
 Normally the signal size used by NanoEdg AI library is much bigger than MAX_WTM,
 but there are cases (sensor with very low ODR) that require a smaller value for the watermark.
 #define LSM6DSOX_MAX_WTM_LEVEL  HSD_MIN(256,(DATA_INPUT_USER))
 #define LIS3DHH_MAX_WTM_LEVEL     HSD_MIN(256,(DATA_INPUT_USER))
 */

/* Memory management macros */
#ifdef __ICCARM__
/* Use malloc debug functions (IAR only) */
#define IAR_MALLOC_DEBUG                          0
#else
#define GCC_MALLOC_DEBUG                          0
#endif /* __ICCARM__ */

#define MALLOC_CRITICAL_SECTION                     1

#if (IAR_MALLOC_DEBUG == 1)
#define HSD_malloc                                 malloc_debug
#define HSD_calloc                                 calloc_debug
#define HSD_free                                   free_debug
#elif (GCC_MALLOC_DEBUG == 1)
#define HSD_malloc                                 gcc_malloc_debug
#define HSD_calloc                                 gcc_calloc_debug
#define HSD_free                                   gcc_free_debug
#elif (MALLOC_CRITICAL_SECTION == 1)
#define HSD_malloc                                 malloc_critical
#define HSD_calloc                                 calloc_critical
#define HSD_free                                   free_critical
#else
#define HSD_malloc                                 malloc
#define HSD_calloc                                 calloc
#define HSD_free                                   free
#endif /* MALLOC_DEBUG */

#ifndef HSD_memset
#define HSD_memset                                  memset
#endif /* HSD_memset */

#ifndef HSD_memcpy
#define HSD_memcpy                                  memcpy
#endif /* HSD_memcpy */

#ifdef __cplusplus
}
#endif

#endif /* INC_HSDCORECONFIG_H_ */
