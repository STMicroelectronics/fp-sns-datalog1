/**
  ******************************************************************************
  * @file    cpu_utils.h
  * @author  SRA
  *
  *
  * @brief   Header for cpu_utils module
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
#ifndef _CPU_UTILS_H__
#define _CPU_UTILS_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define CALCULATION_PERIOD    1000

/* Exported functions ------------------------------------------------------- */
uint16_t osGetCPUUsage(void);
void storeIdleHook(void);
void StartIdleMonitor(void);
void EndIdleMonitor(void);

#ifdef __cplusplus
}
#endif

#endif /* _CPU_UTILS_H__ */

