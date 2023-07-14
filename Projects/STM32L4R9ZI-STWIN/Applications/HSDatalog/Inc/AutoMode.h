/**
  ******************************************************************************
  * @file    AutoMode.h
  * @author  SRA
  *
  *
  * @brief   Auto mode public API declaration.
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
#ifndef INC_AUTOMODE_H_
#define INC_AUTOMODE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "AutoModeModel.h"
#include "systypes.h"

/**
  * This function must be called at least once before using
  * the ::AutoModeCfg object. It reset the model.
  */
void AMReset(void);

/**
  * Get a pointer to the only one instance of the ::AutoModeCfg object.
  *
  * @return a pointer to the only one instance of the ::AutoModeCfg object.
  */
AutoModeCfg *AMGetIstance(void);

/**
  * Initialize the system instance with the value from pxSource.
  * NOT IMPLEMENTED.
  *
  * @param pxSource [IN] specifies an auto mode configuration object.
  * @return SYS_NO_ERROR_CODE if valid, a specific error code otherwise.
  */
sys_error_code_t AMCopyConfiguration(const AutoModeCfg *pxSource);

/**
  * Not implemented.
  *
  * @param pcSerializedCfg
  * @param pxConfig
  * @return
  */
sys_error_code_t AMParseCfgFromString(const char *pcSerializedCfg, AutoModeCfg *pxConfig);

/**
  *  Initialize the system instance with the data coming from the serialized string.
  *
  * @param pcSerializedCfg [IN] specifies the serialized string. It must be formatted in JSON format,
  *                        according to the auto mode specification.
  * @return SYS_NO_ERROR_CODE if success, an error code otherwise.
  */
sys_error_code_t AMLoadCfgFromString(const char *pcSerializedCfg);


#ifdef __cplusplus
}
#endif

#endif /* INC_AUTOMODE_H_ */
