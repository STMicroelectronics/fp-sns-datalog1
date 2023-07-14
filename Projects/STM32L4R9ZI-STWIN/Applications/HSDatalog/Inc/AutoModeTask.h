/**
  ******************************************************************************
  * @file    AutoModeTask.h
  * @author  SRA
  *
  *
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
#ifndef INC_AUTOMODETASK_H_
#define INC_AUTOMODETASK_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"
#include "systypes.h"
#include "FreeRTOS.h"
#include "AutoMode.h"


/**
  * Create  type name for _AMTask. For more information see the AutoModeTask.h file documentation.
  */
typedef struct _AMTask AMTask;

/* Public API declaration */
/**************************/

/**
  * Allocate an object of ::AMTask type. The object must be initialized with the AMTaskInit() method before
  * calling any API functions. This implementation use the singleton design pattern, so it is possible to allocate
  * only one instance for the application.
  *
  * @return a pointer to the new allocated object, or NULL in case of an error.
  */
AMTask *AMTaskAlloc(void);

/**
  * Initialize the object internals status. This method must be called after the object allocation and
  * before any other call to the public API.
  *
  * @param _this [IN] specifies a pointer to the task object.
  * @return SYS_NO_ERROR_CODE if success, an error code otherwise.
  */
sys_error_code_t AMTaskInit(AMTask *_this);

/**
  * This function must be called to inform the task object that the automode configuration
  * has been changed.
  *
  * @param _this [IN] specifies a pointer to the task object.
  * @return SYS_NON_ERROR_CODE if success, an error code otherwise.
  */
sys_error_code_t AMTaskOnNewConfiguration(AMTask *_this);

/**
  * Stop the execution of the plan specified in the auto mode configuration.
  *
  * @param _this [IN] specifies a pointer to the task object.
  * @return SYS_NON_ERROR_CODE if success, an error code otherwise.
  */
sys_error_code_t AMTaskAbortAutoMode(AMTask *_this);

/**
  * Check if the auto mode is started.
  *
  * @param _this [IN] specifies a pointer to the task object.
  * @return TRUE if the auto mode is started, FALSE otherwise.
  */
boolean_t AMTaskIsStarted(AMTask *_this);

/**
  * Check if the auto mode is not started.
  *
  * @param _this [IN] specifies a pointer to the task object.
  * @return TRUE if the auto mode not is started, FALSE otherwise.
  */
boolean_t AMTaskIsNotStarted(AMTask *_this);

/**
  * Check if the auto mode is ended.
  *
  * @param _this [IN] specifies a pointer to the task object.
  * @return TRUE if the auto mode is started and, either it has completed the execution plan or it has been aborted, FALSE otherwise.
  */
boolean_t AMTaskIsEnded(AMTask *_this);

/**
  * Start the execution plan. If the plan is already started the method does nothing.
  *
  * @param _this [IN] specifies a pointer to the task object.
  * @return SYS_NO_ERROR_CODE if success, an error code otherwise.
  */
sys_error_code_t AMTaskStartExecutioPlan(AMTask *_this);


#ifdef __cplusplus
}
#endif

#endif /* INC_AUTOMODETASK_H_ */
