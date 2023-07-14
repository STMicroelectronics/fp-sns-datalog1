/**
  ******************************************************************************
  * @file    apperror.h
  * @author  SRA
  *
  *
  *
  * @brief Application specific error code
  *
  * Application defines its own error code in this file starting form the
  * constant APP_BASE_ERROR_CODE.
  * It is recommended to group the error code in the following groups:
  * - Low Level API error code
  * - Service Level error code
  * - Task Level error code
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
#ifndef APPERROR_H_
#define APPERROR_H_

#ifdef __cplusplus
extern "C" {
#endif

/* General SYS error code */
/**************************/
#define SYS_NO_ERROR_CODE                                     0x0
#define SYS_BASE_ERROR_CODE                                   0X1
#define SYS_UNDEFINED_ERROR_CODE                              SYS_BASE_ERROR_CODE + 1
#define SYS_OUT_OF_MEMORY_ERROR_CODE                          SYS_BASE_ERROR_CODE + 2
#define SYS_INVALID_PARAMETER_ERROR_CODE                      SYS_BASE_ERROR_CODE + 3
#define SYS_INVALID_FUNC_CALL_ERROR_CODE                      SYS_BASE_ERROR_CODE + 4
#define SYS_TIMEOUT_ERROR_CODE                                SYS_BASE_ERROR_CODE + 5

/* Low Level API error code */
/****************************/

/* Service Level error code */
/****************************/

/* CircularBuffer error code */
#define SYS_CB_BASE_ERROR_CODE                                SYS_TIMEOUT_ERROR_CODE + 1
#define SYS_CB_INVALID_ITEM_ERROR_CODE                        SYS_CB_BASE_ERROR_CODE + 1
#define SYS_CB_FULL_ERROR_CODE                                SYS_CB_BASE_ERROR_CODE + 2
#define SYS_CB_NO_READY_ITEM_ERROR_CODE                       SYS_CB_BASE_ERROR_CODE + 3

/* AutoMode error code */
#define SYS_AM_BASE_ERROR_CODE                                SYS_CB_NO_READY_ITEM_ERROR_CODE + 1
#define SYS_AM_MALFORMED_JSON_ERROR_CODE                      SYS_AM_BASE_ERROR_CODE + 1
#define SYS_AM_INVALID_CFG_ERROR_CODE                         SYS_AM_BASE_ERROR_CODE + 2

/* Task Level error code */
/*************************/

/* Predictive Maintenance (PM)Task error code */
#define SYS_PM_TASK_BASE_ERROR_CODE                           SYS_AM_INVALID_CFG_ERROR_CODE + 1
#define SYS_PM_TASK_INIT_ERROR_CODE                           SYS_PM_TASK_BASE_ERROR_CODE + 1
#define SYS_PM_TASK_INVALID_CMD_ERROR_CODE                    SYS_PM_TASK_BASE_ERROR_CODE + 2
#define SYS_PM_TASK_CMD_ERROR_CODE                            SYS_PM_TASK_BASE_ERROR_CODE + 3
#define SYS_PM_TASK_IN_QUEUE_FULL_ERROR_CODE                  SYS_PM_TASK_BASE_ERROR_CODE + 4
#define SYS_PM_TASK_FLASH_ERROR_CODE                          SYS_PM_TASK_BASE_ERROR_CODE + 5

/* Auto Mode (AM)Task error code */
#define SYS_AM_TASK_BASE_ERROR_CODE                           SYS_PM_TASK_FLASH_ERROR_CODE + 1
#define SYS_AM_TASK_INIT_ERROR_CODE                           SYS_AM_TASK_BASE_ERROR_CODE + 1
#define SYS_AM_TASK_INVALID_CMD_ERROR_CODE                    SYS_AM_TASK_BASE_ERROR_CODE + 2
#define SYS_AM_TASK_CMD_ERROR_CODE                            SYS_AM_TASK_BASE_ERROR_CODE + 3
#define SYS_AM_TASK_IN_QUEUE_FULL_ERROR_CODE                  SYS_AM_TASK_BASE_ERROR_CODE + 4


#ifdef __cplusplus
}
#endif

#endif /* APPERROR_H_ */
