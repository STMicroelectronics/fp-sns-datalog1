/**
  ******************************************************************************
  * @file    systypes.h
  * @author  SRA
  *
  *
  * @brief   Common type declaration
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

#ifndef SYSTYPES_H_
#define SYSTYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
  * Boolean type definition.
  */
typedef enum
{
  FALSE = 0, /* FALSE */
  TRUE = !FALSE /* TRUE */
} boolean_t;

/**
  * Specifies a generic error type. It could be a system wide error type definition.
  */
typedef uint32_t sys_error_code_t;

#ifdef __cplusplus
}
#endif

#endif /* SYSTYPES_H_ */
