/**
  ******************************************************************************
  * @file    AutoModeModel.h
  * @author  SRA
  *
  *
  * @brief   Data model for the auto mode.
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
#ifndef INC_AUTOMODEMODEL_H_
#define INC_AUTOMODEMODEL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

#define AMC_CFG_FILE_NAME                  "execution_config.json"
#define AMC_CFG_FILE_VERSION_LENGTH        5
#define AMC_PHASE_ITERATION_INFINITE       0
#define AMC_CFG_MAX_EXECUTION_STEPS        10
#define AMC_CFG_MAX_EXECUTION_STEP_SIZE    8
#define AMC_INFO_OBJ                       "info"
#define AMC_VERSION_STR_VALUE              "version"
#define AMC_GLOBAL_ENABLE_BOOL_VALUE       "auto_mode"
#define AMC_PHASES_ITERATIO_N_VALUE        "phases_iteration"
#define AMC_STAR_DELAY_MS_N_VALUE          "start_delay_ms"
#define AMC_EXECUTION_PLAN_ARRAY           "execution_plan"
#define AMC_LEARN_EXECUTION_PHASE_OBJ      "learn"
#define AMC_DETECT_EXECUTION_PHASE_OBJ     "detect"
#define AMC_DATALOG_EXECUTION_PHASE_OBJ    "datalog"
#define AMC_IDLE_EXECUTION_PHASE_OBJ       "idle"
#define AMC_TIMER_MS_N_VALUE               "timer_ms"
#define AMC_SIGNALS_N_VALUE                "signals"
#define AMC_THRESHOLD_N_VALUE              "threshold"
#define AMC_SNSITIVITY_F_VALUE             "sensitivity"

/**
  * Specifies the configuration parameters for the Learning execution phase.
  */
typedef struct AMLearnCfg_t
{
  /**
    * Specifies the number of signals to use for the learning phases. The execution phase ends
    * after analyzing the specified number of signals.
    */
  uint32_t nSignals;

  /**
    * Specifies a timer in ms for the learning phases. The execution phase ends
    * when the timer expire the.
    */
  uint32_t nTimerMS;
} AMLearnCfg;

/**
  * Specifies the configuration parameters for the Detection execution phase.
  */
typedef struct AMDetectCfg_t
{
  /**
    * Specifies the number of signals to use for the learning phases. The execution phase ends
    * after analyzing the specified number of signals.
    */
  uint32_t nSignals;

  /**
    * Specifies a timer in ms for the learning phases. The execution phase ends
    * when the timer expire the.
    */
  uint32_t nTimerMS;

  /**
    * Specify a threshold value to trigger a warning notification about the
    * current analyzed signal. If the detection algorithm reports a similarity
    * value lower that the threshold, then the system rises a notification.
    */
  uint8_t nThreshold;

  /**
    * Specifies the sensitivity for the detection algorithm.
    */
  float fSensitivity;
} AMDetectCfg;

/**
  * Specifies the configuration parameters for the Datalog execution phase.
  */
typedef struct AMDatalogCfg_t
{
  /**
    * Specifies a timer in ms for the learning phases. The execution phase ends
    * when the timer expire the.
    */
  uint32_t nTimerMS;
} AMDatalogCfg;

/**
  * Specifies the configuration parameters for the Idle execution phase.
  */
typedef struct AMIdleCfg_t
{
  /**
    * Specifies a timer in ms for the learning phases. The execution phase ends
    * when the timer expire the.
    */
  uint32_t nTimerMS;
} AMIdleCfg;

/**
  * Specifies the root of the auto mode configuration data model. It is linked to the
  * JSON file format used to serialize it.
  */
typedef struct AutoModeCfg_t
{
  /**
    * This member is not in the model specified in the JSON file. It is used to check if
    * the model has been initialized or not, for example, with the data loaded from a file.
    */
  uint8_t bIsValid;

  /**
    * Specifies the version of the auto mode configuration file.
    * The format is MM.mm
    */
  char pcVersion[AMC_CFG_FILE_VERSION_LENGTH];

  /**
    * Global flag that specifies if the auto mode plan has to executed at power on.
    */
  uint8_t bEnable;

  /**
    * Specifies the number of time the the auto mode plan has to be executed.
    * Zero means forever.
    */
  uint16_t nPhasesIteration;

  /**
    * Specifies the delay in millisecond to start the auto mode plan at startup.
    */
  uint32_t nStartDelayMS;

  /**
    * The execution plan is a sequence of execution phase. Each execution phase is
    * represented in the plan as a string. Valid value are:
    * - learning
    * - detection
    * - datalog
    * - idle
    *
    * The parameters for each execution phases are specified in a dedicated
    * configuration object.
    */
  char pcExecutoinPlan[AMC_CFG_MAX_EXECUTION_STEPS][AMC_CFG_MAX_EXECUTION_STEP_SIZE];

  /**
    * Specifies the Learning phase actual parameters.
    */
  AMLearnCfg xLeanrCfg;

  /**
    * Specifies the Detection phase actual parameters.
    */
  AMDetectCfg xDetecCfg;

  /**
    * Specifies the Datalog phase actual parameters.
    */
  AMDatalogCfg xDatalogCfg;

  /**
    * Specifies the Idle phase actual parameters.
    */
  AMIdleCfg xIdleCfg;
} AutoModeCfg;

#ifdef __cplusplus
}
#endif

#endif /* INC_AUTOMODEMODEL_H_ */
