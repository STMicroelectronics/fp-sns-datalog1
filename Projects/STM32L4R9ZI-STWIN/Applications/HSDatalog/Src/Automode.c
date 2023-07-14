/**
  ******************************************************************************
  * @file    Automode.c
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

#include "AutoMode.h"
#include "stm32l4xx_hal.h"
#include "apperror.h"
#include "parson.h"
#include "string.h"


#define AMC_CFG_FILE_VERSION "1.0"


/**
  * Specifies the only instance of the auto mode model.
  */
static AutoModeCfg s_xTheModel;

/* Callback function declaration */
/*********************************/

__weak void AMOnNewConfigurationReady(const AutoModeCfg *pxNewAMCfg);

/* Public API definition */
/*************************/

void AMReset(void)
{
  AutoModeCfg *pxAMCfg = AMGetIstance();
  /* set the default value */
  pxAMCfg->bIsValid = 0;
  pxAMCfg->bEnable = 0;
  pxAMCfg->nPhasesIteration = 0;
  pxAMCfg->nStartDelayMS = 0;
  strcpy(pxAMCfg->pcVersion, AMC_CFG_FILE_VERSION);
  strcpy(pxAMCfg->pcExecutoinPlan[0], AMC_IDLE_EXECUTION_PHASE_OBJ);
  pxAMCfg->xDatalogCfg.nTimerMS = 0;
  pxAMCfg->xDetecCfg.fSensitivity = 1.0;
  pxAMCfg->xDetecCfg.nSignals = 0;
  pxAMCfg->xDetecCfg.nThreshold = 90;
  pxAMCfg->xDetecCfg.nTimerMS = 0;
  pxAMCfg->xIdleCfg.nTimerMS = 1000;
  pxAMCfg->xLeanrCfg.nSignals = 0;
  pxAMCfg->xLeanrCfg.nTimerMS  = 0;

  /* validate the configuration. */
  pxAMCfg->bIsValid = 1;

  /* Set the default value */
}

AutoModeCfg *AMGetIstance(void)
{
  return &s_xTheModel;
}

sys_error_code_t AMCopyConfiguration(const AutoModeCfg *pxSource)
{
  assert_param(pxSource);
  sys_error_code_t xRes = SYS_NO_ERROR_CODE;

  return xRes;
}

sys_error_code_t AMParseCfgFromString(const char *pcSerializedCfg, AutoModeCfg *pxConfig)
{
  assert_param(pcSerializedCfg);
  assert_param(pxConfig);
  sys_error_code_t xRes = SYS_NO_ERROR_CODE;

  return xRes;
}

sys_error_code_t AMLoadCfgFromString(const char *pcSerializedCfg)
{
  assert_param(pcSerializedCfg);
  sys_error_code_t xRes = SYS_NO_ERROR_CODE;
  AutoModeCfg *pxModel = AMGetIstance();

  /* invalidate the actual configuration. */
  AMGetIstance()->bIsValid = 0;

  JSON_Value *pxJValue = json_parse_string(pcSerializedCfg);
  JSON_Object *pxJObj = json_value_get_object(pxJValue);

  /* check for the "info.version" string value */
  if (json_object_dothas_value_of_type(pxJObj, "info.version", JSONString))
  {
    const char *pcTmp = json_object_dotget_string(pxJObj, "info.version");
    strncpy(pxModel->pcVersion, pcTmp, AMC_CFG_FILE_VERSION_LENGTH);
  }
  else
  {
    xRes = SYS_AM_MALFORMED_JSON_ERROR_CODE;
  }

  /* all the other parameters are optional. If not present then the system uses the default value */
  /* from the AMReset() function. */

  /* check for the "info.auto_mode" boolean value */
  if ((xRes == SYS_NO_ERROR_CODE) && json_object_dothas_value_of_type(pxJObj, "info.auto_mode", JSONBoolean))
  {
    pxModel->bEnable = json_object_dotget_boolean(pxJObj, "info.auto_mode");
  }

  /* check for the "info.phases_iteration" number value */
  if ((xRes == SYS_NO_ERROR_CODE) && json_object_dothas_value_of_type(pxJObj, "info.phases_iteration", JSONNumber))
  {
    pxModel->nPhasesIteration = (uint16_t)json_object_dotget_number(pxJObj, "info.phases_iteration");
  }

  /* check for the "info.start_delay_ms" number value */
  if ((xRes == SYS_NO_ERROR_CODE) && json_object_dothas_value_of_type(pxJObj, "info.start_delay_ms", JSONNumber))
  {
    pxModel->nStartDelayMS = (uint32_t)json_object_dotget_number(pxJObj, "info.start_delay_ms");
  }

  /* check for the "info.learning.timer_ms" number value */
  if ((xRes == SYS_NO_ERROR_CODE) && json_object_dothas_value_of_type(pxJObj, "info.learn.timer_ms", JSONNumber))
  {
    pxModel->xLeanrCfg.nTimerMS = (uint32_t)json_object_dotget_number(pxJObj, "info.learn.timer_ms");
  }

  /* check for the "info.learning.signals" number value */
  if ((xRes == SYS_NO_ERROR_CODE) && json_object_dothas_value_of_type(pxJObj, "info.learn.signals", JSONNumber))
  {
    pxModel->xLeanrCfg.nSignals = (uint32_t)json_object_dotget_number(pxJObj, "info.learn.signals");
  }

  /* check for the "info.detection.timer_ms" number value */
  if ((xRes == SYS_NO_ERROR_CODE) && json_object_dothas_value_of_type(pxJObj, "info.detect.timer_ms", JSONNumber))
  {
    pxModel->xDetecCfg.nTimerMS = (uint32_t)json_object_dotget_number(pxJObj, "info.detect.timer_ms");
  }

  /* check for the "info.detection.signals" number value */
  if ((xRes == SYS_NO_ERROR_CODE) && json_object_dothas_value_of_type(pxJObj, "info.detect.signals", JSONNumber))
  {
    pxModel->xDetecCfg.nSignals = (uint32_t)json_object_dotget_number(pxJObj, "info.detect.signals");
  }

  /* check for the "info.detection.threshold" number value */
  if ((xRes == SYS_NO_ERROR_CODE) && json_object_dothas_value_of_type(pxJObj, "info.detect.threshold", JSONNumber))
  {
    pxModel->xDetecCfg.nThreshold = (uint8_t)json_object_dotget_number(pxJObj, "info.detect.threshold");
  }

  /* check for the "info.detection.sensitivity" number value */
  if ((xRes == SYS_NO_ERROR_CODE) && json_object_dothas_value_of_type(pxJObj, "info.detect.sensitivity", JSONNumber))
  {
    pxModel->xDetecCfg.fSensitivity = (float)json_object_dotget_number(pxJObj, "info.detect.sensitivity");
  }

  /* check for the "info.datalog.timer_ms" number value */
  if ((xRes == SYS_NO_ERROR_CODE) && json_object_dothas_value_of_type(pxJObj, "info.datalog.timer_ms", JSONNumber))
  {
    pxModel->xDatalogCfg.nTimerMS = (uint32_t)json_object_dotget_number(pxJObj, "info.datalog.timer_ms");
  }

  /* check for the "info.idle.timer_ms" number value */
  if ((xRes == SYS_NO_ERROR_CODE) && json_object_dothas_value_of_type(pxJObj, "info.idle.timer_ms", JSONNumber))
  {
    pxModel->xIdleCfg.nTimerMS = (uint32_t)json_object_dotget_number(pxJObj, "info.idle.timer_ms");
  }

  /* check for the "info.execution_plan" */
  if ((xRes == SYS_NO_ERROR_CODE) && json_object_dothas_value_of_type(pxJObj, "info.execution_plan", JSONArray))
  {
    JSON_Array *pxJArray = json_object_dotget_array(pxJObj, "info.execution_plan");
    const char *pcExecutoinPhase = NULL;
    size_t nExecutionSteps = json_array_get_count(pxJArray);
    if (nExecutionSteps > AMC_CFG_MAX_EXECUTION_STEPS)
    {
      /*ignore the extra steps. */
      /* another option is to signal the JSON file as malformed. */
      nExecutionSteps = AMC_CFG_MAX_EXECUTION_STEPS;
    }
    for (uint8_t i = 0; i < nExecutionSteps; ++i)
    {
      pcExecutoinPhase = json_array_get_string(pxJArray, i);
      if (pcExecutoinPhase != NULL)
      {
        strncpy(pxModel->pcExecutoinPlan[i], pcExecutoinPhase, AMC_CFG_MAX_EXECUTION_STEP_SIZE);
      }
      else
      {
        pxModel->pcExecutoinPlan[i][0] = '\0';
      }
    }
    /* set to NULL the other steps in the model's plan. */
    for (uint8_t j = nExecutionSteps; j < AMC_CFG_MAX_EXECUTION_STEPS; ++j)
    {
      pxModel->pcExecutoinPlan[j][0] = '\0';
    }
  }

  if (xRes == SYS_NO_ERROR_CODE)
  {
    /* now we have a valid auto mode configuration, */
    /* mark the system instance has valid ... */
    AMGetIstance()->bIsValid = 1;
    /* and use the callback to notify the application. */
    AMOnNewConfigurationReady(pxModel);
  }

  return xRes;
}

__weak void AMOnNewConfigurationReady(const AutoModeCfg *pxNewAMCfg)
{

}
