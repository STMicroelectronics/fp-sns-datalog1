/**
  ******************************************************************************
  * @file    AutoModeTask.c
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

#include "AutoModeTask.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"
#include "apperror.h"
#include "main.h"
#include "sdcard_manager.h"
#include <string.h>
#include <stdio.h>

#ifndef AM_TASK_CFG_STACK_DEPTH
#define AM_TASK_CFG_STACK_DEPTH                (configMINIMAL_STACK_SIZE*2)
#endif /* AM_TASK_CFG_STACK_DEPTH */

#ifndef AM_TASK_CFG_PRIORITY
#define AM_TASK_CFG_PRIORITY                   (tskIDLE_PRIORITY+1)
#endif /* AM_TASK_CFG_PRIORITY */

#ifndef AM_TASK_CFG_IN_QUEUE_ITEM_SIZE
#define AM_TASK_CFG_IN_QUEUE_ITEM_SIZE         sizeof(AMCommand)
#endif /* AM_TASK_CFG_IN_QUEUE_ITEM_SIZE */

#ifndef AM_TASK_CFG_IN_QUEUE_ITEM_COUNT
#define AM_TASK_CFG_IN_QUEUE_ITEM_COUNT        20
#endif /* AM_TASK_CFG_IN_QUEUE_ITEM_COUNT */

#define AM_TICK_TO_WAIT_IN_CMD_LOOP            portMAX_DELAY

#define AM_CMD_ID_NEW_CFG     	((uint16_t)0x0001) /* NEW_CFG command ID. For more information see 
                                                      AMTaskOnNewConfiguration() function. */
#define AM_CMD_ID_START_EP    	((uint16_t)0x0002) /* START_EP command ID. Start the execution phase specified in the
                                                      parameter of the command. */
#define AM_CMD_ID_END_OF_EP   	((uint16_t)0x0003) /* END_OF_EP command ID. Notify the task that an execution phase is
                                                      ended. */
#define AM_CMD_ID_ABORT_AM    	((uint16_t)0x0004) /* ABORT_AM command ID. Notify the task that the auto mode has been
                                                      aborted. */
#define AM_CMD_ID_RESTART_PLAN	((uint16_t)0x0005) /* RESTART_PLAN command ID. Notify the task restart the execution 
                                                      plan. */

#define AM_TASK_START_DELAY_MSG	"Auto mode enabled. Execution plan will start after %ld ms.\r\n\r\n"

#ifndef SYS_IS_CALLED_FROM_ISR
#define SYS_IS_CALLED_FROM_ISR() ((SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0 ? 1 : 0)
#endif /* SYS_IS_CALLED_FROM_ISR */

typedef union _UAMCmdParam
{
  uint32_t nParam; /* generic parameter. It depends on the command ID. */
} UAMCmdParam;

/**
  * Specifies a command that can be executed by the ::PMTask.
  */
typedef struct _AMCommand
{
  /**
    * Specify the command ID. Valid value are:
    * - NEW_CFG
    * - START_EP
    * - END_OF_EP
    */
  uint16_t nCmdID;

  /**
    * TRUE when the command must be blocking. It is managed by AMTaskPostCommand() and AMTaskSendComand() API.
    */
  boolean_t bSyncCmd;

  /**
    * Specifies the command parameter.
    */
  UAMCmdParam uParam;
} AMCommand;

/**
  * Specify the internal state of the task object.
  */
typedef enum _EAMState
{
  E_AM_STATE_NOT_STARTED, /* E_AM_STATE_NOT_STARTED */
  E_AM_STATE_STARTED, /* E_AM_STATE_STARTED */
  E_AM_STATE_ENDED /* E_AM_STATE_ENDED   */
} EAMState;

/**
  * Identify an execution phase.
  */
typedef enum _EAMExecutionPhase
{
  E_AM_PHASE_NONE, /* E_AM_PHASE_NONE       */
  E_AM_PHASE_LEARNING, /* E_AM_PHASE_LEARNING   */
  E_AM_PHASE_DETECTION,/*  E_AM_PHASE_DETECTION */
  E_AM_PHASE_DATALOG, /* E_AM_PHASE_DATALOG    */
  E_AM_PHASE_IDLE /* E_AM_PHASE_IDLE       */
} EAMExecutionPhase;

/**
  * PMTask class state. For more information see the PMTask.h file documentation.
  */
struct _AMTask
{
  /**
    * Specifies the native task handle.
    */
  TaskHandle_t m_xTask;

  /**
    * Specifies the task input queue used to send command to the task.
    */
  QueueHandle_t m_xInQueue;

  /**
    * Specifies a binary semaphore used to send a command in a synchronous way.
    * TODO: STF - do we need synchronous command for this task?
    */
  SemaphoreHandle_t m_xSyncCmdMutex;

  /**
    * Specifies the software timer used to implement the IDLE execution phase.
    */
  TimerHandle_t m_xIdleTimer;

  /**
    * Specify the actual state of the task object.
    */
  EAMState m_eState;

  /**
    * Specifies the current step in the execution plan.
    */
  uint8_t m_nExecutionPlanIdx;

  /**
    * Specifies how many time the execution plan has been done.
    */
  uint16_t m_nPlanIterationCount;

  /**
    * Specifies if the startup delay has been executed or npot.
    */
  boolean_t m_bStartupDelayDone;
};

/**
  * The only instance of PMTask object.
  */
static AMTask *s_pxTaskObj = NULL;

/* Private functions forward declaration */
/*****************************************/

/**
  * Task control function.
  *
  * @param pParams [IN] specifies a pointer to the task object. It is safe to cast as (AMTask*)pParams
  */
static void AMTaskRun(void *pParams);

/**
  * Get the execution phase in the plan at the specified index.
  *
  * @param nIndex [IN] index of a step in the execution plan.
  * @return the execution phase in the plan at the specified index, or E_AM_PHASE_NONE in case of error.
  */
static EAMExecutionPhase AMTaskGetExecutoinPhaseFromPlan(uint8_t nIndex);

/**
  * Start an execution phase. This method look in the execution plan for an execution phase at a given index.
  * It it exist and it is valid, then a command is queued in the task queue in order to start the execution phase
  * at the next step.
  *
  * @param _this [IN] specifies a pointer to the task object.
  * @param nIndex [IN] index of a step in the execution plan.
  * @return SYS_NO_ERROR_CODE if success, an error code otherwise.
  */
static sys_error_code_t AMTaskStartExecutionPhase(AMTask *_this, uint8_t nIndex);

/**
  * Start the learning execution phase according to the parameters specified in the auto mode model.
  * @param _this [IN] specifies a pointer to the task object.
  * @return SYS_NO_ERROR_CODE if success, an error code otherwise,
  */
/* static sys_error_code_t AMTaskStartLearning(AMTask *_this); */

/**
  * Start the detection execution phase according to the parameters specified in the auto mode model.
  * @param _this [IN] specifies a pointer to the task object.
  * @return SYS_NO_ERROR_CODE if success, an error code otherwise,
  */
/* static sys_error_code_t AMTaskStartDetect(AMTask *_this); */

/**
  * Start the datalog execution phase according to the parameters specified in the auto mode model.
  * @param _this [IN] specifies a pointer to the task object.
  * @return SYS_NO_ERROR_CODE if success, an error code otherwise,
  */
static sys_error_code_t AMTaskStartDatalog(AMTask *_this);

/**
  * Start the idle execution phase according to the parameters specified in the auto mode model.
  * @param _this [IN] specifies a pointer to the task object.
  * @return SYS_NO_ERROR_CODE if success, an error code otherwise,
  */
static sys_error_code_t AMTaskStartIdle(AMTask *_this);

/**
  * Called when an execution phase is stopped.
  *
  * @param eExecutionPhase [IN] specifies the execution phases that is ended.
  */
/* static void AMStopEPCallback(EPMMode eExecutionPhase); */

/**
  * Called when a datalog execution phase is stopped.
  */
static void AMStopDatalogEPCallback(void);

/**
  * Check the next step of the execution plan.
  *
  * @param _this [IN] specifies a pointer to the task object.
  * @return TRUE if the execution plan has been completed, FALSE if there is another step to be executed.
  */
static boolean_t AMTaskCheckEndOfPlan(AMTask *_this);

/**
  * Callback to notify the task the end of the IDLE execution phase.
  *
  * @param xTimer [IN] specifies a timer handle.
  */
static void AMTaskIdleTimerCallback(TimerHandle_t xTimer);

/* Public functions definition */
/*******************************/

AMTask *AMTaskAlloc(void)
{
  if (s_pxTaskObj == NULL)
  {
    s_pxTaskObj = pvPortMalloc(sizeof(AMTask));
    if (s_pxTaskObj)
    {
      memset(s_pxTaskObj, 0, sizeof(AMTask));
    }
  }

  return s_pxTaskObj;
}

sys_error_code_t AMTaskInit(AMTask *_this)
{
  assert_param(_this);
  sys_error_code_t xRes = SYS_NO_ERROR_CODE;

  /* create the task object. */
  xRes = xTaskCreate(AMTaskRun, "AMT", AM_TASK_CFG_STACK_DEPTH, s_pxTaskObj, AM_TASK_CFG_PRIORITY, 
                     &s_pxTaskObj->m_xTask);
  if (pdPASS == xRes)
  {
    /* create the input queue */
    _this->m_xInQueue = xQueueCreate(AM_TASK_CFG_IN_QUEUE_ITEM_COUNT, AM_TASK_CFG_IN_QUEUE_ITEM_SIZE);
    if (_this->m_xInQueue == NULL)
    {
      xRes = SYS_AM_TASK_INIT_ERROR_CODE;
    }
    else
    {
#ifdef DEBUG
      vQueueAddToRegistry(_this->m_xInQueue, "AM_Q");
#endif /* DEBUG */
      /* create the binary semaphore for the command execution: */
      _this->m_xSyncCmdMutex = xSemaphoreCreateBinary();
      if (_this->m_xSyncCmdMutex == NULL)
      {
        xRes = SYS_AM_TASK_INIT_ERROR_CODE;
      }
      else
      {
        _this->m_xIdleTimer = xTimerCreate("AMter", 1, pdFALSE, _this, AMTaskIdleTimerCallback);
        if (_this->m_xIdleTimer == NULL)
        {
          xRes = SYS_AM_TASK_INIT_ERROR_CODE;
        }
        else
        {
          /* complete the task initialization with the simple type members... if any. */
          _this->m_eState = E_AM_STATE_NOT_STARTED;
          _this->m_nExecutionPlanIdx = 0;
          _this->m_nPlanIterationCount = 0;
          _this->m_bStartupDelayDone = TRUE;
        }
      }
    }
  }

  return xRes;
}

sys_error_code_t AMTaskOnNewConfiguration(AMTask *_this)
{
  assert_param(_this);
  sys_error_code_t xRes = SYS_NO_ERROR_CODE;

  AMCommand xCmd =
  {
    .nCmdID = AM_CMD_ID_NEW_CFG,
    .bSyncCmd = FALSE
  };

  if (SYS_IS_CALLED_FROM_ISR())
  {
    if (pdTRUE != xQueueSendToBackFromISR(_this->m_xInQueue, &xCmd, NULL))
    {
      xRes = SYS_AM_TASK_IN_QUEUE_FULL_ERROR_CODE;
    }
  }
  else
  {
    if (pdTRUE != xQueueSendToBack(_this->m_xInQueue, &xCmd, pdMS_TO_TICKS(100)))
    {
      xRes = SYS_AM_TASK_IN_QUEUE_FULL_ERROR_CODE;
    }
  }

  return xRes;
}

sys_error_code_t AMTaskAbortAutoMode(AMTask *_this)
{
  assert_param(_this);
  sys_error_code_t xRes = SYS_NO_ERROR_CODE;

  /* check if the AutoMode task is active or the automode is enabled */
  if (AMGetIstance()->bIsValid)
  {
    if (_this->m_eState == E_AM_STATE_STARTED)
    {
      AMCommand xCmd =
      {
        .nCmdID = AM_CMD_ID_ABORT_AM,
        .bSyncCmd = FALSE
      };

      if (SYS_IS_CALLED_FROM_ISR())
      {
        if (pdTRUE != xQueueSendToFrontFromISR(_this->m_xInQueue, &xCmd, NULL))
        {
          xRes = SYS_AM_TASK_IN_QUEUE_FULL_ERROR_CODE;
        }
      }
      else
      {
        if (pdTRUE != xQueueSendToFront(_this->m_xInQueue, &xCmd, pdMS_TO_TICKS(100)))
        {
          xRes = SYS_AM_TASK_IN_QUEUE_FULL_ERROR_CODE;
        }
      }
    }
  }
  else
  {
    xRes = SYS_AM_INVALID_CFG_ERROR_CODE;
  }

  return xRes;
}

boolean_t AMTaskIsStarted(AMTask *_this)
{
  assert_param(_this);
  return _this->m_eState == E_AM_STATE_STARTED ? TRUE : FALSE;
}

boolean_t AMTaskIsNotStarted(AMTask *_this)
{
  assert_param(_this);
  return _this->m_eState == E_AM_STATE_NOT_STARTED ? TRUE : FALSE;
}

boolean_t AMTaskIsEnded(AMTask *_this)
{
  assert_param(_this);
  return _this->m_eState == E_AM_STATE_ENDED ? TRUE : FALSE;
}

sys_error_code_t AMTaskStartExecutioPlan(AMTask *_this)
{
  assert_param(_this);
  sys_error_code_t xRes = SYS_NO_ERROR_CODE;

  /* check if we have a valid auto mode configuration */
  if (AMGetIstance()->bIsValid)
  {
    AMCommand xCmd =
    {
      .bSyncCmd = FALSE,
      .nCmdID = AM_CMD_ID_RESTART_PLAN,
    };
    /*check if the first time that we load the configuration after reset or if an auto mode has been already executed*/
    if (SYS_IS_CALLED_FROM_ISR())
    {
      xQueueSendToBackFromISR(_this->m_xInQueue, &xCmd, NULL);
    }
    else
    {
      xQueueSendToBack(_this->m_xInQueue, &xCmd, pdMS_TO_TICKS(100));
    }
  }
  else
  {
    xRes = SYS_AM_INVALID_CFG_ERROR_CODE;
  }
  return xRes;
}

/* Private functions definition */
/********************************/

static void AMTaskRun(void *pParams)
{
  AMTask *pxTask = (AMTask *) pParams;
  sys_error_code_t xRes = SYS_NO_ERROR_CODE;
  AMCommand xCommand =
  {
    0
  };
  TickType_t xTimeoutInTick = AM_TICK_TO_WAIT_IN_CMD_LOOP;
  AutoModeCfg *pxAutoModeCfg = NULL;

  /* we need to wait the start of the console_app otherwise the printf message are not displayed.  */
  /* The console_app has a passive delay of 400msplus an active delay of 1000 ms to start the USB. */
  vTaskDelay(500);

  for (;;)
  {
    if (pdTRUE == xQueueReceive(pxTask->m_xInQueue, &xCommand, xTimeoutInTick))
    {
      switch (xCommand.nCmdID)
      {

        case AM_CMD_ID_NEW_CFG:
          if (pxTask->m_eState == E_AM_STATE_NOT_STARTED)
          {
            /* check the auto mode flag in the system configuration */
            pxAutoModeCfg = AMGetIstance();
            if (pxAutoModeCfg->bIsValid && pxAutoModeCfg->bEnable)
            {
              pxTask->m_eState = E_AM_STATE_STARTED;
              /* notify the user                                                             */
              /*              printf(AM_TASK_START_DELAY_MSG, pxAutoModeCfg->nStartDelayMS); */
              /* check if we need to delay the start of the execution plan                   */
              if (pxAutoModeCfg->nStartDelayMS)
              {
                pxTask->m_bStartupDelayDone = FALSE;
                xTimerChangePeriod(pxTask->m_xIdleTimer, pdMS_TO_TICKS(pxAutoModeCfg->nStartDelayMS), 
				                   pdMS_TO_TICKS(200));
              }
              else
              {
                /* we start immediately the first execution phase */
                xRes = AMTaskStartExecutionPhase(pxTask, pxTask->m_nExecutionPlanIdx);
              }
            }
          }
          break;

        case AM_CMD_ID_START_EP:
          switch ((EAMExecutionPhase) xCommand.uParam.nParam)
          {
            case E_AM_PHASE_DATALOG:
              AMTaskStartDatalog(pxTask);
              break;

            case E_AM_PHASE_IDLE:
              AMTaskStartIdle(pxTask);
              break;

            default:
              break;
          }
          break;

        case AM_CMD_ID_END_OF_EP:
          if (!pxTask->m_bStartupDelayDone)
          {
            pxTask->m_bStartupDelayDone = TRUE;
            /* this is not the first execution phase, but the end of the startup delay.*/
            /* So, we start the first execution phase                                  */
            xRes = AMTaskStartExecutionPhase(pxTask, pxTask->m_nExecutionPlanIdx);
          }
          else if (!AMTaskCheckEndOfPlan(pxTask))
          {
            /* start another phase of the plan. */
            xRes = AMTaskStartExecutionPhase(pxTask, pxTask->m_nExecutionPlanIdx);
          }
          else
          {
            pxTask->m_eState = E_AM_STATE_ENDED;
            /* remove the callback for the end of execution phase notification */
            SDM_SetStopEPCallback(NULL);
            /* reset the command queue to prevent unwanted execution */
            xQueueReset(pxTask->m_xInQueue);
            /* printf("Auto mode: end of the execution plan.\r\n"); */
          }
          break;

        case AM_CMD_ID_ABORT_AM:
          if (pxTask->m_eState == E_AM_STATE_STARTED)
          {
            pxTask->m_eState = E_AM_STATE_ENDED;
            /* remove the callback for the end of execution phase notification */
            SDM_SetStopEPCallback(NULL);
            /* stop the IDLE timer */
            xTimerStop(pxTask->m_xIdleTimer, pdMS_TO_TICKS(200));
            /* reset the command queue to prevent unwanted execution */
            xQueueReset(pxTask->m_xInQueue);
            /* notify the user of the abort of the auto mode */
            /*            printf("Auto mode: aborted.\r\n"); */
          }
          break;

        case AM_CMD_ID_RESTART_PLAN:
          if (pxTask->m_eState != E_AM_STATE_STARTED)
          {
            pxTask->m_eState = E_AM_STATE_STARTED;
            pxTask->m_nExecutionPlanIdx = 0;
            pxTask->m_nPlanIterationCount = 0;
            pxTask->m_bStartupDelayDone = TRUE; /* in case of restart Delay is ignored */
            xRes = AMTaskStartExecutionPhase(pxTask, pxTask->m_nExecutionPlanIdx);
          }
          break;

        default:
          break;
      }
    }

    if (xRes != SYS_NO_ERROR_CODE)
    {
      /* printf("\r\nAMtaskRun() error 0x%x\r\n\r\n", (unsigned int) xRes); */
    }
  }
}

static EAMExecutionPhase AMTaskGetExecutoinPhaseFromPlan(uint8_t nIndex)
{
  if (nIndex < AMC_CFG_MAX_EXECUTION_STEPS)
  {
    char *pcExecutionPhase = AMGetIstance()->pcExecutoinPlan[nIndex];
    if (!strncmp(pcExecutionPhase, AMC_LEARN_EXECUTION_PHASE_OBJ, AMC_CFG_MAX_EXECUTION_STEP_SIZE))
    {
      return E_AM_PHASE_LEARNING;
    }
    else if (!strncmp(pcExecutionPhase, AMC_DETECT_EXECUTION_PHASE_OBJ, AMC_CFG_MAX_EXECUTION_STEP_SIZE))
    {
      return E_AM_PHASE_DETECTION;
    }
    else if (!strncmp(pcExecutionPhase, AMC_DATALOG_EXECUTION_PHASE_OBJ, AMC_CFG_MAX_EXECUTION_STEP_SIZE))
    {
      return E_AM_PHASE_DATALOG;
    }
    else if (!strncmp(pcExecutionPhase, AMC_IDLE_EXECUTION_PHASE_OBJ, AMC_CFG_MAX_EXECUTION_STEP_SIZE))
    {
      return E_AM_PHASE_IDLE;
    }
  }

  return E_AM_PHASE_NONE;
}

static sys_error_code_t AMTaskStartExecutionPhase(AMTask *_this, uint8_t nIndex)
{
  assert_param(_this);
  sys_error_code_t xRes = SYS_NO_ERROR_CODE;

  if (nIndex < AMC_CFG_MAX_EXECUTION_STEPS)
  {
    EAMExecutionPhase ePhase = AMTaskGetExecutoinPhaseFromPlan(nIndex);
    if (ePhase != E_AM_PHASE_NONE)
    {
      AMCommand xCmd =
      {
        .nCmdID = AM_CMD_ID_START_EP,
        .uParam.nParam = (uint32_t) ePhase,
        .bSyncCmd = FALSE
      };

      if (xQueueSendToBack(_this->m_xInQueue, &xCmd, pdMS_TO_TICKS(100)) != pdTRUE)
      {
        xRes = SYS_AM_TASK_IN_QUEUE_FULL_ERROR_CODE;
      }
    }
  }

  return xRes;
}

static sys_error_code_t AMTaskStartDatalog(AMTask *_this)
{
  assert_param(_this);
  sys_error_code_t xRes = SYS_NO_ERROR_CODE;

  /* get the parameters of the execution phase. */
  AutoModeCfg *pxCfg = AMGetIstance();
  uint32_t nStopTimerPeriodMS = pxCfg->xDatalogCfg.nTimerMS;

  /* set the parameter in the PMTask. */
  SDM_SetExecutionContext(nStopTimerPeriodMS);

  /* register the callback to be notified of he stop */
  SDM_SetStopEPCallback(AMStopDatalogEPCallback);

  /* printf("Auto mode: start datalog.\r\n"); */

  /* start the execution phase           */
  /* Cannot wait since we are in an ISR  */
  if (osMessagePut(sdThreadQueue_id, SDM_START_STOP, 0) != osOK)
  {
    while (1)
    {
    }
  }

  return xRes;
}

static sys_error_code_t AMTaskStartIdle(AMTask *_this)
{
  assert_param(_this);
  sys_error_code_t xRes = SYS_NO_ERROR_CODE;

  /* get the parameters of the execution phase. */
  AutoModeCfg *pxCfg = AMGetIstance();
  uint32_t nIdlePeriodMS = pxCfg->xIdleCfg.nTimerMS;

  /* printf("Auto mode: wait for %ld ms.\r\n", nIdlePeriodMS); */

  if (nIdlePeriodMS)
  {
    /* start the software timer. */
    xRes = xTimerChangePeriod(_this->m_xIdleTimer, pdMS_TO_TICKS(nIdlePeriodMS), pdMS_TO_TICKS(100));
  }
  else
  {
    /* when the idle period is set to zero then I can notify the task the end of the execution phase. */
    AMStopDatalogEPCallback();
  }

  return xRes;
}

static boolean_t AMTaskCheckEndOfPlan(AMTask *_this)
{
  assert_param(_this);
  boolean_t bRes = TRUE;
  AutoModeCfg *pxAMCfg = AMGetIstance();

  /* check if we are at the end of the current iteration of the plan. */
  _this->m_nExecutionPlanIdx++;
  if ((_this->m_nExecutionPlanIdx >= AMC_CFG_MAX_EXECUTION_STEPS)
      || (AMTaskGetExecutoinPhaseFromPlan(_this->m_nExecutionPlanIdx) == E_AM_PHASE_NONE))
  {
    _this->m_nExecutionPlanIdx = 0;
    /* we finished an iteration of the plan.   */
    /* check if we are at the end of the plan. */
    if (pxAMCfg->nPhasesIteration)
    {
      /* there is still another iteration to do */
      if (++_this->m_nPlanIterationCount < pxAMCfg->nPhasesIteration)
      {
        /* we need to do another iteration. */
        bRes = FALSE;
      }
    }
    else
    {
      /* we need to execute the plan forever. */
      bRes = FALSE;
    }
  }
  else
  {
    bRes = FALSE;
  }

  return bRes;
}

static void AMTaskIdleTimerCallback(TimerHandle_t xTimer)
{
  AMTask *pxTask = (AMTask *) pvTimerGetTimerID(xTimer);
  AMCommand xCmd =
  {
    .nCmdID = AM_CMD_ID_END_OF_EP,
    .bSyncCmd = FALSE
  };

  xQueueSendToBack(pxTask->m_xInQueue, &xCmd, pdMS_TO_TICKS(100));
}

static void AMStopDatalogEPCallback(void)
{
  AMCommand xCmd =
  {
    .nCmdID = AM_CMD_ID_END_OF_EP,
    .bSyncCmd = FALSE
  };

  if (SYS_IS_CALLED_FROM_ISR())
  {
    xQueueSendToBackFromISR(s_pxTaskObj->m_xInQueue, &xCmd, NULL);
  }
  else
  {
    xQueueSendToBack(s_pxTaskObj->m_xInQueue, &xCmd, pdMS_TO_TICKS(100));
  }
}
