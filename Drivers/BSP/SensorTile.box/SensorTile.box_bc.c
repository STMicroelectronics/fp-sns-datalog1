/**
******************************************************************************
* @file    SensorTile.box_bc.c
* @author  SRA - Central Labs
* @version V1.3.5
* @date    10-Feb-2022
* @brief   This file provides code for the configuration of the STBC02 battery charger
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
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "SensorTile.box_bc.h"
#include <string.h>

/** @addtogroup BSP
 * @{
 */

/** @addtogroup SENSORTILEBOX 
 * @{
 */

/** @addtogroup SENSORTILEBOX_BATTERY_CHARGER 
 * @{
 */

/** @addtogroup SENSORTILEBOX_BATTERY_CHARGER_Private_Constants
 * @{
 */

 /**
 * @brief STBC02 status name and related toggling frequency (in 0.1Hz) of the nCHG pin
 * @note  The sequence must respect the order of stbc02_ChgState_TypeDef
 */
const stbc02_ChgStateNameAndFreq_TypeDef stbc02_ChgStateNameAndFreq[10] = {
  {"Not Valid Input", 00},
  {"Valid Input", 00},
  {"Vbat Low", 00},
  {"End Of Charge", 41},
  {"Charging Phase", 62},
  {"Overcharge Fault", 82},
  {"Charging Timeout", 102},
  {"Battery Voltage Below Vpre", 128},
  {"Charging Thermal Limitation", 142},
  {"Battery Temperature Fault", 162},
};

__IO uint16_t uhADCxConvertedValue = 0;


/**
 * @}
 */

/** @addtogroup SENSORTILEBOX_BATTERY_CHARGER_Private_Variables
 * @{
 */

static stbc02_SwCmd_TypeDef stbc02_SwCmdSel;                    //!< The selected STBC02 SW Cmd
static stbc02_SwState_TypeDef stbc02_SwState = idle;            //!< The state during STBC02 SW Cmd sending
static stbc02_ChgState_TypeDef stbc02_ChgState = NotValidInput; //!< The state of the STBC02
static uint32_t stbc02_ChgPinFreq;                              //!< The toggling frequency of the STBC02 nCHG pin [0.1Hz]
uint32_t stbc02_ChgPinToggledTime = 0;                          //!< SysTick value when STBC02 nCHG pin is toggling
TIM_HandleTypeDef hstbc02_UsedTim;                              //!< Handler of the used timer

/**
 * @}
 */

/* Private Function_Prototypes -----------------------------------------------*/
void BC_InitCommon(void);
void BC_IO_Init(void);
void BC_IO_SW_SEL_Init(void);
void BC_IO_CHG_Init(void);
void BC_IO_CHG_DeInit(void);
void BC_ChgPinFreqGet(void);
void BC_ChgPinFreq2ChgSts(void);
void BC_TIMx_Init(void);

static uint8_t Channel_ADC1_Init(void);
void STBC02_USED_TIM_IRQHandler(void);

void BC_TIM_Base_MspInit(TIM_HandleTypeDef *htim);

static void HAL_ADC_BC_MspDeInit(void);

/** @addtogroup SENSORTILEBOX_BATTERY_CHARGER_Public_Functions
 * @{
 */


/**
  * @brief Initialize the STBC02
  * @retval None
  */
void BSP_BC_Init(void)
{
  BC_InitCommon();

  /* Init the related GPIOs */
  BC_IO_Init();
}


/**
  * @brief Initialize the STBC02 Interrupt pin
  * @retval None
  */
void BSP_BC_ChrgPin_Init(void)
{
  BC_IO_CHG_Init();
}

/**
  * @brief Deinitialize the STBC02 Interrupt pin
  * @retval None
  */
void BSP_BC_ChrgPin_DeInit(void)
{
  BC_IO_CHG_DeInit();
}


/**
 * @brief Send a single wire command to the STBC02
 * @param stbc02_SwCmd The command to be sent
 * @retval BSP status
 */
int32_t BSP_BC_CmdSend(stbc02_SwCmd_TypeDef stbc02_SwCmd)
{
  uint32_t tk = STBC02_GetTick();
  
  stbc02_SwCmdSel = stbc02_SwCmd;
  stbc02_SwState = start;
  
  /* Start the time base */
  HAL_TIM_Base_Start_IT(&hstbc02_UsedTim);
  
  
  while(stbc02_SwState != idle)
  {
    if ( (STBC02_GetTick() - tk) > 10 )
      return BSP_ERROR_COMPONENT_FAILURE;
  }
  
  /* Stop the time base */
  HAL_TIM_Base_Stop_IT(&hstbc02_UsedTim);
  
  return BSP_ERROR_NONE;
}


/**
 * @brief Call the function for measuring the toggling frequency of the charging pin of the STBC02
 * @note Must be called when the charging pin of the STBC02 has toggled
 * @retval None
 */
void BSP_BC_ChgPinHasToggled(void)
{
  BC_ChgPinFreqGet();
  stbc02_ChgPinToggledTime = STBC02_GetTick();
}

/**
 * @brief Check for STBC02 nCHG toggling
 * @note Must be called on SysTick IRQ
 * @retval 0/1 if not toggling/toggling
 */
uint8_t BSP_BC_IsChgPinToggling(void)
{
  GPIO_PinState stbc02_ChgPinState;
  
  if ( (STBC02_GetTick() - stbc02_ChgPinToggledTime) > 500 ) {
    stbc02_ChgPinState = HAL_GPIO_ReadPin(STBC02_CHG_GPIO_PORT, STBC02_CHG_PIN);
    if (stbc02_ChgPinState == GPIO_PIN_SET) {
      stbc02_ChgState = NotValidInput;
    } else {
      stbc02_ChgState = ValidInput;
    }
    return 0;
  }
  return 1;
}

/**
 * @brief  This method initializes the peripherals used to get the current voltage of battery
 * @param  None
 * @retval BSP status
*/
int32_t BSP_BC_BatMS_Init(void)
{

  /* Enable ADC1 if it's necessary */
  BSP_ADC1_Initialization(ADC1_FOR_BC);

  /* Configure the Injection Channel for Voltage ADC convertion */
  Channel_ADC1_Init();

  return BSP_ERROR_NONE;
}


/**
 * @brief  This method reset the peripherals used to get the current voltage of battery
 * @param  None
 * @retval BSP status
*/
int32_t BSP_BC_BatMS_DeInit(void)
{
  int32_t retValue;

  retValue = BSP_ADC1_DeInitialization(ADC1_FOR_BC);

  if(retValue!=BSP_ERROR_NONE) {
    return retValue;
  }
  
  HAL_ADC_BC_MspDeInit();
  
  return BSP_ERROR_NONE;
  
}


/**
 * @brief  Get the status of the STBC02
 * @param  pstbc02_ChgState To be filled with the new value
 * @retval None
 */
void BSP_BC_GetState(stbc02_State_TypeDef *BC_State)
{
  
  /* Check for STBC02 nCHG toggling */
  if (BSP_BC_IsChgPinToggling()) {
    /* Get the status of the STBC02 */
    BC_ChgPinFreq2ChgSts();
  }
  
  BC_State->Id = stbc02_ChgState;
  strncpy((char *) BC_State->Name, (char *)stbc02_ChgStateNameAndFreq[stbc02_ChgState].name, 32);
  
}


/**
 * @brief  This method gets the current voltage of battery and one estimation of Battery % Level
 * @param  uint32_t *Volt battery Voltage Value
 * @param  uint32_t *BatteryLevel Battery % Level
 * @retval BSP status
 */
int32_t BSP_BC_GetVoltageAndLevel(uint32_t *Volt,uint32_t *BatteryLevel)
{
  uint32_t Voltage;
  static int32_t VoltageWindowInit=0;
  static uint32_t WindowVoltage[WINDOW_VOLTAGE_DIM];
  static int32_t WindowPostion =0;

  /*##-3- Start the conversion process #######################################*/
  if (HAL_ADCEx_InjectedStart(&SensorTileADC) != HAL_OK) {
    /* Start Conversation Error */
    return BSP_ERROR_COMPONENT_FAILURE;
  }
  
  /*##-4- Wait for the end of conversion #####################################*/
  if (HAL_ADCEx_InjectedPollForConversion(&SensorTileADC, 10) != HAL_OK){
    /* End Of Conversion flag not set on time */
    return BSP_ERROR_CLOCK_FAILURE;
  }

  /* Check if the continuous conversion of Injected channel is finished */
  if ((HAL_ADC_GetState(&SensorTileADC) & HAL_ADC_STATE_INJ_EOC) == HAL_ADC_STATE_INJ_EOC) {
    /*##-5- Get the converted value of Injected channel  ########################*/
    uhADCxConvertedValue = HAL_ADCEx_InjectedGetValue(&SensorTileADC,ADC_INJECTED_RANK_1);
  }

  Voltage = (2700 * (uint32_t)uhADCxConvertedValue) / (4095);  // [0-2.7V]
  Voltage = ((215+100)*Voltage)/100;   // [0-4.2V]

  *Volt= Voltage;

  /* We Filter the Voltage for understanding the Battery % Level */

  /* Insert the new Value */
  WindowVoltage[WindowPostion]= Voltage;
  WindowPostion++;
  /* Control if we have reached the end */
  if(WindowPostion==WINDOW_VOLTAGE_DIM) {
    if(VoltageWindowInit==0) {
      VoltageWindowInit =1;
    }
    WindowPostion     =0;
  }

  /* Make the mean of latest voltage values */
  if(VoltageWindowInit){
    int32_t Counter;
    Voltage =0;
    for(Counter=0;Counter<WINDOW_VOLTAGE_DIM;Counter++) {
      Voltage+=WindowVoltage[Counter];
    }
    Voltage>>=4;
  }

  /* Limits check */
  if(Voltage > MAX_VOLTAGE) {
    Voltage= MAX_VOLTAGE;
  } else  if(Voltage < MIN_VOLTAGE) {
    Voltage= MIN_VOLTAGE;
  }

  *BatteryLevel= (((Voltage - MIN_VOLTAGE) * 100)/(MAX_VOLTAGE - MIN_VOLTAGE));

  return BSP_ERROR_NONE;
}
/**
 * @}
 */


/** @addtogroup SENSORTILEBOX_BATTERY_CHARGER_Private_Functions
 * @{
 */

/**
 * @brief Initialize the STBC02
 * @retval None
 */
void BC_InitCommon(void)
{
  /* Init the stbc02_ChgPinFreq value */
  stbc02_ChgPinFreq = 0;
  
  /* Init the time base */
  BC_TIMx_Init();
}

 
/**
 * @brief  Initializes the GPIO used for the Li-Ion Battery Charger
 * @retval None
 */
void BC_IO_Init(void)
{
  BC_IO_SW_SEL_Init();
}


/**
 * @brief  Initializes the SW_SEL GPIO used for the Li-Ion Battery Charger
 * @retval None
 */
void BC_IO_SW_SEL_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  STBC02_SW_SEL_GPIO_CLK_ENABLE();
  
  GPIO_InitStruct.Pin = STBC02_SW_SEL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

  HAL_GPIO_WritePin(STBC02_SW_SEL_GPIO_PORT, STBC02_SW_SEL_PIN, GPIO_PIN_RESET);

  HAL_GPIO_Init(STBC02_SW_SEL_GPIO_PORT, &GPIO_InitStruct);
}


/**
 * @brief  Initializes the nCHG GPIO used for the Li-Ion Battery Charger
 * @retval None
 */
void BC_IO_CHG_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  STBC02_CHG_GPIO_CLK_ENABLE();
  
  GPIO_InitStruct.Pin = STBC02_CHG_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;

  HAL_GPIO_Init(STBC02_CHG_GPIO_PORT, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(STBC02_CHG_EXTI_IRQn, STBC02_CHG_EXTI_IRQ_PP, 0);
  HAL_NVIC_EnableIRQ(STBC02_CHG_EXTI_IRQn);
}


/**
 * @brief  De-Initializes the nCHG GPIO used for the Li-Ion Battery Charger
 * @retval None
 */
void BC_IO_CHG_DeInit(void)
{
  HAL_GPIO_DeInit(STBC02_CHG_GPIO_PORT, STBC02_CHG_PIN);
}


/**
 * @brief Single wire command manager for the STBC02
 * @note This function have to be called every 5 us
 * @retval None
 */
void BC_CmdMng(void)
{
  static uint16_t TIMx_Pulse = 0;                           //! Actual timer pulse number
  static uint16_t TIMx_PulseTCS = 0;                        //! Timer pulse number to change state
  static stbc02_SwState_TypeDef stbc02_sw_state_prv = idle; //! Previous state
  static uint8_t CmdPulse = 0;                              //! Cmd pulse number

  TIMx_Pulse++;
  switch (stbc02_SwState)
  {
  case idle:
    break;
  case start:
    HAL_GPIO_TogglePin(STBC02_SW_SEL_GPIO_PORT, STBC02_SW_SEL_PIN);
    TIMx_PulseTCS = TIMx_Pulse + (350/5);
    stbc02_sw_state_prv = stbc02_SwState;
    stbc02_SwState = wait;
    break;
  case pulse_l:
    HAL_GPIO_TogglePin(STBC02_SW_SEL_GPIO_PORT, STBC02_SW_SEL_PIN);
    TIMx_PulseTCS = TIMx_Pulse + (100/5);
    stbc02_sw_state_prv = stbc02_SwState;
    stbc02_SwState = wait;
    break;
  case pulse_h:
    HAL_GPIO_TogglePin(STBC02_SW_SEL_GPIO_PORT, STBC02_SW_SEL_PIN);
    TIMx_PulseTCS = TIMx_Pulse + (100/5);
    stbc02_sw_state_prv = stbc02_SwState;
    stbc02_SwState = wait;
    break;
  case stop_l:
    HAL_GPIO_TogglePin(STBC02_SW_SEL_GPIO_PORT, STBC02_SW_SEL_PIN);
    TIMx_PulseTCS = TIMx_Pulse + (100/5);
    stbc02_sw_state_prv = stbc02_SwState;
    stbc02_SwState = wait;
    break;
  case stop_h:
    HAL_GPIO_TogglePin(STBC02_SW_SEL_GPIO_PORT, STBC02_SW_SEL_PIN);
    TIMx_PulseTCS = TIMx_Pulse + (500/5);
    stbc02_sw_state_prv = stbc02_SwState;
    stbc02_SwState = wait;
    break;
  case wait:
    if ( TIMx_Pulse > TIMx_PulseTCS )
    {
      if ( stbc02_sw_state_prv == stop_h )
      {
        CmdPulse = 0;
        stbc02_SwState = idle;
        HAL_GPIO_WritePin(STBC02_SW_SEL_GPIO_PORT, STBC02_SW_SEL_PIN, GPIO_PIN_RESET);
      }
      else
      {
        if ( stbc02_sw_state_prv == pulse_h )
        {
          CmdPulse++;
          if ( CmdPulse < stbc02_SwCmdSel )
          {
            stbc02_sw_state_prv = start;
          }
          else
            __NOP();
        }
        stbc02_SwState = ++stbc02_sw_state_prv;
        __NOP();
      }
    }
    break;
  }
}


/**
 * @brief Measure the toggling frequency of the charging pin of the STBC02
 * @note This function works with 1 ms as time base
 * @retval None
 */
void BC_ChgPinFreqGet(void)
{
  static uint32_t tk_prev = 0;
  uint32_t tk_now,tk_delta;

  tk_now = STBC02_GetTick();
  tk_delta = tk_now - tk_prev;
  tk_prev = tk_now;
  
  stbc02_ChgPinFreq = 10000/tk_delta;
}


/**
 * @brief Get the status of the STBC02 checking the toggling frequency of the charging pin of the STBC02
 * @retval None
 */
void BC_ChgPinFreq2ChgSts(void)
{
  stbc02_ChgState_TypeDef ChgState = NotValidInput;
  
  if (stbc02_ChgPinFreq > 0) {
    for (ChgState =  EndOfCharge; ChgState < BatteryTemperatureFault; ChgState++) {
      if ( (stbc02_ChgPinFreq <= ((stbc02_ChgStateNameAndFreq[ChgState].freq + stbc02_ChgStateNameAndFreq[ChgState+1].freq)>>1)) ) {
        stbc02_ChgState = ChgState;
        break;
      }
    }
    if(ChgState == NotValidInput) {
      if ( (stbc02_ChgPinFreq > ((stbc02_ChgStateNameAndFreq[ChargingThermalLimitation].freq + stbc02_ChgStateNameAndFreq[BatteryTemperatureFault].freq)>>1)) ) {
        stbc02_ChgState = BatteryTemperatureFault;
      }
    }
  }
}


/**
 * @brief  Initializes the used timer
 * @retval None
 */
void BC_TIMx_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  uint32_t freq = STBC02_USED_TIM_CLKFreq;
  uint32_t prescaler = 0;
  uint32_t period = 0;
  
  do {
    prescaler++;
    period = (uint32_t)(freq/(prescaler) * STBC02_USED_TIM_PERIOD);
    if (prescaler > 65535) {
      while(1);
    }
  } while (period > 65535);

  hstbc02_UsedTim.Instance = STBC02_USED_TIM;
  hstbc02_UsedTim.Init.Prescaler = (prescaler-1);
  hstbc02_UsedTim.Init.CounterMode = TIM_COUNTERMODE_UP;
  hstbc02_UsedTim.Init.Period = period;
#if USE_TIM_AUTORELOAD_PRELOAD
  hstbc02_UsedTim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
#endif //USE_TIM_AUTORELOAD_PRELOAD
  
  BC_TIM_Base_MspInit(&hstbc02_UsedTim);
  
  if (HAL_TIM_Base_Init(&hstbc02_UsedTim) != HAL_OK) {
    while(1);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&hstbc02_UsedTim, &sMasterConfig) != HAL_OK) {
    while(1);
  }
}

/**
 * @}
 */


/** @addtogroup SENSORTILEBOX_BATTERY_CHARGER_MSP_Functions
 * @{
 */

/**
 * @brief  Initializes the TIM Base MSP.
 * @param  htim TIM handle
 * @retval None
 */
void BC_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{

  if(htim->Instance==STBC02_USED_TIM) {
    /* STBC02_USED_TIM clock enable */
    STBC02_USED_TIM_CLK_ENABLE();

    /* STBC02_USED_TIM interrupt Init */
    HAL_NVIC_SetPriority(STBC02_USED_TIM_IRQn, STBC02_USED_TIM_IRQ_PP, STBC02_USED_TIM_IRQ_SP);
    HAL_NVIC_EnableIRQ(STBC02_USED_TIM_IRQn);
  }
}

/**
 * @brief  DeInitialize TIM Base MSP.
 * @param  htim TIM handle
 * @retval None
 */
void BC_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim)
{

  if(htim->Instance==STBC02_USED_TIM) {
    /* Peripheral clock disable */
    STBC02_USED_TIM_CLK_DISABLE();

    /* STBC02_USED_TIM interrupt Deinit */
    HAL_NVIC_DisableIRQ(STBC02_USED_TIM_IRQn);
  }
} 


/**
 * @brief  Initialize the ADC MSP.
 * @param BC_AdcHandle ADC handle
 * @retval None
 */
void HAL_ADC_BC_MspInit(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  /* STBC02_USED_ADC GPIO Configuration */
  STBC02_BATMS_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin = STBC02_BATMS_PIN;
  GPIO_InitStruct.Mode = STBC02_BATMS_GPIO_MODE;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(STBC02_BATMS_GPIO_PORT, &GPIO_InitStruct);
}

/**
 * @brief  DeInitialize the ADC MSP.
 * @param None
 * @note   All ADC instances use the same core clock at RCC level, disabling
 *         the core clock reset all ADC instances).
 * @retval None
 */
static void HAL_ADC_BC_MspDeInit(void)
{
  /* STBC02_USED_ADC GPIO Configuration */
  HAL_GPIO_DeInit(STBC02_BATMS_GPIO_PORT, STBC02_BATMS_PIN);
}

/**
 * @brief  Initializes the used ADC channel
 * @retval 0 in case of success
 * @retval 1 in case of failure
 */
static uint8_t Channel_ADC1_Init(void)
{ 
  ADC_InjectionConfTypeDef sConfigInjected = {0};
  uint32_t PausedADC=0;

  /* Check if the ADC is running and Stop it for Injection Channel configuration */
  if (HAL_ADC_GetState(&SensorTileADC) & HAL_ADC_STATE_REG_BUSY) {
    HAL_ADC_Stop(&SensorTileADC);
    PausedADC=1;
  }

  /* Configure ADC injected channel */
  sConfigInjected.InjectedChannel = STBC02_USED_ADC_CHANNEL;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_NONE;
  sConfigInjected.InjecOversamplingMode = DISABLE;

  HAL_ADC_BC_MspInit();

  if (HAL_ADCEx_InjectedConfigChannel(&SensorTileADC, &sConfigInjected) != HAL_OK) {
    /* Channel Configuration Error */
    return 1;
  }

  /* Restart ADC if it was stopped before Injection Channel configuration */
  if (PausedADC) {
    HAL_ADC_Start(&SensorTileADC);
  }

  return 0;
}

/**
 * @}
 */

/** @addtogroup SENSORTILEBOX_BATTERY_CHARGER_Interrupt_Callback_Functions
 * @{
 */

#if USE_BC_TIM_IRQ_CALLBACK
/**
 * @brief  Period elapsed callback in non-blocking mode
 * @param  htim TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == STBC02_USED_TIM) {
    BC_CmdMng();
  }
}
#endif //USE_BC_TIM_IRQ_CALLBACK

#if USE_BC_IRQ_CALLBACK
/**
 * @brief  EXTI line detection callback.
 * @param  GPIO_Pin: Specifies the port pin connected to corresponding EXTI line.
 * @retval None
 */
void STBC02_CHG_PIN_Callback(void)
{
  BSP_BC_ChgPinHasToggled();
}
#endif //USE_BC_IRQ_CALLBACK

/**
 * @}
 */

/** @addtogroup SENSORTILEBOX_BATTERY_CHARGER_Interrupt_Service_Routines
 * @{
 */
/**
* @brief This function handles STBC02_USED_TIM global interrupt.
*/
void STBC02_USED_TIM_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&hstbc02_UsedTim);
}

#if USE_BC_CHG_IRQ_HANDLER
/**
  * @brief This function handles EXTI for STBC02_CHG_PIN interrupts.
  */
void STBC02_CHG_EXTI_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(STBC02_CHG_PIN);
}
#endif /* USE_BC_CHG_IRQ_HANDLER */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */
