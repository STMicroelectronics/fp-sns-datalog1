/**
******************************************************************************
* @file    SensorTile.box.c
* @author  SRA - Central Labs
* @version V1.3.5
* @date    10-Feb-2022
* @brief   This file provides low level functionalities for SensorTile.box board
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
#include "SensorTile.box.h"
#include "SensorTile.box_audio.h"


/** @addtogroup BSP
* @{
*/ 

/** @addtogroup SENSORTILE_BOX
* @{
*/

  /** @addtogroup SENSORTILE_BOX_LOW_LEVEL
  * @brief This file provides a set of low level firmware functions 
  * @{
  */

/** @defgroup SENSORTILE_BOX_LOW_LEVEL_Private_TypesDefinitions SENSORTILE_BOX_LOW_LEVEL Private Typedef
* @{
*/

/**
* @}
*/

/** @defgroup SENSORTILE_BOX_LOW_LEVEL_Private_Defines SENSORTILE_BOX_LOW_LEVEL Private Defines
* @{
*/


/**
* @brief SensorTile.box BSP Driver version number V2.0.0
*/
#define __SensorTileBOX_BSP_VERSION_MAIN   (0x01) /*!< [31:24] main version */
#define __SensorTileBOX_BSP_VERSION_SUB1   (0x00) /*!< [23:16] sub1 version */
#define __SensorTileBOX_BSP_VERSION_SUB2   (0x00) /*!< [15:8]  sub2 version */
#define __SensorTileBOX_BSP_VERSION_RC     (0x00) /*!< [7:0]  release candidate */
#define __SensorTileBOX_BSP_VERSION         ((__SensorTileBOX_BSP_VERSION_MAIN << 24)\
|(__SensorTileBOX_BSP_VERSION_SUB1 << 16)\
  |(__SensorTileBOX_BSP_VERSION_SUB2 << 8 )\
    |(__SensorTileBOX_BSP_VERSION_RC))


/**
* @}
*/

/** @defgroup SENSORTILE_BOX_LOW_LEVEL_Private_Variables SENSORTILE_BOX_LOW_LEVEL Private Variables 
* @{
*/

GPIO_TypeDef* GPIO_PORT[LEDn] = {LED1_GPIO_PORT, LED2_GPIO_PORT,LED3_GPIO_PORT};
const uint32_t GPIO_PIN[LEDn] = {LED1_PIN, LED2_PIN, LED3_PIN};

GPIO_TypeDef  *BUTTON_PORT[BUTTONn] = {USER_BUTTON_GPIO_PORT};
const uint16_t BUTTON_PIN[BUTTONn] = {USER_BUTTON_PIN};
const uint16_t BUTTON_IRQn[BUTTONn] = {USER_BUTTON_EXTI_IRQn};

/**
* @}
*/


/** @defgroup SENSORTILE_BOX_LOW_LEVEL_Private_Functions SENSORTILE_BOX_LOW_LEVEL Private Functions
* @{
*/ 


/**
* @}
*/

/** @defgroup SENSORTILE_BOX_LOW_LEVEL_Exported_Variables SENSORTILE_BOX_LOW_LEVEL Exported Variables
* @{
*/

ADC_HandleTypeDef SensorTileADC;

/**
* @}
*/

/** @defgroup SENSORTILE_BOX_LOW_LEVEL_Private_Macros SENSORTILE_BOX_LOW_LEVEL Private Macros
* @{
*/

#define OVERALL_DEC_RATIO (ADC_CLOCK / AUDIO_SAMPLING_FREQUENCY)
#define ADC_OVS (OVERALL_DEC_RATIO / (DECIMATION_RATIO_DFSDM))

/**
* @}
*/

/** @defgroup SENSORTILE_BOX_LOW_LEVEL_Exported_Functions SENSORTILE_BOX_LOW_LEVEL Exported Functions
  * @{
  */

static ADC_InitUsedDef ADC_UsedFor = ADC1_NOT_USED;

/**
* @brief  This method initializes the ADC peripheral used for Analog Mic and Battery Voltage Conversion
* @param  ADC_InitFor who wants to Init the ADC: Audio or Battery Charger
* @retval BSP_ERROR_NONE in case of success
* @retval BSP_ERROR_PERIPH_FAILURE in case of failures
*/
int32_t BSP_ADC1_Initialization(ADC_InitUsedDef ADC_InitFor)
{
  /* If the ADC is not yet initialized */
  if(ADC_UsedFor == ADC1_NOT_USED){
   GPIO_InitTypeDef  GPIO_InitStruct; 

    /* For enabling the 2V7_EN necessary for ADC */
    __HAL_RCC_GPIOF_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, GPIO_PIN_SET);

    /**Common config */
    SensorTileADC.Instance = ADC1;
    SensorTileADC.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
    SensorTileADC.Init.Resolution = ADC_RESOLUTION_12B;
    SensorTileADC.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    SensorTileADC.Init.ScanConvMode = DISABLE;
    SensorTileADC.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    SensorTileADC.Init.LowPowerAutoWait = DISABLE;
    SensorTileADC.Init.ContinuousConvMode = ENABLE;
    SensorTileADC.Init.NbrOfConversion = 1; 
    SensorTileADC.Init.NbrOfDiscConversion      = 1;
    SensorTileADC.Init.DiscontinuousConvMode = DISABLE;
    SensorTileADC.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    SensorTileADC.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;

    if(DECIMATION_RATIO_DFSDM ==1)
    {
      SensorTileADC.Init.DMAContinuousRequests = ENABLE;
    }
    else
    {
      SensorTileADC.Init.DMAContinuousRequests = DISABLE;
    }
    
    SensorTileADC.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    
    if(ADC_OVS == 1)
    {
      /* Oversampling enabled */
      SensorTileADC.Init.OversamplingMode = DISABLE;
    }
    else
    {
      /* Oversampling enabled */
      SensorTileADC.Init.OversamplingMode = ENABLE;
      uint8_t valid_adc_ovs = 1;

      /* Oversampling ratio */
      switch(ADC_OVS)
      {
        case 2:
          SensorTileADC.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_2;
          break;
        case 4:
          SensorTileADC.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_4;
          break;
        case 8:
          SensorTileADC.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_8;
          break;
        case 16:
          SensorTileADC.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_16;
          break;
        case 32:
          SensorTileADC.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_32;
          break;
        case 64:
          SensorTileADC.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_64;
          break;
        case 128:
          SensorTileADC.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_128;
          break;
        case 256:
          SensorTileADC.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_256;
          break;
        default:
          /* ADC_OVS has not a valid value. Disable ADC and returns an error */
          valid_adc_ovs = 0;
          break;
      }
      if(valid_adc_ovs == 0)
      {
        /* ADC_OVS has not a valid value. Disable ADC and returns an error */
        return BSP_ERROR_PERIPH_FAILURE;
      }
    }

    SensorTileADC.Init.Oversampling.RightBitShift         = RSHFT_ADC;         /* Right shift of the oversampled summation */
    SensorTileADC.Init.Oversampling.TriggeredMode         = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;         /* Specifies whether or not a trigger is needed for each sample */
    SensorTileADC.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE; /* Specifies whether or not the oversampling buffer is maintained during injection sequence */    
    SensorTileADC.Init.DFSDMConfig = ADC_DFSDM_MODE_ENABLE;

    if(HAL_ADC_Init(&SensorTileADC) != HAL_OK)
    {
      return BSP_ERROR_PERIPH_FAILURE;
    }

    /* ### Start calibration ############################################ */
    if (HAL_ADCEx_Calibration_Start(&SensorTileADC, ADC_SINGLE_ENDED) != HAL_OK)
    {
      return BSP_ERROR_PERIPH_FAILURE;
    }
  }

  /* Set that we had Initiliazed the ADC for Audio or For Battery Charger */
  ADC_UsedFor |= ADC_InitFor;

  return BSP_ERROR_NONE;
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{ 
  /*##-1- Enable peripherals  ################################################*/
  /* ADC Periph clock enable */
  __HAL_RCC_ADC_CLK_ENABLE();
  /* ADC Periph interface clock configuration */
  __HAL_RCC_ADC_CONFIG(RCC_ADCCLKSOURCE_PLLSAI1);
}


void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{ 
  /* ADC Periph clock Disable */
  __HAL_RCC_ADC_CLK_DISABLE();
}

/**
* @brief  This method De initializes the ADC peripheral used for Analog Mic and Battery Voltage Conversion
* @param  ADC_InitFor who wants to Init the ADC: Audio or Battery Charger
* @retval BSP_ERROR_NONE in case of success
* @retval BSP_ERROR_PERIPH_FAILURE in case of failures
*/
int32_t BSP_ADC1_DeInitialization(ADC_InitUsedDef ADC_InitFor)
{
  /* Set that we had DeInitiliazed the ADC for Audio or for Battery Charger */
  ADC_UsedFor &= ~ADC_InitFor;

  /* If the ADC it's not more necessary for Audio&Battery Charger */
  if(ADC_UsedFor == ADC1_NOT_USED){

    if(HAL_ADC_DeInit(&SensorTileADC) != HAL_OK) {
      return BSP_ERROR_PERIPH_FAILURE;
    }

    /* For disabling the 2V7_EN necessary for ADC */
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, GPIO_PIN_RESET);
    __HAL_RCC_GPIOF_CLK_DISABLE();
  }

  return BSP_ERROR_NONE;
}
  
/**
* @brief  This method returns the SensorTile.box EVAL BSP Driver revision
* @param  None
* @retval version: 0xXYZR (8bits for each decimal, R for RC)
*/
uint32_t BSP_GetVersion(void)
{
  return __SensorTileBOX_BSP_VERSION;
}


/**
* @brief  Configures LEDs.
* @param  Led: LED to be configured. 
*          This parameter can be one of the following values:
*            @arg  LED1
*            @arg  LED2
*            @arg  LED3
* @retval None
*/
void BSP_LED_Init(Led_TypeDef Led)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /* Enable the GPIO_LED clock */
  LEDx_GPIO_CLK_ENABLE(Led);
  
  /* Configure the GPIO_LED pin */
  GPIO_InitStruct.Pin = GPIO_PIN[Led];
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  
  HAL_GPIO_Init(GPIO_PORT[Led], &GPIO_InitStruct);

  /* If Red Led... switch off it */
  if(Led == LED3) {
    BSP_LED_Off(Led);
  }
}


/**
* @brief  DeInit LEDs.
* @param  Led: LED to be configured. 
*          This parameter can be one of the following values:
*            @arg  LED1
*            @arg  LED2
*            @arg  LED3
* @note Led DeInit does not disable the GPIO clock nor disable the Mfx 
* @retval None
*/
void BSP_LED_DeInit(Led_TypeDef Led)
{
  HAL_GPIO_DeInit(GPIO_PORT[Led], GPIO_PIN[Led]);
}

/**
* @brief  Turns selected LED On.
* @param  Led: LED to be set on 
*          This parameter can be one of the following values:
*            @arg  LED1
*            @arg  LED2
*            @arg  LED3
* @retval None
*/
void BSP_LED_On(Led_TypeDef Led)
{
  if(Led == LED3) {
    HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET);
  } else {
    HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_SET);
  }
}

/**
* @brief  Turns selected LED Off. 
* @param  Led: LED to be set off
*          This parameter can be one of the following values:
*            @arg  LED1
*            @arg  LED2
*            @arg  LED3
* @retval None
*/
void BSP_LED_Off(Led_TypeDef Led)
{
  if(Led == LED3) {
    HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET);
  }
}

/**
* @brief  Toggles the selected LED.
* @param  Led: LED to be toggled
*          This parameter can be one of the following values:
*            @arg  LED1
*            @arg  LED2
*            @arg  LED3
* @retval None
*/
void BSP_LED_Toggle(Led_TypeDef Led)
{
  HAL_GPIO_TogglePin(GPIO_PORT[Led], GPIO_PIN[Led]);
}

/**
  * @brief  Configure Button GPIO and EXTI Line.
  * @param  Button: Specifies the Button to be configured.
  *   This parameter should be: BUTTON_USER
  * @param  ButtonMode: Specifies Button mode.
  *   This parameter can be one of following parameters:
  *     @arg BUTTON_MODE_GPIO: Button will be used as simple IO
  *     @arg BUTTON_MODE_EXTI: Button will be connected to EXTI line with interrupt
  *                            generation capability
  * @retval None
  */
void BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Enable VddIO2 */
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  
  /* Enable the BUTTON Clock */
  BUTTONx_GPIO_CLK_ENABLE(Button);

  if (ButtonMode == BUTTON_MODE_GPIO)
  {
    /* Configure Button pin as input */
    GPIO_InitStruct.Pin    = BUTTON_PIN[Button];
    GPIO_InitStruct.Mode   = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull   = GPIO_NOPULL;
    GPIO_InitStruct.Speed  = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(BUTTON_PORT[Button], &GPIO_InitStruct);
  }
  else if (ButtonMode == BUTTON_MODE_EXTI)
  {
    /* Configure Button pin as input with External interrupt */
    GPIO_InitStruct.Pin    = BUTTON_PIN[Button];
    GPIO_InitStruct.Mode   = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull   = GPIO_NOPULL;
    GPIO_InitStruct.Speed  = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BUTTON_PORT[Button], &GPIO_InitStruct);

    /* Enable and set Button EXTI Interrupt to the lowest priority */
    HAL_NVIC_SetPriority((IRQn_Type)(BUTTON_IRQn[Button]), 0x0F, 0);
    HAL_NVIC_EnableIRQ((IRQn_Type)(BUTTON_IRQn[Button]));
  }
}

/**
  * @brief  DeInitialize Push Button.
  * @param  Button: Button to be configured
  *   This parameter should be: BUTTON_USER
  * @note BSP_PB_DeInit() does not disable the GPIO clock
  * @retval None
  */
void BSP_PB_DeInit(Button_TypeDef Button)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.Pin = BUTTON_PIN[Button];
  HAL_NVIC_DisableIRQ((IRQn_Type)(BUTTON_IRQn[Button]));
  HAL_GPIO_DeInit(BUTTON_PORT[Button], GPIO_InitStruct.Pin);
}

/**
  * @brief  Return the selected Button state.
  * @param  Button: Specifies the Button to be checked.
  *   This parameter should be: BUTTON_USER
  * @retval Button state.
  */
uint32_t BSP_PB_GetState(Button_TypeDef Button)
{
  return HAL_GPIO_ReadPin(BUTTON_PORT[Button], BUTTON_PIN[Button]);
}

/**
  * @brief  Initialize Power Button GPIO and EXTI Line.
  * @param  None
  * @retval None
  */
void     BSP_PowerButton_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Enable VddIO2 */
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  
  /* Enable the GPIO Clock */
  POWER_BUTTON_GPIO_CLK_ENABLE();

  /* Configure Button pin as input with External interrupt */
  GPIO_InitStruct.Pin    = POWER_BUTTON_PIN;
  GPIO_InitStruct.Mode   = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull   = GPIO_NOPULL;
  GPIO_InitStruct.Speed  = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(POWER_BUTTON_GPIO_PORT, &GPIO_InitStruct);

  /* Enable and set Button EXTI Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(POWER_BUTTON_EXTI_IRQn, 0x0F, 0);
  HAL_NVIC_EnableIRQ(POWER_BUTTON_EXTI_IRQn);
}
/**
  * @brief  Deinitialize Power Button GPIO and EXTI Line.
  * @param  None
  * @retval None
  */
void     BSP_PowerButton_DeInit(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.Pin = POWER_BUTTON_PIN;
  HAL_NVIC_DisableIRQ(POWER_BUTTON_EXTI_IRQn);
  HAL_GPIO_DeInit(POWER_BUTTON_GPIO_PORT, GPIO_InitStruct.Pin);
}


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
