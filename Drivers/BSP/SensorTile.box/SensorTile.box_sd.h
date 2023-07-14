/**
******************************************************************************
* @file    SensorTile.box_sd.h (based on stm32l4r9i_eval_sd.h)
* @author  SRA - Central Labs
* @version V1.3.5
* @date    10-Feb-2022
* @brief   This file contains the common defines and functions prototypes for 
*          the SensorTile.box_sd.c driver.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SENSORTILEBOX_SD_H
#define __SENSORTILEBOX_SD_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/** @defgroup SENSORTILE_SD_Exported_Constants SENSORTILE_SD Exported Constants
  * @{
  */ 

/** 
  * @brief  SD transfer state definition  
  */     
#define SD_TRANSFER_OK                ((uint8_t)0x00)
#define SD_TRANSFER_BUSY              ((uint8_t)0x01)
#define SD_TRANSFER_ERROR             ((uint8_t)0x02)

/**
  * @brief  SD detection on its memory slot
  */
#define SD_PRESENT               ((uint8_t)0x01)
#define SD_NOT_PRESENT           ((uint8_t)0x00)

/**
  * @brief  GPIO for SD Detect PIn
  */  
#define SD_DETECT_GPIO_Pin  GPIO_PIN_12
#define SD_DETECT_GPIO_Port GPIOB

/**
  * @brief  GPIO for SD Select PIN
  */  
#define SD_SEL_Pin  GPIO_PIN_5
#define SD_SEL_Port GPIOE

/**
  * @brief  SD status structure definition  
  */     
#define MSD_OK                        ((uint8_t)0x00)
#define MSD_ERROR                     ((uint8_t)0x01)
#define MSD_ERROR_SD_NOT_PRESENT      ((uint8_t)0x02)
#define SD_DATATIMEOUT                  ((uint32_t)100000000)

/** 
  * @brief SD Card information 
  */
#define BSP_SD_CardInfo HAL_SD_CardInfoTypeDef

/**
  * @}
  */
  

/** @defgroup SENSORTILE_SD_Exported_Functions SENSORTILE_SD Exported Functions
  * @{
  */   
/* Exported functions --------------------------------------------------------*/   
uint8_t BSP_SD_Init(void);
uint8_t BSP_SD_IsDetected(void);
uint8_t BSP_SD_ReadBlocks(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks, uint32_t Timeout);
uint8_t BSP_SD_WriteBlocks(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks, uint32_t Timeout);
uint8_t BSP_SD_Erase(uint32_t StartAddr, uint32_t EndAddr);
uint8_t BSP_SD_GetCardState(void);
void    BSP_SD_GetCardInfo(BSP_SD_CardInfo *CardInfo);

uint8_t BSP_SD_ITConfig(void);
void    BSP_SD_DetectIT(void);
void BSP_SD_Detect_Init(void);
void    BSP_SD_DetectCallback(void);
uint8_t BSP_SD_ReadBlocks_DMA(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks);
uint8_t BSP_SD_WriteBlocks_DMA(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks);


/* These functions can be modified in case the current settings (eg. interrupt priority)
   need to be changed for specific application needs */
void    BSP_SD_AbortCallback(void);
void    BSP_SD_WriteCpltCallback(void);
void    BSP_SD_ReadCpltCallback(void);

/** @defgroup SENSORTILE_SD_Exported_Variables SENSORTILE_SD Exported Variables
  * @{
  */   
/* Exported Variables --------------------------------------------------------*/
extern SD_HandleTypeDef hsd1;
extern __IO uint8_t SD_Status;

/**
  * @}
  */ 

#ifdef __cplusplus
}
#endif

#endif /* __SENSORTILEBOX_SD_H */

/**
 * @}
 */
