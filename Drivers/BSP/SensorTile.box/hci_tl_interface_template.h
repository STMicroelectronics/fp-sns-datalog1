/**
******************************************************************************
* @file    hci_tl_interface_template.h
* @author  SRA - Central Labs
* @version V1.3.5
* @date    10-Feb-2022
* @brief   This file contains all the functions prototypes for the STM32
*          BlueNRG HCI Transport Layer interface
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
#ifndef __HCI_TL_INTERFACE_H
#define __HCI_TL_INTERFACE_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "SensorTile.box.h"

/* Exported Defines ----------------------------------------------------------*/

#define HCI_TL_SPI_EXTI_PORT  GPIOD
#define HCI_TL_SPI_EXTI_PIN   GPIO_PIN_4
#define HCI_TL_SPI_EXTI_IRQn  EXTI4_IRQn

#define HCI_TL_SPI_IRQ_PORT   GPIOD
#define HCI_TL_SPI_IRQ_PIN    GPIO_PIN_4

#define HCI_TL_SPI_CS_PORT    GPIOD
#define HCI_TL_SPI_CS_PIN     GPIO_PIN_0

#define HCI_TL_RST_PORT       GPIOA
#define HCI_TL_RST_PIN        GPIO_PIN_8
   
/* SPI re-mapping functions */
#define BSP_SPI_Init BSP_SPI2_Init
#define BSP_SPI_SendRecv BSP_SPI2_SendRecv

/* Exported Functions --------------------------------------------------------*/
int32_t HCI_TL_SPI_Init    (void* pConf);
int32_t HCI_TL_SPI_DeInit  (void);
int32_t HCI_TL_SPI_Receive (uint8_t* buffer, uint16_t size);
int32_t HCI_TL_SPI_Send    (uint8_t* buffer, uint16_t size);
int32_t HCI_TL_SPI_Reset   (void);

/**
 * @brief  Register hci_tl_interface IO bus services
 *
 * @param  None
 * @retval None
 */
void hci_tl_lowlevel_init(void);

/**
 * @brief HCI Transport Layer Low Level Interrupt Service Routine
 *
 * @param  None
 * @retval None
 */
void hci_tl_lowlevel_isr(void);

#ifdef __cplusplus
}
#endif
#endif /* __HCI_TL_INTERFACE_H */
