/**
  ******************************************************************************
  * @file    ble_comm_manager.h
  * @author  SRA
  *
  *
  * @brief   Header for ble_comm_manager.c module.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BLE_COMM_MANAGER_H
#define __BLE_COMM_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"
#include "cmsis_os.h"


#define BLE_CM_SPI_x                                SPI2
#define BLE_CM_SPIx_CLK_ENABLE()                    __HAL_RCC_SPI2_CLK_ENABLE()

#define BLE_CM_SPI_CLK_PIN                          GPIO_PIN_1
#define BLE_CM_SPI_CLK_GPIO_PORT                    GPIOD
#define BLE_CM_SPI_CLK_PIN_CLK_ENABLE()             __HAL_RCC_GPIOD_CLK_ENABLE()
#define BLE_CM_SPI_CLK_AF                           GPIO_AF5_SPI2

#define BLE_CM_SPI_MISO_PIN                         GPIO_PIN_3
#define BLE_CM_SPI_MISO_GPIO_PORT                   GPIOD
#define BLE_CM_SPI_MISO_PIN_CLK_ENABLE()            __HAL_RCC_GPIOD_CLK_ENABLE()
#define BLE_CM_SPI_MISO_AF                          GPIO_AF5_SPI2

#define BLE_CM_SPI_MOSI_PIN                         GPIO_PIN_3
#define BLE_CM_SPI_MOSI_GPIO_PORT                   GPIOC
#define BLE_CM_SPI_MOSI_PIN_CLK_ENABLE()            __HAL_RCC_GPIOC_CLK_ENABLE()
#define BLE_CM_SPI_MOSI_AF                          GPIO_AF5_SPI2

#define BLE_CM_SPI_EXTI_PORT                        GPIOG
#define BLE_CM_SPI_EXTI_PIN                         GPIO_PIN_1
#define BLE_CM_SPI_EXTI_IRQn                        EXTI1_IRQn

#define BLE_CM_SPI_CS_PORT                          GPIOG
#define BLE_CM_SPI_CS_PIN                           GPIO_PIN_5

#define BLE_CM_RST_PORT                             GPIOA
#define BLE_CM_RST_PIN                              GPIO_PIN_8


#define BLE_COMMAND_MASK           (0xFF000000)
#define BLE_SUB_COMMAND_MASK       (0x00FFFFFF)

#define BLE_COMMAND_HSD_PROTOCOL   (0x00000000)
#define BLE_COMMAND_DEBUG_CONSOLE  (0x01000000)
#define BLE_COMMAND_MLC            (0x02000000)


#define BLE_SUB_CMD_BASE           (0x00000000)
#define BLE_SUB_CMD_BOARD_NAME     (BLE_SUB_CMD_BASE + 1)
#define BLE_SUB_CMD_FW_VERSION     (BLE_SUB_CMD_BASE + 2)
#define BLE_SUB_CMD_INFO           (BLE_SUB_CMD_BASE + 3)
#define BLE_SUB_CMD_FOTA_START     (BLE_SUB_CMD_BASE + 4)
#define BLE_SUB_CMD_FOTA_COMPLETED (BLE_SUB_CMD_BASE + 5)
#define BLE_SUB_CMD_FOTA_ERROR     (BLE_SUB_CMD_BASE + 6)


void BLE_CM_SPI_Init(void);
void BLE_CM_SPI_DeInit(void);
void BLE_CM_SPI_Reset(void);
void BLE_CM_SPI_Enable_IRQ(void);
void BLE_CM_SPI_Disable_IRQ(void);
void BLE_CM_OS_Init(void);

/**
  * @brief  Send and Receive data to/from SPI BUS (Full duplex)
  * @param  pData: Data
  * @param  len: Length of data in byte
  * @retval BSP status
  */
int32_t BLE_CM_SPI_SendRecv(uint8_t *pTxData, uint8_t *pRxData, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __BLE_COMM_MANAGER_H */


