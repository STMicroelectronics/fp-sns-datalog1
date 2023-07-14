/**
  ******************************************************************************
  * @file    usbd_wcid_interface.c
  * @author  SRA
  * @brief   Source file for USBD WCID interface template
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
#include "usbd_wcid_interface.h"
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static int8_t WCID_STREAMING_Itf_Init(void);
static int8_t WCID_STREAMING_Itf_DeInit(void);
static int8_t WCID_STREAMING_Itf_Control(uint8_t isHostToDevice, uint8_t cmd, uint16_t wValue, uint16_t wIndex,
                                         uint8_t *pbuf, uint16_t length);
static int8_t WCID_STREAMING_Itf_Receive(uint8_t *pbuf, uint32_t *Len);

USBD_WCID_STREAMING_ItfTypeDef USBD_WCID_STREAMING_fops =
{
  WCID_STREAMING_Itf_Init,
  WCID_STREAMING_Itf_DeInit,
  WCID_STREAMING_Itf_Control,
  WCID_STREAMING_Itf_Receive
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  WCID_STREAMING_Itf_Init
  *         Initializes the WCID media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t WCID_STREAMING_Itf_Init(void)
{
  return (USBD_OK);
}

/**
  * @brief  WCID_STREAMING_Itf_DeInit
  *         DeInitializes the WCID media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t WCID_STREAMING_Itf_DeInit(void)
{
  return (USBD_OK);
}


/**
  * @brief  WCID_STREAMING_Itf_Control
  *         Manage the WCID class requests
  * @param  Cmd: Command code
  * @param  Buf: Buffer containing command data (request parameters)
  * @param  Len: Number of data (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t WCID_STREAMING_Itf_Control(uint8_t isHostToDevice, uint8_t cmd, uint16_t wValue, uint16_t wIndex,
                                         uint8_t *pbuf, uint16_t length)
{
  return (USBD_OK);
}

/**
  * @brief  SS_WCID_Itf_DataRx
  *         Data received over USB OUT endpoint are sent over WCID interface
  *         through this function.
  * @param  Buf: Buffer of data to be transmitted
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t WCID_STREAMING_Itf_Receive(uint8_t *Buf, uint32_t *Len)
{
  return (USBD_OK);
}








