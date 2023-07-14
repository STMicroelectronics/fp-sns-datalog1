/**
  ******************************************************************************
  * @file    usbd_wcid_interface.h
  * @author  SRA
  *
  *
  * @brief   Header file for USBD WCID interface
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
#ifndef __USBD_WCID_STREAMING_IF_H
#define __USBD_WCID_STREAMING_IF_H

/* Includes ------------------------------------------------------------------*/
#include "usbd_wcid_streaming.h"

/*#define USB_PACKET_SIZE         4096*2 */
/*#define APP_TX_DATA_SIZE        (USB_PACKET_SIZE*2) */
/*#define APP_RX_DATA_SIZE        512 */

#define USBD_WCID_WAITING_FOR_SIZE               (uint8_t)(0x00)
#define USBD_WCID_WAITING_FOR_SIZE_REQUEST       (uint8_t)(0x01)
#define USBD_WCID_WAITING_FOR_DATA_REQUEST       (uint8_t)(0x02)
/*#define STATE_DATA_GET                           (uint8_t)(0x03) */
/*#define STATE_DATA_SET                           (uint8_t)(0x04) */
#define USB_WCID_WAITING_FOR_DATA                (uint8_t)(0x05)

#define CMD_SET_REQ           (uint8_t)(0x00)
#define CMD_SIZE_GET          (uint8_t)(0x01)
#define CMD_DATA_GET          (uint8_t)(0x02)
#define CMD_SIZE_SET          (uint8_t)(0x03)
#define CMD_DATA_SET          (uint8_t)(0x04)

extern USBD_WCID_STREAMING_ItfTypeDef USBD_WCID_STREAMING_fops;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __USBD_WCID_STREAMING_IF_H */

