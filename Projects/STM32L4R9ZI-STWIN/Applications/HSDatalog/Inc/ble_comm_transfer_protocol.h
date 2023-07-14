/**
  ******************************************************************************
  * @file    ble_comm_transfer_protocol.h
  * @author  SRA
  *
  *
  * @brief   This file contains definitions for BLE commands transfer protocol.
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
#ifndef __BLE_COMMANDS_TRANSFER_PROTOCOL_H
#define __BLE_COMMANDS_TRANSFER_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/**
  * @brief  This function is called to parse a BLE_COMM_TP packet.
  * @param  buffer_out: pointer to the output buffer.
  * @param  buffer_in: pointer to the input data.
  * @param  len: buffer in length
  * @retval Buffer out length.
  */
uint32_t BLECommand_TP_Parse(uint8_t **buffer_out, uint8_t *buffer_in, uint32_t len);

/**
  * @brief  This function is called to prepare a BLE_COMM_TP packet.
  * @param  buffer_out: pointer to the buffer used to save BLE_COMM_TP packet.
  * @param  buffer_in: pointer to the input data.
  * @param  len: buffer in length
  * @retval Buffer out length.
  */
uint32_t BLECommand_TP_Encapsulate(uint8_t *buffer_out, uint8_t *buffer_in, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __BLE_COMMANDS_TRANSFER_PROTOCOL_H */


