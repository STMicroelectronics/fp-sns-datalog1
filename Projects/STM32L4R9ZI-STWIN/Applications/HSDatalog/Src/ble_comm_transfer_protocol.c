/**
  ******************************************************************************
  * @file    ble_comm_transfer_protocol.c
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

/* Includes ------------------------------------------------------------------*/
#include "ble_comm_transfer_protocol.h"
#include "main.h"
#include "HSDCore.h"
#include "stdlib.h"
#include <string.h>

#ifndef MIN
#define MIN(a,b)            ((a) < (b) )? (a) : (b)
#endif /* MIN */

extern uint16_t MaxBLECharLen;


typedef enum
{
  BLE_COMM_TP_START_PACKET = 0x00,
  BLE_COMM_TP_START_END_PACKET = 0x20,
  BLE_COMM_TP_MIDDLE_PACKET = 0x40,
  BLE_COMM_TP_END_PACKET = 0x80
} BLE_COMM_TP_Packet_Typedef;

typedef enum
{
  BLE_COMM_TP_WAIT_START = 0,
  BLE_COMM_TP_WAIT_END = 1
} BLE_COMM_TP_Status_Typedef;

/**
  * @brief  This function is called to parse a BLE_COMM_TP packet.
  * @param  buffer_out: pointer to the output buffer.
  * @param  buffer_in: pointer to the input data.
  * @param  len: buffer in length
  * @retval Buffer out length.
  */
uint32_t BLECommand_TP_Parse(uint8_t **buffer_out, uint8_t *buffer_in, uint32_t len)
{
  static uint32_t tot_len = 0;
  uint32_t buff_out_len;
  static BLE_COMM_TP_Status_Typedef status = BLE_COMM_TP_WAIT_START;
  BLE_COMM_TP_Packet_Typedef packet_type;

  packet_type = (BLE_COMM_TP_Packet_Typedef) buffer_in[0];

  switch (status)
  {
    case BLE_COMM_TP_WAIT_START:
    {
      if (packet_type == BLE_COMM_TP_START_PACKET)
      {
        /*First part of an BLE Command packet*/
        /*packet is enqueued*/
        uint16_t message_length = buffer_in[1];
        message_length = message_length << 8;
        message_length |= buffer_in[2];

        *buffer_out = (uint8_t *)HSD_malloc((message_length) * sizeof(uint8_t));

        if (*buffer_out == NULL)
        {
          HSD_PRINTF("Mem alloc error [%d]: %d@%s\r\n", message_length, __LINE__, __FILE__);
        }
        else
        {
          HSD_PRINTF("Mem alloc ok [%d]: %d@%s\r\n", message_length, __LINE__, __FILE__);
        }

        memcpy(*buffer_out + tot_len, (uint8_t *) &buffer_in[3], (len - 3));
        tot_len += len - 3;
        status = BLE_COMM_TP_WAIT_END;
        return 0;
      }
      else if (packet_type == BLE_COMM_TP_START_END_PACKET)
      {
        /*Final part of an BLE Command packet*/
        /*packet is enqueued*/
        uint16_t message_length = buffer_in[1];
        message_length = message_length << 8;
        message_length |= buffer_in[2];

        *buffer_out = (uint8_t *)HSD_malloc((message_length) * sizeof(uint8_t));
        if (*buffer_out == NULL)
        {
          HSD_PRINTF("Mem alloc error [%d]: %d@%s\r\n", message_length, __LINE__, __FILE__);
        }
        else
        {
          HSD_PRINTF("Mem alloc ok [%d]: %d@%s\r\n", message_length, __LINE__, __FILE__);
        }

        memcpy(*buffer_out + tot_len, (uint8_t *) &buffer_in[3], (len - 3));

        tot_len += len - 3;
        /*number of bytes of the output packet*/
        buff_out_len = tot_len;
        /*total length set to zero*/
        tot_len = 0;
        /*reset status*/
        status = BLE_COMM_TP_WAIT_START;
        /*return decoded output dimension*/
        return buff_out_len;
      }
      else
      {
        /* Error */
        return 0;
      }
    }
    case BLE_COMM_TP_WAIT_END:
    {
      if (packet_type == BLE_COMM_TP_MIDDLE_PACKET)
      {
        /*Central part of an BLE Command packet*/
        /*packet is enqueued*/
        memcpy(*buffer_out + tot_len, (uint8_t *) &buffer_in[1], (len - 1));

        tot_len += len - 1;
        return 0;
      }
      else if (packet_type == BLE_COMM_TP_END_PACKET)
      {
        /*Final part of an BLE Command packet*/
        /*packet is enqueued*/
        memcpy(*buffer_out + tot_len, (uint8_t *) &buffer_in[1], (len - 1));

        tot_len += len - 1;
        /*number of bytes of the output packet*/
        buff_out_len = tot_len;
        /*total length set to zero*/
        tot_len = 0;
        /*reset status*/
        status = BLE_COMM_TP_WAIT_START;
        /*return decoded output dimension*/
        return buff_out_len;
      }
      else
      {
        /*reset status*/
        status = BLE_COMM_TP_WAIT_START;
        /*total length set to zero*/
        tot_len = 0;
        return 0; /* error */
      }
    }
  }
  return 0;
}

/**
  * @brief  This function is called to prepare a BLE_COMM_TP packet.
  * @param  buffer_out: pointer to the buffer used to save BLE_COMM_TP packet.
  * @param  buffer_in: pointer to the input data.
  * @param  len: buffer in length
  * @retval Buffer out length.
  */
uint32_t BLECommand_TP_Encapsulate(uint8_t *buffer_out, uint8_t *buffer_in, uint16_t len)
{
  uint32_t size = 0;
  uint32_t tot_size = 0;
  uint32_t counter = 0;
  BLE_COMM_TP_Packet_Typedef packet_type = BLE_COMM_TP_START_PACKET;

  /* One byte header is added to each BLE packet */
  while (counter < len)
  {
    size = MIN((MaxBLECharLen - 1), len - counter);

    if (len - counter <= (MaxBLECharLen - 1))
    {
      if (counter == 0)
      {
        packet_type = BLE_COMM_TP_START_END_PACKET;
      }
      else
      {
        packet_type = BLE_COMM_TP_END_PACKET;
      }
    }

    switch (packet_type)
    {
      case BLE_COMM_TP_START_PACKET:
      {
        /*First part of an BLE Command packet*/
        buffer_out[tot_size] = ((uint8_t)(BLE_COMM_TP_START_PACKET));
        tot_size++;
        packet_type = BLE_COMM_TP_MIDDLE_PACKET;
      }
      break;
      case BLE_COMM_TP_START_END_PACKET:
      {
        /*First and last part of an BLE Command packet*/
        buffer_out[tot_size] = ((uint8_t)(BLE_COMM_TP_START_END_PACKET));
        tot_size++;
        packet_type = BLE_COMM_TP_START_PACKET;
      }
      break;
      case BLE_COMM_TP_MIDDLE_PACKET:
      {
        /*Central part of an BLE Command packet*/
        buffer_out[tot_size] = ((uint8_t)(BLE_COMM_TP_MIDDLE_PACKET));
        tot_size++;
      }
      break;
      case BLE_COMM_TP_END_PACKET:
      {
        /*Last part of an BLE Command packet*/
        buffer_out[tot_size] = ((uint8_t)(BLE_COMM_TP_END_PACKET));
        tot_size++;
        packet_type = BLE_COMM_TP_START_PACKET;
      }
      break;
    }

    /*Input data is incapsulated*/
    memcpy((uint8_t *) &buffer_out[tot_size], (uint8_t *) &buffer_in[counter], size);

    /*length variables update*/
    counter += size;
    tot_size += size;
  }
  return tot_size;
}


