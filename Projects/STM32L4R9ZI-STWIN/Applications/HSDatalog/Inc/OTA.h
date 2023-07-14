/**
  ******************************************************************************
  * @file    OTA.h
  * @author  SRA
  * @brief   Over-the-Air Update API
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
#ifndef _OTA_H_
#define _OTA_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Exported defines ---------------------------------------------------------*/

/* 1008Kbytes Max Program Size */
#define OTA_MAX_PROG_SIZE (0x100000-0x4000-16)

extern uint8_t CurrentActiveBank;

/* Exported functions ---------------------------------------------------------*/

/* API for preparing the Flash for receiving the Update. It defines also the Size of the Update and the CRC value expected */
extern void StartUpdateFWBlueMS(uint32_t SizeOfUpdate, uint32_t uwCRCValue);
/* API for storing chuck of data to Flash.
 * When it has received the total number of byte defined by StartUpdateFWBlueMS,
 * it computes the CRC value and if it matches the expected CRC value,
 * it writes the Magic Number in Flash for BootLoader */
extern int8_t UpdateFWBlueMS(uint32_t *SizeOfUpdateBlueFW, uint8_t *att_data, int32_t data_length,
                             uint8_t WriteMagicNum);

extern void EnableDisableDualBoot(void);
void OTA_Verify_Current_Active_Bank(void);

#ifdef __cplusplus
}
#endif

#endif /* _OTA_H_ */

