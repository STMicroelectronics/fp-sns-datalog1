/**
  ******************************************************************************
  * @file    OTA.c
  * @author  SRA
  * @brief   Over-the-Air Update API implementation
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
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "stm32l4xx_hal.h"
#include "OTA.h"

/* Local types ---------------------------------------------------------------*/

/* Local defines -------------------------------------------------------------*/

#ifndef OTA_PRINTF
#define OTA_PRINTF(...)
#endif /* OTA_PRINTF */
/* FW OTA Position */
/* The 2 addresses are equal due to swap address in dual boot mode */
#define OTA_ADDRESS_START  0x08100000

/* Local Macros -------------------------------------------------------------*/
#define OTA_ERROR_FUNCTION() { while(1);}

/* Private variables ---------------------------------------------------------*/
static uint32_t SizeOfUpdateBlueFW = 0;
static uint32_t ExpecteduwCRCValue = 0;
static uint32_t WritingAddress;

/* Current Active Bank */
uint8_t CurrentActiveBank = 0;

/* Exported functions  --------------------------------------------------*/

/**
  * @brief Function for Updating the Firmware
  * @param uint32_t *SizeOfUpdate Remaining size of the firmware image [bytes]
  * @param uint8_t *att_data attribute data
  * @param int32_t data_length length of the data
  * @param uint8_t WriteMagicNum 1/0 for writing or not the magic number
  * @retval int8_t Return value for checking purpouse (1/-1 == Ok/Error)
  */
int8_t UpdateFWBlueMS(uint32_t *SizeOfUpdate, uint8_t *att_data, int32_t data_length, uint8_t WriteMagicNum)
{
  int8_t ReturnValue = 0;
  /* Save the Packed received */

  if (data_length > (*SizeOfUpdate))
  {
    /* Too many bytes...Something wrong... necessity to send it again... */
    OTA_PRINTF("OTA something wrong data_length=%ld RemSizeOfUpdate=%ld....\r\nPlease Try again\r\n", data_length,
               (*SizeOfUpdate));
    ReturnValue = -1;
    /* Reset for Restarting again */
    *SizeOfUpdate = 0;
  }
  else
  {
    uint64_t ValueToWrite;
    int32_t Counter;
    /* Save the received OTA packed ad save it to flash */
    /* Unlock the Flash to enable the flash control register access *************/
    HAL_FLASH_Unlock();

    for (Counter = 0; Counter < data_length; Counter += 8)
    {
      memcpy((uint8_t *) &ValueToWrite, att_data + Counter, 8);

      if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, WritingAddress, ValueToWrite) == HAL_OK)
      {
        WritingAddress += 8;
      }
      else
      {
        /* Error occurred while writing data in Flash memory.
         User can add here some code to deal with this error
         FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError(); */
        OTA_ERROR_FUNCTION();
      }
    }
    /* Reduce the remaining bytes for OTA completion */
    *SizeOfUpdate -= data_length;

    if (*SizeOfUpdate == 0)
    {
      /* We have received the whole firmware and we have saved it in Flash */
      OTA_PRINTF("OTA Update saved\r\n");

      if (WriteMagicNum)
      {
        uint32_t uwCRCValue = 0;

        if (ExpecteduwCRCValue)
        {
          /* Make the CRC integrity check */
          /* CRC handler declaration */
          CRC_HandleTypeDef CrcHandle;

          /* Init CRC for OTA-integrity check */
          CrcHandle.Instance = CRC;
          /* The default polynomial is used */
          CrcHandle.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;

          /* The default init value is used */
          CrcHandle.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;

          /* The input data are not inverted */
          CrcHandle.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;

          /* The output data are not inverted */
          CrcHandle.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;

          /* The input data are 32-bit long words */
          CrcHandle.InputDataFormat = CRC_INPUTDATA_FORMAT_WORDS;

          if (HAL_CRC_GetState(&CrcHandle) != HAL_CRC_STATE_RESET)
          {
            HAL_CRC_DeInit(&CrcHandle);
          }

          if (HAL_CRC_Init(&CrcHandle) != HAL_OK)
          {
            /* Initialization Error */
            OTA_ERROR_FUNCTION();
          }
          else
          {
            OTA_PRINTF("CRC  Initialized\n\r");
          }
          /* Compute the CRC */
          uwCRCValue = HAL_CRC_Calculate(&CrcHandle, (uint32_t *) OTA_ADDRESS_START, SizeOfUpdateBlueFW >> 2);
          if (uwCRCValue == ExpecteduwCRCValue)
          {
            ReturnValue = 1;
            OTA_PRINTF("OTA CRC-checked\r\n");
          }
          else
          {
            OTA_PRINTF("OTA Error CRC-checking\r\n");
          }
        }
        else
        {
          ReturnValue = 1;

        }
        if (ReturnValue != 1)
        {
          ReturnValue = -1;
          if (ExpecteduwCRCValue)
          {
            OTA_PRINTF("Wrong CRC! Computed=%lx  expected=%lx ... Try again\r\n", uwCRCValue, ExpecteduwCRCValue);
          }
        }
      }
    }

    /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
    HAL_FLASH_Lock();
  }
  return ReturnValue;
}

/**
  * @brief Start Function for Updating the Firmware
  * @param uint32_t SizeOfUpdate  size of the firmware image [bytes]
  * @param uint32_t uwCRCValue expected CRV value
  * @retval None
  */
void StartUpdateFWBlueMS(uint32_t SizeOfUpdate, uint32_t uwCRCValue)
{
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t SectorError = 0;
  OTA_PRINTF("Start FLASH Erase\r\n");

  SizeOfUpdateBlueFW = SizeOfUpdate;
  ExpecteduwCRCValue = uwCRCValue;
  WritingAddress = OTA_ADDRESS_START;
  if (CurrentActiveBank == 1)
  {
    EraseInitStruct.Banks = FLASH_BANK_2;
  }
  else
  {
    EraseInitStruct.Banks = FLASH_BANK_1;
  }

  EraseInitStruct.Page = 0;
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.NbPages = (SizeOfUpdate + 16 + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE;

  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

  /* Clear OPTVERR bit set on virgin samples */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
  /* Clear PEMPTY bit set (as the code is executed from Flash which is not empty) */
  if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_PEMPTY) != 0)
  {
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PEMPTY);
  }

  if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
  {
    /* Error occurred while sector erase.
     User can add here some code to deal with this error.
     SectorError will contain the faulty sector and then to know the code error on this sector,
     user can call function 'HAL_FLASH_GetError()'
     FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError(); */
    OTA_ERROR_FUNCTION();
  }
  else
  {
    OTA_PRINTF("End FLASH Erase %ld Pages of 4KB\r\n", EraseInitStruct.NbPages);
  }

  /* Lock the Flash to disable the flash control register access (recommended
   to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();
}

void OTA_Verify_Current_Active_Bank(void)
{
  FLASH_OBProgramInitTypeDef OBInit;
  /* Allow Access to Flash control registers and user Flash */
  HAL_FLASH_Unlock();
  /* Allow Access to option bytes sector */
  HAL_FLASH_OB_Unlock();
  /* Get the Dual boot configuration status */
  HAL_FLASHEx_OBGetConfig(&OBInit);

  if (((OBInit.USERConfig) & (OB_BFB2_ENABLE)) == OB_BFB2_ENABLE)
  {
    CurrentActiveBank = 2;
  }
  else
  {
    CurrentActiveBank = 1;
  }
  HAL_FLASH_OB_Lock();
  HAL_FLASH_Lock();
}

/**
  * @brief  Enable Disable the jump to second flash bank and reboot board
  * @param  None
  * @retval None
  */
void EnableDisableDualBoot(void)
{
  FLASH_OBProgramInitTypeDef OBInit;
  /* Set BFB2 bit to enable boot from Flash Bank2 */
  /* Allow Access to Flash control registers and user Flash */
  HAL_FLASH_Unlock();

  /* Clear OPTVERR bit set on virgin samples */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

  /* Allow Access to option bytes sector */
  HAL_FLASH_OB_Unlock();

  /* Get the Dual boot configuration status */
  HAL_FLASHEx_OBGetConfig(&OBInit);

  /* Enable/Disable dual boot feature */
  OBInit.OptionType = OPTIONBYTE_USER;
  OBInit.USERType = OB_USER_BFB2;

  if (((OBInit.USERConfig) & (OB_BFB2_ENABLE)) == OB_BFB2_ENABLE)
  {
    OBInit.USERConfig = OB_BFB2_DISABLE;
    OTA_PRINTF("->Disable DualBoot\r\n");
  }
  else
  {
    OBInit.USERConfig = OB_BFB2_ENABLE;
    OTA_PRINTF("->Enable DualBoot\r\n");
  }

  if (HAL_FLASHEx_OBProgram(&OBInit) != HAL_OK)
  {
    /*
     Error occurred while setting option bytes configuration.
     User can add here some code to deal with this error.
     To know the code error, user can call function 'HAL_FLASH_GetError()'
     */
    while (1);
  }

  /* Start the Option Bytes programming process */
  if (HAL_FLASH_OB_Launch() != HAL_OK)
  {
    /*
     Error occurred while reloading option bytes configuration.
     User can add here some code to deal with this error.
     To know the code error, user can call function 'HAL_FLASH_GetError()'
     */
    while (1);
  }
  HAL_FLASH_OB_Lock();
  HAL_FLASH_Lock();
}

/**
  * @brief CRC MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  * @param hcrc: CRC handle pointer
  * @retval None
  */
void HAL_CRC_MspInit(CRC_HandleTypeDef *hcrc)
{
  /* CRC Peripheral clock enable */
  __HAL_RCC_CRC_CLK_ENABLE();
}

/**
  * @brief CRC MSP De-Initialization
  *        This function freeze the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  * @param hcrc: CRC handle pointer
  * @retval None
  */
void HAL_CRC_MspDeInit(CRC_HandleTypeDef *hcrc)
{
  /* Enable CRC reset state */
  __HAL_RCC_CRC_FORCE_RESET();

  /* Release CRC from reset state */
  __HAL_RCC_CRC_RELEASE_RESET();
}

