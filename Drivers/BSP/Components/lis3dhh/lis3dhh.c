/**
  ******************************************************************************
  * @file    lis3dhh.c
  * @author  MEMS Software Solutions Team
  * @brief   LIS3DHH driver file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "lis3dhh.h"

/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup Component Component
 * @{
 */

/** @defgroup LIS3DHH LIS3DHH
 * @{
 */

/** @defgroup LIS3DHH_Exported_Variables LIS3DHH Exported Variables
 * @{
 */

LIS3DHH_CommonDrv_t LIS3DHH_COMMON_Driver =
{
  LIS3DHH_Init,
  LIS3DHH_DeInit,
  LIS3DHH_ReadID,
  LIS3DHH_GetCapabilities,
};

LIS3DHH_ACC_Drv_t LIS3DHH_ACC_Driver =
{
  LIS3DHH_ACC_Enable,
  LIS3DHH_ACC_Disable,
  LIS3DHH_ACC_GetSensitivity,
  LIS3DHH_ACC_GetOutputDataRate,
  LIS3DHH_ACC_SetOutputDataRate,
  LIS3DHH_ACC_GetFullScale,
  LIS3DHH_ACC_SetFullScale,
  LIS3DHH_ACC_GetAxes,
  LIS3DHH_ACC_GetAxesRaw,
};

/**
 * @}
 */

/** @defgroup LIS3DHH_Private_Function_Prototypes LIS3DHH Private Function Prototypes
 * @{
 */

static int32_t ReadRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length);
static int32_t WriteRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length);
static int32_t LIS3DHH_ACC_SetOutputDataRate_When_Enabled(LIS3DHH_Object_t *pObj, float Odr);
static int32_t LIS3DHH_ACC_SetOutputDataRate_When_Disabled(LIS3DHH_Object_t *pObj, float Odr);

/**
 * @}
 */

/** @defgroup LIS3DHH_Exported_Functions LIS3DHH Exported Functions
 * @{
 */

/**
 * @brief  Register Component Bus IO operations
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LIS3DHH_RegisterBusIO(LIS3DHH_Object_t *pObj, LIS3DHH_IO_t *pIO)
{
  int32_t ret = LIS3DHH_OK;

  if (pObj == NULL)
  {
    ret = LIS3DHH_ERROR;
  }
  else
  {
    pObj->IO.Init      = pIO->Init;
    pObj->IO.DeInit    = pIO->DeInit;
    pObj->IO.BusType   = pIO->BusType;
    pObj->IO.Address   = pIO->Address;
    pObj->IO.WriteReg  = pIO->WriteReg;
    pObj->IO.ReadReg   = pIO->ReadReg;
    pObj->IO.GetTick   = pIO->GetTick;

    pObj->Ctx.read_reg  = ReadRegWrap;
    pObj->Ctx.write_reg = WriteRegWrap;
    pObj->Ctx.handle   = pObj;

    if (pObj->IO.Init == NULL)
    {
      ret = LIS3DHH_ERROR;
    }
    else if (pObj->IO.Init() != LIS3DHH_OK)
    {
      ret = LIS3DHH_ERROR;
    }
  }

  return ret;
}

/**
 * @brief  Initialize the LIS3DHH sensor
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LIS3DHH_Init(LIS3DHH_Object_t *pObj)
{
  /* Enable register address automatically incremented during a multiple byte
  access with a serial interface. */
  if (lis3dhh_auto_add_inc_set(&(pObj->Ctx), PROPERTY_ENABLE) != LIS3DHH_OK)
  {
    return LIS3DHH_ERROR;
  }

  /* Enable BDU */
  if (lis3dhh_block_data_update_set(&(pObj->Ctx), PROPERTY_ENABLE) != LIS3DHH_OK)
  {
    return LIS3DHH_ERROR;
  }

  /* FIFO mode selection */
  if (lis3dhh_fifo_mode_set(&(pObj->Ctx), LIS3DHH_BYPASS_MODE) != LIS3DHH_OK)
  {
    return LIS3DHH_ERROR;
  }

  /* Select default output data rate. */
  pObj->acc_odr = 1100.0f;
  
  /* Power mode selection - power down */
  if(lis3dhh_data_rate_set(&(pObj->Ctx), LIS3DHH_POWER_DOWN) != LIS3DHH_OK)
  {
    return LIS3DHH_ERROR;
  }
    
  pObj->is_initialized = 1;

  return LIS3DHH_OK;
}

/**
 * @brief  Deinitialize the LIS3DHH sensor
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LIS3DHH_DeInit(LIS3DHH_Object_t *pObj)
{
  /* Disable the component */
  if (LIS3DHH_ACC_Disable(pObj) != LIS3DHH_OK)
  {
    return LIS3DHH_ERROR;
  }

  /* Reset output data rate. */
  pObj->acc_odr = 0.0f;

  pObj->is_initialized = 0;

  return LIS3DHH_OK;
}

/**
 * @brief  Read component ID
 * @param  pObj the device pObj
 * @param  Id the WHO_AM_I value
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LIS3DHH_ReadID(LIS3DHH_Object_t *pObj, uint8_t *Id)
{
  if (lis3dhh_device_id_get(&(pObj->Ctx), Id) != LIS3DHH_OK)
  {
    return LIS3DHH_ERROR;
  }

  return LIS3DHH_OK;
}

/**
 * @brief  Get LIS3DHH sensor capabilities
 * @param  pObj Component object pointer
 * @param  Capabilities pointer to LIS3DHH sensor capabilities
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LIS3DHH_GetCapabilities(LIS3DHH_Object_t *pObj, LIS3DHH_Capabilities_t *Capabilities)
{
  /* Prevent unused argument(s) compilation warning */
  (void)(pObj);

  Capabilities->Acc          = 1;
  Capabilities->Gyro         = 0;
  Capabilities->Magneto      = 0;
  Capabilities->LowPower     = 0;
  Capabilities->GyroMaxFS    = 0;
  Capabilities->AccMaxFS     = 2;
  Capabilities->MagMaxFS     = 0;
  Capabilities->GyroMaxOdr   = 0.0f;
  Capabilities->AccMaxOdr    = 1100.0f;
  Capabilities->MagMaxOdr    = 0.0f;
  return LIS3DHH_OK;
}

/**
 * @brief  Enable the LIS3DHH accelerometer sensor
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LIS3DHH_ACC_Enable(LIS3DHH_Object_t *pObj)
{
  /* Check if the component is already enabled */
  if (pObj->acc_is_enabled == 1U)
  {
    return LIS3DHH_OK;
  }
  
  /* Output data rate selection. */
  if (LIS3DHH_ACC_SetOutputDataRate_When_Enabled(pObj, pObj->acc_odr) != LIS3DHH_OK)
  {
    return LIS3DHH_ERROR;
  }

  pObj->acc_is_enabled = 1;

  return LIS3DHH_OK;
}

/**
 * @brief  Disable the LIS3DHH accelerometer sensor
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LIS3DHH_ACC_Disable(LIS3DHH_Object_t *pObj)
{
  /* Check if the component is already disabled */
  if (pObj->acc_is_enabled == 0U)
  {
    return LIS3DHH_OK;
  }

  /* Get current output data rate. */
  if (LIS3DHH_ACC_GetOutputDataRate(pObj, &pObj->acc_odr) != LIS3DHH_OK)
  {
    return LIS3DHH_ERROR;
  }

  /* Output data rate selection - power down. */
  if(lis3dhh_data_rate_set(&(pObj->Ctx), LIS3DHH_POWER_DOWN) != LIS3DHH_OK)
  {
    return LIS3DHH_ERROR;
  }

  pObj->acc_is_enabled = 0;

  return LIS3DHH_OK;
}

/**
 * @brief  Get the LIS3DHH accelerometer sensor sensitivity
 * @param  pObj the device pObj
 * @param  Sensitivity pointer
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LIS3DHH_ACC_GetSensitivity(LIS3DHH_Object_t *pObj, float *Sensitivity)
{
  int32_t ret = LIS3DHH_OK;
  
  /* There is only one value of sensitivity */
  *Sensitivity = LIS3DHH_ACC_SENSITIVITY;
  
  return ret;
}

/**
 * @brief  Get the LIS3DHH accelerometer sensor output data rate
 * @param  pObj the device pObj
 * @param  Odr pointer where the output data rate is written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LIS3DHH_ACC_GetOutputDataRate(LIS3DHH_Object_t *pObj, float *Odr)
{
  int32_t ret = LIS3DHH_OK;
  lis3dhh_norm_mod_en_t mode;

  /* Read actual power mode selection from sensor. */
  if (lis3dhh_data_rate_get(&(pObj->Ctx), &mode) != LIS3DHH_OK)
  {
    return LIS3DHH_ERROR;
  }

  switch(mode)
  {
    case LIS3DHH_POWER_DOWN:
      ret = LIS3DHH_ERROR;
      break;
      
    case LIS3DHH_1kHz1:
      *Odr = 1100.0f;
      ret = LIS3DHH_OK;
      break;
     
    default:
      *Odr = -1.0f;
      ret = LIS3DHH_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Set the LIS3DHH accelerometer sensor output data rate
 * @param  pObj the device pObj
 * @param  Odr the output data rate value to be set
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LIS3DHH_ACC_SetOutputDataRate(LIS3DHH_Object_t *pObj, float Odr)
{
  /* Check if the component is enabled */
  if (pObj->acc_is_enabled == 1U)
  {
    return LIS3DHH_ACC_SetOutputDataRate_When_Enabled(pObj, Odr);
  }
  else
  {
    return LIS3DHH_ACC_SetOutputDataRate_When_Disabled(pObj, Odr);
  }
}

/**
 * @brief  Get the LIS3DHH accelerometer sensor full scale
 * @param  pObj the device pObj
 * @param  FullScale pointer where the full scale is written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LIS3DHH_ACC_GetFullScale(LIS3DHH_Object_t *pObj, int32_t *FullScale)
{
  int32_t ret = LIS3DHH_OK;
  
  *FullScale =  2;
  
  return ret;
}

/**
 * @brief  Set the LIS3DHH accelerometer sensor full scale
 * @param  pObj the device pObj
 * @param  FullScale pointer where the full scale is written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LIS3DHH_ACC_SetFullScale(LIS3DHH_Object_t *pObj, int32_t FullScale)
{
  return LIS3DHH_OK;
}

/**
 * @brief  Get the LIS3DHH accelerometer sensor raw axes
 * @param  pObj the device pObj
 * @param  Value pointer where the raw values of the axes are written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LIS3DHH_ACC_GetAxesRaw(LIS3DHH_Object_t *pObj, LIS3DHH_AxesRaw_t *Value)
{
  lis3dhh_axis3bit16_t data_raw;
  lis3dhh_norm_mod_en_t mode;
  int32_t ret = LIS3DHH_OK;

  /* Read actual power mode selection from sensor. */
  if (lis3dhh_data_rate_get(&(pObj->Ctx), &mode) != LIS3DHH_OK)
  {
    return LIS3DHH_ERROR;
  }

  /* Read raw data values. */
  if (lis3dhh_acceleration_raw_get(&(pObj->Ctx), data_raw.i16bit) != LIS3DHH_OK)
  {
    return LIS3DHH_ERROR;
  }

  switch(mode)
  {
  case LIS3DHH_POWER_DOWN:
    ret = LIS3DHH_ERROR;
    break;
    
  case LIS3DHH_1kHz1:
    Value->x = (data_raw.i16bit[0]);
    Value->y = (data_raw.i16bit[1]);
    Value->z = (data_raw.i16bit[2]);
    break;

    default:
      ret = LIS3DHH_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Get the LIS3DHH accelerometer sensor axes
 * @param  pObj the device pObj
 * @param  Acceleration pointer where the values of the axes are written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LIS3DHH_ACC_GetAxes(LIS3DHH_Object_t *pObj, LIS3DHH_Axes_t *Acceleration)
{
  LIS3DHH_AxesRaw_t data_raw;
  
  /* Read raw data values. */
  if (LIS3DHH_ACC_GetAxesRaw(pObj, &data_raw) != LIS3DHH_OK)
  {
    return LIS3DHH_ERROR;
  }

  /* Calculate the data. */
  Acceleration->x = (int32_t)lis3dhh_from_lsb_to_mg(data_raw.x);
  Acceleration->y = (int32_t)lis3dhh_from_lsb_to_mg(data_raw.y);
  Acceleration->z = (int32_t)lis3dhh_from_lsb_to_mg(data_raw.z);

  return LIS3DHH_OK;
}

/**
 * @brief  Get the LIS3DHH register value
 * @param  pObj the device pObj
 * @param  Reg address to be read
 * @param  Data pointer where the value is written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LIS3DHH_Read_Reg(LIS3DHH_Object_t *pObj, uint8_t Reg, uint8_t *Data)
{
  if (lis3dhh_read_reg(&(pObj->Ctx), Reg, Data, 1) != LIS3DHH_OK)
  {
    return LIS3DHH_ERROR;
  }

  return LIS3DHH_OK;
}

/**
 * @brief  Set the LIS3DHH register value
 * @param  pObj the device pObj
 * @param  Reg address to be written
 * @param  Data value to be written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LIS3DHH_Write_Reg(LIS3DHH_Object_t *pObj, uint8_t Reg, uint8_t Data)
{
  if (lis3dhh_write_reg(&(pObj->Ctx), Reg, &Data, 1) != LIS3DHH_OK)
  {
    return LIS3DHH_ERROR;
  }

  return LIS3DHH_OK;
}

/**
 * @brief  Enable DRDY interrupt mode
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LIS3DHH_ACC_Enable_DRDY_Interrupt(LIS3DHH_Object_t *pObj)
{
  /* Interrupt set to INT2 */
  if(lis3dhh_drdy_on_int2_set(&(pObj->Ctx), 1) != LIS3DHH_OK)
  {
    return LIS3DHH_ERROR;
  }
  
  
  return LIS3DHH_OK;
}

/**
 * @brief  Set the filterMode value
 * @param  pObj the device pObj
 * @param  filterMode value to be written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LIS3DHH_ACC_Set_Filter_Mode(LIS3DHH_Object_t *pObj, uint8_t filterMode)
{
  if(lis3dhh_filter_config_set(&(pObj->Ctx), (lis3dhh_dsp_t)filterMode) != LIS3DHH_OK)
  {
    return LIS3DHH_ERROR;
  }
  
  return LIS3DHH_OK;  
}


/**
 * @brief  Get the LIS3DHH ACC data ready bit value
 * @param  pObj the device pObj
 * @param  Status the status of data ready bit
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LIS3DHH_ACC_Get_DRDY_Status(LIS3DHH_Object_t *pObj, uint8_t *Status)
{
  if (lis3dhh_xl_data_ready_get(&(pObj->Ctx), Status) != LIS3DHH_OK)
  {
    return LIS3DHH_ERROR;
  }

  return LIS3DHH_OK;
}

/**
 * @brief  Get the number of samples contained into the FIFO
 * @param  pObj the device pObj
 * @param  NumSamples the number of samples contained into the FIFO
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LIS3DHH_FIFO_Get_Num_Samples(LIS3DHH_Object_t *pObj, uint16_t *NumSamples)
{
  lis3dhh_fifo_src_t FIFO_Samples;

  if (lis3dhh_read_reg(&(pObj->Ctx), LIS3DHH_FIFO_SRC, (uint8_t *)&FIFO_Samples, 1) != LIS3DHH_OK)
  {
    return LIS3DHH_ERROR;
  }

  if(FIFO_Samples.fss == 0x20)
  {
    *NumSamples = 32;
  }
  else
  {
    *NumSamples = FIFO_Samples.fss;
  }

  return LIS3DHH_OK;
}

/**
 * @brief  Set the FIFO mode
 * @param  pObj the device pObj
 * @param  Mode FIFO mode
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LIS3DHH_FIFO_Set_Mode(LIS3DHH_Object_t *pObj, uint8_t Mode)
{
  int32_t ret = LIS3DHH_OK;

  /* Verify that the passed parameter contains one of the valid values. */
  switch ((lis3dhh_fmode_t)Mode)
  {
    case LIS3DHH_BYPASS_MODE:
    case LIS3DHH_FIFO_MODE:
    case LIS3DHH_STREAM_TO_FIFO_MODE:
    case LIS3DHH_BYPASS_TO_STREAM_MODE:
    case LIS3DHH_DYNAMIC_STREAM_MODE:
      break;

    default:
      ret = LIS3DHH_ERROR;
      break;
  }

  if (ret == LIS3DHH_ERROR)
  {
    return ret;
  }

  if (lis3dhh_fifo_mode_set(&(pObj->Ctx), (lis3dhh_fmode_t)Mode) != LIS3DHH_OK)
  {
    return LIS3DHH_ERROR;
  }

  return ret;
}


/** @defgroup LIS3DHH_Private_Functions LIS3DHH Private Functions
 * @{
 */

/**
 * @brief  Set the LIS3DHH accelerometer sensor output data rate when enabled
 * @param  pObj the device pObj
 * @param  Odr the functional output data rate to be set
 * @retval 0 in case of success, an error code otherwise
 */
static int32_t LIS3DHH_ACC_SetOutputDataRate_When_Enabled(LIS3DHH_Object_t *pObj, float Odr)
{
  
  lis3dhh_norm_mod_en_t new_odr;

  if(Odr <= 1100)
  {
    new_odr = LIS3DHH_1kHz1;
    /* Output data rate selection. */
    if (lis3dhh_data_rate_set(&(pObj->Ctx), new_odr) != LIS3DHH_OK)
    {
      return LIS3DHH_ERROR;
    }
  }
  else
  {
    return LIS3DHH_ERROR;
  }
  
  pObj->acc_odr = new_odr;
  
  return LIS3DHH_OK;
}

/**
 * @brief  Set the LIS3DHH accelerometer sensor output data rate when disabled
 * @param  pObj the device pObj
 * @param  Odr the functional output data rate to be set
 * @retval 0 in case of success, an error code otherwise
 */
static int32_t LIS3DHH_ACC_SetOutputDataRate_When_Disabled(LIS3DHH_Object_t *pObj, float Odr)
{

  if(Odr <= 1100.0f)
  {
    pObj->acc_odr = 1100.0f;
  }
  else
  {
    return LIS3DHH_ERROR;
  }

  return LIS3DHH_OK;
}

/**
 * @brief  Wrap Read register component function to Bus IO function
 * @param  Handle the device handler
 * @param  Reg the register address
 * @param  pData the stored data pointer
 * @param  Length the length
 * @retval 0 in case of success, an error code otherwise
 */
static int32_t ReadRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length)
{
  LIS3DHH_Object_t *pObj = (LIS3DHH_Object_t *)Handle;

  return pObj->IO.ReadReg(pObj->IO.Address, Reg, pData, Length);
}

/**
 * @brief  Wrap Write register component function to Bus IO function
 * @param  Handle the device handler
 * @param  Reg the register address
 * @param  pData the stored data pointer
 * @param  Length the length
 * @retval 0 in case of success, an error code otherwise
 */
static int32_t WriteRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length)
{
  LIS3DHH_Object_t *pObj = (LIS3DHH_Object_t *)Handle;

  return pObj->IO.WriteReg(pObj->IO.Address, Reg, pData, Length);
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
