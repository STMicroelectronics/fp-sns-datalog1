/**
******************************************************************************
* @file    SensorTile.box_audio.h
* @author  SRA - Central Labs
* @version V1.3.5
* @date    10-Feb-2022
* @brief   This file contains the common defines and functions prototypes for
*          SensorTile.box_audio.c driver.
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
#ifndef __SENSORTILEBOX_AUDIO_H
#define __SENSORTILEBOX_AUDIO_H

#ifdef __cplusplus
extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "SensorTile.box_conf.h"
#include "SensorTile.box.h"
#if (USE_AUDIO_IN == 1)
#include "audio.h"

/** @addtogroup BSP
  * @{
  */ 

/** @addtogroup SENSORTILEBOX
  * @{
  */
    
/** @addtogroup SENSORTILEBOX_AUDIO 
  * @{
  */
   
/** @defgroup SENSORTILEBOX_AUDIO_Exported_Variables
 * @{
 */
/* AMic OnBoard */
extern DMA_HandleTypeDef AMic_OnBoard_Dma;
extern DFSDM_Filter_HandleTypeDef   AMic_OnBoard_DfsdmFilter;

#ifndef AUDIO_SHARED_TYPES
#define AUDIO_SHARED_TYPES

#define HTONS(A)  ((((uint16_t)(A) & 0xff00) >> 8) | \
                   (((uint16_t)(A) & 0x00ff) << 8))

#define SaturaLH(N, L, H) (((N)<(L))?(L):(((N)>(H))?(H):(N)))

/**
 * @}
 */   

/** @defgroup SENSORTILEBOX_AUDIO_Exported_Types SENSORTILEBOX_AUDIO Exported Types
  * @{
  */

typedef struct {
  int32_t Z;
  int32_t oldOut;
  int32_t oldIn;
}HP_FilterState_TypeDef;  
  
typedef struct {
  uint32_t Device;
  uint32_t SampleRate;
  uint32_t BitsPerSample;
  uint32_t ChannelsNbr;
  uint32_t Volume;
}BSP_AUDIO_Init_t;

typedef struct
{
  uint32_t               Instance;            /* Audio IN instance              */  
  uint32_t               Device;              /* Audio IN device to be used     */ 
  uint32_t               SampleRate;          /* Audio IN Sample rate           */
  uint32_t               BitsPerSample;       /* Audio IN Sample resolution     */
  uint32_t               ChannelsNbr;         /* Audio IN number of channel     */
  uint16_t               *pBuff;              /* Audio IN record buffer         */
  uint8_t                **pMultiBuff;        /* Audio IN multi-buffer          */
  uint32_t               Size;                /* Audio IN record buffer size    */
  uint32_t               Volume;              /* Audio IN volume                */
  uint32_t               State;               /* Audio IN State                 */
  uint32_t               IsMultiBuff;         /* Audio IN multi-buffer usage    */
  uint32_t               IsMspCallbacksValid; /* Is Msp Callbacks registred     */
  HP_FilterState_TypeDef HP_Filters;          /*!< HP filter state*/
  uint32_t DecimationFactor;
}AUDIO_IN_Ctx_t;

typedef struct
{
  uint32_t Instance;            /* Audio OUT instance              */  
  uint32_t Device;              /* Audio OUT device to be used     */ 
  uint32_t SampleRate;          /* Audio OUT Sample rate           */
  uint32_t BitsPerSample;       /* Audio OUT Sample Bit Per Sample */
  uint32_t Volume;              /* Audio OUT volume                */
  uint32_t ChannelsNbr;         /* Audio OUT number of channel     */
  uint32_t IsMute;              /* Mute state                      */   
  uint32_t State;               /* Audio OUT State                 */
  uint32_t IsMspCallbacksValid; /* Is Msp Callbacks registred      */ 
}AUDIO_OUT_Ctx_t;

typedef struct
{
  /* Filter parameters */
  DFSDM_Filter_TypeDef   *FilterInstance;
  uint32_t               RegularTrigger;
  uint32_t               SincOrder;   
  uint32_t               Oversampling;
  /* Channel parameters */
  DFSDM_Channel_TypeDef *ChannelInstance;
  uint32_t              DigitalMicPins;
  uint32_t              DigitalMicType;
  uint32_t              Channel4Filter;
  uint32_t              ClockDivider;
  uint32_t              RightBitShift; 
  FunctionalState       DmaMode;
  uint32_t              Multiplexer;
  FunctionalState       Activation;
}MX_DFSDM_Config;  

#endif /* #if (USE_AUDIO_IN == 1) */

/**
  * @}
  */ 

/** @defgroup SENSORTILEBOX_AUDIO_Exported_Constants SENSORTILEBOX_AUDIO Exported Constants
  * @{
  */

/* AUDIO FREQUENCY */
#ifndef AUDIO_FREQUENCY_192K
#define AUDIO_FREQUENCY_192K     (uint32_t)192000U
#endif
#ifndef AUDIO_FREQUENCY_176K  
#define AUDIO_FREQUENCY_176K     (uint32_t)176400U
#endif
#ifndef AUDIO_FREQUENCY_96K
#define AUDIO_FREQUENCY_96K       (uint32_t)96000U
#endif
#ifndef AUDIO_FREQUENCY_88K
#define AUDIO_FREQUENCY_88K       (uint32_t)88200U
#endif
#ifndef AUDIO_FREQUENCY_48K
#define AUDIO_FREQUENCY_48K       (uint32_t)48000U
#endif
#ifndef AUDIO_FREQUENCY_44K  
#define AUDIO_FREQUENCY_44K       (uint32_t)44100U
#endif
#ifndef AUDIO_FREQUENCY_32K
#define AUDIO_FREQUENCY_32K       (uint32_t)32000U
#endif
#ifndef AUDIO_FREQUENCY_22K
#define AUDIO_FREQUENCY_22K       (uint32_t)22050U
#endif
#ifndef AUDIO_FREQUENCY_16K
#define AUDIO_FREQUENCY_16K       (uint32_t)16000U
#endif
#ifndef AUDIO_FREQUENCY_11K
#define AUDIO_FREQUENCY_11K       (uint32_t)11025U
#endif
#ifndef AUDIO_FREQUENCY_8K
#define AUDIO_FREQUENCY_8K         (uint32_t)8000U 
#endif
   
/* AUDIO RESOLUTION */   
#ifndef AUDIO_RESOLUTION_16b
#define AUDIO_RESOLUTION_16b                16U
#endif
#ifndef AUDIO_RESOLUTION_24b
#define AUDIO_RESOLUTION_24b                24U
#endif
#ifndef AUDIO_RESOLUTION_32b
#define AUDIO_RESOLUTION_32b                32U
#endif

/* Buffer size defines*/
#define CHANNEL_DEMUX_MASK                    	0x55
    
#define MAX_CH_NUMBER           (4)
#define MAX_FS                  (192000)
#define MAX_SAMPLES_PER_CH      ((MAX_FS/1000)*2)	

#define MAX_MIC_FREQ                 	  3072  /*KHz*/
#define MAX_AUDIO_IN_CHANNEL_NBR_PER_IF   2 
#define MAX_AUDIO_IN_CHANNEL_NBR_TOTAL    4 

/*Number of millisecond of audio at each DMA interrupt*/
#define N_MS_PER_INTERRUPT               (1)
    
/*BSP internal buffer size in half words (16 bits)*/  
#define PDM_INTERNAL_BUFFER_SIZE_I2S          ((MAX_MIC_FREQ / 8) * MAX_AUDIO_IN_CHANNEL_NBR_PER_IF * N_MS_PER_INTERRUPT)
#if MAX_AUDIO_IN_CHANNEL_NBR_TOTAL > 2
#define PDM_INTERNAL_BUFFER_SIZE_SPI          ((MAX_MIC_FREQ / 8) * MAX_AUDIO_IN_CHANNEL_NBR_PER_IF * N_MS_PER_INTERRUPT)
#else
#define PDM_INTERNAL_BUFFER_SIZE_SPI          1
#endif

/* Audio In states */
#define AUDIO_IN_STATE_RESET               0U
#define AUDIO_IN_STATE_RECORDING           1U
#define AUDIO_IN_STATE_STOP                2U
#define AUDIO_IN_STATE_PAUSE               3U

/* Audio In instances available in SENSORTILEBOX */
#define AUDIO_IN_INSTANCES_NBR 1U

#endif /* AUDIO_SHARED_TYPES */

/*------------------------------------------------------------------------------
                        AMic_OnBoard defines parameters
------------------------------------------------------------------------------*/
#if (AUDIO_SAMPLING_FREQUENCY == 8000)
#define DECIMATION_RATIO_DFSDM 384
#define SINC_ORDER DFSDM_FILTER_SINC2_ORDER
#define RBITSHIFT 9
#define RSHFT_ADC ADC_RIGHTBITSHIFT_NONE
#define RSHFT_ADC_OFFST 0
#define ADC_CLOCK (uint32_t)(3072000)
#define ADC_SAMPLE_TIME ADC_SAMPLETIME_12CYCLES_5

#elif (AUDIO_SAMPLING_FREQUENCY == 16000)
#define DECIMATION_RATIO_DFSDM 192
#define SINC_ORDER DFSDM_FILTER_SINC2_ORDER
#define RBITSHIFT 7
#define RSHFT_ADC ADC_RIGHTBITSHIFT_NONE
#define RSHFT_ADC_OFFST 0
#define ADC_CLOCK (uint32_t)(3072000)
#define ADC_SAMPLE_TIME ADC_SAMPLETIME_12CYCLES_5

#elif (AUDIO_SAMPLING_FREQUENCY == 32000)
#define DECIMATION_RATIO_DFSDM 96
#define SINC_ORDER DFSDM_FILTER_SINC2_ORDER
#define RBITSHIFT 5
#define RSHFT_ADC ADC_RIGHTBITSHIFT_NONE
#define RSHFT_ADC_OFFST 0
#define ADC_CLOCK (uint32_t)(3072000)
#define ADC_SAMPLE_TIME ADC_SAMPLETIME_12CYCLES_5

#elif (AUDIO_SAMPLING_FREQUENCY == 48000)
#define DECIMATION_RATIO_DFSDM 64
#define SINC_ORDER DFSDM_FILTER_SINC3_ORDER
#define RBITSHIFT 10
#define RSHFT_ADC ADC_RIGHTBITSHIFT_NONE
#define RSHFT_ADC_OFFST 0
#define ADC_CLOCK (uint32_t)(3072000)
#define ADC_SAMPLE_TIME ADC_SAMPLETIME_12CYCLES_5

#elif (AUDIO_SAMPLING_FREQUENCY == 96000)
#define DECIMATION_RATIO_DFSDM 32
#define SINC_ORDER DFSDM_FILTER_SINC3_ORDER
#define RBITSHIFT 7
#define RSHFT_ADC ADC_RIGHTBITSHIFT_NONE
#define RSHFT_ADC_OFFST 0
#define ADC_CLOCK (uint32_t)(3072000)
#define ADC_SAMPLE_TIME ADC_SAMPLETIME_12CYCLES_5

#elif (AUDIO_SAMPLING_FREQUENCY == 192000)
#define DECIMATION_RATIO_DFSDM 16
#define SINC_ORDER DFSDM_FILTER_SINC4_ORDER
#define RBITSHIFT 8
#define RSHFT_ADC ADC_RIGHTBITSHIFT_NONE
#define RSHFT_ADC_OFFST 0
#define ADC_CLOCK (uint32_t)(3072000)
#define ADC_SAMPLE_TIME ADC_SAMPLETIME_12CYCLES_5
#endif

#define AMIC_ONBOARD_CHANNEL                   DFSDM1_Channel0
#define AMIC_ONBOARD_FILTER                    DFSDM1_Filter1

#define DMA_REQUEST_ANALOG                     DMA_REQUEST_DFSDM1_FLT1
#define DFSDM_DMA_ANALOG                       DMA1_Channel4
#define DFSDM_DMA_ANALOG_IRQn                  DMA1_Channel4_IRQn

#define BSP_AUDIO_IN_INSTANCE 1U   /* Audio instance */

#if (USE_AUDIO_IN == 1)

/* Audio In devices */ 
#define AMIC_ONBOARD 0x100U


/**
  * @}
  */
   
/** @defgroup SENSORTILEBOX_AUDIO_Exported_Macros SENSORTILEBOX_AUDIO Exported Macros
  * @{
  */
#define POS_VAL(VAL)                  (POSITION_VAL(VAL) - 4)
#define VOLUME_OUT_CONVERT(Volume)    (((Volume) > 100)? 63:((uint8_t)(((Volume) * 63) / 100)))
#define VOLUME_IN_CONVERT(Volume)     (((Volume) >= 100)? 239:((uint8_t)(((Volume) * 239) / 100)))   
/**
  * @}
  */ 
/** @addtogroup SENSORTILEBOX_AUDIO_Exported_Variables
  * @{
  */
/* Recording context */
extern AUDIO_IN_Ctx_t AudioInCtx;
/**
  * @}
  */

/** @defgroup SENSORTILEBOX_AUDIO_IN_Exported_Functions SENSORTILEBOX_AUDIO_IN Exported Functions
  * @{
  */
int32_t BSP_AUDIO_IN_Init(uint32_t Instance, BSP_AUDIO_Init_t* AudioInit);    
int32_t BSP_AUDIO_IN_DeInit(uint32_t Instance);
int32_t BSP_AUDIO_IN_Record(uint32_t Instance, uint8_t* pBuf, uint32_t NbrOfBytes);
int32_t BSP_AUDIO_IN_Stop(uint32_t Instance);
int32_t BSP_AUDIO_IN_Pause(uint32_t Instance);
int32_t BSP_AUDIO_IN_Resume(uint32_t Instance);

int32_t BSP_AUDIO_IN_RecordChannels(uint32_t Instance, uint8_t **pBuf, uint32_t NbrOfBytes);
int32_t BSP_AUDIO_IN_StopChannels(uint32_t Instance, uint32_t Device);
int32_t BSP_AUDIO_IN_PauseChannels(uint32_t Instance, uint32_t Device);
int32_t BSP_AUDIO_IN_ResumeChannels(uint32_t Instance, uint32_t Device);

int32_t BSP_AUDIO_IN_SetDevice(uint32_t Instance, uint32_t Device);
int32_t BSP_AUDIO_IN_GetDevice(uint32_t Instance, uint32_t *Device);
int32_t BSP_AUDIO_IN_SetSampleRate(uint32_t Instance, uint32_t SampleRate);
int32_t BSP_AUDIO_IN_GetSampleRate(uint32_t Instance, uint32_t *SampleRate);                 
int32_t BSP_AUDIO_IN_SetBitsPerSample(uint32_t Instance, uint32_t BitsPerSample);
int32_t BSP_AUDIO_IN_GetBitsPerSample(uint32_t Instance, uint32_t *BitsPerSample);                
int32_t BSP_AUDIO_IN_SetChannelsNbr(uint32_t Instance, uint32_t ChannelNbr);
int32_t BSP_AUDIO_IN_GetChannelsNbr(uint32_t Instance, uint32_t *ChannelNbr);
int32_t BSP_AUDIO_IN_SetVolume(uint32_t Instance, uint32_t Volume);
int32_t BSP_AUDIO_IN_GetVolume(uint32_t Instance, uint32_t *Volume);
int32_t BSP_AUDIO_IN_GetState(uint32_t Instance, uint32_t *State);

/* User Callbacks: user has to implement these functions in his code if they are needed. */
/* This function should be implemented by the user application.
   It is called into this driver when the current buffer is filled to prepare the next
   buffer pointer and its size. */
void BSP_AUDIO_IN_TransferComplete_CallBack(uint32_t Instance);
void BSP_AUDIO_IN_HalfTransfer_CallBack(uint32_t Instance);

/* This function is called when an Interrupt due to transfer error on or peripheral
   error occurs. */
void BSP_AUDIO_IN_Error_CallBack(uint32_t Instance);


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
#endif /* #if (USE_AUDIO_IN == 1) */

#ifdef __cplusplus
}
#endif

#endif /* __SENSORTILEBOX_AUDIO_H */
