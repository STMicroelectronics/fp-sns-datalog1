/**
  ******************************************************************************
  * @file    HSD_tags.h
  * @author  SRA - MCD
  *
  *
  * @brief   This file contains all the functions prototypes for the main.c
  *          file.
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
#ifndef __HSD_TAGS_H
#define __HSD_TAGS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "device_description.h"

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  HSD_TAGS_Type_Sw = 0,
  HSD_TAGS_Type_Hw,
} HSD_Tags_Type_t;

typedef enum
{
  HSD_TAGS_Disable = 0,
  HSD_TAGS_Enable,
} HSD_Tags_Enable_t;

typedef struct
{
  uint8_t enable :1;
  uint8_t class_type :2;
  uint8_t class_id :5;
} HSD_Tag_Field_t;

typedef struct
{
  double timeStamp;
  HSD_Tag_Field_t tag;
} HSD_Tags_t;

typedef enum
{
  TAG_PIN0 = 0,
  TAG_PIN1,
  TAG_PIN2,
  TAG_PIN3,
  TAG_PIN4
} Tag_Pin_TypeDef;

/* Exported constants --------------------------------------------------------*/
#define HSD_TAGS_DEFAULT_SW "SW_TAG_%d"
#define HSD_TAGS_DEFAULT_HW "HW_TAG_%d"

#define HSD_TAGS_TIMER_PERIOD_MS  (200UL)

/* private -> must be a divider of HSD_TAGS_MAX_PER_ACQUISITION */
/* #define HSD_TAGS_CHUNCK_SIZE (10U) */

#define HSD_TAG_PIN0_NAME "STMOD_7"
#define HSD_TAG_PIN1_NAME "STMOD_8"
#define HSD_TAG_PIN2_NAME "STMOD_9"
#define HSD_TAG_PIN3_NAME "STMOD_10"
#define HSD_TAG_PIN4_NAME "STMOD_11"

#define STMOD_PIN_7_PIN                                 GPIO_PIN_12
#define STMOD_PIN_7_GPIO_PORT                           GPIOD
#define STMOD_PIN_7_GPIO_CLK_ENABLE()                   __HAL_RCC_GPIOD_CLK_ENABLE()
#define STMOD_PIN_7_GPIO_CLK_DISABLE()                  __HAL_RCC_GPIOD_CLK_DISABLE()

#define STMOD_PIN_8_PIN                                 GPIO_PIN_15
#define STMOD_PIN_8_GPIO_PORT                           GPIOB
#define STMOD_PIN_8_GPIO_CLK_ENABLE()                   __HAL_RCC_GPIOB_CLK_ENABLE()
#define STMOD_PIN_8_GPIO_CLK_DISABLE()                  __HAL_RCC_GPIOB_CLK_DISABLE()

#define STMOD_PIN_9_PIN                                 GPIO_PIN_2
#define STMOD_PIN_9_GPIO_PORT                           GPIOC
#define STMOD_PIN_9_GPIO_CLK_ENABLE()                   __HAL_RCC_GPIOC_CLK_ENABLE()
#define STMOD_PIN_9_GPIO_CLK_DISABLE()                  __HAL_RCC_GPIOC_CLK_DISABLE()

#define STMOD_PIN_10_PIN                                GPIO_PIN_13
#define STMOD_PIN_10_GPIO_PORT                          GPIOD
#define STMOD_PIN_10_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOD_CLK_ENABLE()
#define STMOD_PIN_10_GPIO_CLK_DISABLE()                 __HAL_RCC_GPIOD_CLK_DISABLE()

#define STMOD_PIN_11_PIN                                GPIO_PIN_5
#define STMOD_PIN_11_GPIO_PORT                          GPIOC
#define STMOD_PIN_11_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOC_CLK_ENABLE()
#define STMOD_PIN_11_GPIO_CLK_DISABLE()                 __HAL_RCC_GPIOC_CLK_DISABLE()

#define TAG_PINx_GPIO_CLK_ENABLE(__TAG_PIN__)   do{if((__TAG_PIN__) == TAG_PIN0 ) \
                                                                          { STMOD_PIN_7_GPIO_CLK_ENABLE(); } else \
                                                    if((__TAG_PIN__) == TAG_PIN1 ) \
                                                                          { STMOD_PIN_8_GPIO_CLK_ENABLE(); } else \
                                                    if((__TAG_PIN__) == TAG_PIN2 ) \
                                                                          { STMOD_PIN_9_GPIO_CLK_ENABLE(); } else \
                                                    if((__TAG_PIN__) == TAG_PIN3 ) \
                                                                          { STMOD_PIN_10_GPIO_CLK_ENABLE(); } else \
                                                    if((__TAG_PIN__) == TAG_PIN4 ) \
                                                                          { STMOD_PIN_11_GPIO_CLK_ENABLE(); } } \
                                                                            while(0)

#define TAG_PINx_GPIO_CLK_DISABLE(__TAG_PIN__)  do{if((__TAG_PIN__) == TAG_PIN0 ) \
                                                                            { STMOD_PIN_7_GPIO_CLK_DISABLE(); } else \
                                                      if((__TAG_PIN__) == TAG_PIN1 ) \
                                                                            { STMOD_PIN_8_GPIO_CLK_DISABLE(); } else \
                                                      if((__TAG_PIN__) == TAG_PIN2 ) \
                                                                            { STMOD_PIN_9_GPIO_CLK_DISABLE(); } else \
                                                      if((__TAG_PIN__) == TAG_PIN3 ) \
                                                                            { STMOD_PIN_10_GPIO_CLK_DISABLE(); } else \
                                                      if((__TAG_PIN__) == TAG_PIN4 ) \
                                                                            { STMOD_PIN_11_GPIO_CLK_DISABLE(); } } \
                                                                              while(0)

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void HSD_TAGS_init(COM_Device_t *device);
int32_t HSD_TAGS_set_tag_label(COM_Device_t *device, HSD_Tags_Type_t type, uint8_t class_id, const char *label);
int32_t HSD_TAGS_set_tag_enabled(COM_Device_t *device, uint8_t class_id, HSD_Tags_Enable_t enable);
char *HSD_TAGS_get_tag_label(COM_Device_t *device, HSD_Tags_Type_t type, uint8_t class_id);

int32_t HSD_TAGS_add_tag(HSD_Tags_Type_t type, uint8_t class_id, HSD_Tags_Enable_t enable, double time_stamp);
int32_t HSD_TAGS_get_tag(HSD_Tags_Type_t *type, uint8_t *class_id, HSD_Tags_Enable_t *enable, double *time_stamp);

void HSD_TAGS_timer_start(void);
void HSD_TAGS_timer_stop(void);

void HSD_TAGS_PIN_Init_All(void);
void HSD_TAGS_PIN_Init(Tag_Pin_TypeDef Pin);
void HSD_TAGS_PIN_DeInit(Tag_Pin_TypeDef Pin);

void update_tagList(COM_Device_t *oldTagDevice, COM_Device_t *newTagDevice);

#ifdef __cplusplus
}
#endif

#endif /* __HSD_TAGS_H */

