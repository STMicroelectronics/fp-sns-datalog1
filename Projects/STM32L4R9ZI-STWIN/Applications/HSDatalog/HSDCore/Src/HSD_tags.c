/**
  ******************************************************************************
  * @file    HSD_tags.c
  * @author  SRA - MCD
  *
  *
  * @brief   High Speed DataLog Json Interpreter
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
#include "HSDCore.h"
#include "HSD_tags.h"
#include "string.h"
#include "stdio.h"
#include "cmsis_os.h"
#include "sensors_manager.h"

/* Private variables ---------------------------------------------------------*/

const char HSD_HwTagPinNames[HSD_TAGS_MAX_HW_CLASSES][HSD_TAGS_LABEL_LENGTH] =
{
  HSD_TAG_PIN0_NAME,
  HSD_TAG_PIN1_NAME,
  HSD_TAG_PIN2_NAME,
  HSD_TAG_PIN3_NAME,
  HSD_TAG_PIN4_NAME
};

static volatile uint32_t tag_counter = 0;

const uint16_t TAG_PIN[HSD_TAGS_MAX_HW_CLASSES] =
{
  STMOD_PIN_7_PIN,
  STMOD_PIN_8_PIN,
  STMOD_PIN_9_PIN,
  STMOD_PIN_10_PIN,
  STMOD_PIN_11_PIN
};

GPIO_TypeDef *TAG_PIN_PORT[HSD_TAGS_MAX_HW_CLASSES] =
{
  STMOD_PIN_7_GPIO_PORT,
  STMOD_PIN_8_GPIO_PORT,
  STMOD_PIN_9_GPIO_PORT,
  STMOD_PIN_10_GPIO_PORT,
  STMOD_PIN_11_GPIO_PORT
};

uint8_t TAG_Pin_Status[HSD_TAGS_MAX_HW_CLASSES] =
{
  0xFF,
  0xFF,
  0xFF,
  0xFF,
  0xFF
};
uint8_t TAG_Init = 0;

/* Private function prototypes -----------------------------------------------*/
static void HSD_TAGS_TimerCallback(void const *arg);
void HSD_TAGS_PIN_CheckStatusAndAdd(void);

static HSD_Tags_Enable_t HSD_TAGS_get_tag_enabled(COM_Device_t *device, uint8_t class_id);

/* Declare mail queue for tag list */
osMailQDef(tags_pool_q, HSD_TAGS_MAX_PER_ACQUISITION, HSD_Tags_t);
osMailQId(tags_pool_q_id);

osTimerDef(hw_tags, HSD_TAGS_TimerCallback); /* when the timer expires, the function toggle_power is called */
osTimerId osTim_Hw_Tags_id;

/* Public function -----------------------------------------------------------*/

void HSD_TAGS_timer_start(void)
{
  uint32_t i;
  for (i = 0; i < HSD_TAGS_MAX_HW_CLASSES; i++)
  {
    TAG_Pin_Status[i] = 0xFF;
  }

  TAG_Init = 1;
  osTimerStart(osTim_Hw_Tags_id, HSD_TAGS_TIMER_PERIOD_MS);
}

void HSD_TAGS_timer_stop(void)
{
  osTimerStop(osTim_Hw_Tags_id);
}

void HSD_TAGS_init(COM_Device_t *device)
{
  uint8_t i;
  tag_counter = 0;

  for (i = 0; i < HSD_TAGS_MAX_SW_CLASSES; i++)
  {
    sprintf(device->tagList.HSD_SwTagClasses[i], HSD_TAGS_DEFAULT_SW, i);
  }
  for (i = 0; i < HSD_TAGS_MAX_HW_CLASSES; i++)
  {
    sprintf(device->tagList.HwTag[i].label, HSD_TAGS_DEFAULT_HW, i);
  }

  tags_pool_q_id = osMailCreate(osMailQ(tags_pool_q), NULL);

  osTim_Hw_Tags_id = osTimerCreate(osTimer(hw_tags), osTimerPeriodic, (void *) 0);

#if (HSD_TASK_DEBUG_PINS_ENABLE == 0)
  HSD_TAGS_PIN_Init_All();
#endif /* (HSD_TASK_DEBUG_PINS_ENABLE == 0) */
}

static void HSD_TAGS_TimerCallback(void const *arg)
{
  HSD_TAGS_PIN_CheckStatusAndAdd();
}

void HSD_TAGS_reset(void)
{
  HSD_Tags_Type_t type;
  uint8_t class_id;
  HSD_Tags_Enable_t enable;
  double time_stamp;

  while (HSD_TAGS_get_tag(&type, &class_id, &enable, &time_stamp) == 0);
}

int32_t HSD_TAGS_get_tag(HSD_Tags_Type_t *type, uint8_t *class_id, HSD_Tags_Enable_t *enable, double *time_stamp)
{
  osEvent event;
  HSD_Tags_t *tmp;

  event = osMailGet(tags_pool_q_id, 0);

  if (event.status != osEventMail)
  {
    return -1;
  }

  tmp = (HSD_Tags_t *) event.value.p;

  *type = (HSD_Tags_Type_t) tmp->tag.class_type;
  *class_id = tmp->tag.class_id;
  *enable = (HSD_Tags_Enable_t) tmp->tag.enable;
  *time_stamp = tmp->timeStamp;

  osMailFree(tags_pool_q_id, tmp);

  tag_counter--;

  return 0;
}

int32_t HSD_TAGS_add_tag(HSD_Tags_Type_t type, uint8_t class_id, HSD_Tags_Enable_t enable, double time_stamp)
{
  HSD_Tags_t *tag_p;

  tag_p = osMailAlloc(tags_pool_q_id, 0);

  /* no space left */
  if (tag_p == NULL)
  {
    return -1;
  }

  tag_p->tag.class_type = type;
  tag_p->tag.class_id = class_id;
  tag_p->tag.enable = enable;
  tag_p->timeStamp = time_stamp;

  if (osMailPut(tags_pool_q_id, tag_p) != osOK)
  {
    return -1;
  }

  /* Todo: Manage critical section */
  tag_counter++;
  return 0;
}

char *HSD_TAGS_get_tag_label(COM_Device_t *device, HSD_Tags_Type_t type, uint8_t class_id)
{
  char *label;

  if (type == HSD_TAGS_Type_Sw)
  {
    if (class_id < HSD_TAGS_MAX_SW_CLASSES)
    {
      label = device->tagList.HSD_SwTagClasses[class_id];
    }
    else
    {
      return NULL;
    }
  }
  else if (type == HSD_TAGS_Type_Hw)
  {
    if (class_id < HSD_TAGS_MAX_HW_CLASSES)
    {
      label = device->tagList.HwTag[class_id].label;
    }
    else
    {
      return NULL;
    }
  }
  else
  {
    return NULL;
  }

  return label;
}

int32_t HSD_TAGS_set_tag_label(COM_Device_t *device, HSD_Tags_Type_t type, uint8_t class_id, const char *label)
{
  uint8_t label_len;

  label_len = strlen(label);
  label_len = (label_len < HSD_TAGS_LABEL_LENGTH) ? label_len : (HSD_TAGS_LABEL_LENGTH - 1); /* truncate */

  if (type == HSD_TAGS_Type_Sw)
  {
    if (class_id < HSD_TAGS_MAX_SW_CLASSES)
    {
      memcpy(device->tagList.HSD_SwTagClasses[class_id], label, label_len);
      device->tagList.HSD_SwTagClasses[class_id][label_len] = '\0';
    }
    else
    {
      return -1;
    }
  }
  else if (type == HSD_TAGS_Type_Hw)
  {
    if (class_id < HSD_TAGS_MAX_HW_CLASSES)
    {
      memcpy(device->tagList.HwTag[class_id].label, label, label_len);
      device->tagList.HwTag[class_id].label[label_len] = '\0';
    }
    else
    {
      return -1;
    }
  }
  else
  {
    return -1;
  }

  return 0;
}

static HSD_Tags_Enable_t HSD_TAGS_get_tag_enabled(COM_Device_t *device, uint8_t class_id)
{
  HSD_Tags_Enable_t tagEnabled;
  if (class_id < HSD_TAGS_MAX_HW_CLASSES)
  {
    tagEnabled = (HSD_Tags_Enable_t)(device->tagList.HwTag[class_id].enabled);
  }
  else
  {
    tagEnabled = HSD_TAGS_Disable;
  }
  return tagEnabled;
}

int32_t HSD_TAGS_set_tag_enabled(COM_Device_t *device, uint8_t class_id, HSD_Tags_Enable_t enable)
{
  device->tagList.HwTag[class_id].enabled = enable;
  return 0;
}

void HSD_TAGS_PIN_Init_All(void)
{
  HSD_TAGS_PIN_Init(TAG_PIN0);
  HSD_TAGS_PIN_Init(TAG_PIN1);
  HSD_TAGS_PIN_Init(TAG_PIN2);
  HSD_TAGS_PIN_Init(TAG_PIN3);
  HSD_TAGS_PIN_Init(TAG_PIN4);
}

void HSD_TAGS_PIN_Init(Tag_Pin_TypeDef Pin)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable the GPIO clock */
  TAG_PINx_GPIO_CLK_ENABLE(Pin);

  /* Configure the GPIO pin */
  GPIO_InitStructure.Pin = TAG_PIN[Pin];
  GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_InitStructure.Speed = GPIO_SPEED_FAST;

  HAL_GPIO_Init(TAG_PIN_PORT[Pin], &GPIO_InitStructure);
}

void HSD_TAGS_PIN_DeInit(Tag_Pin_TypeDef Pin)
{
  HAL_GPIO_DeInit(TAG_PIN_PORT[Pin], TAG_PIN[Pin]);
  TAG_PINx_GPIO_CLK_DISABLE(Pin);
}

void HSD_TAGS_PIN_CheckStatusAndAdd(void)
{
  uint32_t i = 0;
  uint8_t newStatus;
  double timestamp;
  COM_TagList_t *pTagList = COM_GetTagList();

  if (TAG_Init != 1)
  {
    for (i = 0; i < HSD_TAGS_MAX_HW_CLASSES; i++)
    {
      if (pTagList->HwTag[i].enabled)
      {
        newStatus = (uint8_t) HAL_GPIO_ReadPin(TAG_PIN_PORT[i], TAG_PIN[i]);

        if (newStatus != TAG_Pin_Status[i])
        {
          TAG_Pin_Status[i] = newStatus;
          timestamp = SM_GetTimeStamp();
          HSD_TAGS_add_tag(HSD_TAGS_Type_Hw, i, (HSD_Tags_Enable_t) newStatus, timestamp);
        }
      }
    }
  }
  else
  {
    for (i = 0; i < HSD_TAGS_MAX_HW_CLASSES; i++)
    {
      if (pTagList->HwTag[i].enabled)
      {
        newStatus = (uint8_t) HAL_GPIO_ReadPin(TAG_PIN_PORT[i], TAG_PIN[i]);

        if (newStatus != TAG_Pin_Status[i])
        {
          TAG_Pin_Status[i] = newStatus;
        }
      }
    }
    TAG_Init = 0;
  }
}

void update_tagList(COM_Device_t *oldTagDevice, COM_Device_t *newTagDevice)
{
  char *oldTag;
  char *newTag;

  for (uint8_t id = 0; id < HSD_TAGS_MAX_HW_CLASSES; id++)
  {
    oldTag = HSD_TAGS_get_tag_label(oldTagDevice, HSD_TAGS_Type_Hw, id);
    newTag = HSD_TAGS_get_tag_label(newTagDevice, HSD_TAGS_Type_Hw, id);
    if (strcmp(oldTag, newTag))
    {
      HSD_TAGS_set_tag_label(oldTagDevice, HSD_TAGS_Type_Hw, id, newTag);
    }

    HSD_Tags_Enable_t oldEnabled = HSD_TAGS_get_tag_enabled(oldTagDevice, id);
    HSD_Tags_Enable_t newEnabled = HSD_TAGS_get_tag_enabled(newTagDevice, id);
    if (oldEnabled != newEnabled)
    {
      HSD_TAGS_set_tag_enabled(oldTagDevice, id, newEnabled);
    }
  }

  for (uint8_t id = 0; id < HSD_TAGS_MAX_SW_CLASSES; id++)
  {
    oldTag = HSD_TAGS_get_tag_label(oldTagDevice, HSD_TAGS_Type_Sw, id);
    newTag = HSD_TAGS_get_tag_label(newTagDevice, HSD_TAGS_Type_Sw, id);
    if (strcmp(oldTag, newTag))
    {
      HSD_TAGS_set_tag_label(oldTagDevice, HSD_TAGS_Type_Sw, id, newTag);
    }
  }
}

