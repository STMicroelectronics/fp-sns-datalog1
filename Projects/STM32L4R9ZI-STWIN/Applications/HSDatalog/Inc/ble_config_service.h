/**
  ******************************************************************************
  * @file    ble_config_service.h
  * @author  SRA
  *
  *
  * @brief   Sensors services APIs
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
#ifndef _BLE_CONFIG_SERVICE_H_
#define _BLE_CONFIG_SERVICE_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <stdlib.h>


/* Exported functions ------------------------------------------------------- */

void ble_interface_init(void);
extern void HCI_Event_CB(void *pckt);


/* Exported constants --------------------------------------------------------*/

#define NAME_HSD 'D','T','L','G',HSD_VERSION_MAJOR,HSD_VERSION_MINOR,HSD_VERSION_PATCH



#ifdef __cplusplus
}
#endif

#endif /* _BLE_CONFIG_SERVICE_H_ */

