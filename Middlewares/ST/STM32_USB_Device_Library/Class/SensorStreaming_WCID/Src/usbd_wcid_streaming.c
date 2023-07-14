/**
  ******************************************************************************
  * @file    usbd_wcid_streaming.c
  * @author  SRA
  * @brief   usbd wcid streaming class source code
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

/* Includes ------------------------------------------------------------------*/
#include "usbd_wcid_streaming.h"
#include "usbd_desc.h"
#include "usbd_ctlreq.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_WCID_STREAMING
  * @brief usbd core module
  * @{
  */

/** @defgroup USBD_WCID_STREAMING_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */

/** @defgroup USBD_WCID_STREAMING_Private_Defines
  * @{
  */
/**
  * @}
  */

/** @defgroup USBD_WCID_STREAMING_Private_Macros
  * @{
  */

/**
  * @}
  */


/** @defgroup USBD_WCID_STREAMING_Private_FunctionPrototypes
  * @{
  */

static uint8_t USBD_WCID_STREAMING_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_WCID_STREAMING_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_WCID_STREAMING_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t USBD_WCID_STREAMING_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_WCID_STREAMING_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_WCID_STREAMING_SOF(USBD_HandleTypeDef *pdev);
static uint8_t USBD_WCID_STREAMING_EP0_RxReady(USBD_HandleTypeDef *pdev);
static uint8_t *USBD_WCID_STREAMING_GetFSCfgDesc(uint16_t *length);
static uint8_t *USBD_WCID_STREAMING_GetHSCfgDesc(uint16_t *length);
static uint8_t *USBD_WCID_STREAMING_GetOtherSpeedCfgDesc(uint16_t *length);
static uint8_t *USBD_WCID_STREAMING_GetDeviceQualifierDescriptor(uint16_t *length);
static uint8_t *USBD_WCID_STREAMING_OSCompIDDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *USBD_WCID_STREAMING_OSPropertiesFeatureDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *USBD_WCID_STREAMING_GetUsrStrDescriptor(USBD_HandleTypeDef *pdev, uint8_t index,  uint16_t *length);
static uint8_t USBD_WCID_STREAMING_SetTxBuffer(USBD_HandleTypeDef   *pdev, uint8_t  *pbuff, uint16_t length);
static USBD_StatusTypeDef USBD_WCID_STREAMING_TransmitPacket(USBD_HandleTypeDef *pdev, uint8_t epNumber);
static USBD_StatusTypeDef USBD_WCID_STREAMING_ReceivePacket(USBD_HandleTypeDef *pdev);
static USBD_StatusTypeDef  USBD_VendDevReq(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);

/**
  * @}
  */

/** @defgroup USBD_WCID_STREAMING_Private_Variables
  * @{
  */

/* Microsoft Extended Properties Feature Descriptor Typedef*/
typedef struct
{
  // Header
  uint32_t  dwLength;
  uint16_t  bcdVersion;
  uint16_t  wIndex;
  uint16_t  wCount;
  // Custom Property Section 1
  uint32_t  dwSize;
  uint32_t  dwPropertyDataType;
  uint16_t  wPropertyNameLength;
  uint16_t  bPropertyName[20];
  uint32_t  dwPropertyDataLength;
  uint16_t  bPropertyData[39];
} USBD_ExtPropertiesDescStruct;

/* Microsoft Extended Properties Feature Descriptor */
static USBD_ExtPropertiesDescStruct USBD_ExtPropertiesDesc = { sizeof(USBD_ExtPropertiesDesc), 0x0100, 0x0005, 0x0001, \
                                                               0x00000084, 0x00000001, \
                                                               0x0028,     {'D', 'e', 'v', 'i', 'c', 'e', 'I', 'n', 't', 'e', 'r', 'f', 'a', 'c', 'e', 'G', 'U', 'I', 'D', 0}, \
                                                               0x0000004E, {'{', 'F', '7', '0', '2', '4', '2', 'C', '7', '-', 'F', 'B', '2', '5', '-', '4', '4', '3', 'B', \
                                                                            '-', '9', 'E', '7', 'E', '-', 'A', '4', '2', '6', '0', 'F', '3', '7', '3', '9', '8', '2', '}', 0
                                                                           }
                                                             };

/* Microsoft OS String Descriptor */
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma data_alignment=4
#endif
__ALIGN_BEGIN static uint8_t USBD_OSStringDesc[] __ALIGN_END =
{
  0x12,                       /* bLength */
  USB_DESC_TYPE_STRING,       /* bDescriptorType */
  0x4D,
  0x00,
  0x53,
  0x00,
  /*Signature MSFT100*/
  0x46,
  0x00,
  0x54,
  0x00,
  0x31,
  0x00,
  0x30,
  0x00,
  0x30,
  0x00,
  /******end sig*****/
  0x07,                       /* Vendor Code */
  0x00,                       /*Padding*/
}; /* Microsoft OS String Descriptor */

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_WCID_STREAMING_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};

/* OS ID feature Descriptor */
static uint8_t OS_IDFeatureDescriptor[] =
{
  0x28,  /*LENGTH*/
  0x00,  /* "    */
  0x00,  /* "    */
  0x00,  /* "    */
  0x00,  /*Version 0.1*/
  0x01,
  0x04,  /*Compatibility ID Descriptor index (0x0004) */
  0x00,
  0x01,  /*N of sections*/
  0x00,  /* 7 bytes reserved */
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00, /*Interface number (0) */
  0x01,  /*reserved*/
  /* Compatible ID: "WINUSB\0\"*/
  0x57,
  0x49,
  0x4E,
  0x55,
  0x53,
  0x42,
  0x00,
  0x00,
  /* Sub Compatible ID: Unused*/
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  /*reserved*/
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00
};

/* USB WCID STREAMING interface class callbacks structure */
USBD_ClassTypeDef  USBD_WCID_STREAMING =
{
  USBD_WCID_STREAMING_Init,
  USBD_WCID_STREAMING_DeInit,
  USBD_WCID_STREAMING_Setup,
  NULL,
  USBD_WCID_STREAMING_EP0_RxReady,
  USBD_WCID_STREAMING_DataIn,
  USBD_WCID_STREAMING_DataOut,
  USBD_WCID_STREAMING_SOF,
  NULL,
  NULL,
  USBD_WCID_STREAMING_GetHSCfgDesc,
  USBD_WCID_STREAMING_GetFSCfgDesc,
  USBD_WCID_STREAMING_GetOtherSpeedCfgDesc,
  USBD_WCID_STREAMING_GetDeviceQualifierDescriptor,
  USBD_WCID_STREAMING_GetUsrStrDescriptor,
};

/* USB WCID STREAMING device Configuration Descriptor (Fs) */
__ALIGN_BEGIN static uint8_t USBD_WCID_STREAMING_CfgHSDesc[USB_SS_WCID_CONFIG_DESC_SIZ] __ALIGN_END =
{
  /*Configuration Descriptor*/
  0x09,   /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION,      /* bDescriptorType: Configuration */
  USB_SS_WCID_CONFIG_DESC_SIZ,                /* wTotalLength:no of returned bytes */
  0x00,
  0x01,   /* bNumInterfaces: Total 1 interface */
  0x01,   /* bConfigurationValue: Configuration value */
  0x00,   /* iConfiguration: Index of string descriptor describing the configuration */
  0xC0,   /* bmAttributes: self powered */
  0x32,   /* MaxPower 0 mA */

  /*---------------------------------------------------------------------------*/

  /*Interface Descriptor */
  0x09,   /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: Interface */
  /* Interface descriptor type */
  0x00,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  N_IN_ENDPOINTS + 1,   /* bNumEndpoints: N_IN_ENDPOINTS + 1 OUT endpoints used */
  0x00,   /* bInterfaceClass: Communication Interface Class */
  0x00,   /* bInterfaceSubClass: Abstract Control Model */
  0x00,   /* bInterfaceProtocol: Common AT commands */
  0x00,   /* iInterface: */

  /*Endpoint IN 1 Descriptor*/
  0x07,                           /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,   /* bDescriptorType: Endpoint */
  DATA_IN_EP1,                     /* bEndpointAddress */
  0x02,                           /* bmAttributes: Bulk */
  LOBYTE(SS_WCID_DATA_FS_MAX_PACKET_SIZE),     /* wMaxPacketSize: */
  HIBYTE(SS_WCID_DATA_FS_MAX_PACKET_SIZE),
  0x00,                           /* bInterval: ignore for Bulk transfer */

#if (N_IN_ENDPOINTS > 1)
  /*Endpoint IN 2 Descriptor*/
  0x07,                           /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,   /* bDescriptorType: Endpoint */
  DATA_IN_EP2,                     /* bEndpointAddress */
  0x02,                           /* bmAttributes: Bulk */
  LOBYTE(SS_WCID_DATA_FS_MAX_PACKET_SIZE),     /* wMaxPacketSize: */
  HIBYTE(SS_WCID_DATA_FS_MAX_PACKET_SIZE),
  0x00,                           /* bInterval: ignore for Bulk transfer */

#if (N_IN_ENDPOINTS > 2)
  /*Endpoint IN 3 Descriptor*/
  0x07,                           /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,   /* bDescriptorType: Endpoint */
  DATA_IN_EP3,                     /* bEndpointAddress */
  0x02,                           /* bmAttributes: Bulk */
  LOBYTE(SS_WCID_DATA_FS_MAX_PACKET_SIZE),     /* wMaxPacketSize: */
  HIBYTE(SS_WCID_DATA_FS_MAX_PACKET_SIZE),
  0x00,                           /* bInterval: ignore for Bulk transfer */

#if (N_IN_ENDPOINTS > 3)
  /*Endpoint IN 4 Descriptor*/
  0x07,                           /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,   /* bDescriptorType: Endpoint */
  DATA_IN_EP4,                     /* bEndpointAddress */
  0x02,                           /* bmAttributes: Bulk */
  LOBYTE(SS_WCID_DATA_FS_MAX_PACKET_SIZE),     /* wMaxPacketSize: */
  HIBYTE(SS_WCID_DATA_FS_MAX_PACKET_SIZE),
  0x00,                           /* bInterval: ignore for Bulk transfer */

#if (N_IN_ENDPOINTS > 4)
  /*Endpoint IN 5 Descriptor*/
  0x07,                           /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,   /* bDescriptorType: Endpoint */
  DATA_IN_EP5,                     /* bEndpointAddress */
  0x02,                           /* bmAttributes: Bulk */
  LOBYTE(SS_WCID_DATA_FS_MAX_PACKET_SIZE),     /* wMaxPacketSize: */
  HIBYTE(SS_WCID_DATA_FS_MAX_PACKET_SIZE),
  0x00,                           /* bInterval: ignore for Bulk transfer */

#endif
#endif
#endif
#endif

  /*Endpoint OUT 1 Descriptor*/
  0x07,                           /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,   /* bDescriptorType: Endpoint */
  DATA_OUT_EP1,                     /* bEndpointAddress */
  0x02,                           /* bmAttributes: Bulk */
  LOBYTE(SS_WCID_DATA_FS_MAX_PACKET_SIZE),     /* wMaxPacketSize: */
  HIBYTE(SS_WCID_DATA_FS_MAX_PACKET_SIZE),
  0x00                           /* bInterval: ignore for Bulk transfer */

} ;


/* USB WCID STREAMING device Configuration Descriptor (Hs) */
__ALIGN_BEGIN static uint8_t USBD_WCID_STREAMING_CfgFSDesc[] __ALIGN_END =
{
  /*Configuration Descriptor*/
  0x09,   /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION,      /* bDescriptorType: Configuration */
  USB_SS_WCID_CONFIG_DESC_SIZ,                /* wTotalLength:no of returned bytes */
  0x00,
  0x01,   /* bNumInterfaces: Total 1 interface */
  0x01,   /* bConfigurationValue: Configuration value */
  0x00,   /* iConfiguration: Index of string descriptor describing the configuration */
  0xC0,   /* bmAttributes: self powered */
  0x32,   /* MaxPower 0 mA */

  /*---------------------------------------------------------------------------*/

  /*Interface Descriptor */
  0x09,   /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: Interface */
  /* Interface descriptor type */
  0x00,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  N_IN_ENDPOINTS + 1,   /* bNumEndpoints: N_IN_ENDPOINTS + 1 OUT endpoints used */
  0x00,   /* bInterfaceClass: Communication Interface Class */
  0x00,   /* bInterfaceSubClass: Abstract Control Model */
  0x00,   /* bInterfaceProtocol: Common AT commands */
  0x00,   /* iInterface: */

  /*Endpoint IN 1 Descriptor*/
  0x07,                           /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,   /* bDescriptorType: Endpoint */
  DATA_IN_EP1,                     /* bEndpointAddress */
  0x02,                           /* bmAttributes: Bulk */
  LOBYTE(SS_WCID_DATA_FS_MAX_PACKET_SIZE),     /* wMaxPacketSize: */
  HIBYTE(SS_WCID_DATA_FS_MAX_PACKET_SIZE),
  0x00,                           /* bInterval: ignore for Bulk transfer */

#if (N_IN_ENDPOINTS > 1)
  /*Endpoint IN 2 Descriptor*/
  0x07,                           /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,   /* bDescriptorType: Endpoint */
  DATA_IN_EP2,                     /* bEndpointAddress */
  0x02,                           /* bmAttributes: Bulk */
  LOBYTE(SS_WCID_DATA_FS_MAX_PACKET_SIZE),     /* wMaxPacketSize: */
  HIBYTE(SS_WCID_DATA_FS_MAX_PACKET_SIZE),
  0x00,                           /* bInterval: ignore for Bulk transfer */

#if (N_IN_ENDPOINTS > 2)
  /*Endpoint IN 3 Descriptor*/
  0x07,                           /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,   /* bDescriptorType: Endpoint */
  DATA_IN_EP3,                     /* bEndpointAddress */
  0x02,                           /* bmAttributes: Bulk */
  LOBYTE(SS_WCID_DATA_FS_MAX_PACKET_SIZE),     /* wMaxPacketSize: */
  HIBYTE(SS_WCID_DATA_FS_MAX_PACKET_SIZE),
  0x00,                           /* bInterval: ignore for Bulk transfer */

#if (N_IN_ENDPOINTS > 3)
  /*Endpoint IN 4 Descriptor*/
  0x07,                           /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,   /* bDescriptorType: Endpoint */
  DATA_IN_EP4,                     /* bEndpointAddress */
  0x02,                           /* bmAttributes: Bulk */
  LOBYTE(SS_WCID_DATA_FS_MAX_PACKET_SIZE),     /* wMaxPacketSize: */
  HIBYTE(SS_WCID_DATA_FS_MAX_PACKET_SIZE),
  0x00,                           /* bInterval: ignore for Bulk transfer */

#if (N_IN_ENDPOINTS > 4)
  /*Endpoint IN 5 Descriptor*/
  0x07,                           /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,   /* bDescriptorType: Endpoint */
  DATA_IN_EP5,                     /* bEndpointAddress */
  0x02,                           /* bmAttributes: Bulk */
  LOBYTE(SS_WCID_DATA_FS_MAX_PACKET_SIZE),     /* wMaxPacketSize: */
  HIBYTE(SS_WCID_DATA_FS_MAX_PACKET_SIZE),
  0x00,                           /* bInterval: ignore for Bulk transfer */

#endif
#endif
#endif
#endif

  /*Endpoint OUT 1 Descriptor*/
  0x07,                           /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,   /* bDescriptorType: Endpoint */
  DATA_OUT_EP1,                     /* bEndpointAddress */
  0x02,                           /* bmAttributes: Bulk */
  LOBYTE(SS_WCID_DATA_FS_MAX_PACKET_SIZE),     /* wMaxPacketSize: */
  HIBYTE(SS_WCID_DATA_FS_MAX_PACKET_SIZE),
  0x00                           /* bInterval: ignore for Bulk transfer */

} ;

/* USB WCID STREAMING device Configuration Descriptor (Other) */
__ALIGN_BEGIN static uint8_t USBD_WCID_STREAMING_OtherSpeedCfgDesc[USB_SS_WCID_OTHER_CONFIG_DESC_SIZ] __ALIGN_END =
{
  /*Configuration Descriptor*/
  0x09,   /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION,      /* bDescriptorType: Configuration */
  USB_SS_WCID_OTHER_CONFIG_DESC_SIZ,                /* wTotalLength:no of returned bytes */
  0x00,
  0x01,   /* bNumInterfaces: Total 1 interface */
  0x01,   /* bConfigurationValue: Configuration value */
  0x00,   /* iConfiguration: Index of string descriptor describing the configuration */
  0xC0,   /* bmAttributes: self powered */
  0x32,   /* MaxPower 0 mA */

  /*---------------------------------------------------------------------------*/

  /*Interface Descriptor */
  0x09,   /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: Interface */
  /* Interface descriptor type */
  0x00,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x03,   /* bNumEndpoints: 2 endpoints used */
  0x00,   /* bInterfaceClass: Communication Interface Class */
  0x00,   /* bInterfaceSubClass: Abstract Control Model */
  0x00,   /* bInterfaceProtocol: Common AT commands */
  0x00,   /* iInterface: */

  /*Endpoint 1 Descriptor*/
  0x07,                           /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,   /* bDescriptorType: Endpoint */
  DATA_IN_EP1,                     /* bEndpointAddress */
  0x02,                           /* bmAttributes: Bulk */
  LOBYTE(SS_WCID_DATA_FS_MAX_PACKET_SIZE),     /* wMaxPacketSize: */
  HIBYTE(SS_WCID_DATA_FS_MAX_PACKET_SIZE),
  0x00,                           /* bInterval: ignore for Bulk transfer */

  /*Endpoint 1 Descriptor*/
  0x07,                           /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,   /* bDescriptorType: Endpoint */
  DATA_IN_EP2,                     /* bEndpointAddress */
  0x02,                           /* bmAttributes: Bulk */
  LOBYTE(SS_WCID_DATA_FS_MAX_PACKET_SIZE),     /* wMaxPacketSize: */
  HIBYTE(SS_WCID_DATA_FS_MAX_PACKET_SIZE),
  0x00,                           /* bInterval: ignore for Bulk transfer */

  /*Endpoint 2 Descriptor*/
  0x07,                           /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,   /* bDescriptorType: Endpoint */
  DATA_OUT_EP1,                     /* bEndpointAddress */
  0x02,                           /* bmAttributes: Bulk */
  LOBYTE(SS_WCID_DATA_FS_MAX_PACKET_SIZE),     /* wMaxPacketSize: */
  HIBYTE(SS_WCID_DATA_FS_MAX_PACKET_SIZE),
  0x00                           /* bInterval: ignore for Bulk transfer */
} ;

/**
  * @}
  */

/** @defgroup USBD_WCID_STREAMING_Private_Functions
  * @{
  */

/**
  * @brief  USBD_WCID_STREAMING_Init
  *         Initialize the WCID interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
/* USBD_WCID_STREAMING_HandleTypeDef USBTEST; */
static uint8_t  USBD_WCID_STREAMING_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);
  uint8_t ret = 0;
  USBD_WCID_STREAMING_HandleTypeDef   *hwcid;

  /* Open EP IN */
  (void)USBD_LL_OpenEP(pdev, DATA_IN_EP1, USBD_EP_TYPE_BULK, SS_WCID_DATA_FS_IN_PACKET_SIZE);

#if (N_IN_ENDPOINTS >1)
  /* Open EP IN */
  (void)USBD_LL_OpenEP(pdev, DATA_IN_EP2, USBD_EP_TYPE_BULK, SS_WCID_DATA_FS_IN_PACKET_SIZE);
#if (N_IN_ENDPOINTS >2)
  /* Open EP IN */
  (void)USBD_LL_OpenEP(pdev, DATA_IN_EP3, USBD_EP_TYPE_BULK, SS_WCID_DATA_FS_IN_PACKET_SIZE);
#if (N_IN_ENDPOINTS >3)
  /* Open EP IN */
  (void)USBD_LL_OpenEP(pdev, DATA_IN_EP4, USBD_EP_TYPE_BULK, SS_WCID_DATA_FS_IN_PACKET_SIZE);
#if (N_IN_ENDPOINTS >4)
  /* Open EP IN */
  (void)USBD_LL_OpenEP(pdev, DATA_IN_EP5, USBD_EP_TYPE_BULK, SS_WCID_DATA_FS_IN_PACKET_SIZE);
#endif
#endif
#endif
#endif
  /* Open EP OUT */
  (void)USBD_LL_OpenEP(pdev, DATA_OUT_EP1, USBD_EP_TYPE_BULK, SS_WCID_DATA_FS_OUT_PACKET_SIZE);

  pdev->pClassData = (USBD_WCID_STREAMING_HandleTypeDef *)USBD_malloc(sizeof(USBD_WCID_STREAMING_HandleTypeDef)); /* &USBTEST; */

  if (pdev->pClassData == NULL)
  {
    ret = 1;
  }
  else
  {
    uint8_t i;

    (void)memset((void *)pdev->pClassData, 0, sizeof(USBD_WCID_STREAMING_HandleTypeDef));

    hwcid = (USBD_WCID_STREAMING_HandleTypeDef *) pdev->pClassData;
    /* Init  physical Interface components */
    ((USBD_WCID_STREAMING_ItfTypeDef *)pdev->pUserData)->Init();
    /* Init Xfer states */
    hwcid->RxState = 0;

    for (i = 0; i < N_IN_ENDPOINTS; i++)
    {
      hwcid->TXStates[i] = 0;
      hwcid->TxBuffStatus[i] = 0;
    }
    /* Prepare Out endpoint to receive next packet */
    (void)USBD_LL_PrepareReceive(pdev, DATA_OUT_EP1, hwcid->RxBuffer, SS_WCID_DATA_FS_OUT_PACKET_SIZE);
  }
  return ret;
}

/**
  * @brief  USBD_WCID_STREAMING_DeInit
  *         DeInitialize the WCID layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_WCID_STREAMING_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);
  uint8_t ret = 0;
  /* Open EP IN */
  (void)USBD_LL_CloseEP(pdev, DATA_IN_EP1);
#if (N_IN_ENDPOINTS >1)
  /* Open EP IN */
  (void)USBD_LL_CloseEP(pdev, DATA_IN_EP2);
#if (N_IN_ENDPOINTS >2)
  /* Open EP IN */
  (void)USBD_LL_CloseEP(pdev, DATA_IN_EP3);
#if (N_IN_ENDPOINTS >3)
  /* Open EP IN */
  (void)USBD_LL_CloseEP(pdev, DATA_IN_EP4);
#if (N_IN_ENDPOINTS >4)
  /* Open EP IN */
  (void)USBD_LL_CloseEP(pdev, DATA_IN_EP5);
#endif
#endif
#endif
#endif

  /* Open EP OUT */
  (void)USBD_LL_CloseEP(pdev, DATA_OUT_EP1);

  /* DeInit  physical Interface components */
  if (pdev->pClassData != NULL)
  {
    ((USBD_WCID_STREAMING_ItfTypeDef *)pdev->pUserData)->DeInit();
    USBD_free(pdev->pClassData);
    pdev->pClassData = NULL;
  }
  return ret;
}

/**
  * @brief  USBD_WCID_STREAMING_SOF
  *         Handles start of frame
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t  USBD_WCID_STREAMING_SOF(USBD_HandleTypeDef *pdev)
{
  uint8_t i;
  __IO uint32_t *TX_States = ((USBD_WCID_STREAMING_HandleTypeDef *)(pdev->pClassData))->TXStates;
  __IO uint8_t *TxBuffStatus = ((USBD_WCID_STREAMING_HandleTypeDef *)(pdev->pClassData))->TxBuffStatus;
  uint8_t **TxBuffer = ((USBD_WCID_STREAMING_HandleTypeDef *)(pdev->pClassData))->TxBuffer;
  uint16_t *USB_DataSizePerEp = ((USBD_WCID_STREAMING_HandleTypeDef *)(pdev->pClassData))->USB_DataSizePerEp;
  __IO uint8_t *lastPacketSent = ((USBD_WCID_STREAMING_HandleTypeDef *)(pdev->pClassData))->lastPacketSent;
  uint8_t *status = &(((USBD_WCID_STREAMING_HandleTypeDef *)(pdev->pClassData))->streamingStatus);

  if (*status == STREAMING_STATUS_STARTED)
  {
    for (i = 0; i < (N_IN_ENDPOINTS - 1U); i++)
    {
      if (TX_States[i] != 1U)
      {
        if (TxBuffStatus[i] == 1U)
        {
          lastPacketSent[i] = 0;
          TxBuffStatus[i] = 0;
          /*send*/
          (void)USBD_WCID_STREAMING_SetTxBuffer(pdev, (uint8_t *) & (TxBuffer[i][0]), (USB_DataSizePerEp[i] / 2U));
          if (USBD_WCID_STREAMING_TransmitPacket(pdev, (i + 1U) | 0x80U) != USBD_OK)
          {
            return 1;     /* USBD_FAIL */
          }
        }
        else if (TxBuffStatus[i] == 2U)
        {
          lastPacketSent[i] = 0;
          TxBuffStatus[i] = 0;
          /*send*/
          (void)USBD_WCID_STREAMING_SetTxBuffer(pdev, (uint8_t *) & (TxBuffer[i][(USB_DataSizePerEp[i] / 2U)]),
                                                (USB_DataSizePerEp[i] / 2U));
          if (USBD_WCID_STREAMING_TransmitPacket(pdev, (i + 1U) | 0x80U) != USBD_OK)
          {
            return 1;     /* USBD_FAIL */
          }
        }
        else
        {
        }
      }
    }

    if (TX_States[N_IN_ENDPOINTS - 1U] != 1U)
    {
      for (i = (N_IN_ENDPOINTS - 1U); i < N_CHANNELS_MAX; i++)
      {
        if (TxBuffStatus[i] == 1U)
        {
          /*send*/
          lastPacketSent[i] = 0;
          (void)USBD_WCID_STREAMING_SetTxBuffer(pdev, (uint8_t *) & (TxBuffer[i][0]), (USB_DataSizePerEp[i] / 2U));
          if (USBD_WCID_STREAMING_TransmitPacket(pdev, (N_IN_ENDPOINTS) | 0x80U) == USBD_OK)
          {
            TxBuffStatus[i] = 0;
          }
        }
        else if (TxBuffStatus[i] == 2U)
        {
          /*send*/
          lastPacketSent[i] = 0;
          (void)USBD_WCID_STREAMING_SetTxBuffer(pdev, (uint8_t *) & (TxBuffer[i][(USB_DataSizePerEp[i] / 2U)]),
                                                (USB_DataSizePerEp[i] / 2U));
          if (USBD_WCID_STREAMING_TransmitPacket(pdev, (N_IN_ENDPOINTS) | 0x80U) == USBD_OK)
          {
            TxBuffStatus[i] = 0;
          }
        }
        else
        {
        }
      }
    }
  }
  else if (*status == STREAMING_STATUS_STOPPING)
  {
    uint32_t sum = 0;

    for (i = 0; i < N_IN_ENDPOINTS; i++)
    {
      if (lastPacketSent[i] != 1U)
      {
        lastPacketSent[i] = 1;
        (void)USBD_WCID_STREAMING_SetTxBuffer(pdev, (uint8_t *) & (TxBuffer[i][(USB_DataSizePerEp[i] / 2U)]),
                                              (USB_DataSizePerEp[i] / 2U));
        if (USBD_WCID_STREAMING_TransmitPacket(pdev, (i + 1U) | 0x80U) == USBD_OK)
        {
          TxBuffStatus[i] = 0;
        }
      }
      sum += lastPacketSent[i];
      if (sum == N_IN_ENDPOINTS)
      {
        *status = STREAMING_STATUS_IDLE;
      }
    }
  }
  return 0;     /* USBD_OK */
}

/**
  * @brief  USBD_WCID_STREAMING_Setup
  *         Handle the WCID specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t  USBD_WCID_STREAMING_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  uint8_t ret = 0;
  static uint8_t ifalt = 0;
  USBD_WCID_STREAMING_HandleTypeDef   *hwcid = (USBD_WCID_STREAMING_HandleTypeDef *) pdev->pClassData;

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    case USB_REQ_TYPE_CLASS :
    {
      if (req->wLength != 0U)
      {
        if ((req->bmRequest & 0x80U) != 0U) /* GET */
        {
          ((USBD_WCID_STREAMING_ItfTypeDef *)pdev->pUserData)->Control(0, req->bRequest, req->wValue, req->wIndex,
                                                                       (uint8_t *)hwcid->data,
                                                                       req->wLength);
          (void)USBD_CtlSendData(pdev, (uint8_t *)hwcid->data, req->wLength);
        }
        else /* SET */
        {
          hwcid->CmdOpCode = req->bRequest;
          hwcid->CmdLength = req->wLength;
          hwcid->wValue = req->wValue;
          hwcid->wIndex = req->wIndex;
          (void)USBD_CtlPrepareRx(pdev, (uint8_t *)hwcid->data, req->wLength);
        }
      }
      break;
    }
    case USB_REQ_TYPE_STANDARD :
    {
      if (req->bRequest == USB_REQ_GET_INTERFACE)
      {
        (void)USBD_CtlSendData(pdev, &ifalt, 1);
      }
      break;
    }
    case USB_REQ_TYPE_VENDOR :
    {
      (void)USBD_VendDevReq(pdev, req);
      break;
    }
    default:
      break;
  }
  return ret;
}

/**
  * @brief  USBD_WCID_STREAMING_DataIn
  *         Data sent on non-control IN endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t  USBD_WCID_STREAMING_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_WCID_STREAMING_HandleTypeDef   *hwcid = (USBD_WCID_STREAMING_HandleTypeDef *) pdev->pClassData;
  hwcid->TXStates[(epnum & 0x7FU) - 1U] = 0;
  return 0;
}

/**
  * @brief  USBD_WCID_STREAMING_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t  USBD_WCID_STREAMING_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_WCID_STREAMING_HandleTypeDef   *hwcid = (USBD_WCID_STREAMING_HandleTypeDef *) pdev->pClassData;
  /* Get the received data length */
  hwcid->RxLength = USBD_LL_GetRxDataSize(pdev, epnum);
  if (pdev->pClassData != NULL)
  {
    ((USBD_WCID_STREAMING_ItfTypeDef *)pdev->pUserData)->Receive(hwcid->RxBuffer, hwcid->RxLength);
    (void)USBD_WCID_STREAMING_ReceivePacket(pdev);
    return 0;   /* USBD_OK */
  }
  else
  {
    return 1;   /* USBD_FAIL */
  }
}

/**
  * @brief  USBD_WCID_STREAMING_DataOut
  *         Handle EP0 Rx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t  USBD_WCID_STREAMING_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
  USBD_WCID_STREAMING_HandleTypeDef   *hwcid = (USBD_WCID_STREAMING_HandleTypeDef *) pdev->pClassData;
  if ((pdev->pUserData != NULL) && (hwcid->CmdOpCode != 0xFFU))
  {
    ((USBD_WCID_STREAMING_ItfTypeDef *)pdev->pUserData)->Control(1, hwcid->CmdOpCode, hwcid->wValue,  hwcid->wIndex,
                                                                 (uint8_t *)hwcid->data,
                                                                 hwcid->CmdLength);
    hwcid->CmdOpCode = 0xFF;
  }
  return 0;   /* USBD_OK */
}

/**
  * @brief  USBD_WCID_STREAMING_GetFSCfgDesc
  *         Return configuration descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_WCID_STREAMING_GetFSCfgDesc(uint16_t *length)
{
  *length = (uint16_t)(sizeof(USBD_WCID_STREAMING_CfgFSDesc));
  return USBD_WCID_STREAMING_CfgFSDesc;
}

/**
  * @brief  USBD_WCID_STREAMING_GetHSCfgDesc
  *         Return configuration descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_WCID_STREAMING_GetHSCfgDesc(uint16_t *length)
{
  *length = (uint16_t)(sizeof(USBD_WCID_STREAMING_CfgHSDesc));
  return USBD_WCID_STREAMING_CfgHSDesc;
}

/**
  * @brief  USBD_WCID_STREAMING_GetHSCfgDesc
  *         Return configuration descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_WCID_STREAMING_GetOtherSpeedCfgDesc(uint16_t *length)
{
  *length = (uint16_t)(sizeof(USBD_WCID_STREAMING_OtherSpeedCfgDesc));
  return USBD_WCID_STREAMING_OtherSpeedCfgDesc;
}

/**
  * @brief  USBD_WCID_STREAMING_GetDeviceQualifierDescriptor
  *         return Device Qualifier descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
uint8_t  *USBD_WCID_STREAMING_GetDeviceQualifierDescriptor(uint16_t *length)
{
  *length = (uint16_t)(sizeof(USBD_WCID_STREAMING_DeviceQualifierDesc));
  return USBD_WCID_STREAMING_DeviceQualifierDesc;
}

/**
  * @brief  USBD_WCID_STREAMING_GetUsrStrDescriptor
  *         return User String descriptor
  * @param  length : pointer data length
  * @param  index : index
  * @retval pointer to descriptor buffer
  */
uint8_t   *USBD_WCID_STREAMING_GetUsrStrDescriptor(USBD_HandleTypeDef *pdev, uint8_t index,  uint16_t *length)
{
  UNUSED(pdev);
  if ((index) == 0xEEU)
  {
    *length = (uint16_t)(sizeof(USBD_OSStringDesc));
    return USBD_OSStringDesc;
  }
  return NULL;
}

/**
  * @brief  USBD_WCID_STREAMING_OSCompIDDescriptor
  *         return Microsoft OS String Descriptor
  * @param  length : pointer data length
  * @param  speed: Current device speed
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_WCID_STREAMING_OSCompIDDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  *length = (uint16_t)(sizeof(OS_IDFeatureDescriptor));
  return (uint8_t *)OS_IDFeatureDescriptor;
}

/**
  * @brief  USBD_WCID_STREAMING_OSPropertiesFeatureDescriptor
  *         return USBD_WCID_STREAMING_OSPropertiesFeatureDescriptor;
  * @param  speed: Current device speed
  * @param  length: Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
static uint8_t *USBD_WCID_STREAMING_OSPropertiesFeatureDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  *length = (uint16_t)(sizeof(USBD_ExtPropertiesDescStruct));
  return (uint8_t *)&USBD_ExtPropertiesDesc;
}

/**
  * @brief  USBD_WCID_STREAMING_SetTxBuffer
  * @param  pdev: device instance
  * @param  pbuff: Tx Buffer
  * @retval status
  */
static uint8_t  USBD_WCID_STREAMING_SetTxBuffer(USBD_HandleTypeDef *pdev, uint8_t  *pbuff, uint16_t length)
{
  USBD_WCID_STREAMING_HandleTypeDef   *hwcid = (USBD_WCID_STREAMING_HandleTypeDef *) pdev->pClassData;
  hwcid->CurrentTxBuffer = pbuff;
  hwcid->TxLength = length;
  return 0;   /* USBD_OK */
}


/**
  * @brief  USBD_WCID_STREAMING_SetRxDataBuffer
  * @param  pdev: device instance
  * @param  ptr: Rx Buffer
  * @retval status
  */
uint8_t  USBD_WCID_STREAMING_SetRxDataBuffer(USBD_HandleTypeDef *pdev, uint8_t *ptr)
{
  USBD_WCID_STREAMING_HandleTypeDef   *hwcid = (USBD_WCID_STREAMING_HandleTypeDef *) pdev->pClassData;
  hwcid->RxBuffer = ptr;
  return 0;   /* USBD_OK */
}

/**
  * @brief  USBD_WCID_STREAMING_TransmitPacket
  *         enqueue a packet on the specific endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
USBD_StatusTypeDef  USBD_WCID_STREAMING_TransmitPacket(USBD_HandleTypeDef *pdev, uint8_t epNumber)
{
  USBD_WCID_STREAMING_HandleTypeDef *hwcid = (USBD_WCID_STREAMING_HandleTypeDef *) pdev->pClassData;

  if (pdev->pClassData != NULL)
  {
    if (hwcid->TXStates[(epNumber & 0x7FU) - 1U] == 0U)
    {
      /* Tx Transfer in progress */
      hwcid->TXStates[(epNumber & 0x7FU) - 1U] = 1;
      /* Transmit next packet */
      (void)USBD_LL_Transmit(pdev, (epNumber), hwcid->CurrentTxBuffer, hwcid->TxLength);
      return USBD_OK;
    }
    else
    {
      return USBD_BUSY;
    }
  }
  else
  {
    return USBD_FAIL;
  }
}

/**
  * @brief  USBD_WCID_STREAMING_ReceivePacket
  *         prepare OUT Endpoint for reception
  * @param  pdev: device instance
  * @retval status
  */
static USBD_StatusTypeDef USBD_WCID_STREAMING_ReceivePacket(USBD_HandleTypeDef *pdev)
{
  USBD_WCID_STREAMING_HandleTypeDef   *hwcid = (USBD_WCID_STREAMING_HandleTypeDef *) pdev->pClassData;

  /* Suspend or Resume USB Out process */
  if (pdev->pClassData != NULL)
  {
    /* Prepare Out endpoint to receive next packet */
    (void)USBD_LL_PrepareReceive(pdev, DATA_OUT_EP1, hwcid->RxBuffer, SS_WCID_DATA_FS_OUT_PACKET_SIZE);
    return USBD_OK;
  }
  else
  {
    return USBD_FAIL;
  }
}

/**
  * @brief  USBD_VendDevReq
  *         Handle the WCID vendor requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static USBD_StatusTypeDef  USBD_VendDevReq(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  uint16_t len;
  uint8_t *pbuf;

  switch (req->wIndex)
  {
    case USB_DESC_TYPE_OS_FEATURE_EXT_PROPERTIES:
    {
      pbuf = USBD_WCID_STREAMING_OSCompIDDescriptor(pdev->dev_speed, &len);
      if ((len != 0U) && (req->wLength != 0U))
      {
        len = MIN(len, req->wLength);
        (void)USBD_CtlSendData(pdev, pbuf, len);
      }
      break;
    }
    case USB_DESC_TYPE_OS_FEATURE_EXT_COMPAT_ID:
    {
      pbuf = USBD_WCID_STREAMING_OSPropertiesFeatureDescriptor(pdev->dev_speed, &len);
      if ((len != 0U) && (req->wLength != 0U))
      {
        len = MIN(len, req->wLength);
        (void)USBD_CtlSendData(pdev, pbuf, len);
      }
      break;
    }
    case USB_DESC_TYPE_USER_CONTROL_ID:
    {
      USBD_WCID_STREAMING_HandleTypeDef *hwcid = (USBD_WCID_STREAMING_HandleTypeDef *) pdev->pClassData;
      if (req->wLength != 0U)
      {
        if ((req->bmRequest & 0x80U) != 0U)
        {
          ((USBD_WCID_STREAMING_ItfTypeDef *)pdev->pUserData)->Control(0, req->bRequest, req->wValue, req->wIndex,
                                                                       (uint8_t *)hwcid->data,
                                                                       req->wLength);

          (void)USBD_CtlSendData(pdev, (uint8_t *)hwcid->data, req->wLength);
        }
        else
        {
          hwcid->CmdOpCode = req->bRequest;
          hwcid->CmdLength = req->wLength;
          hwcid->wValue = req->wValue;
          hwcid->wIndex = req->wIndex;
          (void)USBD_CtlPrepareRx(pdev, (uint8_t *)hwcid->data, req->wLength);
        }
      }
      break;
    }
    default :
      break;
  }
  return USBD_OK;
}

/**
  * @}
  */

/** @defgroup USBD_WCID_STREAMING_Public_Functions
  * @{
  */

/**
  * @brief  USBD_WCID_STREAMING_RegisterInterface
  * @param  pdev: device instance
  * @param  fops: WCID Interface callback structure
  * @retval status
  */
uint8_t  USBD_WCID_STREAMING_RegisterInterface(USBD_HandleTypeDef *pdev, USBD_WCID_STREAMING_ItfTypeDef *fops)
{
  uint8_t  ret = 1;     /* USBD_FAIL */
  if (fops != NULL)
  {
    pdev->pUserData = fops;
    ret = 0;    /* USBD_OK */
  }
  return ret;
}

/**
  * @brief  USBD_WCID_STREAMING_FillTxDataBuffer
  *         Enqueue data on a specific channel
  * @param  ch_number: channel number to be used
  * @param  buf: pointer to data to be sent
  * @param  size: length of data in bytes
  * @param  pdev: device instance
  * @retval status
  */
uint8_t USBD_WCID_STREAMING_FillTxDataBuffer(USBD_HandleTypeDef *pdev, uint8_t ch_number, uint8_t *buf, uint32_t size)
{
  USBD_WCID_STREAMING_HandleTypeDef *hwcid = (USBD_WCID_STREAMING_HandleTypeDef *) pdev->pClassData;
  uint8_t **TxBuffer = hwcid->TxBuffer;

  __disable_irq();
  __IO uint8_t *TxBuffStatus = hwcid->TxBuffStatus;
  __enable_irq();

  uint32_t *TxBuffIdx = hwcid->TxBuffIdx;
  uint16_t *USB_DataSizePerEp = hwcid->USB_DataSizePerEp;
  __IO uint8_t *TxBuffReset = hwcid->TxBuffReset;

  uint32_t i;
  uint8_t *txBuffptr = (uint8_t *)TxBuffer[ch_number];

  if (TxBuffReset[ch_number] == 1U)
  {
    if (ch_number >= (N_IN_ENDPOINTS - 1U))
    {
      txBuffptr[0] = ch_number;
      TxBuffIdx[ch_number] = 1;
    }
    else
    {
      TxBuffIdx[ch_number] = 0;
    }
    __disable_irq();
    TxBuffStatus[ch_number] = 0;
    __enable_irq();

    TxBuffReset[ch_number] = 0;
  }

  uint32_t txBuffIdx = TxBuffIdx[ch_number];

  for (i = 0; i < size; i++)
  {
    txBuffptr[txBuffIdx] = buf[i];
    txBuffIdx = (txBuffIdx + 1U) % (USB_DataSizePerEp[ch_number]);

    if (txBuffIdx == ((uint32_t)USB_DataSizePerEp[ch_number] / 2U))
    {
      __disable_irq();
      TxBuffStatus[ch_number] = 1;
      __enable_irq();

      if (ch_number >= (N_IN_ENDPOINTS - 1U))
      {
        txBuffptr[txBuffIdx] = ch_number;
        txBuffIdx = (txBuffIdx + 1U);
      }
    }
    else if (txBuffIdx == 0U)
    {
      __disable_irq();
      TxBuffStatus[ch_number] = 2;
      __enable_irq();

      if (ch_number >= (N_IN_ENDPOINTS - 1U))
      {
        txBuffptr[txBuffIdx] = ch_number;
        txBuffIdx = (txBuffIdx + 1U);
      }
    }
    else
    {
    }
  }
  TxBuffIdx[ch_number] = txBuffIdx;
  return 0;     /* USBD_OK */
}

/**
  * @brief  USBD_WCID_STREAMING_CleanTxDataBuffer
  *         discard remaining data in the buffer of the specified ch
  * @param  ch_number: channel number
  * @retval status
  */
uint8_t USBD_WCID_STREAMING_CleanTxDataBuffer(USBD_HandleTypeDef *pdev, uint8_t ch_number)
{
  USBD_WCID_STREAMING_HandleTypeDef *hwcid = (USBD_WCID_STREAMING_HandleTypeDef *) pdev->pClassData;
  __IO uint8_t *TxBuffReset = hwcid->TxBuffReset;
  TxBuffReset[ch_number] = 1;
  return 0;     /* USBD_OK */
}

/**
  * @brief  USBD_WCID_STREAMING_SetTxDataBuffer
  *         Set buffer data for a specific channel.
  *         A data "frame" of size bytes will be sent on the USB endpoints
  * @param  ch_number: channel number
  * @param  ptr: pointer to data
  *              NOTE! Memory allocated must be at least equal to (size * 2) + 2;
  * @param  size: length of each packet in bytes
  * @retval status
  */
uint8_t USBD_WCID_STREAMING_SetTxDataBuffer(USBD_HandleTypeDef *pdev, uint8_t ch_number, uint8_t *ptr, uint16_t size)
{
  USBD_WCID_STREAMING_HandleTypeDef *hwcid = (USBD_WCID_STREAMING_HandleTypeDef *) pdev->pClassData;
  uint8_t **TxBuffer = hwcid->TxBuffer;
  uint32_t *TxBuffIdx = hwcid->TxBuffIdx;
  uint16_t *USB_DataSizePerEp = hwcid->USB_DataSizePerEp;

  if (ch_number >= (N_IN_ENDPOINTS - 1U))
  {
    TxBuffer[ch_number] = ptr;
    USB_DataSizePerEp[ch_number] = (size * 2U) + 2U;  /* Double buffer contains 2 * user data packets + 1st byte of tag for each half*/
    ptr[0] = ch_number;
    TxBuffIdx[ch_number] = 1;
  }
  else
  {
    TxBuffer[ch_number] = ptr;
    USB_DataSizePerEp[ch_number] = size * 2U;  /* Double buffer contains 2 * user data packets*/
  }
  return 0;     /* USBD_OK */
}

/**
  * @brief  USBD_WCID_STREAMING_StartStreaming
  *         Enable USB streaming.
  * @param  pdev: device instance
  * @retval status
  */
uint8_t USBD_WCID_STREAMING_StartStreaming(USBD_HandleTypeDef *pdev)
{
  uint8_t *status = &(((USBD_WCID_STREAMING_HandleTypeDef *)(pdev->pClassData))->streamingStatus);
  __IO uint8_t *lastPacketSent = ((USBD_WCID_STREAMING_HandleTypeDef *)(pdev->pClassData))->lastPacketSent;

  for (uint8_t i = 0; i < (N_IN_ENDPOINTS - 1U); i++)
  {
    lastPacketSent[i] = 1;
  }
  *status = STREAMING_STATUS_STARTED;
  return 0;     /* USBD_OK */
}

/**
  * @brief  USBD_WCID_STREAMING_StopStreaming
  *         Enable USB streaming.
  * @param  pdev: device instance
  * @retval status
  */
uint8_t USBD_WCID_STREAMING_StopStreaming(USBD_HandleTypeDef *pdev)
{
  uint8_t *status = &(((USBD_WCID_STREAMING_HandleTypeDef *)(pdev->pClassData))->streamingStatus);
  *status = STREAMING_STATUS_STOPPING;
  return 0;     /* USBD_OK */
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


