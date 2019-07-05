/**
******************************************************************************
* @file    usbd_audio.c
* @author  MCD Application Team
* @version V2.4.2
* @date    11-December-2015
* @brief   This file provides the Audio core functions.
*
* @verbatim
*      
*          ===================================================================      
*                                AUDIO Class  Description
*          ===================================================================
*           This driver manages the Audio Class 1.0 following the "USB Device Class Definition for
*           Audio Devices V1.0 Mar 18, 98".
*           This driver implements the following aspects of the specification:
*             - Device descriptor management
*             - Configuration descriptor management
*             - Standard AC Interface Descriptor management
*             - 1 Audio Streaming Interface (with single channel, PCM, Stereo mode)
*             - 1 Audio Streaming Endpoint
*             - 1 Audio Terminal Input (1 channel)
*             - Audio Class-Specific AC Interfaces
*             - Audio Class-Specific AS Interfaces
*             - AudioControl Requests: only SET_CUR and GET_CUR requests are supported (for Mute)
*             - Audio Feature Unit (limited to Mute control)
*             - Audio Synchronization type: Asynchronous
*             - Single fixed audio sampling rate (configurable in usbd_conf.h file)
*          The current audio class version supports the following audio features:
*             - Pulse Coded Modulation (PCM) format
*             - sampling rate: 48KHz. 
*             - Bit resolution: 16
*             - Number of channels: 2
*             - No volume control
*             - Mute/Unmute capability
*             - Asynchronous Endpoints 
*          
* @note     In HS mode and when the DMA is used, all variables and data structures
*           dealing with the DMA during the transaction process should be 32-bit aligned.
*           
*      
*  @endverbatim
*
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
*
* Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
* You may not use this file except in compliance with the License.
* You may obtain a copy of the License at:
*
*        http://www.st.com/software_license_agreement_liberty_v2
*
* Unless required by applicable law or agreed to in writing, software 
* distributed under the License is distributed on an "AS IS" BASIS, 
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
******************************************************************************
*/ 

/* Includes ------------------------------------------------------------------*/
#include "math.h"
#include "usbd_audio.h"
#include "usbd_desc.h"
#include "usbd_ctlreq.h"
#include "vcxo.h"
#include "hd_version.h"

#define SN_Length 30
#define SEED_Length 30
extern TIM_HandleTypeDef htim7;
extern I2S_HandleTypeDef hi2s1;
extern I2S_HandleTypeDef hi2s2;
static FLASH_EraseInitTypeDef EraseInitStruct;
static int count_message_frame=0;       //usb通过此变量通知audio_mix发送信息帧
int flag_usb_read=0;    //0-usb不在读取，1-处于usb读取状态
int flag_usb_write=0;   //0-spk并不在工作，1-usb spk工作状态
int updata = 1;
static uint16_t data_u16[6];
static uint16_t data_seed30[16];
uint32_t Address = 0,PageError = 0;;
/** @addtogroup STM32_USB_DEVICE_LIBRARY
 * @{
 */


/** @defgroup USBD_AUDIO 
 * @brief usbd core module
 * @{
 */ 

/** @defgroup USBD_AUDIO_Private_TypesDefinitions
 * @{
 */ 
/**
 * @}
 */ 


/** @defgroup USBD_AUDIO_Private_Defines
 * @{
 */ 

/**
 * @}
 */ 


/** @defgroup USBD_AUDIO_Private_Macros
 * @{
 */ 
#define AUDIO_SAMPLE_FREQ(frq)      (uint8_t)(frq), (uint8_t)((frq >> 8)), (uint8_t)((frq >> 16))

#define AUDIO_PACKET_SZE_OUT(frq,ch)          (uint8_t)(((frq * BYTES_PER_SAMPLE_USB_OUT * ch)/1000) & 0xFF), \
        (uint8_t)((((frq * BYTES_PER_SAMPLE_USB_OUT * ch)/1000) >> 8) & 0xFF)
#define AUDIO_PACKET_SZE_IN(frq,ch)          (uint8_t)(((frq * BYTES_PER_SAMPLE_USB_IN * ch)/1000) & 0xFF), \
        (uint8_t)((((frq * BYTES_PER_SAMPLE_USB_IN * ch)/1000) >> 8) & 0xFF)
                                         
/**
 * @}
 */ 




/** @defgroup USBD_AUDIO_Private_FunctionPrototypes
 * @{
 */


static uint8_t  USBD_AUDIO_Init (USBD_HandleTypeDef *pdev, 
                                 uint8_t cfgidx);

static uint8_t  USBD_AUDIO_DeInit (USBD_HandleTypeDef *pdev, 
                                   uint8_t cfgidx);

static uint8_t  USBD_AUDIO_Setup (USBD_HandleTypeDef *pdev, 
                                  USBD_SetupReqTypedef *req);

static uint8_t  *USBD_AUDIO_GetCfgDesc (uint16_t *length);

static uint8_t  *USBD_AUDIO_GetDeviceQualifierDesc (uint16_t *length);

static uint8_t  USBD_AUDIO_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t  USBD_AUDIO_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t  USBD_AUDIO_EP0_RxReady (USBD_HandleTypeDef *pdev);

static uint8_t  USBD_AUDIO_EP0_TxReady (USBD_HandleTypeDef *pdev);

static uint8_t  USBD_AUDIO_SOF (USBD_HandleTypeDef *pdev);

static uint8_t  USBD_AUDIO_IsoINIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t  USBD_AUDIO_IsoOutIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum);

static void AUDIO_REQ_GetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);

static void AUDIO_REQ_SetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void HidLedRGB(uint8_t hidDataBuf[]);
static void Flash_Program_Sn(uint8_t hidDataBuf[]);
static void Flash_Program_Seed(uint8_t hidDataBuf[]);
/**
 * @}
 */ 

/** @defgroup USBD_AUDIO_Private_Variables
 * @{
 */ 

USBD_ClassTypeDef  USBD_AUDIO = 
{
    USBD_AUDIO_Init,
    USBD_AUDIO_DeInit,
    USBD_AUDIO_Setup,
    USBD_AUDIO_EP0_TxReady,  
    USBD_AUDIO_EP0_RxReady,
    USBD_AUDIO_DataIn,
    USBD_AUDIO_DataOut,
    USBD_AUDIO_SOF,
    USBD_AUDIO_IsoINIncomplete,
    USBD_AUDIO_IsoOutIncomplete,      
    USBD_AUDIO_GetCfgDesc,
    USBD_AUDIO_GetCfgDesc, 
    USBD_AUDIO_GetCfgDesc,
    USBD_AUDIO_GetDeviceQualifierDesc,
};

/* USB AUDIO device Configuration Descriptor */
#if 1   //1-tony，1-nicek
__ALIGN_BEGIN static uint8_t USBD_AUDIO_CfgDesc[USB_AUDIO_CONFIG_DESC_SIZ] __ALIGN_END =
{
    /* Configuration 1 */
    0x09,                                 /* bLength */
    USB_DESC_TYPE_CONFIGURATION,          /* bDescriptorType */
    LOBYTE(USB_AUDIO_CONFIG_DESC_SIZ),    /* wTotalLength  109 bytes*/
    HIBYTE(USB_AUDIO_CONFIG_DESC_SIZ),      
    0x04,                                 /* bNumInterfaces */
    0x01,                                 /* bConfigurationValue */
    0x00,                                 /* iConfiguration */
    0xC0,                                 /* bmAttributes  BUS Powred*/
    0x32,                                 /* bMaxPower = 100 mA*/
    /* 09 byte*/

		/*======================Usb Audio interface=========================*/	
    /* USB Speaker Standard interface descriptor */
    AUDIO_INTERFACE_DESC_SIZE,            /* bLength */
    USB_DESC_TYPE_INTERFACE,              /* bDescriptorType */
    0x00,                                 /* bInterfaceNumber */
    0x00,                                 /* bAlternateSetting */
    0x00,                                 /* bNumEndpoints */
    USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */
    AUDIO_SUBCLASS_AUDIOCONTROL,          /* bInterfaceSubClass */
    AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */
    0x00,                                 /* iInterface */
    /* 09 byte*/
  
    /* USB Speaker Class-specific AC Interface Descriptor */
    USB_AUDIO_DESC_SIZ,            												/* bLength */
    AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
    AUDIO_CONTROL_HEADER,                 /* bDescriptorSubtype */
    0x00,          /* 1.00 */             /* bcdADC */
    0x01,
    0x34,                                 /* wTotalLength = 61*/
    0x00,
    0x02,                                 /* bInCollection */
    0x01,                                 /* baInterfaceNr */
    0x02,                                 /* baInterfaceNr */
    /* 10 byte*/
  
    /* USB Speaker Input Terminal Descriptor */
    AUDIO_INPUT_TERMINAL_DESC_SIZE,       /* bLength */
    AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
    AUDIO_CONTROL_INPUT_TERMINAL,         /* bDescriptorSubtype */
    0x01,                                 /* bTerminalID */
    0x01,                                 /* wTerminalType AUDIO_TERMINAL_USB_STREAMING   0x0101 */
    0x01,
    0x00,                                 /* bAssocTerminal */
    AUDIO_OUT_EP,                         /* bNrChannels */
    0x03,                                 /* wChannelConfig 0x0003  streo */
    0x00,
    0x00,                                 /* iChannelNames */
    0x00,                                 /* iTerminal */
    /* 12 byte*/
	
    /* USB Capture Input Terminal Descriptor */
    AUDIO_INPUT_TERMINAL_DESC_SIZE,       /* bLength */
    AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
    AUDIO_CONTROL_INPUT_TERMINAL,         /* bDescriptorSubtype */
    0x02,                                 /* bTerminalID */
    0x01,                                 /* wTerminalType microphone  0x0201  mic array 0x0205*/
    0x02,
    0x00,                                 /* bAssocTerminal */
    AUDIO_IN_EP,                          /* bNrChannels */
    0x3f,                                 /* wChannelConfig 0x0000  Mono */
    0x00,
    0x00,                                 /* iChannelNames */
    0x00,                                 /* iTerminal */
    /* 12 byte*/
#if 0  
    /* USB Speaker Audio Feature Unit Descriptor */
    0x09,                                 /* bLength */
    AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
    AUDIO_CONTROL_FEATURE_UNIT,           /* bDescriptorSubtype */
    AUDIO_OUT_STREAMING_CTRL,             /* bUnitID */
    0x01,                                 /* bSourceID */
    0x01,                                 /* bControlSize */
    AUDIO_CONTROL_MUTE,// |AUDIO_CONTROL_VOLUME, /* bmaControls(0) */
    0,                                    /* bmaControls(1) */
    0x00,                                 /* iTerminal */
    /* 09 byte*/
#endif
    /*USB Speaker Output Terminal Descriptor */
    0x09,      /* bLength */
    AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
    AUDIO_CONTROL_OUTPUT_TERMINAL,        /* bDescriptorSubtype */
    0x03,                                 /* bTerminalID */
    0x01,                                 /* wTerminalType  0x0301*/
    0x03,
    0x00,                                 /* bAssocTerminal */
    0x01,                                 /* bSourceID */
    0x00,                                 /* iTerminal */
    /* 09 byte*/
  
    /*USB Capture Input Terminal Descriptor */
    0x09,      /* bLength */
    AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
    AUDIO_CONTROL_OUTPUT_TERMINAL,        /* bDescriptorSubtype */
    0x04,                                 /* bTerminalID */
    0x01,                                 /* wTerminalType  0x0101*/
    0x01,
    0x00,                                 /* bAssocTerminal */
    0x02,                                 /* bSourceID */
    0x00,                                 /* iTerminal */
    /* 09 byte*/
	
    /* USB Speaker Standard AS Interface Descriptor - Audio Streaming Zero Bandwith */
    /* Interface 1, Alternate Setting 0                                             */
    AUDIO_INTERFACE_DESC_SIZE,  /* bLength */
    USB_DESC_TYPE_INTERFACE,        /* bDescriptorType */
    0x01,                                 /* bInterfaceNumber */
    0x00,                                 /* bAlternateSetting */
    0x00,                                 /* bNumEndpoints */
    USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */
    AUDIO_SUBCLASS_AUDIOSTREAMING,        /* bInterfaceSubClass */
    AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */
    0x00,                                 /* iInterface */
    /* 09 byte*/
  
    /* USB Speaker Standard AS Interface Descriptor - Audio Streaming Operational */
    /* Interface 1, Alternate Setting 1                                           */
    AUDIO_INTERFACE_DESC_SIZE,  /* bLength */
    USB_DESC_TYPE_INTERFACE,        /* bDescriptorType */
    0x01,                                 /* bInterfaceNumber */
    0x01,                                 /* bAlternateSetting */
    0x01,                                 /* bNumEndpoints */
    USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */
    AUDIO_SUBCLASS_AUDIOSTREAMING,        /* bInterfaceSubClass */
    AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */
    0x00,                                 /* iInterface */
    /* 09 byte*/
  
    /* USB Speaker Audio Streaming Interface Descriptor */
    AUDIO_STREAMING_INTERFACE_DESC_SIZE,  /* bLength */
    AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
    AUDIO_STREAMING_GENERAL,              /* bDescriptorSubtype */
    0x01,                                 /* bTerminalLink */
    0x01,                                 /* bDelay */
    0x01,                                 /* wFormatTag AUDIO_FORMAT_PCM  0x0001*/
    0x00,
    /* 07 byte*/
  
    /* USB Speaker Audio Type I Format Interface Descriptor */
    0x0B,                                 /* bLength */
    AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
    AUDIO_STREAMING_FORMAT_TYPE,          /* bDescriptorSubtype */
    AUDIO_FORMAT_TYPE_I,                  /* bFormatType */ 
    AUDIO_OUT_CHNS,                        /* bNrChannels */
    BYTES_PER_SAMPLE_USB_OUT,                          /* bSubFrameSize :  2 Bytes per frame (16bits) */
    BITS_PER_SAMPLE_USB_OUT,          							/* bBitResolution (16-bits per sample) */ 
    0x01,                                 /* bSamFreqType only one frequency supported */ 
    AUDIO_SAMPLE_FREQ(USBD_OUT_FREQ),         /* Audio sampling frequency coded on 3 bytes */
    /* 11 byte*/
  
    /* Endpoint 1 - Standard Descriptor */
    AUDIO_STANDARD_ENDPOINT_DESC_SIZE,    /* bLength */
    USB_DESC_TYPE_ENDPOINT,               /* bDescriptorType */
    AUDIO_OUT_EP,                         /* bEndpointAddress 1 out endpoint*/
    USBD_EP_TYPE_ISOC|0x08,                    /* bmAttributes */
    AUDIO_PACKET_SZE_OUT(USBD_OUT_FREQ,AUDIO_OUT_CHNS),    /* wMaxPacketSize in Bytes (Freq(Samples)*2(Stereo)*2(HalfWord)) */
    0x01,                                 /* bInterval */
    0x00,                                 /* bRefresh */
    0x00,                                 /* bSynchAddress */
    /* 09 byte*/
  
    /* Endpoint - Audio Streaming Descriptor*/
    AUDIO_STREAMING_ENDPOINT_DESC_SIZE,   /* bLength */
    AUDIO_ENDPOINT_DESCRIPTOR_TYPE,       /* bDescriptorType */
    AUDIO_ENDPOINT_GENERAL,               /* bDescriptor */
    0x01,                                 /* bmAttributes */
    0x01,                                 /* bLockDelayUnits */
    0x01,                                 /* wLockDelay */
    0x00,
    /* 07 byte*/
	
    /*add by tony*/
    /* USB Capture Standard AS Interface Descriptor - Audio Streaming Zero Bandwith */
    /* Interface 1, Alternate Setting 0                                             */
    AUDIO_INTERFACE_DESC_SIZE,  /* bLength */
    USB_DESC_TYPE_INTERFACE,        /* bDescriptorType */
    0x02,                                 /* bInterfaceNumber */
    0x00,                                 /* bAlternateSetting */
    0x00,                                 /* bNumEndpoints */
    USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */
    AUDIO_SUBCLASS_AUDIOSTREAMING,        /* bInterfaceSubClass */
    AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */
    0x00,                                 /* iInterface */
    /* 09 byte*/
  
    /* USB Capture Standard AS Interface Descriptor - Audio Streaming Operational */
    /* Interface 1, Alternate Setting 1                                           */
    AUDIO_INTERFACE_DESC_SIZE,  /* bLength */
    USB_DESC_TYPE_INTERFACE,        /* bDescriptorType */
    0x02,                                 /* bInterfaceNumber */
    0x01,                                 /* bAlternateSetting */
    0x01,                                 /* bNumEndpoints */
    USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */
    AUDIO_SUBCLASS_AUDIOSTREAMING,        /* bInterfaceSubClass */
    AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */
    0x00,                                 /* iInterface */
    /* 09 byte*/
  
    /* USB Capture Audio Streaming Interface Descriptor */
    AUDIO_STREAMING_INTERFACE_DESC_SIZE,  /* bLength */
    AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
    AUDIO_STREAMING_GENERAL,              /* bDescriptorSubtype */
    0x04,                                 /* bTerminalLink */
    0x01,                                 /* bDelay */
    0x01,                                 /* wFormatTag AUDIO_FORMAT_PCM  0x0001*/
    0x00,
    /* 07 byte*/
  
    /* USB Capture Audio Type III Format Interface Descriptor */
    0x0B,                                 /* bLength */
    AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
    AUDIO_STREAMING_FORMAT_TYPE,          /* bDescriptorSubtype */
    AUDIO_FORMAT_TYPE_I,                  /* bFormatType */ 
    AUDIO_IN_CHNS,                         /* bNrChannels */
    BYTES_PER_SAMPLE_USB_IN,                          /* bSubFrameSize :  2 Bytes per frame (16bits) */
    BITS_PER_SAMPLE_USB_IN,                        /* bBitResolution (16-bits per sample) */ 
    0x01,                                 /* bSamFreqType only one frequency supported */ 
    AUDIO_SAMPLE_FREQ(USBD_AUDIO_FREQ),         /* Audio sampling frequency coded on 3 bytes */
    /* 11 byte*/
  
    /* Endpoint 2 - Standard Descriptor */
    AUDIO_STANDARD_ENDPOINT_DESC_SIZE,    /* bLength */
    USB_DESC_TYPE_ENDPOINT,               /* bDescriptorType */
    AUDIO_IN_EP,                         /* bEndpointAddress 1 out endpoint*/
    USBD_EP_TYPE_ISOC|0x08,                    /* bmAttributes */
    AUDIO_PACKET_SZE_IN(USBD_AUDIO_FREQ,AUDIO_IN_CHNS),    /* wMaxPacketSize in Bytes (Freq(Samples)*2(Stereo)*2(HalfWord)) */
    0x01,                                 /* bInterval */
    0x00,                                 /* bRefresh */
    0x00,                                 /* bSynchAddress */
    /* 09 byte*/
  
    /* Endpoint - Audio Streaming Descriptor*/
    AUDIO_STREAMING_ENDPOINT_DESC_SIZE,   /* bLength */
    AUDIO_ENDPOINT_DESCRIPTOR_TYPE,       /* bDescriptorType */
    AUDIO_ENDPOINT_GENERAL,               /* bDescriptor */
    0x01,                                 /* bmAttributes */
    0x00,                                 /* bLockDelayUnits */
    0x00,                                 /* wLockDelay */
    0x00,
    /* 07 byte*/	
			
		/*======================HID interface=========================*/
    /* Descriptor of Custom HID interface */
    AUDIO_INTERFACE_DESC_SIZE,         	/* bLength: Interface Descriptor size */
    USB_DESC_TYPE_INTERFACE,					 	/* bDescriptorType: Interface descriptor type */
    0x03,         											/* bInterfaceNumber: Number of Interface */
    0x00,         											/* bAlternateSetting: Alternate setting */
    0x02,         											/* bNumEndpoints */
    0x03,         											/* bInterfaceClass: HID */
    0x00,         											/* bInterfaceSubClass : 1=BOOT, 0=no boot */
    0x00,         											/* nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse */
    0,            											/* iInterface: Index of string descriptor */
	  /* 09 */
		
    /* Descriptor of Custom HID HID */
    0x09,         				/* bLength: HID Descriptor size */
    0x21, 								/* bDescriptorType: HID */
    0x10,          				/* bcdHID: HID Class Spec release number */
    0x01,				
    0x00,         				/* bCountryCode: Hardware target country */
    0x01,         				/* bNumDescriptors: Number of HID class descriptors to follow */
    0x22,         				/* bDescriptorType */
    LOBYTE(HID_REPORT_DESC_SIZ),	/* wItemLength: Total length of Report descriptor */
    HIBYTE(HID_REPORT_DESC_SIZ),
		/* 09 */
		
    /* Descriptor of Custom HID endpoints */
    0x07,          									/* bLength: Endpoint Descriptor size */
    USB_DESC_TYPE_ENDPOINT, 				/* bDescriptorType: */
    HID_IN_EP,          									/* bEndpointAddress: Endpoint Address (IN) */
    0x03,          									/* bmAttributes: Interrupt endpoint */
    HID_PACKET,          									/* wMaxPacketSize: 32 Bytes max */
    0x00,									
    0x20,          									/* bInterval: Polling Interval (32 ms) */
    /* 7 */
		
    /* Descriptor of Custom HID endpoints */
    0x07,														/* bLength: Endpoint Descriptor size */
    USB_DESC_TYPE_ENDPOINT,					/* bDescriptorType: */
    HID_OUT_EP,														/* bEndpointAddress: */
    0x03,														/* bmAttributes: Interrupt endpoint */
    HID_PACKET,														/* wMaxPacketSize: 32 Bytes max  */
    0x00,
    0x20,														/* bInterval: Polling Interval (32 ms) */
    /* 7 */
} ;
#else
__ALIGN_BEGIN static uint8_t USBD_AUDIO_CfgDesc[USB_AUDIO_CONFIG_DESC_SIZ] __ALIGN_END =
{
    0x09, /* bLength: Configuration Descriptor size */
    USB_DESC_TYPE_CONFIGURATION, /* bDescriptorType: Configuration */
    41,
    /* wTotalLength: Bytes returned */
    0x00,
    0x01,         /*bNumInterfaces: 1 interface*/
    0x01,         /*bConfigurationValue: Configuration value*/
    0x00,         /*iConfiguration: Index of string descriptor describing
                    the configuration*/
    0xC0,         /*bmAttributes: bus powered */
    0x32,         /*MaxPower 100 mA: this current is used for detecting Vbus*/
  
    /************** Descriptor of CUSTOM HID interface ****************/
    /* Descriptor of Custom HID interface */
    AUDIO_INTERFACE_DESC_SIZE,         	/* bLength: Interface Descriptor size */
    USB_DESC_TYPE_INTERFACE,					 	/* bDescriptorType: Interface descriptor type */
    0x03,         											/* bInterfaceNumber: Number of Interface */
    0x00,         											/* bAlternateSetting: Alternate setting */
    0x02,         											/* bNumEndpoints */
    0x03,         											/* bInterfaceClass: HID */
    0x00,         											/* bInterfaceSubClass : 1=BOOT, 0=no boot */
    0x00,         											/* nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse */
    0,            											/* iInterface: Index of string descriptor */
    /* 09 */
		
    /* Descriptor of Custom HID HID */
    0x09,         				/* bLength: HID Descriptor size */
    0x21, 								/* bDescriptorType: HID */
    0x10,         				/* bcdHID: HID Class Spec release number */
    0x01,				
    0x00,         				/* bCountryCode: Hardware target country */
    0x01,         				/* bNumDescriptors: Number of HID class descriptors to follow */
    0x22,         				/* bDescriptorType */
    LOBYTE(HID_REPORT_DESC_SIZ), /* wItemLength: Total length of Report descriptor */
    HIBYTE(HID_REPORT_DESC_SIZ),
    /* 09 */
		
    /* Descriptor of Custom HID endpoints */
    0x07,          									/* bLength: Endpoint Descriptor size */
    USB_DESC_TYPE_ENDPOINT, 				/* bDescriptorType: */
    HID_IN_EP,          									/* bEndpointAddress: Endpoint Address (IN) */
    0x03,          									/* bmAttributes: Interrupt endpoint */
    HID_PACKET,          									/* wMaxPacketSize: 32 Bytes max */
    0x00,									
    0x20,          									/* bInterval: Polling Interval (32 ms) */
    /* 7 */
		
    /* Descriptor of Custom HID endpoints */
    0x07,														/* bLength: Endpoint Descriptor size */
    USB_DESC_TYPE_ENDPOINT,					/* bDescriptorType: */
    HID_OUT_EP,														/* bEndpointAddress: */
    0x03,														/* bmAttributes: Interrupt endpoint */
    HID_PACKET,														/* wMaxPacketSize: 32 Bytes max  */
    0x00,
    0x20,														/* bInterval: Polling Interval (32 ms) */
    /* 7 */
} ;
#endif

__ALIGN_BEGIN static uint8_t CustomHID_ReportDescriptor[HID_REPORT_DESC_SIZ] __ALIGN_END =
  {                    
    0x06, 0x00, 0xFF,      /* USAGE_PAGE (Vendor Page: 0xFF00) */                       
    0x09, 0x01,            /* USAGE (Demo Kit)               */    
    0xa1, 0x01,            /* COLLECTION (Application)       */            
    /* 7 */
    
    /* Out Report 1 */        
    0x85, 0x01,            /*     REPORT_ID (1)		     */
    0x09, 0x01,            /*     USAGE (LED 1)	             */
    0x15, 0x00,            /*     LOGICAL_MINIMUM (0)        */          
    0x25, 0xFF,            /*     LOGICAL_MAXIMUM (1)        */           
    0x75, 0x08,            /*     REPORT_SIZE (8)            */        
    0x95, 0x1f,//0x40,//0x1F,            /*     REPORT_COUNT (2)           */       
    0xB1, 0x82,             /*    FEATURE (Data,Var,Abs,Vol) */     

    0x85, 0x01,            /*     REPORT_ID (1)              */
    0x09, 0x01,            /*     USAGE (LED 1)              */
    0x91, 0x82,            /*     OUTPUT (Data,Var,Abs,Vol)  */
    /* 27 */

    /* key Push Button */  
    0x85, 0x02,            /*     REPORT_ID (5)              */
    0x09, 0x05,            /*     USAGE (Push Button)        */      
    0x15, 0x00,            /*     LOGICAL_MINIMUM (0)        */      
    0x25, 0x01,            /*     LOGICAL_MAXIMUM (1)        */      
    0x75, 0x01,            /*     REPORT_SIZE (1)            */  
    0x81, 0x82,            /*     INPUT (Data,Var,Abs,Vol)   */   
    
    0x09, 0x05,            /*     USAGE (Push Button)        */               
    0x75, 0x01,            /*     REPORT_SIZE (1)            */           
    0xb1, 0x82,            /*     FEATURE (Data,Var,Abs,Vol) */  
         
    0x75, 0x07,            /*     REPORT_SIZE (7)            */           
    0x81, 0x83,            /*     INPUT (Cnst,Var,Abs,Vol)   */                    
    0x85, 0x02,            /*     REPORT_ID (2)              */         
                    
    0x75, 0x07,            /*     REPORT_SIZE (7)            */           
    0xb1, 0x83,            /*     FEATURE (Cnst,Var,Abs,Vol) */                      
    /* 55 */
		
    0xc0 	          			 /* END_COLLECTION */
}; /* CustomHID_ReportDescriptor */

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_AUDIO_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END=
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

/**
 * @}
 */ 

/** @defgroup USBD_AUDIO_Private_Functions
 * @{
 */ 

/**
 * @brief  USBD_AUDIO_Init
 *         Initialize the AUDIO interface
 * @param  pdev: device instance
 * @param  cfgidx: Configuration index
 * @retval status
 */
__align(4) USBD_AUDIO_HandleTypeDef pdata = {0};
//#pragma pack (4)
//USBD_AUDIO_HandleTypeDef pdata = {0};
extern DMA_HandleTypeDef hdma_spi2_rx;

static uint8_t  USBD_AUDIO_Init (USBD_HandleTypeDef *pdev, 
                                 uint8_t cfgidx)
{
    USBD_AUDIO_HandleTypeDef   *haudio;

    /* Open EP OUT */
    USBD_LL_OpenEP(pdev,
                   AUDIO_OUT_EP,
                   USBD_EP_TYPE_ISOC,
                   AUDIO_OUT_PACKET);
    /* Open EP IN */
    USBD_LL_OpenEP(pdev,
                   AUDIO_IN_EP,
                   USBD_EP_TYPE_ISOC,
                   AUDIO_IN_PACKET);
	
    USBD_LL_OpenEP(pdev,
                   HID_IN_EP,
                   USBD_EP_TYPE_INTR,
                   HID_PACKET);
    USBD_LL_OpenEP(pdev,
                   HID_OUT_EP,
                   USBD_EP_TYPE_INTR,
                   HID_PACKET);
  
    /* Allocate Audio structure */
    pdev->pClassData = &pdata;
  
    if(pdev->pClassData == NULL)
    {
        return USBD_FAIL; 
    }
    else
    {
        haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassData;
        haudio->alt_setting[0] = 0;
        haudio->alt_setting[1] = 0;
    
        /* Initialize the Audio output Hardware layer */
        if (((USBD_AUDIO_ItfTypeDef *)pdev->pUserData)->Init(USBD_AUDIO_FREQ, AUDIO_DEFAULT_VOLUME, 0) != USBD_OK)
        {
            return USBD_FAIL;
        }
        /* Prepare Out endpoint to receive 1st packet */
        USBD_LL_PrepareReceive(pdev, HID_OUT_EP, haudio->hid_out_buffer, HID_PACKET);
    }
    return USBD_OK;
}

/**
 * @brief  USBD_AUDIO_Init
 *         DeInitialize the AUDIO layer
 * @param  pdev: device instance
 * @param  cfgidx: Configuration index
 * @retval status
 */
static uint8_t  USBD_AUDIO_DeInit (USBD_HandleTypeDef *pdev, 
                                   uint8_t cfgidx)
{
  
    /* Open EP OUT */
    USBD_LL_CloseEP(pdev,
                    AUDIO_OUT_EP);
    /* Open EP OUT */
    USBD_LL_CloseEP(pdev,
                    AUDIO_IN_EP);

    USBD_LL_CloseEP(pdev, HID_IN_EP);
    USBD_LL_CloseEP(pdev, HID_OUT_EP);

    /* DeInit  physical Interface components */
    if(pdev->pClassData != NULL)
    {
        ((USBD_AUDIO_ItfTypeDef *)pdev->pUserData)->DeInit(0);
        USBD_free(pdev->pClassData);
        pdev->pClassData = NULL;
    }
  
    return USBD_OK;
}

/**
 * @brief  USBD_AUDIO_Setup
 *         Handle the AUDIO specific requests
 * @param  pdev: instance
 * @param  req: usb requests
 * @retval status
 */
static uint8_t  USBD_AUDIO_Setup (USBD_HandleTypeDef *pdev, 
                                  USBD_SetupReqTypedef *req)
{
    USBD_AUDIO_HandleTypeDef   *haudio;
    uint16_t len;
    uint8_t *pbuf;
    uint8_t ret = USBD_OK;
    haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassData;
  
    switch (req->bmRequest & USB_REQ_TYPE_MASK)
    {
    case USB_REQ_TYPE_CLASS :  
        switch (req->bRequest)
        {
        case AUDIO_REQ_GET_CUR:
            AUDIO_REQ_GetCurrent(pdev, req);
            break;
      
        case AUDIO_REQ_SET_CUR:
            AUDIO_REQ_SetCurrent(pdev, req);   
            break;
        case 0x0a:	/*set idle*/
            break;
        default:
            USBD_CtlError (pdev, req);
            ret = USBD_FAIL; 
        }
        break;
    
    case USB_REQ_TYPE_STANDARD:
        switch (req->bRequest)
        {
        case USB_REQ_GET_DESCRIPTOR:      
            if( (req->wValue >> 8) == AUDIO_DESCRIPTOR_TYPE){
                pbuf = USBD_AUDIO_CfgDesc + 18;
                len = MIN(USB_AUDIO_DESC_SIZ , req->wLength);
                USBD_CtlSendData (pdev, pbuf, len);
            }
            /* else if( (req->wValue >> 8) == HID_REPORT_DESCRIPTOR_TYPE){ */
            /*     pbuf = CustomHID_ReportDescriptor; */
            /*     len = req->wLength; */
            /*     USBD_CtlSendData (pdev, pbuf, len); */
            else if( req->wValue >> 8 == 0x22)  //CUSTOM_HID_REPORT_DESC
            {
                len = MIN(HID_REPORT_DESC_SIZ , req->wLength);
                pbuf = CustomHID_ReportDescriptor;
                USBD_CtlSendData (pdev, pbuf, len);
            }
            else if( req->wValue >> 8 == 0x21)        //CUSTOM_HID_DESCRIPTOR_TYPE
            {
                pbuf = USBD_AUDIO_CfgDesc + 183;   
                len = MIN(AUDIO_INTERFACE_DESC_SIZE , req->wLength);
                USBD_CtlSendData (pdev, pbuf, len);
            }
            break;
      
        case USB_REQ_GET_INTERFACE :
            if ( (uint8_t)req->wIndex == 1)
                USBD_CtlSendData (pdev,(uint8_t *)&(haudio->alt_setting[0]),1);
            else if ( (uint8_t)req->wIndex == 2)
                USBD_CtlSendData (pdev,(uint8_t *)&(haudio->alt_setting[1]),1);
            else
                USBD_CtlError (pdev, req);
            break;
      
        case USB_REQ_SET_INTERFACE :
            if ( (uint8_t)req->wIndex == 1){
                haudio->alt_setting[0] = (uint8_t)(req->wValue);
                if((uint8_t)(req->wValue != 0)){
                    USBD_LL_PrepareReceive(pdev,AUDIO_OUT_EP,haudio->playback_buffer,AUDIO_OUT_PACKET);
                    PCD_WritePMA_SetLimit(48*2*2);    //限制usb in写入缓存大小，防止把usb out覆盖
                    flag_usb_write = 1;
                }else{
                    USBD_LL_FlushEP(pdev, AUDIO_OUT_EP);
                    memset(haudio->playback_buffer,0,sizeof(haudio->playback_buffer));
                    PCD_WritePMA_SetLimit(1024);    //解除限制
                    flag_usb_write = 0;
                }
            }
            else if ( (uint8_t)req->wIndex == 2){
                haudio->alt_setting[1] = (uint8_t)(req->wValue);
                if((uint8_t)(req->wValue != 0)){
                    haudio->buf_cnt = 0;
                    pdata.dma_complete = 0;
                    USBD_LL_Transmit(pdev,AUDIO_IN_EP,haudio->in_buffer,AUDIO_IN_PACKET);
                    //HAL_I2S_Receive_DMA(&hi2s1,(uint16_t*)pdata.mic12_buffer,AUDIO_MIC_BUF_SIZE/sizeof(uint16_t));
                    if(HD_VERSION_ANALOG_MIC == hd_version_getVersion())
                        HAL_I2S_Receive_DMA(&hi2s2,(uint16_t*)pdata.mic34_buffer,AUDIO_MIC_BUF_SIZE/sizeof(uint16_t));
                    else if(HD_VERSION_DIGITAL_MIC == hd_version_getVersion())
                        HAL_I2S_Receive_DMA_Both(&hi2s1,&hi2s2,(uint16_t*)pdata.mic12_buffer,AUDIO_MIC_BUF_SIZE/sizeof(uint16_t),(uint16_t*)pdata.mic34_buffer,AUDIO_MIC_BUF_SIZE/sizeof(uint16_t));
                    count_message_frame = 30;
                    flag_usb_read = 1;
                }else{
                    USBD_LL_FlushEP(pdev, AUDIO_IN_EP);
                    HAL_I2S_DMAStop(&hi2s1);
                    HAL_I2S_DMAStop(&hi2s2);
                    memset(haudio->in_buffer,0,sizeof(haudio->in_buffer));
                    flag_usb_read = 0;
                }
            }
            else
                USBD_CtlError (pdev, req);
            break;      
      
        default:
            USBD_CtlError (pdev, req);
            ret = USBD_FAIL;     
        }
    }
    return ret;
}

uint8_t USBD_CUSTOM_HID_SendReport     (USBD_HandleTypeDef  *pdev, 
                                        uint8_t *report,
                                        uint16_t len)
{
    if (pdev->dev_state == USBD_STATE_CONFIGURED )
    {
        USBD_LL_Transmit (pdev, 
                          HID_IN_EP,
                          report,
                          len);
    }
    return USBD_OK;
}

/**
 * @brief  USBD_AUDIO_GetCfgDesc 
 *         return configuration descriptor
 * @param  speed : current device speed
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
static uint8_t  *USBD_AUDIO_GetCfgDesc (uint16_t *length)
{
    *length = sizeof (USBD_AUDIO_CfgDesc);
    return USBD_AUDIO_CfgDesc;
}
static int float_to_byte(float f_value, uint8_t* dst)
{
    int i;
    for(i=0;i<4;i++) {
        *(dst+i) = *(((uint8_t*)(&f_value)) + i);
    }
    return 4;
}
/*
  已知：
  USB mic:
  #define USBD_AUDIO_FREQ          //usb mic sample freq
  #define AUDIO_IN_CHNS            //usb mic channel num
  #define BYTES_PER_SAMPLE_USB_IN  //usb mic sample bytes
  #define BITS_PER_SAMPLE_USB_IN  
  USB spk:
  #define USBD_OUT_FREQ            //usb spk sample freq
  #define AUDIO_OUT_CHNS           //usb spk channel num
  #define BYTES_PER_SAMPLE_USB_OUT //usb spk sample bytes
  #define BITS_PER_SAMPLE_USB_OUT 
  MCU mic: 
  #define MCU_SAMPLE_FREQ          //mcu对真实mic的采样频率
  #define CHANNEL_NUM              //mcu的真实采样通道数（不包含2个参考通道）
  #define BYTES_PER_SAMPLE_MCU     //mcu的真实采样通道的采样位数
  #define BITS_PER_SAMPLE_MCU
  position:
  #define MIC0_X  1
  #define MIC0_Y  0
  #define MIC1_X  0
  #define MIC1_Y  1
  #define MIC2_X  -1
  #define MIC2_Y  0
  #define MIC3_X  0
  #define MIC3_Y  -1
  要求：
  信息帧传输上述所有信息；
  数据帧按通道顺序传输各个mcu mic通道、2个参考通道信号。
*/
 
void audio_info_fix(uint8_t* dst){
    int i;
    count_message_frame--;
    //0~10字节信息帧标志
    for(i=0;i<10;i++) {
        *(dst++) = (uint8_t)i;
    }
    *(dst++) = (uint8_t)SAMPLES_PER_FRAME_IN;       //第11字节
    *(dst++) = (uint8_t)AUDIO_IN_CHNS;              //第12字节
    *(dst++) = (uint8_t)BYTES_PER_SAMPLE_USB_IN;    //第13字节
    *(dst++) = (uint8_t)SAMPLES_PER_FRAME_OUT;      //第14字节
    *(dst++) = (uint8_t)AUDIO_OUT_CHNS;             //第15字节
    *(dst++) = (uint8_t)BYTES_PER_SAMPLE_USB_OUT;   //第16字节
    *(dst++) = (uint8_t)SAMPLES_PER_FRAME_MCU;      //第17字节
    *(dst++) = (uint8_t)CHANNEL_NUM;                //第18字节
    *(dst++) = (uint8_t)BYTES_PER_SAMPLE_MCU;       //第19字节
    //下面传输坐标
    dst += float_to_byte(MIC0_X, dst);      //第20、21、22、23字节
    dst += float_to_byte(MIC0_Y, dst);      //第24、25、26、27字节
    dst += float_to_byte(MIC0_Z, dst);      //第28、29、30、31字节                            
    dst += float_to_byte(MIC1_X, dst);      //第32、33、34、35字节
    dst += float_to_byte(MIC1_Y, dst);      //第36、37、38、39字节                            
    dst += float_to_byte(MIC1_Z, dst);      //第40、41、42、43字节
    dst += float_to_byte(MIC2_X, dst);      //第44、45、46、47字节                            
    dst += float_to_byte(MIC2_Y, dst);      //第48、49、50、51字节
    dst += float_to_byte(MIC2_Z, dst);      //第52、53、54、55字节                            
    dst += float_to_byte(MIC3_X, dst);      //第56、57、58、59字节
    dst += float_to_byte(MIC3_Y, dst);      //第60、61、62、63字节
    dst += float_to_byte(MIC3_Z, dst);      //第64、65、66、67字节
    dst += float_to_byte(MIC4_X, dst);      //第68、69、70、71字节
    dst += float_to_byte(MIC4_Y, dst);      //第72、73、74、75字节
    dst += float_to_byte(MIC4_Z, dst);      //第76、77、78、79字节
    dst += float_to_byte(MIC5_X, dst);      //第80、81、82、83字节
    dst += float_to_byte(MIC5_Y, dst);      //第84、85、86、87字节
    dst += float_to_byte(MIC5_Z, dst);      //第88、89、90、91字节
    //其他
    *(dst++)=(uint8_t)MIC_TYPE;     //第92字节
    {   //传输单片机序列号
        unsigned char i;
        for(i=0;i<12;i++) {     //第93字节~第104字节
            *(dst++) = *((unsigned char*)(0x0801F800) + i);
        }
    }
    //6个通道的big参数
    *(dst++) = 64;      //第105字节
    *(dst++) = 64;      //第106字节
    *(dst++) = 64;      //第107字节
    *(dst++) = 64;      //第108字节
    *(dst++) = 64;      //第109字节
    *(dst++) = 64;      //第110字节
}

void audio_mix_digital_mic(uint8_t* dst, uint8_t* ch[])
{
    int s, c;
    for(s = 0; s < SAMPLES_PER_FRAME_MCU; s++) {
        //先拷贝mic通道数据
      for(c = 0; c < (CHANNEL_NUM/2); c++) {
        //c = 0;
            /* stm32是小端存储，高位存放在高地址
               假设32位i2s产生的一个数据为 0xABCD，A是高位，D是低位，那么：
               在i2s里的传输顺序是：                                ABCD    (先传高位)
               按照32位保存，在内存中的顺序为：                     DCBA    (小端存储)
               由于DMA是按照16位传输，经过DMA传输后的顺序为：       BADC
               mic的18位数据在ABC[6:7]里，整合后仍然按照小端对齐格式输出。      */
#define OFFSET 2
            uint8_t dat[2];
            dat[0] = ((*(ch[c]+0)) << OFFSET) | ((*(ch[c]+3)) >> (8-OFFSET));       //低8位
            dat[1] = ((*(ch[c]+1)) << OFFSET) | ((*(ch[c]+0)) >> (8-OFFSET));       //高8位
            memcpy(dst, dat, 2);
            dst += 2;
            ch[c] += 4;

            dat[0] = ((*(ch[c]+0)) << OFFSET) | ((*(ch[c]+3)) >> (8-OFFSET));       //低8位
            dat[1] = ((*(ch[c]+1)) << OFFSET) | ((*(ch[c]+0)) >> (8-OFFSET));       //高8位
            memcpy(dst, dat, 2);
            dst += 2;
            ch[c] += 4;
      }    
        *(dst++)=(*(ch[2]+0));              
        *(dst++)=(*(ch[2]+1));              
        *(dst++)=(*(ch[2]+2));               
        *(dst++)=(*(ch[2]+3));
        ch[2] += 4;    
       
        //参考通道已经在usb的缓存里了，不需要拷贝
    }
    if(flag_usb_write == 0) {   //当前spk没有工作，给spk部分赋值0
        memset(dst, 0x00, AUDIO_OUT_PACKET);
    }
}
void audio_mix_analgo_mic(uint8_t* dst, uint8_t* ch[])
{
    //发送数据帧
    int s;
    for(s = 0; s < SAMPLES_PER_FRAME_MCU; s++) {
        //先拷贝mic通道数据，模拟mic仅用到ch[1]的数据
        memcpy(dst, ch[1], BYTES_PER_SAMPLE_MCU);
        dst += BYTES_PER_SAMPLE_MCU;
        memcpy(dst, ch[1]+2*BYTES_PER_SAMPLE_MCU, BYTES_PER_SAMPLE_MCU);
        dst += BYTES_PER_SAMPLE_MCU;
        memcpy(dst, ch[1]+1*BYTES_PER_SAMPLE_MCU, BYTES_PER_SAMPLE_MCU);
        dst += BYTES_PER_SAMPLE_MCU;
        memcpy(dst, ch[1]+3*BYTES_PER_SAMPLE_MCU, BYTES_PER_SAMPLE_MCU);
        dst += BYTES_PER_SAMPLE_MCU;
        ch[1] += BYTES_PER_SAMPLE_MCU*4;
    }
    if(flag_usb_write == 0) {   //当前spk没有工作，给spk部分赋值0
        memset(dst, 0x00, AUDIO_OUT_PACKET);
    }
}

/**
 * @brief  USBD_AUDIO_DataIn
 *         handle data IN Stage
 * @param  pdev: device instance
 * @param  epnum: endpoint index
 * @retval status
 */
static uint8_t  USBD_AUDIO_DataIn (USBD_HandleTypeDef *pdev, 
                                   uint8_t epnum)
{
    USBD_AUDIO_HandleTypeDef   *haudio;
    haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassData;
    //haudio->buf_cnt = (haudio->buf_cnt + 1) % AUDIO_PACKET_NUM;
    uint8_t *ch[4],*in_buf;
	
    haudio->buf_cnt = (haudio->buf_cnt + 1) % AUDIO_PACKET_NUM;
    if(abs((int)((AUDIO_MIC_BUF_SIZE -  hdma_spi2_rx.Instance->CNDTR * 2)/AUDIO_MIC_PACKET) - (int)haudio->buf_cnt) <= 1)
    	haudio->buf_cnt = (haudio->buf_cnt + AUDIO_PACKET_NUM/2) % AUDIO_PACKET_NUM;
	
    if (epnum == (AUDIO_IN_EP & 0x7F)){
    
        USBD_LL_Transmit(pdev,
                         AUDIO_IN_EP,
                         haudio->in_buffer,
                         192);//AUDIO_IN_PACKET);
		
        pdata.buf_delay = pdata.dma_complete + AUDIO_MIC_BUF_SIZE -  hdma_spi2_rx.Instance->CNDTR * 2;
        vcxo_adjust(pdata.buf_delay);

        // prepare next in buffer
        if(HD_VERSION_DIGITAL_MIC == hd_version_getVersion())
            ch[0] = haudio->mic12_buffer + haudio->buf_cnt * AUDIO_MIC_PACKET;
        ch[1] = haudio->mic34_buffer + haudio->buf_cnt * AUDIO_MIC_PACKET;
        ch[2] = haudio->playback_buffer + haudio->buf_cnt * AUDIO_OUT_PACKET;
        in_buf = haudio->in_buffer;
        
	audio_mix_digital_mic(in_buf,ch);			
        /*if(count_message_frame)
            audio_info_fix(in_buf);
        else {
            if(HD_VERSION_DIGITAL_MIC == hd_version_getVersion())
                audio_mix_digital_mic(in_buf,ch);
            else if(HD_VERSION_ANALOG_MIC == hd_version_getVersion())
                audio_mix_analgo_mic(in_buf, ch);
            {   //把数据的前32个字节加密
                int i;
                for(i=0;i<32;i++) {     //每帧开始的32字节进行加密
                    in_buf[i] ^= 0x34;
                }
            }
        }*/
		
        pdata.dma_complete -= AUDIO_MIC_PACKET;
    }
    return USBD_OK;
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if(hi2s == &hi2s2)
        pdata.dma_complete += AUDIO_MIC_BUF_SIZE;
}

/**
 * @brief  USBD_AUDIO_EP0_RxReady
 *         handle EP0 Rx Ready event
 * @param  pdev: device instance
 * @retval status
 */
static uint8_t  USBD_AUDIO_EP0_RxReady (USBD_HandleTypeDef *pdev)
{
    USBD_AUDIO_HandleTypeDef   *haudio;
    haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassData;
  
    if (haudio->control.cmd == AUDIO_REQ_SET_CUR)
    {/* In this driver, to simplify code, only SET_CUR request is managed */

        if (haudio->control.unit == AUDIO_OUT_STREAMING_CTRL)
        {
            ((USBD_AUDIO_ItfTypeDef *)pdev->pUserData)->MuteCtl(haudio->control.data[0]);     
            haudio->control.cmd = 0;
            haudio->control.len = 0;
        }
    } 

    return USBD_OK;
}
/**
 * @brief  USBD_AUDIO_EP0_TxReady
 *         handle EP0 TRx Ready event
 * @param  pdev: device instance
 * @retval status
 */
static uint8_t  USBD_AUDIO_EP0_TxReady (USBD_HandleTypeDef *pdev)
{
    /* Only OUT control data are processed */
    return USBD_OK;
}
/**
 * @brief  USBD_AUDIO_SOF
 *         handle SOF event
 * @param  pdev: device instance
 * @retval status
 */
#define SOF_FILTER_LEN 32
#define SOF_WINDOW_LEN 32

uint32_t sof_filter[SOF_FILTER_LEN] = {0} ;
uint8_t sof_cnt = 0;
uint8_t sof_window = 0;
uint32_t sof_total = 0;
uint32_t sof_len = 0;
uint32_t sof_temp = 0;

static uint8_t  USBD_AUDIO_SOF (USBD_HandleTypeDef *pdev)
{
#if 0
    sof_temp += TIM1->CNT;
    TIM1->EGR = TIM_EGR_UG;
    sof_window = (sof_window + 1) % SOF_WINDOW_LEN;
    if(!sof_window){
        sof_total += sof_temp;
        sof_total -= sof_filter[sof_cnt];
        sof_filter[sof_cnt] = sof_temp;
        sof_cnt = (sof_cnt+1)%SOF_FILTER_LEN;
        sof_len = sof_total / (SOF_FILTER_LEN * SOF_WINDOW_LEN / 2);
        sof_temp = 0; 
    }
#endif
    return USBD_OK;
}

/**
 * @brief  USBD_AUDIO_SOF
 *         handle SOF event
 * @param  pdev: device instance
 * @retval status
 */
void  USBD_AUDIO_Sync (USBD_HandleTypeDef *pdev, AUDIO_OffsetTypeDef offset)
{

}

/**
 * @brief  USBD_AUDIO_IsoINIncomplete
 *         handle data ISO IN Incomplete event
 * @param  pdev: device instance
 * @param  epnum: endpoint index
 * @retval status
 */
static uint8_t  USBD_AUDIO_IsoINIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum)
{

    return USBD_OK;
}
/**
 * @brief  USBD_AUDIO_IsoOutIncomplete
 *         handle data ISO OUT Incomplete event
 * @param  pdev: device instance
 * @param  epnum: endpoint index
 * @retval status
 */
static uint8_t  USBD_AUDIO_IsoOutIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum)
{

    return USBD_OK;
}
/**
 * @brief  USBD_AUDIO_DataOut
 *         handle data OUT Stage
 * @param  pdev: device instance
 * @param  epnum: endpoint index
 * @retval status
 */
uint8_t hid_buf[32];
static uint8_t  USBD_AUDIO_DataOut (USBD_HandleTypeDef *pdev, 
                                    uint8_t epnum)
{
    USBD_AUDIO_HandleTypeDef   *haudio;
    haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassData;
    static int cnt = 0;
    cnt = (cnt  + 1) % AUDIO_PACKET_NUM;
    if (epnum == AUDIO_OUT_EP)
    { 
        /* Prepare Out endpoint to receive next audio packet */
        USBD_LL_PrepareReceive(pdev,
                               AUDIO_OUT_EP,
                               haudio->playback_buffer + cnt * AUDIO_OUT_PACKET, 
                               AUDIO_OUT_PACKET); 
    } else if (epnum == HID_OUT_EP) {
      USBD_LL_PrepareReceive(pdev, HID_OUT_EP, hid_buf, HID_PACKET);
      /*********************************************************
      windows hid 驱动 hid_buf[0] = 0x01  report id 
                       hid_buf[1] = 0xaa  
                       hid_buf[2] ..hid_buf[18] sn号
      16位sn号 保存在0x0801F000开始 62page
      *********************************************************/
       if(hid_buf[0] == 0x01 && hid_buf[1] == 0xaa)
       {
         Flash_Program_Sn(hid_buf);
       }
      /*********************************************************
      windows hid 驱动 hid_buf[0] = 0x01  report id 
                       hid_buf[1] = 0xbb  
                       hid_buf[2] ..hid_buf[32] seed号
      30位seed号，保存在0x0801f800开始的 63page
      *********************************************************/
       else if(hid_buf[0] == 0x01 && hid_buf[1] == 0xbb)
       {
         Flash_Program_Seed(hid_buf);
       }
      /*********************************************************
      windows hid 驱动 hid_buf[0] = 0x01  report id 
                       hid_buf[1] = 0xcc  
       读取sn号
      *********************************************************/
       else if(hid_buf[0] == 0x01 && hid_buf[1] == 0xcc)
       {
         memset(hid_buf,0,32);
         hid_buf[0] = 0x02;
         for(uint8_t i =0;i<SN_Length;i++)
         {
           hid_buf[i+1] = *((unsigned char*)(0x0801F000) + i);
         }
         USBD_CUSTOM_HID_SendReport(pdev, hid_buf, HID_PACKET);
       }
      /*********************************************************
      windows hid 驱动 hid_buf[0] = 0x01  report id 
                       hid_buf[1] = 0xdd  
       读取seed号
      *********************************************************/
       else if(hid_buf[0] == 0x01 && hid_buf[1] == 0xdd)
       {
         memset(hid_buf,0,32);
         hid_buf[0] = 0x02;
         for(uint8_t i =0;i<SEED_Length;i++)
         {
           hid_buf[i+1] = *((unsigned char*)(0x0801F800) + i);
         }
         USBD_CUSTOM_HID_SendReport(pdev, hid_buf, HID_PACKET);
       }
      /*********************************************************
       hid_buf[0] hid_buf[1] hid_buf[2] hid_buf[3]
       00        r--ff/00    g-ff/00    b--ff/00
       hid_buf[4] hid_buf[5] hid_buf[6] hid_buf[7]
       00        r--ff/00    g-ff/00    b--ff/00  
       hid_buf[8] hid_buf[9] hid_buf[10] hid_buf[11]
       00        r--ff/00    g-ff/00    b--ff/00 
       hid_buf[12] hid_buf[13] hid_buf[14] hid_buf[15]
       00        r--ff/00    g-ff/00    b--ff/00 
      *********************************************************/
       else
       {
         //USBD_LL_PrepareReceive(pdev, HID_OUT_EP, hid_buf, HID_PACKET);
	       HidLedRGB(hid_buf);
         memset(hid_buf,0,32);
       }
				// modify hid report id 
	/*haudio->hid_out_buffer[0] = 2;
	sprintf((char*)haudio->hid_out_buffer+1,"recv:%s tick:%d",hid_buf,HAL_GetTick());
        USBD_CUSTOM_HID_SendReport(pdev, haudio->hid_out_buffer, HID_PACKET);
        memcpy(hid_buf, haudio->hid_out_buffer, HID_PACKET);
       */
    }
  
    return USBD_OK;
}

/**
 * @brief  AUDIO_Req_GetCurrent
 *         Handles the GET_CUR Audio control request.
 * @param  pdev: instance
 * @param  req: setup class request
 * @retval status
 */
static void AUDIO_REQ_GetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{  
    USBD_AUDIO_HandleTypeDef   *haudio;
    haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassData;
  
    memset(haudio->control.data, 0, 64);
    /* Send the current mute state */
    USBD_CtlSendData (pdev, 
                      haudio->control.data,
                      req->wLength);
}

/**
 * @brief  AUDIO_Req_SetCurrent
 *         Handles the SET_CUR Audio control request.
 * @param  pdev: instance
 * @param  req: setup class request
 * @retval status
 */
static void AUDIO_REQ_SetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{ 
    USBD_AUDIO_HandleTypeDef   *haudio;
    haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassData;
  
    if (req->wLength)
    {
        /* Prepare the reception of the buffer over EP0 */
        USBD_CtlPrepareRx (pdev,
                           haudio->control.data,                                  
                           req->wLength);    
    
        haudio->control.cmd = AUDIO_REQ_SET_CUR;     /* Set the request value */
        haudio->control.len = req->wLength;          /* Set the request data length */
        haudio->control.unit = HIBYTE(req->wIndex);  /* Set the request target unit */
    }
}


/**
 * @brief  DeviceQualifierDescriptor 
 *         return Device Qualifier descriptor
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
static uint8_t  *USBD_AUDIO_GetDeviceQualifierDesc (uint16_t *length)
{
    *length = sizeof (USBD_AUDIO_DeviceQualifierDesc);
    return USBD_AUDIO_DeviceQualifierDesc;
}

/**
 * @brief  USBD_AUDIO_RegisterInterface
 * @param  fops: Audio interface callback
 * @retval status
 */
uint8_t  USBD_AUDIO_RegisterInterface  (USBD_HandleTypeDef   *pdev, 
                                        USBD_AUDIO_ItfTypeDef *fops)
{
    if(fops != NULL)
    {
        pdev->pUserData= fops;
    }
    return 0;
}
/**
 * @}
 */ 

void HidLedRGB(uint8_t hidDataBuf[])
{

 //  4 led rgb
 // 0x00 off /0xff on
 // 0xff ff ff ff   α(无效位)r  g  b
 // hiddatabuff[0] 为无效数据
    if(hidDataBuf[0] == 0x00)
    {
       
    }
    else if(hidDataBuf[0] == 0xff)
    {
        
    }
    if(hidDataBuf[1] == 0x00)
    { 
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	     // printf("%d ",hidDataBuf[1]);
    }
    else if(hidDataBuf[1] == 0xff)
    {
      if(updata  == 1)
	    {
		    HAL_TIM_Base_Stop_IT(&htim7);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
                 
	   }
	   updata = 0;
     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	   //printf("%d ",hidDataBuf[1]);
    }
    if(hidDataBuf[2] == 0x00)
    {

       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
	     //printf("%d ",hidDataBuf[2]);
    }
    else if(hidDataBuf[2] == 0xff)
    {
      if(updata  == 1)
	    {
		    HAL_TIM_Base_Stop_IT(&htim7);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
                 
	    }
	    updata = 0;
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
	    //printf("%d ",hidDataBuf[2]);
    }	
   if(hidDataBuf[3] == 0x00)
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
	    //printf("%d ",hidDataBuf[3]);
    }
	 else if(hidDataBuf[3] == 0xff)
   {
			if(updata  == 1)
	    {
		    HAL_TIM_Base_Stop_IT(&htim7);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);            
	    }
	    updata = 0;
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
	    //printf("%d ",hidDataBuf[3]);
   }
    if(hidDataBuf[4] == 0x00)
    {
       
    }
	 else if(hidDataBuf[4] == 0xff)
    {
       
    }
	if(hidDataBuf[5] == 0x00)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	  //printf("%d ",hidDataBuf[5]);
  }
	else if(hidDataBuf[5] == 0xff)
  {
	 if(updata  == 1)
	 {
		 HAL_TIM_Base_Stop_IT(&htim7);
     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);             
	 }
	 updata = 0;
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	 //printf("%d ",hidDataBuf[5]);
  }
	if(hidDataBuf[6] == 0x00)
  {
     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
	   //printf("%d ",hidDataBuf[6]);
  }
	else if(hidDataBuf[6] == 0xff)
  {
	 if(updata  == 1)
	 {
		 HAL_TIM_Base_Stop_IT(&htim7);
     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);              
	 }
	 updata = 0;
   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
	 //printf("%d ",hidDataBuf[6]);
  }	
	if(hidDataBuf[7] == 0x00)
  {
   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
	 //printf("%d ",hidDataBuf[7]);
  }
	else if(hidDataBuf[7] == 0xff)
  {
   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
	 //printf("%d ",hidDataBuf[7]);
  }
  if(hidDataBuf[8] == 0x00)
  {
       
  }
	else if(hidDataBuf[8] == 0xff)
  {
       
  }
	if(hidDataBuf[9] == 0x00)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	  //printf("%d ",hidDataBuf[9]);
  }
	else if(hidDataBuf[9] == 0xff)
  {
	  if(updata  == 1)
	  {
		 HAL_TIM_Base_Stop_IT(&htim7);
     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
                 
	  }
	  updata = 0;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	  //printf("%d ",hidDataBuf[9]);
  }
	if(hidDataBuf[10] == 0x00)
  {
     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
	   //printf("%d ",hidDataBuf[10]);
  }
	else if(hidDataBuf[10] == 0xff)
  {
		if(updata  == 1)
	  {
		 HAL_TIM_Base_Stop_IT(&htim7);
     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
                 
	  }
	  updata = 0;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
	  //printf("%d ",hidDataBuf[10]);
  }	
	if(hidDataBuf[11] == 0x00)
  {
     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
	   //printf("%d ",hidDataBuf[11]);
  }
	else if(hidDataBuf[11] == 0xff)
  {
	 if(updata  == 1)
	 {
		 HAL_TIM_Base_Stop_IT(&htim7);
     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);             
	 }
	  updata = 0;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
	  //printf("%d ",hidDataBuf[11]);
   }


	if(hidDataBuf[12] == 0x00)
  {
       
  }
	else if(hidDataBuf[12] == 0xff)
  {
       
  }
	if(hidDataBuf[13] == 0x00)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	  //printf("%d ",hidDataBuf[13]);
  }
	else if(hidDataBuf[13] == 0xff)
  {
	 if(updata  == 1)
	 {
		 HAL_TIM_Base_Stop_IT(&htim7);
     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);            
	 }
	   updata = 0;
     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	   //printf("%d ",hidDataBuf[13]);
  }
	if(hidDataBuf[14] == 0x00)
  {
     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
	   //printf("%d ",hidDataBuf[14]);
   }
	else if(hidDataBuf[14] == 0xff)
  {
   if(updata  == 1)
	 {
		 HAL_TIM_Base_Stop_IT(&htim7);
     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);                 
	 }
	  updata = 0;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
		//printf("%d ",hidDataBuf[14]);
  }	
	if(hidDataBuf[15] == 0x00)
  {
     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
	   //printf("%d ",hidDataBuf[15]);
  }
	else if(hidDataBuf[15] == 0xff)
  {
	 if(updata  == 1)
	 {
		 HAL_TIM_Base_Stop_IT(&htim7);
     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);                 
	 }
	 updata = 0;
   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
	 //printf("%d ",hidDataBuf[15]);
  }	
}

/**
 * @}
 */ 

void Flash_Program_Sn(uint8_t hidDataBuf[])
{
  uint8_t tmp_data;
  for(uint8_t i=0;i<(SN_Length+2);i++)
  {
    hidDataBuf[i] = hidDataBuf[i+2];
  }
  HAL_FLASH_Unlock();
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.PageAddress = ADDR_FLASH_PAGE_62;
 // EraseInitStruct.NbPages = (FLASH_USER_END_ADDR - FLASH_USER_START_ADDR) / FLASH_PAGE_SIZE;
  EraseInitStruct.NbPages = 1;
  if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
  {
  }
   Address = ADDR_FLASH_PAGE_62;
  for(uint8_t i =0;i<SN_Length/2;i++)
  {
    tmp_data = hidDataBuf[i*2];
    hidDataBuf[i*2] = hidDataBuf[i*2+1];
    hidDataBuf[i*2+1] = tmp_data;
  }
  for(uint8_t i = 0;i<SN_Length/2;i++)
  {
    data_u16[i] = hidDataBuf[i*2]<<8|hidDataBuf[i*2+1];
  }
  
  for(uint8_t i =0;i<SN_Length/2;i++)
  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, Address, data_u16[i]) == HAL_OK)
  {
      Address = Address + 2;
  }
  /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();
  /*for(uint8_t i =0;i<SN_Length;i++)
  {
     printf("%02x ",*((unsigned char*)(0x0801F000) + i));
  }
  printf("sn:ok\n");
  */
  
}

void Flash_Program_Seed(uint8_t hidDataBuf[])
{
 
  uint8_t tmp_data;
  for(uint8_t i=0;i<(SEED_Length+2);i++)
  {
    hidDataBuf[i] = hidDataBuf[i+2];
  }
  HAL_FLASH_Unlock();
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.PageAddress = ADDR_FLASH_PAGE_63;
 // EraseInitStruct.NbPages = (FLASH_USER_END_ADDR - FLASH_USER_START_ADDR) / FLASH_PAGE_SIZE;
  EraseInitStruct.NbPages = 1;
  if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
  {
  }
   Address = ADDR_FLASH_PAGE_63;
  for(uint8_t i =0;i<SEED_Length/2;i++)
  {
    tmp_data = hidDataBuf[i*2];
    hidDataBuf[i*2] = hidDataBuf[i*2+1];
    hidDataBuf[i*2+1] = tmp_data;
  }
  for(uint8_t i = 0;i<SEED_Length/2;i++)
  {
    data_seed30[i] = hidDataBuf[i*2]<<8|hidDataBuf[i*2+1];
  }
  
  for(uint8_t i =0;i<SEED_Length/2;i++)
  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, Address, data_seed30[i]) == HAL_OK)
  {
      Address = Address + 2;
  }
  /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();
  /*for(uint8_t i =0;i<SEED_Length;i++)
  {
     printf("%02x ",*((unsigned char*)(0x0801F800) + i));
  }
  printf("seed:ok\n");*/
  
  
}

/**
 * @}
 */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
