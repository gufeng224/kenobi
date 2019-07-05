/**
  ******************************************************************************
  * @file    usbd_audio.h
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   header file for the usbd_audio.c file.
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
 
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_AUDIO_H
#define __USB_AUDIO_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include  "usbd_ioreq.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */
  
/** @defgroup USBD_AUDIO
  * @brief This file is the Header file for usbd_audio.c
  * @{
  */ 


/** @defgroup USBD_AUDIO_Exported_Defines
  * @{
  */ 
#define AUDIO_OUT_EP                                  0x02
#define AUDIO_IN_EP                                   0x81
#define HID_OUT_EP                                  	0x03
#define HID_IN_EP                                   	0x83
#define HID_PACKET                                  	0x20
//#define HID_PACKET                                  	0x40  
#define USB_AUDIO_CONFIG_DESC_SIZ                     206
#define HID_REPORT_DESC_SIZ               						56
#define AUDIO_INTERFACE_DESC_SIZE                     9
#define USB_AUDIO_DESC_SIZ                            0x0a
#define AUDIO_STANDARD_ENDPOINT_DESC_SIZE             0x09
#define AUDIO_STREAMING_ENDPOINT_DESC_SIZE            0x07

#define AUDIO_DESCRIPTOR_TYPE                         0x21
#define HID_REPORT_DESCRIPTOR_TYPE                    0x22
#define USB_DEVICE_CLASS_AUDIO                        0x01
#define AUDIO_SUBCLASS_AUDIOCONTROL                   0x01
#define AUDIO_SUBCLASS_AUDIOSTREAMING                 0x02
#define AUDIO_PROTOCOL_UNDEFINED                      0x00
#define AUDIO_STREAMING_GENERAL                       0x01
#define AUDIO_STREAMING_FORMAT_TYPE                   0x02

/* Audio Descriptor Types */
#define AUDIO_INTERFACE_DESCRIPTOR_TYPE               0x24
#define AUDIO_ENDPOINT_DESCRIPTOR_TYPE                0x25

/* Audio Control Interface Descriptor Subtypes */
#define AUDIO_CONTROL_HEADER                          0x01
#define AUDIO_CONTROL_INPUT_TERMINAL                  0x02
#define AUDIO_CONTROL_OUTPUT_TERMINAL                 0x03
#define AUDIO_CONTROL_FEATURE_UNIT                    0x06

#define AUDIO_INPUT_TERMINAL_DESC_SIZE                0x0C
#define AUDIO_OUTPUT_TERMINAL_DESC_SIZE               0x09
#define AUDIO_STREAMING_INTERFACE_DESC_SIZE           0x07

#define AUDIO_CONTROL_MUTE                            0x0001

#define AUDIO_FORMAT_TYPE_I                           0x01
#define AUDIO_FORMAT_TYPE_III                         0x03

#define AUDIO_ENDPOINT_GENERAL                        0x01

#define AUDIO_REQ_GET_CUR                             0x81
#define AUDIO_REQ_SET_CUR                             0x01

#define AUDIO_OUT_STREAMING_CTRL                      0x02



/************* USB mic *************/
#define USBD_AUDIO_FREQ                                 I2S_AUDIOFREQ_48K      //usb mic sample freq
#define AUDIO_IN_CHNS                                   0x2                     //usb mic channel num
#define BYTES_PER_SAMPLE_USB_IN                         2                       //usb mic sample bytes
#define BITS_PER_SAMPLE_USB_IN                          (BYTES_PER_SAMPLE_USB_IN * 8)
/************* USB spk *************/
#define USBD_OUT_FREQ                                   I2S_AUDIOFREQ_16K       //usb spk sample freq
#define AUDIO_OUT_CHNS                                  0x2                     //usb spk channel num
#define BYTES_PER_SAMPLE_USB_OUT                        2                       //usb spk sample bytes
#define BITS_PER_SAMPLE_USB_OUT                         (BYTES_PER_SAMPLE_USB_OUT * 8)
/************* MCU mic *************/
#define MCU_SAMPLE_FREQ                                 I2S_AUDIOFREQ_16K       //mcu对真实mic的采样频率
#define CHANNEL_NUM                                     4                       //mcu的真实采样通道数（不包含2个参考通道）
#define BYTES_PER_SAMPLE_MCU                            2                       //mcu的真实采样通道的采样位数
#define BITS_PER_SAMPLE_MCU                             (BYTES_PER_SAMPLE_MCU * 8)
/************* frame *************/
#define SAMPLES_PER_FRAME_IN                            ((uint32_t)((USBD_AUDIO_FREQ ) /1000))  //每帧（每次USB读取，1ms一次）USB mic的采样次数
#define SAMPLES_PER_FRAME_OUT                           ((uint32_t)((USBD_OUT_FREQ ) /1000))    //每帧（每次USB读取，1ms一次）USB spk的采样次数
#define SAMPLES_PER_FRAME_MCU                           ((uint32_t)((MCU_SAMPLE_FREQ ) /1000))    //每帧（每次USB读取，1ms一次）单片机实际采样通道的采样次数
/************* size & buf ***********/
#define AUDIO_IN_PACKET                              	((uint32_t)(SAMPLES_PER_FRAME_IN * BYTES_PER_SAMPLE_USB_IN * AUDIO_IN_CHNS))   //usb mic 每frame读取的数据量
#define AUDIO_OUT_PACKET                                ((uint32_t)(SAMPLES_PER_FRAME_OUT * BYTES_PER_SAMPLE_USB_OUT * AUDIO_OUT_CHNS)) //usb spk 每frame输入的数据量
//#define AUDIO_MIC_PACKET                                ((uint32_t)(SAMPLES_PER_FRAME_MCU * 2 * BYTES_PER_SAMPLE_MCU))     //2个mic在1ms产生的数据量
#define AUDIO_MIC_PACKET                                ((uint32_t)(SAMPLES_PER_FRAME_MCU * 2 * BYTES_PER_SAMPLE_MCU * 2))      //for 32bits
/************* position *************/
#define MIC0_X  0.0645
#define MIC0_Y  0.0
#define MIC0_Z  0.0
#define MIC1_X  0.0215
#define MIC1_Y  0.0
#define MIC1_Z  0.0
#define MIC2_X  -0.0215
#define MIC2_Y  0.0
#define MIC2_Z  0.0
#define MIC3_X  -0.0645
#define MIC3_Y  0.0
#define MIC3_Z  0.0
#define MIC4_X  0.0
#define MIC4_Y  0.0
#define MIC4_Z  0.0
#define MIC5_X  0.0
#define MIC5_Y  0.0
#define MIC5_Z  0.0
/****************** other ***************/
#define MIC_TYPE        0       //0-使用的是数字mic，1-使用的是模拟mic

/* Number of sub-packets in the audio transfer buffer. You can modify this value but always make sure
  that it is an even number and higher than 3 */
#define AUDIO_PACKET_NUM                                8
/* Total size of the audio transfer buffer */
#define AUDIO_OUT_BUF_SIZE                          ((uint32_t)(AUDIO_OUT_PACKET * AUDIO_PACKET_NUM))
#define AUDIO_IN_BUF_SIZE                           ((uint32_t)(AUDIO_IN_PACKET))
#define AUDIO_MIC_BUF_SIZE                      		((uint32_t)(AUDIO_MIC_PACKET * AUDIO_PACKET_NUM))      //2路mic的buf



#if (BYTES_PER_SAMPLE_MCU == 2)
	#define I2S_DATAFORMAT															I2S_DATAFORMAT_16B_EXTENDED
#elif (BYTES_PER_SAMPLE_MCU == 3)
  #define I2S_DATAFORMAT															I2S_DATAFORMAT_24B
#elif (BYTES_PER_SAMPLE_MCU == 4)
  #define I2S_DATAFORMAT															I2S_DATAFORMAT_32B
#endif


#define AUDIO_DEFAULT_VOLUME                          70
    
    
    /* Audio Commands enumeration */
typedef enum
{
  AUDIO_CMD_START = 1,
  AUDIO_CMD_PLAY,
  AUDIO_CMD_STOP,
}AUDIO_CMD_TypeDef;


typedef enum
{
  AUDIO_OFFSET_NONE = 0,
  AUDIO_OFFSET_HALF,
  AUDIO_OFFSET_FULL,  
  AUDIO_OFFSET_UNKNOWN,    
}
AUDIO_OffsetTypeDef;
/**
  * @}
  */ 


/** @defgroup USBD_CORE_Exported_TypesDefinitions
  * @{
  */
 typedef struct
{
   uint8_t cmd;   
   uint8_t data[USB_MAX_EP0_SIZE];  
   uint8_t len;  
   uint8_t unit;    
}
USBD_AUDIO_ControlTypeDef; 



typedef struct
{
	uint8_t										mic12_buffer[AUDIO_MIC_BUF_SIZE];
  uint8_t										mic34_buffer[AUDIO_MIC_BUF_SIZE];
	uint8_t                   in_buffer[AUDIO_IN_BUF_SIZE];
	uint8_t										playback_buffer[AUDIO_OUT_BUF_SIZE];
	uint8_t										buf_cnt;
	uint8_t             			alt_setting[2]; 
	int16_t										dma_complete;
	int16_t										buf_diff;
	int16_t									  buf_delay;
    //hid
    uint8_t             hid_in_buffer[HID_PACKET];
    uint8_t             hid_out_buffer[HID_PACKET];

  USBD_AUDIO_ControlTypeDef control;   
}
USBD_AUDIO_HandleTypeDef; 


typedef struct
{
    int8_t  (*Init)         (uint32_t  AudioFreq, uint32_t Volume, uint32_t options);
    int8_t  (*DeInit)       (uint32_t options);
    int8_t  (*AudioCmd)     (uint8_t* pbuf, uint32_t size, uint8_t cmd);
    int8_t  (*VolumeCtl)    (uint8_t vol);
    int8_t  (*MuteCtl)      (uint8_t cmd);
    int8_t  (*PeriodicTC)   (uint8_t cmd);
    int8_t  (*GetState)     (void);
}USBD_AUDIO_ItfTypeDef;
/**
  * @}
  */ 



/** @defgroup USBD_CORE_Exported_Macros
  * @{
  */ 

/**
  * @}
  */ 

/** @defgroup USBD_CORE_Exported_Variables
  * @{
  */ 

extern USBD_ClassTypeDef  USBD_AUDIO;
#define USBD_AUDIO_CLASS    &USBD_AUDIO
/**
  * @}
  */ 

/** @defgroup USB_CORE_Exported_Functions
  * @{
  */ 
uint8_t  USBD_AUDIO_RegisterInterface  (USBD_HandleTypeDef   *pdev, 
                                        USBD_AUDIO_ItfTypeDef *fops);

void  USBD_AUDIO_Sync (USBD_HandleTypeDef *pdev, AUDIO_OffsetTypeDef offset);
/**
  * @}
  */ 

#ifdef __cplusplus
}
#endif

#endif  /* __USB_AUDIO_H */
/**
  * @}
  */ 

/**
  * @}
  */ 
  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
