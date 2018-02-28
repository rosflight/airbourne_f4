/**
  ******************************************************************************
  * @file    usb_core.h
  * @author  MCD Application Team
  * @version V2.1.0
  * @date    19-March-2012
  * @brief   Header of the Core Layer
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
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
#ifndef __USB_CORE_H__
#define __USB_CORE_H__

/* Includes ------------------------------------------------------------------*/
#include "usb_conf.h"
#include "usb_regs.h"
#include "usb_defines.h"


/** @addtogroup USB_OTG_DRIVER
  * @{
  */
  
/** @defgroup USB_CORE
  * @brief usb otg driver core layer
  * @{
  */ 


/** @defgroup USB_CORE_Exported_Defines
  * @{
  */ 

#define USB_OTG_EP0_IDLE                          0
#define USB_OTG_EP0_SETUP                         1
#define USB_OTG_EP0_DATA_IN                       2
#define USB_OTG_EP0_DATA_OUT                      3
#define USB_OTG_EP0_STATUS_IN                     4
#define USB_OTG_EP0_STATUS_OUT                    5
#define USB_OTG_EP0_STALL                         6

#define USB_OTG_EP_TX_DIS       0x0000
#define USB_OTG_EP_TX_STALL     0x0010
#define USB_OTG_EP_TX_NAK       0x0020
#define USB_OTG_EP_TX_VALID     0x0030
 
#define USB_OTG_EP_RX_DIS       0x0000
#define USB_OTG_EP_RX_STALL     0x1000
#define USB_OTG_EP_RX_NAK       0x2000
#define USB_OTG_EP_RX_VALID     0x3000
/**
  * @}
  */ 
#define   MAX_DATA_LENGTH                        0x200

/** @defgroup USB_CORE_Exported_Types
  * @{
  */ 


typedef enum {
  USB_OTG_OK = 0,
  USB_OTG_FAIL
}USB_OTG_STS;

typedef enum {
  HC_IDLE = 0,
  HC_XFRC,
  HC_HALTED,
  HC_NAK,
  HC_NYET,
  HC_STALL,
  HC_XACTERR,  
  HC_BBLERR,   
  HC_DATATGLERR,  
}HC_STATUS;

typedef enum {
  URB_IDLE = 0,
  URB_DONE,
  URB_NOTREADY,
  URB_ERROR,
  URB_STALL
}URB_STATE;

typedef enum {
  CTRL_START = 0,
  CTRL_XFRC,
  CTRL_HALTED,
  CTRL_NAK,
  CTRL_STALL,
  CTRL_XACTERR,  
  CTRL_BBLERR,   
  CTRL_DATATGLERR,  
  CTRL_FAIL
}CTRL_STATUS;


typedef struct USB_OTG_hc
{
  uint8_t       dev_addr ;
  uint8_t       ep_num;
  uint8_t       ep_is_in;
  uint8_t       speed;
  uint8_t       do_ping;  
  uint8_t       ep_type;
  uint16_t      max_packet;
  uint8_t       data_pid;
  uint8_t       *xfer_buff;
  uint32_t      xfer_len;
  uint32_t      xfer_count;  
  uint8_t       toggle_in;
  uint8_t       toggle_out;
  uint32_t       dma_addr;  
}
USB_OTG_HC , *PUSB_OTG_HC;

typedef struct USB_OTG_ep
{
  uint8_t        num;
  uint8_t        is_in;
  uint8_t        is_stall;  
  uint8_t        type;
  uint8_t        data_pid_start;
  uint8_t        even_odd_frame;
  uint16_t       tx_fifo_num;
  uint32_t       maxpacket;
  /* transaction level variables*/
  uint8_t        *xfer_buff;
  uint32_t       dma_addr;  
  uint32_t       xfer_len;
  uint32_t       xfer_count;
  /* Transfer level variables*/  
  uint32_t       rem_data_len;
  uint32_t       total_data_len;
  uint32_t       ctl_data_len;  

}

USB_OTG_EP , *PUSB_OTG_EP;



typedef struct USB_OTG_core_cfg
{
  uint8_t       host_channels;
  uint8_t       dev_endpoints;
  uint8_t       speed;
  uint8_t       dma_enable;
  uint16_t      mps;
  uint16_t      TotalFifoSize;
  uint8_t       phy_itface;
  uint8_t       Sof_output;
  uint8_t       low_power;
  uint8_t       coreID;
 
}
USB_OTG_CORE_CFGS, *PUSB_OTG_CORE_CFGS;



typedef  struct  usb_setup_req {
    
    uint8_t   bmRequest;                      
    uint8_t   bRequest;                           
    uint16_t  wValue;                             
    uint16_t  wIndex;                             
    uint16_t  wLength;                            
} USB_SETUP_REQ;

typedef struct _Device_TypeDef
{
  uint8_t  *(*GetDeviceDescriptor)( uint8_t speed , uint16_t *length);  
  uint8_t  *(*GetLangIDStrDescriptor)( uint8_t speed , uint16_t *length); 
  uint8_t  *(*GetManufacturerStrDescriptor)( uint8_t speed , uint16_t *length);  
  uint8_t  *(*GetProductStrDescriptor)( uint8_t speed , uint16_t *length);  
  uint8_t  *(*GetSerialStrDescriptor)( uint8_t speed , uint16_t *length);  
  uint8_t  *(*GetConfigurationStrDescriptor)( uint8_t speed , uint16_t *length);  
  uint8_t  *(*GetInterfaceStrDescriptor)( uint8_t speed , uint16_t *length);   
} USBD_DEVICE, *pUSBD_DEVICE;

//typedef struct USB_OTG_hPort
//{
//  void (*Disconnect) (void *phost);
//  void (*Connect) (void *phost); 
//  uint8_t ConnStatus;
//  uint8_t DisconnStatus;
//  uint8_t ConnHandled;
//  uint8_t DisconnHandled;
//} USB_OTG_hPort_TypeDef;

typedef struct _Device_cb
{
  uint8_t  (*Init)         (volatile void *pdev , uint8_t cfgidx);
  uint8_t  (*DeInit)       (volatile void *pdev , uint8_t cfgidx);
 /* Control Endpoints*/
  uint8_t  (*Setup)        (volatile void *pdev , USB_SETUP_REQ  *req);
  uint8_t  (*EP0_TxSent)   (volatile void *pdev );
  uint8_t  (*EP0_RxReady)  (volatile void *pdev );
  /* Class Specific Endpoints*/
  uint8_t  (*DataIn)       (volatile void *pdev , uint8_t epnum);
  uint8_t  (*DataOut)      (volatile void *pdev , uint8_t epnum);
  uint8_t  (*SOF)          (volatile void *pdev);
  uint8_t  (*IsoINIncomplete)  (volatile void *pdev);
  uint8_t  (*IsoOUTIncomplete)  (volatile void *pdev);

  uint8_t  *(*GetConfigDescriptor)( uint8_t speed , uint16_t *length); 
#ifdef USB_OTG_HS_CORE 
  uint8_t  *(*GetOtherConfigDescriptor)( uint8_t speed , uint16_t *length);   
#endif

#ifdef USB_SUPPORT_USER_STRING_DESC 
  uint8_t  *(*GetUsrStrDescriptor)( uint8_t speed ,uint8_t index,  uint16_t *length);   
#endif  
  
} USBD_Class_cb_TypeDef;



typedef struct _USBD_USR_PROP
{
  void (*Init)(void);   
  void (*DeviceReset)(uint8_t speed); 
  void (*DeviceConfigured)(void);
  void (*DeviceSuspended)(void);
  void (*DeviceResumed)(void);  
  
  void (*DeviceConnected)(void);  
  void (*DeviceDisconnected)(void);    
  
}
USBD_Usr_cb_TypeDef;

typedef struct _DCD
{
  uint8_t        device_config;
  uint8_t        device_state;
  uint8_t        device_status;
  uint8_t        device_old_status;
  uint8_t        device_address;
  uint8_t        connection_status;  
  uint8_t        test_mode;
  uint32_t       DevRemoteWakeup;
  USB_OTG_EP     in_ep   [USB_OTG_MAX_TX_FIFOS];
  USB_OTG_EP     out_ep  [USB_OTG_MAX_TX_FIFOS];
  uint8_t        setup_packet [8*3];
  USBD_Class_cb_TypeDef         *class_cb;
  USBD_Usr_cb_TypeDef           *usr_cb;
  USBD_DEVICE                   *usr_device;  
  uint8_t        *pConfig_descriptor;
 }
DCD_DEV , *DCD_PDEV;


typedef struct _HCD
{
  uint8_t                  Rx_Buffer [MAX_DATA_LENGTH];  
  __IO uint32_t            ConnSts;
  __IO uint32_t            ErrCnt[USB_OTG_MAX_TX_FIFOS];
  __IO uint32_t            XferCnt[USB_OTG_MAX_TX_FIFOS];
  __IO HC_STATUS           HC_Status[USB_OTG_MAX_TX_FIFOS];  
  __IO URB_STATE           URB_State[USB_OTG_MAX_TX_FIFOS];
  USB_OTG_HC               hc [USB_OTG_MAX_TX_FIFOS];
  uint16_t                 channel [USB_OTG_MAX_TX_FIFOS];
//  USB_OTG_hPort_TypeDef    *port_cb;  
}
HCD_DEV , *USB_OTG_USBH_PDEV;


typedef struct _OTG
{
  uint8_t    OTG_State;
  uint8_t    OTG_PrevState;  
  uint8_t    OTG_Mode;    
}
OTG_DEV , *USB_OTG_USBO_PDEV;

typedef struct USB_OTG_handle
{
  USB_OTG_CORE_CFGS    cfg;
  USB_OTG_CORE_REGS    regs;
#ifdef USE_DEVICE_MODE
  DCD_DEV     dev;
#endif
#ifdef USE_HOST_MODE
  HCD_DEV     host;
#endif
#ifdef USE_OTG_MODE
  OTG_DEV     otg;
#endif
}
USB_OTG_CORE_HANDLE , *PUSB_OTG_CORE_HANDLE;

/**
  * @}
  */ 


/** @defgroup USB_CORE_Exported_Macros
  * @{
  */ 

/**
  * @}
  */ 

/** @defgroup USB_CORE_Exported_Variables
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup USB_CORE_Exported_FunctionsPrototype
  * @{
  */ 


USB_OTG_STS  USB_OTG_CoreInit        (volatile USB_OTG_CORE_HANDLE *pdev);
USB_OTG_STS  USB_OTG_SelectCore      (volatile USB_OTG_CORE_HANDLE *pdev, USB_OTG_CORE_ID_TypeDef coreID);
USB_OTG_STS  USB_OTG_EnableGlobalInt (volatile USB_OTG_CORE_HANDLE *pdev);
USB_OTG_STS  USB_OTG_DisableGlobalInt(volatile USB_OTG_CORE_HANDLE *pdev);
void*           USB_OTG_ReadPacket   (volatile USB_OTG_CORE_HANDLE *pdev ,
    uint8_t *dest,
    uint16_t len);
USB_OTG_STS  USB_OTG_WritePacket     (volatile USB_OTG_CORE_HANDLE *pdev ,
    uint8_t *src,
    uint8_t ch_ep_num,
    uint16_t len);
USB_OTG_STS  USB_OTG_FlushTxFifo     (volatile USB_OTG_CORE_HANDLE *pdev , uint32_t num);
USB_OTG_STS  USB_OTG_FlushRxFifo     (volatile USB_OTG_CORE_HANDLE *pdev);

uint32_t     USB_OTG_ReadCoreItr     (volatile USB_OTG_CORE_HANDLE *pdev);
uint32_t     USB_OTG_ReadOtgItr      (volatile USB_OTG_CORE_HANDLE *pdev);
uint8_t      USB_OTG_IsHostMode      (volatile USB_OTG_CORE_HANDLE *pdev);
uint8_t      USB_OTG_IsDeviceMode    (volatile USB_OTG_CORE_HANDLE *pdev);
uint32_t     USB_OTG_GetMode         (volatile USB_OTG_CORE_HANDLE *pdev);
USB_OTG_STS  USB_OTG_PhyInit         (volatile USB_OTG_CORE_HANDLE *pdev);
USB_OTG_STS  USB_OTG_SetCurrentMode  (volatile USB_OTG_CORE_HANDLE *pdev,
    uint8_t mode);

/*********************** HOST APIs ********************************************/
#ifdef USE_HOST_MODE
USB_OTG_STS  USB_OTG_CoreInitHost    (volatile USB_OTG_CORE_HANDLE *pdev);
USB_OTG_STS  USB_OTG_EnableHostInt   (volatile USB_OTG_CORE_HANDLE *pdev);
USB_OTG_STS  USB_OTG_HC_Init         (volatile USB_OTG_CORE_HANDLE *pdev, uint8_t hc_num);
USB_OTG_STS  USB_OTG_HC_Halt         (volatile USB_OTG_CORE_HANDLE *pdev, uint8_t hc_num);
USB_OTG_STS  USB_OTG_HC_StartXfer    (volatile USB_OTG_CORE_HANDLE *pdev, uint8_t hc_num);
USB_OTG_STS  USB_OTG_HC_DoPing       (volatile USB_OTG_CORE_HANDLE *pdev , uint8_t hc_num);
uint32_t     USB_OTG_ReadHostAllChannels_intr    (volatile USB_OTG_CORE_HANDLE *pdev);
uint32_t     USB_OTG_ResetPort       (volatile USB_OTG_CORE_HANDLE *pdev);
uint32_t     USB_OTG_ReadHPRT0       (volatile USB_OTG_CORE_HANDLE *pdev);
void         USB_OTG_DriveVbus       (volatile USB_OTG_CORE_HANDLE *pdev, uint8_t state);
void         USB_OTG_InitFSLSPClkSel (volatile USB_OTG_CORE_HANDLE *pdev ,uint8_t freq);
uint8_t      USB_OTG_IsEvenFrame     (volatile USB_OTG_CORE_HANDLE *pdev) ;
void         USB_OTG_StopHost        (volatile USB_OTG_CORE_HANDLE *pdev);
#endif
/********************* DEVICE APIs ********************************************/
#ifdef USE_DEVICE_MODE
USB_OTG_STS  USB_OTG_CoreInitDev         (volatile USB_OTG_CORE_HANDLE *pdev);
USB_OTG_STS  USB_OTG_EnableDevInt        (volatile USB_OTG_CORE_HANDLE *pdev);
uint32_t     USB_OTG_ReadDevAllInEPItr   (volatile USB_OTG_CORE_HANDLE *pdev);
enum USB_OTG_SPEED USB_OTG_GetDeviceSpeed (volatile USB_OTG_CORE_HANDLE *pdev);
USB_OTG_STS  USB_OTG_EP0Activate (volatile USB_OTG_CORE_HANDLE *pdev);
USB_OTG_STS  USB_OTG_EPActivate  (volatile USB_OTG_CORE_HANDLE *pdev , volatile USB_OTG_EP *ep);
USB_OTG_STS  USB_OTG_EPDeactivate(volatile USB_OTG_CORE_HANDLE *pdev , volatile USB_OTG_EP *ep);
USB_OTG_STS  USB_OTG_EPStartXfer (volatile USB_OTG_CORE_HANDLE *pdev , volatile USB_OTG_EP *ep);
USB_OTG_STS  USB_OTG_EP0StartXfer(volatile USB_OTG_CORE_HANDLE *pdev , volatile USB_OTG_EP *ep);
USB_OTG_STS  USB_OTG_EPSetStall          (volatile USB_OTG_CORE_HANDLE *pdev , volatile USB_OTG_EP *ep);
USB_OTG_STS  USB_OTG_EPClearStall        (volatile USB_OTG_CORE_HANDLE *pdev , volatile USB_OTG_EP *ep);
uint32_t     USB_OTG_ReadDevAllOutEp_itr (volatile USB_OTG_CORE_HANDLE *pdev);
uint32_t     USB_OTG_ReadDevOutEP_itr    (volatile USB_OTG_CORE_HANDLE *pdev , uint8_t epnum);
void         USB_OTG_InitDevSpeed        (volatile USB_OTG_CORE_HANDLE *pdev , uint8_t speed);
uint8_t      USBH_IsEvenFrame (volatile USB_OTG_CORE_HANDLE *pdev);
void         USB_OTG_EP0_OutStart(volatile USB_OTG_CORE_HANDLE *pdev);
void         USB_OTG_ActiveRemoteWakeup(volatile USB_OTG_CORE_HANDLE *pdev);
void         USB_OTG_UngateClock(volatile USB_OTG_CORE_HANDLE *pdev);
void         USB_OTG_StopDevice(volatile USB_OTG_CORE_HANDLE *pdev);
void         USB_OTG_SetEPStatus (volatile USB_OTG_CORE_HANDLE *pdev , volatile USB_OTG_EP *ep , uint32_t Status);
uint32_t     USB_OTG_GetEPStatus(volatile USB_OTG_CORE_HANDLE *pdev ,USB_OTG_EP *ep);
#endif
/**
  * @}
  */ 

#endif  /* __USB_CORE_H__ */


/**
  * @}
  */ 

/**
  * @}
  */ 
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

