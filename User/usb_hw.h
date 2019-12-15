
#ifndef _USB_HW_H
#define _USB_HW_H

#include "usb_lib.h"
#include "usb_desc.h"


#define BULK_MAX_PACKET_SIZE  64

// 唯一id
#define         ID1          (0x1FFFF7E8)
#define         ID2          (0x1FFFF7EC)
#define         ID3          (0x1FFFF7F0)

// USB软连接IO
#define USB_CONNECT_GPIO	GPIOA
#define USB_CONNECT_PIN		GPIO_Pin_8

// usb_conf
#define BTABLE_ADDRESS      (0x00)
/* EP0  */
/* rx/tx buffer base address */
#define ENDP0_RXADDR        (0x18)
#define ENDP0_TXADDR        (0x58)
/* EP1  */
/* tx buffer base address */
#define ENDP1_TXADDR        (0x98)
/* EP2  */
/* Rx buffer base address */
#define ENDP2_RXADDR        (0xD8)
/* ISTR events */
/* IMR_MSK */
/* mask defining which events has to be handled */
/* by the device application software */
#define IMR_MSK (CNTR_CTRM  | CNTR_WKUPM | CNTR_SUSPM | CNTR_ERRM  | CNTR_SOFM \
                 | CNTR_ESOFM | CNTR_RESETM )

// usb_pwr
typedef enum _RESUME_STATE
{
  RESUME_EXTERNAL,
  RESUME_INTERNAL,
  RESUME_LATER,
  RESUME_WAIT,
  RESUME_START,
  RESUME_ON,
  RESUME_OFF,
  RESUME_ESOF
} RESUME_STATE;

typedef enum _DEVICE_STATE
{
  UNCONNECTED,
  ATTACHED,
  POWERED,
  SUSPENDED,
  ADDRESSED,
  CONFIGURED
} DEVICE_STATE;


extern  __IO uint32_t bDeviceState; /* USB device status */
extern __IO bool fSuspendEnabled;  /* true when suspend is possible */


void USB_HW_Init(void);
void USB_Connect(bool con);
void Get_SerialNum(void);
void Suspend(void);
void Resume_Init(void);
void Resume(RESUME_STATE eResumeSetVal);
RESULT PowerOn(void);
RESULT PowerOff(void);


// usb_istr
void EP1_IN_Callback(void);
void EP2_OUT_Callback(void);

#ifdef CTR_CALLBACK
void CTR_Callback(void);
#endif

#ifdef DOVR_CALLBACK
void DOVR_Callback(void);
#endif

#ifdef ERR_CALLBACK
void ERR_Callback(void);
#endif

#ifdef WKUP_CALLBACK
void WKUP_Callback(void);
#endif

#ifdef SUSP_CALLBACK
void SUSP_Callback(void);
#endif

#ifdef RESET_CALLBACK
void RESET_Callback(void);
#endif

#ifdef SOF_CALLBACK
void SOF_Callback(void);
#endif

#ifdef ESOF_CALLBACK
void ESOF_Callback(void);
#endif

#endif  /*_USB_HW_H*/

