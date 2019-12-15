/**********************
* USB描述符
**********************/

#include "usb_desc.h"

// 设备描述符
const uint8_t USB_DeviceDescriptor[] =
{
	0x12,                       /*bLength */
	USB_DEVICE_DESCRIPTOR_TYPE, /*bDescriptorType*/
	WBVAL(0x0200),							/*bcdUSB 2.0*/
	0x00,                       /*bDeviceClass*/
	0x00,                       /*bDeviceSubClass*/
	0x00,                       /*bDeviceProtocol*/
	0x40,                       /*bMaxPacketSize 64*/
	WBVAL(0x0483),              /*idVendor (0x0483)*/
	WBVAL(0x0000),              /*idProduct = 0x0000*/
	WBVAL(0x0200),              /*bcdDevice rel. 2.00*/
	1,                          /*Index of string descriptor describing manufacturer */
	2,                          /*Index of string descriptor describing product*/
	3,                          /*Index of string descriptor describing the device serial number */
	0x01                        /*bNumConfigurations*/
};

// 配置描述符集合
const uint8_t USB_ConfigDescriptor[] =
{
	// 配置描述符
	0x09, 															/* bLength: Configuration Descriptor size */
	USB_CONFIGURATION_DESCRIPTOR_TYPE,	/* bDescriptorType: Configuration */
	WBVAL(CONFIG_DESC_TOTAL_SIZE),	/* wTotalLength: Bytes returned */
	0x01,         											/*bNumInterfaces: 1 interface*/
	0x01,         											/*bConfigurationValue: Configuration value*/
	0x00,         											/*iConfiguration: Index of string descriptor describing the configuration*/
	USB_CONFIG_BUS_POWERED,         		/*bmAttributes: bus powered */
	USB_CONFIG_POWER_MA(100),						/*MaxPower 100 mA: this current is used for detecting Vbus*/
	// 接口描述符
	0x09,         										/*bLength: Interface Descriptor size*/
	USB_INTERFACE_DESCRIPTOR_TYPE,		/*bDescriptorType: Interface descriptor type*/
	0x00,         										/*bInterfaceNumber: Number of Interface*/
	0x00,         										/*bAlternateSetting: Alternate setting*/
	0x02,         										/*bNumEndpoints*/
	USB_DEVICE_CLASS_STORAGE,					/*bInterfaceClass: STORAGE*/
	0x06,         										/*bInterfaceSubClass : SCSI transparent*/
	0x50,         										/*nInterfaceProtocol : Bulk-Only*/
	0x00,            									/*iInterface: Index of string descriptor*/
	// 端点描述符
	0x07,          								/*bLength: Endpoint Descriptor size*/
	USB_ENDPOINT_DESCRIPTOR_TYPE, /*bDescriptorType:*/
	USB_ENDPOINT_IN(1),          	/*bEndpointAddress: Endpoint Address (IN)*/
	USB_ENDPOINT_TYPE_BULK,				/*bmAttributes: bulk endpoint*/
	WBVAL(0x0040),          			/*wMaxPacketSize: 64 bytes max */
	0x00,          								/*bInterval: Polling Interval (meaningless here)*/
	// 端点描述符
	0x07,          								/*bLength: Endpoint Descriptor size*/
	USB_ENDPOINT_DESCRIPTOR_TYPE, /*bDescriptorType:*/
	USB_ENDPOINT_OUT(2),          	/*bEndpointAddress: Endpoint Address (IN)*/
	USB_ENDPOINT_TYPE_BULK,				/*bmAttributes: bulk endpoint*/
	WBVAL(0x0040),          			/*wMaxPacketSize: 64 bytes max */
	0x00,          								/*bInterval: Polling Interval (meaningless here)*/
};

// 字符串描述符
const uint8_t USB_StringLangID[] =
{
	STRING_LANGID_SIZE,
	USB_STRING_DESCRIPTOR_TYPE,
	WBVAL(0x0409)
}; /* LangID = 0x0409: U.S. English */
const uint8_t USB_StringVendor[] =
{
	STRING_VENDOR_SIZE,						/* Size of Vendor string */
	USB_STRING_DESCRIPTOR_TYPE,		/* bDescriptorType*/
	'y', 0, 											/* yanglx2022 */
	'a', 0, 
	'n', 0, 
	'g', 0, 
	'l', 0, 
	'x', 0, 
	'2', 0, 
	'0', 0,
	'2', 0, 
	'2', 0
};
const uint8_t USB_StringProduct[] =
{
	STRING_PRODUCT_SIZE,					/* bLength */
	USB_STRING_DESCRIPTOR_TYPE,		/* bDescriptorType */
  'M', 0,												/* MIDI Keyboard Update Disk*/ // 弹出U盘名称
  'I', 0,
  'D', 0,
  'I', 0,
  ' ', 0,
  'K', 0,
  'e', 0,
  'y', 0,
  'b', 0,
  'o', 0,
  'a', 0,
  'r', 0,
  'd', 0,
	' ', 0,
  'U', 0,
  'p', 0,
  'd', 0,
  'a', 0,
  't', 0,
  'e', 0,
	' ', 0,
  'D', 0,
  'i', 0,
  's', 0,
  'k', 0,
};
uint8_t USB_StringSerial[STRING_SERIAL_SIZE] =
{
	STRING_SERIAL_SIZE,						/* bLength */
	USB_STRING_DESCRIPTOR_TYPE,		/* bDescriptorType */
	'S', 0,												/* STM32 */
	'T', 0,
	'M', 0,
	'3', 0,
	'2', 0
};



