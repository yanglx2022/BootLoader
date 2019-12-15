
#ifndef __USB_DESC_H
#define __USB_DESC_H


#include <stdint.h>

#define WBVAL(x) (x & 0xFF),((x >> 8) & 0xFF)

/* bmAttributes in Configuration Descriptor */
#define USB_CONFIG_POWERED_MASK                0xC0
#define USB_CONFIG_BUS_POWERED                 0x80
#define USB_CONFIG_SELF_POWERED                0xC0
#define USB_CONFIG_REMOTE_WAKEUP               0x20

/* bMaxPower in Configuration Descriptor */
#define USB_CONFIG_POWER_MA(mA)                ((mA)/2)

/* bEndpointAddress in Endpoint Descriptor */
#define USB_ENDPOINT_DIRECTION_MASK            0x80
#define USB_ENDPOINT_OUT(addr)                 ((addr) | 0x00)
#define USB_ENDPOINT_IN(addr)                  ((addr) | 0x80)

/* USB Device Classes */
#define USB_DEVICE_CLASS_RESERVED              0x00
#define USB_DEVICE_CLASS_AUDIO                 0x01
#define USB_DEVICE_CLASS_COMMUNICATIONS        0x02
#define USB_DEVICE_CLASS_HUMAN_INTERFACE       0x03
#define USB_DEVICE_CLASS_MONITOR               0x04
#define USB_DEVICE_CLASS_PHYSICAL_INTERFACE    0x05
#define USB_DEVICE_CLASS_POWER                 0x06
#define USB_DEVICE_CLASS_PRINTER               0x07
#define USB_DEVICE_CLASS_STORAGE               0x08
#define USB_DEVICE_CLASS_HUB                   0x09
#define USB_DEVICE_CLASS_MISCELLANEOUS         0xEF
#define USB_DEVICE_CLASS_VENDOR_SPECIFIC       0xFF

/* bmAttributes in Endpoint Descriptor */
#define USB_ENDPOINT_TYPE_MASK                 0x03
#define USB_ENDPOINT_TYPE_CONTROL              0x00
#define USB_ENDPOINT_TYPE_ISOCHRONOUS          0x01
#define USB_ENDPOINT_TYPE_BULK                 0x02
#define USB_ENDPOINT_TYPE_INTERRUPT            0x03
#define USB_ENDPOINT_SYNC_MASK                 0x0C
#define USB_ENDPOINT_SYNC_NO_SYNCHRONIZATION   0x00
#define USB_ENDPOINT_SYNC_ASYNCHRONOUS         0x04
#define USB_ENDPOINT_SYNC_ADAPTIVE             0x08
#define USB_ENDPOINT_SYNC_SYNCHRONOUS          0x0C
#define USB_ENDPOINT_USAGE_MASK                0x30
#define USB_ENDPOINT_USAGE_DATA                0x00
#define USB_ENDPOINT_USAGE_FEEDBACK            0x10
#define USB_ENDPOINT_USAGE_IMPLICIT_FEEDBACK   0x20
#define USB_ENDPOINT_USAGE_RESERVED            0x30

/* descriptor type */
#define USB_DEVICE_DESCRIPTOR_TYPE             0x01
#define USB_CONFIGURATION_DESCRIPTOR_TYPE      0x02
#define USB_STRING_DESCRIPTOR_TYPE             0x03
#define USB_INTERFACE_DESCRIPTOR_TYPE          0x04
#define USB_ENDPOINT_DESCRIPTOR_TYPE           0x05


// size
#define DEVICE_DESC_SIZE                				18
#define CONFIG_DESC_TOTAL_SIZE              		32
#define STRING_LANGID_SIZE              				4
#define STRING_VENDOR_SIZE              				22
#define STRING_PRODUCT_SIZE             				52
#define STRING_SERIAL_SIZE              				26



extern const uint8_t USB_DeviceDescriptor[];
extern const uint8_t USB_ConfigDescriptor[];
extern const uint8_t USB_StringLangID[];
extern const uint8_t USB_StringVendor[];
extern const uint8_t USB_StringProduct[];
extern uint8_t USB_StringSerial[STRING_SERIAL_SIZE];


#endif /* __USB_DESC_H */


