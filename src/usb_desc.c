/**
  ******************************************************************************
  * @file    USB_Example/usb_desc.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    20-September-2012
  * @brief   Descriptors for Demo
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

/* Includes ------------------------------------------------------------------*/
#include "usb_lib.h"
#include "usb_desc.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* USB Standard Device Descriptor */
const uint8_t Hid_DeviceDescriptor[HID_SIZ_DEVICE_DESC] =
  {
    0x12,                       /*bLength */
    USB_DEVICE_DESCRIPTOR_TYPE, /*bDescriptorType*/
    0x00,                       /*bcdUSB */
    0x02,
    0x00,                       /*bDeviceClass*/
    0x00,                       /*bDeviceSubClass*/
    0x00,                       /*bDeviceProtocol*/
    0x40,                       /*bMaxPacketSize 64*/
    0x33,                       /*idVendor rift (0x2833)*/
    0x28,
    0x01,                       /*idProduct rift dev-kit (0x0001)*/
    0x00,
    0x00,                       /*bcdDevice (or version) 0x0100 */
    0x01,
    1,                          /*Index of string descriptor describing
                                                  manufacturer */
    2,                          /*Index of string descriptor describing
                                                 product*/
    3,                          /*Index of string descriptor describing the
                                                 device serial number */
    0x01                        /*bNumConfigurations*/
  }
  ; /* Hid_DeviceDescriptor */


/* USB Configuration Descriptor */
/*   All Descriptors (Configuration, Interface, Endpoint, Class, Vendor */
const uint8_t Hid_ConfigDescriptor[HID_SIZ_CONFIG_DESC] =
  {
    0x09, /* bLength: Configuration Descriptor size */
    USB_CONFIGURATION_DESCRIPTOR_TYPE, /* bDescriptorType: Configuration */
    HID_SIZ_CONFIG_DESC,	//+HID_SIZ_DEVICE_DESC+HID_SIZ_REPORT_DESC,	//Total length [L] 0x5E
    /* wTotalLength: Bytes returned */
    0x00,					//Total length [H]
    0x01,         /*bNumInterfaces: 1 interface*/
    0x01,         /*bConfigurationValue: Configuration value*/
    0x00,         /*iConfiguration: Index of string descriptor describing
                                     the configuration*/
    0xE0,         /*bmAttributes: bus powered */
    0xFA,         /*MaxPower 500 mA: this current is used for detecting Vbus*/

    /************** Descriptor of Hid RIFT interface ****************/
    /* 09 */
    0x09,         /*bLength: Interface Descriptor size*/
    USB_INTERFACE_DESCRIPTOR_TYPE,/*bDescriptorType: Interface descriptor type*/
    0x00,         /*bInterfaceNumber: Number of Interface*/
    0x00,         /*bAlternateSetting: Alternate setting*/
    0x02,         /*bNumEndpoints*/
    0x03,         /*bInterfaceClass: HID*/
    0x00,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
    0x00,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
    0,            /*iInterface: Index of string descriptor*/
    /******************** Descriptor of Hid RIFT HID ********************/
    /* 18 */
    0x09,         /*bLength: HID Descriptor size*/
    HID_DESCRIPTOR_TYPE, /*bDescriptorType: HID*/
    0x11,         /*bcdHID: HID Class Spec release number*/
    0x01,
    0x00,         /*bCountryCode: Hardware target country*/
    0x01,         /*bNumDescriptors: Number of HID class descriptors to follow*/
    0x22,         /*bDescriptorType*/
    HID_SIZ_REPORT_DESC,/*wItemLength: Total length of Report descriptor*/
    0x00,
    /******************** Descriptor of Hid RIFT endpoint ********************/
//    Defines a interrupt in and out endpoint
    /* 27 */
    0x07,          /*bLength: Endpoint Descriptor size*/
    USB_ENDPOINT_DESCRIPTOR_TYPE, /*bDescriptorType:*/

    0x81,          /*bEndpointAddress: Endpoint Address (IN)*/
    0x03,          /*bmAttributes: Interrupt endpoint*/
    64,            /*wMaxPacketSize: 64 Byte max */
    0x00,
    0x01,          /*bInterval: Polling Interval (1 ms)*/
    /* 34 */
    0x07,          /*bLength: Endpoint Descriptor size*/
    USB_ENDPOINT_DESCRIPTOR_TYPE, /*bDescriptorType:*/

    0x01,          /*bEndpointAddress: Endpoint Address (OUT)*/
    0x03,          /*bmAttributes: Interrupt endpoint*/
    USB_PACKET_SIZE,          /*wMaxPacketSize: Byte max */
    0x00,
    0x80,          /*bInterval: Polling Interval (128 ms)*/
    /* 34 */
  }; /* RIFT_ConfigDescriptor */

// Old report descriptor (works only on Linux as no Reprt IDs are defined)
//		    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
//		    0x09, 0x00,                    // USAGE (Undefined)
//		    0xa1, 0x01,                    // COLLECTION (Application)
//		    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
//		    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
//		    0x75, 0x08,                    //   REPORT_SIZE (8)
//		    0x95, 0x3e,                    //   REPORT_COUNT (62)
//		    0x09, 0x00,                    //   USAGE (Undefined)
//		    0x82, 0x02, 0x01,              //   INPUT (Data,Var,Abs,Buf)
//		    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
//		    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
//		    0x75, 0x08,                    //   REPORT_SIZE (8)
//		    0x95, 63,         				//   REPORT_COUNT (63)
//		    0x09, 0x00,                    //   USAGE (Undefined)
//		    0xb2, 0x02, 0x01,              //   FEATURE (Data,Var,Abs,Buf)
//		    0xc0                           // END_COLLECTION

const uint8_t Hid_ReportDescriptor[HID_SIZ_REPORT_DESC] =
  {
		    0x06, 0x00, 0xff,              // USAGE_PAGE (Vendor Defined Page 1)
		    0x09, 0x01,                    //   USAGE (Vendor Usage 1)
		    0xa1, 0x01,                    // COLLECTION (Application)
		    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
		    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
		    0x75, 0x08,                    //   REPORT_SIZE (8)
		    0x85, 0x01,                    //   REPORT_ID (1)
		    0x95, 0x3d,                    //   REPORT_COUNT (62-1)
		    0x09, 0x01,                    //   USAGE (Vendor Usage 1)
		    0x82, 0x02, 0x01,              //   INPUT (Data,Var,Abs,Buf)
		    0x85, 0x02,                    //   REPORT_ID (2)
		    0x95, 0x06,                    //   REPORT_COUNT (7-1)
		    0x09, 0x01,                    //   USAGE (Vendor Usage 1)
		    0xb2, 0x02, 0x01,              //   FEATURE (Data,Var,Abs,Buf)
		    0x85, 0x04,                    //   REPORT_ID (4)
		    0x95, 0x07,                    //   REPORT_COUNT (8-1)
		    0x09, 0x01,                    //   USAGE (Vendor Usage 1)
		    0xb2, 0x02, 0x01,              //   FEATURE (Data,Var,Abs,Buf)
		    0x85, 0x08,                    //   REPORT_ID (8)
		    0x95, 0x04,                    //   REPORT_COUNT (5-1)
		    0x09, 0x01,                    //   USAGE (Vendor Usage 1)
		    0xb2, 0x02, 0x01,              //   FEATURE (Data,Var,Abs,Buf)
		    0x85, 0x09,                    //   REPORT_ID (9)
		    0x95, 0x37,                    //   REPORT_COUNT (56-1)
		    0x09, 0x01,                    //   USAGE (Vendor Usage 1)
		    0xb2, 0x02, 0x01,              //   FEATURE (Data,Var,Abs,Buf)
		    0xc0                           // END_COLLECTION
  }  ; /* Hid_ReportDescriptor */

/* USB String Descriptors (optional) */
const uint8_t Hid_StringLangID[HID_SIZ_STRING_LANGID] =
  {
    HID_SIZ_STRING_LANGID,
    USB_STRING_DESCRIPTOR_TYPE,
    0x09,
    0x04
  }
  ; /* LangID = 0x0409: U.S. English */

const uint8_t Hid_StringVendor[HID_SIZ_STRING_VENDOR] =
  {
    HID_SIZ_STRING_VENDOR, /* Size of Vendor string */
    USB_STRING_DESCRIPTOR_TYPE,  /* bDescriptorType*/
    /* Manufacturer: 'O', 'c', 'u', 'l', 'u', 's', ' ', 'V', 'R', ',', ' ', 'I', 'n', 'c', '.' */
    'O', 0, 'c', 0, 'u', 0, 'l', 0, 'u', 0, 's', 0, ' ', 0, 'V', 0, 'R', 0, ',', 0, ' ', 0, 'I', 0, 'n', 0, 'c', 0, '.', 0
  };

const uint8_t Hid_StringProduct[HID_SIZ_STRING_PRODUCT] =
  {
    HID_SIZ_STRING_PRODUCT,          /* bLength */
    USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
//    'T', 'r', 'a', 'c', 'k', 'e', 'r', ' ', 'D', 'K'
    'T', 0, 'r', 0, 'a', 0, 'c', 0, 'k', 0, 'e', 0, 'r', 0, ' ', 0, 'D', 0, 'K', 0
  };
uint8_t Hid_StringSerial[HID_SIZ_STRING_SERIAL] =
  {
    HID_SIZ_STRING_SERIAL,           /* bLength */
    USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
//'3', '9', 'M', 'A', 'I', '3', 'T', 'G', 'Q', 'K', '3', '7'
    '3', 0, '9', 0, 'M', 0, 'A', 0, 'I', 0, '3', 0, 'T', 0, 'G', 0, 'Q', 0, 'K', 0, '3', 0, '7', 0
  };


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/ 
