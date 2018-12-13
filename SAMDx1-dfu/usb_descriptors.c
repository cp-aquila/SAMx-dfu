/*
 * Copyright (c) 2016, Alex Taradov <alex@taradov.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


/*- Includes ----------------------------------------------------------------*/
#include "usb.h"
#include "usb_descriptors.h"

#define DFU_TRANSFER_SIZE 64

/*- Variables ---------------------------------------------------------------*/
usb_device_descriptor_t usb_device_descriptor __attribute__((aligned(4))) =   /* MUST BE IN RAM for USB peripheral */
{
  .bLength            = sizeof(usb_device_descriptor_t),
  .bDescriptorType    = USB_DEVICE_DESCRIPTOR,

  .bcdUSB                 = 0x0200,
  .bDeviceClass           = 0,
  .bDeviceSubClass        = 0, /* DFU */
  .bDeviceProtocol        = 0,

  .bMaxPacketSize0        = 64,
  .idVendor               = 0x1209,
  .idProduct              = 0x2003,
  .bcdDevice              = 0x0005,

  .iManufacturer          = USB_STRING_MANU,
  .iProduct               = USB_STRING_PRODUCT,
  .iSerialNumber          = USB_STRING_SERIAL,

  .bNumConfigurations     = 1
};

usb_string_descriptor usb_string_lang __attribute__((aligned(4))) = {
  .bLength = 4,
  .bDescriptorType = USB_STRING_DESCRIPTOR,
  .bString = {USB_LANGUAGE_EN_US},
};

usb_string_descriptor usb_string_manu __attribute__((aligned(4))) = {
  .bLength = 20,
  .bDescriptorType = USB_STRING_DESCRIPTOR,
  .bString = {'A', 0, 'Q', 0, 'I', 0, 'L', 0, 'A', 0, ' ', 0, 'B', 0, 'I', 0, 'O', 0},
};

usb_string_descriptor usb_string_serial __attribute__((aligned(4))) = {
  .bLength = 10,
  .bDescriptorType = USB_STRING_DESCRIPTOR,
  .bString = {'1', 0, '3', 0, '3', 0, '7', 0},
};

usb_string_descriptor usb_string_empty __attribute__((aligned(4))) = {
  .bLength = 4,
  .bDescriptorType = USB_STRING_DESCRIPTOR,
  .bString = {0, 0},
};

usb_string_descriptor usb_string_dfu_flash __attribute__((aligned(4))) = {
  .bLength = 12,
  .bDescriptorType = USB_STRING_DESCRIPTOR,
  .bString = {'F', 0, 'L', 0, 'A', 0, 'S', 0, 'H', 0},
};

usb_string_descriptor usb_string_msftos __attribute__((aligned(4))) = {
  .bLength = 18,
  .bDescriptorType = USB_STRING_DESCRIPTOR,
  .bString = {'M', 0, 'S', 0, 'F', 0, 'T', 0, '1', 0, '0', 0, '0', 0, 0xee, 0},
};

USB_MicrosoftCompatibleDescriptor msft_compatible __attribute__((aligned(4))) = {
  .dwLength = sizeof(USB_MicrosoftCompatibleDescriptor) + sizeof(USB_MicrosoftCompatibleDescriptor_Interface),
  .bcdVersion = 0x0100,
  .wIndex = 0x0004,
  .bCount = 1,
  .reserved = {0, 0, 0, 0, 0, 0, 0},
  .interfaces = {
    {
      .bFirstInterfaceNumber = 0,
      .reserved1 = 0,
      .compatibleID = "WINUSB\0\0",
      .subCompatibleID = {0, 0, 0, 0, 0, 0, 0, 0},
      .reserved2 = {0, 0, 0, 0, 0, 0},
    }
  }
};

usb_configuration_hierarchy_t usb_configuration_hierarchy __attribute__((aligned(4))) =   /* MUST BE IN RAM for USB peripheral */
{
  .configuration =
  {
    .bLength             = sizeof(usb_configuration_descriptor_t),
    .bDescriptorType     = USB_CONFIGURATION_DESCRIPTOR,
    .wTotalLength        = sizeof(usb_configuration_hierarchy_t),
    .bNumInterfaces      = 1,
    .bConfigurationValue = 1,
    .iConfiguration      = USB_STR_ZERO,
    .bmAttributes        = 0x80,
    .bMaxPower           = 50, // 100 mA
  },

  .interface =
  {
    .bLength             = sizeof(usb_interface_descriptor_t),
    .bDescriptorType     = USB_INTERFACE_DESCRIPTOR,
    .bInterfaceNumber    = 0,
    .bAlternateSetting   = 0,
    .bNumEndpoints       = 0,
    .bInterfaceClass     = DFU_INTERFACE_CLASS,
    .bInterfaceSubClass  = DFU_INTERFACE_SUBCLASS,
    .bInterfaceProtocol  = DFU_INTERFACE_PROTOCOL,
    .iInterface          = USB_STRING_DFU_FLASH,
  },

  .dfu =
  {
    .bLength             = sizeof(usb_dfu_descriptor_t),
    .bDescriptorType     = DFU_DESCRIPTOR_TYPE,
    .bmAttributes        = DFU_ATTR_CAN_DOWNLOAD | DFU_ATTR_CAN_UPLOAD,
    .wDetachTimeout      = 0,
    .wTransferSize       = DFU_TRANSFER_SIZE,
    .bcdDFU              = 0x101,
  },
};
