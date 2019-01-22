/*
 * 1kByte USB DFU bootloader for Atmel SAMD11 microcontrollers
 *
 * Copyright (c) 2019, Carsten Presser
 * Copyright (c) 2018, Peter Lawrence
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

/*
NOTES:
- anything pointed to by udc_mem[*].*.ADDR.reg *MUST* BE IN RAM and be 32-bit aligned... no exceptions
*/


/*- Includes ----------------------------------------------------------------*/
#include <stdbool.h>
#include <string.h>
#include <sam.h>
#include "clock.h"
#include "crc16.h"
#include "usb.h"
#include "nvm_data.h"
#include "usb_descriptors.h"
#include "peripherals.h"

/*- Definitions -------------------------------------------------------------*/
#define USB_CMD(dir, rcpt, type) ((USB_##dir##_TRANSFER << 7) | (USB_##type##_REQUEST << 5) | (USB_##rcpt##_RECIPIENT << 0))
#define SIMPLE_USB_CMD(rcpt, type) ((USB_##type##_REQUEST << 5) | (USB_##rcpt##_RECIPIENT << 0))
#define GCLK_SYSTEM 0
#define FLASH_BOOT_SIZE 4096
#define FLASH_FW_ADDR   FLASH_BOOT_SIZE
#define LED_BLINK_CYCLES 40000UL


/*- Types -------------------------------------------------------------------*/
typedef struct {
  UsbDeviceDescBank  out;
  UsbDeviceDescBank  in;
} udc_mem_t;

/*- Variables ---------------------------------------------------------------*/
static uint32_t dfu_done = 0;
static uint32_t usb_config = 0;
static uint32_t dfu_status_choices[6] = {
  0x00000000, 0x00000002, /* normal */
  0x00000000, 0x00000005, /* dl */
  0x00000000, 0x00000006, /* manifest sync */
};

static udc_mem_t udc_mem[USB_EPT_NUM];
static uint32_t udc_ctrl_in_buf[16];
static uint32_t udc_ctrl_out_buf[16];

__attribute__((section(".version")))
__attribute__((used))
const struct {
  uint16_t version;
  char git_short_tag[8];
  char datetime[32];
} version_data = {
  .version = DFU_VERSION,
  .git_short_tag = GIT_SHORT_TAG,
  .datetime = __DATE__ " " __TIME__
};

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static void udc_control_send(const uint32_t* data, uint32_t size)
{
  /* USB peripheral *only* reads valid data from 32-bit aligned RAM locations */
  udc_mem[0].in.ADDR.reg = (uint32_t)data;

  udc_mem[0].in.PCKSIZE.reg = USB_DEVICE_PCKSIZE_BYTE_COUNT(size) | USB_DEVICE_PCKSIZE_MULTI_PACKET_SIZE(0) | USB_DEVICE_PCKSIZE_SIZE(3 /*64 Byte*/);

  USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT1;
  USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.bit.BK1RDY = 1;

  while (0 == USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.bit.TRCPT1);
}

//-----------------------------------------------------------------------------
static void udc_control_send_zlp(void)
{
  udc_control_send(NULL, 0); /* peripheral can't read from NULL address, but size is zero and this value takes less space to compile */
}

#define USB_STRING_MAX_LEN (16 * 2) // 16 chars for a 64bit mac address without columns
#define USB_STRING_PLACEHOLDER "                                  "
usb_string_descriptor usb_string_g __attribute__((aligned(4))) = {
  .bLength = USB_STRING_MAX_LEN,
  .bDescriptorType = USB_STRING_DESCRIPTOR,
  .bString = USB_STRING_PLACEHOLDER,
};

static void usb_send_string(const char* text, uint32_t len)
{
  int s = strlen(text);
  if (s > USB_STRING_MAX_LEN) {
    s = USB_STRING_MAX_LEN;
  }
  usb_string_g.bLength = s * 2 + 2;
  for (int i = 0; i < s; i++) {
    usb_string_g.bString[i * 2] = text[i];
    usb_string_g.bString[i * 2 + 1] = 0;
  }
  udc_control_send((uint32_t*)&usb_string_g, len);
}

static void usb_send_serial(uint32_t len)
{
  ee_data_t* e = (ee_data_t*)0x804008;
  usb_string_g.bLength = 34;
  for (int i = 0; i < 16; i += 2) {
    uint8_t b = e->ieee_address[i / 2];
    usb_string_g.bString[i * 2 + 0] = "0123456789ABCDEF"[b >> 4 & 0x0F];
    usb_string_g.bString[i * 2 + 1] = 0;
    usb_string_g.bString[i * 2 + 2] = "0123456789ABCDEF"[b & 0x0F];
    usb_string_g.bString[i * 2 + 3] = 0;
  }
  udc_control_send((uint32_t*)&usb_string_g, len);
}


//-----------------------------------------------------------------------------
static void USB_Service(void)
{
  static uint32_t dfu_addr;

  if (USB->DEVICE.INTFLAG.bit.EORST) { /* End Of Reset */
    USB->DEVICE.INTFLAG.reg = USB_DEVICE_INTFLAG_EORST;
    USB->DEVICE.DADD.reg = USB_DEVICE_DADD_ADDEN;

    for (int ep = 0; ep < USB_EPT_NUM; ep++)
    { USB->DEVICE.DeviceEndpoint[ep].EPCFG.reg = 0; }

    USB->DEVICE.DeviceEndpoint[0].EPCFG.reg = USB_DEVICE_EPCFG_EPTYPE0(1 /*CONTROL*/) | USB_DEVICE_EPCFG_EPTYPE1(1 /*CONTROL*/);
    USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.bit.BK0RDY = 1;
    USB->DEVICE.DeviceEndpoint[0].EPSTATUSCLR.bit.BK1RDY = 1;

    udc_mem[0].in.ADDR.reg = (uint32_t)udc_ctrl_in_buf;
    udc_mem[0].in.PCKSIZE.reg = USB_DEVICE_PCKSIZE_BYTE_COUNT(0) | USB_DEVICE_PCKSIZE_MULTI_PACKET_SIZE(0) | USB_DEVICE_PCKSIZE_SIZE(3 /*64 Byte*/);

    udc_mem[0].out.ADDR.reg = (uint32_t)udc_ctrl_out_buf;
    udc_mem[0].out.PCKSIZE.reg = USB_DEVICE_PCKSIZE_BYTE_COUNT(64) | USB_DEVICE_PCKSIZE_MULTI_PACKET_SIZE(0) | USB_DEVICE_PCKSIZE_SIZE(3 /*64 Byte*/);

    USB->DEVICE.DeviceEndpoint[0].EPSTATUSCLR.bit.BK0RDY = 1;
  }

  // Handle incoming DFU data packets, write to Flash
  if (USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.bit.TRCPT0) {
    if (dfu_addr) {
      if (0 == ((dfu_addr >> 6) & 0x3)) {
        NVMCTRL->ADDR.reg = dfu_addr >> 1;
#ifdef __SAME54N19A__
        NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMDEX_KEY | NVMCTRL_CTRLB_CMD(NVMCTRL_CTRLB_CMD_EB_Val);
        while (!NVMCTRL->INTFLAG.bit.DONE);
#else
        NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD(NVMCTRL_CTRLA_CMD_ER);
        while (!NVMCTRL->INTFLAG.bit.READY);
#endif
      }

      uint16_t* nvm_addr = (uint16_t*)(dfu_addr);
      uint16_t* ram_addr = (uint16_t*)udc_ctrl_out_buf;
      for (unsigned i = 0; i < 32; i++)
      { *nvm_addr++ = *ram_addr++; }
#ifdef __SAME54N19A__
      while (!NVMCTRL->INTFLAG.bit.DONE);
#else
      while (!NVMCTRL->INTFLAG.bit.READY);
#endif

      udc_control_send_zlp();
      dfu_addr = 0;
    }

    USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT0;
  }

  // Handle Setup packets
  if (USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.bit.RXSTP) { /* Received Setup */
    USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_RXSTP;
    USB->DEVICE.DeviceEndpoint[0].EPSTATUSCLR.bit.BK0RDY = 1;

    usb_request_t* request = (usb_request_t*)udc_ctrl_out_buf;
    uint8_t type = request->wValue >> 8;
    uint8_t index = request->wValue & 0xff;
    uint16_t length = request->wLength;
    static uint32_t* dfu_status = dfu_status_choices + 0;

    /* for these other USB requests, we must examine all fields in bmRequestType */
    if (USB_CMD(OUT, INTERFACE, STANDARD) == request->bmRequestType) {
      udc_control_send_zlp();
      return;
    }

    // handle Microsoft thing
    if (USB_CMD(IN, DEVICE, VENDOR) == request->bmRequestType) {
      // 0x20, since we put a whitespace (=0x20) after "MSFT100" String.
      if ((request->bRequest == 0x20) && (request->wIndex == 0x0004)) {
        udc_control_send((uint32_t*)&msft_compatible, length);
      } else {
        USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.bit.STALLRQ1 = 1;
      }
      return;
    }

    /* for these "simple" USB requests, we can ignore the direction and use only bRequest */
    switch (request->bmRequestType & 0x7F) {
      case SIMPLE_USB_CMD(DEVICE, STANDARD):
      case SIMPLE_USB_CMD(INTERFACE, STANDARD):
        switch (request->bRequest) {
          case USB_GET_DESCRIPTOR:
            switch (type) {
              case USB_DEVICE_DESCRIPTOR:
                udc_control_send((uint32_t*)&usb_device_descriptor, length);
                break;
              case USB_CONFIGURATION_DESCRIPTOR:
                udc_control_send((uint32_t*)&usb_configuration_hierarchy, length);
                break;
              case USB_STRING_DESCRIPTOR:
                switch (index) {
                  case USB_STRING_LANG:
                    udc_control_send((uint32_t*)&usb_string_lang, length);
                    break;
                  case USB_STRING_MANU:
                    usb_send_string("AquilaBiolabs", length);
                    break;
                  case USB_STRING_PRODUCT:
                    usb_send_string("DFU Bootloader", length);
                    break;
                  case USB_STRING_SERIAL:
                    usb_send_serial(length);
                    break;
                  case USB_STRING_DFU_FLASH:
                    usb_send_string("Flash", length);
                    break;
                  case USB_STRING_MSFT:
                    usb_send_string("MSFT100 ", length);
                    break;
                  default:
                    USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.bit.STALLRQ1 = 1;
                    break;
                }
                break;
              default:
                USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.bit.STALLRQ1 = 1;
                break;
            }
            break;
          case USB_GET_CONFIGURATION:
            udc_control_send(&usb_config, 1);
            break;
          case USB_GET_STATUS:
            udc_control_send(dfu_status_choices + 0, 2); /* a 32-bit aligned zero in RAM is all we need */
            break;
          case USB_SET_FEATURE:
          case USB_CLEAR_FEATURE:
            USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.bit.STALLRQ1 = 1;
            break;
          case USB_SET_ADDRESS:
            udc_control_send_zlp();
            USB->DEVICE.DADD.reg = USB_DEVICE_DADD_ADDEN | USB_DEVICE_DADD_DADD(request->wValue);
            break;
          case USB_SET_CONFIGURATION:
            usb_config = request->wValue;
            udc_control_send_zlp();
            break;
        }
        break;
      case SIMPLE_USB_CMD(INTERFACE, CLASS):
        switch (request->bRequest) {
          case DFU_GETSTATUS: // DFU_GETSTATUS
            udc_control_send(&dfu_status[0], 6);
            break;
          case DFU_GETSTATE: // DFU_GETSTATE
            udc_control_send(&dfu_status[1], 1);
            break;
          case DFU_DNLOAD: // DFU_DNLOAD
            dfu_status = dfu_status_choices + 0;
            if (request->wLength) {
              dfu_status = dfu_status_choices + 2;
              dfu_addr = FLASH_FW_ADDR + request->wValue * 64;
            } else {
              // zero length packet, end of download
              dfu_status = dfu_status_choices + 4;
              dfu_addr = 0;
              dfu_done = 1;
            }
          // fall through to below
          case DFU_UPLOAD:
          case DFU_ABORT:
          case DFU_CLRSTATUS:
          case DFU_DETACH:
          default:
            /* 0x04 == DFU_CLRSTATUS, 0x06 == DFU_ABORT, and 0x01 == DFU_DNLOAD and 0x02 == DFU_UPLOAD */
            if (!dfu_addr) {
              udc_control_send_zlp();
            }
            break;
        }
        break;
    }
  }
}

static bool flash_valid()
{
  unsigned sp = ((unsigned*)FLASH_FW_ADDR)[0];
  unsigned ip = ((unsigned*)FLASH_FW_ADDR)[1];

  // sp needs to point somewhere inside the RAM
  // ip needs to point into the flash
  return     sp > 0x20000000
             && ip >= 0x00001000
             && ip <  0x00400000;
}


static bool wdt_reset_entry_condition(void)
{
  // Was reset caused by watchdog timer (WDT)?
  // but RTC not running
#ifdef __SAME54N19A__
  return ((RSTC->RCAUSE.reg & RSTC_RCAUSE_WDT) && !(RTC->MODE1.CTRLA.reg & RTC_MODE1_CTRLA_ENABLE));
#else
  return ((PM->RCAUSE.reg & PM_RCAUSE_WDT) && !(RTC->MODE1.CTRL.reg & RTC_MODE0_CTRL_ENABLE));
#endif
}

void bootloader(void)
{
  if (!flash_valid() || usb_dongle_present() || wdt_reset_entry_condition()) {
    goto run_bootloader;
  }
  jump_to_flash(FLASH_FW_ADDR, 0);

run_bootloader:
  // configure system to run on external XTAL
  clock_init_crystal(GCLK_SYSTEM, 1);

  // startup i2c
  i2c_setup();

  //  initialize USB
  pin_mux(PIN_DP);
  pin_mux(PIN_DM);
#ifdef __SAME54N19A__
  MCLK->APBBMASK.reg |= MCLK_APBBMASK_USB;
  //GCLK_USB == 10 (table 14-9)
  GCLK->PCHCTRL[10].reg = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN;
#else
  PM->APBBMASK.reg |= PM_APBBMASK_USB;
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_ID(USB_GCLK_ID) | GCLK_CLKCTRL_GEN(0);
#endif

  USB->DEVICE.CTRLA.reg = USB_CTRLA_SWRST;
  while (USB->DEVICE.SYNCBUSY.bit.SWRST);

#ifdef __SAME54N19A__
  USB->DEVICE.PADCAL.reg = USB_PADCAL_TRANSN(NVM_READ_CAL(USB_FUSES_TRANSN)) | USB_PADCAL_TRANSP(NVM_READ_CAL(USB_FUSES_TRANSP)) | USB_PADCAL_TRIM(NVM_READ_CAL(USB_FUSES_TRIM));
#else
  USB->DEVICE.PADCAL.reg = USB_PADCAL_TRANSN(NVM_READ_CAL(NVM_USB_TRANSN)) | USB_PADCAL_TRANSP(NVM_READ_CAL(NVM_USB_TRANSP)) | USB_PADCAL_TRIM(NVM_READ_CAL(NVM_USB_TRIM));
#endif

  USB->DEVICE.DESCADD.reg = (uint32_t)udc_mem;

  USB->DEVICE.CTRLA.reg = USB_CTRLA_MODE_DEVICE | USB_CTRLA_RUNSTDBY;
  USB->DEVICE.CTRLB.reg = USB_DEVICE_CTRLB_SPDCONF_FS;
  USB->DEVICE.CTRLA.reg |= USB_CTRLA_ENABLE;

  // service USB
  int cnt = 0, cnt2 = 0;
  while (1)  {
    USB_Service();
    if (dfu_done == 1) {
      if (cnt2++ == 5 * LED_BLINK_CYCLES) {
        i2c_cleanup();
        USB->DEVICE.CTRLA.reg &= !USB_CTRLA_ENABLE;
        jump_to_flash(FLASH_FW_ADDR, 0);
      }
    }
    if (cnt++ == LED_BLINK_CYCLES) {
      i2c_led_toggle();
      cnt = 0;
    }
  }
  static char str[] = "123456789";
  crc_t crc;

  crc = crc_init();
  crc = crc_update(crc, (unsigned char*)str, strlen(str));
  crc = crc_finalize(crc);

  //printf("0x%lx\n", (unsigned long)crc);
}