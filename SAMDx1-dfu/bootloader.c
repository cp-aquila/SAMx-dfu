/*
 * 1kByte USB DFU bootloader for Atmel SAMD11 microcontrollers
 *
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
#include "usb.h"
#include "nvm_data.h"
#include "usb_descriptors.h"

/*- Definitions -------------------------------------------------------------*/
#define USB_CMD(dir, rcpt, type) ((USB_##dir##_TRANSFER << 7) | (USB_##type##_REQUEST << 5) | (USB_##rcpt##_RECIPIENT << 0))
#define SIMPLE_USB_CMD(rcpt, type) ((USB_##type##_REQUEST << 5) | (USB_##rcpt##_RECIPIENT << 0))
#define GCLK_SYSTEM 0
#define FLASH_BOOT_SIZE 4096
#define FLASH_FW_ADDR   FLASH_BOOT_SIZE


/*- Types -------------------------------------------------------------------*/
typedef struct {
  UsbDeviceDescBank  out;
  UsbDeviceDescBank  in;
} udc_mem_t;

/*- Variables ---------------------------------------------------------------*/
static uint32_t usb_config = 0;
static uint32_t dfu_status_choices[4] = {
  0x00000000, 0x00000002, /* normal */
  0x00000000, 0x00000005, /* dl */
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

  // Handle incoming DFU data packets on
  if (USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.bit.TRCPT0) {
    if (dfu_addr) {
      if (0 == ((dfu_addr >> 6) & 0x3)) {
        NVMCTRL->ADDR.reg = dfu_addr >> 1;
        NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD(NVMCTRL_CTRLA_CMD_ER);
        while (!NVMCTRL->INTFLAG.bit.READY);
      }

      uint16_t* nvm_addr = (uint16_t*)(dfu_addr);
      uint16_t* ram_addr = (uint16_t*)udc_ctrl_out_buf;
      for (unsigned i = 0; i < 32; i++)
      { *nvm_addr++ = *ram_addr++; }
      while (!NVMCTRL->INTFLAG.bit.READY);

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
      if ((request->bRequest == USB_GET_MSFT) && (request->wIndex = 0x0004)) {
        udc_control_send((uint32_t*)&msft_compatible, msft_compatible.dwLength);
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
                    udc_control_send((uint32_t*)&usb_string_lang, usb_string_lang.bLength);
                    break;
                  case USB_STRING_MANU:
                  case USB_STRING_PRODUCT:
                    udc_control_send((uint32_t*)&usb_string_manu, usb_string_manu.bLength);
                    break;
                  case USB_STRING_SERIAL:
                    udc_control_send((uint32_t*)&usb_string_serial, usb_string_serial.bLength);
                    break;
                  case USB_STRING_DFU_FLASH:
                    udc_control_send((uint32_t*)&usb_string_dfu_flash, usb_string_dfu_flash.bLength);
                    break;
                  case USB_STRING_F0:
                    udc_control_send((uint32_t*)&usb_string_empty, usb_string_empty.bLength);
                    break;
                  case USB_STRING_MSFT:
                    udc_control_send((uint32_t*)&usb_string_msftos, usb_string_msftos.bLength);
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
            }
          /* fall through to below */
          case DFU_UPLOAD:
          case DFU_ABORT:
          case DFU_DETACH:
          case DFU_CLRSTATUS:
          default: // DFU_UPLOAD & others
            // DFU_DN
            /* 0x00 == DFU_DETACH, 0x04 == DFU_CLRSTATUS, 0x06 == DFU_ABORT, and 0x01 == DFU_DNLOAD and 0x02 == DFU_UPLOAD */
            if (!dfu_addr)
            { udc_control_send_zlp(); }
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

  return     sp > 0x20000000
             && ip >= 0x00001000
             && ip <  0x00400000;
}


static bool wdt_reset_entry_condition(void)
{
  // Was reset caused by watchdog timer (WDT)?
  // but RTC not running
  return ((PM->RCAUSE.reg & PM_RCAUSE_WDT) && !(RTC->MODE1.CTRL.reg & RTC_MODE0_CTRL_ENABLE));
}

static bool button_pressed(void)
{
  return false;
}

inline static void jump_to_flash(uint32_t addr_p, uint32_t r0_val) {
  uint32_t *addr = (void*) addr_p;
  __disable_irq();

  // Disable SysTick
  SysTick->CTRL = 0;

  // TODO: reset peripherals

  // Switch to the the interrupt vector table in flash
  SCB->VTOR = (uint32_t) addr;

  // Set up the stack and jump to the reset vector
  uint32_t sp = addr[0];
  uint32_t pc = addr[1];
  register uint32_t r0 __asm__ ("r0") = r0_val;
  __asm__ volatile("mov sp, %0; bx %1" :: "r" (sp), "r" (pc), "r" (r0));
  (void) r0_val;
}

void bootloader(void)
{
  if (!flash_valid() || button_pressed() || wdt_reset_entry_condition()) {
    goto run_bootloader;
  }
  jump_to_flash(FLASH_FW_ADDR, 0);

run_bootloader:
  clock_init_crystal(GCLK_SYSTEM, 1);
  /*
  initialize USB
  */

  PORT->Group[0].PINCFG[24].reg |= PORT_PINCFG_PMUXEN;
  PORT->Group[0].PINCFG[25].reg |= PORT_PINCFG_PMUXEN;
  PORT->Group[0].PMUX[24 >> 1].reg = PORT_PMUX_PMUXO(PORT_PMUX_PMUXE_G_Val) | PORT_PMUX_PMUXE(PORT_PMUX_PMUXE_G_Val);

  PM->APBBMASK.reg |= PM_APBBMASK_USB;

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_ID(USB_GCLK_ID) | GCLK_CLKCTRL_GEN(0);

  USB->DEVICE.CTRLA.reg = USB_CTRLA_SWRST;
  while (USB->DEVICE.SYNCBUSY.bit.SWRST);

  USB->DEVICE.PADCAL.reg = USB_PADCAL_TRANSN(NVM_READ_CAL(NVM_USB_TRANSN)) | USB_PADCAL_TRANSP(NVM_READ_CAL(NVM_USB_TRANSP)) | USB_PADCAL_TRIM(NVM_READ_CAL(NVM_USB_TRIM));

  USB->DEVICE.DESCADD.reg = (uint32_t)udc_mem;

  USB->DEVICE.CTRLA.reg = USB_CTRLA_MODE_DEVICE | USB_CTRLA_RUNSTDBY;
  USB->DEVICE.CTRLB.reg = USB_DEVICE_CTRLB_SPDCONF_FS;
  USB->DEVICE.CTRLA.reg |= USB_CTRLA_ENABLE;

  /*
  service USB
  */

  while (1)
  { USB_Service(); }
}
