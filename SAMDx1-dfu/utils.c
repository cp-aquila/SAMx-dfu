/*
 * utils.c
 *
 * Created: 11.01.2019 09:45:56
 *  Author: Carsten
 */

#include <sam.h>
#include "utils.h"

void pin_mux(Pin p)
{
  if (p.pin & 1) {
    PORT->Group[p.group].PMUX[p.pin / 2].bit.PMUXO = p.mux;
  } else {
    PORT->Group[p.group].PMUX[p.pin / 2].bit.PMUXE = p.mux;
  }

  PORT->Group[p.group].PINCFG[p.pin].bit.PMUXEN = 1;
}

void jump_to_flash(uint32_t addr_p, uint32_t r0_val)
{
  uint32_t* addr = (void*) addr_p;
  __disable_irq();

  // Disable SysTick
  SysTick->CTRL = 0;

  // TODO: reset peripherals

  // Switch to the the interrupt vector table in flash
  SCB->VTOR = (uint32_t) addr;

  // Set up the stack and jump to the reset vector
  uint32_t sp = addr[0];
  uint32_t pc = addr[1];
  register uint32_t r0 __asm__("r0") = r0_val;
  __asm__ volatile("mov sp, %0; bx %1" :: "r"(sp), "r"(pc), "r"(r0));
  (void) r0_val;
}