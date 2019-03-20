/*
 * utils.c
 *
 * Created: 11.01.2019 09:45:56
 *  Author: Carsten
 */

#include <sam.h>
#include <stdbool.h>
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

void pin_out(Pin p)
{
  PORT->Group[p.group].PINCFG[p.pin].bit.PMUXEN = 0;
  PORT->Group[p.group].DIRSET.reg = (1 << p.pin);
}

void pin_low(Pin p)
{
  PORT->Group[p.group].OUTCLR.reg = (1 << p.pin);
}

void pin_toggle(Pin p)
{
  PORT->Group[p.group].OUTTGL.reg = (1 << p.pin);
}

void pin_high(Pin p)
{
  PORT->Group[p.group].OUTSET.reg = (1 << p.pin);
}

void pin_in(Pin p)
{
  PORT->Group[p.group].PINCFG[p.pin].bit.PMUXEN = 0;
  PORT->Group[p.group].PINCFG[p.pin].bit.INEN = 1;
  PORT->Group[p.group].DIRCLR.reg = (1 << p.pin);
}

void pin_pull_up(Pin p)
{
  PORT->Group[p.group].PINCFG[p.pin].bit.PMUXEN = 0;
  PORT->Group[p.group].PINCFG[p.pin].bit.PULLEN = 1;
  pin_high(p);
}

bool pin_read(Pin p)
{
  return (PORT->Group[p.group].IN.reg & (1 << p.pin)) != 0;
}

void delay_8_cycles(uint32_t cy)
{
  // when compiled with -O3 and -g3 this loop takes
  // 8 instructions for one iteration
  // subne
  // nopeq
  // nopeq
  // nopeq
  // nopeq
  // b
  // cmp
  // bhi
  cy = cy / 8;
  while (cy > 1) {
    cy--;
    // we need this nop, otherwise the optimizer
    // will throw the whole thing away since it has no side effects
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
  }
  return;
}


void jump_to_flash(uint32_t addr_p, uint32_t r0_val)
{
  uint32_t* addr = (void*) addr_p;

  // Disable SysTick
  SysTick->CTRL = 0;

  // Switch to the the interrupt vector table in flash
  SCB->VTOR = (uint32_t) addr;

  // Set up the stack and jump to the reset vector
  uint32_t sp = addr[0];
  uint32_t pc = addr[1];
  register uint32_t r0 __asm__("r0") = r0_val;
  __asm__ volatile("mov sp, %0; bx %1" :: "r"(sp), "r"(pc), "r"(r0));
  (void) r0_val;
}