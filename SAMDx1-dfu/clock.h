#ifndef _CLOCK_H_
#define _CLOCK_H_

#include <stdint.h>
#include <sam.h>

void clock_init_crystal(uint8_t clk_system, uint8_t clk_32k);
void gclk_enable(uint32_t id, uint32_t src, uint32_t div);

#endif