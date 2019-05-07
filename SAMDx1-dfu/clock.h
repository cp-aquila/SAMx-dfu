#ifndef _CLOCK_H_
#define _CLOCK_H_

#include <stdint.h>
#include <sam.h>

void clock_init_crystal(void);
void gclk_enable(uint32_t id, uint32_t src, uint32_t div);

#endif