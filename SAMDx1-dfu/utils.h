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

#ifndef _UTILS_H_
#define _UTILS_H_

/*- Definitions -------------------------------------------------------------*/
#define PACK            __attribute__((packed))

#define INLINE          static inline __attribute__((always_inline))

#define LIMIT(a, b)     (((a) > (b)) ? (b) : (a))

#define ATSAMR21G18_MR210UA_USERROW_ADDRESS (0x804008)
#define ATSAMR21G18_MR210UA_USERROW_MIB_VERSION (0x1501)


/*- Typedefs -------------------------------------------------------------*/
typedef struct Pin {
  uint8_t mux;
  uint8_t group;
  uint8_t pin;
  uint8_t chan;
} Pin;

typedef struct __attribute__((packed))
{
  uint16_t mib_revision;
  uint8_t  ieee_address[8];
  char     board_serial[10];
  char     atmel_part_number[8];
  uint8_t  pcba_rev;
  uint8_t  xtal_trim;
  uint16_t crc;
}
ee_data_t;

/*- Prototypes -------------------------------------------------------------*/
void pin_mux(Pin p);
void pin_out(Pin p);
void pin_low(Pin p);
void pin_high(Pin p);
void pin_in(Pin p);
void pin_pull_up(Pin p);
bool pin_read(Pin p);

void delay_cycles(uint32_t cy);

void jump_to_flash(uint32_t addr_p, uint32_t r0_val);

#endif // _UTILS_H_

