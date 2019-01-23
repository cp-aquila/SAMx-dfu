#include <stdint.h>
#include <sam.h>

#define NVM_DFLL_COARSE_POS    58
#define NVM_DFLL_COARSE_SIZE   6
#define NVM_DFLL_FINE_POS      64
#define NVM_DFLL_FINE_SIZE     10

uint32_t dfll_nvm_val()
{
#ifdef __SAME54N19A__
  return OSCCTRL_DFLLVAL_COARSE(0x1f) | OSCCTRL_DFLLVAL_FINE(0x1ff);
#else
  uint32_t coarse = (*((uint32_t*)(NVMCTRL_OTP4)
                       + (NVM_DFLL_COARSE_POS / 32))
                     >> (NVM_DFLL_COARSE_POS % 32))
                    & ((1 << NVM_DFLL_COARSE_SIZE) - 1);
  if (coarse == 0x3f) {
    coarse = 0x1f;
  }
  uint32_t fine = (*((uint32_t*)(NVMCTRL_OTP4)
                     + (NVM_DFLL_FINE_POS / 32))
                   >> (NVM_DFLL_FINE_POS % 32))
                  & ((1 << NVM_DFLL_FINE_SIZE) - 1);
  if (fine == 0x3ff) {
    fine = 0x1ff;
  }

  return SYSCTRL_DFLLVAL_COARSE(coarse) | SYSCTRL_DFLLVAL_FINE(fine);
#endif
}

void dfll_wait_for_sync()
{
#ifdef __SAME54N19A__
  while (!OSCCTRL->INTFLAG.bit.DFLLRDY);
#else
  while (!SYSCTRL->PCLKSR.bit.DFLLRDY);
#endif
}

void gclk_enable(uint32_t id, uint32_t src, uint32_t div)
{
#ifdef __SAME54N19A__
  GCLK->GENCTRL[id].reg = GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC(src) | GCLK_GENCTRL_DIV(div);
#else
  GCLK->GENDIV.reg = GCLK_GENDIV_ID(id) | GCLK_GENDIV_DIV(div);
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(id) | GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC(src);
#endif
}

void gclk_init()
{
  // Initialize GCLK
#ifdef __SAME54N19A__
  MCLK->APBAMASK.reg |= MCLK_APBAMASK_GCLK;
  GCLK->CTRLA.reg = GCLK_CTRLA_SWRST;
  while (GCLK->CTRLA.reg & GCLK_CTRLA_SWRST);

#else
  // Various bits in the INTFLAG register can be set to one at startup.
  SYSCTRL->INTFLAG.reg = SYSCTRL_INTFLAG_BOD33RDY | SYSCTRL_INTFLAG_BOD33DET |  SYSCTRL_INTFLAG_DFLLRDY;

  PM->APBAMASK.reg |= PM_APBAMASK_GCLK;
  GCLK->CTRL.reg = GCLK_CTRL_SWRST;
  while (GCLK->CTRL.reg & GCLK_CTRL_SWRST);
#endif
}

void clock_init_crystal(uint8_t clk_system, uint8_t clk_32k)
{
  gclk_init();

#ifdef __SAME54N19A__
  // configure XOSC1 for 12Mhz XTAL Operation
  OSCCTRL->XOSCCTRL[1].reg = OSCCTRL_XOSCCTRL_ENABLE | OSCCTRL_XOSCCTRL_XTALEN |  OSCCTRL_XOSCCTRL_IPTAT(3) | OSCCTRL_XOSCCTRL_IMULT(4) | OSCCTRL_XOSCCTRL_CFDPRESC(3);
  while (!(OSCCTRL->STATUS.reg & OSCCTRL_STATUS_XOSCRDY1));
  // configure DPLL0 with GCLK1 source, output 48Mhz
  OSCCTRL->Dpll[0].DPLLCTRLB.reg = OSCCTRL_DPLLCTRLB_REFCLK_XOSC1 | OSCCTRL_DPLLCTRLB_DIV(2); // effective divider 6 (28.8.14)
  OSCCTRL->Dpll[0].DPLLRATIO.reg = OSCCTRL_DPLLRATIO_LDR(23); // effective multiplier 24
  OSCCTRL->Dpll[0].DPLLCTRLA.reg = OSCCTRL_DPLLCTRLA_ENABLE;
  while (!(OSCCTRL->Dpll[0].DPLLSTATUS.reg & OSCCTRL_DPLLSTATUS_LOCK));

  // run GCLK0 (48MHz)
  gclk_enable(0, GCLK_GENCTRL_SRC_DPLL0, 1);

  // GCLK1 (32kHz)
  gclk_enable(1, GCLK_GENCTRL_SRC_OSCULP32K, 1);
#else
  SYSCTRL->XOSC32K.reg
    = SYSCTRL_XOSC32K_ENABLE
      | SYSCTRL_XOSC32K_XTALEN
      | SYSCTRL_XOSC32K_EN32K
      | SYSCTRL_XOSC32K_STARTUP(0x03)
      | SYSCTRL_XOSC32K_RUNSTDBY;

  while (!SYSCTRL->PCLKSR.bit.XOSC32KRDY);


  gclk_enable(clk_32k, GCLK_SOURCE_XOSC32K, 1);

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |
                      GCLK_CLKCTRL_GEN(clk_32k) |
                      GCLK_CLKCTRL_ID(SYSCTRL_GCLK_ID_DFLL48);

  SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_ENABLE;
  dfll_wait_for_sync();
  SYSCTRL->DFLLVAL.reg = dfll_nvm_val();
  dfll_wait_for_sync();
  SYSCTRL->DFLLMUL.reg
    = SYSCTRL_DFLLMUL_MUL(1465) // round(48000000 / 32768)
      | SYSCTRL_DFLLMUL_CSTEP((0x1f / 4))
      | SYSCTRL_DFLLMUL_FSTEP((0xff / 4));
  dfll_wait_for_sync();
  SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_ENABLE | SYSCTRL_DFLLCTRL_MODE;
  dfll_wait_for_sync();
  SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_ENABLE | SYSCTRL_DFLLCTRL_MODE | SYSCTRL_DFLLCTRL_ONDEMAND;

  gclk_enable(clk_system, GCLK_SOURCE_DFLL48M, 1);
  while (GCLK->STATUS.bit.SYNCBUSY);
#endif
}
