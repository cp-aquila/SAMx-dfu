#include <stdbool.h>
#include <string.h>
#include <sam.h>
#include "clock.h"
#include "utils.h"

const static Pin PIN_VBUS = {.group = 0, .pin = 13, .mux = MUX_PA13A_EIC_EXTINT13 };
const static Pin PIN_DP   = {.group = 0, .pin = 25, .mux = MUX_PA25G_USB_DP };
const static Pin PIN_DM   = {.group = 0, .pin = 24, .mux = MUX_PA24G_USB_DM };
const static Pin PIN_SCL  = {.group = 0, .pin = 16, .mux = MUX_PA16C_SERCOM1_PAD0 };
const static Pin PIN_SDA  = {.group = 0, .pin = 17, .mux = MUX_PA17C_SERCOM1_PAD1 };

static uint8_t mcp23008_gpio_val;

#define I2C_ADDR_MCP23008 (32)
#define I2C_BAUD_VAL 59 // 48Mhz to 400kHz
#define MCP23008_REG_IODIR   0x00
#define MCP23008_REG_GPIO    0x09

#define MCP23008_BIT_LED_GREEN   (0)
#define MCP23008_BIT_LED_RED     (1)
#define MCP23008_BIT_AMP_NPWR    (2)
#define MCP23008_BIT_PUMP_NSLEEP (3)
#define MCP23008_BIT_PUMP_IN2    (4)
#define MCP23008_BIT_PUMP_IN1    (5)
#define MCP23008_BIT_PWR         (6)
#define MCP23008_BIT_PWR_STB     (7)

#define GCLK_SERCOM1_CORE 0x15
#define GCLK_SERCOMx_SLOW 0x13

static bool i2c_master_wait_for_bus(void)
{
  // Wait for reply.
  uint16_t timeout_counter = 0;
  while (!(SERCOM1->I2CM.INTFLAG.reg & SERCOM_I2CM_INTFLAG_MB) &&
         !(SERCOM1->I2CM.INTFLAG.reg & SERCOM_I2CM_INTFLAG_SB)) {

    // Check timeout condition
    if (++timeout_counter >= 10000) {
      return false;
    }
  }
  return true;
}

static inline void i2c_master_wait_for_sync()
{
  while (SERCOM1->I2CM.SYNCBUSY.reg & SERCOM_I2CM_SYNCBUSY_MASK) {
  }
}

static void i2c_mcp_data(uint8_t reg, uint8_t data)
{
  // Set action to ACK
  i2c_master_wait_for_sync();
  SERCOM1->I2CM.CTRLB.reg &= ~SERCOM_I2CM_CTRLB_ACKACT;

  i2c_master_wait_for_sync();
  SERCOM1->I2CM.ADDR.reg = (I2C_ADDR_MCP23008 << 1);
  i2c_master_wait_for_bus();

  // check if slave is on bus
  if (SERCOM1->I2CM.STATUS.bit.RXNACK == false) {
    // check if we own the bus
    if (SERCOM1->I2CM.STATUS.reg & SERCOM_I2CM_STATUS_BUSSTATE(2)) {
      // write the first byte..
      i2c_master_wait_for_sync();
      SERCOM1->I2CM.DATA.reg = reg;
      i2c_master_wait_for_bus();
    }

    if (SERCOM1->I2CM.STATUS.reg & SERCOM_I2CM_STATUS_BUSSTATE(2)) {
      // write the second byte
      i2c_master_wait_for_sync();
      SERCOM1->I2CM.DATA.reg = data;
      i2c_master_wait_for_bus();
    }

    // send stop
    i2c_master_wait_for_sync();
    SERCOM1->I2CM.CTRLB.reg |= SERCOM_I2CM_CTRLB_CMD(3);
  }
}

void i2c_led_toggle(void)
{
  if (mcp23008_gpio_val & (1 << MCP23008_BIT_LED_GREEN)) {
    mcp23008_gpio_val &= 0xFC;
    mcp23008_gpio_val |= (1 << MCP23008_BIT_LED_RED);
  } else {
    mcp23008_gpio_val &= 0xFC;
    mcp23008_gpio_val |= (1 << MCP23008_BIT_LED_GREEN);
  }
  i2c_mcp_data(MCP23008_REG_GPIO, mcp23008_gpio_val);
}


void i2c_setup(void)
{
  // set pin muxes
  pin_mux(PIN_SCL);
  pin_mux(PIN_SDA);

  // turn on clock to module
  PM->APBCMASK.reg |= 1 << (PM_APBCMASK_SERCOM1_Pos);

  // attach sercom slow clock to generator 1 (32kHz)
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCLK_SERCOMx_SLOW) | GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(1);
  // attach sercom1 clock to generator 0 (48Mhz)
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCLK_SERCOM1_CORE) | GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(0);

  // reset module
  SERCOM1->SPI.CTRLA.reg = SERCOM_SPI_CTRLA_SWRST;
  while (SERCOM1->SPI.CTRLA.reg & SERCOM_SPI_CTRLA_SWRST);

  // configure module as master
  i2c_master_wait_for_sync();
  SERCOM1->I2CM.CTRLA.reg = SERCOM_I2CM_CTRLA_MODE_I2C_MASTER | SERCOM_I2CM_CTRLA_SDAHOLD(2) | SERCOM_I2CM_CTRLA_RUNSTDBY;

  // set baud rate
  i2c_master_wait_for_sync();
  SERCOM1->I2CM.BAUD.reg = I2C_BAUD_VAL;

  // enable smart mode
  i2c_master_wait_for_sync();
  SERCOM1->I2CM.CTRLB.reg = SERCOM_I2CM_CTRLB_SMEN;

  // enable the module
  i2c_master_wait_for_sync();
  SERCOM1->I2CM.CTRLA.reg |= SERCOM_I2CM_CTRLA_ENABLE;

  // force bus into idle state
  i2c_master_wait_for_sync();
  SERCOM1->I2CM.STATUS.reg = SERCOM_I2CM_STATUS_BUSSTATE(1);

  // set initial state
  mcp23008_gpio_val = (0 << MCP23008_BIT_LED_GREEN)
                      | (1 << MCP23008_BIT_LED_RED)     // LEDs are inverted, start with green on and red off
                      | (0 << MCP23008_BIT_AMP_NPWR)
                      | (1 << MCP23008_BIT_PUMP_NSLEEP)
                      | (1 << MCP23008_BIT_PUMP_IN2)
                      | (1 << MCP23008_BIT_PUMP_IN1)
                      | (1 << MCP23008_BIT_PWR)         // don't shut down :)
                      | (0 << MCP23008_BIT_PWR_STB);

  i2c_mcp_data(MCP23008_REG_GPIO, mcp23008_gpio_val);
  // set all gpios to outputs
  i2c_mcp_data(MCP23008_REG_IODIR, 0x00);
}

void i2c_cleanup(void)
{
  // reset module
  SERCOM1->SPI.CTRLA.reg = SERCOM_SPI_CTRLA_SWRST;
  while (SERCOM1->SPI.CTRLA.reg & SERCOM_SPI_CTRLA_SWRST);
}

bool usb_dongle_present(void)
{
  bool dm, dp, vb;

  pin_in(PIN_VBUS);
  delay_cycles(100);
  vb = pin_read(PIN_VBUS);
  if (vb == true) {
    // there is power on VBUS. better stop right here
    return false;
  }

  // step1, drive dp low, check if both pins have a low level
  pin_out(PIN_DP);
  pin_low(PIN_DP);
  pin_pull_up(PIN_DM);
  delay_cycles(100);
  dm = pin_read(PIN_DM);
  dp = pin_read(PIN_DP);
  if ((dm == true) || (dp == true)) {
    return false;
  }

  // step2, drive dm low, check if both pins have a low level
  pin_out(PIN_DM);
  pin_low(PIN_DM);
  pin_pull_up(PIN_DP);
  delay_cycles(100);
  dm = pin_read(PIN_DM);
  dp = pin_read(PIN_DP);
  if ((dm == true) || (dp == true)) {
    return false;
  }

  // all checks were positive
  pin_in(PIN_DP);
  pin_in(PIN_DM);
  return true;
}