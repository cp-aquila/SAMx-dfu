#ifndef _PERIPHERALS_H_
#define _PERIPHERALS_H_

const static Pin PIN_VBUS = {.group = 0, .pin = 13, .mux = MUX_PA13A_EIC_EXTINT13 };
#ifdef __SAME54N19A__
const static Pin PIN_DP   = {.group = 0, .pin = 25, .mux = MUX_PA25H_USB_DP };
const static Pin PIN_DM   = {.group = 0, .pin = 24, .mux = MUX_PA24H_USB_DM };
#else
const static Pin PIN_DP   = {.group = 0, .pin = 25, .mux = MUX_PA25G_USB_DP };
const static Pin PIN_DM   = {.group = 0, .pin = 24, .mux = MUX_PA24G_USB_DM };
#endif
const static Pin PIN_SCL  = {.group = 0, .pin = 16, .mux = MUX_PA16C_SERCOM1_PAD0 };
const static Pin PIN_SDA  = {.group = 0, .pin = 17, .mux = MUX_PA17C_SERCOM1_PAD1 };

void i2c_led_toggle(void);
void i2c_setup(void);
void i2c_cleanup(void);

bool usb_dongle_present(void);

#endif