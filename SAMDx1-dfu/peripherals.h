#ifndef _PERIPHERALS_H_
#define _PERIPHERALS_H_

#ifdef __SAME54N19A__
const static Pin PIN_SCL  = {.group = 0, .pin = 16, .mux = MUX_PA16C_SERCOM1_PAD0 };
const static Pin PIN_SDA  = {.group = 0, .pin = 17, .mux = MUX_PA17C_SERCOM1_PAD1 };
const static Pin PIN_VBUS = {.group = 0, .pin = 21, .mux = 0 };
const static Pin PIN_DP   = {.group = 0, .pin = 25, .mux = MUX_PA25H_USB_DP };
const static Pin PIN_DM   = {.group = 0, .pin = 24, .mux = MUX_PA24H_USB_DM };
const static Pin PIN_LEDR = {.group = 0, .pin = 1,  .mux = MUX_PA01A_EIC_EXTINT1 };
const static Pin PIN_LEDG = {.group = 0, .pin = 0,  .mux = MUX_PA00A_EIC_EXTINT0 };
const static Pin PIN_MISO = {.group = 1, .pin = 11, .mux = MUX_PB11D_SERCOM4_PAD3 };
const static Pin PIN_MOSI = {.group = 1, .pin = 8,  .mux = MUX_PB08D_SERCOM4_PAD0 };
const static Pin PIN_SCK  = {.group = 1, .pin = 9,  .mux = MUX_PB09D_SERCOM4_PAD1 };
const static Pin PIN_CS  =  {.group = 1, .pin = 10, .mux = 0 };
#else
const static Pin PIN_SCL  = {.group = 0, .pin = 16, .mux = MUX_PA16C_SERCOM1_PAD0 };
const static Pin PIN_SDA  = {.group = 0, .pin = 17, .mux = MUX_PA17C_SERCOM1_PAD1 };
const static Pin PIN_VBUS = {.group = 0, .pin = 13, .mux = MUX_PA13A_EIC_EXTINT13 };
const static Pin PIN_DP   = {.group = 0, .pin = 25, .mux = MUX_PA25G_USB_DP };
const static Pin PIN_DM   = {.group = 0, .pin = 24, .mux = MUX_PA24G_USB_DM };
const static Pin PIN_MISO = {.group = 1, .pin = 22, .mux = MUX_PA22D_SERCOM5_PAD0 };
const static Pin PIN_MOSI = {.group = 1, .pin = 23, .mux = MUX_PA23D_SERCOM5_PAD1 };
const static Pin PIN_SCK  = {.group = 1, .pin = 3,  .mux = MUX_PB03D_SERCOM5_PAD1 };
const static Pin PIN_CS  =  {.group = 1, .pin = 2,  .mux = 0 };
#endif


#ifdef __SAME54N19A__
#define SERCOM_MODULE   SERCOM4
#define SERCOM_DIPO     (3)
#define SERCOM_DOPO     (0)
#define SERCOM_BAUD_VAL (5)
#else
#define SERCOM_MODULE   SERCOM5
#define SERCOM_DIPO     (2)
#define SERCOM_DOPO     (2)
#define SERCOM_BAUD_VAL (5)
#endif

void i2c_led_toggle(void);
void i2c_setup(void);
void i2c_cleanup(void);

bool usb_dongle_present(void);

#endif