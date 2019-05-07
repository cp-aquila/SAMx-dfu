#ifndef _PERIPHERALS_H_
#define _PERIPHERALS_H_

#ifdef __SAME54N19A__
const static Pin PIN_I2C_SCL    = {.group = 0, .pin = 16, .mux = MUX_PA16C_SERCOM1_PAD0 };
const static Pin PIN_I2C_SDA    = {.group = 0, .pin = 17, .mux = MUX_PA17C_SERCOM1_PAD1 };
const static Pin PIN_USB_VBUS   = {.group = 0, .pin = 21, .mux = 0 };
const static Pin PIN_USB_DP     = {.group = 0, .pin = 25, .mux = MUX_PA25H_USB_DP };
const static Pin PIN_USB_DM     = {.group = 0, .pin = 24, .mux = MUX_PA24H_USB_DM };
const static Pin PIN_FLASH_MISO = {.group = 1, .pin = 11, .mux = MUX_PB11D_SERCOM4_PAD3 };
const static Pin PIN_FLASH_MOSI = {.group = 1, .pin = 8,  .mux = MUX_PB08D_SERCOM4_PAD0 };
const static Pin PIN_FLASH_SCK  = {.group = 1, .pin = 9,  .mux = MUX_PB09D_SERCOM4_PAD1 };
const static Pin PIN_FLASH_CS   = {.group = 1, .pin = 10, .mux = 0 };
const static Pin PIN_APA_MOSI   = {.group = 0, .pin = 0,  .mux = MUX_PA00D_SERCOM1_PAD0 };
const static Pin PIN_APA_SCK    = {.group = 0, .pin = 1,  .mux = MUX_PA01D_SERCOM1_PAD1 };
const static Pin PIN_SW2        = {.group = 1, .pin = 31, .mux = 0 };

static const uint8_t apa102_led_s0[16] = {0, 0, 0, 0, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0xFF, 0xFF, 0xFF};
static const uint8_t apa102_led_s1[16] = {0, 0, 0, 0, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

#else
const static Pin PIN_I2C_SCL    = {.group = 0, .pin = 16, .mux = MUX_PA16C_SERCOM1_PAD0 };
const static Pin PIN_I2C_SDA    = {.group = 0, .pin = 17, .mux = MUX_PA17C_SERCOM1_PAD1 };
const static Pin PIN_USB_VBUS   = {.group = 0, .pin = 13, .mux = MUX_PA13A_EIC_EXTINT13 };
const static Pin PIN_USB_DP     = {.group = 0, .pin = 25, .mux = MUX_PA25G_USB_DP };
const static Pin PIN_USB_DM     = {.group = 0, .pin = 24, .mux = MUX_PA24G_USB_DM };
const static Pin PIN_FLASH_MISO = {.group = 1, .pin = 22, .mux = MUX_PA22D_SERCOM5_PAD0 };
const static Pin PIN_FLASH_MOSI = {.group = 1, .pin = 23, .mux = MUX_PA23D_SERCOM5_PAD1 };
const static Pin PIN_FLASH_SCK  = {.group = 1, .pin = 3,  .mux = MUX_PB03D_SERCOM5_PAD1 };
const static Pin PIN_FLASH_CS   = {.group = 1, .pin = 2,  .mux = 0 };
#endif


#ifdef __SAME54N19A__
#define FLASH_SERCOM_MODULE   SERCOM4
#define FLASH_SERCOM_DIPO     (3)
#define FLASH_SERCOM_DOPO     (0)
#define FLASH_SERCOM_BAUD_VAL (5)

#define APA101_SERCOM_MODULE   SERCOM1
#define APA101_SERCOM_DIPO     (3)
#define APA101_SERCOM_DOPO     (0)
#define APA101_SERCOM_BAUD_VAL (5)
#else

#define I2C_SERCOM_MODULE     SERCOM1

#define FLASH_SERCOM_MODULE   SERCOM5
#define FLASH_SERCOM_DIPO     (2)
#define FLASH_SERCOM_DOPO     (2)
#define FLASH_SERCOM_BAUD_VAL (5)
#endif

void i2c_led_toggle(void);
void i2c_setup(void);

void apa102_led_toggle(void);
void apa102_led_setup(void);

void spi_flash_setup(void);
bool spi_flash_check(void);
void spi_flash_read(int addr, uint8_t* buf, size_t size);

void do_cleanup(void);
bool usb_dongle_present(void);

#endif