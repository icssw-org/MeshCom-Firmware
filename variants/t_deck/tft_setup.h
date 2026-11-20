// TFT_eSPI setup for the T-Deck (ST7789 240x320), picked up via __has_include(<tft_setup.h>).
// Same as lib/TFT_eSPI/User_Setups/Setup210_LilyGo_T_Deck.h, but only the GLCD font:
// TFT_eSPI is used as a pure display driver here, all text is drawn by LVGL.
#define USER_SETUP_ID 210

#define ST7789_DRIVER

#define TFT_WIDTH  240
#define TFT_HEIGHT 320

#define TFT_RGB_ORDER TFT_BGR

#define TFT_MISO 38
#define TFT_MOSI 41
#define TFT_SCLK 40
#define TFT_CS   12 // Not connected
#define TFT_DC   11
#define TFT_RST  -1
#define TFT_BACKLIGHT_ON 1
#define TFT_BL     42

#define LOAD_GLCD

#define SPI_FREQUENCY  40000000
#define SPI_READ_FREQUENCY  20000000
#define SPI_TOUCH_FREQUENCY  2500000
