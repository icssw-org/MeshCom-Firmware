// GPIO-01: firmware side of the pin policy (src/pin_policy.h), the ONE translation unit that reads the board's pin macros.
//
// Why a .cpp and not inline functions in a header: the variant macros (LORA_*, RADIO_*, I2C_SDA, BUTTON_PIN, ...) mean
// different things depending on which headers a TU saw before it. src/batt_functions.h includes nrf52/t_echo_utilities.h
// on EVERY nRF52 build (T-Echo radio pins 17, 19-25 and I2C 26/27), so a command_functions.cpp copy of an inline list
// reserved WisBlock IO pins on a RAK4631 that web_functions.cpp (which never sees that header) did not: two different
// bodies under one inline name (ODR) and wrong refusals. esp32_main.cpp likewise builds a LORA_DIO1 alias. Here the
// includes are chosen on purpose: the configuration/variant headers, t_echo_utilities.h ONLY under BOARD_T_ECHO, the T5
// pin header ONLY under BOARD_T5_EPAPER. Callers include pin_board.h (declarations only), in any order.
//
// The reserved list is the macro list that used to live in wxWindPinOk() (WX-01), now compiled on every platform, plus the
// classes docs/gpio-pin-map.md section 3 found missing (display reset, Vext/power enable, ADC control, battery pin) where
// a macro names them. It is split in HARD (radio, SPI bus, Ethernet, SD) and SOFT (I2C, GPS, button, display reset, Vext,
// ADC control, battery): both refuse a user SET, only HARD is repaired at load.
// The user-function defaults (OneWire_GPIO, BUTTON_PIN, ANALOG_PIN) and BOARD_LED are NOT reserved: the defaults are
// exempt in pinCanAssign(), BOARD_LED is a "claimed" pin (pinBoardClaimed()).
//
// The masks are constexpr (kHardMask, kSoftMask), so a regression check can sit at the end of this file as static_assert.
#include <Arduino.h>
#include <configuration.h>
#include <debugconf.h>
#include <meshcom_settings.h>
#if defined(BOARD_T_ECHO)
#include "nrf52/t_echo_utilities.h"      // T-Echo radio / I2C pins; included for no other nRF52 board
#endif
#if defined(BOARD_T5_EPAPER)
#include <t5-epaper/utilities.h>         // BOARD_LORA_IRQ (pin defines only)
#endif
#include "mc_board_id.h"
#include "pin_board.h"

extern bool bONEWIRE;
extern bool bButtonCheck;
extern bool bAnalogCheck;
#if defined(ESP32)
extern bool bWINDON;
#endif
extern uint8_t iButtonPin;   // loop_functions.cpp: the button pin in use since boot (99 = none)

// WX_RES(x) is safe for a macro that is defined EMPTY (LORA_DIO0 on the T-Beam Supreme: "+ 0" makes it 0, which
// only reserves pin 0 = "off" anyway) and for one that is a runtime expression (SS, RST_LoRa: Arduino constants).
#define WX_RES(x) , (int)(x + 0)

namespace
{
// C++11 constexpr: one return expression, recursion instead of a loop
constexpr uint64_t wxBit(int pin) { return (pin < 0 || pin > 63) ? 0ULL : (1ULL << pin); }
constexpr uint64_t wxMask(const int *a, unsigned n, unsigned i) { return i >= n ? 0ULL : (wxBit(a[i]) | wxMask(a, n, i + 1)); }

// HARD: electrically busy whenever the firmware runs (radio, its SPI bus, Ethernet, SD card). Refused when set AND repaired at load.
constexpr int kHard[] = {

        -1
    #if defined(LORA_CS)
    WX_RES(LORA_CS)
    #endif
    #if defined(LORA_NSS)
    WX_RES(LORA_NSS)
    #endif
    #if defined(LORA_RST)
    WX_RES(LORA_RST)
    #endif
    #if defined(LORA_RESET)
    WX_RES(LORA_RESET)
    #endif
    #if defined(LORA_DIO0)
    WX_RES(LORA_DIO0)
    #endif
    #if defined(LORA_DIO1)
    WX_RES(LORA_DIO1)
    #endif
    #if defined(LORA_DIO2)
    WX_RES(LORA_DIO2)
    #endif
    #if defined(LORA_DIO3)
    WX_RES(LORA_DIO3)
    #endif
    #if defined(LORA_SCK)
    WX_RES(LORA_SCK)
    #endif
    #if defined(LORA_MISO)
    WX_RES(LORA_MISO)
    #endif
    #if defined(LORA_MOSI)
    WX_RES(LORA_MOSI)
    #endif
    #if defined(LORA_IRQ)
    WX_RES(LORA_IRQ)
    #endif
    #if defined(LORA_IO0)
    WX_RES(LORA_IO0)
    #endif
    #if defined(LORA_IO1)
    WX_RES(LORA_IO1)
    #endif
    #if defined(LORA_IO2)
    WX_RES(LORA_IO2)
    #endif
    #if defined(SX1262X_CS)
    WX_RES(SX1262X_CS)
    #endif
    #if defined(SX1262X_IRQ)
    WX_RES(SX1262X_IRQ)
    #endif
    #if defined(SX1262X_RST)
    WX_RES(SX1262X_RST)
    #endif
    #if defined(SX1262X_GPIO)
    WX_RES(SX1262X_GPIO)
    #endif
    #if defined(SX1268_CS)
    WX_RES(SX1268_CS)
    #endif
    #if defined(SX1268_IRQ)
    WX_RES(SX1268_IRQ)
    #endif
    #if defined(SX1268_RST)
    WX_RES(SX1268_RST)
    #endif
    #if defined(SX1268_GPIO)
    WX_RES(SX1268_GPIO)
    #endif
    #if defined(SX126x_CS)
    WX_RES(SX126x_CS)
    #endif
    #if defined(SX126x_IRQ)
    WX_RES(SX126x_IRQ)
    #endif
    #if defined(SX126x_RST)
    WX_RES(SX126x_RST)
    #endif
    #if defined(SX126x_GPIO)
    WX_RES(SX126x_GPIO)
    #endif
    #if defined(SX126x_BUSY)
    WX_RES(SX126x_BUSY)
    #endif
    #if defined(RF95_SCK)
    WX_RES(RF95_SCK)
    #endif
    #if defined(RF95_MISO)
    WX_RES(RF95_MISO)
    #endif
    #if defined(RF95_MOSI)
    WX_RES(RF95_MOSI)
    #endif
    #if defined(RF95_NSS)
    WX_RES(RF95_NSS)
    #endif
    #if defined(RADIO_SCLK_PIN)
    WX_RES(RADIO_SCLK_PIN)
    #endif
    #if defined(RADIO_MISO_PIN)
    WX_RES(RADIO_MISO_PIN)
    #endif
    #if defined(RADIO_MOSI_PIN)
    WX_RES(RADIO_MOSI_PIN)
    #endif
    #if defined(RADIO_CS_PIN)
    WX_RES(RADIO_CS_PIN)
    #endif
    #if defined(RADIO_RST_PIN)
    WX_RES(RADIO_RST_PIN)
    #endif
    #if defined(RADIO_IRQ_PIN)
    WX_RES(RADIO_IRQ_PIN)
    #endif
    #if defined(RADIO_BUSY_PIN)
    WX_RES(RADIO_BUSY_PIN)
    #endif
    #if defined(RADIO_DIO1_PIN)
    WX_RES(RADIO_DIO1_PIN)
    #endif
    #if defined(RADIO_SCK)
    WX_RES(RADIO_SCK)
    #endif
    #if defined(RADIO_MISO)
    WX_RES(RADIO_MISO)
    #endif
    #if defined(RADIO_MOSI)
    WX_RES(RADIO_MOSI)
    #endif
    #if defined(RADIO_CTRL)
    WX_RES(RADIO_CTRL)
    #endif
    #if defined(RADIO_LDO_EN)
    WX_RES(RADIO_LDO_EN)
    #endif
    #if defined(PIN_LORA_NSS)
    WX_RES(PIN_LORA_NSS)
    #endif
    #if defined(PIN_LORA_DIO_1)
    WX_RES(PIN_LORA_DIO_1)
    #endif
    #if defined(PIN_LORA_NRST)
    WX_RES(PIN_LORA_NRST)
    #endif
    #if defined(PIN_LORA_BUSY)
    WX_RES(PIN_LORA_BUSY)
    #endif
    #if defined(PIN_LORA_SCK)
    WX_RES(PIN_LORA_SCK)
    #endif
    #if defined(PIN_LORA_MISO)
    WX_RES(PIN_LORA_MISO)
    #endif
    #if defined(PIN_LORA_MOSI)
    WX_RES(PIN_LORA_MOSI)
    #endif
    #if defined(E22_NSS)
    WX_RES(E22_NSS)
    #endif
    #if defined(E22_BUSY)
    WX_RES(E22_BUSY)
    #endif
    #if defined(E22_DIO1)
    WX_RES(E22_DIO1)
    #endif
    #if defined(E22_NRST)
    WX_RES(E22_NRST)
    #endif
    #if defined(E22_RXEN)
    WX_RES(E22_RXEN)
    #endif
    #if defined(E22_TXEN)
    WX_RES(E22_TXEN)
    #endif
    #if defined(E22_SCK)
    WX_RES(E22_SCK)
    #endif
    #if defined(ETH_CS_PIN)
    WX_RES(ETH_CS_PIN)
    #endif
    #if defined(ETH_INT_PIN)
    WX_RES(ETH_INT_PIN)
    #endif
    #if defined(ETH_RST_PIN)
    WX_RES(ETH_RST_PIN)
    #endif
    #if defined(ETH_SCLK_PIN)
    WX_RES(ETH_SCLK_PIN)
    #endif
    #if defined(ETH_MISO_PIN)
    WX_RES(ETH_MISO_PIN)
    #endif
    #if defined(ETH_MOSI_PIN)
    WX_RES(ETH_MOSI_PIN)
    #endif
    #if defined(SDCARD_CS)
    WX_RES(SDCARD_CS)
    #endif
    #if defined(SDCARD_SCLK)
    WX_RES(SDCARD_SCLK)
    #endif
    #if defined(SDCARD_MISO)
    WX_RES(SDCARD_MISO)
    #endif
    #if defined(SDCARD_MOSI)
    WX_RES(SDCARD_MOSI)
    #endif
    #if defined(TDECK_SDCARD_CS)
    WX_RES(TDECK_SDCARD_CS)
    #endif
    #if defined(TDECK_SPI_SCK)
    WX_RES(TDECK_SPI_SCK)
    #endif
    #if defined(TDECK_SPI_MISO)
    WX_RES(TDECK_SPI_MISO)
    #endif
    #if defined(TDECK_SPI_MOSI)
    WX_RES(TDECK_SPI_MOSI)
    #endif
    // GPIO-04: T-Deck Pro radio power enable (variants/t_deck_pro/configuration.h, set HIGH in tdeck_pro.cpp)
    #if defined(BOARD_T_DECK_PRO) && defined(BOARD_LORA_EN)
    WX_RES(BOARD_LORA_EN)
    #endif
    // GPIO-06: the other T-Deck Pro enables (variants/t_deck_pro/configuration.h) and the keyboard interrupt
    #if defined(BOARD_T_DECK_PRO) && defined(TDECK_KEYBOARD_INT)
    WX_RES(TDECK_KEYBOARD_INT)   // 15: keyboard IRQ
    #endif
    #if defined(BOARD_T_DECK_PRO) && defined(BOARD_GPS_EN)
    WX_RES(BOARD_GPS_EN)         // 39: GPS module enable
    #endif
    #if defined(BOARD_T_DECK_PRO) && defined(BOARD_1V8_EN)
    WX_RES(BOARD_1V8_EN)         // 38: 1.8 V rail enable (gyroscope module)
    #endif
    #if defined(BOARD_T_DECK_PRO) && defined(BOARD_6609_EN)
    WX_RES(BOARD_6609_EN)        // 41: modem (7682/7672) module enable
    #endif
    // T5 e-paper: the SX1262 DIO1 (IRQ). It used to be reserved only through the LORA_DIO1 alias that esp32_main.cpp
    // builds from BOARD_LORA_IRQ; the alias is gone from this TU, so it is listed here.
    #if defined(BOARD_LORA_IRQ)
    WX_RES(BOARD_LORA_IRQ)
    #endif
    // GPIO-03: the rest of the T5 e-paper SX1262 (src/t5-epaper/utilities.h:34-40; SPI shared with the SD card).
    #if defined(BOARD_T5_EPAPER)
    #if defined(BOARD_LORA_CS)
    WX_RES(BOARD_LORA_CS)
    #endif
    #if defined(BOARD_LORA_RST)
    WX_RES(BOARD_LORA_RST)
    #endif
    #if defined(BOARD_LORA_BUSY)
    WX_RES(BOARD_LORA_BUSY)
    #endif
    #if defined(BOARD_LORA_SCLK)
    WX_RES(BOARD_LORA_SCLK)
    #endif
    #if defined(BOARD_LORA_MOSI)
    WX_RES(BOARD_LORA_MOSI)
    #endif
    #if defined(BOARD_LORA_MISO)
    WX_RES(BOARD_LORA_MISO)
    #endif
    #endif // BOARD_T5_EPAPER
    #if defined(BOARD_RAK4630) && !defined(BOARD_T_ECHO) && !defined(BOARD_HELTEC_T114)
    // GPIO-03: RAK4631 SX1262, hard-wired in lora_rak4630_meshcom_init() (src/nrf52/api_functions.cpp, copy of the SX126x-Arduino one), no variant macros:
    // RXEN (antenna-switch power) 37, RESET 38, NSS 42, SCLK 43, MOSI 44, MISO 45, BUSY 46, DIO1 47. Pin 17 (WB_IO1) stays free.
    , 37, 38, 42, 43, 44, 45, 46, 47
    #endif
    #if defined(BOARD_HELTEC_T114)
    // T114 SX1262, hard-wired in lora_t114_init() (src/nrf52/api_functions.cpp, USE_HELTEC_T114), no variant macros:
    // BUSY 17, SCLK 19, DIO1 20, MOSI 22, MISO 23, NSS 24, RESET 25
    , 17, 19, 20, 22, 23, 24, 25
    #endif
};

// SOFT: a convention while its function is off (I2C, GPS, button, display reset, Vext, ADC control, battery). Refused when a user SETS a pin, never repaired at load (a Heltec V2 node with `onewire gpio 13` keeps its sensor).
constexpr int kSoft[] = {

        -1
    #if defined(I2C_SDA)
    WX_RES(I2C_SDA)
    #endif
    #if defined(I2C_SCL)
    WX_RES(I2C_SCL)
    #endif
    #if defined(I2C1_SDA)
    WX_RES(I2C1_SDA)
    #endif
    #if defined(I2C1_SCL)
    WX_RES(I2C1_SCL)
    #endif
    #if defined(SDA_PIN)
    WX_RES(SDA_PIN)
    #endif
    #if defined(SCL_PIN)
    WX_RES(SCL_PIN)
    #endif
    #if defined(GPS_RX_PIN)
    WX_RES(GPS_RX_PIN)
    #endif
    #if defined(GPS_TX_PIN)
    WX_RES(GPS_TX_PIN)
    #endif
    #if defined(GPS_EN_PIN)
    WX_RES(GPS_EN_PIN)
    #endif
    #if defined(GPS_PPS_PIN)
    WX_RES(GPS_PPS_PIN)
    #endif
    #if defined(HV4_GPS_EN_PIN)
    WX_RES(HV4_GPS_EN_PIN)
    #endif
    #if defined(HV4_GPS_WAKE_PIN)
    WX_RES(HV4_GPS_WAKE_PIN)
    #endif
    #if defined(HV4_GPS_RST_PIN)
    WX_RES(HV4_GPS_RST_PIN)
    #endif
    #if defined(HV4_GPS_PPS_PIN)
    WX_RES(HV4_GPS_PPS_PIN)
    #endif
    #if defined(BUTTON_PIN)
    WX_RES(BUTTON_PIN)
    #endif
    // docs/gpio-pin-map.md section 3: display reset, Vext / power enable, ADC control, battery pin. Only macros that are
    // a pin number in every env that defines them: RESET_OLED / VEXT_ENABLE are aliases to Arduino constants (RST_OLED,
    // Vext) that not every framework variant declares, and a LEVEL (1) on t114/t_echo; RST_OLED / OLED_RST are
    // `static const`, never macros.
    #if defined(PIN_DISPLAY_RST)
    WX_RES(PIN_DISPLAY_RST)
    #endif
    // VEXT_CTRL / ADC_CTRL: a pin number on the ESP32-S3 Heltec Wireless Tracker; the same names are unused leftovers
    // in the nRF52 variants (heltec_t114, t_echo), so only the ESP32 build counts them
    #if defined(ESP32) && defined(VEXT_CTRL)
    WX_RES(VEXT_CTRL)
    #endif
    #if defined(ESP32) && defined(ADC_CTRL)
    WX_RES(ADC_CTRL)
    #endif
    #if defined(PIN_PCB_VEXT)
    WX_RES(PIN_PCB_VEXT)
    #endif
    #if defined(ADC_CTRL_PIN)
    WX_RES(ADC_CTRL_PIN)
    #endif
    #if defined(BATTERY_PIN)
    WX_RES(BATTERY_PIN)
    #endif
    // Arduino variant SPI constants (not macros, always present): the radio bus on Heltec V2 and the E22 DevKits,
    // ordinary pins on T3-S3 or T-ETH-Elite whose radio uses other pins, so SOFT: refused for new settings,
    // never reset at load (GPIO-01 W1 advisor)
        WX_RES(SS)
        WX_RES(SCK)
        WX_RES(MISO)
        WX_RES(MOSI)
};

constexpr uint64_t kHardMask = wxMask(kHard, sizeof(kHard) / sizeof(kHard[0]), 0);
constexpr uint64_t kSoftMask = wxMask(kSoft, sizeof(kSoft) / sizeof(kSoft[0]), 0);
} // namespace

#undef WX_RES

// Regression check (compile time): a RAK4631 must not inherit the T-Echo radio / I2C pins. Pin 17 is WB_IO1, the board's
// 1-Wire default (variants/wiscore_rak4631/variant.h), and a WisBlock IO pin; it is T-Echo's radio BUSY pin.
// BOARD_RAK4630 is defined on every nRF52 env (nrf52_base), so T-Echo and T114 are left out.
#if defined(BOARD_RAK4630) && !defined(BOARD_T_ECHO) && !defined(BOARD_HELTEC_T114)
static_assert((kHardMask & (1ULL << 17)) == 0, "RAK4631: pin 17 (WB_IO1) is reserved HARD: T-Echo pins leaked into the list");
static_assert((kSoftMask & (1ULL << 17)) == 0, "RAK4631: pin 17 (WB_IO1) is reserved SOFT: T-Echo pins leaked into the list");
#endif
// Regression check: the T114 radio has no variant macros (pins hard-wired in lora_t114_init(), src/nrf52/api_functions.cpp),
// so its block in kHard is the only thing that keeps 1-Wire / analog / the Pins page off the SX1262 pins.
#if defined(BOARD_HELTEC_T114)
static_assert((kHardMask & ((1ULL << 17) | (1ULL << 19) | (1ULL << 20) | (1ULL << 22) | (1ULL << 23) | (1ULL << 24) | (1ULL << 25))) ==
                  ((1ULL << 17) | (1ULL << 19) | (1ULL << 20) | (1ULL << 22) | (1ULL << 23) | (1ULL << 24) | (1ULL << 25)),
              "T114: SX1262 pins (17, 19, 20, 22-25) are not reserved HARD");
#endif
// GPIO-03: the T5 e-paper SX1262 (BOARD_LORA_*, src/t5-epaper/utilities.h:34-40) must be HARD in full, not only its DIO1.
#if defined(BOARD_T5_EPAPER)
static_assert((kHardMask & ((1ULL << 46) | (1ULL << 1) | (1ULL << 47) | (1ULL << 14) | (1ULL << 13) | (1ULL << 21))) ==
                  ((1ULL << 46) | (1ULL << 1) | (1ULL << 47) | (1ULL << 14) | (1ULL << 13) | (1ULL << 21)),
              "T5 e-paper: SX1262 CS 46, RST 1, BUSY 47, SCLK 14, MOSI 13, MISO 21 are not reserved HARD");
#endif
// GPIO-03: the RAK4631 radio is hard-wired in lora_rak4630_meshcom_init() (src/nrf52/api_functions.cpp), no variant macros:
// RESET 38, NSS 42, SCLK 43, MOSI 44, MISO 45, BUSY 46, DIO1 47, antenna-switch power (RXEN) 37. Pin 17 stays free.
#if defined(BOARD_RAK4630) && !defined(BOARD_T_ECHO) && !defined(BOARD_HELTEC_T114)
static_assert((kHardMask & ((1ULL << 37) | (1ULL << 38) | (1ULL << 42) | (1ULL << 43) | (1ULL << 44) | (1ULL << 45) | (1ULL << 46) | (1ULL << 47))) ==
                  ((1ULL << 37) | (1ULL << 38) | (1ULL << 42) | (1ULL << 43) | (1ULL << 44) | (1ULL << 45) | (1ULL << 46) | (1ULL << 47)),
              "RAK4631: SX1262 pins (37, 38, 42-47) are not reserved HARD");
#endif
// GPIO-04: the T-Deck Pro powers its radio through BOARD_LORA_EN (46, driven HIGH in tdeck_pro.cpp); driving it from a
// user function would switch the radio off.
#if defined(BOARD_T_DECK_PRO) && defined(BOARD_LORA_EN)
static_assert((kHardMask & (1ULL << BOARD_LORA_EN)) != 0, "T-Deck Pro: BOARD_LORA_EN (radio power enable) is not reserved HARD");
#endif
// GPIO-08: BOARD_GPS_EN (39) powers the GNSS module; it was once also the SD card CS in the variant header (now 48,
// TDECK_SDCARD_CS), so it must stay reserved HARD against user functions.
#if defined(BOARD_T_DECK_PRO) && defined(BOARD_GPS_EN)
static_assert(BOARD_GPS_EN == 39 && (kHardMask & (1ULL << BOARD_GPS_EN)) != 0, "T-Deck Pro: BOARD_GPS_EN (GNSS power enable, 39) is not reserved HARD");
#endif
// GPIO-09: T-Deck copies (power-on 10, TFT CS/DC/backlight, ES7210 audio) are wrong pins on the Pro and must stay out of
// its variant header.
#if defined(BOARD_T_DECK_PRO) && (defined(TDECK_POWERON) || defined(TDECK_TFT_CS) || defined(TDECK_TFT_DC) || defined(TDECK_TFT_BACKLIGHT) || \
                                  defined(TDECK_ES7210_MCLK) || defined(TDECK_ES7210_LRCK) || defined(TDECK_ES7210_SCK) || defined(TDECK_ES7210_DIN))
#error "GPIO-09: T-Deck copies in variants/t_deck_pro/configuration.h (TDECK_POWERON / TDECK_TFT_* / TDECK_ES7210_*)"
#endif
// GPIO-06: keyboard IRQ 15 (was a copy of the T-Deck 46, which is the LoRa EN pin here), GPS EN 39, 1V8 EN 38, modem EN 41.
#if defined(BOARD_T_DECK_PRO) && defined(TDECK_KEYBOARD_INT) && defined(BOARD_GPS_EN) && defined(BOARD_1V8_EN) && defined(BOARD_6609_EN)
static_assert(TDECK_KEYBOARD_INT == 15 && BOARD_1V8_EN == 38 && BOARD_GPS_EN == 39 && BOARD_6609_EN == 41 && BOARD_LORA_EN == 46,
              "T-Deck Pro: keyboard IRQ must be 15, enables 38 / 39 / 41, LoRa EN 46");
static_assert((kHardMask & ((1ULL << TDECK_KEYBOARD_INT) | (1ULL << BOARD_GPS_EN) | (1ULL << BOARD_1V8_EN) | (1ULL << BOARD_6609_EN))) ==
                  ((1ULL << TDECK_KEYBOARD_INT) | (1ULL << BOARD_GPS_EN) | (1ULL << BOARD_1V8_EN) | (1ULL << BOARD_6609_EN)),
              "T-Deck Pro: keyboard IRQ 15 and enables 38 / 39 / 41 are not reserved HARD");
#endif
// GPSV4-01: Heltec V4 GNSS (Meshtastic variants/esp32s3/heltec_v4/variant.h): GNSS TX -> ESP RX is GPIO39, ESP TX -> GNSS RX
// is GPIO38, enable GPIO34 (active LOW), reset GPIO42, PPS GPIO41. I2C therefore must not sit on 41/42 (it moved to 4/3).
#if defined(BOARD_HELTEC_V4)
static_assert(GPS_RX_PIN == 39 && GPS_TX_PIN == 38, "Heltec V4: GNSS RX must be 39 (GNSS TX) and TX 38 (GNSS RX), not swapped");
static_assert(HV4_GPS_EN_PIN == 34 && HV4_GPS_RST_PIN == 42, "Heltec V4: GNSS enable must be 34 and reset 42");
static_assert(I2C_SDA != 41 && I2C_SDA != 42 && I2C_SCL != 41 && I2C_SCL != 42, "Heltec V4: I2C must not use 41/42 (GNSS PPS / reset)");
#endif

uint64_t pinBoardReservedHard() { return kHardMask; }

uint64_t pinBoardReservedSoft() { return kSoftMask; }

PinChip pinBoardChip()
{
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    return PIN_CHIP_ESP32S3;
#elif defined(ESP32)
    return PIN_CHIP_ESP32;
#elif defined(NRF52_SERIES)
    return PIN_CHIP_NRF52840;
#else
    return PIN_CHIP_NONE;
#endif
}

// octal PSRAM/flash (qio_opi, opi_opi in the board JSON -> sdkconfig.h) takes GPIO 33-37 on the S3
bool pinBoardOctal()
{
#if defined(CONFIG_SPIRAM_MODE_OCT) || defined(CONFIG_ESPTOOLPY_OCT_FLASH)
    return true;
#else
    return false;
#endif
}

// Board default pin of a function; -1 = none (0 is a real GPIO: BUTTON_PIN 0 on 13 variants). Wind has no default.
// 1-Wire / analog: a macro of 0 or below (OneWire_GPIO -1) means no default.
int pinBoardDefault(PinFunc f)
{
    int d = -1;
    switch (f)
    {
    case PIN_FN_ONEWIRE:
#if defined(OneWire_GPIO)
        d = (int)(OneWire_GPIO + 0);
#endif
        break;
    case PIN_FN_BUTTON:
#if defined(BUTTON_PIN)
        d = (int)(BUTTON_PIN + 0);
#endif
        break;
    case PIN_FN_ANALOG:
#if defined(ANALOG_PIN)
        d = (int)(ANALOG_PIN + 0);
#endif
        break;
    default: break;
    }
    if (f != PIN_FN_BUTTON && d <= 0) return -1;
    return d >= 0 ? d : -1;
}

// Effective pin of a function (a setting of 0 / unset means the board default), -1 = none.
int pinBoardEffective(PinFunc f)
{
    int v = 0;
    switch (f)
    {
    case PIN_FN_ONEWIRE: v = meshcom_settings.node_owgpio; break;
    case PIN_FN_BUTTON: v = meshcom_settings.node_button_pin; break;
    case PIN_FN_ANALOG: v = meshcom_settings.node_analog_pin; break;
#if defined(ESP32)
    case PIN_FN_WIND: v = meshcom_settings.node_wind_pin; break;
#endif
    default: break;
    }
    if (f == PIN_FN_ANALOG && v >= 99) v = 0;   // 99 = unset
    if (f == PIN_FN_BUTTON && v >= 99) return -1; // 99 = no button (not the board default)
#if defined(BOARD_TBEAM)
    if (f == PIN_FN_ONEWIRE && v == 16) v = 0;  // onewire_functions.cpp: stored 16 -> 0 -> OneWire_GPIO
#endif
    if (f == PIN_FN_WIND) return v > 0 ? v : -1; // wind: 0 = off, no default
    if (v <= 0) v = pinBoardDefault(f);
    return v;
}

// The pin a function uses right now. Same as pinBoardEffective(), except the button: its pin is read once at boot
// (esp32_main.cpp / nrf52_main.cpp -> iButtonPin), so until a restart a newly stored value is not in use yet.
int pinBoardInUse(PinFunc f)
{
    if (f == PIN_FN_BUTTON) return iButtonPin >= 99 ? -1 : (int)iButtonPin; // 0 is GPIO 0
    return pinBoardEffective(f);
}

// Pins of the OTHER enabled functions, plus the board LED pad.
uint64_t pinBoardClaimed(PinFunc except)
{
    uint64_t m = 0;
    if (except != PIN_FN_ONEWIRE && bONEWIRE) m |= pinBit(pinBoardEffective(PIN_FN_ONEWIRE));
    if (except != PIN_FN_BUTTON && bButtonCheck)
    {
        // the stored pin AND the one OneButton reads right now (they differ after --button gpio until a restart);
        // pinBit() is 0 for a negative or out-of-range pin; -1 = none, 0 is GPIO 0
        m |= pinBit(pinBoardEffective(PIN_FN_BUTTON));
        const int inUse = pinBoardInUse(PIN_FN_BUTTON);
        if (inUse >= 0) m |= pinBit(inUse);
    }
    if (except != PIN_FN_ANALOG && bAnalogCheck) m |= pinBit(pinBoardEffective(PIN_FN_ANALOG));
#if defined(ESP32)
    if (except != PIN_FN_WIND && bWINDON) m |= pinBit(pinBoardEffective(PIN_FN_WIND));
#endif
#if defined(BOARD_LED)
    m |= pinBit((int)(BOARD_LED + 0));
#endif
    return m;
}

// Pins the console setter of a function refuses on top of the policy (command_functions.cpp), so the web mask can match.
uint64_t pinBoardSetterRefused(PinFunc f)
{
    switch (f)
    {
    case PIN_FN_ANALOG: return 1ULL;   // --analog gpio takes 1..98 only: 0 is refused
#if defined(BOARD_RAK4630)
    case PIN_FN_ONEWIRE: return 7ULL;  // --onewire gpio refuses 0-2 on BOARD_RAK4630 (set on every nRF52 env)
#endif
    default: return 0;
    }
}

PinContext pinBoardContext(PinFunc except)
{
    PinContext c;
    c.chip = pinBoardChip();
    c.octal = pinBoardOctal();
    c.reserved = pinBoardReservedHard();
    c.soft = pinBoardReservedSoft();
    c.claimed = pinBoardClaimed(except);
    c.table = pinBoardFor(MC_ENV_NAME);
    return c;
}

// A stored pin that the loader may keep: not forbidden and, for the wind input, not input-only (no pull-up).
// Strapping / claimed / unreviewed pins stay usable so no working setup breaks (GPIO-01).
bool pinBoardLoadedUsable(PinFunc f, int pin)
{
    if (pin == 0) return false;
    const PinClass k = pinClassify(pinBoardContext(f), pin);
    return k != PIN_FORBIDDEN && (k != PIN_INPUT_ONLY || f != PIN_FN_WIND);
}
