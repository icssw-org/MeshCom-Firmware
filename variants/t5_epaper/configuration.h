/**
 * definitions for LilyGo T5 4.7" ePaper S3 Pro Board
 */

#pragma once

#include <Arduino.h>
#include <configuration_global.h>

// T5 ePaper specific config
#define MODUL_HARDWARE T5_EPAPER

// --- No onboard sensors and no GPS on the T5 ePaper -> deliberately NOT enabled ---
#define ENABLE_GPS_DISABLED
#define ENABLE_BMX280_DISABLED
#define ENABLE_BMX680_DISABLED
#define ENABLE_RTC_DISABLED
#define ENABLE_BMP390_DISABLED
#define ENABLE_AHT20_DISABLED
#define ENABLE_MCP23017_DISABLED
#define ENABLE_SHT21_DISABLED
#define ENABLE_MC811_DISABLED
#define ENABLE_INA226_DISABLED

// RF defaults. The board runs its own SX1262 driver (src/t5-epaper/peri_lora.cpp),
// which reads LORA_FREQUENNCY/LORA_BANDWIDTH/... from src/t5-epaper/peripheral.h
// directly. These generic macros are still required by board-independent code
// (country_profile.cpp, esp32_flash.cpp) that every board defines.

// SX1262: RadioLib output power range is -9 .. 22 dBm (comment at
// src/t5-epaper/peri_lora.cpp:105), matching TX_POWER_MIN/MAX on t_deck_pro/t_deck_plus.
// peripheral.h:15 carries "-17 - 22 dBm" in its own comment; that is the SX127x
// range, not this part's, and peri_lora.cpp is the file that actually calls
// setOutputPower().

// Default TX power, consumed by resolve_tx_power() in command_functions.cpp
// when the stored value is 0 or the -20 "unset" sentinel (#1132).

// GPIO 0, the BOOT button -- src/t5-epaper/utilities.h:44 BOARD_BOOT_BTN.
#define BUTTON_PIN 0

/** RadioLib Spreading Factor
 * case 6: SF_6;
    case 7: SF_7;
    case 8: SF_8;
    case 9: SF_9;
    case 10: SF_10;
    case 11: SF_11;
    case 12: SF_12;
*/


/**
 * RadioLib Coding Rate: Allowed values range from 5 to 8.
 * case 5: CR_4_5;
    case 6: CR_4_6;
    case 7: CR_4_7;
    case 8: CR_4_8;
*/

// RadioLib LoRa Bandwidth Setting in kHz. Spelled "250.0" (not "250") to stay
// token-identical with src/t5-epaper/peripheral.h's LORA_BANDWIDTH, which is
// included ahead of this file in the same translation unit in several
// src/t5-epaper/*.cpp files (peripheral.h -> lora_functions.h -> configuration.h);
// a differently-spelled redefinition would warn.
#define LORA_BANDWIDTH 250.0

// W7 (D6-01): die Flottenvorgaben stehen in src/configuration_default.h.
// Der Include gehoert ans ENDE: was diese Datei oben selbst setzt, hat es
// dann schon gesetzt, und die #ifndef-Waechter dort ueberspringen es.
#include <configuration_default.h>   // W7: Flottenvorgaben, #ifndef -- was oben steht, gewinnt
