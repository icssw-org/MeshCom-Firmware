// GPIO-01: firmware side of the pin policy (src/pin_policy.h): declarations only. The bodies live in src/pin_board.cpp,
// the one translation unit that reads the board's pin macros (LORA_*, RADIO_*, I2C_*, BUTTON_PIN, ...), so every caller
// gets the same answer whatever it included before. Include this header anywhere in a firmware TU, in any order; never
// from host tests (the .cpp is not part of the native envs).
#pragma once
#include <stdint.h>
#include "pin_policy.h"

// HARD reserved pins: radio, its SPI bus, Ethernet, SD. Refused when set AND repaired at load.
uint64_t pinBoardReservedHard();
// SOFT reserved pins: I2C, GPS, button, display reset, Vext, ADC control, battery. Refused when set, never repaired at load.
uint64_t pinBoardReservedSoft();

PinChip pinBoardChip();
// octal PSRAM/flash takes GPIO 33-37 on the S3
bool pinBoardOctal();
// Board default pin of a function, -1 = none (0 is a real GPIO) (the anemometer has none).
int pinBoardDefault(PinFunc f);
// Effective pin from the stored setting (0 / unset = board default; analog and button 99 = unset / no button -> -1 = none).
int pinBoardEffective(PinFunc f);
// The pin a function uses right now: the button pin is read once at boot (iButtonPin), the others follow the setting.
int pinBoardInUse(PinFunc f);
// Pins of the OTHER enabled functions, plus the board LED pad.
uint64_t pinBoardClaimed(PinFunc except);
// Pins the console setter of this function refuses on top of the policy (analog 0; 1-Wire 0-2 on BOARD_RAK4630).
uint64_t pinBoardSetterRefused(PinFunc f);
PinContext pinBoardContext(PinFunc except);
// A stored pin the loader may keep (not forbidden, and for the wind input not input-only).
bool pinBoardLoadedUsable(PinFunc f, int pin);
