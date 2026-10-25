#ifndef _PMU_H_
#define _PMU_H_

#include <Arduino.h>
#include "charge_led_mode.h"

void setupPMU();

// Disable the LoRa and GPS power rails ahead of --deepsleep. No-op on boards
// without an AXP192/AXP2101 PMU (PMU stays NULL) and a no-op if setupPMU()
// never found a chip. Leaves the ESP32-feeding rail, the OLED rail and every
// other channel (SD, sensors, m.2) alone -- only the two channels setupPMU()
// documents as LoRa/GPS are touched here.
void pmuSleepRails();

// LED-02/LED-01: the RM "chgled" override lives here (one ChargeLedOverride instance, esp32_pmu.cpp). Declared on
// every board so surfaces need no guard: without an AXP chip the effective mode is the stored one and the
// override calls do nothing.
ChargeLedMode chargeLedEffective(void);          // stored console mode, or the RM override while one is active
bool chargeLedOverrideActive(void);              // true between an RM "chgled" and the next console write / deep sleep
void chargeLedOverrideSet(ChargeLedMode mode);   // record the override and drive the PMU
bool chargeLedFitted(void);                      // AXP compiled in and a PMU answered (RM chgled: err unsupported otherwise)
void chargeLedOverrideClear(void);               // drop the override and re-apply the stored mode (no-op when none)

#if defined(XPOWERS_CHIP_AXP192) || defined(XPOWERS_CHIP_AXP2101)

// Function to enable and disable (0) Charging Led Auto Mode (e.g. T-BEAM AXP2101 v1.2)
void setChargeLedOn(bool enableAutoMode);

// Mode form (ChargeLedMode of charge_led_mode.h as int): auto = PMU follows charging, on = steady, off = dark.
bool setChargeLedMode(int mode);   // false = no PMU, nothing changed

#endif // defined(XPOWERS_CHIP_AXP192) || defined(XPOWERS_CHIP_AXP2101)

#endif
