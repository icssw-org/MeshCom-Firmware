#ifndef _PMU_H_
#define _PMU_H_

#include <Arduino.h>

void setupPMU();

// Disable the LoRa and GPS power rails ahead of --deepsleep. No-op on boards
// without an AXP192/AXP2101 PMU (PMU stays NULL) and a no-op if setupPMU()
// never found a chip. Leaves the ESP32-feeding rail, the OLED rail and every
// other channel (SD, sensors, m.2) alone -- only the two channels setupPMU()
// documents as LoRa/GPS are touched here.
void pmuSleepRails();

// Function to enable and disable (0) the Charging Led (e.g. T-BEAM AXP2101 v1.2)
int setChargeLedOn(int on);

#endif
