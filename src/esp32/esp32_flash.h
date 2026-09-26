#ifndef _ESP32_FLASH_H_
#define _ESP32_FLASH_H_

#include <Arduino.h>

// The settings struct lives in src/meshcom_settings.h, one definition for both platforms (D1-04).
#include <meshcom_settings.h>

void save_settings(void);
// Get LoRa parameter
void init_flash(void);
// LoRa parameter zurück setzen
void clear_flash(void);

#endif
