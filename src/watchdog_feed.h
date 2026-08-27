#ifndef _WATCHDOG_FEED_H_
#define _WATCHDOG_FEED_H_

/**
 * @file watchdog_feed.h
 * @brief Ein einziger, greppbarer Einstiegspunkt zum Füttern des Task-Watchdogs.
 *
 * Hintergrund (N-25): der Task-Watchdog wird ausschließlich auf ESP32 scharf
 * geschaltet (esp32_main.cpp, am Ende von esp32setup()). Dateien wie
 * gps_functions.cpp werden aber von beiden MCU-Familien übersetzt. Würde jede
 * Fundstelle ihr eigenes `#if defined(ESP32)` + esp_task_wdt_reset() tragen,
 * driften die Zweige auseinander -- genau so sind die beiden
 * detectBaudrate()-Implementierungen entstanden.
 *
 * Deshalb: EIN Helfer, auf nRF52 ein No-op, und alle Fundstellen sind über
 * `grep -rn meshcom_wdt_feed src/` vollständig auffindbar.
 *
 * Wichtig -- was NICHT füttert: delay(), yield(), vTaskDelay(). Nur
 * esp_task_wdt_reset() setzt den Task-Watchdog zurück.
 *
 * Der Helfer ist KEIN Ersatz dafür, unbegrenzte Schleifen zu begrenzen. In
 * einer Schleife ohne Abbruchbedingung verwandelt Füttern einen Panic-Reboot
 * in ein dauerhaftes stilles Einfrieren -- für einen Mesh-Knoten schlechter.
 * Nur an nachweislich endlich begrenzten Stellen einsetzen.
 */

#if defined(ESP32)

#include "esp_task_wdt.h"

static inline void meshcom_wdt_feed(void)
{
    // Liefert ESP_ERR_NOT_FOUND, solange der Task nicht angemeldet ist
    // (z.B. während esp32setup()). Das ist der harmlose Normalfall.
    esp_task_wdt_reset();
}

#else

static inline void meshcom_wdt_feed(void) {}

#endif

#endif  // _WATCHDOG_FEED_H_
