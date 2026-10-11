#pragma once

// Gate in startNetwork(): ohne SSID wird der WiFi-Start abgebrochen.
//
// Die Pruefung lag vor dem bWIFIAP-Zweig. Ein frischer oder geloeschter
// ESP32-Knoten hat node_ssid = "none" (esp32_flash.cpp); nach `--wifiap on`
// kam der softAP dadurch nie hoch, das Display zeigte "AP : <call>" ohne IP.
// Im AP-Modus ist die SSID irrelevant -- das Gate gilt nur fuer STA.

#include <stdbool.h>
#include <string.h>

/**
 * @brief Blockiert eine fehlende SSID den WiFi-Start?
 *
 * @param apMode  true wenn der softAP gestartet werden soll (bWIFIAP).
 * @param ssid    node_ssid, darf NULL sein.
 * @return true nur im STA-Modus bei NULL, leerer SSID oder exakt "none"
 *         (Gross-/Kleinschreibung wie bei is_equ()).
 */
static inline bool wifiSsidMissingBlocksStart(bool apMode, const char *ssid)
{
    if(apMode)
        return false;

    return ssid == NULL || ssid[0] == 0x00 || strcmp(ssid, "none") == 0;
}
