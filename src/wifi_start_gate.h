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

/**
 * @brief D15: soll WiFi fuer das Firmware-Auto-Update (alleine) gestartet werden?
 *
 * Nur mit gesetzter SSID und nicht im AP-Modus: ein Knoten ohne SSID (oder als softAP)
 * hat keinen STA-Link, auf dem AU laufen koennte; WiFi bliebe nur ohne Nutzen an.
 *
 * @param autoupd  node_autoupd (0 off, 1 notify, 2 auto).
 * @param apMode   bWIFIAP.
 * @param ssid     node_ssid, darf NULL sein.
 */
static inline bool wifiAutoUpdateWantsStart(int autoupd, bool apMode, const char *ssid)
{
    return autoupd > 0 && !apMode && !wifiSsidMissingBlocksStart(apMode, ssid);
}

/**
 * @brief Startet startNetwork() beim Boot? Die vier Dienste (Gateway, Ext-UDP,
 *        Web, Net-Konsole) wie bisher, AU nur nach wifiAutoUpdateWantsStart().
 */
static inline bool wifiStartWanted(bool gateway, bool extudp, bool webserver, bool netconsole,
                                   int autoupd, bool apMode, const char *ssid)
{
    return gateway || extudp || webserver || netconsole || wifiAutoUpdateWantsStart(autoupd, apMode, ssid);
}
