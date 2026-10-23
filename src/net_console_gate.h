#pragma once

// Gate in loopNetConsole(): ist das Netz bereit fuer den Listening-Socket?
//
// Ohne laufenden lwIP-Stack loest ::socket() einen Assert aus (Panic/Reboot),
// daher wartet die Netconsole auf ein Interface mit IP. Frueher zaehlte nur
// WL_CONNECTED (STA) und, ab a12fbcc6, Ethernet. Ein reiner Access Point
// erfuellt WL_CONNECTED nie -- im AP-Modus startete die Netconsole deshalb
// nie (P-0111). Der softAP hat seinen eigenen IP-Stack und genuegt.

#include <stdbool.h>

/**
 * @brief Darf loopNetConsole() den Listening-Socket oeffnen?
 *
 * @param staConnected  WiFi.status() == WL_CONNECTED
 * @param apActive      softAP laeuft (WiFi.getMode() AP oder APSTA)
 * @param ethReady      Ethernet-Modus mit IP (nur HAS_ETHERNET-Boards)
 * @return true sobald eines der Interfaces bereit ist.
 */
static inline bool netConsoleNetworkReady(bool staConnected, bool apActive, bool ethReady)
{
    return staConnected || apActive || ethReady;
}
