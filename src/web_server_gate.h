#pragma once

// Gate in stopWebserver(): nur ein laufender Server wird gestoppt.
//
// esp32loop() ruft startWebserver() in jedem Durchlauf; ohne IP (WLAN aus)
// rief das jedes Mal stopWebserver() auf -- MDNS.end(), web_server.stop() und
// die Debugzeile "WEBServer stopped" kamen im Loop-Takt (WEB-STOP-01).

#include <stdbool.h>

/**
 * @brief Soll stopWebserver() MDNS/Server anhalten und loggen?
 *
 * @param running  bweb_server_running
 * @return true nur wenn der Server laeuft.
 */
static inline bool webStopShouldAct(bool running)
{
    return running;
}
