#include "loop_functions.h"
#include "loop_functions_extern.h"
#include "gateway_service.h"
#include "udp_functions.h"
#include "extudp_functions.h"
#include "printfdeb_functions.h"
#include "instrument.h"
#include <WiFi.h>

// C4 carve-out of the gateway service block out of esp32_main.cpp; see
// gateway_service.h for why it is carved but not unified. Moved unchanged.

extern bool hb_warn_logged;
extern unsigned long last_upd_timer;

void gatewayService_esp32(void)
{
////////////////////////////////////////////////
// WIFI Gateway functions
if(bGATEWAY && meshcom_settings.node_hasIPaddress)
{
    { INSTR_SECTION("udp"); getMeshComUDP(); sendMeshComUDP(); }

    // heartbeat
    if ((uint32_t)(millis() - hb_timer) >= (HEARTBEAT_INTERVAL * 1000))
    {
        sendMeshComHeartbeat();
        hb_timer = millis();

        if (last_upd_timer > 0)
        {
            unsigned long hb_age = millis() - last_upd_timer;


            // Stage 1: diagnostic warning at 35s
            if (hb_age > (HB_WARN_TIME * 1000) && !hb_warn_logged)
            {
                bool wifi_ok = (WiFi.status() == WL_CONNECTED);
                printfdeb("[UDP] Server not responding for %lus — WiFi %s\n",
                              hb_age / 1000, wifi_ok ? "CONNECTED" : "NOT_CONNECTED");
                hb_warn_logged = true;

                // WiFi actually down → reset immediately, don't wait
                if (!wifi_ok)
                {
                    printfdeb("[UDP] WiFi down — resetting");
                    resetMeshComUDP();
                    last_upd_timer = millis();
                    hb_warn_logged = false;
                }
            }

            // Stage 2: timeout at 65s
            if (hb_age > (MAX_HB_RX_TIME * 1000))
            {
                bool wifi_ok = (WiFi.status() == WL_CONNECTED);

                if (!wifi_ok)
                {
                    printfdeb("[UDP] Heartbeat timeout %lus — WiFi NOT_CONNECTED, resetting\n",
                                  hb_age / 1000);
                    resetMeshComUDP();
                }
                else
                {
                    printfdeb("[UDP] Heartbeat timeout %lus — WiFi CONNECTED, server unresponsive, waiting\n",
                                  hb_age / 1000);
                }

                last_upd_timer = millis();
                hb_warn_logged = false;
            }
        }
    }

    meshcom_settings.node_last_upd_timer = hb_timer;

}
else if(meshcom_settings.node_hasIPaddress)
{
    // TM-45: bGATEWAY is off, so the block above never runs and never
    // reads the socket -- do only the NTP-reply harvest instead, not
    // the full gateway receive path (no double read: exactly one of
    // the two branches runs per loop pass).
    INSTR_SECTION("udp"); ntpHarvestUDP();
}
}
