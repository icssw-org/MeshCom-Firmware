#include <SPI.h>
#include <RAK13800_W5100S.h>   // IPAddress for nrf_eth.h, same order as nrf_eth.cpp
#include <Arduino.h>
#include "loop_functions.h"
#include "loop_functions_extern.h"
#include "gateway_service.h"
#include <nrf_eth.h>
#include "printfdeb_functions.h"
#include "instrument.h"

// C4 carve-out of the gateway service block out of nrf52_main.cpp; see
// gateway_service.h for why it is carved but not unified. Moved unchanged.

extern NrfETH neth;
extern bool hb_warn_logged;
extern unsigned long iReceiveTimeOutTime;
void sendUDP(void);
void startRadioReceive(void);

void gatewayService_nrf52(void)
{
// get UDP & send UDP message from ringBufferOut if there is one to tx
if(bGATEWAY)
{
    INSTR_SECTION("gateway");
    int bUDPReceived = false;

    // check if we received a UDP packet
    if (neth.hasIPaddress)
    {
        bSPI_ETH_Active = true;   // SPI guard: Ethernet owns bus
        INSTR_SECTION("eth_udp");
        // DR-14, DECIDED 2026-09-12, both-valid -- do not "fix" this to
        // match ESP32. ESP32 drains the TX ring unconditionally, every
        // bGATEWAY pass (gateway_service_esp32.cpp: getMeshComUDP() then
        // sendMeshComUDP(), no condition between them). nRF52 instead calls
        // sendUDP() ONLY on a pass where getUDP() just reported "no packet
        // received", because on RAK4631 the W5100S (Ethernet) and the SX1262
        // (LoRa radio) share one SPI bus: bSPI_ETH_Active is held across the
        // whole Ethernet access above and a radio re-arm is deferred
        // (bPendingRadioRx) until it releases below. Draining TX on every
        // pass, including RX-received passes, would lengthen that shared-bus
        // window and risk the radio missing LoRa frames. Both platforms are
        // therefore correct for their own hardware; the asymmetry is
        // intentional and the residual risk (TX ring only drains on
        // RX-free passes here) was accepted knowingly at that review.
        // Two independent readers have already misread this as a bug and
        // proposed unifying it -- the original DRY audit (OPT-D5) and the
        // M2 drift-matrix row -- which is why this is pinned by a test, not
        // just this comment: test/test_gateway_service_twin (env
        // native_gateway_twin) case
        // test_dr14_nrf52_sends_only_when_no_packet_received asserts
        // sendUDP() fires only on the no-packet pass, and will fail if this
        // selectivity is ever "unified" away. See docs/BACKLOG.md DR-14.
        if(neth.getUDP() == 1)  // 1...no udp-paket received
        {
            { INSTR_SECTION("eth_udp_tx"); sendUDP(); }
        }
        else
        {
            bUDPReceived=true;

            if(bDEBUG)
                Serial.println("LOOP GATEWAY actions UDP received");
        }
        bSPI_ETH_Active = false;  // SPI guard: release bus
        if(bPendingRadioRx) { bPendingRadioRx = false; startRadioReceive(); }
    }
    else
    {
        //neth.last_upd_timer = 0; // ETH new
    }

    // UDP Action for next loop
    if(!bUDPReceived)
    {
        meshcom_settings.node_hasIPaddress = neth.hasIPaddress;
        meshcom_settings.node_last_upd_timer = neth.last_upd_timer;
        
        // DR-03, DECIDED 2026-09-12 esp32-correct: bis heute war nRF52 in
        // der ersten Stufe STUMM. ESP32 warnt nach HB_WARN_TIME (35 s) ohne
        // serverseitige Antwort und handelt erst nach MAX_HB_RX_TIME (65 s)
        // (gateway_service_esp32.cpp:36-62); nRF52 hatte nur die zweite
        // Stufe, also 65 s ohne jede Ausgabe und danach direkt eine
        // Neuinitialisierung. Die 30 s dazwischen sind genau das Fenster, in
        // dem man am Log erkennen koennte, OB der Server oder der Link das
        // Problem ist -- deshalb benennt die Zeile hier den Ethernet-Zustand,
        // wo ESP32 den WiFi-Zustand benennt.
        //
        // Bewusst NUR die Diagnose portiert, nicht die ESP32-Sofortaktion:
        // dort loest ein toter WiFi-Link in Stufe 1 gleich ein
        // resetMeshComUDP() aus. Der nRF52-Gegenpart waere initethDHCP()/
        // resetDHCP() -- und genau dieser Pfad ist N-20: ein einmal
        // gezogenes Kabel verband nie wieder, weil jeder Retry den W5100S
        // hardware-resettet (Begruendung im resetDHCP()-Zweig unten). Eine
        // zweite, fruehere Ausloesestelle dafuer waere ein Rueckschritt.
        // Stufe 2 bleibt daher unveraendert die einzige handelnde Stufe.
        //
        // neth.last_upd_timer > 0 wie auf ESP32 (dort last_upd_timer > 0).
        // ACHTUNG, die Bedeutung ist auf nRF52 eine ANDERE als auf ESP32, und
        // die erste Fassung dieses Kommentars hatte sie falsch: auf nRF52
        // setzt nicht nur der Serververkehr diesen Zeitstempel, sondern auch
        // der ETH-Aufbau selbst (nrf_eth.cpp:246 initethfixIP, :857 startUDP).
        // Der Zaehler laeuft hier also ab LINK-UP, nicht ab der ersten
        // Serverantwort -- ein Knoten, dem nie jemand antwortet, warnt 35 s
        // nach dem ETH-Start. Das ist richtig so und passt zur handelnden
        // Stufe, die 65 s nach demselben Bezugspunkt greift. Die Abfrage
        // schuetzt damit genau einen Zustand: die Sekunden vor ETH-Up, in
        // denen millis() - 0 sofort jede Schwelle reissen wuerde.
        if(neth.last_upd_timer > 0)
        {
            uint32_t hb_age = (uint32_t)(millis() - neth.last_upd_timer);

            if(hb_age > (uint32_t)(HB_WARN_TIME * 1000) && !hb_warn_logged)
            {
                printfdeb("[UDP] Server not responding for %lus - ETH %s\n",
                          (unsigned long)(hb_age / 1000),
                          neth.hasIPaddress ? "UP" : "DOWN");
                hb_warn_logged = true;
            }
        }

        // check HB response (we also check successful sending KEEP. check if they work together!)
        if((uint32_t)(millis() - neth.last_upd_timer) >= (uint32_t)(MAX_HB_RX_TIME * 1000))
        {
            // DR-03: Stufe 2 setzt die Alterung zurueck, also faellt hier
            // auch der Latch -- sonst bliebe er nach dem ersten Mal
            // gesetzt und die Warnung erschiene nie wieder.
            hb_warn_logged = false;

            if(bDEBUG)
                Serial.println("LOOP GATEWAY last_upd_timer actions");

            neth.last_upd_timer = millis();

            // avoid TX and UDP
            if(!neth.hasIPaddress)
            {
                neth.hasIPaddress = false;
                iReceiveTimeOutTime = millis();

                if(strlen(meshcom_settings.node_ownip) > 6 && strlen(meshcom_settings.node_ownms) > 6 && strlen(meshcom_settings.node_owngw) > 6)
                {
                    if(bDEBUG)
                    {
                        Serial.print(getTimeString());
                        Serial.println(" [MAIN] initethETH fix-IP");
                    }

                    neth.initethfixIP();
                }
                else
                {
                    Serial.print(getTimeString());
                    Serial.println(" [MAIN] resetDHCP (retry)");

                    // N-20: initethDHCP() wuerde den W5100S bei jedem
                    // Retry per initETH_HW() hardware-resetten — danach
                    // braucht die PHY-Aushandlung mehrere Sekunden und der
                    // Link-Check in startETH() sieht dauerhaft LinkOFF:
                    // ein einmal gezogenes Kabel verbindet nie wieder (auf
                    // Hardware beobachtet). Das volle HW-Init ist nur beim
                    // Boot noetig (Setup); hier reicht resetDHCP() ohne
                    // PHY-Reset — der Link-Zustand ist dann echt, und bei
                    // LinkOFF bricht startETH() sofort ab statt 10 s zu
                    // blocken.
                    neth.resetDHCP();
                }
            }
        }
        // ETH-01: DHCP refresh moved above, ahead of this if(bGATEWAY)
        // block, so it also runs when bGATEWAY is off.
    }
}
else if(neth.hasIPaddress)
{
    // TM-45: the block above never runs while bGATEWAY is off, so it
    // never reads the socket -- do only the NTP-reply harvest instead
    // of the full gateway receive path (no double read: exactly one of
    // the two branches runs per loop pass).
    INSTR_SECTION("udp"); neth.harvestNTP();
}
}
