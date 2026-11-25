/**
 *  @author      Ralph Weich (DD5RW)
 *  @date        2025-12-03
 */
#include "mc_text.h"
#include "msg_body.h"      // CHARS-01: strip only the trailing {NNN ACK marker
#include "posinfo_next.h"   // POSINFO-NEXT-01: clamped NEXT countdown
#include "uptime_min.h"   // wrap-safe 16-bit uptime minutes (NBR stamps)
#include "node_position.h"   // nodeSignedLat/Lon: own position as signed degrees (#1279)
#include <Arduino.h>

#include <configuration.h>
#include <debugconf.h>
#include "pin_status_json.h" // GET /pinstatus: pure JSON formatter (test/test_pin_status)
#include "pin_assign_json.h" // GET /pinassign: pure JSON formatter (test/test_pin_status)
#include "gpio_status_json.h" // GET /gpiostatus: pure JSON formatter (test/test_pin_status)
#include "gpio_control.h"      // GPIO-01 Part B: state of the free-GPIO control (glue calls ESP32 only)
#include "web_functions.h"
#include "web_txbuf.h"     // WEB-TX-01: webTxBeginChunked in send_http_header
#include "esp32/netif_mtu.h" // NMTU-01 (lwIP part ESP32 only)
#include <loop_functions.h>
#include <loop_functions_extern.h>
#include <time.h>
#include <vector>
#include <lora_setchip.h>
#include <rtc_functions.h>
#include <time_functions.h>
#include <spectral_scan.h>
#include <maxhop.h>         // CS-02: drop-down values for the text hop limit
#include <config_json.h>   // CS-03: config download/upload as one JSON object
#include <loop_breadcrumb.h> // INS-05: loopCrumbClear() before a deliberate reboot
#include <ArduinoJson.h>    // JSN-01: call_function()/setparam()/getparam() JSON escaping
#include <txring_functions.h> // WQ-01: LoRa queue panel -- txRingPrioCounts()
#include <setlog_lines.h>      // WQ-01: LoRa queue panel -- setlogDedupWindowMin()
#include "track_warning.h"    // TRK-01: Warnhinweis-Text neben dem Track-Switch
#include "nbr_matrix.h"        // NBR-W2: Nachbarschaftsmatrix -- Datenquelle fuer die neue Neighbours-Seite
#include "nbr_views.h"         // W4b: MHeard/Pfad-Seiten lesen nur noch ueber die Abfrageschicht
#include <TinyGPSPlus.h>       // DIST auf der MHeard-Seite -- reine distanceBetween()-Rechnung, kein GPS-Modul noetig
#include "sto_notice.h"        // stage 4: stoHolder() for the messages-page held mark, all boards
#include "own_msg_status.h" // durable per-message delivery state for the messages-page tick
#include "rm_runtime.h"          // RM-07 (#1189): RmStats/g_rmStats for the info page RM row
#include <prod_lock.h>      // prodLockNormalise(): an imported file may hold values the setters now refuse
#include <blocked_calls.h> // BLK-01: info row + setup card
#include <instrument.h>     // INSTRUMENT_ENABLED
#if INSTRUMENT_ENABLED && (defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS))
#include <t-deck/tdeck_debug.h> // sub_screen_rgb565()
#endif
#include "web_server_gate.h" // WEB-STOP-01: stopWebserver() acts on a running server only
#include "watchdog_feed.h"  // meshcom_wdt_feed(): neighbour matrix rows run in the WDT-guarded loop task
#include "board_led.h"         // LED-08: ledFitted() for the setup controls and info rows
#include "pin_board.h"         // GPIO-01: pin policy with the board context (GET /pinassign)
#if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
#include "t-deck/tdeck_palette.h" // TD-THEME-01 W3: tdeck_theme_parse/name (header only)
#include "command_functions.h"   // commandAction() for setparam tdecktheme
#include "t-deck/tdeck_helpers.h" // WEB-TD-01: brightnessMode/ActiveLevel/SetActiveLevel for the display controls
#include "daylight.h"            // WEB-TD-01: brightness_mode_* (BlMode names and segments)
#include <t-deck/lv_obj_functions.h> // WEB-TD-01: tdeck_refresh_SET_view() after setparam tdblbright
#endif
#include "charge_led_mode.h"   // LED-02: chargeLedModeName() for the info row
#if defined(XPOWERS_CHIP_AXP192) || defined(XPOWERS_CHIP_AXP2101)
#include "esp32/esp32_pmu.h"    // LED-02: effective charge-LED mode (RM override)
#endif
#include "web_auth.h"           // WEB-AUTH-01: nonce login + cookie sessions
#include "web_guard.h"          // RM GUI W1c: Host allowlist (DNS rebinding) and Sec-Fetch-Site (CSRF), header-only
#include "web_rm_util.h"       // RM GUI G1: rm_wipe / rm_wipe_string / rm_form_decode / rm_json_str shared with web_rm_page.cpp
#include "web_rm_page.h"       // RM GUI W2b: sub_page_remote(), rmScaffoldJs()
#include "web_rm_handlers.h"   // RM GUI W2a: /rmpasswd, /rmnodes, /rmheard, rmSendBySlot()
#include "web_rm_parse.h"      // RM GUI W2a: rmParseSendBody()
#include "web_assets.h"        // gzip blobs of the GUI's CSS/JS (tools/gen_web_assets.py); include here only
// RM GUI W1c: set per request by the guard in work_webpage(). No CORS headers are emitted any more
// (the GUI is same-origin and there is no documented cross-origin consumer); kept for the guard plumbing.
static bool web_cors_ok = true;
#if defined(ESP32)
#include "esp32/fw_update_net.h" // AU-09 (#1187): FwNetStatus/fwNetGetStatus for the info row and the update banner
#include <esp_timer.h>    // AU web GUI: /austatus "up" (seconds since boot)
#include <esp_idf_version.h>
#if ESP_IDF_VERSION_MAJOR >= 5
#include <esp_app_desc.h> // AU web GUI: /austatus "img" (running image ELF SHA-256)
#define MC_RUNNING_APP_DESC() esp_app_get_description()
#else
#include <esp_ota_ops.h>  // AU web GUI: /austatus "img" (running image ELF SHA-256); IDF 4.4 name
#define MC_RUNNING_APP_DESC() esp_ota_get_app_description()
#endif
#include "au_ui_state.h"  // AU web GUI: AuUiIn/auStatusJson for GET /austatus, auEtaS
#include "safeboot/safeboot_ver.h" // AU-12: AU_SAFEBOOT_MIN for the Firmware Update card footnote (header-only)
#ifndef MC_ENV_NAME
#define MC_ENV_NAME ""
#endif
static void sub_au_status(); // GET /austatus (defined next to call_function)
#ifndef MC_BUILD_TAG
#define MC_BUILD_TAG ""
#endif
#endif
#include <url_decode.h>         // #1173: decodeURLPercentCoding() -- full percent-decoding of WebUI parameters
#include <charset_filter.h>     // #1173: UTF-8-safe cut of the 150-byte web message
#include "cet_age.h"           // WEB-TD-01: cetEpochFromFields() for the ticking page time
#include "clock.h"              // TZ-01: tzActiveAbbrev() -- flags an unparsable node_tz on the info page
#if defined(ENABLE_MSGSTORE)
#include <msgstore_api.h> // stage 3 store node: /?page=mailbox, mboxpurge/mboxdeliver, the setup card
#endif

#include "web_UIComponents.h"
#include "web_setup.h"
#include "web_nodefunctioncalls.h"
#include "web_commonServer.h"
#include "gw_link_status.h"
#include "gw_reg_runtime.h"
#include "gw_servers.h"
#if defined(NRF52_SERIES)
#include <RAK13800_W5100S.h> // IPAddress for nrf_eth.h
#include <nrf_eth.h>          // ethIsStaticConfigured()
#endif
#if defined(BOARD_RAK4630)
#include "w5100.h"          // #1183: W5100.readSn*/writeSnMSSR for the web socket's MSS
#endif


CommonWebServer web_server(80);
CommonWebClient web_client;

void web_client_html(CommonWebClient web_client);

// NBR-W2: lokale Vorwaertsdeklaration, damit der Dispatcher weiter oben in
// dieser Datei sub_page_neighbours() aufrufen kann, ohne web_functions.h
// anzufassen (das liegt ausserhalb dieses Datei-Sets). Gehoert dort neben
// die anderen sub_page_*()-Deklarationen -- das ist ein Punkt fuer den
// naechsten, der web_functions.h anfassen darf.
void sub_page_neighbours();

// Pins page + GET /pinstatus: local declarations, as above (web_functions.h is outside this change)
void sub_page_pins();
// noinline: an inlined copy puts the JSON buffer in work_webpage's frame, i.e. on every web request
static void __attribute__((noinline)) sub_pin_status();
static void __attribute__((noinline)) sub_pin_assign();
static void __attribute__((noinline)) sub_gpio_status();

// GPIO-01 Part B: the pins the board lists as free (PIN_FREE when no function claims anything), at most `max`. A pin that an
// enabled function or the board LED claims right now gets that owner (pinFuncName() / "led") and no switch; the others come with
// the gpio control state. ESP32 only: nRF52 has no gpio control, the count is 0 and the code compiles away.
static size_t gpioStatusCollect(GpioStatusPin *out, size_t max)
{
#if defined(ESP32)
    PinContext base = pinBoardContext(PIN_FN_COUNT);
    const uint64_t claimedAll = pinBoardClaimed(PIN_FN_COUNT);
    base.claimed = 0;
    GpioPinRow rows[GPIO_STATUS_MAX_PINS];
    const size_t nr = out ? gpioCtlSnapshot(rows, GPIO_STATUS_MAX_PINS) : 0;
    size_t n = 0;
    for (int p = 0; p <= pinChipMax(base.chip) && p < 64; p++)
    {
        if (pinClassify(base, p) != PIN_FREE) continue;
        if (!out) return 1; // only asked whether there is any
        if (n >= max) break;
        GpioStatusPin &o = out[n++];
        o.pin = p;
        o.own = "";
        o.dir = 0;
        o.level = -1;
        if (claimedAll & pinBit(p))
        {
            o.own = "fn"; // two functions on one pin
            for (int f = 0; f < (int)PIN_FN_COUNT; f++)
                if (!(pinBoardClaimed((PinFunc)f) & pinBit(p)))
                {
                    o.own = pinFuncName((PinFunc)f);
                    break;
                }
#if defined(BOARD_LED)
            if (p == (int)(BOARD_LED + 0) && o.own[0] == 'f') o.own = "led";
#endif
            continue;
        }
        for (size_t i = 0; i < nr; i++)
            if (rows[i].pin == p)
            {
                o.dir = (int)rows[i].dir;
                o.level = (int)rows[i].level;
            }
    }
    return n;
#else
    (void)out;
    (void)max;
    return 0;
#endif
}

// GPIO-01 W2: may the Pins page change this assignment on this build? The same guards as the wsParams rows of
// web_setup.cpp (writable rows only). Used by GET /pinassign and by the nav button, so the page is reachable
// whenever it has something to offer.
static bool pinAssignAvail(PinFunc f)
{
    switch (f)
    {
    case PIN_FN_ONEWIRE:
#if defined(OneWire_GPIO)
        return true;
#else
        return false;
#endif
    case PIN_FN_BUTTON:
#if !defined(BOARD_T_DECK_PRO)
        return true;
#else
        return false;
#endif
    case PIN_FN_ANALOG:
#if defined(ANALOG_PIN)
        return true;
#else
        return false;
#endif
    case PIN_FN_WIND:
#if defined(ENABLE_WIND) && defined(ESP32)
        return true;
#else
        return false;
#endif
    default:
        return false;
    }
}


String web_header;

unsigned long web_currentTime = millis(); // Current time
unsigned long web_previousTime = 0;       // Previous time
#define WEB_TIMEOUT_TIME 2000             // Define timeout time in milliseconds (example: 2000ms = 2s)
bool bweb_server_running = false;

#if defined(BOARD_RAK4630)
/**
 * Issue #1183: the W5100S has no path-MTU discovery and sends full 1460-byte MSS
 * segments with DF set. Behind a tunnel with a path MTU below 1500 (HAMNET) the
 * first large web page chunk is blackholed and the socket times out. The user-set
 * Ethernet MTU therefore caps the MSS (MTU - 40) the web server advertises.
 *
 * Sn_MSSR is latched by the chip at Sock_OPEN (datasheet 3.2.9); writing it on a
 * socket that is already LISTENing changes nothing. So the target is pre-written
 * to every CLOSED socket (the register survives CLOSE, and whichever index
 * socketBegin() picks next latches it; a UDP socket inheriting it is harmless,
 * its datagrams stay far below 548 B), and a port-80 LISTEN socket that latched
 * an older value is closed and re-opened through web_server.begin().
 *
 * Called after every available() (which re-listens on a fresh socket) and once
 * after begin(). Steady state: SR (+MSSR) reads only, no writes. The clamp keeps
 * a corrupt stored value away from MSS 0 or above 1460.
 */
static void webApplyEthMss()
{
    int mtu = meshcom_settings.node_ethmtu;

    if (mtu < 1280) mtu = 1280;
    if (mtu > 1500) mtu = 1500;

    const uint16_t mss = (uint16_t)(mtu - 40);

    // same cap as EthernetServer::available(): W5100S/W5100 have 4 sockets, and
    // chip 0 means no Ethernet hardware (no SPI traffic then)
    const uint8_t chip = W5100.getChip();
    if (!chip)
        return;

    uint8_t nsock = MAX_SOCK_NUM;
    if (nsock > 4 && (chip == 50 || chip == 51))
        nsock = 4;

    bool closedStale = false;

    // pass A: pre-write CLOSED sockets, close a stale port-80 LISTEN socket
    for (uint8_t i = 0; i < nsock; i++)
    {
        SPI.beginTransaction(SPI_ETHERNET_SETTINGS);

        const uint8_t sr = W5100.readSnSR(i);

        if (sr == SnSR::CLOSED)
        {
            if (W5100.readSnMSSR(i) != mss)
                W5100.writeSnMSSR(i, mss);
        }
        else if (sr == SnSR::LISTEN && EthernetServer::server_port[i] == 80 && W5100.readSnMSSR(i) != mss)
        {
            W5100.execCmdSn(i, Sock_CLOSE);
            EthernetServer::server_port[i] = 0;
            closedStale = true;
        }

        SPI.endTransaction();
    }

    if (!closedStale)
        return;

    // pass B: the socket just closed must hold the target before it is re-opened
    for (uint8_t i = 0; i < nsock; i++)
    {
        SPI.beginTransaction(SPI_ETHERNET_SETTINGS);

        if (W5100.readSnSR(i) == SnSR::CLOSED && W5100.readSnMSSR(i) != mss)
            W5100.writeSnMSSR(i, mss);

        SPI.endTransaction();
    }

    web_server.begin();   // socketBegin() takes the first CLOSED index and latches its MSSR
}
#endif

// WEB-AUTH-01: platform random source for login nonces and session tokens (web_auth.h WebAuthRng)
#if defined(ESP32)
#include <esp_random.h>
static bool web_auth_rng(uint8_t *out, size_t n)
{
    esp_fill_random(out, n); // hardware TRNG
    return true;
}
#elif defined(NRF52_SERIES)
#include <nrf_soc.h>
#include <nrf_sdm.h>
static bool web_auth_rng(uint8_t *out, size_t n)
{
    // via the SoftDevice when it is enabled (NRF_RNG is owned by it then), else the peripheral directly
    uint8_t sd_en = 0;
    sd_softdevice_is_enabled(&sd_en);
    size_t i = 0;
    uint32_t t0 = millis();
    if (sd_en)
    {
        while (i < n && (uint32_t)(millis() - t0) < 200)
        {
            uint8_t avail = 0;
            sd_rand_application_bytes_available_get(&avail);
            size_t want = n - i;
            if (avail > 0 && want > avail)
                want = avail;
            if (avail > 0 && sd_rand_application_vector_get(out + i, (uint8_t)want) == NRF_SUCCESS)
                i += want;
            else
                delay(1);
        }
    }
    else
    {
        NRF_RNG->EVENTS_VALRDY = 0;
        NRF_RNG->TASKS_START = 1;
        while (i < n)
        {
            uint32_t t1 = millis();
            while (!NRF_RNG->EVENTS_VALRDY && (uint32_t)(millis() - t1) < 50)
                ;
            if (!NRF_RNG->EVENTS_VALRDY)
                break;
            out[i++] = (uint8_t)NRF_RNG->VALUE;
            NRF_RNG->EVENTS_VALRDY = 0;
        }
        NRF_RNG->TASKS_STOP = 1;
    }
    // the hardware RNG did not answer in time (should never happen): no nonce, no session
    return i == n;
}
#else
#error "web_auth_rng: no random source for this platform"
#endif

// WEB-AUTH-01: client address for the serial log lines (IPAddress::toString() does not exist on nRF52)
static const char *web_remote_ip(char *buf, size_t n)
{
    IPAddress ip = web_client.remoteIP();
    snprintf(buf, n, "%i.%i.%i.%i", ip[0], ip[1], ip[2], ip[3]);
    return buf;
}

// gps_functions.cpp instanziiert das TinyGPSPlus-Objekt unbedingt (auch ohne
// ENABLE_GPS); hier nur fuer distanceBetween() auf der MHeard-Seite genutzt.
extern TinyGPSPlus gps;

double dlat;
double dlon;
char clat;
char clon;

bool bMDNSOK=true;

/**
 * ###########################################################################################################################
 * initialize the Web Server
 */
void startWebserver()
{
    // esp32loop() calls this every pass while iWlanWait == 0; without an IP
    // the debug line came out ~125 times per second (TM-21). Log the state
    // once, again only after it changed.
    static bool s_noIpLogged = false;

    if (bweb_server_running)
        return;
    #ifdef HAS_ETHERNET
        if (meshcom_settings.node_netmode == 0)
        {
            if (strlen(meshcom_settings.node_ip) < 7 && !bWIFIAP)
            {
                if(bDEBUG && !s_noIpLogged)
                {
                    Serial.print("[WEB]...no ip set :");
                    Serial.println(meshcom_settings.node_ip);
                    s_noIpLogged = true;
                }

                stopWebserver();
                return;
            }
        }
    #else
        if (strlen(meshcom_settings.node_ip) < 7 && !bWIFIAP)
        {
            if(bDEBUG && !s_noIpLogged)
            {
                Serial.print("[WEB]...no ip set :");
                Serial.println(meshcom_settings.node_ip);
                s_noIpLogged = true;
            }

            stopWebserver();
            return;
        }
    #endif

    s_noIpLogged = false;               // IP is set now; log again if it goes away

#ifdef ESP32
    // Check if WiFi is actually connected or AP is active before trying to start MDNS
    bool is_connected = (WiFi.status() == WL_CONNECTED);
    bool is_ap_active = (WiFi.getMode() == WIFI_MODE_AP) || (WiFi.getMode() == WIFI_MODE_APSTA);

    #ifdef HAS_ETHERNET
    bool ethernet_mode = (meshcom_settings.node_netmode == 1);
    #else
    bool ethernet_mode = false;
    #endif

    if (!is_connected && !is_ap_active && !ethernet_mode)
    {
        return;
    }

    web_server.stop();

    MDNS.end();

    // Set up mDNS responder:
    // - first argument is the domain name, in this example
    //   the fully-qualified domain name is "esp32.local"
    // - second argument is the IP address to advertise
    //   we send our IP address on the WiFi network
    if(bMDNSOK)
    {
        if (!MDNS.begin(meshcom_settings.node_call))
        {
            Serial.print(getTimeString());
            Serial.println("[Web]...Error setting up MDNS responder!");
            
            stopWebserver();

            bMDNSOK=false;

            return;
        }
        else
        {
            if (bDEBUG)
            {
                Serial.print(getTimeString());
                Serial.println("[Web]...mDNS responder started");
            }
        }
    }

    netif_mtu::applyConfiguredMtu(meshcom_settings.node_ethmtu); // NMTU-01: before the first accept
    web_server.begin();


#else
    if (web_server.server_port[1] == 0)
    {
        web_server.begin();
    }
    #if defined(BOARD_RAK4630)
    webApplyEthMss();
    #endif
#endif
    bweb_server_running = true;

    if (bDEBUG)
    {
        Serial.print(getTimeString());
        Serial.println("[Web]...WEBServer started");
    }
}

/**
 * ###########################################################################################################################
 * Stop the Web Server
 */
void stopWebserver()
{
    // startWebserver() calls this every loop pass without an IP (WEB-STOP-01)
    if (!webStopShouldAct(bweb_server_running))
        return;

#ifdef ESP32
    MDNS.end();
    web_server.stop();
#endif
    bweb_server_running = false;

    if (bDEBUG)
    {
        Serial.print(getTimeString());
        Serial.println("[Web]...WEBServer stopped");
    }
}

/**
 * ###########################################################################################################################
 * loop function for Web Server
 */
void loopWebserver()
{
    if (!bweb_server_running)
        return;

    #ifdef HAS_ETHERNET
    if (meshcom_settings.node_netmode == 0)
    {
        if (strlen(meshcom_settings.node_ip) < 7)
        {
            return;
        }
    }
    #else
    if (strlen(meshcom_settings.node_ip) < 7)
    {
        return;
    }
    #endif

    web_client = web_server.available(); // Create a client connection.

    #if defined(BOARD_RAK4630)
    webApplyEthMss();   // #1183: available() may just have re-listened on a fresh socket
    #endif

    // HTML Page formating
    if (web_client)
    {
        // If a new client connects,
        web_client_html(web_client);
    }

    // Close the connection
    web_client.stop();
}

/**
 * ###########################################################################################################################
 * Web Client Handler
 */
void web_client_html(CommonWebClient web_client)
{
    if (bDEBUG)
    {
        Serial.print("[Web]...Client IP: ");
        Serial.println(web_client.remoteIP());
    }

    // WEB-AUTH-01: the login state is a session cookie, checked in work_webpage()
    work_webpage();
}

/**
 * ###########################################################################################################################
 * CS-03: config download / upload
 *
 * One buffer serves both directions -- the export never runs while an upload
 * body is being read. The worst-case export (every string field filled to its
 * limit) measures about 3.1 kB, so CONFIG_JSON_MAX is roughly a factor of two
 * of headroom and at the same time the hard cap on what an upload may send.
 *
 * On the heap, NOT in BSS: 6 kB of static buffer overflows dram0_0_seg on the
 * plain ESP32 (E22-DevKitC links with ~4 kB to spare). It is allocated for the
 * duration of one /config.json or POST /config request and released again --
 * both are rare, operator-triggered requests.
 */
static char *web_cfg_buf = NULL;

static bool web_cfg_alloc(void)
{
    if (web_cfg_buf == NULL)
        web_cfg_buf = (char *)malloc(CONFIG_JSON_MAX + 1);

    if (web_cfg_buf != NULL)
        web_cfg_buf[0] = '\0';

    return web_cfg_buf != NULL;
}

static void web_cfg_free(void)
{
    if (web_cfg_buf != NULL)
    {
        free(web_cfg_buf);
        web_cfg_buf = NULL;
    }
}

/**
 * Reads the request BODY.
 *
 * work_webpage() has only ever read the request HEADER -- it stops at the
 * blank line and answers. A file upload is the first thing here that carries
 * a body, so this is the reader for it: bounded by the Content-Length the
 * caller picked out of the header, hard-capped at CONFIG_JSON_MAX, and given
 * up on after WEB_TIMEOUT_TIME without a byte so a lying Content-Length
 * cannot pin the loop.
 *
 * @return bytes read into web_cfg_buf (NUL-terminated), or negative:
 *         -1 no/!invalid Content-Length, -2 too large, -3 short read.
 */
static int web_read_body(long content_length)
{
    web_cfg_buf[0] = '\0';

    if (content_length <= 0)
        return -1;

    if (content_length > (long)CONFIG_JSON_MAX)
        return -2;

    long          got = 0;
    unsigned long last = millis();
    const unsigned long start = last;

    // Total deadline as well as the idle one: a client dripping one byte per
    // ~2 s would otherwise pin the loop task past the 5 s ESP32 task WDT.
    while (got < content_length && (millis() - last) <= WEB_TIMEOUT_TIME && (millis() - start) <= 4000UL)
    {
        yield();

        if (web_client.available())
        {
            int c = web_client.read();
            if (c < 0)
                break;

            web_cfg_buf[got++] = (char)c;
            last = millis();
        }
        else if (!web_client.connected())
        {
            break;
        }
    }

    web_cfg_buf[got] = '\0';

    return (got == content_length) ? (int)got : -3;
}

/**
 * ###########################################################################################################################
 * RM-09 / W2 (#1189): remote management web endpoints -- POST /rmsend and GET /rmstatus.
 *
 * Both run in the loop task (the ESP32 and the nRF52 web server are both driven from there), which is
 * what the rm_runtime.h sender/status API requires.
 *
 * W2-1: the TARGET's password travels in the BODY of POST /rmsend only. It is therefore never part of a
 * request line, so it never reaches web_header or web_currentLine (the request-line Strings), the bDEBUG
 * request echo in work_webpage() (that one is switched off for /rmsend anyway), /setparam, /callfunction or
 * any param print. The handler reads the body straight from the socket into ONE fixed stack buffer, parses
 * it in place, hands the password pointer to rmSendCommand() (no copy there either) and wipes the buffer
 * with volatile stores before it returns.
 */

/** POST /rmsend   body: dst=<call>&pw=<target password>&cmd=<cmd>&args=<args>   (all percent-encoded)
 *             or: slot=<0..2>&cmd=<cmd>&args=<args>   (a saved node: its key comes from the store, never a password)
 *             optional &force=1: use the one "try once more" after a re-key (only where canForce is reported)
 *             optional &call=<CALL> (slot form only): refused with "slot" unless the slot holds exactly that call
 *  answer: {"ok":true,"ctr":N[,"viaSync":true]} or {"ok":false,"err":"<reason>","msg":"<sentence>","retry":S,"canForce":0|1}
 *  err "range" (txpower outside the range learnt for that target) adds "range":"<min>..<max>"
 *  -- never an echo of any input */
static void sub_rm_send(long content_length)
{
    char body[RM_FORM_BODY_MAX + 1];
    char err[16] = {0};
    uint32_t ctr = 0;
    bool ok = false;
    bool viaSync = false; // RM GUI W1b: the command waits behind an automatic sync (ctr 0)
    uint32_t retryS = 0;  // refusal only: seconds until the target accepts a send again
    bool canForce = false; // refusal only: the one "try once more" after a re-key is available (force=1)
    char rangeDst[16] = {0}; // refusal "range" only: the target call, copied before the body is wiped

    body[0] = '\0';

    // RM GUI W2a: shared absolute-deadline body read and the pure parser (web_rm_parse.h)
    const char *e = rmReadBody(body, sizeof(body), content_length);
    if (e != nullptr)
        snprintf(err, sizeof(err), "%s", e);
    else
    {
        const RmSendReq q = rmParseSendBody(body);
        if (q.err != nullptr)
            snprintf(err, sizeof(err), "%s", q.err);
        else if (q.slot >= 0)
        {
            ok = rmSendBySlot(q.slot, q.cmd, q.args, err, sizeof(err), &ctr, &viaSync, q.force, q.call);
            if (!ok)
                rmSlotTargetInfo(q.slot, &retryS, &canForce, rangeDst, sizeof(rangeDst));
        }
        else
        {
            ok = rmSendCommand(q.dst, q.pw, q.cmd, q.args, err, sizeof(err), &ctr, &viaSync, q.force);
            if (!ok)
            {
                rmTargetMaySend(q.dst, &retryS, &canForce);
                snprintf(rangeDst, sizeof(rangeDst), "%s", q.dst != nullptr ? q.dst : "");
            }
        }
    }

    rm_wipe(body, sizeof(body)); // dst/pw/cmd/args point into it: the password is gone from here on

    send_http_header(ok ? 200 : 422, RESPONSE_TYPE_JSON);
    if (ok)
        web_client.printf("{\"ok\":true,\"ctr\":%lu%s}\n", (unsigned long)ctr, viaSync ? ",\"viaSync\":true" : "");
    else
    {
        const char *tok = err[0] ? err : "send";
        web_client.print("{\"ok\":false,\"err\":");
        rm_json_str(tok);
        web_client.print(",\"msg\":");
        rm_json_str(rmErrTokenMessage(tok)); // the sentence comes from rmTokenTable, the JS keeps no copy
        int rMin = 0, rMax = 0;
        if (strcmp(tok, "range") == 0 && rmTargetTxRange(rangeDst, &rMin, &rMax))
            web_client.printf(",\"range\":\"%d..%d\"", rMin, rMax); // the txpower range learnt for that target
        web_client.printf(",\"retry\":%lu,\"canForce\":%d}\n", (unsigned long)retryS, canForce ? 1 : 0);
    }
}

/** GET /rmstatus -- this node's RM state, the last 5 executed commands, the whole sent book (<= 12) and
 *  every managed node (distinct dst of the book + the pending chain's target); streamed entry by entry.
 *  Never a key, a tag or a password. All text is printable ASCII, < > & escaped (rm_json_str).
 *  {"on":0|1,"pw":0|1,"ok":N,"rej":N,"lock":0|1,"lockS":N,"hwm":N,
 *   "log":[{"ago":S,"src":"","ctr":N,"cmd":"","res":""},...],
 *   "sent":[{"dst":"","ctr":N,"cmd":"","ago":S,"rep":0|1,"ver":0|1,"reply":"","st":"queued|waiting|noanswer|ok|err|unverified","msg":""},...],
 *   "targets":[{"dst":"","locked":0|1,"retry":S,"canForce":0|1,"pending":0|1,"chainErr":null|"token","chainMsg":null|"sentence"},...]} */
static void sub_rm_status(void)
{
    const uint32_t now = (uint32_t)millis();

    send_http_header(200, RESPONSE_TYPE_JSON);

    {
        RmStatusHead st;
        rmGetStatusHead(st);
        const unsigned long rej = (unsigned long)st.stats.rej_format + st.stats.rej_tag + st.stats.rej_replay +
                                  st.stats.rej_blocked + st.stats.rej_rate + st.stats.rej_lockout + st.stats.rej_disabled;
        web_client.printf("{\"on\":%d,\"pw\":%d,\"ok\":%lu,\"rej\":%lu,\"lock\":%d,\"lockS\":%lu,\"hwm\":%lu,\"log\":[",
                          st.on ? 1 : 0, st.passwdSet ? 1 : 0, (unsigned long)st.stats.ok, rej, st.lockActive ? 1 : 0,
                          (unsigned long)st.lockRemainS, (unsigned long)st.hwm);
        RmLogEntry le; // one entry at a time (P10): no copy of the whole log on the stack
        for (uint8_t i = 0, shown = 0; i < rmLogCount() && rmLogAt(i, &le); i++, shown++)
        {
            web_client.printf("%s{\"ago\":%lu,\"src\":", shown ? "," : "", (unsigned long)((uint32_t)(now - le.ms) / 1000UL));
            rm_json_str(le.src);
            web_client.printf(",\"ctr\":%lu,\"cmd\":", (unsigned long)le.ctr);
            rm_json_str(le.cmd);
            web_client.print(",\"res\":");
            rm_json_str(le.result);
            web_client.print("}");
        }
        web_client.print("]");
    }

    {
        RmSent sent; // the whole book (<= 12), one entry at a time
        web_client.print(",\"sent\":[");
        for (uint8_t i = 0; rmSentAt(i, &sent); i++)
        {
            web_client.printf("%s{\"dst\":", i ? "," : "");
            rm_json_str(sent.dst);
            web_client.printf(",\"ctr\":%lu,\"cmd\":", (unsigned long)sent.ctr);
            rm_json_str(sent.cmd);
            web_client.printf(",\"ago\":%lu,\"rep\":%d,\"ver\":%d,\"reply\":", (unsigned long)((uint32_t)(now - sent.sentMs) / 1000UL),
                              sent.replied ? 1 : 0, sent.verified ? 1 : 0);
            rm_json_str(sent.reply);
            web_client.print(",\"st\":");
            rm_json_str(sent.stateName);
            web_client.print(",\"msg\":");
            rm_json_str(sent.msg);
            web_client.print("}");
        }
        web_client.print("]");
    }

    {   // RM GUI W1b: per managed node, the sender policy as the GUI needs it (locked after 2 unanswered sends)
        RmTarget tg; // every distinct dst of the book plus the pending chain's target, one at a time
        web_client.print(",\"targets\":[");
        for (uint8_t i = 0; rmTargetAt(i, &tg); i++)
        {
            web_client.printf("%s{\"dst\":", i ? "," : "");
            rm_json_str(tg.dst);
            web_client.printf(",\"locked\":%d,\"retry\":%lu,\"canForce\":%d,\"pending\":%d,\"chainErr\":", tg.locked ? 1 : 0,
                              (unsigned long)tg.retryS, tg.canForce ? 1 : 0, tg.pending ? 1 : 0);
            if (tg.chainErr != nullptr)
                rm_json_str(tg.chainErr);
            else
                web_client.print("null");
            web_client.print(",\"chainMsg\":");
            if (tg.chainMsg != nullptr)
                rm_json_str(tg.chainMsg);
            else
                web_client.print("null");
            web_client.print("}");
        }
        web_client.print("]");
    }

    web_client.println("}");
}

/**
 * GET /config.json -- the whole configuration as a downloadable file.
 *
 * Sends its own header instead of send_http_header(): only this response
 * carries a Content-Disposition, which is what turns a browser navigation
 * into a download instead of a page full of JSON.
 */
static void sub_config_download(void)
{
    if (!web_cfg_alloc())
    {
        send_http_header(422, RESPONSE_TYPE_JSON);
        web_client.println("{\"error\":\"out of memory\"}");
        return;
    }

    size_t n = configExportJson(web_cfg_buf, CONFIG_JSON_MAX);

    if (n == 0)
    {
        web_cfg_free();
        send_http_header(422, RESPONSE_TYPE_JSON);
        web_client.println("{\"error\":\"config export did not fit the buffer\"}");
        return;
    }

    /* the callsign goes into a filename -- keep it to characters that cannot
     * break out of the quoted header value or of a directory */
    char fname[16];
    size_t fi = 0;
    for (const char *p = meshcom_settings.node_call; *p && fi < sizeof(fname) - 1; p++)
    {
        char c = *p;
        if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || c == '-')
            fname[fi++] = c;
    }
    fname[fi] = '\0';
    if (fi == 0)
        snprintf(fname, sizeof(fname), "node");

    web_client.printf("HTTP/1.1 200 OK \n");
    web_client.println("Content-type:application/json");
    web_client.printf("Content-Disposition: attachment; filename=\"meshcom-%s.json\"\n", fname);
    web_client.printf("Content-Length: %u\n", (unsigned int)n);
    web_client.println("Connection: close");
    web_client.println("Cache-Control: no-cache, no-store, must-revalidate");
    web_client.println("Pragma: no-cache");
    web_client.println("Expires: 0");
    web_client.println();

    web_client.print(web_cfg_buf);

    web_cfg_free();
}

/**
 * POST /config -- restore a configuration file and reboot once.
 *
 * Nothing is written unless configImportJson() accepted every value; on
 * failure the answer is a 400 with the reason and NVRAM is untouched. On
 * success the response is flushed and the connection closed BEFORE the reset,
 * otherwise the browser never sees the answer. Reboot pattern copied from
 * commandAction()'s --cleanflash/--reboot (command_functions.cpp).
 */
static void sub_config_upload(long content_length)
{
    char err[160] = {0};

    if (!web_cfg_alloc())
    {
        send_http_header(400, RESPONSE_TYPE_TEXT);
        web_client.printf("<p>upload rejected: out of memory</p>");
        return;
    }

    int n = web_read_body(content_length);

    if (n < 0)
    {
        web_cfg_free();
        send_http_header(400, RESPONSE_TYPE_TEXT);
        web_client.printf("<p>upload rejected: %s</p>",
                          (n == -2) ? "file too large" : (n == -1) ? "no Content-Length" : "incomplete upload");
        /* own marker: these codes are the body reader's, not configImportJson()'s */
        Serial.printf("[CONFIG];upload;rc;%d;len;%ld\n", n, content_length);
        return;
    }

    int rc = configImportJson(web_cfg_buf, (size_t)n, err, sizeof(err));

    web_cfg_free();

    if (rc != 0)
    {
        send_http_header(400, RESPONSE_TYPE_TEXT);
        web_client.printf("<p>config import failed: %s</p>", err);
        return;
    }

    // Production lock: a file from an earlier build may hold values the setters refuse now (ptime 1800,
    // pingmax 100, locked spectrum/NBR values). The import accepts them; repair before they are persisted.
    {
#if defined(NRF52_SERIES)
        const bool hasAu = false;
#else
        const bool hasAu = true;
#endif
        const int locked = prodLockNormalise(meshcom_settings, INSTRUMENT_ENABLED != 0, hasAu);
        if(locked > 0)
            Serial.printf("[CONFIG];import;normalised;%d\n", locked);
    }

    save_settings();

    send_http_header(200, RESPONSE_TYPE_TEXT);
    web_client.printf("<html><body><h3>config imported</h3><p>%s</p><p>the node reboots now.</p></body></html>", err);
    web_client.flush();
    web_client.stop();

    Serial.printf("[CONFIG];reboot;after;import\n");

    delay(2000);

    #ifdef ESP32
        loopCrumbClear();   // INS-05: deliberate reboot, no LAST_LOOP_SECTION at the next boot
        ESP.restart();
    #endif

    #if defined NRF52_SERIES
        NVIC_SystemReset();     // resets the device
    #endif
}

/**
 * ###########################################################################################################################
 * Serves one of the static GUI assets (web_assets.h): gzip blob, own header (send_http_header() says no-cache and
 * text/html). Needs no login: the login page is the scaffold and loads them too. The URL carries the content tag
 * (?v=), so the browser may keep the file. The body goes out through write(): print()/printf() would stop at a 0 byte
 * or, on the nRF, over-read the printf buffer. Returns false when the request line names no asset.
 */
static bool deliver_web_asset(const String &req)
{
    const uint8_t *blob = NULL;
    size_t len = 0;
    const char *type = "application/javascript";
    if (req.startsWith("GET /mc.css"))
    {
        blob = web_mc_css_gz;
        len = sizeof(web_mc_css_gz);
        type = "text/css";
    }
    else if (req.startsWith("GET /mc.js"))
    {
        blob = web_mc_js_gz;
        len = sizeof(web_mc_js_gz);
    }
    else if (req.startsWith("GET /mcrm.js"))
    {
        blob = web_mc_rm_js_gz;
        len = sizeof(web_mc_rm_js_gz);
    }
#if defined(ESP32)
    else if (req.startsWith("GET /mcau.js"))
    {
        blob = web_mc_au_js_gz;
        len = sizeof(web_mc_au_js_gz);
    }
#endif
    else
        return false;

    web_client.println("HTTP/1.1 200 OK");
    web_client.print("Content-type:");
    web_client.println(type);
    web_client.println("Content-Encoding: gzip");
    web_client.print("Content-Length: ");
    web_client.println((unsigned)len);
    web_client.println("Cache-Control: public, max-age=31536000, immutable");
    web_client.println("Connection: close");
    web_client.println();
    web_client.write(blob, len);
    return true;
}

/**
 * ###########################################################################################################################
 * Handle Web requests and call the matching sub function
 */

void work_webpage(void)
{
    // RAM-Rueckgewinn (2026-09-20): der 1-kB-Sammelpuffer web_header_collect
    // lag als statisches Feld im DRAM und wurde am Ende ohnehin in den String
    // kopiert. Jetzt sammelt der String selbst; reserve() holt die 1 kB EINMAL
    // beim ersten Aufruf vom Heap und behaelt sie (der String ist global),
    // also kein Wachsen und kein Freigeben je Anfrage -- dieselbe Obergrenze
    // wie vorher (WEB_HEADER_MAX), nur dass sie nicht mehr im Linkerbild steht.
    // Das ist eine Verschiebung aus dem statischen Bild in den Heap, kein
    // Byte weniger zur Laufzeit; sie entlastet die Linkregion, die auf den
    // klassischen ESP32 knapp ist.
    static const uint16_t WEB_HEADER_MAX = 1023;
    web_header.reserve(WEB_HEADER_MAX + 1);
    web_header = "";

    web_currentTime = millis();
    web_previousTime = web_currentTime;
    String web_currentLine = ""; // make a String to hold incoming data from the client

    // CS-03: an upload needs its body length, and the header may well be
    // longer than WEB_HEADER_MAX. Picked off the line as it completes,
    // so it does not depend on that 1 kB window.
    long web_content_length = -1;
    bool web_rm_quiet = false; // W2-1: set once the request line is /rmsend, suppresses the bDEBUG echo
                               // (WEB-AUTH-01: also /?login= and the legacy /?nodepassword=)
    bool web_line_quiet = false; // WEB-AUTH-01: the current line is the Cookie header, not echoed
    // WEB-AUTH-01: Cookie header value, captured as its line completes (it often arrives behind the
    // WEB_HEADER_MAX window of web_header)
    char web_cookie[256];
    size_t web_cookie_len = 0;
    web_cookie[0] = 0;

    if (bDEBUG)
    {
        #ifdef ESP
        Serial.printf("%s;[HEAP]Cnew;%d;(free)\n", getTimeString().c_str(), ESP.getFreeHeap());
        #endif
        Serial.println("New Client."); // print a message out in the serial port
    }
       
    while (web_client.connected() && (web_currentTime - web_previousTime) <= WEB_TIMEOUT_TIME)
    { // loop while the client's connected
        yield();
        web_currentTime = millis();
        if (web_client.available())
        {
            // if there's bytes to read from the client,
            char c = web_client.read(); // read a byte, then

            if (web_header.length() < WEB_HEADER_MAX)
                web_header += c;

            // W2-1: nothing of a /rmsend request is echoed to the serial monitor (its body is never
            // read here, but a GET variant would carry the target password in the request line)
            if (!web_rm_quiet && ((web_header.length() == 8 && web_header.startsWith("POST /rm")) ||
                                  (web_header.length() == 7 && web_header.startsWith("GET /rm"))))
                web_rm_quiet = true; // /rmsend, /rmpasswd, /rmnodes, ... (RM GUI W2 adds more /rm* paths)
            if (!web_rm_quiet && ((web_header.length() == 12 && web_header.startsWith("GET /?login=")) ||
                                  (web_header.length() == 19 && web_header.startsWith("GET /?nodepassword="))))
                web_rm_quiet = true; // WEB-AUTH-01: login argument / legacy plaintext password
            if (web_currentLine.length() == 7 && web_currentLine.equalsIgnoreCase("cookie:"))
                web_line_quiet = true; // WEB-AUTH-01: session token

            if (bDEBUG && !web_rm_quiet && (!web_line_quiet || c == '\n'))
                Serial.write(c); // print it out the serial monitor

            if (c == '\n')
            {
                // if the byte is a newline character
                // if the current line is blank, you got two newline characters in a row.
                // that's the end of the client HTTP request, so send a response:
                if (web_currentLine.length() == 0)
                {
                    // Serial.println(web_header);

                    // RM GUI W1c: refuse DNS-rebinding Hosts and cross-site browser requests before any
                    // routing -- /setparam, /callfunction, /rmsend ... are plain GETs a foreign page could
                    // fire with an <img>. curl/python tools send no Sec-Fetch-Site and an IP/.local Host.
                    // Must run before the login routes below (WEB-AUTH-01), which wipe web_header.
                    String wg_path = "/";
                    {
                        int wa = web_header.indexOf(' ');
                        int wb = (wa >= 0) ? web_header.indexOf(' ', wa + 1) : -1;
                        if (wa >= 0 && wb > wa)
                            wg_path = web_header.substring(wa + 1, wb);
                    }
                    // MC_WEB_GUARD=1 (default) guards every path; with 0 only requests the router sends to
                    // an RM handler are guarded and the rest is unchecked. Decided on the
                    // header, the way the router matches (web_guard.h).
                    const bool wg_on = webGuardRequestApplies(web_header.c_str(), web_header.length(), wg_path.c_str());
                    const WebGuardVerdict wgv = wg_on ? webGuardCheck(web_header.c_str(), web_header.length(), wg_path.c_str()) : WG_OK;
                    web_cors_ok = wg_on ? webGuardCorsAllowed(wg_path.c_str(), web_header.startsWith("POST")) : true;
                    if (wgv != WG_OK)
                    {
                        if (bDEBUG)
                            Serial.printf("[WEB];guard;%s\n", wgv == WG_BAD_HOST ? "bad_host" : "cross_site");
                        rm_wipe_string(web_header);
                        rm_wipe_string(web_currentLine);
                        web_cors_ok = false;
                        send_http_header(403, RESPONSE_TYPE_JSON);
                        web_client.println("{\"ok\":false,\"err\":\"forbidden\"}");
                        web_client.stop();
                        continue;
                    }

#if defined(ESP32)
                    // OTA-LOWHEAP-01 A4: while an AU session runs (gate to TLS end) only /austatus and the GUI assets
                    // are served; every other request, the 10 s mailbox refresh included, would take heap the TLS
                    // handshake needs (a page costs 4.5-7 KB peak, about 3 KB held in TIME_WAIT). The page JS retries.
                    if (!fwWebAdmit(fwNetSessionActive(), wg_path.c_str()))
                    {
                        static const char kBusy[] = "{\"ok\":false,\"err\":\"au_session_active\"}";
                        // a full page load (reload, bookmark) gets a readable page that reloads itself
                        static const char kBusyHtml[] =
                            "<!DOCTYPE html><meta charset=utf-8><meta http-equiv=refresh content=10>"
                            "<p>Update l\u00e4uft, die Seite l\u00e4dt in 10 s neu. / Update running, reloading in 10 s.";
                        const bool page = wg_path == "/" || wg_path.startsWith("/?");
                        const char *body = page ? kBusyHtml : kBusy;
                        const unsigned blen = (unsigned)(page ? sizeof(kBusyHtml) - 1 : sizeof(kBusy) - 1);
                        web_cors_ok = false;
                        web_client.println("HTTP/1.1 503 Service Unavailable");
                        web_client.println("Retry-After: 10");
                        web_client.println(page ? "Content-Type: text/html; charset=utf-8" : "Content-Type: application/json");
                        web_client.print("Content-Length: ");
                        web_client.println(blen);
                        web_client.println("Connection: close");
                        web_client.println();
                        web_client.write((const uint8_t *)body, blen);
                        rm_wipe_string(web_header);
                        rm_wipe_string(web_currentLine);
                        web_client.stop();
                        continue;
                    }
#endif


                    // static CSS/JS of the GUI: before the login gate (the login page needs them), after the guard
                    if (deliver_web_asset(web_header))
                    {
                        web_client.stop();
                        continue;
                    }

                    // WEB-AUTH-01: login routes (Web-API_documentation.txt "Login"), after the guard and before the auth gate
                    if (wg_path.startsWith("/?loginnonce"))
                    {
                        char nonce[33];
                        if (webAuthIssueNonce(millis(), web_auth_rng, nonce))
                        {
                            send_http_header(200, RESPONSE_TYPE_TEXT);
                            web_client.print(nonce);
                        }
                        else
                        {
                            send_http_header(403, RESPONSE_TYPE_JSON);
                            web_client.println("{\"ok\":false,\"err\":\"throttled\"}");
                        }
                        web_client.stop();
                        continue;
                    }
                    if (wg_path.startsWith("/?login="))
                    {
                        String wa_arg = wg_path.substring(8);
                        int wa_amp = wa_arg.indexOf('&');
                        if (wa_amp >= 0)
                            wa_arg = wa_arg.substring(0, wa_amp);
                        char token[33];
                        char wa_ip[20];
                        WebAuthResult war = webAuthLogin(wa_arg.c_str(), wa_arg.length(), meshcom_settings.node_webpwd, millis(),
                                                         web_auth_rng, token);
                        if (war == WA_OK)
                        {
                            char hdr[96];
                            snprintf(hdr, sizeof(hdr), "Set-Cookie: mcs=%s; Path=/; HttpOnly; SameSite=Strict", token);
                            send_http_header(200, RESPONSE_TYPE_JSON, hdr);
                            web_client.println("{\"ok\":true}");
                            memset(hdr, 0, sizeof(hdr));
                            memset(token, 0, sizeof(token));
                            Serial.print(getTimeString());
                            Serial.printf(" WEBServer Password OK IP:<%s>\n", web_remote_ip(wa_ip, sizeof(wa_ip)));
                        }
                        else
                        {
                            send_http_header(403, RESPONSE_TYPE_JSON);
                            web_client.printf("{\"ok\":false,\"err\":\"%s\"}\n", webAuthErrCode(war));
                            Serial.print(getTimeString());
                            Serial.printf(" WEBServer Password rejected (%s) IP:<%s>\n", webAuthErrCode(war),
                                          web_remote_ip(wa_ip, sizeof(wa_ip)));
                        }
                        rm_wipe_string(wa_arg);
                        rm_wipe_string(wg_path);
                        rm_wipe_string(web_header);
                        rm_wipe_string(web_currentLine);
                        web_client.stop();
                        continue;
                    }
                    if (wg_path.startsWith("/?logout") || wg_path == "/?nodepassword=" || wg_path == "/?nodepassword")
                    {
                        webAuthLogout(web_cookie, web_cookie_len);
                        send_http_header(200, RESPONSE_TYPE_TEXT, "Set-Cookie: mcs=; Path=/; Max-Age=0; HttpOnly; SameSite=Strict");
                        if (bDEBUG)
                            Serial.println("WebUI requested logout");
                        web_client.stop();
                        continue;
                    }
                    if (wg_path.startsWith("/?nodepassword="))
                    {
                        // legacy plaintext login: refused, the value is neither used nor logged
                        rm_wipe_string(wg_path);
                        rm_wipe_string(web_header);
                        rm_wipe_string(web_currentLine);
                        send_http_header(403, RESPONSE_TYPE_JSON);
                        web_client.println("{\"ok\":false,\"err\":\"plaintext\"}");
                        web_client.stop();
                        continue;
                    }

                    // WEB-AUTH-01: a password is set and the request carries no live session cookie
                    const bool bget_password = strlen(meshcom_settings.node_webpwd) > 0 &&
                                               !webAuthSessionValid(web_cookie, web_cookie_len, millis());

                    // in every other case, we check if authentication is required but not yet provided
                    if (bget_password)
                    {
                        // user requested a page ? Send a login page instead.
                        if (web_header.indexOf("/?page=") >= 0)
                        {
                            send_http_header(200, RESPONSE_TYPE_TEXT);
                            sub_page_login();
                        }
                        // user requested anything else (but not just "/")? Send a 401 - Unauthorized as answear
                        else if (web_header.indexOf("/") >= 0 && web_header.indexOf("HTTP/1.1") > web_header.indexOf("/") + 2)
                        {
                            send_http_header(401, RESPONSE_TYPE_TEXT);
                        }
                        else
                        {
                            Serial.print(getTimeString());
                            {
                                char wa_ip[20];
                                Serial.printf(" WEBServer no session IP:<%s> show LOGIN\n", web_remote_ip(wa_ip, sizeof(wa_ip)));
                            }
                            deliver_scaffold(true);
                        }
                    }
                    else
                    {

                        if (web_header.startsWith("POST /rmsend"))
                        { // W2-1: remote management send, the password is in the BODY, read by the handler
                            sub_rm_send(web_content_length);
                        }
                        else if (web_header.startsWith("GET /rmsend"))
                        { // W2-1: refused, and the request line is wiped in case it carried a password
                            rm_wipe_string(web_header);
                            send_http_header(405, RESPONSE_TYPE_JSON);
                            web_client.println("{\"ok\":false,\"err\":\"use POST\"}");
                        }
                        else if (web_header.startsWith("GET /rmstatus"))
                        { // W2: status + sent list for the remote management card
                            sub_rm_status();
                        }
                        else if (web_header.startsWith("POST /rmpasswd"))
                        { // RM GUI: set / clear node_passwd, the password is in the BODY
                            sub_rm_passwd(web_content_length);
                        }
                        else if (web_header.startsWith("POST /rmwebpwd"))
                        { // WEB-PWD-01: set / clear the web login password, the password is in the BODY
                            sub_web_passwd(web_content_length);
                        }
                        else if (web_header.startsWith("GET /pinstatus"))
                        { // Pins page: cached state of the board LED, charge LED and MCP23017 pins, read-only, GET only
                            sub_pin_status();
                        }
                        else if (web_header.startsWith("GET /pinassign"))
                        { // Pins page: the four pin assignments (1-Wire, button, analog, anemometer), read-only, GET only
                            sub_pin_assign();
                        }
                        else if (web_header.startsWith("GET /gpiostatus"))
                        { // Pins page: the free GPIOs and the gpio control state, read-only, GET only
                            sub_gpio_status();
                        }
                        else if (web_header.startsWith("GET /rmnodes"))
                        { // RM GUI: the three saved nodes (slot, used, call), never a key
                            sub_rm_nodes_get();
                        }
                        else if (web_header.startsWith("POST /rmnodes"))
                        { // RM GUI: save / delete / forget saved nodes, the password is in the BODY
                            sub_rm_nodes_post(web_content_length);
                        }
                        else if (web_header.startsWith("GET /rmheard"))
                        { // RM GUI: up to 12 heard nodes for the Remote page chips
                            sub_rm_heard();
                        }
                        else if (web_header.indexOf("/?page=remote") >= 0)
                        { // RM GUI: the Remote page
                            send_http_header(200, RESPONSE_TYPE_TEXT);
                            sub_page_remote();
                        }
#if defined(ESP32)
                        else if (web_header.startsWith("GET /austatus"))
                        { // AU web GUI: read-only update state for the Info page row, no CORS (node-local)
                            web_cors_ok = false;
                            sub_au_status();
                        }
#endif
                        else if (web_header.indexOf("/callfunction/") >= 0)
                        { // user requested to invoke a function
                            // ### !!function will generate a HTML header itself
                            call_function(web_header);
                        }
                        else if (web_header.indexOf("/?sendmessage") >= 0)
                        { // user requested to send a message to the mesh
                            send_http_header(200, RESPONSE_TYPE_TEXT);
                            send_message(web_header);
                        }
                        else if (web_header.indexOf("/?getmessages") >= 0)
                        { // user requested to retrieve the stored messages
                            send_http_header(200, RESPONSE_TYPE_TEXT);
                            sub_content_messages();
                        }
                        else if (web_header.indexOf("/setparam/") >= 0)
                        { // user requested to set a parameter
                            // ### !!function will generate a HTML header itself
                            setparam(web_header);
                        }
                        else if (web_header.indexOf("/getparam/") >= 0)
                        { // user requested to get a parameter
                            // ### !!function will generate a HTML header itself
                            getparam(web_header);
                        }
                        else if (web_header.indexOf("/config.json") >= 0)
                        { // CS-03: download the configuration as a file
                            // ### !!function will generate a HTML header itself
                            sub_config_download();
                        }
                        else if (web_header.indexOf("POST /config") >= 0)
                        { // CS-03: restore a configuration file, then reboot
                            // ### !!function will generate a HTML header itself
                            sub_config_upload(web_content_length);
                        }
                        else if (web_header.indexOf("/?page=setup") >= 0)
                        { // user requested the position page
                            send_http_header(200, RESPONSE_TYPE_TEXT);
                            sub_page_setup();
                        }
                        else if (web_header.indexOf("/?page=position") >= 0)
                        { // user requested the position page
                            send_http_header(200, RESPONSE_TYPE_TEXT);
                            sub_page_position();
                        }
                        else if (web_header.indexOf("/?page=wx") >= 0)
                        { // user requested the weather page
                            send_http_header(200, RESPONSE_TYPE_TEXT);
                            sub_page_wx();
                        }
                        else if (web_header.indexOf("/?page=mheard") >= 0)
                        { // user requested the mheard page
                            send_http_header(200, RESPONSE_TYPE_TEXT);
                            sub_page_mheard();
                        }
                        else if (web_header.indexOf("/?page=neighbours") >= 0)
                        { // NBR-W2: user requested the neighbour-matrix page
                            send_http_header(200, RESPONSE_TYPE_TEXT);
                            sub_page_neighbours();
                        }
                        else if (web_header.indexOf("/?page=messages") >= 0)
                        { // user requested the messages page
                            send_http_header(200, RESPONSE_TYPE_TEXT);
                            sub_page_messages();
                        }
#if defined(ENABLE_MSGSTORE)
                        else if (web_header.indexOf("/?page=mailbox") >= 0)
                        { // user requested the mailbox (store node) page, stage 3
                            send_http_header(200, RESPONSE_TYPE_TEXT);
                            sub_page_mailbox();
                        }
#endif
                        else if (web_header.indexOf("/?page=rxlog") >= 0)
                        { // user requested the rx log page
                            send_http_header(200, RESPONSE_TYPE_TEXT);
                            sub_page_rxlog();
                        }
                        else if (web_header.indexOf("/?page=path") >= 0)
                        { // user requested the path page
                            send_http_header(200, RESPONSE_TYPE_TEXT);
                            sub_page_path();
                        }
                        else if (web_header.indexOf("/?page=spectrum") >= 0)
                        { // user requested the path page
                            send_http_header(200, RESPONSE_TYPE_TEXT);
                            sub_page_spectrum();
                        }
                        else if (web_header.indexOf("/?page=mcp23017") >= 0)
                        { // user requested the path page
                            send_http_header(200, RESPONSE_TYPE_TEXT);
                            sub_page_mcp23017();
                        }
                        else if (web_header.indexOf("/?page=mcp23017") >= 0)
                        { // user requested the path page
                            send_http_header(200, RESPONSE_TYPE_TEXT);
                            sub_page_mcp23017();
                        }
                        else if (web_header.indexOf("/?page=pins") >= 0)
                        { // user requested the pins page
                            send_http_header(200, RESPONSE_TYPE_TEXT);
                            sub_page_pins();
                        }
                        else if (web_header.indexOf("/?page=info") >= 0)
                        { // user requested the info page
                            send_http_header(200, RESPONSE_TYPE_TEXT);
                            sub_page_info();
                        }
                        else if (web_header.indexOf("/?page=") >= 0)
                        { // user requested a page we do not know
                            send_http_header(404, RESPONSE_TYPE_TEXT);
                            sub_page_unknown();
                        }
                        else
                        {
                            deliver_scaffold(bget_password);
                        }
                    } // if (bget_password)

                    web_client.stop();
                }
                else
                { // if you got a newline, then clear currentLine
                    if (web_content_length < 0)
                    {
                        String hdr_name = web_currentLine;
                        hdr_name.toLowerCase();
                        if (hdr_name.startsWith("content-length:"))
                            web_content_length = web_currentLine.substring(15).toInt();
                    }

                    if (web_line_quiet)
                    {
                        // WEB-AUTH-01: keep our "mcs=" pair of the Cookie header in its own bounded buffer
                        // (found anywhere in the line, so foreign cookies of the same host cannot push it
                        // out; a second Cookie header without "mcs=" keeps the first one's value)
                        web_line_quiet = false;
                        int k = 6;
                        while ((k = web_currentLine.indexOf("mcs=", k + 1)) >= 0 &&
                               !(web_currentLine[k - 1] == ':' || web_currentLine[k - 1] == ' ' || web_currentLine[k - 1] == ';'))
                            ;
                        if (k >= 0)
                        {
                            web_cookie_len = 0;
                            for (; k < (int)web_currentLine.length() && web_currentLine[k] != ';' &&
                                   web_cookie_len < sizeof(web_cookie) - 1; k++)
                                web_cookie[web_cookie_len++] = web_currentLine[k];
                            web_cookie[web_cookie_len] = 0;
                        }
                        rm_wipe_string(web_currentLine);
                    }

                    if (web_currentLine.startsWith("GET /rmsend") || web_currentLine.startsWith("GET /?nodepassword=") ||
                        web_currentLine.startsWith("GET /?login="))
                        rm_wipe_string(web_currentLine); // W2-1: never leave a password in a freed String

                    web_currentLine = "";
                }
            }
            else if (c != '\r')
            {
                if (web_currentLine.length() >= WEB_HEADER_MAX)
                {
                    rm_wipe_string(web_currentLine); // a GET /rmsend line may carry the password
                    rm_wipe_string(web_header);
                    web_cors_ok = false;
                    send_http_header(431, RESPONSE_TYPE_JSON);
                    web_client.println("{\"ok\":false,\"err\":\"header line too large\"}");
                    web_client.stop();
                    break;
                }
                // if you got anything else but a carriage return character,
                web_currentLine += c; // add it to the end of the currentLine
            }
        }
    }

    if (bDEBUG)
    {
        Serial.println("Client disconnected.");
        #ifdef ESP
        Serial.printf("%s;[HEAP]Cdis;%d;(free)\n", getTimeString().c_str(), ESP.getFreeHeap());
        Serial.println("");
        #endif
    }

    memset(web_cookie, 0, sizeof(web_cookie));
}

/**
 * ###########################################################################################################################
 * decodes Percent-Coded Strings (e.g. URLs and its parameters)
 */
String decodeURLPercentCoding(String input)
{
    // #1173: full UTF-8 decode instead of a fixed list of escapes -- see url_decode.h
    std::vector<char> buf(input.c_str(), input.c_str() + input.length() + 1);
    url_percent_decode(buf.data(), input.length()); // NUL-terminates, drops %00
    return String(buf.data());
}

#if defined(ESP32)
// AU web GUI: hooks of the shared loadPage() string; empty on the RAK so nRF52 gains no AU bytes.
#define AU_LP_LEAVE "if(typeof auPageLeave=='function')auPageLeave();"
#define AU_LP_INIT "if(page=='info'&&cpage=='info'&&typeof auPageInit=='function')auPageInit();"
#else
#define AU_LP_LEAVE ""
#define AU_LP_INIT ""
#endif

// WEB-LOAD-01: the RAK4631 (W5100S, 4 sockets, some held by the gateway UDP and NTP) refuses part of the parallel
// asset requests a browser makes for the scaffold head; a refused script left the page without mc.js (no theme
// switch, no card toggles, no queue chart). Every asset tag calls mcRe(this) on error: the element is replaced by a
// fresh copy after 250 ms x attempt, at most 8 attempts. mcLate() catches up what ran while the script was missing
// (the clock tick of DOMContentLoaded, the Auto Update row of the info page; auPageInit() starts with auPageLeave(),
// so a second call is harmless). The scripts may arrive in any order, so every late load runs both catch-ups.
// WEBLOAD-JS-BEGIN
static const char WEB_LOAD_JS[] =
    "function mcRe(e){var n=(e.mcN||0)+1;if(n>8)return;var c=e.tagName=='LINK',x=document.createElement(c?'link':'script');x.mcN=n;"
    "x.onerror=function(){mcRe(x);};if(c){x.rel='stylesheet';x.href=e.href;}else{x.src=e.src;x.onload=mcLate;}"
    "setTimeout(function(){if(e.parentNode)e.parentNode.replaceChild(x,e);else document.head.appendChild(x);},250*n);}"
    "function mcLate(){if(typeof mcClockTick=='function')mcClockTick();"
    "if(typeof cpage!='undefined'&&cpage=='info'&&typeof auPageInit=='function')auPageInit();}";
// WEBLOAD-JS-END

/**
 * ###########################################################################################################################
 * delivers the WebUI scaffold including the info page
 */
void deliver_scaffold(bool bget_password)
{
    send_http_header(200, RESPONSE_TYPE_TEXT);

    // Serial.println("Sending Scaffold");
    web_client.println("<!DOCTYPE html>\n<html>\n<head>\n");
    web_client.println("<meta charset=\"utf-8\">\n");
    web_client.println("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n");
    web_client.printf("<title>%s - Meshcom</title>\n",  meshcom_settings.node_call);

    // The static CSS/JS are gzip blobs served as /mc.css, /mc.js, /mcau.js, /mcrm.js (web_assets.h, made by
    // tools/gen_web_assets.py from src/web_functions/web/). The ?v= tag is their content hash. mc.js comes first:
    // it wraps fetch() with the X-MC header. Sub-pages are injected with innerHTML, their <script> never runs,
    // so every script a sub-page needs is loaded here, in the head.
    web_client.print("<script>");
    web_client.print(WEB_LOAD_JS);   // WEB-LOAD-01: before the first asset tag
    web_client.println("</script>");
    web_client.println("<link rel=\"stylesheet\" href=\"/mc.css?v=" WEB_MC_CSS_TAG "\" onerror=\"mcRe(this)\">");
    web_client.println("<script src=\"/mc.js?v=" WEB_MC_JS_TAG "\" onerror=\"mcRe(this)\"></script>");
    // Inline: what depends on the build or on the headers (rmLen, the AU hook in loadPage, the mailbox refresh)
    web_client.println("<script type=\"text/javascript\">\n");
    rmScaffoldJs(); // var rmLen=...: mcrm.js reads it while it loads
#if defined(ENABLE_MSGSTORE)
    web_client.println("function autorefreshMbox(){if(cpage=='mailbox')loadPage('mailbox',csender,false);}setInterval(autorefreshMbox,10000);");
#endif
    // WEB-AUTH-01: login(), logout(), mcLoginFocus() -- challenge-response, must stay after the X-MC wrapper (mc.js above)
    webLoginScaffoldJs();
    // this function is used to load content depending on the navigation button pressed
    web_client.println("function loadPage(page,sender,useSpinner) {" AU_LP_LEAVE "if(typeof rmPageLeave=='function')rmPageLeave();if(typeof pinsLeave=='function')pinsLeave();cpage=page;csender=sender;if(useSpinner){document.getElementById(\"content_layer\").innerHTML=\"<span class=\\\"loader\\\"></span>\"};var xhttp = new XMLHttpRequest(); xhttp.onreadystatechange=function(){if(this.readyState==4 && this.status==200){document.getElementById(\"content_layer\").innerHTML=this.responseText;if(typeof mcLoginFocus=='function')mcLoginFocus();if(typeof mcClockTick=='function')mcClockTick();mcRenderQueue();if(page=='messages' && document.getElementById('messages_panel'))mcRenderHistory();if(page=='remote'&&cpage=='remote')rmPageInit();if(page=='pins'&&cpage=='pins'&&typeof pinsInit=='function')pinsInit();" AU_LP_INIT "if(page=='setup'&&typeof raSt=='function')raSt();if(page=='setup'&&typeof mcLookMark=='function')mcLookMark();}else if(this.readyState==4&&useSpinner)document.getElementById('content_layer').innerHTML='Node did not answer completely. Please reload.';};xhttp.open(\"GET\",\"?page=\"+page,true);xhttp.send();Array.from(document.querySelectorAll('.nav_button.nbactive ')).forEach((el) => el.classList.remove('nbactive')); sender.classList.add('nbactive');}\n");
    web_client.println("</script>\n");
#if defined(ESP32)
    web_client.println("<script src=\"/mcau.js?v=" WEB_MC_AU_JS_TAG "\" onerror=\"mcRe(this)\"></script>"); // AU web GUI: Info page Auto update row
#endif
    web_client.println("<script src=\"/mcrm.js?v=" WEB_MC_RM_JS_TAG "\" onerror=\"mcRe(this)\"></script>\n"); // RM GUI W2b: Remote page scripts

    // scaffold body
    web_client.println("</head>\n<body>\n");
    web_client.println("<div id=\"nav_layer\">\n");
    // MC Logo (Image as base64 and split to several print() to adress limited print buffer
    web_client.print("<svg id=\"mc_logo\" width=\"48px\" height=\"48px\" viewBox=\"0 0 270.93332 270.93332\" xmlns=\"http://www.w3.org/2000/svg\"><g transform=\"translate(21.572702,-6.7150369)\">");
    web_client.print("<path style=\"fill:none;stroke:#ff0000;stroke-width:6.00001;stroke-dasharray:none\" d=\"M 43.312893,84.905251 C 59.073749,118.8832 128.87181,227.98106 128.87181,227.98106 L 136.6499,64.641293 36.353556,");
    web_client.print("184.58753 192.73397,156.95486 Z M 30.622336,75.899048 25.505175,189.50001 124.98278,237.806 206.65266,155.52206 146.06547,48.880441 c 0,0 -75.488902,17.574879 -115.443134,27.018607 z\"/>");
    web_client.print("<path style=\"fill:#000;stroke:#ff0000;stroke-width:6\" d=\"M 55.701471,187.15109 A 29.524067,29.524067 0 0 1 26.177404,216.67516 29.524067,29.524067 0 0 1 -3.3466625,187.15109 29.524067,29.524067 0 0 1 ");
    web_client.print("26.177404,157.62703 29.524067,29.524067 0 0 1 55.701471,187.15109 Z m 98.954169,47.625 a 29.524067,29.524067 0 0 1 -29.52407,29.52407 29.524067,29.524067 0 0 1 -29.524068,-29.52407 29.524067,29.524067 0 0 ");
    web_client.print("1 29.524068,-29.52406 29.524067,29.524067 0 0 1 29.52407,29.52406 z m 78.58126,-80.16874 a 29.524067,29.524067 0 0 1 -29.52407,29.52406 29.524067,29.524067 0 0 1 -29.52407,-29.52406 29.524067,29.524067 0 0 1 ");
    web_client.print("29.52407,-29.52407 29.524067,29.524067 0 0 1 29.52407,29.52407 z M 175.02855,49.303177 A 29.524067,29.524067 0 0 1 145.50449,78.827244 29.524067,29.524067 0 0 1 115.98042,49.303177 29.524067,29.524067 0 0 1 ");
    web_client.print("145.50449,19.77911 29.524067,29.524067 0 0 1 175.02855,49.303177 Z M 60.993137,76.290672 A 29.524067,29.524067 0 0 1 31.46907,105.81474 29.524067,29.524067 0 0 1 1.9450035,76.290672 29.524067,29.524067 0 0 1 ");
    web_client.print("31.46907,46.766605 29.524067,29.524067 0 0 1 60.993137,76.290672 Z\"/><circle style=\"fill:#ff0000;fill-opacity:1;stroke:none;stroke-width:6.35477;stroke-dasharray:none\" id=\"mclogo_circle\" cx=\"108.51074\" ");
    web_client.print("cy=\"139.66409\" r=\"52.916668\"/><text xml:space=\"preserve\" style=\"font-size:63.5px;text-align:start;writing-mode:lr-tb;direction:ltr;text-anchor:start;display:inline;fill:#ffffff;fill-opacity:1;");
    web_client.print("stroke:#ffffff;stroke-width:6.00001;stroke-dasharray:none\" x=\"63.16748\" y=\"161.0486\"><tspan style=\"font-style:normal;font-variant:normal;font-weight:bold;font-stretch:normal;font-size:63px;");
    web_client.println("font-family:sans-serif;stroke:none;stroke-width:6\" x=\"63.16748\" y=\"161.0486\">" MESHCOM_VERSION_SHORT "</tspan></text></g></svg>");
    
    

    web_client.println("<Button class=\"nav_button nbactive\" onclick=\"loadPage('info',this,true)\"><svg viewBox=\"-0.5 0 25 25\" fill=\"none\" xmlns=\"http://www.w3.org/2000/svg\"><g stroke-width=\"0\"></g><g stroke-linecap=\"round\" stroke-linejoin=\"round\"></g><g> <path d=\"M12 21.5C17.1086 21.5 21.25 17.3586 21.25 12.25C21.25 7.14137 17.1086 3 12 3C6.89137 3 2.75 7.14137 2.75 12.25C2.75 17.3586 6.89137 21.5 12 21.5Z\" stroke=\"currentColor\" stroke-width=\"1.5\" stroke-linecap=\"round\" stroke-linejoin=\"round\"></path> <path d=\"M12.9309 8.15005C12.9256 8.39231 12.825 8.62272 12.6509 8.79123C12.4767 8.95974 12.2431 9.05271 12.0008 9.05002C11.8242 9.04413 11.6533 8.98641 11.5093 8.884C11.3652 8.7816 11.2546 8.63903 11.1911 8.47415C11.1275 8.30927 11.1139 8.12932 11.152 7.95675C11.19 7.78419 11.278 7.6267 11.405 7.50381C11.532 7.38093 11.6923 7.29814 11.866 7.26578C12.0397 7.23341 12.2192 7.25289 12.3819 7.32181C12.5446 7.39072 12.6834 7.506 12.781 7.65329C12.8787 7.80057 12.9308 7.97335 12.9309 8.15005ZM11.2909 16.5301V11.1501C11.2882 11.0556 11.3046 10.9615 11.3392 10.8736C11.3738 10.7857 11.4258 10.7057 11.4922 10.6385C11.5585 10.5712 11.6378 10.518 11.7252 10.4822C11.8126 10.4464 11.9064 10.4286 12.0008 10.43C12.094 10.4299 12.1863 10.4487 12.272 10.4853C12.3577 10.5218 12.4352 10.5753 12.4997 10.6426C12.5642 10.7099 12.6143 10.7895 12.6472 10.8767C12.6801 10.9639 12.6949 11.0569 12.6908 11.1501V16.5301C12.6908 16.622 12.6727 16.713 12.6376 16.7979C12.6024 16.8828 12.5508 16.96 12.4858 17.025C12.4208 17.09 12.3437 17.1415 12.2588 17.1767C12.1738 17.2119 12.0828 17.23 11.9909 17.23C11.899 17.23 11.8079 17.2119 11.723 17.1767C11.6381 17.1415 11.5609 17.09 11.4959 17.025C11.4309 16.96 11.3793 16.8828 11.3442 16.7979C11.309 16.713 11.2909 16.622 11.2909 16.5301Z\" fill=\"currentColor\"></path> </g></svg></Button>\n");
    web_client.println("<Button class=\"nav_button\" onclick=\"loadPage('messages',this,true)\"><svg viewBox=\"0 0 24 24\" fill=\"none\" xmlns=\"http://www.w3.org/2000/svg\"><g stroke-width=\"0\"></g><g stroke-linecap=\"round\" stroke-linejoin=\"round\"></g><g> <path d=\"M7 9H17M7 13H17M21 20L17.6757 18.3378C17.4237 18.2118 17.2977 18.1488 17.1656 18.1044C17.0484 18.065 16.9277 18.0365 16.8052 18.0193C16.6672 18 16.5263 18 16.2446 18H6.2C5.07989 18 4.51984 18 4.09202 17.782C3.71569 17.5903 3.40973 17.2843 3.21799 16.908C3 16.4802 3 15.9201 3 14.8V7.2C3 6.07989 3 5.51984 3.21799 5.09202C3.40973 4.71569 3.71569 4.40973 4.09202 4.21799C4.51984 4 5.0799 4 6.2 4H17.8C18.9201 4 19.4802 4 19.908 4.21799C20.2843 4.40973 20.5903 4.71569 20.782 5.09202C21 5.51984 21 6.0799 21 7.2V20Z\" stroke=\"currentColor\" stroke-width=\"2\" stroke-linecap=\"round\" stroke-linejoin=\"round\"></path> </g></svg></Button>\n");
#if defined(ENABLE_MSGSTORE)
    // Stage 3 store node: envelope icon from docs/design/mailbox-page-mockup.html, placed
    // right after Messages. Non-eligible boards never emit this button, so a stray click
    // would 404 anyway, but there is no click to have.
    web_client.println("<Button class=\"nav_button\" onclick=\"loadPage('mailbox',this,true)\"><svg viewBox=\"0 0 24 24\" fill=\"none\" xmlns=\"http://www.w3.org/2000/svg\"><g stroke-width=\"0\"></g><g stroke-linecap=\"round\" stroke-linejoin=\"round\"></g><g> <path d=\"M3 8l9 6 9-6\" stroke=\"currentColor\" stroke-width=\"1.6\" stroke-linecap=\"round\" stroke-linejoin=\"round\"></path> <rect x=\"3\" y=\"5\" width=\"18\" height=\"14\" rx=\"2\" stroke=\"currentColor\" stroke-width=\"1.6\"></rect> </g></svg></Button>\n");
#endif
    web_client.println("<Button class=\"nav_button\" onclick=\"loadPage('wx',this,true)\"><svg version=\"1.1\" xmlns=\"http://www.w3.org/2000/svg\" xmlns:xlink=\"http://www.w3.org/1999/xlink\" viewBox=\"0 0 512 512\" xml:space=\"preserve\" fill=\"#000000\"><g stroke-width=\"0\"></g><g stroke-linecap=\"round\" stroke-linejoin=\"round\"></g><g > <style type=\"text/css\"> .st0{fill:currentColor;} </style> <g> <path class=\"st0\" d=\"M115.958,269.922c16.999-10.12,36.842-15.916,58.04-15.916c2.556,0,5.127,0.078,7.682,0.234 c7.199-24.681,20.957-46.355,39.203-63.12c-3.49-39.437-36.562-70.32-76.879-70.32c-42.647,0-77.207,34.56-77.207,77.199 C66.798,230.766,87.194,258.719,115.958,269.922z\"></path> <rect x=\"135.652\" y=\"54.002\" class=\"st0\" width=\"16.696\" height=\"45.911\"></rect> <polygon class=\"st0\" points=\"102.184,108.88 79.232,69.116 64.772,77.467 87.724,117.232 \"></polygon> <polygon class=\"st0\" points=\"15.114,133.233 54.878,156.185 63.23,141.726 23.466,118.774 \"></polygon> <polygon class=\"st0\" points=\"45.919,189.654 0,189.654 0,206.35 45.919,206.342 \"></polygon> <polygon class=\"st0\" points=\"15.114,262.77 23.466,277.23 63.23,254.27 54.878,239.811 \"></polygon> <rect x=\"240.478\" y=\"114.523\" transform=\"matrix(0.4998 0.8661 -0.8661 0.4998 243.5358 -146.7501)\" class=\"st0\" width=\"16.694\" height=\"45.913\"></rect> <polygon class=\"st0\" points=\"223.228,77.467 208.776,69.116 185.817,108.88 200.269,117.232 \"></polygon> <path class=\"st0\" d=\"M431.997,298c-0.031,0-0.062,0.008-0.101,0.008c0.054-1.332,0.101-2.665,0.101-4.004 C431.997,229.932,380.064,178,316,178c-60.012,0-109.382,45.575-115.388,104.006c-8.414-2.602-17.342-4.005-26.614-4.005 C124.294,278.001,84,318.295,84,368c0,49.704,40.294,89.998,89.998,89.998h257.999c44.182,0,80.003-35.814,80.003-79.995 C512,333.814,476.178,298,431.997,298z\"></path> </g> </g></svg></Button>\n");
    { // Pins page: any switchable pin (board LED, charge LED, MCP23017) or a pin assignment; the page, /pinstatus and /pinassign show what exists
        const LedFitted pinHw = ledFittedNow();
        bool pinAsg = false;
        for (int i = 0; i < (int)PIN_FN_COUNT; i++)
            pinAsg = pinAsg || pinAssignAvail((PinFunc)i);
        if (pinHw.board || pinHw.charge || bMCP23017 || pinAsg || gpioStatusCollect(NULL, 0))
            web_client.println("<Button class=\"nav_button\" title=\"Pins\" aria-label=\"Pins\" onclick=\"loadPage('pins',this,true)\"><svg viewBox=\"0 0 24 24\" fill=\"none\" xmlns=\"http://www.w3.org/2000/svg\"><g stroke=\"currentColor\" stroke-width=\"1.6\" stroke-linecap=\"round\"><circle cx=\"7\" cy=\"7\" r=\"2.5\"></circle><circle cx=\"17\" cy=\"7\" r=\"2.5\" fill=\"currentColor\"></circle><circle cx=\"7\" cy=\"17\" r=\"2.5\" fill=\"currentColor\"></circle><circle cx=\"17\" cy=\"17\" r=\"2.5\"></circle></g></svg></Button>\n");
    }
    web_client.println("<Button class=\"nav_button\" onclick=\"loadPage('position',this,true)\"><svg viewBox=\"0 0 512 512\" xmlns=\"http://www.w3.org/2000/svg\" fill=\"currentColor\" stroke=\"currentColor\"><g stroke-width=\"0\"></g><g istroke-linecap=\"round\" stroke-linejoin=\"round\"></g><g><path fill=\"currentColor\" d=\"M256 17.108c-75.73 0-137.122 61.392-137.122 137.122.055 23.25 6.022 46.107 11.58 56.262L256 494.892l119.982-274.244h-.063c11.27-20.324 17.188-43.18 17.202-66.418C393.122 78.5 331.73 17.108 256 17.108zm0 68.56a68.56 68.56 0 0 1 68.56 68.562A68.56 68.56 0 0 1 256 222.79a68.56 68.56 0 0 1-68.56-68.56A68.56 68.56 0 0 1 256 85.67z\"></path></g></svg></Button>\n");
    web_client.println("<Button class=\"nav_button\" onclick=\"loadPage('remote',this,true)\"><svg viewBox=\"0 0 24 24\" fill=\"none\" xmlns=\"http://www.w3.org/2000/svg\"><g stroke=\"currentColor\" stroke-width=\"1.6\" stroke-linecap=\"round\" stroke-linejoin=\"round\"><circle cx=\"12\" cy=\"11\" r=\"2\"></circle><path d=\"M8.5 7.5a5 5 0 0 0 0 7M15.5 7.5a5 5 0 0 1 0 7M5.5 4.5a9 9 0 0 0 0 13M18.5 4.5a9 9 0 0 1 0 13M12 13v8M9 21h6\"></path></g></svg></Button>\n");   // RM GUI W2b: Remote
    web_client.println("<Button class=\"nav_button\" onclick=\"loadPage('mheard',this,true)\"><svg version=\"1.1\" xmlns=\"http://www.w3.org/2000/svg\" xmlns:xlink=\"http://www.w3.org/1999/xlink\" viewBox=\"0 0 32 32\" xml:space=\"preserve\" fill=\"#000000\"><g stroke-width=\"0\"></g><g stroke-linecap=\"round\" stroke-linejoin=\"round\"></g><g> <style type=\"text/css\"> .linesandangles_een{fill:currentColor;} </style> <path class=\"linesandangles_een\" d=\"M25,13c0,3.348-2.208,7.455-4.286,9.618c-0.527,0.549-0.902,1.188-1.299,1.863 C18.447,26.131,17.35,28,14,28c-3.616,0-5.077-2.068-6.043-3.437c-0.238-0.337-0.464-0.657-0.664-0.856l1.414-1.414 C9.028,22.614,9.301,23,9.59,23.41C10.49,24.683,11.42,26,14,26c2.205,0,2.796-1.007,3.69-2.531 c0.417-0.711,0.891-1.517,1.581-2.236C21.064,19.366,23,15.687,23,13c0-3.86-3.14-7-7-7s-7,3.14-7,7H7c0-4.962,4.038-9,9-9 S25,8.038,25,13z M12,17h-1v2h1c1.206,0,3-0.799,3-3c0-1.639-0.994-2.5-2-2.833v-0.161C13.006,12.503,13.177,10,16,10 s2.994,2.503,3,3.005L20,13h1c0-1.729-1.045-5-5-5s-5,3.271-5,5l0.014,1.975L11.988,15C12.45,15.012,13,15.195,13,16 S12.45,16.988,12,17z\"></path> </g></svg></Button>\n");
    web_client.println("<Button class=\"nav_button\" onclick=\"loadPage('path',this,true)\"><svg viewBox=\"0 0 16 16\" xmlns=\"http://www.w3.org/2000/svg\" fill=\"none\"><g stroke-width=\"0\"></g><g stroke-linecap=\"round\" stroke-linejoin=\"round\"></g><g><path fill=\"currentColor\" fill-rule=\"evenodd\" d=\"M13 0a3 3 0 00-1.65 5.506 7.338 7.338 0 01-.78 1.493c-.22.32-.472.635-.8 1.025a1.509 1.509 0 00-.832.085 12.722 12.722 0 00-1.773-1.124c-.66-.34-1.366-.616-2.215-.871a1.5 1.5 0 10-2.708 1.204c-.9 1.935-1.236 3.607-1.409 5.838a1.5 1.5 0 101.497.095c.162-2.07.464-3.55 1.25-5.253.381-.02.725-.183.979-.435.763.23 1.367.471 1.919.756a11.13 11.13 0 011.536.973 1.5 1.5 0 102.899-.296c.348-.415.64-.779.894-1.148.375-.548.665-1.103.964-1.857A3 3 0 1013 0zm-1.5 3a1.5 1.5 0 113 0 1.5 1.5 0 01-3 0z\" clip-rule=\"evenodd\"></path></g></svg></Button>\n");
    // NBR-W2: Menueknopf fuer die Nachbarschaftsmatrix -- 3x3-Raster als Icon, Konzept 4.5.
    web_client.println("<Button class=\"nav_button\" onclick=\"loadPage('neighbours',this,true)\"><svg viewBox=\"0 0 24 24\" xmlns=\"http://www.w3.org/2000/svg\" fill=\"none\"><g stroke=\"currentColor\" stroke-width=\"2\"><rect x=\"3\" y=\"3\" width=\"5\" height=\"5\"></rect><rect x=\"9.5\" y=\"3\" width=\"5\" height=\"5\"></rect><rect x=\"16\" y=\"3\" width=\"5\" height=\"5\"></rect><rect x=\"3\" y=\"9.5\" width=\"5\" height=\"5\"></rect><rect x=\"9.5\" y=\"9.5\" width=\"5\" height=\"5\"></rect><rect x=\"16\" y=\"9.5\" width=\"5\" height=\"5\"></rect><rect x=\"3\" y=\"16\" width=\"5\" height=\"5\"></rect><rect x=\"9.5\" y=\"16\" width=\"5\" height=\"5\"></rect><rect x=\"16\" y=\"16\" width=\"5\" height=\"5\"></rect></g></svg></Button>\n");
    web_client.println("<Button class=\"nav_button\" onclick=\"loadPage('rxlog',this,true)\"><svg viewBox=\"0 0 32 32\" version=\"1.1\" xmlns=\"http://www.w3.org/2000/svg\" xmlns:xlink=\"http://www.w3.org/1999/xlink\" xmlns:sketch=\"http://www.bohemiancoding.com/sketch/ns\" fill=\"currentColor\"><g stroke-width=\"0\"></g><g stroke-linecap=\"round\" stroke-linejoin=\"round\"></g><g> <title>book-album</title> <desc>Created with Sketch Beta.</desc><defs></defs><g stroke=\"none\" stroke-width=\"1\" fill=\"none\" fill-rule=\"evenodd\" sketch:type=\"MSPage\"> <g sketch:type=\"MSLayerGroup\" transform=\"translate(-412.000000, -99.000000)\" fill=\"currentColor\"> <path d=\"M442,124 C442,125.104 441.073,125.656 440,126 C440,126 434.557,127.515 429,128.977 L429,104 L440,101 C441.104,101 442,101.896 442,103 L442,124 L442,124 Z M427,128.998 C421.538,127.53 416,126 416,126 C414.864,125.688 414,125.104 414,124 L414,103 C414,101.896 414.896,101 416,101 L427,104 L427,128.998 L427,128.998 Z M440,99 C440,99 434.211,100.594 428.95,102 C428.291,102.025 427.627,102 426.967,102 C421.955,100.656 416,99 416,99 C413.791,99 412,100.791 412,103 L412,124 C412,126.209 413.885,127.313 416,128 C416,128 421.393,129.5 426.967,131 L428.992,131 C434.612,129.5 440,128 440,128 C442.053,127.469 444,126.209 444,124 L444,103 C444,100.791 442.209,99 440,99 L440,99 Z\" sketch:type=\"MSShapeGroup\"> </path> </g> </g> </g></svg></Button>\n");
    web_client.println("<Button class=\"nav_button\" onclick=\"loadPage('spectrum',this,true)\"><svg viewBox=\"0 0 24 24\" xmlns=\"http://www.w3.org/2000/svg\"><g><path d=\"M13,11v4M9,7v8m8-6v6\" style=\"fill:none;stroke:currentColor;stroke-linecap:round;stroke-linejoin:round;stroke-width:2;\"></path><path d=\"M3,19H21M5,3V21\" style=\"fill:none;stroke:currentColor;stroke-linecap:round;stroke-linejoin:round;stroke-width:2;\"></path></g></svg></Button>\n");
    if(bMCP23017) {
        web_client.println("<Button class=\"nav_button\" onclick=\"loadPage('mcp23017',this,true)\"><svg width=\"48\" height=\"48\" version=\"1.1\" viewBox=\"0 0 12.7 12.7\" xmlns=\"http://www.w3.org/2000/svg\"><g fill=\"none\" stroke=\"currentColor\" stroke-linejoin=\"round\"><rect x=\".78644\" y=\".78644\" width=\"11.127\" height=\"5.4354\" stroke-width=\"1.0\"/><g stroke-linecap=\"round\" stroke-width=\"1.0\"><path d=\"m0.79071 8.2113h11.119\"/><path d=\"m0.79679 9.7926h11.119\"/><path d=\"m0.79679 11.38h11.119\"/></g></g></svg></Button>\n");
    }

    // Setup Button (Image as base64 and split t0 several print() so RAK can handle it)
    web_client.print("<Button class=\"nav_button\" onclick=\"loadPage('setup',this,true)\"><img src=\"data:image/svg+xml;base64,PHN2ZyB2aWV3Qm94PSIwIDAgMjQgMjQiIGZpbGw9Im5vbmUiIHhtbG5zPSJodHRwOi8vd3d3LnczLm9yZy8yMDAwL3N2ZyI+PGcgc3Ryb2tlLXdpZHRoPSIwIj48L2c+PGcgc3Ryb2tlLWxpbmVjYXA9InJvdW5kIiBzdHJva2UtbGluZWpvaW49InJvdW5kIj48L2c+PGc+PGNpcmNsZSBjeD0iMTIiIGN5PSIxMiIgcj0iMyIgc3Ryb2tlPSIjZmZmZmZmIiBzdHJva2Utd2lkdGg9IjEuNSI+PC9jaXJjbGU+PHBhdGggZD0iTTEzLjc2NTQgMi4xNTIyNEMxMy4zOTc4IDIgMTIuOTMxOSAyIDEyIDJDMTEuMDY4MSAyIDEwLjYwMjIgMiAxMC4yMzQ2IDIuMTUyMjRDOS43NDQ1NyAyLjM1NTIzIDkuMzU1MjIgMi43NDQ1OCA5LjE1MjIzIDMuMjM0NjNDOS4wNTk1NyAzLjQ1ODM0IDkuMDIzMyAzLjcxODUgOS4wMDkxMSA0LjA5Nzk5QzguOTg4MjYgNC42NTU2OCA4LjcwMjI2IDUuMTcxODkgOC4yMTg5NCA1LjQ1MDkzQzcuNzM1NjQgNS43Mjk5NiA3LjE0NTU5IDUuNzE5NTQgNi42NTIxOSA1LjQ1ODc2QzYuMzE2NDUgNS4yODEzIDYuMDczMDEgNS4xODI2MiA1LjgzMjk0IDUuMTUxMDJDNS4zMDcwNCA1LjA4MTc4IDQuNzc1MTggNS4yMjQyOSA0LjM1NDM2IDUuNTQ3MkM0LjAzODc0IDUuNzg5MzggMy44MDU3NyA2LjE5MjkgMy4zMzk4MyA2Ljk5OTkzQzIuODczODkgNy44MDY5NyAyLjY0MDkyIDguMjEwNDggMi41ODg5OSA4LjYwNDkxQz");
    web_client.print("IuNTE5NzYgOS4xMzA4IDIuNjYyMjcgOS42NjI2NiAyLjk4NTE4IDEwLjA4MzVDMy4xMzI1NiAxMC4yNzU2IDMuMzM5NyAxMC40MzcgMy42NjExOSAxMC42MzlDNC4xMzM4IDEwLjkzNiA0LjQzNzg5IDExLjQ0MTkgNC40Mzc4NiAxMkM0LjQzNzgzIDEyLjU1ODEgNC4xMzM3NSAxMy4wNjM5IDMuNjYxMTggMTMuMzYwOEMzLjMzOTY1IDEzLjU2MjkgMy4xMzI0OCAxMy43MjQ0IDIuOTg1MDggMTMuOTE2NUMyLjY2MjE3IDE0LjMzNzMgMi41MTk2NiAxNC44NjkxIDIuNTg4OSAxNS4zOTVDMi42NDA4MiAxNS43ODk0IDIuODczNzkgMTYuMTkzIDMuMzM5NzMgMTdDMy44MDU2OCAxNy44MDcgNC4wMzg2NSAxOC4yMTA2IDQuMzU0MjYgMTguNDUyN0M0Ljc3NTA4IDE4Ljc3NTYgNS4zMDY5NCAxOC45MTgxIDUuODMyODQgMTguODQ4OUM2LjA3Mjg5IDE4LjgxNzMgNi4zMTYzMiAxOC43MTg2IDYuNjUyMDQgMTguNTQxMkM3LjE0NTQ3IDE4LjI4MDQgNy43MzU1NiAxOC4yNyA4LjIxODkgMTguNTQ5QzguNzAyMjQgMTguODI4MSA4Ljk4ODI2IDE5LjM0NDMgOS4wMDkxMSAxOS45MDIxQzkuMDIzMzEgMjAuMjgxNSA5LjA1OTU3IDIwLjU0MTcgOS4xNTIyMyAyMC43NjU0QzkuMzU1MjIgMjEuMjU1NCA5Ljc0NDU3IDIxLjY0NDggMTAuMjM0NiAyMS44NDc4QzEwLjYwMjIgMjIgMTEuMDY4MSAyMiAxMiAyMkMxMi45MzE5IDIyIDEzLjM5NzggMjIgMTMuNzY1NCAyMS44NDc4QzE0LjI1NTQgMjEuNjQ0OCAxNC42NDQ4IDIxLjI1NTQgMTQuODQ3NyAyMC43NjU0QzE0Ljk0MDQgM");
    web_client.print("jAuNTQxNyAxNC45NzY3IDIwLjI4MTUgMTQuOTkwOSAxOS45MDJDMTUuMDExNyAxOS4zNDQzIDE1LjI5NzcgMTguODI4MSAxNS43ODEgMTguNTQ5QzE2LjI2NDMgMTguMjY5OSAxNi44NTQ0IDE4LjI4MDQgMTcuMzQ3OSAxOC41NDEyQzE3LjY4MzYgMTguNzE4NiAxNy45MjcgMTguODE3MiAxOC4xNjcgMTguODQ4OEMxOC42OTI5IDE4LjkxODEgMTkuMjI0OCAxOC43NzU2IDE5LjY0NTYgMTguNDUyN0MxOS45NjEyIDE4LjIxMDUgMjAuMTk0MiAxNy44MDcgMjAuNjYwMSAxNi45OTk5QzIxLjEyNjEgMTYuMTkyOSAyMS4zNTkxIDE1Ljc4OTQgMjEuNDExIDE1LjM5NUMyMS40ODAyIDE0Ljg2OTEgMjEuMzM3NyAxNC4zMzcyIDIxLjAxNDggMTMuOTE2NEMyMC44Njc0IDEzLjcyNDMgMjAuNjYwMiAxMy41NjI4IDIwLjMzODcgMTMuMzYwOEMxOS44NjYyIDEzLjA2MzkgMTkuNTYyMSAxMi41NTggMTkuNTYyMSAxMS45OTk5QzE5LjU2MjEgMTEuNDQxOCAxOS44NjYyIDEwLjkzNjEgMjAuMzM4NyAxMC42MzkyQzIwLjY2MDMgMTAuNDM3MSAyMC44Njc1IDEwLjI3NTcgMjEuMDE0OSAxMC4wODM1QzIxLjMzNzggOS42NjI3MyAyMS40ODAzIDkuMTMwODcgMjEuNDExMSA4LjYwNDk3QzIxLjM1OTIgOC4yMTA1NSAyMS4xMjYyIDcuODA3MDMgMjAuNjYwMiA3QzIwLjE5NDMgNi4xOTI5NyAxOS45NjEzIDUuNzg5NDUgMTkuNjQ1NyA1LjU0NzI3QzE5LjIyNDkgNS4yMjQzNiAxOC42OTMgNS4wODE4NSAxOC4xNjcxIDUuMTUxMDlDMTcuOTI3MSA1LjE4MjY5IDE3LjY4MzcgNS4y");
    web_client.println("ODEzNiAxNy4zNDc5IDUuNDU4OEMxNi44NTQ1IDUuNzE5NTkgMTYuMjY0NCA1LjczMDAyIDE1Ljc4MTEgNS40NTA5NkMxNS4yOTc3IDUuMTcxOTEgMTUuMDExNyA0LjY1NTY2IDE0Ljk5MDkgNC4wOTc5NEMxNC45NzY3IDMuNzE4NDggMTQuOTQwNCAzLjQ1ODMzIDE0Ljg0NzcgMy4yMzQ2M0MxNC42NDQ4IDIuNzQ0NTggMTQuMjU1NCAyLjM1NTIzIDEzLjc2NTQgMi4xNTIyNFoiIHN0cm9rZT0iI2ZmZmZmZiIgc3Ryb2tlLXdpZHRoPSIxLjUiPjwvcGF0aD48L2c+PC9zdmc+\"></Button>\n");

    web_client.println("<Button class=\"nav_button extra_space\" onclick=\"if(confirm('Node will reboot, are you sure?')){callfunction('reboot','');}\"><svg viewBox=\"0 0 16 16\" xmlns=\"http://www.w3.org/2000/svg\"><g stroke-width=\"0\"></g><g stroke-linecap=\"round\" stroke-linejoin=\"round\" stroke=\"currentColor\" stroke-width=\"0.032\"></g><g><path d=\"m 8 0 c -0.550781 0 -1 0.449219 -1 1 v 5 c 0 0.550781 0.449219 1 1 1 s 1 -0.449219 1 -1 v -5 c 0 -0.550781 -0.449219 -1 -1 -1 z m -7 1 l 2.050781 2.050781 c -2.117187 2.117188 -2.652343 5.355469 -1.332031 8.039063 c 1.324219 2.683594 4.214844 4.238281 7.179688 3.851562 c 2.96875 -0.386718 5.367187 -2.625 5.960937 -5.554687 c 0.59375 -2.933594 -0.75 -5.929688 -3.335937 -7.433594 c -0.476563 -0.28125 -1.089844 -0.117187 -1.367188 0.359375 s -0.117188 1.089844 0.359375 1.367188 c 1.851563 1.078124 2.808594 3.207031 2.382813 5.3125 c -0.421876 2.101562 -2.128907 3.691406 -4.253907 3.96875 c -2.128906 0.273437 -4.183593 -0.828126 -5.128906 -2.753907 s -0.566406 -4.226562 0.949219 -5.742187 l 1.535156 1.535156 v -4.003906 c 0 -0.519532 -0.449219 -0.996094 -1 -0.996094 z m 0 0 \" fill=\"currentColor\"></path></g></svg></Button>\n");

    // if the suer has set a password, we deliver a logout button
    if (strlen(meshcom_settings.node_webpwd) > 0)
    {
        web_client.println("<Button class=\"nav_button\" onclick=\"if(confirm('Logging out, are you sure?')){logout()};\"><svg fill=\"currentColor\" viewBox=\"0 0 24 24\" xmlns=\"http://www.w3.org/2000/svg\"><g stroke-width=\"0\"></g><g stroke-linecap=\"round\" stroke-linejoin=\"round\"></g><g><path d=\"M7.707,8.707,5.414,11H17a1,1,0,0,1,0,2H5.414l2.293,2.293a1,1,0,1,1-1.414,1.414l-4-4a1,1,0,0,1,0-1.414l4-4A1,1,0,1,1,7.707,8.707ZM21,1H13a1,1,0,0,0,0,2h7V21H13a1,1,0,0,0,0,2h8a1,1,0,0,0,1-1V2A1,1,0,0,0,21,1Z\"></path></g></svg></Button>\n");
    }
    web_client.printf("</div>\n<div id=\"head_layer\"><p class=\"font-small\">Meshcom %s%s</p><p class=\"font-bold\">%s</p></div>\n</div>\n", SOURCE_VERSION, SOURCE_VERSION_WEB_SUB, meshcom_settings.node_call);
    if(gwRegBlocked())
        web_client.println("<div class=\"mbx-warn\">! node registration failed, please check support</div>");
    web_client.println("<div id=\"content_layer\">\n");

    // initial content
    if (bget_password)
    {
        sub_page_login();
    }
    else
    {
        sub_page_info();
#if defined(ESP32)
        web_client.println("<script type=\"text/javascript\">if(typeof auPageInit=='function')auPageInit();</script>"); // the row is filled from /austatus
#endif
    }

    web_client.println("</div>\n</body>\n</html>");
    web_client.println(); // The HTTP response ends with another blank line
}

/**
 * ###########################################################################################################################
 * delivers a page telling the user that the requested page is not known
 */
void sub_page_unknown()
{
    _create_meshcom_subheader("Oh no...(404)");
    web_client.println("<div id=\"content_inner\">");
    web_client.println("<p>The page you have requested is not known.</p>");
    web_client.println("</div>");
    web_client.println(); // The HTTP response ends with another blank line
}

/**
 * ###########################################################################################################################
 * Increments i by 1 within a modular range.
 */
static int increment_mod(int i, int n) {
    return (i + 1) % n;
}

/**
 * ###########################################################################################################################
 * escapes a string for safe inclusion in HTML output (WEB-03a: mesh-derived strings such as
 * message payloads, callsigns and paths are attacker-controlled and must not reach innerHTML raw)
 */
static String htmlEscape(const String &input)
{
    String out;
    out.reserve(input.length());
    for (unsigned int i = 0; i < input.length(); i++)
    {
        char c = input.charAt(i);
        switch (c)
        {
            case '&':  out += "&amp;";  break;
            case '<':  out += "&lt;";   break;
            case '>':  out += "&gt;";   break;
            case '"':  out += "&quot;"; break;
            case '\'': out += "&#39;";  break;
            default:   out += c;        break;
        }
    }
    return out;
}

// percent-encode a value for use inside a URL query string (RFC 3986 unreserved set stays as is)
static String urlEncode(const char *input)
{
    String out;
    for (; input && *input; input++)
    {
        unsigned char c = (unsigned char)*input;
        if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || c == '-' || c == '.' || c == '_' || c == '~')
            out += (char)c;
        else
        {
            char pct[4];
            snprintf(pct, sizeof(pct), "%%%02X", c);
            out += pct;
        }
    }
    return out;
}

/**
 * ###########################################################################################################################
 * resolves the NTP server the node actually uses (WEB-01): own override if set, else the same
 * default selection udp_functions.cpp:1495-1536 applies when connecting. No clean read-only
 * extern exposes that resolved choice, so the selection is duplicated here (read-only mirror).
 */
static String getEffectiveNtpServer()
{
    // nRF52 STATIC-IP node: the path uses the configured node_ownip and the legacy rule for the
    // stored mode 0 (44er -> Hamnet); gwEffectiveMode keeps this status equal to that path.
    bool staticLegacy = false;
    const char *webNtpIpText = meshcom_settings.node_ip;
#if defined(NRF52_SERIES)
    if (ethIsStaticConfigured())
    {
        staticLegacy = true;
        webNtpIpText = meshcom_settings.node_ownip;
    }
#endif
    IPAddress webNtpIp;
    webNtpIp.fromString(webNtpIpText);
    uint8_t ownIp[4] = {webNtpIp[0], webNtpIp[1], webNtpIp[2], webNtpIp[3]};
    const uint8_t netMode = gwEffectiveMode((uint8_t)meshcom_settings.node_hamnet_only, ownIp, staticLegacy);

    // nRF52 has no DNS resolver: an own NTP host name is ignored there (gwResolve sets ownntp_ignored)
#if defined(NRF52_SERIES)
    const bool hasResolver = false;
#else
    const bool hasResolver = true;
#endif

    GwTarget t;
    if (!gwResolveNode(meshcom_settings.node_gwsrv, netMode, ownIp,
                       meshcom_settings.node_ownntp, hasResolver, &t))
        return String("n/a");

    // own NTP in effect (IPv4 literal, or host name where a resolver exists)
    if (strlen(meshcom_settings.node_ownntp) >= 7 && !t.ownntp_ignored)
    {
        if (t.ntp_host != NULL)
            return String(t.ntp_host);
        char obuf[20];
        snprintf(obuf, sizeof(obuf), "%u.%u.%u.%u", t.ntp_ip[0], t.ntp_ip[1], t.ntp_ip[2], t.ntp_ip[3]);
        return String(obuf);
    }

    String def;
    if (t.ntp_host != NULL)
    {
        def = String(t.ntp_host) + " (default)";
    }
    else
    {
        char buf[32];
        snprintf(buf, sizeof(buf), "%u.%u.%u.%u (default)", t.ntp_ip[0], t.ntp_ip[1], t.ntp_ip[2], t.ntp_ip[3]);
        def = String(buf);
    }
    if (t.ownntp_ignored)
        def += " (own NTP ignored)";
    return def;
}

/**
 * ###########################################################################################################################
 * delivers the rxlog-page to be injected into the scaffold
 */
void sub_page_rxlog()
{
    // Start at the write slot: once the ring has wrapped it holds the oldest
    // line (addRingPointer() keeps the read pointer one slot ahead of it).
    int iRead = RAWLoRaWrite;
    _create_meshcom_subheader("RX Log");

    // WQ-01: LoRa Queue panel. This fragment only carries data-* attributes;
    // mcRenderQueue() (scaffold JS, see deliver_scaffold()) turns them into
    // the bars/text, because a <script> tag injected via innerHTML never
    // runs. stat_last_window_ms == 0 means no 5-min window has completed
    // since boot -- data-win covers that for the JS side.
    uint8_t mcqPrio[6] = {0};
    txRingPrioCounts(mcqPrio);
    uint8_t mcqWait[6] = {0};
    txRingAckWaitCounts(mcqWait);
    uint32_t mcqDedupWin = setlogDedupWindowMin(stat_last_window.newid, PRIO_STAT_INTERVAL_S, MAX_DEDUP_RING);
    uint32_t mcqDrop = 0;
    for (int i = 0; i < 5; i++)
    {
        mcqDrop += stat_last_window.drop[i];
    }
    uint32_t mcqAgeS = 0;
    if (stat_last_window_ms != 0)
    {
        mcqAgeS = (millis() - stat_last_window_ms) / 1000UL;
    }

    // WQ-01: the card lives inside #content_inner so it shares the 4 % left
    // margin of the log lines; fixed 600 px wide (max-width:100% keeps it on
    // a phone screen), it does not scale with the page.
    web_client.println("<div id=\"content_inner\" class=\"logoutput\">");
    web_client.println("<div class=\"cardlayout\" style=\"width:600px;max-width:100%;box-sizing:border-box;\">");
    web_client.println("<label class=\"cardlabel\">LoRa Queue</label>");
    web_client.println("<button id=\"mcqtogglebtn\" class=\"mcq-toggle\" onclick=\"mcQueueToggle(this)\">hide</button>");
    web_client.printf("<div id=\"mcq\" data-ring=\"%u\" data-p0=\"%u\" data-p1=\"%u\" data-p2=\"%u\" data-p3=\"%u\"\n",
                       (unsigned int)MAX_RING, (unsigned int)mcqPrio[0], (unsigned int)mcqPrio[1], (unsigned int)mcqPrio[2], (unsigned int)mcqPrio[3]);
    web_client.printf(" data-p4=\"%u\" data-p5=\"%u\" data-bp=\"%d\" data-bpname=\"%s\" data-qrs=\"%d\" data-qrsf=\"%d\" data-qrt=\"%d\"\n",
                       (unsigned int)mcqPrio[4], (unsigned int)mcqPrio[5], bpCurrentState(), bpStateName(), bpQrsThreshold(), bpQrsForecast((int)mcqPrio[0]), bpRefuseThreshold());
    web_client.printf(" data-win=\"%d\" data-rx=\"%lu\" data-tx=\"%lu\" data-int=\"%d\" data-newid=\"%lu\"\n",
                       (stat_last_window_ms != 0) ? 1 : 0, (unsigned long)stat_last_window.rx_ms, (unsigned long)stat_last_window.tx_ms,
                       (int)PRIO_STAT_INTERVAL_S, (unsigned long)stat_last_window.newid);
    web_client.printf(" data-dup=\"%lu\" data-dedup=\"%u\" data-dwin=\"%lu\" data-age=\"%lu\" data-drop=\"%lu\"\n",
                       (unsigned long)stat_last_window.dup, (unsigned int)MAX_DEDUP_RING, (unsigned long)mcqDedupWin, (unsigned long)mcqAgeS, (unsigned long)mcqDrop);
    web_client.printf(" data-w0=\"%u\" data-w1=\"%u\" data-w2=\"%u\" data-w3=\"%u\" data-w4=\"%u\" data-w5=\"%u\"></div>\n",
                       (unsigned int)mcqWait[0], (unsigned int)mcqWait[1], (unsigned int)mcqWait[2], (unsigned int)mcqWait[3], (unsigned int)mcqWait[4], (unsigned int)mcqWait[5]);
    web_client.println("</div>");

    web_client.println("<div style=\"overflow:scroll;\">");

    // R1-04: DIES ist die Stelle, die den Puffer ueberhaupt erst anlegt -- er
    // hat keinen anderen Leser. Der erste Aufruf dieser Seite kostet die
    // Zuteilung, ab dann fuellt der RX-Pfad ihn. Schlaegt sie fehl, bleibt die
    // Seite bedienbar und sagt warum, statt auf einem NULL-Zeiger zu landen.
    if(!rawLogEnsure())
    {
        web_client.println("<p class=\"no-wrap\">RX log buffer not available (out of memory)</p>");
    }
    else
    {
        do
        {
            // WQ-01: normal text size (was font-small) -- the page uses three sizes only:
            // title, normal (log lines, panel text), small (legend, notes, tick labels).
            // Empty slots (calloc'd, never written) are skipped.
            if (ringbufferRAWLoraRX[iRead][0] != 0x00)
            {
                // over-the-air text: escape. print() pieces, not printf: the escaped line can
                // reach several hundred bytes and Print::printf mallocs above 64 B.
                web_client.printf("<p class=\"no-wrap\"><%i>", iRead);
                web_client.print(htmlEscape(String((const char *)ringbufferRAWLoraRX[iRead])));
                web_client.print("</p>\n");
            }
            iRead = increment_mod(iRead, MAX_LOG);
        } while (RAWLoRaWrite != iRead);
    }
    web_client.println("</div></div>");
    web_client.println(); // The HTTP response ends with another blank line
}

/**
 * ###########################################################################################################################
 * delivers the wx-page to be injected into the scaffold
 */
void sub_page_wx()
{
    bool bBMX = (bBMEON || bBMPON);
    char cbme[10] = {0};
    if (bBMX)
        snprintf(cbme, sizeof(cbme), " (%s)", (bmx_found ? "found" : "error"));

    char c680[10] = {0};
    if (bBME680ON)
        snprintf(c680, sizeof(c680), " (%s)", (bme680_found ? "found" : "error"));

    char c811[10] = {0};
    if (bMCU811ON)
        snprintf(c811, sizeof(c811), " (%s)", (mcu811_found ? "found" : "error"));

    _create_meshcom_subheader("WX Information");
    web_client.println("<div id=\"content_inner\">");
    web_client.println("<table class=\"table mw-600\">");
    web_client.println("<thead><tr class=\"font-bold\"><td>Item</td><td>Value</td></tr></thead>");
    web_client.printf("<tr><td>BME(P)280</td><td>%s %s</td></tr>\n", (bBMX ? "on" : "off"), cbme);
    web_client.printf("<tr><td>BME680</td><td>%s %s</td></tr>\n", (bBME680ON ? "on" : "off"), c680);
    web_client.printf("<tr><td>MCU811</td><td>%s %s</td></tr>\n", (bMCU811ON ? "on" : "off"), c811);
    web_client.printf("<tr><td>LPS33</td><td>%s (RAK)</td></tr>\n", (bLPS33 ? "on" : "off"));
    web_client.printf("<tr><td>1-Wire</td><td>%s (GPIO %i, DHT%i)</td></tr>\n", (bONEWIRE ? "on" : "off"), meshcom_settings.node_owgpio, meshcom_settings.node_dht_type); // SENS-05(c): DHT11 or DHT22 (--dht)
    web_client.printf("<tr><td>Temperature</td><td>%.1f &deg;C</td></tr>\n", meshcom_settings.node_temp);
    web_client.printf("<tr><td>Tout</td><td>%.1f &deg;C</td></tr>\n", meshcom_settings.node_temp2);
    web_client.printf("<tr><td>Humidity</td><td>%.1f%% rH</td></tr>\n", meshcom_settings.node_hum);
    web_client.printf("<tr><td>QFE</td><td>%.1f hPa</td></tr>\n", meshcom_settings.node_press);
    web_client.printf("<tr><td>QNH</td><td>%.1f hPa</td></tr>\n", meshcom_settings.node_press_asl);
    web_client.printf("<tr><td>Altitude asl</td><td>%i m</td></tr>\n", meshcom_settings.node_press_alt);
    web_client.printf("<tr><td>Gas</td><td>%.1f kOhm</td></tr>\n", meshcom_settings.node_gas_res);
    web_client.printf("<tr><td>eCO2</td><td>%.1f ppm</td></tr>\n", meshcom_settings.node_co2);
    #if defined(NTC_PIN) && defined(FAN_CTRL) // BOARD_TBEAM_1W
    web_client.printf("<tr><td>NTC / FAN %s</td><td>%.1f &deg;C</td></tr>\n", digitalRead(FAN_CTRL) ? "on" : "off", meshcom_settings.node_ntctemp);
    #endif
    // web_client.printf("<tr><td></td><td></td></tr>\n", );
    // web_client.printf("<tr><td></td><td></td></tr>\n", );
    web_client.println("</table></div>");
    web_client.println(); // The HTTP response ends with another blank line
}

/**
 * ###########################################################################################################################
 * delivers the position-page to be injected into the scaffold
 */
void sub_page_position()
{
    _create_meshcom_subheader("Position Information");
    web_client.println("<div id=\"content_inner\">");
    web_client.println("<table class=\"table mw-600\">");
    web_client.println("<thead><tr class=\"font-bold\"><td>Item</td><td>Value</td></tr></thead>");
    web_client.printf("<tr><td>Latitude</td><td>%.4lf %c</td></tr>\n", meshcom_settings.node_lat, meshcom_settings.node_lat_c);
    web_client.printf("<tr><td>Longitude</td><td>%.4lf %c</td></tr>\n", meshcom_settings.node_lon, meshcom_settings.node_lon_c);
    web_client.printf("<tr><td>Altitude</td><td>%i</td></tr>\n", meshcom_settings.node_alt);
    web_client.printf("<tr><td>Satellites</td><td>%i - %s - HDOP %i</td></tr>\n", (int)posinfo_satcount, (posinfo_fix ? "fix" : "nofix"), (int)fposinfo_hdop);
    web_client.printf("<tr><td>Rate</td><td>%i</td></tr>\n", (int)posinfo_interval);
    web_client.printf("<tr><td>Next</td><td>%i sec</td></tr>\n", posinfoNextSeconds(millis(), (uint32_t)posinfo_timer, (uint32_t)posinfo_interval));
    web_client.printf("<tr><td>Distance</td><td>%.0lf m</td></tr>\n", posinfo_distance);
    web_client.printf("<tr><td>Dir (current)</td><td>%i&deg;</td></tr>\n", (int)posinfo_direction);
    web_client.printf("<tr><td>Dir (last)</td><td>%i&deg;</td></tr>\n", (int)posinfo_last_direction);
    web_client.printf("<tr><td>Date</td><td>%i.%02i.%02i %02i:%02i:%02i %s</td></tr>\n", meshcom_settings.node_date_year, meshcom_settings.node_date_month, meshcom_settings.node_date_day, meshcom_settings.node_date_hour, meshcom_settings.node_date_minute, meshcom_settings.node_date_second, getTimeZone().c_str());
    web_client.printf("<tr><td>Symbol</td><td>%c %c</td></tr>\n", meshcom_settings.node_symid, meshcom_settings.node_symcd);
    web_client.printf("<tr><td>GPS</td><td>%s</td></tr>\n", (bGPSON ? "on" : "off"));
    web_client.printf("<tr><td>Track</td><td>%s</td></tr>\n", (bDisplayTrack ? "on" : "off"));
    web_client.printf("<tr><td></td><td><button onclick=\"callfunction('sendpos','')\"><i class=\"btnrefresh\"></i>Send Position</button></td></tr>\n");
    // web_client.printf("<tr><td></td><td></td></tr>\n", );
    // web_client.printf("<tr><td></td><td></td></tr>\n", );
    web_client.println("</table></div>");
    web_client.println(); // The HTTP response ends with another blank line
}

// Firmware-Stand als "v4.40a", "-" wenn unbekannt. Auf der Luft stehen nur
// die Stellen hinter "4." (msg_source_fw_version, 40 = 4.40) und der
// Buchstabe (msg_source_fw_sub_version). ver/sub kommen aus DERSELBEN
// Momentaufnahme wie das Rufzeichen der Zeile (NbrRowView/NbrMhView): ein
// zweites nbrRowGet() koennte auf nRF52 (OnRxDone im LORA-Task) schon eine
// neu vergebene Zeile lesen. Eigene Zeile: shortVERSION()/shortSUBVERSION().
static void webPrintFw(uint8_t ver, char sub)
{
    if (ver == 0)
        web_client.print("-");
    else if (sub >= 'a' && sub <= 'z')
        web_client.printf("v4.%02u%c", (unsigned)ver, sub);
    else
        web_client.printf("v4.%02u", (unsigned)ver);
}

/**
 * ###########################################################################################################################
 * delivers the mheard-page to be injected into the scaffold
 *
 * W4b (docs/meshcom5-campaign.md Welle 4, Konzept 4.6/4.9 in
 * docs/meshcom5-topologie/body/04-ansichten.html): liest nur noch ueber
 * src/nbr_views.h -- mheard_functions.* ist mit dieser Welle weg. Die
 * Zeilenauswahl kommt aus nbrMhRows() (schon neueste zuerst, 3h-Fenster wie
 * bisher), Detailwerte je Zeile aus nbrMhGet(). Ein Heap-Block fuer die
 * Zeilenindizes statt eines Stack-Arrays, dieselbe Begruendung wie
 * sub_page_neighbours() unten (Seite wird selten geoeffnet).
 */
void sub_page_mheard()
{
    // Konzept 4.6: dieselbe Minuten-seit-Boot-Uhr wie der OnRxDone-Haken und
    // die Neighbours-Seite (nbrFresh() etc.) -- alle Leser rechnen gegen
    // dieselbe Uhr.
    uint16_t now_min = uptimeMin16();

    // Konzept 4.6, Regel 2: ohne gueltige Uhr (Jahr < 2025, wie bisher
    // isWallClockValid() in mheard_functions.cpp) bleibt die Topologie
    // gueltig -- DATE/TIME zeigt dann nur das Alter in Minuten.
    bool bClockValid = (meshcom_settings.node_date_year >= 2025);
    unsigned long nowEpoch = bClockValid ? getUnixClock() : 0;

    _create_meshcom_subheader("MHeard Information");
    web_client.println("<div id=\"content_inner\">");
    // W4d: Legende fuer die zweite Kaertchenzeile (title= wirkt auf dem
    // Telefon nicht, deshalb als Text).
    web_client.print("<p style=\"font-size:var(--fs-s);color:var(--mcmute);max-width:900px;\">"
                     "Stations I heard directly in the last 3 h, newest first. "
                     "Last heard: minutes since I last received it directly. "
                     "Hears me: its own reports say it hears me, at this SNR. "
                     "It hears: stations it hears, as far as my table knows. "
                     "Only it hears: of those, how many no other direct neighbour of mine hears -- its value as a relay for me. "
                     "Relay role: Super node = by far the most exclusive stations; Needed = at least one exclusive station; Redundant = everything it hears is heard by others too. "
                     "It reports: the neighbour count the station announces itself (NCNT). "
                     "Firmware: the version the station puts into its own frames, '-' until one arrives.</p>");

    uint8_t *idx = (uint8_t *)malloc((size_t)NBR_MAX_ROWS);
    if (idx == NULL)
    {
        web_client.println("<p>Not enough memory to render MHeard.</p>");
        web_client.println("</div>");
        web_client.println(); // The HTTP response ends with another blank line
        return;
    }

    int total = nbrMhRows(nbrMatrix, now_min, 180, idx, NBR_MAX_ROWS); // 3h Fenster wie bisher
    int shown = (total < NBR_MAX_ROWS) ? total : NBR_MAX_ROWS;

    for (int k = 0; k < shown; k++)
    {
        meshcom_wdt_feed(); // one feed per card; shown <= NBR_MAX_ROWS keeps the loop bounded
        NbrMhView v;
        if (!nbrMhGet(nbrMatrix, idx[k], now_min, &v))
            continue;

        web_client.printf("<div class=\"cardlayout\" style=\"max-width:900px;\">\n");
        web_client.printf("<label class=\"cardlabel\"><a href=\"https://aprs.fi/?call=%s\" target=\"_blank\">%s</a> <span class=\"font-small\">(", urlEncode(v.call).c_str(), htmlEscape(String(v.call)).c_str());
        if (bClockValid)
        {
            // Sekunde aus dem Slot statt der Zeilenminute (Konzept 4.6);
            // dieselbe utcoff-Umrechnung wie frueher fuer die Pfadseite.
            unsigned long base = nowEpoch - (nowEpoch % 60UL) - (unsigned long)v.age_min * 60UL;
            if (v.sec < 60)
                base += v.sec;
            base += (unsigned long)(long)(meshcom_settings.node_utcoff * 3600.0);
            web_client.print(convertUNIXtoString(base));
        }
        else
        {
            web_client.printf("%u min ago", (unsigned)v.age_min);
        }
        web_client.printf(")</span></label>");
        web_client.printf("<div class=\"flex-auto-wrap\">");
        {
            const char *ptype = nbrPayloadTypeName(v.plt);
            if (v.plt == ':')
                ptype = "Text message";
            else if (v.plt == '!')
                ptype = "Position";
            else if (v.plt == '@')
                ptype = "Heartbeat (HEY)";
            web_client.printf("<div><span class=\"font-bold\">Last frame:</span><br><span>%s</span></div>", ptype);
        }
        web_client.printf("<div><span class=\"font-bold\">Hardware:</span><br><span>%s</span></div>", nbrHardwareName(v.hw));
        web_client.print("<div><span class=\"font-bold\">Firmware:</span><br><span>");
        webPrintFw(v.fw_ver, v.fw_sub);
        web_client.print("</span></div>");
        web_client.printf("<div><span class=\"font-bold\">Country / mode:</span><br><span title=\"high nibble: country index, low nibble: LoRa modulation\">%01X / %01X</span></div>", (v.mod >> 4), (v.mod & 0x0f));
        if (v.rssi == NBR_MH_RSSI_UNKNOWN)
            web_client.printf("<div><span class=\"font-bold\">RSSI:</span><br><span></span></div>");
        else
            web_client.printf("<div><span class=\"font-bold\">RSSI:</span><br><span>%d dBm</span></div>", (int)v.rssi);
        if (v.snr == NBR_SNR_UNKNOWN)
            web_client.printf("<div><span class=\"font-bold\">SNR (avg):</span><br><span>-</span></div>");
        else
            web_client.printf("<div><span class=\"font-bold\">SNR (avg):</span><br><span>%d dB</span></div>", (int)v.snr);

        // DIST weiterhin aus der eigenen Position gerechnet, 0/0 = unbekannt
        // (dieselbe Regel wie src/mh_phone.h fuer den App-Rahmen).
        double dist = -1.0;
        if (nbrPosKnown(v.lat, v.lon) && !(meshcom_settings.node_lat == 0.0 && meshcom_settings.node_lon == 0.0))
            dist = gps.distanceBetween(v.lat, v.lon, nodeSignedLat(meshcom_settings.node_lat, meshcom_settings.node_lat_c), nodeSignedLon(meshcom_settings.node_lon, meshcom_settings.node_lon_c)) / 1000.0;
        if (dist >= 0.0)
            web_client.printf("<div><span class=\"font-bold\">Distance:</span><br><span>%.1lf km</span></div>", dist);
        else
            web_client.printf("<div><span class=\"font-bold\">Distance:</span><br><span>-</span></div>");

        // NCNT (Konzept 4.8) steht jetzt in der zweiten Zeile, bei den Nachbarschaftswerten.

        if (nbrPosKnown(v.lat, v.lon))
        {
            dlat = v.lat;
            clat = 'N';
            if (dlat < 0)
            {
                dlat = dlat * (-1);
                clat = 'S';
            }
            dlon = v.lon;
            clon = 'E';
            if (dlon < 0)
            {
                dlon = dlon * (-1);
                clon = 'W';
            }
            web_client.printf("<div><span class=\"font-bold\">Lat:</span><br><span>%c%06.3lf</span></div>", clat, dlat);
            web_client.printf("<div><span class=\"font-bold\">Lon:</span><br><span>%c%07.3lf</span></div>", clon, dlon);
        }
        else
        {
            web_client.printf("<div><span class=\"font-bold\">Lat:</span><br><span></span></div>");
            web_client.printf("<div><span class=\"font-bold\">Lon:</span><br><span></span></div>");
        }
        if (v.alt != NBR_MH_ALT_UNKNOWN)
            web_client.printf("<div><span class=\"font-bold\">Altitude:</span><br><span>%d m</span></div>", (int)v.alt);
        else
            web_client.printf("<div><span class=\"font-bold\">Altitude:</span><br><span>-</span></div>");

        // W4b, Konzept 4.9: die Nachbarschaftswerte -- W4d: eigene zweite
        // Zeile im Kaertchen, ausgeschriebene Namen statt AGE/HM/#X/#N/R,
        // Legende oben auf der Seite.
        web_client.printf("</div><div class=\"flex-auto-wrap\" style=\"margin-top:8px;\">");
        web_client.printf("<div><span class=\"font-bold\">Last heard:</span><br><span>%u min ago</span></div>", (unsigned)v.age_min);
        if (v.hm_snr == NBR_SNR_UNKNOWN)
            web_client.printf("<div><span class=\"font-bold\">Hears me:</span><br><span>not reported</span></div>");
        else
            web_client.printf("<div><span class=\"font-bold\">Hears me:</span><br><span>yes, %d dB</span></div>", (int)v.hm_snr);
        const char *role = "-";
        if (v.role == 'S')
            role = "Super node";
        else if (v.role == 'N')
            role = "Needed";
        else if (v.role == 'R')
            role = "Redundant";
        web_client.printf("<div><span class=\"font-bold\">Relay role:</span><br><span>%s</span></div>", role);
        web_client.printf("<div><span class=\"font-bold\">Only it hears:</span><br><span>%u station%s</span></div>", (unsigned)v.ex, v.ex == 1 ? "" : "s");
        web_client.printf("<div><span class=\"font-bold\">It hears:</span><br><span>%u station%s</span></div>", (unsigned)v.nb, v.nb == 1 ? "" : "s");
        web_client.printf("<div><span class=\"font-bold\">It reports:</span><br><span>%u neighbour%s</span></div>", (unsigned)v.ncnt, v.ncnt == 1 ? "" : "s");
        web_client.printf("<div><span class=\"font-bold\">Gateway:</span><br><span>%s</span></div>", v.gw ? "yes" : "no");
        web_client.printf("</div></div>");
    }
    free(idx);

    if (shown == 0)
        web_client.println("<p>No Nodes heard so far.</p>"); // no nodes available? Tell the user
    web_client.println("</div>");
    web_client.println(); // The HTTP response ends with another blank line
}

// W2c CONTRACT: jede Zeile ausserhalb von nbr_matrix.cpp liest nur ueber die
// oeffentliche API (nbrRowGet()/nbrEdgeGet()/die Maskenfunktionen aus
// nbr_matrix.h) -- die Zeilen- und Kantenfelder von NbrMatrix sind seit dem
// Kantenpool-Umbau (Welle 2) nicht mehr oeffentlich, und NBR_MAX_ROWS ist
// jetzt 64 (klassischer ESP32) bzw. 128 (S3, nRF52) statt vormals <= 21.
// nbrRowGet() ist ein O(1)-Zeilenzugriff (NbrMatrix.row[] ist ein Array, kein
// Pool) -- es GIBT bewusst kein Zeilen-Array als Zwischenspeicher mehr
// (Orchestrator-Review 2026-09-25: eine NbrRowView[NBR_MAX_ROWS]-Kopie kostet
// ~3.6 kB BSS auf S3/nRF52, fast die ganze Kampagnen-Ersparnis). Ein
// Rufzeichen wird deshalb hier geholt, jedes Mal wenn es gebraucht wird --
// nbrRowGet() nimmt die Scheduler-Klammer auf nRF52 selbst.
static void nbrPrintCall(uint8_t row)
{
    NbrRowView v;
    if (nbrRowGet(nbrMatrix, row, &v))
        web_client.print(htmlEscape(String(v.call)));
    else
        web_client.print("?");
}

/**
 * ###########################################################################################################################
 * delivers the path-page to be injected into the scaffold
 *
 * W4c (docs/meshcom5-topologie/, Zusammenlegung der alten Zeilentabelle der
 * Neighbours-Seite in diese Seite): EINE Tabelle, eine Zeile je bekannter
 * Station, sortiert nach Hops aufsteigend, dann Alter aufsteigend -- Hops 0
 * ist die eigene Zeile (immer gezeigt, solange die Topologie bereit ist),
 * Hops 1 die frischen Direktnachbarn (nbrDirectMask()), Hops 2 und mehr jeder
 * nbrRouteCount()/nbrRouteGet()-Eintrag (2-Hop-Zeile ODER Horizont). Die
 * Spalten Hears me/Covered by (Hops 1) und GW/Mesh/Hears me/Reach (2-Hop-
 * Zeile, ueber das neue NbrRouteView::row) sind aus der alten Zeilentabelle
 * von sub_page_neighbours() uebernommen -- Via/Covered by bei Hops >= 2 ist
 * die unveraenderte Via-Logik der alten Pfadseite (entry-Maske, Horizont
 * A&gt;B-Form).
 *
 * Sortierung: ein einziger transienter Heap-Block (dieselbe Begruendung wie
 * `scratch` in sub_page_neighbours() oben) haelt nur Art+Index je Zeile, nie
 * eine Kopie von NbrRowView/NbrRouteView (W2c CONTRACT oben). nbrRouteGet()
 * ist O(i) -- darum genau ein Aufruf je Weg-Eintrag im Sammeldurchlauf (fuer
 * Hops/Alter) und genau einer im Renderdurchlauf, keiner davon in der
 * Sortierschleife selbst (einfaches Einfuegesortieren).
 */
void sub_page_path()
{
    uint16_t now_min = uptimeMin16();

    _create_meshcom_subheader("Path Information");
    web_client.println("<div id=\"content_inner\">");

    NbrRowView r0;
    if (!nbrRowGet(nbrMatrix, 0, &r0) || r0.call[0] == 0)
    { // noch kein Frame ausgewertet -- Zeile 0 ist unbelegt (wie sub_page_neighbours())
        web_client.println("<p>No frames received yet.</p>");
        web_client.println("</div>");
        web_client.println(); // The HTTP response ends with another blank line
        return;
    }

    NbrMask directMask = nbrDirectMask(nbrMatrix, now_min);
    int route_total = nbrRouteCount(nbrMatrix, now_min);

    // Datensatz fuer die Sortierung: nur Art (0 eigene Zeile, 1 direkt, 2 Weg)
    // und Index (Matrixzeile bei 0/1, laufender Weg-Index bei 2), dazu die
    // beiden Sortierschluessel. Obergrenze: jede Matrixzeile hoechstens
    // einmal (eigene Zeile ODER direkt ODER 2-Hop-Zeile, nie mehrfach) plus
    // der Horizont -- NBR_MAX_ROWS + NBR_HZ_ENTRIES + 1 deckt das mit Reserve.
    struct PathRec
    {
        uint8_t  kind; // 0 eigene Zeile, 1 direkt, 2 Weg (2-Hop-Zeile oder Horizont)
        uint8_t  hops;
        uint16_t age;
        uint16_t idx;
    };
    size_t maxrec = (size_t)NBR_MAX_ROWS + (size_t)NBR_HZ_ENTRIES + 1;
    PathRec *rec = (PathRec *)malloc(maxrec * sizeof(PathRec));
    if (rec == NULL)
    {
        web_client.println("<p>Not enough memory to render the path table.</p>");
        web_client.println("</div>");
        web_client.println(); // The HTTP response ends with another blank line
        return;
    }
    size_t nrec = 0;

    rec[nrec].kind = 0;
    rec[nrec].hops = 0;
    rec[nrec].age = nbrRowAgeMin(nbrMatrix, 0, now_min);
    rec[nrec].idx = 0;
    nrec++;

    for (int X = nbrMaskNext(directMask, -1); X >= 0 && nrec < maxrec; X = nbrMaskNext(directMask, X))
    {
        rec[nrec].kind = 1;
        rec[nrec].hops = 1;
        rec[nrec].age = nbrRowAgeMin(nbrMatrix, (uint8_t)X, now_min);
        rec[nrec].idx = (uint16_t)X;
        nrec++;
    }

    for (int i = 0; i < route_total && nrec < maxrec; i++)
    {
        NbrRouteView r;
        if (!nbrRouteGet(nbrMatrix, i, now_min, &r))
            continue;
        rec[nrec].kind = 2;
        rec[nrec].hops = r.hops;
        rec[nrec].age = r.age_min;
        rec[nrec].idx = (uint16_t)i;
        nrec++;
    }
    meshcom_wdt_feed(); // once after collecting routes; loop bounded by route_total and maxrec

    // Einfuegesortieren nach Hops, dann Alter (beide aufsteigend) -- rec[0]
    // (eigene Zeile, Hops 0) bleibt dabei an erster Stelle.
    for (size_t a = 1; a < nrec; a++)
    {
        PathRec key = rec[a];
        long b = (long)a - 1;
        while (b >= 0 && (rec[b].hops > key.hops || (rec[b].hops == key.hops && rec[b].age > key.age)))
        {
            rec[b + 1] = rec[b];
            b--;
        }
        rec[b + 1] = key;
    }
    meshcom_wdt_feed(); // once after the sort; nrec <= maxrec

    // Tabelle DIREKT unter #content_inner, keine Wrapper-div: das Tabellen-
    // CSS des Scaffolds greift nur auf "#content_inner > table" (siehe
    // sub_page_neighbours() unten, Bench 2026-09-20).
    web_client.println("<table class=\"table\">");
    web_client.println("<thead><tr class=\"font-bold\"><td>Call</td><td>Hops</td><td>GW</td><td>Mesh</td>"
                        "<td>Firmware</td><td>Hears me</td><td>Via / Covered by</td><td>Reach</td><td>Age (min)</td></tr></thead>");

    for (size_t k = 0; k < nrec; k++)
    {
        meshcom_wdt_feed(); // one feed per record; nrec <= maxrec keeps the loop bounded
        PathRec &pr = rec[k];

        if (pr.kind == 0)
        { // eigene Zeile: dieselben Spaltenregeln wie eine direkte Zeile, ohne Hears me/Via
            web_client.printf("<tr><td><a href=\"https://aprs.fi/?call=%s\" target=\"_blank\">%s</a></td>", urlEncode(r0.call).c_str(), htmlEscape(String(r0.call)).c_str());
            web_client.print("<td>0</td>");
            web_client.printf("<td>%s</td>", (r0.flags & NBR_FLAG_GW) ? "Y" : "N");
            if (!(r0.flags & NBR_FLAG_POS))
                web_client.print("<td>-</td>");
            else
                web_client.printf("<td>%s</td>", (r0.flags & NBR_FLAG_MESH) ? "Y" : "N");
            web_client.print("<td>");
            webPrintFw(shortVERSION(), shortSUBVERSION());
            web_client.print("</td>");
            web_client.print("<td>-</td>"); // Hears me: keine Kante nach mir selbst
            web_client.print("<td>-</td>"); // Via / Covered by: nicht anwendbar auf die eigene Zeile

            int partner = -1;
            float reach = nbrReach(nbrMatrix, 0, now_min, &partner);
            if (reach >= 0 && partner >= 0)
            {
                web_client.printf("<td>%.1f km @ ", (double)reach);
                nbrPrintCall((uint8_t)partner);
                web_client.print("</td>");
            }
            else
                web_client.print("<td>-</td>");

            web_client.printf("<td>%u</td></tr>\n", (unsigned)pr.age);
            continue;
        }

        if (pr.kind == 1)
        { // direkte Zeile
            uint8_t X = (uint8_t)pr.idx;
            NbrRowView v;
            if (!nbrRowGet(nbrMatrix, X, &v))
                continue; // Zeile ist zwischen Sammel- und Renderdurchlauf verschwunden (EVICT)

            web_client.printf("<tr><td><a href=\"https://aprs.fi/?call=%s\" target=\"_blank\">%s</a></td>", urlEncode(v.call).c_str(), htmlEscape(String(v.call)).c_str());
            web_client.print("<td>1</td>");
            web_client.printf("<td>%s</td>", (v.flags & NBR_FLAG_GW) ? "Y" : "N");
            if (!(v.flags & NBR_FLAG_POS))
                web_client.print("<td>-</td>");
            else
                web_client.printf("<td>%s</td>", (v.flags & NBR_FLAG_MESH) ? "Y" : "N");
            web_client.print("<td>");
            webPrintFw(v.fw_ver, v.fw_sub);
            web_client.print("</td>");

            NbrEdgeView hm;
            if (nbrEdgeGet(nbrMatrix, 0, X, &hm) && hm.snr != NBR_SNR_UNKNOWN && nbrFresh(hm.last_min, now_min))
                web_client.printf("<td>%d</td>", (int)hm.snr);
            else
                web_client.print("<td>-</td>");

            // Covered by (wie 6.2, Regel 3 der alten Zeilentabelle): direkte
            // Nachbarn, die X ebenfalls frisch hoeren -- leer ist E_self,
            // "nur ich erreiche diesen Knoten".
            web_client.print("<td style=\"max-width:220px;overflow-wrap:anywhere;\">");
            {
                NbrMask covered = nbrMaskAnd(nbrHearersMask(nbrMatrix, X, now_min), directMask);
                bool first = true;
                for (int m = nbrMaskNext(covered, -1); m >= 0; m = nbrMaskNext(covered, m))
                {
                    if (!first)
                        web_client.print(", ");
                    nbrPrintCall((uint8_t)m);
                    first = false;
                }
                if (first)
                    web_client.print("only me");
            }
            web_client.print("</td>");

            int partner = -1;
            float reach = nbrReach(nbrMatrix, X, now_min, &partner);
            if (reach >= 0 && partner >= 0)
            {
                web_client.printf("<td>%.1f km @ ", (double)reach);
                nbrPrintCall((uint8_t)partner);
                web_client.print("</td>");
            }
            else
                web_client.print("<td>-</td>");

            web_client.printf("<td>%u</td></tr>\n", (unsigned)pr.age);
            continue;
        }

        // pr.kind == 2: Weg-Eintrag (2-Hop-Zeile oder Horizont).
        NbrRouteView r;
        if (!nbrRouteGet(nbrMatrix, (int)pr.idx, now_min, &r) || r.hops != pr.hops)
            continue; // Momentaufnahme hat sich zwischen den Durchlaeufen verschoben
                      // (Index zeigt auf einen anderen Eintrag -- nicht falsch einsortieren)

        bool hasRow = (r.row != 0xFF);
        NbrRowView rv;
        bool rowOk = hasRow && nbrRowGet(nbrMatrix, r.row, &rv);
        uint8_t rowFlags = rowOk ? rv.flags : 0;

        web_client.printf("<tr><td><a href=\"https://aprs.fi/?call=%s\" target=\"_blank\">%s</a></td>", urlEncode(r.call).c_str(), htmlEscape(String(r.call)).c_str());
        web_client.printf("<td>%u</td>", (unsigned)r.hops);
        web_client.printf("<td>%s</td>", hasRow ? ((rowFlags & NBR_FLAG_GW) ? "Y" : "N") : (r.gw ? "Y" : "N"));

        if (!hasRow || !(rowFlags & NBR_FLAG_POS))
            web_client.print("<td>-</td>"); // Horizont hat keine Zeile, oder keine Positions-Info
        else
            web_client.printf("<td>%s</td>", (rowFlags & NBR_FLAG_MESH) ? "Y" : "N");

        web_client.print("<td>");
        if (rowOk)
            webPrintFw(rv.fw_ver, rv.fw_sub);
        else
            web_client.print("-"); // Horizont (keine Zeile) oder Zeile zwischendurch verschwunden
        web_client.print("</td>");

        if (hasRow)
        {
            NbrEdgeView hm;
            if (nbrEdgeGet(nbrMatrix, 0, r.row, &hm) && hm.snr != NBR_SNR_UNKNOWN && nbrFresh(hm.last_min, now_min))
                web_client.printf("<td>%d</td>", (int)hm.snr);
            else
                web_client.print("<td>-</td>");
        }
        else
            web_client.print("<td>-</td>");

        // Via / Covered by: unveraenderte Via-Logik der alten Pfadseite.
        web_client.print("<td style=\"max-width:220px;overflow-wrap:anywhere;\">");
        {
            bool first = true;
            for (int a = nbrMaskNext(r.entry, -1); a >= 0; a = nbrMaskNext(r.entry, a))
            {
                NbrRowView av;
                const char *acall = nbrRowGet(nbrMatrix, a, &av) ? av.call : "?";

                if (r.is_row)
                { // 2-Hop-Zeile: entry IST schon die B-Menge (die direkten Nachbarn)
                    web_client.printf("%s%s", first ? "" : ", ", htmlEscape(String(acall)).c_str());
                    first = false;
                    continue;
                }

                // Horizont: entry sind die Eintrittszeilen A; wo billig, dazu
                // die B's, ueber die jedes A hereinkommt.
                NbrMask viaB = nbrMaskAnd(nbrHearersMask(nbrMatrix, a, now_min), directMask);
                if (nbrMaskEmpty(viaB))
                {
                    web_client.printf("%s%s", first ? "" : ", ", htmlEscape(String(acall)).c_str());
                    first = false;
                    continue;
                }
                for (int b = nbrMaskNext(viaB, -1); b >= 0; b = nbrMaskNext(viaB, b))
                {
                    NbrRowView bv;
                    const char *bcall = nbrRowGet(nbrMatrix, b, &bv) ? bv.call : "?";
                    web_client.printf("%s%s&gt;%s", first ? "" : ", ", htmlEscape(String(acall)).c_str(), htmlEscape(String(bcall)).c_str());
                    first = false;
                }
            }
        }
        web_client.print("</td>");

        if (hasRow)
        {
            int partner = -1;
            float reach = nbrReach(nbrMatrix, r.row, now_min, &partner);
            if (reach >= 0 && partner >= 0)
            {
                web_client.printf("<td>%.1f km @ ", (double)reach);
                nbrPrintCall((uint8_t)partner);
                web_client.print("</td>");
            }
            else
                web_client.print("<td>-</td>");
        }
        else
            web_client.print("<td>-</td>");

        web_client.printf("<td>%u</td></tr>\n", (unsigned)r.age_min);
    }
    free(rec);
    web_client.println("</table>");

    // Legende, wie bei sub_page_neighbours() unten: title= wirkt auf dem
    // Telefon nicht, deshalb stehen dieselben Erklaerungen hier als Text.
    web_client.print("<p style=\"font-size:var(--fs-s);color:var(--mcmute);\">"
                      "Hops: 0 = me, 1 = direct neighbour, 2+ = reached via other nodes."
                      " | Firmware: the version the station puts into its own frames, '-' until one arrives or beyond 2 hops."
                      " | Hears me: SNR at which that neighbour last reported hearing me, '-' if unknown or not a direct/2-hop row."
                      " | Via: the entry path this station was heard on (2-hop: the direct neighbour it is via; horizon: the entry row, and beyond it if cheap to show)."
                      " Covered by (direct neighbours only): other direct neighbours that also hear this one -- 'only me' means I am its only link."
                      " | Reach: the farthest station with a known position that is linked to this one (heard by it or hearing it).");
    web_client.println("</p>");

    web_client.println("</div>");
    web_client.println(); // The HTTP response ends with another blank line
}

/**
 * ###########################################################################################################################
 * delivers the neighbour-matrix page to be injected into the scaffold (NBR-W2, Konzept 4.5;
 * Stufe 2a, docs/nbr-wichtigkeit-konzept.md Abschnitt 6: Rollenspalten, Legende, Sortierung,
 * "Covered by" und der Block "My relay decision")
 *
 * W2c (Kantenpool, docs/meshcom5-campaign.md Welle 2, Konzept 4.5 in
 * docs/meshcom5-topologie/body/03-kern.html): NBR_MAX_ROWS kann jetzt 128
 * erreichen (S3/nRF52). Zwei Kostenpunkte, die die alte, dichte Matrix nicht
 * hatte, und die diese Fassung beide vermeidet:
 *  - RAM: vier zeilengrosse uint8_t-Felder teilen sich EINEN malloc()-Block
 *    (`scratch`, 4 * NBR_MAX_ROWS Byte, mit free() vor jedem Rueckkehrpunkt)
 *    statt permanent im BSS zu liegen (Orchestrator-Review 2026-09-25, zweite
 *    Runde: 1580 B `static` fuer eine selten geoeffnete Seite war noch immer
 *    zu viel gegen eine geplante Kampagnen-Ersparnis von ~300 B). malloc()
 *    schlaegt hier nicht wie printf() je Logzeile zu (siehe
 *    printf-malloc-starves-nimble.md), sondern hoechstens einmal pro
 *    Seitenaufruf -- ein Fehlschlag bricht mit einer Meldung ab, nie mit
 *    einem Crash. Kein Array eines struct-Typs (NbrRowView, NbrMask) liegt
 *    ueberhaupt im Speicher; ein einzelnes Exemplar auf dem Stack, direkt am
 *    Ort seiner Benutzung, kostet nichts, solange es keine Zeile im Array
 *    ist.
 *  - CPU: nbrEdgeGet() sucht in einem Kantenpool (bis zu NBR_MAX_EDGES = 4 *
 *    NBR_MAX_ROWS Eintraege) -- O(1) ist es NICHT. Jede Schleife hier fragt
 *    deshalb zuerst eine Maske (nbrDirectMask()/nbrHearersMask(), beide auf
 *    den mitgefuehrten Bitmasken der Matrix und damit billig) und ruft
 *    nbrEdgeGet() nur noch fuer ein Bit, das die Maske schon als gesetzt
 *    gemeldet hat -- dort, wo tatsaechlich ein cnt/snr-Wert fuer die Anzeige
 *    gebraucht wird.
 * Die Kreuztabelle "wer hoert wen" zeigt bei mehr als 64 sichtbaren Zeilen
 * nur einen 64-Spalten-Ausschnitt, ueber &x=<Spalte> weitergeblaettert --
 * alle ANDEREN Spalten der Kopfzeile (D/I, G, M, #N, #X, Cov, Role) bleiben
 * dabei fuer alle n Zeilen vollstaendig, nur das n*n-Gitter selbst wird
 * seitenweise gerendert.
 * W4c: die zweite Tabelle dieser Seite (Call/GW/Mesh/Hears me/Hearers/
 * Covered by/Reach/Age, eine Zeile je sichtbarer Nachbarschaftszeile) ist mit
 * der Path-Seite zusammengelegt -- dort steht jetzt jede Station in einer
 * einzigen, nach Hops sortierten Tabelle. Was diese Seite noch je Zeile
 * zeigt, ist auf die neue Cov-Spalte der Kreuztabelle geschrumpft (Covered by,
 * nur als Zahl+title); Details (Hears me/Reach/Via) gibt es nur noch auf der
 * Path-Seite.
 */
void sub_page_neighbours()
{
    // Vertragspunkt mit dem OnRxDone-Haken (lora_functions.cpp): dieselbe
    // Minuten-seit-Boot-Umrechnung, sonst laufen Web-Seite und Schreiber
    // gegen unterschiedliche Uhren.
    uint16_t now_min = uptimeMin16();

    _create_meshcom_subheader("Neighbours");
    web_client.println("<div id=\"content_inner\">");

    NbrRowView r0;
    if (!nbrRowGet(nbrMatrix, 0, &r0) || r0.call[0] == 0)
    { // noch kein Frame ausgewertet -- Zeile 0 ist unbelegt
        web_client.println("<p>No frames received yet.</p>");
        web_client.println("</div>");
        web_client.println(); // The HTTP response ends with another blank line
        return;
    }

    // W2c CONTRACT: ein einziger transienter Heap-Block statt vier einzelner
    // static uint8_t[NBR_MAX_ROWS]-Arrays (Orchestrator-Review 2026-09-25,
    // zweite Runde): die Seite wird selten geoeffnet, ~0.5 kB fuer die Dauer
    // EINES Renderaufrufs ist unproblematisch -- anders als der printf-Heap-
    // Churn je Logzeile (siehe printf-malloc-starves-nimble.md), der einmal
    // pro RX/TX zuschlaegt, nicht einmal pro Seitenaufruf. free() steht direkt
    // vor jedem Rueckkehrpunkt; die Funktion hat ab hier nur noch das
    // natuerliche Ende (kein weiteres return), das free() steht dort.
    // W4c: order[]/directList[]/indirectList[]/hearers[] sind mit der
    // Zeilentabelle weg (in die Path-Seite zusammengelegt) -- der Block ist
    // von acht auf vier Zeilenfelder geschrumpft.
    uint8_t *scratch = (uint8_t *)malloc(4 * (size_t)NBR_MAX_ROWS);
    if (scratch == NULL)
    {
        web_client.println("<p>Not enough memory to render the neighbour matrix.</p>");
        web_client.println("</div>");
        web_client.println(); // The HTTP response ends with another blank line
        return;
    }
    uint8_t *idx = scratch + 0 * NBR_MAX_ROWS;
    uint8_t *eself = scratch + 1 * NBR_MAX_ROWS;
    uint8_t *xCount = scratch + 2 * NBR_MAX_ROWS;
    uint8_t *nCount = scratch + 3 * NBR_MAX_ROWS;

    // Sichtbare Zeilen: Zeile 0 immer dabei, sonst USED (nbrRowGet() liefert
    // false fuer eine unbelegte Zeile != 0) und frisch (Konzept 4.5, Fenster
    // 12h). idx[i] ist die Matrix-Zeilennummer -- nur diese Nummer wird
    // gespeichert, keine Kopie der Zeile selbst (siehe Funktionskopf).
    uint8_t n = 0;
    idx[n++] = 0;
    for (uint8_t r = 1; r < NBR_MAX_ROWS; r++)
    {
        NbrRowView v;
        if (nbrRowGet(nbrMatrix, r, &v) && nbrFresh(v.last_min, now_min))
            idx[n++] = r;
    }

    // E_self (Konzept 4, 6.1 Zeilenfarbe / 6.2 "Covered by"): direkt gehoerte
    // Zeilen, die kein ANDERER direkter Nachbar frisch hoert -- ersetzt die
    // alte <verdict>-Faerbung (frueher ueber die 2-Hop-Funktion) vollstaendig.
    int neself = nbrExclusiveDirect(nbrMatrix, now_min, eself, NBR_MAX_ROWS);
    uint8_t neself_shown = (neself > 0) ? ((neself < (int)NBR_MAX_ROWS) ? (uint8_t)neself : (uint8_t)NBR_MAX_ROWS) : 0;

    web_client.printf("<p>Window: %u h, %u row(s) fresh. ", (unsigned)(NBR_WINDOW_MIN / 60), (unsigned)n);
    if (neself < 0)
    {
        web_client.println("Nothing heard directly yet.</p>");
    }
    else if (neself == 0)
    {
        web_client.println("No exclusive nodes. Mesh is redundant here.</p>");
    }
    else
    {
        web_client.print("Exclusive to me (only I hear them directly): ");
        for (uint8_t i = 0; i < neself_shown; i++)
        {
            if (i)
                web_client.print(", ");
            nbrPrintCall(eself[i]);
        }
        web_client.println(". Mesh needed.</p>");
    }

    // Direkt(X) (Konzept 4.3) EIN Mal geholt -- eine Bitmaske ueber
    // Zeilenindizes, nicht 128 einzelne Kantenabfragen. Bit 0 ist darin per
    // Vertrag nie gesetzt (nbrDirectMask() liefert nur X != 0), ein Test auf
    // Zeile 0 gibt also ohnehin "nicht direkt" zurueck -- die expliziten
    // "X == 0"-Zweige unten bleiben trotzdem, weil Zeile 0 andere Zellen
    // zeigt (";-" statt "Indirect").
    NbrMask directMaskGlobal = nbrDirectMask(nbrMatrix, now_min);

    // Vorpass fuer 6.1/6.2: xCount[i] (#X, nur wenn direkt -- die
    // Anteilsregel aus Konzept 4.3 steckt in nbrRowMeshNeedCount() selbst,
    // wird hier nicht nachgebaut) je sichtbarer Zeile.
    for (uint8_t i = 0; i < n; i++)
    {
        meshcom_wdt_feed(); // one feed per row; n <= NBR_MAX_ROWS keeps the loop bounded
        uint8_t X = idx[i];
        // -1 ("NA") ist moeglich, wenn der LORA-Task (nRF52) die Kante (X, 0)
        // zwischen nbrDirectMask() oben und diesem Aufruf freigibt -- als 0
        // zaehlen, nicht als 255 (Advisor Welle 2, Befund B).
        int xc = nbrMaskTest(directMaskGlobal, X) ? nbrRowMeshNeedCount(nbrMatrix, X, now_min) : 0;
        xCount[i] = (xc < 0) ? 0 : (uint8_t)xc;
    }

    // #N je ZEILENNUMMER (nicht Position -- siehe Render-Schleife unten, die
    // per idx[i] direkt hineinindiziert): "wie viele Zeilen hat X gehoert".
    // Das ist die SPALTE X der alten dichten Matrix (cell[frm][X] fuer alle
    // frm), keine der vorhandenen Masken liefert diese Richtung direkt (nur
    // nbrDirectMask()/nbrHeardMeMask() fuer Zeile 0 fest verdrahtet). Billig
    // trotzdem: fuer jede Zeile frm liefert nbrHearersMask(frm) die (kleine,
    // bitmaskenbasierte) Menge der Y, die frm gehoert haben -- steht X darin,
    // hat X frm gehoert, zaehlt also fuer nCount[X]. Zeile 0 bekommt ihren
    // Wert separat: nbrHearersMask() liefert nie Y == 0 (Vertrag), aber
    // nbrDirectMask() IST "die Menge der von mir gehoerten Zeilen" = #N(0).
    for (uint8_t r = 0; r < NBR_MAX_ROWS; r++)
        nCount[r] = 0;
    nCount[0] = (uint8_t)((nbrMaskCount(directMaskGlobal) > 255) ? 255 : nbrMaskCount(directMaskGlobal));
    for (uint8_t frm = 0; frm < NBR_MAX_ROWS; frm++)
    {
        meshcom_wdt_feed(); // one feed per row; frm < NBR_MAX_ROWS keeps the loop bounded
        NbrMask hm = nbrHearersMask(nbrMatrix, frm, now_min);
        for (int y = nbrMaskNext(hm, -1); y >= 0; y = nbrMaskNext(hm, y))
        {
            if (nCount[y] < 255)
                nCount[y]++;
        }
    }

    // Super-Node (Konzept 5.3/6.1): der direkte Nachbar mit dem groessten #X,
    // wenn #X >= 2 UND mindestens doppelt so gross wie der naechstbeste --
    // sonst kein Super-Node in Reichweite.
    int superI = -1;
    int superTop2 = 0;
    {
        int topVal = -1, top2Val = -1, topI = -1;
        for (uint8_t i = 0; i < n; i++)
        {
            if (!nbrMaskTest(directMaskGlobal, idx[i]))
                continue;
            int v = (int)xCount[i];
            if (v > topVal)
            {
                top2Val = topVal;
                topVal = v;
                topI = i;
            }
            else if (v > top2Val)
            {
                top2Val = v;
            }
        }
        int top2 = (top2Val < 0) ? 0 : top2Val;
        if (topI >= 0 && topVal >= 2 && topVal >= 2 * top2)
        {
            superI = topI;
            superTop2 = top2;
        }
    }

    // W2c: bei mehr als 64 sichtbaren Zeilen wird das n*n-Gitter (die
    // "wer hoert wen"-Spalten) seitenweise zu 64 Spalten gerendert, ueber
    // &x=<Startspalte> (0-basiert, auf 64 gerundet) weitergeblaettert --
    // ALLE anderen Spalten (Kopf D/I..Role) und Tabelle 2 unten bleiben
    // fuer alle n Zeilen vollstaendig, siehe Funktionskopf-Kommentar.
    // web_header traegt weiterhin die rohe Request-Zeile ("GET /?page=
    // neighbours&x=64 HTTP/1.1"); dieselbe indexOf()/substring()-Technik wie
    // send_message() (Zeile ~2973) fuer &tocall=/&message=.
    uint16_t colStart = 0;
    if (n > 64)
    {
        int xp = web_header.indexOf("&x=");
        if (xp >= 0)
        {
            long xv = web_header.substring(xp + 3).toInt();
            if (xv > 0)
                colStart = (uint16_t)(((unsigned long)xv / 64) * 64);
        }
        if (colStart >= n)
            colStart = (uint16_t)(((n - 1) / 64) * 64);
    }
    uint16_t colEnd = (n > 64) ? ((colStart + 64 < n) ? (uint16_t)(colStart + 64) : n) : n;

    if (n > 64)
    {
        web_client.printf("<p>Cross table columns %u-%u of %u. ", (unsigned)(colStart + 1), (unsigned)colEnd, (unsigned)n);
        if (colStart > 0)
            web_client.printf("<a href=\"?page=neighbours&x=%u\">&laquo; prev</a> ", (unsigned)((colStart >= 64) ? colStart - 64 : 0));
        if (colEnd < n)
            web_client.printf("<a href=\"?page=neighbours&x=%u\">next &raquo;</a>", (unsigned)(colStart + 64));
        web_client.println("</p>");
    }

    // Kreuztabelle: Spaltenkoepfe sind Anzeige-Nummern (1..n) mit dem
    // Rufzeichen darunter, senkrecht gesetzt (6.1, Regel 1) -- die Tabelle
    // waechst dadurch in der Hoehe, nicht in der Breite. Das Tabellen-CSS
    // des Scaffolds greift nur auf "#content_inner > table" (Zeile ~1096);
    // ein Wrapper-div fuer das seitliche Scrollen wuerde die Matrix aus dem
    // Selektor werfen (Bench 2026-09-20: Matrix ohne Rahmen). Deshalb
    // scrollt die Tabelle selbst als Block.
    web_client.println("<table class=\"table\" style=\"display:inline-block;overflow-x:auto;width:auto;max-width:100%;\">");
    web_client.print("<thead><tr class=\"font-bold\">");
    web_client.print("<td></td>");
    web_client.print("<td title=\"Direct: heard by me over the air. Indirect: only via a neighbour.\">D/I</td>");
    web_client.print("<td title=\"Gateway: a HEY addressed to HG was seen from this node. 'no' means not observed.\">G</td>");
    web_client.print("<td title=\"Mesh: relays foreign frames, from its last position frame.\">M</td>");
    web_client.print("<td title=\"Neighbours: nodes this node hears, as far as this table can hold them.\">#N</td>");
    web_client.print("<td title=\"Exclusive: nodes that ONLY this neighbour hears. This is the value of a relay.\">#X</td>");
    web_client.print("<td title=\"Covered by: other direct neighbours that also hear it. 0 = only I reach it. Details on the Path page.\">Cov</td>");
    web_client.print("<td title=\"Super: largest exclusive share. Needed: has exclusive nodes. Redundant: everything it hears is heard by others.\">Role</td>");
    for (uint8_t j = colStart; j < colEnd; j++)
    {
        meshcom_wdt_feed(); // one feed per header column; colEnd - colStart <= 64 keeps the loop bounded
        web_client.printf("<td>%u<br><span style=\"writing-mode:vertical-rl;transform:rotate(180deg);white-space:nowrap;\">", (unsigned)(j + 1));
        nbrPrintCall(idx[j]);
        web_client.print("</span></td>");
    }
    web_client.println("</tr></thead>");

    for (uint8_t i = 0; i < n; i++)
    {
        meshcom_wdt_feed(); // one feed per row; n <= NBR_MAX_ROWS keeps the loop bounded
        uint8_t X = idx[i];
        bool rowESelf = false;
        for (uint8_t e = 0; e < neself_shown; e++)
        {
            if (eself[e] == X)
            {
                rowESelf = true;
                break;
            }
        }
        bool isDirect = nbrMaskTest(directMaskGlobal, X);

        // Hoerer von X (Konzept 4.3): EIN Mal je Zeile geholt (Bitmaske, kein
        // Kantenpool-Scan), zweifach genutzt -- fuer die D/I-Spalte (die erste
        // gesetzte Spalte ist "via") UND fuer jede Gitterzelle dieser Zeile
        // unten (nbrMaskTest statt eines eigenen nbrEdgeGet() je Zelle).
        NbrMask hm = nbrHearersMask(nbrMatrix, X, now_min);

        // Zeilenfarbe nach Rolle, nicht mehr nach <verdict> (6.1, Regel 2):
        // Zeile 0 blau, der Super-Node gruen, E_self weiterhin rot.
        if (X == 0)
            web_client.print("<tr class=\"nbr-me\">");
        else if ((int)i == superI)
            web_client.print("<tr class=\"nbr-super\">");
        else if (rowESelf)
            web_client.print("<tr class=\"nbr-eself\">");
        else
            web_client.print("<tr>");
        web_client.printf("<td class=\"font-bold\">%u ", (unsigned)(i + 1));
        nbrPrintCall(X);
        web_client.print("</td>");

        // D/I. Bei X==0 gibt es keine Kante nach 0 -- direkt "-". Sonst nur
        // fuer direkte Zeilen ein nbrEdgeGet() (fuer die SNR, die einzige
        // Kanteneigenschaft, die hier angezeigt wird); "via" oben aus hm.
        if (X == 0)
        {
            web_client.print("<td>-</td>");
        }
        else if (isDirect)
        {
            NbrEdgeView e0;
            if (nbrEdgeGet(nbrMatrix, X, 0, &e0) && e0.snr != NBR_SNR_UNKNOWN)
                web_client.printf("<td title=\"Heard directly, SNR %d dB\">D</td>", (int)e0.snr);
            else
                web_client.print("<td title=\"Heard directly, SNR unknown\">D</td>");
        }
        else
        {
            int via = nbrMaskNext(hm, -1);
            if (via >= 0)
            {
                web_client.print("<td title=\"Indirect, via ");
                nbrPrintCall((uint8_t)via);
                web_client.print("\">I</td>");
            }
            else
                web_client.print("<td title=\"Indirect\">I</td>");
        }

        // G/M brauchen die Flags dieser einen Zeile -- ein lokales
        // NbrRowView, kein Array (siehe Funktionskopf-Kommentar).
        NbrRowView rowV;
        uint8_t flags = nbrRowGet(nbrMatrix, X, &rowV) ? rowV.flags : 0;

        // G
        if (flags & NBR_FLAG_GW)
            web_client.print("<td title=\"Gateway HEY seen\">Y</td>");
        else
            web_client.print("<td title=\"No gateway HEY observed\">N</td>");

        // M
        if (!(flags & NBR_FLAG_POS))
            web_client.print("<td title=\"No position frame in window\">-</td>");
        else if (flags & NBR_FLAG_MESH)
            web_client.print("<td title=\"Mesh enabled\">Y</td>");
        else
            web_client.print("<td title=\"Mesh disabled\">N</td>");

        // #N -- ueber die Zeilennummer X indiziert (siehe Vorpass oben), nicht
        // ueber die Position i.
        web_client.printf("<td title=\"Hears %u node(s) (table holds %u rows)\">%u</td>",
                           (unsigned)nCount[X], (unsigned)NBR_MAX_ROWS, (unsigned)nCount[X]);

        // #X -- die title= nennt zugleich die beiden Zaehler, die #X
        // entschieden haben: wie viele Knoten X ueberhaupt hoert (nCount, das
        // Divisor der Anteilsregel) und wie viele davon niemand sonst deckt
        // (xCount, der Zaehler); ein Aufschluesseln je Kandidaten-Kante
        // braeuchte einen weiteren nbrEdgeGet()-Aufruf je Hoerer und ist das
        // hier nicht wert (Konzept 4.3 wird nicht nachgebaut, nur angezeigt).
        if (isDirect)
            web_client.printf("<td title=\"%u of %u heard by nobody else in my range\">%u</td>",
                               (unsigned)xCount[i], (unsigned)nCount[X], (unsigned)xCount[i]);
        else
            web_client.print("<td>-</td>");

        // Cov (W4c, ersetzt die "Covered by"-Spalte der geloeschten
        // Zeilentabelle): fuer eine direkte Zeile die anderen direkten
        // Nachbarn, die X ebenfalls frisch hoeren -- hm ist oben schon
        // geholt, also nur noch mit directMaskGlobal verunden, keine neue
        // Maskenberechnung.
        if (isDirect)
        {
            NbrMask covered = nbrMaskAnd(hm, directMaskGlobal);
            int covCount = nbrMaskCount(covered);
            web_client.print("<td title=\"Covered by: ");
            if (covCount == 0)
                web_client.print("nobody");
            else
            {
                bool firstCov = true;
                for (int m = nbrMaskNext(covered, -1); m >= 0; m = nbrMaskNext(covered, m))
                {
                    if (!firstCov)
                        web_client.print(", ");
                    nbrPrintCall((uint8_t)m);
                    firstCov = false;
                }
            }
            web_client.printf("\">%u</td>", (unsigned)covCount);
        }
        else
            web_client.print("<td>-</td>");

        // Role
        if (X == 0 || !isDirect)
        {
            web_client.print("<td>-</td>");
        }
        else if ((int)i == superI)
        {
            web_client.printf("<td title=\"Super node: %u exclusive, next best %u\">Super</td>", (unsigned)xCount[i], (unsigned)superTop2);
        }
        else if (xCount[i] >= 1)
        {
            web_client.printf("<td title=\"Needed: %u exclusive\">Needed</td>", (unsigned)xCount[i]);
        }
        else
        {
            web_client.print("<td title=\"Redundant: 0 exclusive, heard by others\">Redundant</td>");
        }

        // Gitterzeile: hm (oben, EIN Aufruf je Zeile) sagt per nbrMaskTest,
        // ob die Zelle ueberhaupt etwas zeigt -- nbrEdgeGet() (der teure
        // Kantenpool-Scan) laeuft nur noch fuer eine Zelle, die die Maske
        // schon als gesetzt gemeldet hat, und nur fuer cnt/snr zur Anzeige.
        for (uint8_t j = colStart; j < colEnd; j++)
        {
            uint8_t Y = idx[j];
            if (X == Y || !nbrMaskTest(hm, Y))
            {
                web_client.print((X == Y) ? "<td>-</td>" : "<td></td>");
                continue;
            }
            NbrEdgeView c;
            if (!nbrEdgeGet(nbrMatrix, X, Y, &c))
                web_client.print("<td></td>"); // Maske war gesetzt, Kante ist inzwischen weg (Sweep/EVICT lief dazwischen)
            else if (c.snr != NBR_SNR_UNKNOWN)
                web_client.printf("<td title=\"cnt:%u snr:%d\">%u</td>", (unsigned)c.cnt, (int)c.snr, (unsigned)c.cnt);
            else
                web_client.printf("<td title=\"cnt:%u snr:NA\">%u</td>", (unsigned)c.cnt, (unsigned)c.cnt);
        }
        web_client.println("</tr>");
    }
    web_client.println("</table>");

    if (n > 64)
    {
        web_client.printf("<p>Cross table columns %u-%u of %u. ", (unsigned)(colStart + 1), (unsigned)colEnd, (unsigned)n);
        if (colStart > 0)
            web_client.printf("<a href=\"?page=neighbours&x=%u\">&laquo; prev</a> ", (unsigned)((colStart >= 64) ? colStart - 64 : 0));
        if (colEnd < n)
            web_client.printf("<a href=\"?page=neighbours&x=%u\">next &raquo;</a>", (unsigned)(colStart + 64));
        web_client.println("</p>");
    }

    // Legende (6.1, Kopf): title= wirkt auf dem Telefon nicht, deshalb
    // stehen dieselben Erklaerungen hier zusaetzlich als Text (W4c: Cov dazu,
    // die "Covered by"-Spalte der geloeschten Zeilentabelle).
    web_client.print("<p style=\"font-size:var(--fs-s);color:var(--mcmute);\">"
                      "D/I: Direct: heard by me over the air. Indirect: only via a neighbour."
                      " | G: Gateway: a HEY addressed to HG was seen from this node. 'no' means not observed."
                      " | M: Mesh: relays foreign frames, from its last position frame."
                      " | #N: Neighbours: nodes this node hears, as far as this table can hold them."
                      " | #X: Exclusive: nodes that ONLY this neighbour hears. This is the value of a relay."
                      " | Cov: other direct neighbours that also hear it; 0 = only I reach it. Details on the Path page."
                      " | Role: Super: largest exclusive share. Needed: has exclusive nodes."
                      " Redundant: everything it hears is heard by others.");
    web_client.println("</p>");

    // 6.3: "My relay decision" -- drei Zeilen Text, keine Tabelle. Die
    // Zaehler kommen aus loop_functions.cpp (5.8 Punkt 3); solange
    // --nbrrelay off ist (Default), bleiben sie 0.
    web_client.println("<h4>My relay decision</h4>");
    web_client.print("<p>Exclusive to me: ");
    if (neself_shown == 0)
    {
        web_client.print("none");
    }
    else
    {
        for (uint8_t e = 0; e < neself_shown; e++)
        {
            if (e)
                web_client.print(", ");
            nbrPrintCall(eself[e]);
        }
    }
    web_client.println("</p>");

    web_client.print("<p>Super node in range: ");
    if (superI >= 0)
    {
        nbrPrintCall(idx[superI]);
        web_client.printf(" (%u exclusive)", (unsigned)xCount[superI]);
    }
    else
        web_client.print("none");
    web_client.println("</p>");

    web_client.printf("<p>Relay mode: %s -- relays A: %lu, B: %lu, cancelled: %lu, could have cancelled: %lu, refused (sole provider): %lu</p>",
                       (bNBRCANCEL ? "on" : (bNBRRELAY ? "count" : "off")),
                       (unsigned long)stat_nbr_relay_a, (unsigned long)stat_nbr_relay_b, (unsigned long)stat_nbr_cancel,
                       (unsigned long)stat_nbr_cancel_possible, (unsigned long)stat_nbr_refuse_alone);

    // Stufe 2 (--nbrsym): mirrors bNBRSYM, the flag the live relay decision
    // in lora_functions.cpp actually reads per frame -- this page never
    // calls nbrRelayNeed()/nbrCoverMask() itself, so there is no sym-aware
    // call to thread the flag through, only this status line.
    web_client.printf("<p>Symmetry: %s (SNR &gt;= %d dB)</p>", (bNBRSYM ? "on" : "off"), (int)NBR_SYM_MIN_SNR);

    web_client.println("</div>");
    web_client.println(); // The HTTP response ends with another blank line
    free(scratch);
}

/**
 * ###########################################################################################################################
 * delivers the message-page to be injected into the scaffold
 * Includes all messages received/stored so far. To update message while using the Message Page, there is another callback function
 * (sub_content_messages) that will only deliver the message block
 */
void sub_page_messages()
{
    _create_meshcom_subheader("Messages");
    web_client.println("<div id=\"content_inner\">");

    // tab bar filtering the panel below by data-dst; mcTab()/mcApplyTab() in
    // the scaffold do the actual filtering (sub-page <script> never runs)
    web_client.println("<div id=\"mctabs\">");
    web_client.println("<button class=\"mctab mctab-on\" data-tab=\"all\" onclick=\"mcTab(this)\">All</button>");
    web_client.println("<button class=\"mctab\" data-tab=\"*\" onclick=\"mcTab(this)\">*</button>");
    for (int i = 0; i < (int)sizeof(meshcom_settings.node_gcb) / (int)sizeof(meshcom_settings.node_gcb[0]); i++)
    {
        if (meshcom_settings.node_gcb[i] > 0 && meshcom_settings.node_gcb[i] < 100000)
        {
            web_client.printf("<button class=\"mctab\" data-tab=\"%i\" onclick=\"mcTab(this)\">%i</button>\n", meshcom_settings.node_gcb[i], meshcom_settings.node_gcb[i]);
        }
    }
    web_client.println("<button class=\"mctab\" data-tab=\"dm\" onclick=\"mcTab(this)\">DM</button>");
    web_client.println("</div>");

    // this is where the asynchronous received messages will be displayed
    web_client.println("<div id=\"messages_panel\" class=\"mw-600\">");
    sub_content_messages(); // deliver all known messages
    web_client.println("</div>");

    // the input panel for new messages
    web_client.println("<table class=\"mw-600\"><tbody><tr>");
    web_client.println("<td><label for=\"sendcall\" class=\"font-small font-bold\">DM Call (or empty):</label></td>");
    web_client.println("<td><input type=\"text\" id=\"sendcall\" name=\"sendcall\" maxlength=\"9\" size=\"9\" oninput=\"updateCharsLeft()\";></td>");
    web_client.println("</tr><tr>");
    web_client.println("<td><label for=\"messagetext\" class=\"font-small font-bold\">Message:</label><p class=\"font-small\"><span id=\"indicator_charsleft\">149</span> bytes left</p></td>");
    web_client.println("<td><textarea id=\"messagetext\" name=\"messagetext\" maxlength=\"149\" rows=\"5\" cols=\"40\" oninput=\"updateCharsLeft()\";></textarea></td>");
    web_client.println("</tr><tr>");
    web_client.println("<td><button onclick=\"updateMessages()\"><i class=\"btnrefresh\"></i>Update</button></td></td><td><button id=\"sendmessage\" onclick=\"sendMessage(); updateCharsLeft(); updateMessages();\"><i class=\"btncheckmark\"></i>Send</button></td>");
    web_client.println("</tr></tbody></table>");
    web_client.println("</div>");

    web_client.println("</div>");
    web_client.println(); // The HTTP response ends with another blank line
}

#if defined(ENABLE_MSGSTORE)
/**
 * ###########################################################################################################################
 * Formats a millisecond duration as "Ns" / "N min" / "Nh NNmin" into a caller-supplied stack
 * buffer -- stage 3 mailbox page helper, kept out of sub_page_mailbox() only so every one of
 * its six call sites (age, hold left, jitter/next) stays a single line. No heap, no String
 * concatenation, matching the rest of this file's per-row discipline.
 */
static void mbxFormatDuration(char *buf, size_t n, uint32_t ms)
{
    uint32_t secs = ms / 1000UL;
    if (secs < 60UL)
        snprintf(buf, n, "%lus", (unsigned long)secs);
    else if (secs < 3600UL)
        snprintf(buf, n, "%lu min", (unsigned long)(secs / 60UL));
    else
        snprintf(buf, n, "%lu h %02lu min", (unsigned long)(secs / 3600UL), (unsigned long)((secs % 3600UL) / 60UL));
}

/**
 * ###########################################################################################################################
 * delivers the mailbox (store node) page to be injected into the scaffold. Stage 3,
 * docs/dm-stage3-wave-plan-20260914.md -- read-only list of what this node is holding for
 * other stations plus operator actions (deliver now / purge); the stored message TEXT is
 * never rendered, only its stripped length (plen). Owner-gated like every other page: the
 * password check already ran in work_webpage() before this is ever called.
 */
void sub_page_mailbox()
{
    _create_meshcom_subheader("Mailbox");
    web_client.println("<div id=\"content_inner\">");

    char buf_next[24];
    uint32_t next_in_ms = msgstoreNextActionInMs();
    if (next_in_ms == 0) // msgstore_api.h: 0 == nothing due, not "due now"
        snprintf(buf_next, sizeof(buf_next), "none due");
    else
        mbxFormatDuration(buf_next, sizeof(buf_next), next_in_ms);

    enum MsgStoreMode mode = msgstoreMode();
    int used = msgstoreUsed();

    web_client.println("<div class=\"mbx-toolbar\">");
    web_client.printf("<span class=\"mctab mctab-on\">store node: %s<span class=\"mcbadge\">%s</span></span>\n",
                       msgstoreModeName(mode), mode == MSGSTORE_OFF ? "OFF" : "ON");
    web_client.printf("<span class=\"mctab\">notice %s</span>\n", msgstoreNotice() ? "on" : "off");
    web_client.printf("<span class=\"mctab\">slots %d / %u</span>\n", used, (unsigned)msgstoreSlots());
    web_client.printf("<span class=\"mctab\">actions this hour %u / %u</span>\n", (unsigned)msgstoreActionsLastHour(), (unsigned)MSGSTORE_ACTIONS_PER_HOUR);
    web_client.printf("<span class=\"mctab\">next action in %s</span>\n", buf_next);
    web_client.println("<span class=\"spacer\"></span>");
    web_client.println("<button type=\"button\" onclick=\"loadPage('mailbox',csender,false)\" title=\"Reload the list\">Refresh</button>\n");
    web_client.printf("<button type=\"button\" title=\"Drop every stored message\" onclick=\"if(confirm('Purge all %d stored messages? The senders are not told.')){callfunction('mboxpurge','all');}\">Purge all</button>\n", used);
    web_client.println("</div>");

    // No wrapper div: the scaffold table rules match `#content_inner > table` only.
    web_client.println("<table><thead><tr>");
    web_client.println("<th>Destination</th><th>Last heard</th><th>Source</th><th>NNN</th><th>Size</th><th>Age</th><th>Hold left</th><th>State</th><th>Attempts</th><th>Notified</th><th>Actions</th>");
    web_client.println("</tr></thead><tbody>");

    uint32_t now_ms = millis();
    uint32_t hold_total_ms = (uint32_t)msgstoreHoldHours() * 3600000UL;
    int nslots = (int)msgstoreSlots();

    for (int slot = 0; slot < nslots; slot++)
    {
        const struct MsgStoreEntry *e = msgstoreEntry(slot);
        if (e == NULL)
            continue;

        char buf_heard[24], buf_age[24], buf_hold[24], buf_attempt[48];

        // feature-snf port: fork-main reads mheardAgeMs(), which does not exist on this
        // branch (mheard_functions.* removed by ccb3ec23) -- same replacement as
        // msgstore_glue.cpp's glueHeardAgeMs(), the neighbour-matrix topology directly.
        int32_t heard_ms;
        {
            uint16_t now_min = uptimeMin16();
            int row = nbrFind(nbrMatrix, e->dst);
            NbrMhView v;
            heard_ms = nbrMhGet(nbrMatrix, row, now_min, &v) ? (int32_t)v.age_min * 60000L : -1;
        }
        bool stale = false;
        if (heard_ms < 0)
        {
            snprintf(buf_heard, sizeof(buf_heard), "never");
        }
        else
        {
            mbxFormatDuration(buf_heard, sizeof(buf_heard), (uint32_t)heard_ms);
            stale = ((uint32_t)heard_ms >= MSGSTORE_HEARD_WINDOW_MS);
        }

        uint32_t age_ms = now_ms - e->stored_ms; // monotonic, wraps the same way millis() does
        mbxFormatDuration(buf_age, sizeof(buf_age), age_ms);

        uint32_t held_ms = (age_ms >= hold_total_ms) ? hold_total_ms : age_ms;
        mbxFormatDuration(buf_hold, sizeof(buf_hold), hold_total_ms - held_ms);

        // advisor F3 (docs/review/fable-dm-stage3-verdict-20260914.md): millis-wrap
        // safe "time left" -- plain > misreports across the 49.7-day wrap.
        int32_t remain_signed = (int32_t)(e->next_ms - now_ms);
        uint32_t remain_ms = (remain_signed > 0) ? (uint32_t)remain_signed : 0;
        if (e->state == MSGSTORE_ARMED)
        {
            char buf_remain[16];
            mbxFormatDuration(buf_remain, sizeof(buf_remain), remain_ms);
            snprintf(buf_attempt, sizeof(buf_attempt), "%u.%u of %u &middot; jitter %s", e->cycles, e->attempt, MSGSTORE_LADDER_STEPS, buf_remain);
        }
        else if (e->state == MSGSTORE_LADDER || e->state == MSGSTORE_COOLDOWN)
        {
            char buf_remain[16];
            mbxFormatDuration(buf_remain, sizeof(buf_remain), remain_ms);
            snprintf(buf_attempt, sizeof(buf_attempt), "%u.%u of %u &middot; next %s", e->cycles, e->attempt, MSGSTORE_LADDER_STEPS, buf_remain);
        }
        else
        {
            snprintf(buf_attempt, sizeof(buf_attempt), "%u.%u of %u", e->cycles, e->attempt, MSGSTORE_LADDER_STEPS);
        }

        const char *state_class;
        switch (e->state)
        {
        case MSGSTORE_HELD: state_class = "mbx-held"; break;
        case MSGSTORE_ARMED: state_class = "mbx-armed"; break;
        case MSGSTORE_LADDER: state_class = "mbx-ladder"; break;
        case MSGSTORE_COOLDOWN: state_class = "mbx-cooldown"; break;
        default: state_class = ""; break;
        }

        web_client.printf("<tr><td class=\"font-bold no-wrap\">%s</td>", htmlEscape(String(e->dst)).c_str());
        if (stale)
            web_client.printf("<td class=\"no-wrap mbx-stale\" title=\"Older than the store set window\">%s</td>", buf_heard);
        else
            web_client.printf("<td class=\"no-wrap\">%s</td>", buf_heard);
        web_client.printf("<td class=\"no-wrap\">%s</td>", htmlEscape(String(e->src)).c_str());
        web_client.printf("<td class=\"num\">%03u</td>", (unsigned)e->nnn);
        web_client.printf("<td class=\"num\">%u B</td>", (unsigned)e->plen);
        web_client.printf("<td class=\"no-wrap\">%s</td>", buf_age);
        web_client.printf("<td class=\"no-wrap\">%s</td>", buf_hold);
        web_client.printf("<td><span class=\"mbx-state %s\">%s</span></td>", state_class, msgstoreStateName(e->state));
        web_client.printf("<td>%s</td>", buf_attempt);
        web_client.printf("<td>%s</td>", (e->notice == 2) ? "sent" : (e->notice == 1) ? "pending" : "-");

        web_client.println("<td class=\"mbx-actions\">");
        if (e->state == MSGSTORE_ARMED || e->state == MSGSTORE_LADDER)
        {
            web_client.printf("<button type=\"button\" disabled title=\"%s\">Deliver</button>",
                               (e->state == MSGSTORE_ARMED) ? "Delivery is already scheduled" : "A ladder is already running");
        }
        else
        {
            // callsigns travel in HTML-escaped data attributes and are read back via the DOM, never spliced into a JS string
            web_client.printf("<button type=\"button\" data-src=\"%s\" data-dst=\"%s\" onclick=\"if(confirm('Start a delivery ladder now for '+this.dataset.src+' to '+this.dataset.dst+' (NNN %03u)? Ignores the cooldown, still subject to the node caps.')){callfunction('mboxdeliver','%d');}\">Deliver</button>",
                               htmlEscape(String(e->src)).c_str(), htmlEscape(String(e->dst)).c_str(), (unsigned)e->nnn, slot);
        }
        web_client.printf("<button type=\"button\" data-src=\"%s\" data-dst=\"%s\" onclick=\"if(confirm('Purge the message for '+this.dataset.dst+' from '+this.dataset.src+' (NNN %03u)?')){callfunction('mboxpurge','%d');}\">Purge</button>",
                           htmlEscape(String(e->src)).c_str(), htmlEscape(String(e->dst)).c_str(), (unsigned)e->nnn, slot);
        web_client.println("</td></tr>");
    }

    web_client.println("</tbody></table>");

    web_client.println("<p class=\"mbx-legend\">");
    web_client.println("<span class=\"mbx-state mbx-held\">HELD</span>waiting for the destination to be heard directly");
    web_client.println("<span class=\"mbx-state mbx-armed\">ARMED</span>heard, random 5-60 s wait, cancelled if another store node delivers first");
    web_client.println("<span class=\"mbx-state mbx-ladder\">LADDER</span>9 sends over 9 min at hop 0");
    web_client.println("<span class=\"mbx-state mbx-cooldown\">COOLDOWN</span>ladder done without ack, 1 h before the next");
    web_client.println("<br>Attempts read cycle.attempt. Last heard in red is older than the 12 h store-set window: the entry is kept until hold runs out, but delivery needs a fresh direct sighting. Message text is never shown here.");
    web_client.println("</p>");

    const struct MsgStoreCounters *cnt = msgstoreCounters();
    web_client.println("<div class=\"cardlayout\">");
    web_client.println("<span class=\"cardlabel\">Counters since boot</span>");
    web_client.println("<div class=\"mbx-counters\">");
    web_client.printf("<div><span>stored</span><b>%u</b></div>", (unsigned)cnt->stored);
    web_client.printf("<div><span>refreshed</span><b>%u</b></div>", (unsigned)cnt->refreshed);
    web_client.printf("<div><span>delivered</span><b>%u</b></div>", (unsigned)cnt->delivered);
    web_client.printf("<div><span>purged by ack</span><b>%u</b></div>", (unsigned)cnt->purged_ack);
    web_client.printf("<div><span>dropped storetime</span><b>%u</b></div>", (unsigned)cnt->dropped_storetime);
    web_client.printf("<div><span>dropped by cap</span><b>%u</b></div>", (unsigned)cnt->dropped_cap);
    web_client.printf("<div><span>dropped no slot</span><b>%u</b></div>", (unsigned)cnt->dropped_slots);
    web_client.printf("<div><span>cancelled by peer</span><b>%u</b></div>", (unsigned)cnt->cancelled_peer);
    web_client.printf("<div><span>blocked by caps</span><b>%u</b></div>", (unsigned)cnt->blocked_bp);
    web_client.printf("<div><span>notices sent</span><b>%u</b></div>", (unsigned)cnt->notified);
    web_client.printf("<div><span>notices blocked</span><b>%u</b></div>", (unsigned)cnt->notice_blocked);
    web_client.printf("<div><span>refused, already acked</span><b>%u</b></div>", (unsigned)cnt->dropped_acked);   // SNF-ACK-01
    web_client.println("</div>");
    web_client.println("<p class=\"font-small\" style=\"margin:7px;\">Same numbers as the <code>MBOX</code> setlog line. Dropped by cap and dropped no slot are two counters on purpose: the first means the 20-per-hour ceiling ate a hold time, the second means the mailbox was full.</p>");
    web_client.println("</div>");

    web_client.println("</div>");
    web_client.println(); // The HTTP response ends with another blank line
}
#endif // ENABLE_MSGSTORE

// LED-08: charge LED text for the info page: the stored console value, plus the RM override while one
// is active ("auto (RM on)"), as in --info.
static const char *webChargeLedText()
{
#if defined(XPOWERS_CHIP_AXP192) || defined(XPOWERS_CHIP_AXP2101)
    const char *stored = bCHARGE_LED ? "auto" : "off";
    if (!chargeLedOverrideActive())
        return stored;
    static char txt[24];
    snprintf(txt, sizeof(txt), "%s (RM %s)", stored, chargeLedModeName(chargeLedEffective()));
    return txt;
#else
    return "off";
#endif
}

/**
 * ###########################################################################################################################
 * delivers the setup-page to be injected into the scaffold
 *
 */
__attribute__((unused)) static void _create_setup_led_mode_element(const char id[], const char labelText[], const char descriptionText[], int mode);   // WEB-LED-01, below
void sub_page_setup()
{
    _create_meshcom_subheader("Node Setup");
    web_client.println("<div id=\"content_inner\">");

    // Manual Command Section
    web_client.println("<div class=\"cardlayout collapsablecard\">\n");
    web_client.println("<label class=\"cardlabel\">Manual Command</label>\n");
    web_client.println("<span>Open this to send a manual command.</span>\n");
    web_client.println("<button class=\"cardtoggle\" onclick=\"togglecard(this);\"><i></i></button>\n");
    web_client.println("<div class=\"grid\">");
    web_client.println("<span>Enter manual command:</span>");
    web_client.println("<input type=\"text\" id=\"manualcommand\" maxlength=\"40\" size=\"20\" style=\"width:100%\">");
    web_client.println("<button onclick=\"setvalue('manualcommand', document.getElementById('manualcommand').value,false); document.getElementById('manualcommand').value='';\" style=\"justify-self:self-end;\">send command</button>");
    web_client.println("</div></div>");

    // Common Settings Section
    web_client.println("<div class=\"cardlayout collapsablecard cardopen\">");
    web_client.println("<label class=\"cardlabel\">Common Settings</label>");
    web_client.println("<span>Open this for common settings.</span>\n");
    web_client.println("<button class=\"cardtoggle\" onclick=\"togglecard(this);\"><i></i></button>\n");
    web_client.println("<div class=\"grid grid3\">");
    _create_setup_textinput_element("nodecall", "Call-Sign", meshcom_settings.node_call, "AB1CDE-12", "setcall", 9, false, false); // create Textinput-Element including Label and Button
    _create_setup_textinput_element("viacall", "Via-Call-Sign", meshcom_settings.node_via, "AB1CDE-12", "setvia", 9, false, false); // create Textinput-Element including Label and Button

    web_client.println("<label for=\"country\">Country</label>");
    web_client.println("<select id=\"country\" name=\"country\">");
    for (int ic = 0; ic < 21; ic++)
    {
        if (getCountry(ic) != "none")
        {
            web_client.printf("\t<option value=\"%i\" %s>%s</option>\n", ic, (ic == meshcom_settings.node_country) ? "selected" : "", getCountry(ic).c_str());
        }
    }
    web_client.println("</select>");
    web_client.println("<button onclick=\"setvalue('setctry', document.getElementById('country').value,false)\"><i class=\"btncheckmark\"></i></button>");

    _create_setup_numberinput_element("txpower", "TX Power", meshcom_settings.node_power, (int)TX_POWER_MIN, (int)TX_POWER_MAX, "txpower"); // D10: number input with the board range

    // CS-02: Max-Hop als Drop-down. Angeboten werden 4/3/2; hat --maxhop einen
    // Wert ausserhalb dieser Liste gesetzt (1, 5, 6), steht er zusaetzlich drin,
    // damit die Seite den echten Zustand zeigt statt ihn stillschweigend zu
    // aendern. Die Liste baut maxhop.h, dieselbe Quelle wie die serielle Pruefung.
    {
        int hops[MAXHOP_OPTION_MAX];
        int nhops = maxHopOptionList(meshcom_settings.max_hop_text, hops, MAXHOP_OPTION_MAX);

        web_client.println("<label for=\"maxhop\">Max Hop</label>");
        web_client.println("<select id=\"maxhop\" name=\"maxhop\">");
        for (int ih = 0; ih < nhops; ih++)
        {
            web_client.printf("\t<option value=\"%i\" %s>%i</option>\n", hops[ih], (hops[ih] == meshcom_settings.max_hop_text) ? "selected" : "", hops[ih]);
        }
        web_client.println("</select>");
        web_client.println("<button onclick=\"setvalue('maxhop', document.getElementById('maxhop').value,false)\"><i class=\"btncheckmark\"></i></button>");
    }

    _create_setup_textinput_element("utcoffset", "UTC Offset", String(meshcom_settings.node_utcoff, 1).c_str(), "1.0", "utcoffset", 4, false, false); // create Textinput-Element including Label and Button
    // TZ-01: a set POSIX TZ rule overrides the fixed offset above; the offset then shows the derived value
    // node_tz is HTML-escaped: tzParse() accepts any alnum name inside <...>
    // (e.g. "<plaintext>-1"); a config import refuses a value tzParse() rejects (TZ-02).
    _create_setup_textinput_element("tz", "Timezone (POSIX TZ)", String(meshcom_settings.node_tz), "CET-1CEST,M3.5.0,M10.5.0/3", "tz", 39, false, false);
    web_client.println("<p class=\"font-small\" style=\"grid-column:1/-1\">A set POSIX TZ rule overrides the fixed UTC offset (DST handled automatically); the UTC offset field then shows the derived value. Empty or &quot;none&quot; = fixed offset only.</p>");
    _create_setup_textinput_element("maxv", "max. Voltage", String(meshcom_settings.node_maxv, 3), "4.125", "maxv", 5, false, false);                 // create Textinput-Element including Label and Button

    web_client.println("</div><div class=\"grid grid2\">");

    _create_setup_switch_element("display", "Display", "keep display active permanently", !bDisplayOff);        // create Switch-Element inclucing Label and Description
    //NOT USED _create_setup_switch_element("small", "small Display", "reduce content for small displays", bSMALLDISPLAY); // create Switch-Element inclucing Label and Description
    _create_setup_switch_element("volt", "Voltage", "show batt. voltage, not percent", bDisplayVolt);           // create Switch-Element inclucing Label and Description
    _create_setup_switch_element("msgwake", "Message wake", "an incoming message turns an off display on", bMsgWake);   // T114-DARK-01 (#1314)
    _create_setup_switch_element("mesh", "Mesh", "enable mesh/forwarding of received LoRa messages", bMESH);    // create Switch-Element inclucing Label and Description
    _create_setup_switch_element("via", "Via", "enable via", bVIA);    // create Switch-Element inclucing Label and Description
    { // LED-08: only on a node that has the LED (the info page says "not fitted" otherwise); same console row as --charge led.
      // The Board LED control lives on the Pins page only (operator 2026-11-07); its /setparam boardled handler stays.
        const LedFitted ledHw = ledFittedNow();
        if (ledHw.charge)
            _create_setup_switch_element("chargeled", "Charge LED", "on = follows charging, off = dark", bCHARGE_LED);
    }
    { // WEB-LOOK-01: look of this web page only (per browser, localStorage); buttons are marked by mcLookMark() after the page loads
        // WEBLOOK-HTML-BEGIN
        static const char *const themeNames[3] = {"auto", "light", "dark"};
        static const char *const fontNames[3] = {"s", "m", "l"};
        web_client.println("<div class=\"mcsegrow\"><label id=\"weblook_theme_lbl\">Web page theme <span class=\"font-small\">(this page, this browser)</span></label>");
        web_client.println("<div class=\"mcledgrp\" role=\"group\" aria-labelledby=\"weblook_theme_lbl\">");
        for (int i = 0; i < 3; i++)
            web_client.printf("<button type=\"button\" class=\"mctab mcledbtn\" id=\"weblook_theme_%s\" aria-pressed=\"false\" onclick=\"mcLookSet('mcTheme','%s')\">%s</button>\n", themeNames[i], themeNames[i], themeNames[i]);
        web_client.println("</div></div>");
        web_client.println("<div class=\"mcsegrow\"><label id=\"weblook_font_lbl\">Web page font size <span class=\"font-small\">(this page, this browser)</span></label>");
        web_client.println("<div class=\"mcledgrp\" role=\"group\" aria-labelledby=\"weblook_font_lbl\">");
        for (int i = 0; i < 3; i++)
            web_client.printf("<button type=\"button\" class=\"mctab mcledbtn\" id=\"weblook_font_%s\" aria-pressed=\"false\" onclick=\"mcLookSet('mcFont','%s')\">%s</button>\n", fontNames[i], fontNames[i], fontNames[i]);
        web_client.println("</div></div>");
        // WEBLOOK-HTML-END
    }
#if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
    { // TD-THEME-01 W3: T-Deck DISPLAY theme (not the web page theme); stored value 0 auto, 1 dark, 2 light; setparam tdecktheme -> --theme
        // TD-HTML-BEGIN
        web_client.println("<div class=\"mcsegrow\"><label id=\"tdtheme_lbl\">T-Deck display theme</label>");
        web_client.println("<div class=\"mcledgrp\" role=\"group\" aria-labelledby=\"tdtheme_lbl\">");
        static const int themeOrder[3] = {0, 2, 1}; // stored 0 auto, 1 dark, 2 light; shown auto - light - dark
        for (int k = 0; k < 3; k++)
        {
            const int i = themeOrder[k];
            const bool on = (meshcom_settings.node_tdeck_theme == i) || (i == 0 && meshcom_settings.node_tdeck_theme > 2);
            web_client.printf("<button type=\"button\" class=\"mctab mcledbtn%s\" id=\"tdtheme_%s\" aria-pressed=\"%s\" onclick=\"tdSet('%s')\">%s</button>\n",
                              on ? " mctab-on" : "", tdeck_theme_name(i), on ? "true" : "false", tdeck_theme_name(i), tdeck_theme_name(i));
        }
        web_client.println("</div></div>");
        // TD-HTML-END
    }
    { // WEB-TD-01: T-Deck font size, display (backlight mode) and brightness, same order as the T-Deck setup page
        // TD2-HTML-BEGIN
        static const char *const fontNames[3] = {"s", "m", "l"};
        static const char *const dispNames[3] = {"auto", "light", "dark"};
        const int fontCur = meshcom_settings.node_tdeck_chatfont > 2 ? 0 : meshcom_settings.node_tdeck_chatfont;
        const int dispCur = (int)brightness_mode_to_seg(brightnessMode());
        web_client.println("<div class=\"mcsegrow\"><label id=\"tdfont_lbl\">T-Deck font size</label>");
        web_client.println("<div class=\"mcledgrp\" role=\"group\" aria-labelledby=\"tdfont_lbl\">");
        for (int i = 0; i < 3; i++)
            web_client.printf("<button type=\"button\" class=\"mctab mcledbtn%s\" id=\"tdfont_%s\" aria-pressed=\"%s\" onclick=\"tdFont('%s')\">%s</button>\n",
                              fontCur == i ? " mctab-on" : "", fontNames[i], fontCur == i ? "true" : "false", fontNames[i], fontNames[i]);
        web_client.println("</div></div>");
        web_client.println("<div class=\"mcsegrow\"><label id=\"tddisp_lbl\">T-Deck display backlight</label>");
        web_client.println("<div class=\"mcledgrp\" role=\"group\" aria-labelledby=\"tddisp_lbl\">");
        for (int i = 0; i < 3; i++)
            web_client.printf("<button type=\"button\" class=\"mctab mcledbtn%s\" id=\"tddisp_%s\" aria-pressed=\"%s\" onclick=\"tdDisp('%s')\">%s</button>\n",
                              dispCur == i ? " mctab-on" : "", dispNames[i], dispCur == i ? "true" : "false", dispNames[i], dispNames[i]);
        web_client.println("</div></div>");
        web_client.printf("<div class=\"mcsegrow\"><label for=\"tdbright\">T-Deck brightness</label><div class=\"mcsegval\"><input type=\"range\" id=\"tdbright\" min=\"1\" max=\"16\" value=\"%d\" oninput=\"document.getElementById('tdbright_val').textContent=this.value\" onchange=\"tdBright(this.value)\"> <span id=\"tdbright_val\">%d</span></div></div>\n",
                          (int)brightnessActiveLevel(), (int)brightnessActiveLevel());
        // TD2-HTML-END
    }
#endif

// We support OTA only for ESP based devices, not RAK
#ifdef ESP32
// We support OTA only for ESP based devices, not RAK
#ifdef ESP32
    web_client.println("<span>Reboot into OTA Updater</span>");
    web_client.println("<button onclick=\"if(confirm('Node will reboot to OTA Updater, are you sure?')){callfunction('otaupdate', '');setTimeout(function(){window.location.reload();},10000);}\"><i class=\"btncheckmark\"></i></button>");
#endif
#endif

    web_client.println("</div></div>");

#if defined(ESP32)
    // AU-09 (#1187): firmware auto update, ONE dropdown off / prod / dev. It applies on change through
    // setparam aumode (web_setup.cpp runs --updchan + --autoupdate, read-back is the stored state).
    // The old params autoupdate / updchan stay valid for the Web API but have no select any more.
    {
        const int au_mode = meshcom_settings.node_autoupd;                 // 1 notify (default), 2 auto (console only, blocked)
        const int au_chan = meshcom_settings.node_updchan ? 1 : 0;         // 0 prod, 1 dev
        const bool au_sb_ok = fwNetSafebootCapable();                      // AU-12: Safeboot new enough for staged updates
        web_client.println("<div class=\"cardlayout collapsablecard\">");
        web_client.println("<label class=\"cardlabel\">Firmware Update</label>");
        web_client.println("<span>Open this for the firmware update settings.</span>\n");
        web_client.println("<button class=\"cardtoggle\" onclick=\"togglecard(this);\"><i></i></button>\n");
        web_client.println("<div class=\"grid grid2\">");
        web_client.println("<label for=\"aumode\">Auto update</label>");
        web_client.printf("<select id=\"aumode\" name=\"aumode\"%s onchange=\"setvalue('aumode',this.value,true);\">\r\n", au_sb_ok ? "" : " disabled");
        // D3: notify (prod), notify (dev), auto greyed. A console-set auto shows greyed and selected.
        web_client.printf("\t<option value=\"prod\" %s>notify (prod, icssw-org releases)</option>\n", (au_mode != 2 && au_chan == 0) ? "selected" : "");
#if defined(MC_AU_DEV_CHANNEL)   // dev channel: fork builds only (fw_update.h)
        web_client.printf("\t<option value=\"dev\" %s>notify (dev, DK5EN releases)</option>\n", (au_mode != 2 && au_chan == 1) ? "selected" : "");
#endif
        web_client.printf("\t<option value=\"auto\" disabled %s>auto (coming soon)</option>\n", (au_mode == 2) ? "selected" : "");
        web_client.println("</select>");
        web_client.println("<p class=\"font-small\" style=\"grid-column:1/-1\">auto: coming soon -- for now the node notifies only. Install new firmware from the Info page.</p>");
        { // install schedule: shown on the Info page, nothing installs by itself (src/fw_schedule.h)
            static const char *const dayVal[] = {"off", "mon", "tue", "wed", "thu", "fri", "sat", "sun"};
            static const char *const dayTxt[] = {"none (use the delay)", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"};
            const int s_day = (meshcom_settings.node_auday >= 0 && meshcom_settings.node_auday <= 7) ? meshcom_settings.node_auday : 0;
            const int s_delay = (meshcom_settings.node_audelay >= 0 && meshcom_settings.node_audelay <= FWSCHED_DELAY_MAX) ? meshcom_settings.node_audelay : FWSCHED_DELAY_DEFAULT;
            const int s_tm = (meshcom_settings.node_autime >= 0 && meshcom_settings.node_autime <= 1439) ? meshcom_settings.node_autime : FWSCHED_TIME_DEFAULT;
            char s_tms[8];
            snprintf(s_tms, sizeof(s_tms), "%02d:%02d", s_tm / 60, s_tm % 60);
            web_client.println("</div><div class=\"grid grid3\">"); // text fields emit label + field + button
            _create_setup_textinput_element("audelay", "Planned install: days after release (0-7)", String(s_delay), "3", "audelay", 1, false, false);
            web_client.println("<label for=\"auday\">Planned install: weekday :</label>");
            web_client.println("<select id=\"auday\" name=\"auday\" style=\"grid-column:2/-1\" onchange=\"setvalue('auday',this.value,true);\">");
            for (int i = 0; i < 8; i++)
                web_client.printf("\t<option value=\"%s\" %s>%s</option>\n", dayVal[i], i == s_day ? "selected" : "", dayTxt[i]);
            web_client.println("</select>");
            _create_setup_textinput_element("autime", "Planned install: time (HH:MM)", String(s_tms), "03:00", "autime", 5, false, false);
            web_client.println("<p class=\"font-small\" style=\"grid-column:1/-1\">The plan is shown on the Info page. A weekday replaces the delay. Automatic install is not active in this firmware: the node does not install by itself, use Download or Install on the Info page.</p>");
        }

        if (!au_sb_ok && !fwNetBootloaderQio())
        {
            // AU-16: a DIO bootloader cannot start the Safeboot; the web flasher renews both
            web_client.println("<p class=\"font-small\" style=\"grid-column:1/-1\">Auto update is locked: the bootloader (DIO) cannot start the Safeboot. Flash this node once with the web flasher to renew bootloader and Safeboot; OTA and auto update never replace them.</p>");
        }
        else if (!au_sb_ok)
        {
            // AU-12: update is locked until Safeboot is new enough; OTA and auto update never replace Safeboot
            const int au_sb_ver = fwNetSafebootVersion();
            char au_sb_found[16];
            if (au_sb_ver >= 1)
                snprintf(au_sb_found, sizeof(au_sb_found), "v%d", au_sb_ver);
            else
                snprintf(au_sb_found, sizeof(au_sb_found), "%s", au_sb_ver == 0 ? "none (old)" : "unreadable");
            web_client.printf("<p class=\"font-small\" style=\"grid-column:1/-1\">Auto update needs a newer Safeboot (found: %s, required: v%d). Flash this node once with the web flasher to install it; OTA and auto update never replace Safeboot.</p>\n",
                au_sb_found, AU_SAFEBOOT_MIN);
        }
        web_client.println("</div>");
        web_client.println("</div>");
    }
#endif

    // NMTU-01 / #1190: MTU card, collapsible like every other card (UI-04 / D5), all boards. Applies on change through
    // setparam mtu (--mtu, read-back node_ethmtu); the MSS shown is MTU - 40. A stored value outside the
    // three presets (console --mtu 1350) gets its own selected "custom" option so the page never lies.
    {
        const int mtu_cur = meshcom_settings.node_ethmtu;
        const bool mtu_preset = (mtu_cur == 1280 || mtu_cur == 1400 || mtu_cur == 1500);
        web_client.println("<div class=\"cardlayout collapsablecard\">");
        web_client.println("<label class=\"cardlabel\">MTU</label>");
        web_client.println("<span>Open this for the Ethernet MTU.</span>\n");
        web_client.println("<button class=\"cardtoggle\" onclick=\"togglecard(this);\"><i></i></button>\n");
        web_client.println("<div class=\"grid grid3\">");
        web_client.println("<label for=\"mtu\">MTU</label>");
        web_client.println("<select id=\"mtu\" name=\"mtu\" onchange=\"document.getElementById('mtu_mss').textContent='MSS '+(parseInt(this.value)-40);setvalue('mtu',this.value,true);\">");
        web_client.printf("\t<option value=\"1280\" %s>1280 (HAMNET / VPN, default)</option>\n", (mtu_cur == 1280) ? "selected" : "");
        web_client.printf("\t<option value=\"1400\" %s>1400</option>\n", (mtu_cur == 1400) ? "selected" : "");
        web_client.printf("\t<option value=\"1500\" %s>1500 (plain LAN)</option>\n", (mtu_cur == 1500) ? "selected" : "");
        if (!mtu_preset)
            web_client.printf("\t<option value=\"%i\" selected>custom (%i)</option>\n", mtu_cur, mtu_cur);
        web_client.println("</select>");
        web_client.printf("<span id=\"mtu_mss\">MSS %i</span>\n", mtu_cur - 40);
        web_client.println("</div>");
        web_client.println("</div>");
    }

    // Remote Admin card: password and remote management of this node (moved from the Remote page). Filled by raSt().
    // RA-HTML-BEGIN
    web_client.println("<div class=\"cardlayout collapsablecard\">");
    web_client.println("<label class=\"cardlabel\">Remote Admin</label>");
    web_client.println("<span>Open this for the password and remote management.</span>\n");
    web_client.println("<button class=\"cardtoggle\" onclick=\"togglecard(this);\"><i></i></button>\n");
    web_client.println("<div class=\"grid grid3\">");
    web_client.println("<label for=\"ra_on\">Remote management</label><span></span>"
                       "<input type=\"checkbox\" role=\"switch\" id=\"ra_on\" disabled onchange=\"raOn()\">"
                       "<span id=\"ra_hint\" class=\"font-small\" style=\"grid-column:1/-1\"></span>"
                       "<style>#ra_hint:empty,#ra_msg:empty{display:none}</style>");
    web_client.println("<label for=\"ra_pw\">Password :</label>"
                       "<input type=\"password\" id=\"ra_pw\" maxlength=\"14\" size=\"10\" autocomplete=\"new-password\" placeholder=\"New password\">"
                       "<button id=\"ra_set\" onclick=\"raSet()\">Set</button>");
    web_client.println("<span id=\"ra_st\" class=\"font-small\" style=\"grid-column:1/3\"></span><button id=\"ra_clr\" onclick=\"raClr()\">Clear</button>"
                       "<p id=\"ra_msg\" class=\"font-small\" style=\"grid-column:1/-1\"></p>"
                       "<p class=\"font-small\" style=\"grid-column:1/-1\">This password also protects the net console (port 2323) and the KISS port, not only remote management.</p>");
    web_client.println("</div></div>");
    // RA-HTML-END

    // Web login password card (WEB-PWD-01): the login of this web page, not the RM/KISS password of the card above.
    // WP-HTML-BEGIN
    web_client.println("<div class=\"cardlayout collapsablecard\">");
    web_client.println("<label class=\"cardlabel\">Web login password</label>");
    web_client.println("<span>Open this for the password that protects this web page.</span>\n");
    web_client.println("<button class=\"cardtoggle\" onclick=\"togglecard(this);\"><i></i></button>\n");
    web_client.println("<div class=\"grid grid3\">");
    web_client.println("<p class=\"font-small\" style=\"grid-column:1/-1\">This is the login of this web page. It is not the Remote Admin password (RM, net console, KISS) above.</p>"
                       "<style>#wp_msg:empty{display:none}</style>");
    web_client.println("<label for=\"wp_pw\">New password :</label>"
                       "<input type=\"password\" id=\"wp_pw\" maxlength=\"19\" size=\"10\" autocomplete=\"new-password\" placeholder=\"New web password\"><span></span>"
                       "<label for=\"wp_pw2\">Repeat :</label>"
                       "<input type=\"password\" id=\"wp_pw2\" maxlength=\"19\" size=\"10\" autocomplete=\"new-password\" placeholder=\"Repeat\">"
                       "<button id=\"wp_set\" onclick=\"wpSet()\">Set</button>");
    web_client.print("<span id=\"wp_st\" class=\"font-small\" style=\"grid-column:1/3\">");
#define WP_ST_SET "web login password: set"
#define WP_ST_NOT "web login password: not set"
    web_client.print(strlen(meshcom_settings.node_webpwd) > 0 ? WP_ST_SET : WP_ST_NOT);   // only set / not set, never the password
    web_client.println("</span><button id=\"wp_clr\" onclick=\"wpClr()\">Clear</button>"
                       "<p id=\"wp_msg\" class=\"font-small\" style=\"grid-column:1/-1\"></p>"
                       "<p class=\"font-small\" style=\"grid-column:1/-1\">1 to 19 plain characters. After setting or changing it you are logged out and log in again.</p>");
    web_client.println("</div></div>");
    // WP-HTML-END

    // IP Network Settings Section
    web_client.println("<div class=\"cardlayout collapsablecard\">");
    web_client.println("<label class=\"cardlabel\">IP Network Settings</label>");
    web_client.println("<span>Open this for network-specific settings.</span>\n");
    web_client.println("<button class=\"cardtoggle\" onclick=\"togglecard(this);\"><i></i></button>\n");
    web_client.println("<div class=\"grid grid3\">");

    #ifndef BOARD_RAK4630
    _create_setup_textinput_element("wifissid", "SSID", String(meshcom_settings.node_ssid), "wifi-name", "setssid", 50, false, true);  // create Textinput-Element including Label and Button
    _create_setup_textinput_element("wifipassword", "WiFi Password", String(meshcom_settings.node_pwd), "", "setpwd", 50, true, true); // create Textinput-Element including Label and Button
    #endif

    _create_setup_textinput_element("ownip", "fixed IP", String(meshcom_settings.node_ip), "192.168.2.100", "setownip", 50, false, true);        // create Textinput-Element including Label and Button
    _create_setup_textinput_element("ownsn", "Subnet Mask", String(meshcom_settings.node_subnet), "255.255.255.0", "setownms", 50, false, true); // create Textinput-Element including Label and Button
    _create_setup_textinput_element("owngw", "Gateway", String(meshcom_settings.node_gw), "192.168.2.1", "setowngw", 50, false, true);           // create Textinput-Element including Label and Button
    _create_setup_textinput_element("owndns", "DNS", String(meshcom_settings.node_dns), "192.168.2.1", "setowndns", 50, false, true);             // create Textinput-Element including Label and Button
    _create_setup_textinput_element("ownntp", "NTP", String(meshcom_settings.node_ownntp), "192.168.2.1", "setownntp", 50, false, true);          // create Textinput-Element including Label and Button

    _create_setup_textinput_element("extudpip", "ext. UDP IP", String(meshcom_settings.node_extern), "192.168.100.100", "extudpip", 50, false, false); // create Textinput-Element including Label and Button

    web_client.println("</div><div class=\"grid grid2\">");
    #if defined(HAS_ETHERNET)
    _create_setup_switch_element("netmode", "Ethernet Mode", "switch between WiFi and Ethernet", meshcom_settings.node_netmode == 1);
    #endif
    _create_setup_switch_element("extudp", "ext UDP", "enable ext. UDP", bEXTUDP); // create Switch-Element inclucing Label and Description
    #if !defined(BOARD_RAK4630) && !defined(DISABLE_NET_CONSOLE)
    _create_setup_switch_element("netconsole", "net console", "enable net console (port 2323, HMAC auth)", bNETCONSOLE); // create Switch-Element inclucing Label and Description
    #endif
    #if defined(ESP32) && !defined(DISABLE_KISS_TCP)
    _create_setup_switch_element("kiss", "KISS/TCP", "enable KISS interface (port 8001)", bKISS);
    _create_setup_switch_element("kisstx", "KISS TX", "allow transmit from KISS clients", bKISSTX);
    _create_setup_switch_element("kissmeta", "KISS RxMeta", "send RSSI/SNR frames (KISS port 1)", bKISSMETA);
    _create_setup_switch_element("kissauth", "KISS Auth", "require HMAC auth on port 8001 (uses --passwd)", bKISSAUTH);
    #endif
    _create_setup_switch_element("gateway", "Gateway", "enable gateway", bGATEWAY);   // create Switch-Element inclucing Label and Description
    // BF-01: remote-management brute-force protection (--rmstrictsecurity). Console and web only, never over RM.
    _create_setup_switch_element("rmstrict", "RM strict security", "brute-force protection: lockout and rate limits for remote management", meshcom_settings.node_rmstrict == 1);
    web_client.println("</div></div>");

    // Position Settings Section
    web_client.println("<div class=\"cardlayout collapsablecard\">");
    web_client.println("<label class=\"cardlabel\">Position Settings</label>");
    web_client.println("<span>Open this for position-specific settings.</span>\n");
    web_client.println("<button class=\"cardtoggle\" onclick=\"togglecard(this);\"><i></i></button>\n");
    web_client.println("<div class=\"grid grid3\">");

    _create_setup_textinput_element("latitude", "Position Latitude (+/-)", String(meshcom_settings.node_lat, 8), "48.26940877", "setlat", 12, false, false);   // create Textinput-Element including Label and Button
    _create_setup_textinput_element("longitude", "Position Longitude (+/-)", String(meshcom_settings.node_lon, 8), "16.40922749", "setlon", 12, false, false); // create Textinput-Element including Label and Button
    _create_setup_textinput_element("altitude", "Position Altitude (meter)", String(meshcom_settings.node_alt), "300", "setalt", 5, false, false);             // create Textinput-Element including Label and Button

    web_client.println("</div><div class=\"grid grid2\">");

    #if defined (ENABLE_GPS) or defined(BOARD_RAK4630) or defined(BOARD_HELTEC_T114) or defined(BOARD_T_ECHO)
    _create_setup_switch_element("gps", "GPS", "enable GPS", bGPSON);                                  // create Switch-Element inclucing Label and Description
    #endif
    _create_setup_switch_element("track", "Track", "enable display of SmartBeaconing", bDisplayTrack, TRACK_WARNING_TEXT, bDisplayTrack); // create Switch-Element inclucing Label and Description; TRK-01: Warnhinweis neben dem Switch

    web_client.println("</div></div>");

    // APRS Settings Section
    web_client.println("<div class=\"cardlayout collapsablecard\">");
    web_client.println("<label class=\"cardlabel\">APRS Settings</label>");
    web_client.println("<span>Open this for APRS-specific settings.</span>\n");
    web_client.println("<button class=\"cardtoggle\" onclick=\"togglecard(this);\"><i></i></button>\n");
    web_client.println("<div class=\"grid grid3\">");

    _create_setup_textinput_element("nametext", "Operator Name", String(meshcom_settings.node_name), "aprsname", "setname", 25, false, false); // create Textinput-Element including Label and Button
    _create_setup_textinput_element("aprstext", "APRS Comment", String(meshcom_settings.node_atxt), "aprstext", "atxt", 25, false, false);    // create Textinput-Element including Label and Button
    _create_setup_textinput_element("aprssymbol", "APRS Group", String(meshcom_settings.node_symid), "S", "symid", 1, false, false);      // create Textinput-Element including Label and Button
    _create_setup_textinput_element("aprsgroup", "APRS Symbol", String(meshcom_settings.node_symcd), "/", "symcd", 1, false, false);        // create Textinput-Element including Label and Button

    web_client.println("</div></div>");

    // Hardware Settings Section
    web_client.println("<div class=\"cardlayout collapsablecard\">");
    web_client.println("<label class=\"cardlabel\">External Hardware Settings</label>");
    web_client.println("<span>Open this for hardware-related settings like sensors.</span>\n");
    web_client.println("<button class=\"cardtoggle\" onclick=\"togglecard(this);\"><i></i></button>\n");
    web_client.println("<div class=\"grid grid3\">");

    #ifdef OneWire_GPIO
    _create_setup_textinput_element("owgpio", "1-Wire GPIO", String(meshcom_settings.node_owgpio), "36", "onewiregpio", 3, false, false); // create Textinput-Element including Label and Button
    #endif
    #if defined(ENABLE_WIND)
    _create_setup_textinput_element("windpin", "Anemometer GPIO", String(meshcom_settings.node_wind_pin), "0", "windpin", 2, false, false);
    _create_setup_textinput_element("windcal", "Anemometer km/h per Hz", String(meshcom_settings.node_wind_cal, 2), "2.4", "windcal", 6, false, false);
    #endif

    web_client.println("</div><div class=\"grid grid2\">");

    #ifdef OneWire_GPIO
    _create_setup_switch_element("onewire", "1-Wire", "enable 1-Wire capability", bONEWIRE); // create Switch-Element inclucing Label and Description
    #endif

    web_client.println("</div>");
    web_client.println("<div class=\"grid grid3\">");

    #ifndef BOARD_T_DECK_PRO
    int iButtonPin = 0;
    #ifdef BUTTON_PIN
        iButtonPin = BUTTON_PIN;
    #else
        iButtonPin = 99;
    #endif

    if(meshcom_settings.node_button_pin > 0)
        iButtonPin = meshcom_settings.node_button_pin;

    _create_setup_textinput_element("ubgpio", "Userbutton GPIO", String(iButtonPin), "0", "buttongpio", 3, false, false); // create Textinput-Element including Label and Button
    #endif

    web_client.println("</div>");
    web_client.println("<div class=\"grid grid2\">");

    _create_setup_switch_element("button", "Userbutton", "enable user-button", bButtonCheck); // create Switch-Element inclucing Label and Description

    web_client.println("</div>");

    web_client.println("<div class=\"grid grid3\">");

    #if defined(ANALOG_PIN)
    _create_setup_textinput_element("angpio", "Analog GPIO", String(meshcom_settings.node_analog_pin), "2", "angpio", 3, false, false);                // create Textinput-Element including Label and Button
    _create_setup_textinput_element("afactor", "Analog Factor", String(meshcom_settings.node_analog_faktor, 6), "1.000", "afactor", 10, false, false); // create Textinput-Element including Label and Button
    _create_setup_textinput_element("aslope", "Analog Slope", String(meshcom_settings.node_analog_slope, 6), "1.000", "aslope", 10, false, false); // create Textinput-Element including Label and Button
    _create_setup_textinput_element("aoffset", "Analog Offset", String(meshcom_settings.node_analog_offset, 6), "0", "aoffset", 3, false, false); // create Textinput-Element including Label and Button
    #endif

    web_client.println("</div>");
    web_client.println("<div class=\"grid grid2\">");

    #if defined(ANALOG_PIN)
    _create_setup_switch_element("analogcheck", "Analog", "enable analog GPIO measurement", bAnalogCheck); // create Switch-Element inclucing Label and Description
    #endif
    #if defined(ENABLE_BMX280)
    _create_setup_switch_element("bmp", "BMP280", "enable BMP280 sensor", bBMPON);                         // create Switch-Element inclucing Label and Description
    _create_setup_switch_element("bme", "BME280", "enable BME280 sensor", bBMEON);                         // create Switch-Element inclucing Label and Description
    _create_setup_switch_element("680", "BME680", "enable BME680 sensor", bBME680ON);                      // create Switch-Element inclucing Label and Description
    _create_setup_switch_element("811", "MCU811", "enable MCU811 sensor", bMCU811ON);                      // create Switch-Element inclucing Label and Description
    #endif
    #if defined (ENABLE_INA226)
    _create_setup_switch_element("ina226", "INA226", "enable INA226 sensor", bINA226ON);                   // create Switch-Element inclucing Label and Description
    #endif
    #if defined(ENABLE_AHT20)
    _create_setup_switch_element("aht20", "AHT20", "enable AHT20 sensor", bAHT20ON);                       // create Switch-Element inclucing Label and Description
    #endif
    #if defined(ENABLE_LTR390)
    _create_setup_switch_element("uv", "UV (LTR390)", "enable LTR390 UV sensor", bUVON);
    #endif
    #if defined(ENABLE_WIND)
    _create_setup_switch_element("wind", "Anemometer", "enable anemometer (reed contact)", bWINDON);
    #endif
    #if defined(ENABLE_SHT21)
    _create_setup_switch_element("sht21", "SHT21", "enable SHT21 sensor", bSHT21ON);                       // create Switch-Element inclucing Label and Description
    #endif
    #if defined(ENABLE_SOFTSER)
    _create_setup_switch_element("softser", "SoftSer", "enable software serial console", bSOFTSERON);      // create Switch-Element inclucing Label and Description
    #endif

    web_client.println("</div>");
    
    web_client.println("<div class=\"grid grid3\">");
    _create_setup_textinput_element("tempoffi", "Indoor Temp Offset", String(meshcom_settings.node_tempi_off), "0.0", "tempoffsetindoor", 6, false, false);                // create Textinput-Element including Label and Button
    _create_setup_textinput_element("tempoffa", "Outdoor Temp Offset", String(meshcom_settings.node_tempo_off), "0.0", "tempoffsetoutdoor", 6, false, false); // create Textinput-Element including Label and Button
    web_client.println("</div>");

    web_client.println("</div>");

    // Groups Settings Section
    web_client.println("<div class=\"cardlayout collapsablecard\">");
    web_client.println("<label class=\"cardlabel\">Groups / Messaging</label>");
    web_client.println("<span>Open this for messaging-groups-related settings.</span>\n");
    web_client.println("<button class=\"cardtoggle\" onclick=\"togglecard(this);\"><i></i></button>\n");
    web_client.println("<div class=\"grid grid2\">");

    for (int i = 0; i < (int)sizeof(meshcom_settings.node_gcb) / (int)sizeof(meshcom_settings.node_gcb[0]); i++)
    {
        web_client.printf("<label for=\"grp%i\">Listen to Group:</label>\n", i);
        web_client.printf("<input type=\"text\" id=\"grp%i\" value=\"%i\" maxlength=\"5\" size=\"20\" placeholder=\"0\"/>\n", i, meshcom_settings.node_gcb[i]);
    }
    web_client.println("<i></i>");
    web_client.println("<button onclick=\"setvalue('setgrc', (document.getElementById('grp0').value+';'+document.getElementById('grp1').value+';'+document.getElementById('grp2').value+';'+document.getElementById('grp3').value+';'+document.getElementById('grp4').value+';'+document.getElementById('grp5').value),false)\"><i class=\"btncheckmark\"></i></button>");

    _create_setup_switch_element("nomsgall", "No MSG All", "do not show messages send to all", bNoMSGtoALL); // create Switch-Element inclucing Label and Description

    web_client.println("</div>");
    web_client.println("</div>");

#if defined(ENABLE_MSGSTORE)
    // Store node card (stage 3, docs/dm-stage3-wave-plan-20260914.md / M7). The mode select is
    // hand-built like --maxhop above rather than _create_setup_switch_element, because it has
    // four states and switching away from "off" must confirm() first and revert the select on
    // cancel -- a plain on/off switch can't carry that. storecall/storetime/storeslots are
    // ordinary text settings and do fit _create_setup_textinput_element.
    web_client.println("<div class=\"cardlayout collapsablecard\">");
    web_client.println("<label class=\"cardlabel\">Store node</label>");
    web_client.println("<span>Open this for the message store node settings.</span>\n");
    web_client.println("<button class=\"cardtoggle\" onclick=\"togglecard(this);\"><i></i></button>\n");
    web_client.println("<div class=\"cardbody\">"); // one body div: the sub-grids below live inside it
    web_client.println("<div class=\"grid grid2\">");
    {
        static const char *s_storemode_val[4] = {"off", "own", "list", "heard"};
        static const char *s_storemode_lbl[4] = {"off", "own callsign, all SSIDs", "callsign list", "every station heard directly (12 h)"};
        const char *cur_mode = msgstoreModeName(msgstoreMode());

        web_client.println("<label for=\"storemode\">Store messages for</label>");
        web_client.printf("<select id=\"storemode\" name=\"storemode\" data-prev=\"%s\" onchange=\"var s=this;if(s.value!='off' && !confirm('This node must run 24/7 on continuous power. Stored messages live in RAM only; a reboot discards all of them without notice.')){s.value=s.getAttribute('data-prev');return;}s.setAttribute('data-prev',s.value);setvalue('store',s.value,true);\">\n", cur_mode);
        for (int im = 0; im < 4; im++)
        {
            web_client.printf("\t<option value=\"%s\" %s>%s</option>\n", s_storemode_val[im], (strcmp(s_storemode_val[im], cur_mode) == 0) ? "selected" : "", s_storemode_lbl[im]);
        }
        web_client.println("</select>");
    }
    // Textfelder liefern Label + Feld + Knopf, brauchen also grid3 (wie die Node-Karte oben).
    web_client.println("</div><div class=\"grid grid3\">");
    _create_setup_textinput_element("storecall", "Callsign list (list mode, up to 16)", msgstoreListCsv(), "OE1KBC-4,DK5EN-14", "storecall", MSGSTORE_LIST_MAX * MSGSTORE_CALL_MAX, false, false);
    _create_setup_textinput_element("storetime", "Hold time in hours (1 to 168)", String(msgstoreHoldHours()), "24", "storetime", 3, false, false);
    _create_setup_textinput_element("storeslots", "Slots (1 to 50)", String(msgstoreSlots()), "50", "storeslots", 3, false, false);
    web_client.println("</div><div class=\"grid grid2\">");
    _create_setup_switch_element("storenotice", "Notify sender", "tell the sender when a message is held", msgstoreNotice()); // stage 4: --storenotice on|off
    web_client.println("</div>");
    web_client.printf("<div class=\"mbx-warn\"><b>Before you switch this on.</b> This node must run 24/7 on continuous power. Stored messages live in RAM only; a reboot discards all of them without notice, and nobody is told. About %.1f kB of RAM is reserved for %u slots.</div>\n",
                       (float)(msgstoreSlots() * sizeof(struct MsgStoreEntry)) / 1024.0f, (unsigned)msgstoreSlots());
    web_client.println("</div>"); // cardbody
    web_client.println("</div>");
#endif

    // BLK-01: blocked callsigns card (not tied to the store node).
    web_client.println("<div class=\"cardlayout collapsablecard\">");
    web_client.println("<label class=\"cardlabel\">Blocked callsigns</label>");
    web_client.println("<span>Open this to drop frames from unwanted stations.</span>\n");
    web_client.println("<button class=\"cardtoggle\" onclick=\"togglecard(this);\"><i></i></button>\n");
    web_client.println("<div class=\"cardbody\">");
    web_client.println("<div class=\"grid grid3\">");
    {
        char blkcsv[BLOCKLIST_CSV_MAX];
        blockListGet(blkcsv, sizeof(blkcsv));
        _create_setup_textinput_element("blockcall", "Callsign list (up to 16)", String(blkcsv), "DL1ABC,DL2XYZ-12", "blockcall", BLOCKLIST_CSV_MAX + BLOCKLIST_MAX, false, false);
    }
    web_client.println("</div>");
    web_client.println("<div class=\"mbx-warn\">Frames from, or relayed by, a listed call are dropped: not shown, not uploaded, not relayed. A call without SSID blocks all its SSIDs. An empty field clears the list.</div>");
    web_client.println("</div>"); // cardbody
    web_client.println("</div>");

    // Config Backup / Restore Section (CS-03)
    // The download is a plain navigation to /config.json -- the response
    // carries a Content-Disposition, so the browser saves it instead of
    // rendering it. uploadconfig() lives in the scaffold's script block
    // (this page is injected with innerHTML, an inline <script> would not run).
    web_client.println("<div class=\"cardlayout collapsablecard\">");
    web_client.println("<label class=\"cardlabel\">Config Backup / Restore</label>");
    web_client.println("<span>Open this to save the whole node configuration to a file, or to restore it.</span>\n");
    web_client.println("<button class=\"cardtoggle\" onclick=\"togglecard(this);\"><i></i></button>\n");
    web_client.println("<div class=\"grid grid2\">");
    web_client.println("<span><b>The file contains secrets in clear text</b> &ndash; the WiFi password, the web password and the BT code. Keep it like a password store.</span><i></i>");
    web_client.println("<span>Download the configuration as JSON</span>");
    web_client.println("<button onclick=\"window.location='/config.json';\">download</button>");
    web_client.println("<span>Restore a configuration file. The node checks the layout version and the checksum, applies all values at once and then reboots.</span><i></i>");
    web_client.println("<input type=\"file\" id=\"cfgfile\" accept=\".json,application/json\">");
    web_client.println("<button onclick=\"uploadconfig();\">restore</button>");
    web_client.println("</div></div>");

    web_client.println("</div>");
    web_client.println(); // The HTTP response ends with another blank line
}

/**
 * ###########################################################################################################################
 * This will only deliver the preformatted messages to be loaded asyncronous into the WebUI scaffold
 */
// Die Seite liest den VERLAUF des Telefon-Rings, nicht die ungelesenen
// Frames: ein Frame bleibt nach dem Senden ans Telefon liegen, bis sein Platz
// gebraucht wird (byte_fifo.h, "oldest"). Vorher lief die Schleife ueber alle
// MAX_RING Schlitze ab toPhoneWrite -- dasselbe, nur dass der Verlauf jetzt
// so viele Frames haelt, wie in RING_BYTES_PHONE passen. Upstream origin
// 87c6c200.
void sub_content_messages()
{
    int rendered = 0;
    bf_iter_t it;
    bf_iter_begin(&phoneRing, &it);

    if (bDEBUG)
        Serial.printf("phoneRing frames:%u unread:%u\n", (unsigned)bf_frames(&phoneRing), (unsigned)bf_unread(&phoneRing));

    for (;;)
    {
        uint8_t frame[256];
        uint8_t blelen = bf_iter_next(&phoneRing, &it, frame, sizeof(frame));
        if (blelen == 0)
            break;

        if (bDEBUG)
            Serial.printf("frame len:%u [0]:%02X\n", blelen, frame[0]);

        uint8_t toPhoneBuff[MAX_MSG_LEN_PHONE] = {0}; // we need to insert the first byte text msg flag

        if (frame[0] == 0x91)
        { // Mheard
        }
        else if (frame[0] == 0x44)
        { // Data Message (JSON)
        }
        else if (blelen >= 4 && (size_t)(blelen - 4) <= sizeof(toPhoneBuff))
        { // Text Message and Position
            uint8_t tbuffer[5];
            unsigned long unix_time = 0;
            char timestamp[21];
            String ccheck = "";

            memcpy(toPhoneBuff, frame, blelen - 4);
            memcpy(tbuffer, frame + (blelen - 4), 4);
            unix_time = (tbuffer[0] << 24) | (tbuffer[1] << 16) | (tbuffer[2] << 8) | tbuffer[3];
            time_t unix_t = (time_t)(unix_time + (long)(meshcom_settings.node_utcoff * 60 * 60));
            struct tm *oldt = gmtime(&unix_t);
            strftime(timestamp, 20, "%Y-%m-%d %H:%M:%S", oldt);
            struct aprsMessage aprsmsg;
            uint8_t msg_type_b_lora = decodeAPRS(toPhoneBuff, blelen, aprsmsg); // print which message type we got

            // Textmessage
            if (msg_type_b_lora == 0x3A)
            {
                // {CET} time beacons sit in the ring for the phone app's clock
                // sync; they are not operator traffic and would light the tab
                // badges on every beacon, so the web list skips them
                if (mcIndexOfStr(aprsmsg.msg_payload, ":ack") < 1 && !mcStartsWith(aprsmsg.msg_payload, "{CET}"))
                {
                    String msgtxt = aprsmsg.msg_payload;
                    if (bDEBUG)
                        Serial.printf("aprsmsg.msg_source_call:%s, aprsmsg.msg_gateway_call:%s, aprsmsg.msg_destination_call:%s, aprsmsg.msg_payload:%s\n", aprsmsg.msg_source_call, aprsmsg.msg_source_last, aprsmsg.msg_destination_call, aprsmsg.msg_payload);

                    // strip only the trailing "{NNN" ACK marker; a '{' inside the text stays (msg_body.h)
                    msgtxt = msgtxt.substring(0, msg_body_visible_len(msgtxt.c_str(), msgtxt.length()));

                    // WEB-03a: mesh-derived strings (payload, path, callsigns) are attacker-controlled -- escape before HTML output
                    String msgtxt_esc = htmlEscape(msgtxt);
                    String msg_source_path_esc = htmlEscape(aprsmsg.msg_source_path);
                    String msg_destination_path_esc = htmlEscape(aprsmsg.msg_destination_path);

                    // own messages (source == us): the browser's DM tab keys on the destination call
                    if (is_equ(meshcom_settings.node_call, aprsmsg.msg_source_call))
                    {
                        String dst_esc = htmlEscape(aprsmsg.msg_destination_call);

                        // The tick comes from the durable own-text status table first: own_msg_id is a
                        // 20-slot ring shared with every own frame (positions, acks, gateway forwards)
                        // and loses the status within the hour. checkOwnTx stays as the fallback for
                        // ids the table does not know (sent before boot of this image / not registered).
                        // docs/webgui-ack-ticks-verdict-20261004.md, Finding 1.
                        int tickst = ownMsgStatusGet(aprsmsg.msg_id);
                        if (tickst < 0)
                        {
                            int icheck = checkOwnTx(aprsmsg.msg_id);
                            if (icheck >= 0)
                                tickst = own_msg_id[icheck][4];
                        }

                        // 00...not heard, 01...heard, 02...ACK, 03...failed (retransmit gave up on a user-originated DM), 04...held (a store node holds this DM)
                        // The span is machine-readable (data-st) so the browser can keep a tick the server no longer knows (mcTickKeep).
                        if (tickst == 1)
                            ccheck = "<span class=\"mctick\" data-st=\"1\">&#x2713;&nbsp;</span>";
                        else if (tickst == 2)
                            ccheck = "<span class=\"mctick\" data-st=\"2\">&#x2611;&nbsp;</span>";
                        else if (tickst == 3)
                            ccheck = "<span class=\"mctick\" data-st=\"3\" title=\"delivery failed\">&#x2717;&nbsp;</span>";
                        else if (tickst == 4)
                        {
                            String holder = stoHolder(aprsmsg.msg_id);
                            String holdtitle = holder.length() > 0 ? ("held by " + htmlEscape(holder)) : "held by a store node";
                            ccheck = "<span class=\"mctick\" data-st=\"4\" title=\"" + holdtitle + "\">&#x2709;&nbsp;</span>";
                        }

                        web_client.printf("<div class=\"message message-send\" data-id=\"%u\" data-dst=\"%s\" data-ts=\"%lu\"><div>", aprsmsg.msg_id, dst_esc.c_str(), unix_time);

                        web_client.printf("<p class=\"font-small font-bold\">%s", ccheck.c_str());
                        web_client.printf("<a target=\"_blank\" href=\"https://aprs.fi/?call=%s\">%s</a>", msg_source_path_esc.c_str(), msg_source_path_esc.c_str());
                        web_client.printf("%s%s</p>", (char *)">", msg_destination_path_esc.c_str());

                        web_client.printf("<p class=\"font-small font-bold\">%s</p>", timestamp);
                        web_client.printf("<p class=\"font-normal\">%s</p>", msgtxt_esc.c_str());
                        web_client.printf("</div></div>");
                    }
                    // messages by others: "*" and group numbers key on the destination call as-is,
                    // a DM to us keys on the source call so the DM tab shows both directions
                    else
                    {
                        bool isGroupDst = is_equ(aprsmsg.msg_destination_call, "*");
                        if (!isGroupDst && strlen(aprsmsg.msg_destination_call) > 0)
                        {
                            isGroupDst = true;
                            for (unsigned int ci = 0; ci < strlen(aprsmsg.msg_destination_call); ci++)
                            {
                                if (!isDigit(aprsmsg.msg_destination_call[ci]))
                                {
                                    isGroupDst = false;
                                    break;
                                }
                            }
                        }
                        String dst_esc = htmlEscape(isGroupDst ? aprsmsg.msg_destination_call : aprsmsg.msg_source_call);

                        web_client.printf("<div class=\"message message-received\" data-id=\"%u\" data-dst=\"%s\" data-ts=\"%lu\"><div>", aprsmsg.msg_id, dst_esc.c_str(), unix_time);

                        // no delivery tick on received frames: a gateway keeps server-forwarded foreign frames in own_msg_id and they showed a false tick (Finding 2)
                        web_client.printf("<p class=\"font-small font-bold\">");
                        web_client.printf("<a target=\"_blank\" href=\"https://aprs.fi/?call=%s\">%s</a>", msg_source_path_esc.c_str(), msg_source_path_esc.c_str());
                        web_client.printf("%s%s</p>", (char *)">", msg_destination_path_esc.c_str());

                        web_client.printf("<p class=\"font-small font-bold\">%s</p>", timestamp);
                        web_client.printf("<p class=\"font-normal\">%s</p>", msgtxt_esc.c_str());
                        web_client.printf("</div></div>");
                    }

                    rendered++;
                }
            }
        }
    }

    if (rendered == 0)
    {
        web_client.printf("<p>No messages available.</p>");
    }

    web_client.println(); // The HTTP response ends with another blank line
}

/**
 * ###########################################################################################################################
 * delivers the ispectrumpage to be injected into the scaffold
 */
void sub_page_spectrum()
{
    _create_meshcom_subheader("Spectrum Scan");
    web_client.println("<div id=\"content_inner\">");
#if defined(SX1262X) || defined(SX126X) || defined(SX1262_V3) || defined(SX1262_E290) || defined(USING_SX1262)

    // Local values only: the page used to overwrite (and persist) the node_spec* settings, which
    // broke the production-locked 432/434/0.025/2048 (docs/production-locked-settings.md).
    float spec_start = 432.0;
    float spec_end = 434.0;
    float spec_step = 0.025;
    int spec_samples = 2048;

    if(getFreq() > 860)
    {
        spec_start = 863.0;
        spec_end = 870.0;
        spec_step = 0.100;
        spec_samples = 2048;
    }

    float spec_curr_freq = spec_start; // scan start frequency

    uint16_t step_pixel_width = 10;  // the amout of pixel we use for a single frequency step
    uint16_t step_pixel_height = 10; // the amout of pixel we use for a single frequency step

    uint16_t num_fsteps = roundf((spec_end - spec_curr_freq) / spec_step);
    #if not defined(BOARD_T_DECK_PRO)
    uint16_t current_fStep = 0;                                       // current iteration  counter
    #endif
    uint16_t start_x = 60;                                            // x-position where the diagramm starts
    uint16_t start_y = 10;                                            // y-position where the diagramm starts
    uint16_t end_x = start_x + ((num_fsteps + 1) * step_pixel_width); // calculate the end. Use one more fstep as the last frequency step also starts a scan and giving a result
    uint16_t end_y = start_y + (RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE * step_pixel_height);

    uint16_t own_freq_marker_width = round(((meshcom_settings.node_bw / 1000) / spec_step) * step_pixel_width);
    uint16_t own_freq_marker_center = start_x + (((meshcom_settings.node_freq - spec_curr_freq) / spec_step) * step_pixel_width);
    uint16_t own_freq_marker_start = own_freq_marker_center - (own_freq_marker_width / 2);



    #if defined(BOARD_T_DECK_PRO)
        web_client.println("<p>unable to initialize spectrum scan</p>");
    #else
    if (sx126x_spectral_init_scan(spec_curr_freq) != RADIOLIB_ERR_NONE)
    {
        web_client.println("<p>unable to initialize spectrum scan</p>");
    }
    else
    #endif
    {
        web_client.printf("<svg viewbox=\"0, 0, %d, %d\" id=\"spectrum_display\" style=\"font-size:var(--fs-s);\">", end_x + 40, end_y + 60);

        web_client.printf("<line x1=\"%d\" y1=\"%d\" x2=\"%d\" y2=\"%d\" style=\"stroke:black;stroke-width:1\"/>\n", start_x, end_y, end_x, end_y);                    // X-Line at bottom
        web_client.printf("<line x1=\"%d\" y1=\"%d\" x2=\"%d\" y2=\"%d\" style=\"stroke:black;stroke-width:1\"/>\n", start_x, start_y, start_x, end_y);                // Y-Line at left
        web_client.printf("<text x=\"%d\" y=\"%d\" style=\"color: black;\">Freq [MHz]</text>\n", start_x, end_y + 40);                                // caption for X-Line (frequency)
        web_client.printf("<text x=\"0\" y=\"0\" f style=\"color: black;\" transform=\"translate(10, %d) rotate(-90)\")>RSSI [dBm]</text>\n", end_y); // caption for Y-Line (power bins)

        web_client.printf("<rect width=\"%d\" height=\"%d\" x=\"%d\" y=\"%d\" fill=\"rgba(0, 110, 129, 0.2)\" />", own_freq_marker_width, end_y - start_y, own_freq_marker_start, start_y); // mark the frequency we are on
        web_client.printf("<line x1=\"%d\" y1=\"%d\" x2=\"%d\" y2=\"%d\" style=\"stroke:rgba(114, 0, 129, 0.2); stroke-width:1\"/>\n", own_freq_marker_center, start_y, own_freq_marker_center, end_y);

        // draw y axis titles and lines
        for (uint8_t i = 0; i < RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE; i++)
        {
            if (i % 3 == 0)
            {                                                                                                                                                                                                            // print frequency value every 3 steps
                web_client.printf("<line x1=\"%d\" y1=\"%d\" x2=\"%d\" y2=\"%d\" style=\"stroke:grey; stroke-width:1\"/>\n", start_x - 10, start_y + (i * step_pixel_height), end_x, start_y + (i * step_pixel_height)); // axis lines for Y axis
                web_client.printf("<text x=\"%d\" y=\"%d\" style=\"color: black;\">-%d</text>\n", start_x - 40, start_y + (i * step_pixel_height) + 6, 11 + (i * 4));                                   // axis title for Y-axis (power)
            }
        }

        #if not defined(BOARD_T_DECK_PRO)
        while (spec_curr_freq < spec_end)
        {                                                                                                 // loop through the frequency range
            uint16_t *res = sx126x_spectral_scan_freq(spec_curr_freq, spec_samples); // get spectrum analysis for this given frequency
            for (uint8_t i = 0; i < RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE; i++)
            { // loop through the power bins (usually 33)
                if (res[i] > 0)
                {                                                               // did the scan found anything above zero?
                    float alpha = (res[i] / spec_samples); // calculate the alpha-value (transperancy) ranging 0.0 - 1.0
                    if (alpha < 0.2)
                        alpha = 0.2; // use a minimum of alpha. Smaller values might be hard to see
                    web_client.printf("<rect width=\"%d\" height=\"%d\" x=\"%d\" y=\"%d\" fill=\"rgba(0,0,0,%0.3f)\"/>", step_pixel_width, step_pixel_height, start_x + (current_fStep * step_pixel_width), start_y + (i * step_pixel_height), alpha);
                }
            }

            // draw scale values
            if (current_fStep % 5 == 0)                                                                                                                                                                                                      // draw axis line every 5 steps
                web_client.printf("<line x1=\"%d\" y1=\"%d\" x2=\"%d\" y2=\"%d\" style=\"stroke:grey; stroke-width:1\"/>\n", start_x + (current_fStep * step_pixel_width), end_y, start_x + (current_fStep * step_pixel_width), end_y + 10); // axis lines for X-axis
            if (current_fStep % 10 == 0)                                                                                                                                                                                                     // draw axis title every 10 steps
                web_client.printf("<text x=\"%d\" y=\"%d\" style=\"color: black;\">%.3f</text>\n", start_x + (current_fStep * step_pixel_width) - 10, end_y + 25, spec_curr_freq);                                          // axis title for X-axis (frequency)

            delay(50); // lets wait for a moment (the example code used 100ms but 50ms seems to work, too)
            yield();   // this loop runs for a long time, pet the watchdog and give other tasks a chance to operate
            current_fStep++;
            spec_curr_freq += spec_step;
        }
        #endif

        web_client.println("</svg>");

        #if not defined(BOARD_T_DECK_PRO)
        sx126x_spectral_finish_scan(); // finish scan, return to normale lora operation
        #endif
    }

    web_client.printf("<p>The Node scans the frequency range (%.3fMHz - %.3fMHz) in small steps using a given scan bandwith (%.3fMHz).</p>\n", spec_start, spec_end, spec_step);
    web_client.printf("<p>It performs %i measurements each step and puts them into %i power-bins. That is why you will see several measurement-dots for each step. The darker the dot, the more measurements have been put into that certain power bin.</p>", spec_samples, RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE);
#else
    web_client.println("<p>Spectrum scan is not supported on this device.</p>");
#endif
    web_client.println("</div>");
}

/**
 * ###########################################################################################################################
 * delivers the info-page to be injected into the scaffold
 */
void sub_page_info()
{
    _create_meshcom_subheader("Node Information");
    web_client.println("<div id=\"content_inner\">");


    web_client.println("<table class=\"table mw-600\">");
    web_client.println("<thead><tr class=\"font-bold\"><td>Item</td><td>Value</td></tr></thead>");

    web_client.printf("<tr><td>Firmware</td><td>Meshcom %-4.4s%s<br>(build: %s / %s)<br>(flash-version %i)</td></tr>\n", SOURCE_VERSION, SOURCE_VERSION_WEB_SUB, __DATE__, __TIME__, meshcom_settings.node_fversion);
    web_client.printf("<tr><td>Start Date</td><td>%s</td></tr>\n", meshcom_settings.node_update);
    web_client.printf("<tr><td>Call</td><td>%s</td></tr>\n", meshcom_settings.node_call);
    web_client.printf("<tr><td>Hardware</td><td>%s</td></tr>\n", nbrHardwareName(BOARD_HARDWARE));
    web_client.printf("<tr><td>UTC offset</td><td>%.1f [%s]</td></tr>\n", meshcom_settings.node_utcoff, cTimeSource);
    // TZ-01: escaped, see sub_page_setup(); an unparsable value (config import) is flagged
    web_client.printf("<tr><td>Timezone</td><td>%s%s</td></tr>\n",
        meshcom_settings.node_tz[0] ? htmlEscape(String(meshcom_settings.node_tz)).c_str() : "none",
        (meshcom_settings.node_tz[0] && !tzActiveAbbrev()) ? " (invalid, ignored)" : "");
    // BAT-01: global_batt==0.0 is the established "no reading" convention (grounded pin, or
    // the ADC-path no-battery detection in batt_functions.cpp) -- same check the on-device
    // displays already use, see loop_functions.cpp.
    {   // BAT-04: charge state appended inside the same cell; BCS_UNKNOWN leaves the HTML unchanged
        const BattChargeInfo bci = global_batt_charge;
        char bcsText[40];
        bcsText[0] = '\0';
        if (bci.state != BCS_UNKNOWN) battChargeFormat(bcsText, sizeof(bcsText), bci);
        if (global_batt == 0.0) {
            web_client.print("<tr><td>Battery</td><td>USB (no battery)");
        } else {
            web_client.printf("<tr><td>Battery</td><td>%.3fV (%d%%) max %.3fV", global_batt / 1000.0, global_proz, meshcom_settings.node_maxv);
        }
        if (bcsText[0]) web_client.printf(", %s", bcsText);
        web_client.print("</td></tr>\n");
    }
#if defined(ESP32)
    { // AU web GUI: the fixed lines come from here, the message and the action button are filled by the page JS from GET /austatus
        const char *auChan = meshcom_settings.node_updchan ? "dev" : "prod";
        web_client.printf("<tr id=\"au_tr\"><td>Auto update</td><td id=\"au_row\" data-blepause=\"%d\">mode %s, channel %s<br>env %s, running %s",
            AU_BLE_PAUSE, meshcom_settings.node_autoupd == 2 ? "auto (notify only)" : "notify", auChan,
            MC_ENV_NAME[0] ? MC_ENV_NAME : "(unknown)", MC_BUILD_TAG[0] ? MC_BUILD_TAG : "local");
        web_client.print("<div id=\"au_msg\"></div><div id=\"au_sched\"></div><div id=\"au_err\"></div><div id=\"au_hint\"></div><button type=\"button\" id=\"au_btn\" disabled>Check now</button></td></tr>\n");
    }
#endif
    // WEB-SW: grouped switch overview, one <tr> per group, labels matching
    // sub_page_setup()'s _create_setup_switch_element() calls verbatim, same
    // #if guards as there. test/golden/info_switch_lint.py checks every
    // setup-page switch label appears somewhere on this page.
    web_client.printf("<tr><td>Settings</td><td>");
    web_client.printf("Gateway: %s<br>", (bGATEWAY ? "on" : "off"));
    web_client.printf("Mesh: %s<br>", (bMESH ? "on" : "off"));
    { // RM-07 (#1189): label equals the setup switch label (info_switch_lint); counters as in --info, no passwd, no tag
        const RmStats &rm = g_rmStats;
        unsigned long rmRej = (unsigned long)rm.rej_format + rm.rej_tag + rm.rej_replay + rm.rej_blocked +
                              rm.rej_rate + rm.rej_lockout + rm.rej_disabled;
        web_client.printf("Remote management (RM1): %s (ok=%lu rej=%lu)<br>", (meshcom_settings.node_rm == 1 ? "on" : "off"), (unsigned long)rm.ok, rmRej);
        web_client.printf("RM strict security: %s<br>", (meshcom_settings.node_rmstrict == 1 ? "on" : "off")); // BF-01
    }
    web_client.printf("Via: %s<br>", (bVIA ? "on" : "off"));
    web_client.printf("Userbutton: %s<br>", (bButtonCheck ? "on" : "off"));
    { // LED-08: outside the battery branch (USB-powered nodes too); labels equal the setup switch labels (info_switch_lint)
        const LedFitted ledHw = ledFittedNow();
        web_client.printf("Board LED: %s<br>", !ledHw.board ? "not fitted" : !bUSER_BOARD_LED ? "off" : boardLedPinUsable() ? boardLedModeName(boardLedModeGet()) : (boardLedModeGet() == BOARD_LED_MODE_STEADY ? "steady (pin busy)" : "flash (pin busy)"));   // as --info, WEB-LED-01
        web_client.printf("Charge LED: %s<br>", ledHw.charge ? webChargeLedText() : "not fitted");
    }
    #if defined(ANALOG_PIN)
    if (!bAnalogCheck)
        web_client.printf("Analog: off<br>");
    else if (meshcom_settings.node_analog_pin <= 0 || meshcom_settings.node_analog_pin >= 99)
        web_client.printf("Analog: on (GPIO not set, measurement paused)<br>");
    else
        web_client.printf("Analog: on (GPIO %i)<br>", meshcom_settings.node_analog_pin);
    #endif
    web_client.printf("</td></tr>\n");

    web_client.printf("<tr><td>Display</td><td>");
    web_client.printf("Display: %s<br>", (!bDisplayOff ? "on" : "off"));
    web_client.printf("Voltage: %s<br>", (bDisplayVolt ? "on" : "off"));
    web_client.printf("Message wake: %s<br>", (bMsgWake ? "on" : "off"));   // T114-DARK-01 (#1314)
    web_client.printf("</td></tr>\n");

    web_client.printf("<tr><td>Network</td><td>");
    { // GW-LINK: MeshCom server path (Internet/Hamnet) and whether it still answers
        char gwl[96];
        gwLinkFormat(gwl, sizeof(gwl), bGATEWAY, meshcom_settings.node_hasIPaddress, millis());
        web_client.printf("Gateway: %s<br>", htmlEscape(String(gwl)).c_str());
    }
    #if defined(HAS_ETHERNET)
    web_client.printf("Ethernet Mode: %s<br>", (meshcom_settings.node_netmode == 1 ? "on" : "off"));
    #endif
    web_client.printf("ext UDP: %s<br>", (bEXTUDP ? "on" : "off"));
    #if !defined(BOARD_RAK4630) && !defined(DISABLE_NET_CONSOLE)
    web_client.printf("net console: %s<br>", (bNETCONSOLE ? "on" : "off"));
    #endif
    #if defined(ESP32) && !defined(DISABLE_KISS_TCP)
    web_client.printf("KISS/TCP: %s<br>", (bKISS ? "on" : "off"));
    // security: flag transmit-enabled KISS clients with no HMAC auth gate
    web_client.printf("KISS TX: %s%s<br>", (bKISSTX ? "on" : "off"), (bKISSTX && bKISS && !bKISSAUTH) ? " (!)" : "");
    web_client.printf("KISS RxMeta: %s<br>", (bKISSMETA ? "on" : "off"));
    web_client.printf("KISS Auth: %s<br>", (bKISSAUTH ? "on" : "off"));
    #endif
    web_client.printf("</td></tr>\n");

    web_client.printf("<tr><td>Position</td><td>");
    #if defined (ENABLE_GPS) or defined(BOARD_RAK4630) or defined(BOARD_HELTEC_T114) or defined(BOARD_T_ECHO)
    web_client.printf("GPS: %s<br>", (bGPSON ? "on" : "off"));
    #endif
    web_client.printf("Track: %s<br>", (bDisplayTrack ? "on" : "off"));
    web_client.printf("</td></tr>\n");

    web_client.printf("<tr><td>Sensors</td><td>");
    #ifdef OneWire_GPIO
    web_client.printf("1-Wire: %s<br>", (bONEWIRE ? "on" : "off"));
    #endif
    #if defined(ENABLE_BMX280)
    web_client.printf("BMP280: %s<br>", (bBMPON ? "on" : "off"));
    web_client.printf("BME280: %s<br>", (bBMEON ? "on" : "off"));
    web_client.printf("BME680: %s<br>", (bBME680ON ? "on" : "off"));
    web_client.printf("MCU811: %s<br>", (bMCU811ON ? "on" : "off"));
    #endif
    #if defined (ENABLE_INA226)
    web_client.printf("INA226: %s<br>", (bINA226ON ? "on" : "off"));
    #endif
    #if defined(ENABLE_AHT20)
    web_client.printf("AHT20: %s<br>", (bAHT20ON ? "on" : "off"));
    #endif
    #if defined(ENABLE_LTR390)
    {
        float fuv = 0.0f;
        if(bUVON)
        {
            if(wxMeanGet(&wx_uv_mean, &fuv))
                web_client.printf("UV (LTR390): on (%s), UVI %.1f, mean %.1f (n=%lu)<br>", (ltr390_found ? "found" : "error"), wx_uv_last, fuv, (unsigned long)wxMeanCount(&wx_uv_mean));
            else
                web_client.printf("UV (LTR390): on (%s), UVI %.1f, mean - (n=0)<br>", (ltr390_found ? "found" : "error"), wx_uv_last);
        }
        else
            web_client.printf("UV (LTR390): off<br>");
    }
    #endif
    #if defined(ENABLE_WIND)
    {
        float fwm = 0.0f, fwg = 0.0f;
        if(bWINDON)
        {
            char swm[12] = "-", swg[12] = "-";
            if(wxMeanGet(&wx_wind_mean, &fwm)) snprintf(swm, sizeof(swm), "%.1f", fwm);
            if(wxGustGet(&wx_wind_gust, &fwg)) snprintf(swg, sizeof(swg), "%.1f", fwg);
            web_client.printf("Anemometer: on (pin %i), %.1f km/h, mean %s, gust %s<br>", meshcom_settings.node_wind_pin, wx_wind_last_kmh, swm, swg);
        }
        else
            web_client.printf("Anemometer: off<br>");
    }
    #endif
    #if defined(ENABLE_SHT21)
    web_client.printf("SHT21: %s<br>", (bSHT21ON ? "on" : "off"));
    #endif
    #if defined(ENABLE_SOFTSER)
    web_client.printf("SoftSer: %s<br>", (bSOFTSERON ? "on" : "off"));
    #endif
    web_client.printf("</td></tr>\n");

    web_client.printf("<tr><td>Messaging</td><td>");
    web_client.printf("No MSG All: %s<br>", (bNoMSGtoALL ? "on" : "off"));
    #if defined(ENABLE_MSGSTORE)
    web_client.printf("Notify sender: %s<br>", msgstoreNotice() ? "on" : "off"); // info_switch_lint: matches sub_page_setup()'s "storenotice" label
    #endif
    web_client.printf("</td></tr>\n");

    // Neighbourhood: not on the setup page (no _create_setup_switch_element there --
    // these are console-only --nbr* flags), so info_switch_lint.py does not require
    // them, but they are the security/behaviour-relevant state for mesh relay decisions.
    web_client.printf("<tr><td>Neighbourhood</td><td>");
    web_client.printf("NBR debug: %s<br>", (bNBRDEBUG ? "on" : "off"));
    web_client.printf("NBR relay: %s<br>", (bNBRCANCEL ? "on" : (bNBRRELAY ? "count" : "off")));
    web_client.printf("NBR sym: %s<br>", (bNBRSYM ? "on" : "off"));
    web_client.printf("</td></tr>\n");

    #if defined(ESP32) && (defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS) || defined(BOARD_T_DECK_PRO))
    web_client.printf("<tr><td>Persistence</td><td>");
    web_client.printf("SD: %s<br>", (meshcom_settings.node_persist_to_sd ? "on" : "off"));
    web_client.printf("Flash: %s<br>", (meshcom_settings.node_persist_to_flash ? "on" : "off"));
    web_client.printf("</td></tr>\n");
    #endif

    web_client.printf("<tr><td>Debug</td><td>");
    web_client.printf("Debug: %s<br>", (bDEBUG ? "on" : "off"));
    web_client.printf("Debug LoRa: %s<br>", (bLORADEBUG ? "on" : "off"));
    web_client.printf("Debug GPS: %s<br>", (iGPSDEBUG ? "on" : "off"));
    web_client.printf("Debug WX: %s<br>", (bWXDEBUG ? "on" : "off"));
    web_client.printf("Debug BLE: %s<br>", (bBLEDEBUG ? "on" : "off"));
    web_client.printf("</td></tr>\n");
    web_client.printf("<tr><td>APRS text</td><td>%s</td></tr>\n", htmlEscape(String(meshcom_settings.node_atxt)).c_str());
    web_client.printf("<tr><td>Mesh settings</td><td>");
    web_client.printf("max_hop_text: %i<br>", meshcom_settings.max_hop_text);
    web_client.printf("max_hop_pos: %i<br>", meshcom_settings.max_hop_pos);
    web_client.printf("</td></tr>\n");
    web_client.printf("<tr><td>Country</td><td>%s</td></tr>\n", getCountry(meshcom_settings.node_country).c_str());
    web_client.printf("<tr><td>Frequency</td><td>%.4f MHz</td></tr>\n", getFreq());
    web_client.printf("<tr><td>Bandwidth</td><td>%.0f kHz</td></tr>\n", getBW());
    web_client.printf("<tr><td>Spreading Factor (SF)</td><td>%i</td></tr>\n", getSF());
    web_client.printf("<tr><td>Coding Rate (CR)</td><td>%i</td></tr>\n", getCR());
    web_client.printf("<tr><td>TX Power</td><td>%i dBm (%.2f mW)</td></tr>\n", getPower(), 1000 * powf(10, ((float)getPower() - 30) / 10));

    #if defined(HAS_ETHERNET)
    web_client.printf("<tr><td>Netmode</td><td>%s</td></tr>\n", (meshcom_settings.node_netmode == 1 ? "Ethernet" : "WiFi"));
    #endif

    #ifndef BOARD_RAK4630
    if(meshcom_settings.node_netmode == 0)
    {
        if (bWIFIAP)
            web_client.printf("<tr><td>WiFi SSID</td><td>%s</td></tr>\n", htmlEscape(String(cBLEName)).c_str());
        else
        {
            web_client.printf("<tr><td>WiFi SSID</td><td>%s</td></tr>\n", htmlEscape(String(meshcom_settings.node_ssid)).c_str());
            // WEB-02: shows which AP the node associated with when several APs share the same SSID (mesh sets)
            web_client.printf("<tr><td>WiFi BSSID</td><td>%s</td></tr>\n", WiFi.BSSIDstr().c_str());
        }

        web_client.printf("<tr><td>WiFi AP</td><td>%s</td></tr>\n", (bWIFIAP ? "yes" : "no"));
        web_client.printf("<tr><td>WiFi RSSI</td><td>%i</td></tr>\n", WiFi.RSSI()); 
        web_client.printf("<tr><td>WiFi POWER SET</td><td>%i dBm\n</td></tr>\n", WiFi.getTxPower()/4);
    }
    #endif

    // wenn WIFI unterbrochen wird
    // if(meshcom_settings.node_hasIPaddress && strcmp(meshcom_settings.node_ip, "0.0.0.0") == 0)
    //    meshcom_settings.node_hasIPaddress = false;
    //    web_client.printf("<tr><td><b>hasIpAddress</b></td><td>%s</td></tr>\n", (meshcom_settings.node_hasIPaddress?"yes":"no"));
    
    web_client.printf("<tr><td>hasIpAddress</td><td>%s</td></tr>\n", (meshcom_settings.node_hasIPaddress ? "yes" : "no"));

    // NMTU-01 / #1190: configured MTU and the MSS derived from it (all boards)
    web_client.printf("<tr><td>MTU</td><td>MTU %i (MSS %i)</td></tr>\n", (int)meshcom_settings.node_ethmtu, (int)meshcom_settings.node_ethmtu - 40);

    if (meshcom_settings.node_hasIPaddress)
    {
        web_client.printf("<tr><td>IP address</td><td>%s</td></tr>\n", meshcom_settings.node_ip);
        web_client.printf("<tr><td>SUB-MASK</td><td>%s</td></tr>\n", meshcom_settings.node_subnet);

        if (!bWIFIAP)
        {
            web_client.printf("<tr><td>GW address</td><td>%s</td></tr>\n", meshcom_settings.node_gw);
            web_client.printf("<tr><td>DNS address</td><td>%s</td></tr>\n", meshcom_settings.node_dns);
            web_client.printf("<tr><td>NTP address</td><td>%s</td></tr>\n", getEffectiveNtpServer().c_str());
        }
    
    }

    if (bINA226ON)
    {
        web_client.println("<tr><td>INA226</td><td>");
        web_client.printf("vBUS<: %.2fV<br>", meshcom_settings.node_vbus);
        web_client.printf("vSHUNT: %.2fmV<br>", meshcom_settings.node_vshunt);
        web_client.printf("vCURRENT: %.1fmA<br>", meshcom_settings.node_vcurrent);
        web_client.printf("vPOWER: %.1fmW<br>", meshcom_settings.node_vpower);
        web_client.printf("</td></tr>\n");
    }

    if (bAnalogCheck)
    {
        web_client.println("<tr><td>Analog</td><td>");
        if (meshcom_settings.node_analog_pin <= 0 || meshcom_settings.node_analog_pin >= 99)
            web_client.printf("ANALOG GPIO: not set (measurement paused)<br>");
        else
            web_client.printf("ANALOG GPIO: %i<br>", meshcom_settings.node_analog_pin);
        web_client.printf("Factor: %.4fV<br>", meshcom_settings.node_analog_faktor);
        web_client.printf("Value: %.2fV<br>", fAnalogValue);
        web_client.printf("</td></tr>\n");
    }

    #if defined ENABLE_RTC
        if (bRTCON)
        {
            web_client.printf("<tr><td>RTC UTC Date/Time</td><td>%s</td></tr>\n", getStringRTCNow().c_str());
        }
    #endif

    { // BLK-01
        char blkcsv[BLOCKLIST_CSV_MAX];
        blockListGet(blkcsv, sizeof(blkcsv));
        String blkshow = blkcsv[0] ? String(blkcsv) : String("none");
        blkshow.replace(",", ", ");
        web_client.printf("<tr><td>Blocked calls</td><td>%s (dropped %lu)</td></tr>\n", htmlEscape(blkshow).c_str(), (unsigned long)blockListDrops());
    }

    // web_client.printf("<tr><td></td><td></td></tr>\n", );
    // web_client.printf("<tr><td></td><td></td></tr>\n", );
    // web_client.printf("<tr><td></td><td></td></tr>\n", );
    web_client.println("</table></div>");
    web_client.println(); // The HTTP response ends with another blank line
}

void sub_page_mcp23017()
{

    char onclick[100];
    char caption[40];
    char id[40];
    char value[40];

    uint16_t t_io = meshcom_settings.node_mcp17io;
    uint16_t t_out = meshcom_settings.node_mcp17out;
    uint16_t t_in = meshcom_settings.node_mcp17in;

    _create_meshcom_subheader("MCP23017 Status");
    web_client.println("<div id=\"content_inner\">");

    web_client.println("<table class=\"table mw-600\">");
    web_client.println("<colgroup>");
    web_client.println("<col style=\"width: 10%;\">");
    web_client.println("<col style=\"width: 16%;\">");
    web_client.println("<col style=\"width: 49%;\">");
    web_client.println("<col style=\"width: 15%;\">");
    web_client.println("<col style=\"width: 10%;\">");
    web_client.println("</colgroup>");
    web_client.println("<thead><tr class=\"font-bold\"><td>PORT</td><td>In/Out</td><td>Name</td><td>Status</td><td>Set</td></tr></thead>");


    if(bDEBUG)
    {
        Serial.printf("t_out = %i\n", t_out);
        Serial.printf("t_in = %i\n", t_in);
    }


    for (int io = 0; io < 16; io++)
    {
        bool bOut = false;
        if ((t_io & 0x0001) == 0x0001)
            bOut = true;

        bool bOutValue = false;
        if ((t_out & 0x0001) == 0x0001)
            bOutValue = true;

        bool bInValue = false;
        if ((t_in & 0x0001) == 0x0001)
            bInValue = true;

        char cAB = 'B';
        int iAB = io - 8;
        if (io < 8)
        {
            cAB = 'A';
            iAB = io;
        }

        if(bDEBUG)
            Serial.printf("Port %c%i has mask %i and t_in %i and t_out %i\n",cAB, iAB, t_io, t_in, t_out);

        web_client.printf("<tr><td>[%c%i]</td><td>", cAB, iAB);
        snprintf(onclick, 100, "setvalue('mcpio%c%i','%s',true)", cAB, iAB, bOut ?"in":"out");
        snprintf(caption, 4, "%s",  bOut ?"out":"in");
        uic_button(&web_client, onclick, caption);
        web_client.println("</td><td>");

        snprintf(id, 40, "mcpname%c%i", cAB, iAB);
        snprintf(value, 100, "%s", meshcom_settings.node_mcp17t[io]);
        uic_input(&web_client, id, (char*)"", value);

        snprintf(onclick, 100, "setvalue('mcpname%c%i', document.getElementById('mcpname%c%i').value,true);", cAB, iAB, cAB, iAB);
        snprintf(caption, 4,  "set");
        uic_button(&web_client, onclick, caption);

        web_client.println("</td>");

        if (bOut)
        {
                web_client.printf("<td>%s</td><td>", (bOutValue ? "HIGH" : "LOW"));
                snprintf(onclick, 100, "setvalue('mcpout%c%i','%s',true)", cAB, iAB, (bOutValue ? "off" : "on"));
                snprintf(caption, 5,  "%s", (bOutValue ? "LOW" : "HIGH"));
                uic_button(&web_client, onclick, caption);
                web_client.println("</td></tr>");
        }
        else
        {
            if (meshcom_settings.node_mcp17t[io][0] == 0x00)
                web_client.printf("<td>%s</td><td></td></tr>\n", (bInValue ? "HIGH" : "LOW"));
            else
                web_client.printf("<td><b>%s</b></td><td></td></tr>\n", (bInValue ? "HIGH" : "LOW"));
        }

        t_io >>= 1;
        t_out >>= 1;
        t_in >>= 1;
    }

    web_client.println("<tr><td colspan=\"5\">");
    snprintf(onclick, 100, "setvalue('mcpclear','',true)");
    snprintf(caption, 10,  "%s", "clear all");
    uic_button(&web_client, onclick, caption);
    web_client.println("</td></tr></table></div>");
}

/**
 * ###########################################################################################################################
 * Pins page: only the frame. The rows are made by pinsInit()/pnRender() in web/mc.js from GET /pinstatus (scripts of
 * sub-pages never run). PINS-HTML-BEGIN is read by tools/webgui_pins_test.js.
 * Switches: board LED through setparam boardled, MCP pins through mcpout/mcpio/mcpname, the same handlers as the
 * MCP23017 page (kept as it is: server-rendered, no polling). Everything is hidden until the first answer says what exists.
 */
void sub_page_pins()
{
    _create_meshcom_subheader("Pins");
    // PINS-HTML-BEGIN
    web_client.println("<div id=\"content_inner\">");
    web_client.println("<div id=\"pn_led\" hidden><label id=\"pn_led_lbl\">Board LED</label>");
    web_client.println("<div class=\"mcledgrp\" role=\"group\" aria-labelledby=\"pn_led_lbl\">");
    web_client.println("<button type=\"button\" class=\"mctab mcledbtn\" id=\"pn_led_off\" aria-pressed=\"false\" onclick=\"pinsLed('off')\">off</button>");
    web_client.println("<button type=\"button\" class=\"mctab mcledbtn\" id=\"pn_led_flash\" aria-pressed=\"false\" onclick=\"pinsLed('flash')\">flash</button>");
    web_client.println("<button type=\"button\" class=\"mctab mcledbtn\" id=\"pn_led_steady\" aria-pressed=\"false\" onclick=\"pinsLed('steady')\">steady</button>");
    web_client.println("</div><p class=\"font-small\" id=\"pn_led_st\"></p></div>");
    web_client.println("<div id=\"pn_chg\" hidden><label id=\"pn_chg_lbl\">Charge LED</label>");
    web_client.println("<div class=\"mcledgrp\" role=\"group\" aria-labelledby=\"pn_chg_lbl\">");
    web_client.println("<button type=\"button\" class=\"mctab mcledbtn\" id=\"pn_chg_off\" aria-pressed=\"false\" onclick=\"pinsChg('off')\">off</button>");
    web_client.println("<button type=\"button\" class=\"mctab mcledbtn\" id=\"pn_chg_auto\" aria-pressed=\"false\" onclick=\"pinsChg('auto')\">auto</button>");
    web_client.println("</div><p class=\"font-small\">auto = follows charging, off = dark. Now: <span id=\"pn_chg_st\"></span></p></div>");
    web_client.println("<table class=\"table mw-600\" id=\"pn_asg\" hidden><caption class=\"font-small\">Pin assignments</caption>");
    web_client.println("<thead><tr class=\"font-bold\"><th scope=\"col\">Function</th><th scope=\"col\">Pin</th><th scope=\"col\">Default</th><th scope=\"col\">In use</th><th scope=\"col\">Note</th></tr></thead>");
    web_client.println("<tbody id=\"pn_asg_body\"></tbody></table>");
    web_client.println("<p class=\"font-small\" id=\"pn_asg_hint\" hidden>A pin the board uses itself is refused.</p>");
    web_client.println("<div id=\"pn_gpioctl\" hidden><label id=\"pn_gpioctl_lbl\">GPIO control</label>");
    web_client.println("<div class=\"mcledgrp\" role=\"group\" aria-labelledby=\"pn_gpioctl_lbl\">");
    web_client.println("<button type=\"button\" class=\"mctab mcledbtn\" id=\"pn_gpioctl_off\" aria-pressed=\"false\" onclick=\"pinsGpioCtl('off')\">off</button>");
    web_client.println("<button type=\"button\" class=\"mctab mcledbtn\" id=\"pn_gpioctl_on\" aria-pressed=\"false\" onclick=\"pinsGpioCtl('on')\">on</button>");
    web_client.println("</div></div>");
    web_client.println("<table class=\"table mw-600\" id=\"pn_gpio\" hidden><caption class=\"font-small\">Free GPIO</caption>");
    web_client.println("<thead><tr class=\"font-bold\"><th scope=\"col\">Pin</th><th scope=\"col\">Owner</th><th scope=\"col\">Direction</th><th scope=\"col\">Level</th><th scope=\"col\">Switch</th></tr></thead>");
    web_client.println("<tbody id=\"pn_gpio_body\"></tbody></table>");
    web_client.println("<p class=\"font-small\" id=\"pn_gpio_off\" hidden>GPIO control is off. Turn it on above to switch the pins; every pin returns to input after a restart.</p>");
    web_client.println("<p id=\"pn_none\" hidden>No switchable outputs on this node.</p>");
    web_client.println("<table class=\"table mw-600\" id=\"pn_mcp\" hidden><caption class=\"font-small\">MCP23017 pins</caption>");
    web_client.println("<thead><tr class=\"font-bold\"><th scope=\"col\">Pin</th><th scope=\"col\">Name</th><th scope=\"col\">In/Out</th><th scope=\"col\">Level</th></tr></thead>");
    web_client.println("<tbody id=\"pn_body\"></tbody></table>");
    web_client.println("<p class=\"font-small pn-note\" id=\"pn_note\" role=\"status\"></p>");
    web_client.println("<p class=\"font-small pn-note\" id=\"pn_msg\" role=\"status\"></p>");
    web_client.println("<p class=\"font-small\">Refreshed every 5 s while this page is open. Output levels are what the node was told to set; input levels are sampled by the node every 5 s.</p>");
    web_client.println("</div>");
    // PINS-HTML-END
}

/**
 * ###########################################################################################################################
 * GET /pinassign -- the four user pin assignments with board default, pin in use and the pins the policy accepts
 * (src/pin_assign_json.h documents the shape). Read-only and GET only; behind the WEB-AUTH gate like the pages.
 * Writes go through /setparam/ (onewiregpio, buttongpio, angpio, windpin) -> console setters -> pinCanAssign().
 */
static void __attribute__((noinline)) sub_pin_assign()
{
    PinAssignIn in;
    memset(&in, 0, sizeof(in));
    in.chip = pinBoardChip();

    for (int i = 0; i < (int)PIN_FN_COUNT; i++)
    {
        const PinFunc f = (PinFunc)i;
        PinAssignFn &o = in.fn[i];
        o.def = pinBoardDefault(f);
        o.eff = pinBoardInUse(f);   // the button: the pin read at boot (a new setting needs a restart)
        o.ok = pinAssignMask(pinBoardContext(f), f, o.def < 0 ? 0 : o.def, pinBoardSetterRefused(f));   // policy: 0 = no default
    }
    for (int i = 0; i < (int)PIN_FN_COUNT; i++)
        in.fn[i].avail = pinAssignAvail((PinFunc)i);
    in.fn[PIN_FN_ONEWIRE].pin = meshcom_settings.node_owgpio;
    in.fn[PIN_FN_ONEWIRE].on = bONEWIRE;
    in.fn[PIN_FN_BUTTON].pin = meshcom_settings.node_button_pin;
    in.fn[PIN_FN_BUTTON].on = bButtonCheck;
    in.fn[PIN_FN_BUTTON].restart = true;     // GPIO-01 W0: the button pin is read at boot
    in.fn[PIN_FN_ANALOG].pin = meshcom_settings.node_analog_pin >= 99 ? 0 : meshcom_settings.node_analog_pin;   // 99 = unset
    in.fn[PIN_FN_ANALOG].on = bAnalogCheck;
#if defined(ESP32)
    in.fn[PIN_FN_WIND].pin = meshcom_settings.node_wind_pin;
    in.fn[PIN_FN_WIND].on = bWINDON;
#endif

#if defined(NRF52_SERIES)
    static char body[PIN_ASSIGN_JSON_MAX];   // the 8 KB loop stack is tight; the web server runs in this one task only
#else
    char body[PIN_ASSIGN_JSON_MAX];
#endif
    const size_t n = pinAssignJson(in, body, sizeof(body));
    web_cors_ok = false; // node-local read, as /pinstatus
    send_http_header(n ? 200 : 500, RESPONSE_TYPE_JSON);
    if (n)
        web_client.println(body);
    else
        web_client.println("{\"ok\":false}");
}

/**
 * ###########################################################################################################################
 * GET /gpiostatus -- the free GPIOs with owner, direction and level (src/gpio_status_json.h documents the shape). Read-only,
 * GET only, node-local. Switching goes through /setparam/?gpio<N>=on|off|in|release (web_setup.cpp, needs X-MC); the master
 * switch has one web path, /setparam/?gpioctl=on|off (X-MC), besides the console --gpiocontrol. nRF52: always the empty document.
 */
static void __attribute__((noinline)) sub_gpio_status()
{
#if defined(ESP32)
    GpioStatusPin pins[GPIO_STATUS_MAX_PINS];
    GpioStatusIn in = {false, false, pins, 0};
    in.n = gpioStatusCollect(pins, GPIO_STATUS_MAX_PINS);
    in.avail = in.n > 0;
    in.master = gpioCtlMaster();
    char body[GPIO_STATUS_JSON_MAX];
    const size_t n = gpioStatusJson(in, body, sizeof(body));
    web_cors_ok = false; // node-local read, as /pinassign
    send_http_header(n ? 200 : 500, RESPONSE_TYPE_JSON);
    if (n)
        web_client.println(body);
    else
        web_client.println("{\"ok\":false}");
#else
    // nRF52: constant empty document, no buffers on the 8 KB loop stack
    web_cors_ok = false;
    send_http_header(200, RESPONSE_TYPE_JSON);
    web_client.println("{\"avail\":0,\"master\":0,\"pins\":[]}");
#endif
}

/**
 * ###########################################################################################################################
 * GET /pinstatus -- cached state of the switched pins as one JSON document (src/pin_status_json.h documents the shape).
 * Read-only and GET only; behind the WEB-AUTH gate like the pages. No I2C access here: the MCP levels are the settings
 * fields as loopMCP23017() (scheduler, every 5 s, and right after --setout / --setio) last left them.
 */
static void __attribute__((noinline)) sub_pin_status()
{
    PinStatusIn in;
    memset(&in, 0, sizeof(in));

    const LedFitted ledHw = ledFittedNow();
    in.ledFitted = ledHw.board;
#if defined(BOARD_LED) || (defined(LED_PIN) && defined(LED_PIXEL) && LED_PIXEL > 0)
    if (ledHw.board)
    {
        in.ledMode = bUSER_BOARD_LED ? (boardLedModeGet() == BOARD_LED_MODE_STEADY ? PIN_LED_STEADY : PIN_LED_FLASH) : PIN_LED_OFF;
        in.ledBusy = !boardLedPinUsable();
    }
#endif
#if defined(REMOTE_LED_PIN)
    in.ledRm = bRemoteLedFlash ? PIN_RM_FLASH : (bRemoteLed ? PIN_RM_ON : PIN_RM_NONE);
#endif

    in.chgMode = "off";
#if defined(XPOWERS_CHIP_AXP192) || defined(XPOWERS_CHIP_AXP2101)
    in.chgFitted = ledHw.charge;
    in.chgMode = bCHARGE_LED ? "auto" : "off";
    in.chgEff = chargeLedModeName(chargeLedEffective());
    in.chgRm = chargeLedOverrideActive();
#endif

    in.mcpPresent = bMCP23017;
    in.mcpIo = (uint16_t)meshcom_settings.node_mcp17io;
    in.mcpOut = (uint16_t)meshcom_settings.node_mcp17out;
    in.mcpIn = (uint16_t)meshcom_settings.node_mcp17in;
    in.names = meshcom_settings.node_mcp17t;   // not copied; pinJsonEscape reads at most PIN_NAME_MAX - 1 bytes

#if defined(NRF52_SERIES)
    static char body[PIN_STATUS_JSON_MAX];   // the 8 KB loop stack is tight; the web server runs in this one task only
#else
    char body[PIN_STATUS_JSON_MAX];
#endif
    const size_t n = pinStatusJson(in, body, sizeof(body));
    web_cors_ok = false; // node-local read, as /austatus
    send_http_header(n ? 200 : 500, RESPONSE_TYPE_JSON);
    if (n)
        web_client.println(body);
    else
        web_client.println("{\"ok\":false}");
}

/**
 * ###########################################################################################################################
 * Sends a valid HTTP header. Takes care of the status code.
 * We may need to set more status_text cases once we need them.
 */
void send_http_header(uint16_t http_status_code, uint8_t content_type, const char *extra_header)
{
    String status_text = "";
    switch (http_status_code)
    {
    case 200:
        status_text = "OK";
        break; // use this when ever a request was successful
    case 400:
        status_text = "Bad Request";
        break; // use this when a request body was rejected (CS-03 config upload)
    case 431:
        status_text = "Request Header Fields Too Large";
        break;
    case 401:
        status_text = "Unauthorized";
        break; // use this when ever a request was successful
    case 403:
        status_text = "Forbidden";
        break; // RM GUI W1c: web guard (rebinding Host / cross-site request)
    case 404:
        status_text = "Not Found";
        break; // use this if a request was not known
    case 405:
        status_text = "Method Not Allowed";
        break; // W2-1: GET /rmsend (the password must travel in a POST body)
    case 422:
        status_text = "Unprocessable Entity";
        break; // use this if a parameter was not processable (e.g. out of bounds or somehow wrong)
    default:
        status_text = "Unknown"; // fallback
    }

    // Serial.println("STATUS_TEXT="+status_text);
    web_client.printf("HTTP/1.1 %i %s \n", http_status_code, status_text.c_str());
    if (content_type == RESPONSE_TYPE_JSON)
        web_client.println("Content-type:application/json");
    else
        web_client.println("Content-type:text/html; charset=utf-8");
    // No Access-Control-Allow-* headers: the GUI is same-origin and no cross-origin consumer is documented
    if (extra_header && extra_header[0])
        web_client.println(extra_header); // WEB-AUTH-01: Set-Cookie of the login / logout answers
    web_client.println("Transfer-Encoding: chunked");                         // WEB-TX-01: a cut page is visibly incomplete
    web_client.println("Connection: close");                                  // tell broser that the connection will be closed (in opposite to keep-alive)
    web_client.println("Cache-Control: no-cache, no-store, must-revalidate"); // set caching policy
    web_client.println("Pragma: no-cache");                                   // Disable caching or request/respinse
    web_client.println("Expires: 0");                                         // set cache expiring time to 0
    web_client.println();                                                     // two CR-LF marks the end of the header
    webTxBeginChunked(web_client);                                            // WEB-TX-01: every flush from here on is one chunk
}

/**
 * Creates the Sub-Header containing the Title and the Date/Time when that page was created
 */
void _create_meshcom_subheader(String title)
{
    // WEB-TD-01 item 6: mc.js advances the time every second (mcClockTick). data-epoch is the shown local date/time read as UTC seconds
    // (so JS needs no tz maths); without a valid node time no id/attribute is written and the text stays as rendered.
    uint32_t clkEpoch = 0;
    const bool clkOk = cetEpochFromFields(meshcom_settings.node_date_year, meshcom_settings.node_date_month, meshcom_settings.node_date_day,
                                          meshcom_settings.node_date_hour, meshcom_settings.node_date_minute, meshcom_settings.node_date_second, &clkEpoch);
    char clkAttr[48] = "";
    if (clkOk)
        snprintf(clkAttr, sizeof(clkAttr), " id=\"mc_clock\" data-epoch=\"%lu\"", (unsigned long)clkEpoch);
    web_client.printf("<div id=\"content_title\"><p class=\"font-small\"><span%s>%i-%02i-%02i&nbsp;%02i:%02i:%02i</span>&nbsp;%s&nbsp;[%s]</p><h1 class=\"font-bold\">%s</h1></div>", clkAttr, meshcom_settings.node_date_year, meshcom_settings.node_date_month, meshcom_settings.node_date_day, meshcom_settings.node_date_hour, meshcom_settings.node_date_minute, meshcom_settings.node_date_second, getTimeZone().c_str(), cTimeSource, title.c_str());
}

/**
 * ###########################################################################################################################
 * Creates a common setup text-input elementsrc/web_functions/web_functions.cpp src/web_functions/web_setup.cpp including label and apply-button
 * Parameter inputValue is chosen to be String so conversions from integers, floats, etc is easier.
 *
 * @param id the DOM id of that element. Must be unique.
 * @param labelText the text in the label
 * @param inputValue the current value of that field
 * @param placeHolder the placeholder that is displayed if the textinput is empty
 * @param parameterName the name of the parameter wich is used to identify the parameter
 * @param maxlength the maximum allowed input length
 * @param isPassword if set to TRUE, the input will be a password-type input
 */
void _create_setup_textinput_element(const char id[], const char labelText[], String inputValue, const char placeHolder[], const char  parameterName[], uint8_t maxlength, bool isPassword, bool needConfirm){
    char onclick[100]; 
    char caption[100];
    snprintf(onclick, 100, "setvalue('%s', document.getElementById('%s').value,false)", parameterName, id);
    snprintf(caption, 100, "<i class=\"btncheckmark\"></i>");

    web_client.printf("<label for=\"%s\">%s :</label>\n", id, labelText);
    web_client.printf("<input type=\"%s\" name=\"%s\" id=\"%s\" value=\"%s\" maxlength=\"%i\" size=\"10\" placeholder=\"%s\">\n", isPassword ? "password" : "text", id, id, htmlEscape(inputValue).c_str(), maxlength, placeHolder); // RM W1b: escape at output (value= attribute)

    if(needConfirm) {
        char confirm[200];
        snprintf(confirm, sizeof(confirm), "Are you sure you want to set &quot;%s&quot; to &quot;'+document.getElementById('%s').value+'&quot;?", labelText, id);
        uic_button(&web_client, onclick, caption, confirm);
    } else {
         uic_button(&web_client, onclick, caption);
    }
}


/**
 * Number input including label, range hint and button (D10). Same setvalue() button as the text variant;
 * the JS reads the element by id, so type=number works unchanged. Placeholder is the board default power.
 */
void _create_setup_numberinput_element(const char id[], const char labelText[], int inputValue, int minValue, int maxValue, const char parameterName[]){
    char onclick[100];
    char caption[100];
    snprintf(onclick, 100, "setvalue('%s', document.getElementById('%s').value,false)", parameterName, id);
    snprintf(caption, 100, "<i class=\"btncheckmark\"></i>");

    web_client.printf("<label for=\"%s\">%s :</label>\n", id, labelText);
    web_client.printf("<input type=\"number\" name=\"%s\" id=\"%s\" value=\"%i\" min=\"%i\" max=\"%i\" step=\"1\" size=\"10\" placeholder=\"%i\">\n", id, id, inputValue, minValue, maxValue, (int)TX_OUTPUT_POWER);
    uic_button(&web_client, onclick, caption);
    web_client.printf("<div class=\"font-small\" style=\"grid-column:1/-1\">Allowed on this board: %i to %i dBm</div>\n", minValue, maxValue);
}


/**
 * ###########################################################################################################################
 * Creates a common setup switch-input element including label and description
 *
 * @param id the DOM id of that element. Must be unique.
 * @param labelText the text in the label
 * @param descriptionText the smaller text in brackets
 * @param checked TRUE, if the switch should be displayed as activated
 * @param warnText TRK-01: optionaler Warnhinweis-Text neben dem Switch; nullptr = kein Hinweis
 * @param warnVisible TRK-01: TRUE, wenn der Warnhinweis beim Seitenaufbau sichtbar sein soll
 */
void _create_setup_switch_element(const char id[], const char labelText[], const char descriptionText[], bool checked, const char warnText[], bool warnVisible)
{
    web_client.printf("<label for=\"%s\">%s <span class=\"font-small\">(%s)</span>", id, labelText, descriptionText);
    if (warnText != nullptr)
    { // TRK-01: zweiter Span mit der id "<id>_warn", damit setvalue() ihn live umschalten kann
        web_client.printf("<span id=\"%s_warn\" class=\"font-small\" style=\"color:var(--mcred)%s\"> %s</span>", id, warnVisible ? "" : ";display:none", warnText);
    }
    web_client.println("</label>");
    web_client.printf("<input type=\"checkbox\" role=\"switch\" id=\"%s\" %s onchange=\"setvalue(this.id,this.checked?'on':'off',false)\"/>\n", id, checked ? "checked" : "");
}

// WEB-LED-01: three-way LED mode (off / flash / steady) as aria-pressed buttons; posts setparam <id>=off|flash|steady
__attribute__((unused)) static void _create_setup_led_mode_element(const char id[], const char labelText[], const char descriptionText[], int mode)
{
    web_client.printf("<label id=\"%s_lbl\" for=\"%s_off\">%s <span class=\"font-small\">(%s)</span></label>\n", id, id, labelText, descriptionText);
    web_client.printf("<div class=\"mcledgrp\" role=\"group\" aria-labelledby=\"%s_lbl\">\n", id);
    const char *names[3] = {"off", "flash", "steady"};
    for (int i = 0; i < 3; i++)
    {
        const bool on = (mode == i);
        web_client.printf("<button type=\"button\" class=\"mctab mcledbtn%s\" id=\"%s_%s\" aria-pressed=\"%s\" onclick=\"setvalue('%s','%s',true)\">%s</button>\n",
                          on ? " mctab-on" : "", id, names[i], on ? "true" : "false", id, names[i], names[i]);
    }
    web_client.println("</div>");
}

/**
 * ###########################################################################################################################
 * handles incoming sendmessage request
 * ToDo: this might be better located anywhere else - maybe nodefunctioncalls?
 * ToDo: switch response to JSON
 */
void send_message(String web_header)
{
    if ((web_header.indexOf("&tocall=") >= 0) && (web_header.indexOf("&message=") >= 0))
    { // check if all neccessary parameters are there
        if (web_header.lastIndexOf(" HTTP/1.1") >= 0)
        {
            web_header = web_header.substring(0, web_header.lastIndexOf(" HTTP/1.1")); // remove last occurance of " HTTP/1.1" wich is path of the html header
        }
        String tocall = web_header.substring(web_header.indexOf("&tocall=") + 8, web_header.indexOf("&message"));
        String message = web_header.substring(web_header.indexOf("&message=") + 9);

        // replace Percent-Coded Chars with its original Char (e.g.: replace %20 with SPACE)
        tocall = decodeURLPercentCoding(tocall);
        message = decodeURLPercentCoding(message);

        tocall.trim();
        message.trim();

        tocall.toUpperCase();

        // BP-09: default outcome string, overridden below by the actual
        // sendMessage() result. The page's own sendMessage() JS (further
        // below, search for "onreadystatechange") tests this response for
        // "sendmessage ok" and only clears the input fields on a match, so
        // a refused or dropped message leaves the operator's text on screen.
        // Keep the four strings and that test in sync.
        const char *bp_result_text = "sendmessage ok";

        if (message.length() > 0)
        {
            if (tocall.length() > 0)
                message = ":{" + tocall + "}" + message;
            // snprintf(message_text, sizeof(message_text), ":{%s}%s", message_call.c_str(), message.c_str());
            else
                message = ":" + message;
            // snprintf(message_text, sizeof(message_text), ":%s", message.c_str());
            // force whole Message to have a maximum length of 150 chars including a destination callsign
            // #1173: cut on a UTF-8 character boundary, never inside a multi-byte sequence
            if (message.length() > 150)
                message = message.substring(0, charset_utf8_safe_truncate(message.c_str(), message.length(), 150));

            // We expect a char array instead of a String, so we need to convert the string to char array
            // ToDo: We might think about changing everything to String instead of Char Array later.
            // That might be easier to handle, we won't waste memory and we cannot forget the NULL-Char at the end.
            char message_text[200];
            strncpy(message_text, message.c_str(), sizeof(message_text)); // c_str() automagically adds a NULL-character to the end

            int iml = strlen(message_text);
            // new massage has to be different from previous message
            if (iml > 0)
            {
                hasMsgFromPhone = true;
                setMsgOrigin(ORIGIN_WEB);   // BP-01: notice goes back to the web GUI
                int bp_rc = sendMessage(message_text, iml);
                setMsgOrigin(ORIGIN_NONE);
                hasMsgFromPhone = false;
                // Serial.print("Message send: ");
                // Serial.println(message_text);

                switch (bp_rc)
                {
                    case BP_SEND_REFUSED: bp_result_text = "sendmessage refused"; break;
                    case BP_SEND_DROPPED: bp_result_text = "sendmessage dropped"; break;
                    case BP_SEND_INVALID: bp_result_text = "sendmessage invalid"; break;
                    default:              bp_result_text = "sendmessage ok";      break;
                }
            }
        }

        web_client.println(bp_result_text);
    }
    else
    {
        web_client.println("sendmessage failed");
    }
    web_client.println(); // The HTTP response ends with another blank line
}

#if defined(ESP32)
/**
 * ###########################################################################################################################
 * GET /austatus -- state of the firmware update for the Info page row (spec 5.1). Read-only.
 */
static void sub_au_status()
{
    FwNetStatus au;
    fwNetGetStatus(au);
    AuUiIn in;
    memset(&in, 0, sizeof(in));
    const char *stn = "idle";
    if (au.state == FWS_BUSY)
        stn = "busy";
    else if (au.state == FWS_DONE_OK)
        stn = "ok";
    else if (au.state == FWS_DONE_FAIL)
        stn = "fail";
    const char *jobn = "none";
    if (au.job == FWJ_CHECK)
        jobn = "check";
    else if (au.job == FWJ_DOWNLOAD)
        jobn = "download";
    else if (au.job == FWJ_STAGELAN)
        jobn = "stagelan";
    snprintf(in.st, sizeof(in.st), "%s", stn);
    snprintf(in.job, sizeof(in.job), "%s", jobn);
    snprintf(in.run, sizeof(in.run), "%.23s", MC_BUILD_TAG[0] ? MC_BUILD_TAG : "local");
    snprintf(in.avail, sizeof(in.avail), "%.23s", au.availTag);
    snprintf(in.staged, sizeof(in.staged), "%.23s", au.staged ? au.stagedTag : "");
    snprintf(in.err, sizeof(in.err), "%.23s", au.state == FWS_BUSY ? "" : au.lastErr);
    snprintf(in.mode, sizeof(in.mode), "%s", meshcom_settings.node_autoupd == 2 ? "auto" : "notify"); // never "off" (D1)
    snprintf(in.chan, sizeof(in.chan), "%s", meshcom_settings.node_updchan ? "dev" : "prod");
    in.newer = (au.availNewer && au.availTag[0]) ? 1 : 0;
    in.inst = au.installable ? 1 : 0;
    // pct: compressed bytes written / the length the running job stages (progLen), only while a download runs
    const bool dl = au.state == FWS_BUSY && (au.job == FWJ_DOWNLOAD || au.job == FWJ_STAGELAN);
    if (dl)
        in.pct = fwProgressPct(au.bytesDone, au.progLen);
    in.phone = fwNetPhoneHeld() ? 1 : 0; // same expression as the lowheap_phone token
    in.guard = auBleGuardActive() ? 1 : 0; // BLE guard (campaign-au-webgui-ux section 7)
    in.sbok = fwNetSafebootCapable() ? 1 : 0;
    snprintf(in.sbwhy, sizeof(in.sbwhy), "%s", fwNetSafebootWhy()); // ok|old|dio, tells the page why the install is blocked
    in.etaS = auEtaS(au.stagedIlen);
    fwNetHeapNow(&in.ifree, &in.iblk);

    char body[832];
    size_t n = auStatusJson(in, body, sizeof(body));
    { // install schedule: the countdown notice rides outside auStatusJson (its golden strings stay as they are)
        FwSchedView sv;
        fwNetSchedGet(sv);
        n = auAppendSched(body, sizeof(body), n, sv.text);
    }
    // AU-30 "vpend": the staged record's dry run was deferred to the install (the page says "checked at install").
    // One NVS read, only while a record is staged.
    n = auAppendVpend(body, sizeof(body), n, (au.staged && fwNetVerifyPending(au.stagedTag)) ? 1 : 0);

    // "up": seconds since boot, appended outside auStatusJson. The page compares it across the install restart
    // when the build tag cannot be compared (an image built without MC_BUILD_TAG reports run "local").
    n = fwJsonAppendUp(body, sizeof(body), n, (uint32_t)(esp_timer_get_time() / 1000000));
    // "img": first 8 hex of the running image's ELF SHA-256. With run "local" the page counts the install as done
    // only when it changed: a Safeboot fallback restarts the node too (uptime drops) but keeps the old image.
    const esp_app_desc_t *d = MC_RUNNING_APP_DESC();
    if (d != NULL)
        n = fwJsonAppendImg(body, sizeof(body), n, d->app_elf_sha256, sizeof(d->app_elf_sha256));
    send_http_header(n ? 200 : 500, RESPONSE_TYPE_JSON);
    if (n)
        web_client.println(body);
    else
        web_client.println("{\"ok\":false}");
}
#endif

/**
 * ###########################################################################################################################
 * Tries to execute a Web-API function call.
 * This will return a JSON containing the result of the function execution ("ok" or "failed")
 */
void call_function(String web_header)
{
    const String full_header = web_header; // the X-MC check below needs the header lines
    web_header = web_header.substring(web_header.indexOf("/callfunction/?") + 15, web_header.indexOf("HTTP/1.1"));
    web_header.trim();
    funCallStruct functionData;

    if (web_header.indexOf("=") > 0)
    {
        functionData.functionName = decodeURLPercentCoding(web_header.substring(0, web_header.indexOf("=")));
        functionData.functionParameter = decodeURLPercentCoding(web_header.substring(web_header.indexOf("=") + 1));
    }
    else
    {
        functionData.functionName = decodeURLPercentCoding(web_header);
        functionData.functionParameter = "";
    }

    // AU web GUI: the update actions need the page's own `X-MC: 1` request header (people yes, foreign web pages no).
    // Checked on the full header, before anything runs; independent of MC_WEB_GUARD.
    if (!webGuardCallAllowed(functionData.functionName.c_str(), full_header.c_str(), full_header.length()))
    {
        if (bDEBUG)
            Serial.println("[WEB];guard;upd_no_xmc");
        web_cors_ok = false;
        send_http_header(403, RESPONSE_TYPE_JSON);
        web_client.println("{\"ok\":false,\"err\":\"forbidden\"}");
        return;
    }

#if defined(ENABLE_MSGSTORE)
    // Stage 3 store node operator actions (docs/dm-stage3-wave-plan-20260914.md). Handled here
    // rather than in webFunctionCall() (web_nodefunctioncalls.cpp, out of this file's exclusive
    // set) -- both keep the same WF_RETURNCODE_OKAY/FAIL convention call_function() already
    // serialises below, so the mailbox page's callfunction() calls need no special handling.
    if (functionData.functionName.equals("mboxpurge"))
    {
        if (functionData.functionParameter.equals("all"))
        {
            msgstorePurgeAll();
            functionData.returnCode = WF_RETURNCODE_OKAY;
        }
        else
        {
            functionData.returnCode = msgstorePurge(functionData.functionParameter.toInt()) ? WF_RETURNCODE_OKAY : WF_RETURNCODE_FAIL;
        }
    }
    else if (functionData.functionName.equals("mboxdeliver"))
    {
        functionData.returnCode = msgstoreDeliverNow(functionData.functionParameter.toInt()) ? WF_RETURNCODE_OKAY : WF_RETURNCODE_FAIL;
    }
    else
#endif
    {
        webFunctionCall(&functionData); // try to execute that command
    }

    send_http_header(functionData.returnCode == WF_RETURNCODE_OKAY ? 200 : 422, RESPONSE_TYPE_JSON); // send header, either 200 if command was executed or 422 if not

    // JSN-01: functionName is percent-decoded, attacker-controlled query input
    // and used to land here unescaped via a raw %s -- a '"' or '\' in it broke
    // the response. Build through a JsonDocument instead: ArduinoJson escapes
    // both the key and the value on serialise.
    JsonDocument doc;
    doc[functionData.functionName] = (functionData.returnCode == WF_RETURNCODE_OKAY) ? "ok" : "failed"; // {"functionName":"ok|failed"}
    serializeJson(doc, web_client);
    web_client.println();
}

/**
 * ###########################################################################################################################
 * Set the Value of a parameter (if known)
 * This will return a JSON containing the result like {"parameterName":"parameterValue"} using a HTTP return code (e.g. 200 or 422)
 */
void setparam(String web_header)
{
    const String full_header = web_header; // the X-MC check below needs the header lines

    web_header = web_header.substring(web_header.indexOf("/setparam/?") + 11, web_header.indexOf(" HTTP/1.1"));
    String param_name = decodeURLPercentCoding(web_header.substring(0, web_header.indexOf("=")));
    String param_value = decodeURLPercentCoding(web_header.substring(web_header.indexOf("=") + 1));
    param_name.toLowerCase();

    // AU web GUI: no setparam name may run an update command for a foreign page (every value lands in a
    // "--<cmd> %s" line that commandAction() splits on "--"), and aumode/autoupdate/updchan change the update
    // mode or channel: the upd* gate of call_function() would be a bypass otherwise. Same X-MC proof,
    // independent of MC_WEB_GUARD.
    // The install schedule settings (audelay, auday, autime) run --autoupdate sub-commands: webGuardIsUpdParam covers them.
    if (!webGuardSetparamAllowed(param_name.c_str(), param_value.c_str(), full_header.c_str(), full_header.length()))
    {
        if (bDEBUG)
            Serial.println("[WEB];guard;setparam_upd_no_xmc");
        web_cors_ok = false;
        send_http_header(403, RESPONSE_TYPE_JSON);
        web_client.println("{\"ok\":false,\"err\":\"forbidden\"}");
        return;
    }

    setupStruct setupData;
    setupData.paramName = param_name;
    setupData.paramValue = param_value;
    setupData.returnCode = 255;
    setupData.returnValue = "";

#if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
    if (param_name.equals("tdecktheme"))
    { // TD-THEME-01 W3: same path as the console --theme (store, save_settings, tdeck_theme_reapply); only auto|dark|light
        const int t = tdeck_theme_parse(param_value.c_str());
        if (t >= 0)
        {
            char message_text[32];
            snprintf(message_text, sizeof(message_text), "--theme %s", tdeck_theme_name((uint8_t)t));
            commandAction(message_text, isPhoneReady == 1);
            setupData.returnCode = (meshcom_settings.node_tdeck_theme == t) ? WS_RETURNCODE_OKAY : WS_RETURNCODE_FAIL;
        }
        else
            setupData.returnCode = WS_RETURNCODE_FAIL;
        setupData.returnValue = tdeck_theme_name(meshcom_settings.node_tdeck_theme);
    }
    else if (param_name.equals("tdchatfont"))
    { // WEB-TD-01: same path as the console --chatfont s|m|l
        const int f = tdeck_chatfont_parse(param_value.c_str());
        if (f >= 0)
        {
            char message_text[32];
            snprintf(message_text, sizeof(message_text), "--chatfont %s", tdeck_chatfont_name((uint8_t)f));
            commandAction(message_text, isPhoneReady == 1);
            setupData.returnCode = (meshcom_settings.node_tdeck_chatfont == f) ? WS_RETURNCODE_OKAY : WS_RETURNCODE_FAIL;
        }
        else
            setupData.returnCode = WS_RETURNCODE_FAIL;
        setupData.returnValue = tdeck_chatfont_name(meshcom_settings.node_tdeck_chatfont);
    }
    else if (param_name.equals("tdbldisp"))
    { // WEB-TD-01: display mode auto|light|dark = --bl mode auto|day|night; the answer is "<mode>:<active level>"
        const char *v = param_value.c_str();
        const char *cmd = strcmp(v, "auto") == 0 ? "auto" : strcmp(v, "light") == 0 ? "day" : strcmp(v, "dark") == 0 ? "night" : NULL;
        if (cmd != NULL)
        {
            char message_text[32];
            snprintf(message_text, sizeof(message_text), "--bl mode %s", cmd);
            commandAction(message_text, isPhoneReady == 1);
            setupData.returnCode = (brightnessMode() == brightness_mode_parse(cmd)) ? WS_RETURNCODE_OKAY : WS_RETURNCODE_FAIL;
        }
        else
            setupData.returnCode = WS_RETURNCODE_FAIL;
        static const char *const dispNames[3] = {"auto", "light", "dark"};
        setupData.returnValue = String(dispNames[brightness_mode_to_seg(brightnessMode())]) + ":" + String((int)brightnessActiveLevel());
    }
    else if (param_name.equals("tdblbright"))
    { // WEB-TD-01: level of the slot the display mode selects -- the device slider's own call (no console form edits "the active slot")
        char *end = NULL;
        const long lv = strtol(param_value.c_str(), &end, 10);
        if (end != param_value.c_str() && *end == 0 && lv >= 1 && lv <= BRIGHTNESS_STEPS)
        {
            brightnessSetActiveLevel((uint8_t)lv);
            tdeck_refresh_SET_view(); // the device's own setup slider shows the new level (loop task, like --bl mode / --chatfont)
            setupData.returnCode = WS_RETURNCODE_OKAY;
        }
        else
            setupData.returnCode = WS_RETURNCODE_FAIL;
        setupData.returnValue = String((int)brightnessActiveLevel());
    }
    else
#endif
    webSetup_setParam(&setupData);

    //Serial.printf("pName: %s\n", setupData.paramName);
    //Serial.printf("pValue: %s\n", setupData.paramValue);
    //Serial.printf("retcode: %i\n", setupData.returnCode);
    //Serial.printf("retvalue: %s\n", setupData.returnValue);

    if (setupData.returnCode == WS_RETURNCODE_OKAY)
        send_http_header(200, RESPONSE_TYPE_JSON);
    else
        send_http_header(422, RESPONSE_TYPE_JSON);

    // JSN-01: paramName/returnValue are unescaped user input (SSID, password,
    // node_atxt free text, the static IP fields, ...) landing here via a raw
    // %s -- a '"' broke the response. Build via ArduinoJson so both key and
    // value are escaped on serialise. Example: {"returncode":1,"setcall":"AB1CDE-12"}
    JsonDocument doc;
    doc["returncode"] = setupData.returnCode;
    doc[setupData.paramName] = setupData.returnValue;
    serializeJson(doc, web_client);
    web_client.println();
}

/**
 * ###########################################################################################################################
 * Get the Value of a parameter (if known)
 * This will return a JSON containing the result like {"parameterName":"parameterValue"} using a HTTP return code (e.g. 200 or 422)
 */
void getparam(String web_header)
{
    // CS-04, zwei Fehler in zwei Zeilen -- beide auf DK5EN-98 nachgestellt:
    //
    //  1. Gesucht wurde "/setparam/?" statt "/getparam/?". indexOf() liefert
    //     dann -1, und substring(-1+11 = 10, ...) schneidet den Namen aus der
    //     falschen Stelle des Headers: aus "GET /getparam/?gateway HTTP/1.1"
    //     wurde der Parametername "ram/?gateway" -> immer returncode 2.
    //  2. Bei vorhandenem "=" wurde substring(indexOf("=")) genommen, also der
    //     Teil AB dem Gleichheitszeichen. Der Kommentar sagt das Gegenteil
    //     ("We only do need the parameter Name"): "/getparam/?gateway=" ergab
    //     den Namen "=". Richtig ist substring(0, indexOf("=")).
    //
    // Damit war JEDES Lesen ueber die Web-API defekt, waehrend /setparam/
    // funktionierte -- die Web-GUI faellt es nicht auf, weil sie ihre Werte aus
    // den gerenderten Seiten nimmt, nicht ueber /getparam/.
    web_header = web_header.substring(web_header.indexOf("/getparam/?") + 11, web_header.indexOf(" HTTP/1.1"));
    if (web_header.indexOf("=") > 0)
    {
        web_header = web_header.substring(0, web_header.indexOf("=")); // maybe there is an unintended "=" or anything more. We only do need the parameter Name.
    }

    web_header.trim();

    String param_name = decodeURLPercentCoding(web_header);
    param_name.toLowerCase();

    setupStruct setupData;
    setupData.paramName = param_name;
    setupData.paramValue = "";
    setupData.returnCode = 255;
    setupData.returnValue = "";

    webSetup_getParam(&setupData);

    // Serial.printf("pName: %s\n", setupData.paramName);
    // Serial.printf("pValue: %s\n", setupData.paramValue);
    // Serial.printf("retcode: %i\n", setupData.returnCode);
    // Serial.printf("retvalue: %s\n", setupData.returnValue);

    if (setupData.returnCode == WS_RETURNCODE_OKAY)
        send_http_header(200, RESPONSE_TYPE_JSON);
    else
        send_http_header(422, RESPONSE_TYPE_JSON);

    // JSN-01: same fix as setparam() above -- build via ArduinoJson instead
    // of an unescaped %s. Example: {"returncode":1,"setcall":"AB1CDE-12"}
    JsonDocument doc;
    doc["returncode"] = setupData.returnCode;
    doc[setupData.paramName] = setupData.returnValue;
    serializeJson(doc, web_client);
    web_client.println();
}
