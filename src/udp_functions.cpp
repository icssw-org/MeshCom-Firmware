/**
 *  @author      Ralph Weich (DD5RW)
 *  @date        2025-12-03
 */
#include <Arduino.h>

#include <udp_functions.h>
#include <extudp_functions.h>
#include <debugconf.h>
#include <batt_functions.h>
#include <command_functions.h>
#include <loop_functions_extern.h>
#include <dedup_functions.h>
#include "ack_attribution.h"
#include <lora_functions.h>
#include <time_functions.h>
#include <lora_setchip.h>
#include <configuration.h>
#include "configuration_global.h"
#include "ArduinoJson.h"
#include "web_functions/web_functions.h"

#include "printfdeb_functions.h"

#include "via_functions.h"
#include "regex_functions.h"
#include "conf_frame.h"
#include "setlog_lines.h"

#if defined(ESP32)
#include "esp_task_wdt.h"
#endif

#if defined(BOARD_T_ETH_ELITE) || defined(BOARD_T_CONNECT_PRO)
#include "esp32/esp32_eth.h"
extern EspETH neth;
#endif

String grc_ids;

///////////////////////////////////////////////////////////////////////////////
// ESP32

#ifdef ESP32

#include "ntp_async.h"
#include <time.h>

// WIFI
#include <WiFi.h>
#include <WiFiClient.h>
#include "esp_wifi.h"
#include <ESP32Ping.h>
#include <Preferences.h>

IPAddress node_ip = IPAddress(0,0,0,0);
IPAddress node_gw = IPAddress(0,0,0,0);
IPAddress node_ms = IPAddress(0,0,0,0);
IPAddress node_dns = IPAddress(0,0,0,0);
IPAddress node_ntp = IPAddress(0,0,0,0);

IPAddress node_hostip = IPAddress(0,0,0,0);
// TM-31 bench hook (fork-only): --srvip <a.b.c.d> points the gateway at a test
// server (tools/mock/meshcom_server.py) instead of the built-in names.
IPAddress bench_srvip = IPAddress(0,0,0,0);

String s_node_ip = "";
String s_node_hostip = "";

String strSource_call;

bool hasIPaddress = false;

extern bool hasExternIPaddress;

WiFiUDP Udp;

NtpAsync timeClient(Udp);   // TM-35: non-blocking, reply harvested in getMeshComUDP()

unsigned char incomingPacket[UDP_TX_BUF_SIZE];  // buffer for incoming packets
int packetSize=0;
uint8_t convBuffer[UDP_TX_BUF_SIZE + 50]; // we need an extra buffer for udp tx, as we add other stuff (ID, RSSI, SNR, MODE)

bool udp_is_busy = false;
uint16_t lora_tx_msg_len = 0;

unsigned long last_upd_timer = 0; // last time we got a HB
extern bool hb_warn_logged;
bool had_initial_udp_conn = false;  // indicator that we had already a udp connection

// RX-01 (BACKLOG 3.8k), second door: shared with lora_functions.cpp, which
// defines both (the primary RX-side guard lives there, in OnRxDone).
extern uint32_t stat_rx_drop_unconfigured;
void logRxDropUnconfigured(const char *call);

uint8_t err_cnt_udp_tx = 0;    // counter on errors sending message via UDP

// ---- TM-31 UDP instrument (fork-only) --------------------------------------
// The receive path had no observable output at all: every DEBUG_MSG() compiles
// away (DO_DEBUG 0) and the printfdeb() lines need --debug on, so "no datagram
// arrives" could not be told apart from "nothing is logged". These counters are
// always kept; --udplog on adds one line per datagram, --udpstat prints them.
bool bUDPLOG = false;
static uint32_t s_udpRxCount = 0;
static uint32_t s_udpTxCount = 0;
static uint32_t s_udpTxFail = 0;
static uint32_t s_udpRxLastMs = 0;
static uint32_t s_udpTxLastMs = 0;
static IPAddress s_udpRxLastIp = IPAddress(0,0,0,0);
static uint16_t s_udpRxLastPort = 0;
static uint16_t s_udpRxLastLen = 0;

void udpCountTx(bool ok)
{
  s_udpTxCount++;
  s_udpTxLastMs = millis();
  if(!ok)
    s_udpTxFail++;
}

void udpPrintStat()
{
  Serial.printf("[UDPSTAT];bind;%u;ip;%s;dest;%s;rx;%lu;rx_last_ms;%lu;rx_from;%s;%u;rx_len;%u;tx;%lu;tx_fail;%lu;tx_last_ms;%lu\n",
                (unsigned)LOCAL_PORT,
                s_node_ip.c_str(),
                s_node_hostip.c_str(),
                (unsigned long)s_udpRxCount,
                (unsigned long)s_udpRxLastMs,
                s_udpRxLastIp.toString().c_str(), (unsigned)s_udpRxLastPort,
                (unsigned)s_udpRxLastLen,
                (unsigned long)s_udpTxCount,
                (unsigned long)s_udpTxFail,
                (unsigned long)s_udpTxLastMs);
}


void getMeshComUDP()
{
  if(bWIFIAP)
    return;

  meshcom_settings.node_hasIPaddress = hasIPaddress;
  
  if(!hasIPaddress)
    return;

  ifalseping = 5;

  // check if we received a UDP packet
  packetSize = Udp.parsePacket();
  
  if (packetSize > 0)
  {
    IPAddress remote_ip = Udp.remoteIP();
    uint16_t remote_port = Udp.remotePort();

    int len = Udp.read(incomingPacket, UDP_TX_BUF_SIZE - 1);

    if (len > 0)
    {
      incomingPacket[len] = 0;

      // TM-31: one always-kept counter set, one optional line per datagram
      s_udpRxCount++;
      s_udpRxLastMs = millis();
      s_udpRxLastIp = remote_ip;
      s_udpRxLastPort = remote_port;
      s_udpRxLastLen = (uint16_t)len;

      if(bUDPLOG)
        Serial.printf("[UDP];rx;ip;%s;port;%u;len;%d;head;%02X%02X%02X%02X\n",
                      remote_ip.toString().c_str(), (unsigned)remote_port, len,
                      incomingPacket[0], len > 1 ? incomingPacket[1] : 0,
                      len > 2 ? incomingPacket[2] : 0, len > 3 ? incomingPacket[3] : 0);

      // TM-35: the NTP reply shares this socket with the gateway traffic
      if(timeClient.tryConsume(remote_ip, remote_port, incomingPacket, len))
        return;

      // DR-20 (2026-09-12 decided, implemented 2026-09-17 wave W6): the
      // handler is a parser, not connectivity policy -- it now only reports
      // the too-many-zeros verdict (1) instead of resetting the socket
      // itself. This caller performs the reset, mirroring NrfETH::getUDP()
      // (nrf52/nrf_eth.cpp:442-453), which resets DHCP on the same verdict
      // from handleUdpFrame_nrf52(). Losing this call would silently stop
      // re-arming the UDP socket on every too-many-zeros datagram -- a real
      // gateway regression, not just a caller-decides refactor. The verdict
      // no longer always tears WiFi down (see H6-01 below).
      //
      // H6-01: a malformed datagram (too many consecutive zero bytes) is
      // evidence the socket needs re-arming, not that the link is dead. The
      // full WiFi teardown in resetMeshComUDP() belongs to the heartbeat
      // paths in gateway_service_esp32.cpp, which only call it once
      // WiFi.status() != WL_CONNECTED already confirms the link is down,
      // and to the MAX_ERR_UDP_TX path in udp_drain_esp32.cpp.
      if(handleUdpFrame_esp32(incomingPacket, len, remote_ip) != 0)
      {
        if(WiFi.status() == WL_CONNECTED)
        {
          Udp.stop();
          err_cnt_udp_tx = 0;
          if(Udp.begin(LOCAL_PORT))
            printlndeb("[WIFI-DBG] too-many-zeros: UDP socket re-armed, WiFi kept");
          else
            resetMeshComUDP();   // bind failed: the old recovery is all that is left
        }
        else
        {
          resetMeshComUDP();
        }
      }
    }
  }
}

// TM-45: harvest-only substitute for getMeshComUDP(), for a bGATEWAY-off
// node. Same socket, same NtpAsync instance -- but no GATE/CONF/BEAT
// parsing and no gateway bookkeeping (last_upd_timer, [GW];rx, ...), since
// there is no gateway consumer for a non-NTP datagram on this path.
// esp32_main.cpp calls this only from the bGATEWAY-off branch, so it never
// runs in the same loop pass as getMeshComUDP() -- one read of the socket
// per pass either way.
void ntpHarvestUDP()
{
  if(bWIFIAP)
    return;

  meshcom_settings.node_hasIPaddress = hasIPaddress;

  if(!hasIPaddress)
    return;

  ntpHarvestReply(Udp, timeClient);
}


// C2 carve-out (DRY unification U2): the three socket primitives the datagram
// write goes through, paired one for one with the _nrf52 set in
// nrf52/nrf_eth.h, so a native test can replace them with a recording sink.
//
// Three and not one, because the caller's error policy runs *between* them: a
// failed write() that trips MAX_ERR_UDP_TX resets the socket and returns
// without ever calling endPacket(), while a failed write() that does not trip
// it still falls through to endPacket(). A single send primitive cannot
// express that ordering, and begin stays separate because on nRF52 a debug
// print sits between begin and write.
//
// The write result is checked here and nowhere on nRF52: NrfETH::sendUDP()
// writes byte by byte, ignores every Udp.write() result and reports only
// endPacket(), so a failed write cannot reset the socket there. That
// asymmetry is D1 drift-matrix material; preserved here, not fixed.
bool udpBeginRaw_esp32()
{
    return Udp.beginPacket(node_hostip , UDP_PORT) != 0;
}

bool udpWriteRaw_esp32(const uint8_t *buf, uint16_t len)
{
    return Udp.write(buf, len) != 0;
}

bool udpEndRaw_esp32()
{
    return Udp.endPacket() != 0;
}

// C2/U2 carve: sendMeshComUDP() lives in esp32/udp_drain_esp32.cpp


// (innerhalb #ifdef ESP32 seit Zeile 37)
///////////////////////////////////////////////////////////////////////////////
// WLAN-Aufbau (TM-34, 2026-08-30). Grundsaetze, aus wifi-findings-20260829.md:
//  - Der Treiber waehlt den AP (ALL_CHANNEL_SCAN + Sortierung nach Signal),
//    bei jedem Verbindungsversuch neu. Kein BSSID/Kanal-Pin mehr (F1/F2).
//  - Der Treiber besitzt das Wiederverbinden (setAutoReconnect). Wir holen
//    das Ergebnis ueber das GOT_IP-Ereignis ab, nicht ueber ein Zeitfenster
//    (F3) -- ein spaeter Verbindungsaufbau wird nicht mehr uebersehen.
//  - Der Watchdog greift spaet und sanft: erst nach WIFI_WD_GRACE_S ein
//    Neuverbinden mit stehendem Funk, erst nach der zweiten Frist ein
//    Funk-Reset (F4).
//  - WPA2/WPA3-Uebergangs-APs: SAE braucht PMF; ohne PMF authentifiziert die
//    Station mit WPA2-PSK. Auf dem Bench-Orbi schlug die SAE-Anmeldung in
//    24/24 Starts fehl (AUTH_EXPIRE), WPA2-PSK gelang 24/24 (Arme A0/A5).
//    WIFI_SAE_POLICY: 0 = Treiber-Standard (SAE, wenn der AP es anbietet),
//    1 = PMF aus -> WPA2-PSK (Standard), 2 = SAE mit H2E+Hunt-and-Peck.
//  - Jede blockierende Treiberstelle meldet sich ab WIFI_STALL_MS als
//    [WIFI];stall;<site>;ms;N;task;X (F5); DNS laeuft asynchron (F6).
// Bench-Marken sind rohes Serial.printf: printfdeb() entfernt ';' ausserhalb
// von --debug csv.

#if !defined(WIFI_SAE_POLICY)
  #define WIFI_SAE_POLICY 1
#endif
#if !defined(WIFI_STALL_MS)
  #define WIFI_STALL_MS 50
#endif
#if !defined(WIFI_WD_GRACE_S)
  #define WIFI_WD_GRACE_S 180
#endif
#define WIFI_POLL_GIVEUP     20      // 1-s-Takte doWiFiConnect(), danach nur noch Ereignis-Abholung
#define WIFI_DRIVER_IDLE_MS  60000   // kein Treiberereignis so lange -> Treiber hat aufgegeben
#define WIFI_OFFLINE_MAX_MS  900000  // laengste Zeit ohne got_ip, bevor der 5-min-Pfad den Funk zyklisch neu startet

#include "lwip/dns.h"

// Interne Treibersymbole (libnet80211.a, esp_wifi_driver.h des Supplicant,
// nicht in den oeffentlichen Headern): nur fuer die Protokollzeile, welcher
// Anmeldemodus tatsaechlich ausgehandelt wurde.
extern "C" bool esp_wifi_sta_prof_is_wpa3_internal(void);
extern "C" bool esp_wifi_sta_pmf_enabled(void);

// F5: benannte Blockierstelle. SNR gibt es auf dem ESP32 nicht, nur RSSI.
struct WifiStall
{
  const char *site;
  uint32_t t0;
  WifiStall(const char *s) : site(s), t0(millis()) {}
  ~WifiStall()
  {
    uint32_t d = (uint32_t)(millis() - t0);
    if(d >= WIFI_STALL_MS)
      Serial.printf("[WIFI];stall;%s;ms;%lu;task;%s\n", site, (unsigned long)d, pcTaskGetName(NULL));
  }
};

static bool s_wifiInitDone = false;
static volatile bool s_wifiStaUp = false;         // STA_START..STA_STOP (kein esp_wifi_*-Aufruf noetig)
static volatile bool s_wifiGotIpPending = false;  // vom Ereignis-Task gesetzt, von loopTask abgeholt
static volatile bool s_wifiRestartWanted = false; // resetMeshComUDP(): 5-min-Pfad sofort
static volatile uint32_t s_wifiLastEventMs = 0;
static volatile uint32_t s_wifiLastGotIpMs = 0;
static volatile uint32_t s_wifiLastDiscMs = 0;
static volatile uint32_t s_wifiGotIpCount = 0;
static volatile int s_wifiLastReason = 0;
static uint32_t s_wifiBringupMs = 0;              // letzter startNetwork()
static uint32_t s_wifiDownSince = 0;              // F4: erster Tick ohne WL_CONNECTED bei hasIPaddress
static uint8_t  s_wifiWdStage = 0;
static wifi_ap_record_t s_wifiAp;                 // AP bei connected/got_ip, fuer die disconnect-Zeile
static bool s_wifiApValid = false;

static void wifiAssocLog(const char *what, int reason)
{
  wifi_ap_record_t ap;
  bool live = (esp_wifi_sta_get_ap_info(&ap) == ESP_OK);
  if(live) { s_wifiAp = ap; s_wifiApValid = true; }
  else if(s_wifiApValid) ap = s_wifiAp;
  else memset(&ap, 0, sizeof(ap));

  char phy[8] = {0};
  if(ap.phy_11b) strcat(phy, "b");
  if(ap.phy_11g) strcat(phy, "g");
  if(ap.phy_11n) strcat(phy, "n");
  if(ap.phy_lr)  strcat(phy, "L");

  Serial.printf("[WIFI];assoc;%s;ssid;%s;bssid;%02X:%02X:%02X:%02X:%02X:%02X;chan;%d;rssi;%d;auth;%d;phy;%s;pmf;%d;wpa3;%d;live;%d;reason;%d;ms;%lu\n",
    what, (const char*)ap.ssid,
    ap.bssid[0], ap.bssid[1], ap.bssid[2], ap.bssid[3], ap.bssid[4], ap.bssid[5],
    (int)ap.primary, (int)ap.rssi, (int)ap.authmode, phy,
    esp_wifi_sta_pmf_enabled() ? 1 : 0, esp_wifi_sta_prof_is_wpa3_internal() ? 1 : 0,
    live ? 1 : 0, reason, (unsigned long)millis());
}

// Laeuft auf dem Arduino-Ereignis-Task, nicht auf loopTask: nur Flags und Log.
static void wifiEventLog(WiFiEvent_t event, WiFiEventInfo_t info)
{
  s_wifiLastEventMs = millis();
  switch(event)
  {
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      s_wifiLastReason = (int)info.wifi_sta_disconnected.reason;
      s_wifiLastDiscMs = millis();
      Serial.printf("[WIFI];event;disconnected;reason;%d;ms;%lu\n", s_wifiLastReason, (unsigned long)millis());
      wifiAssocLog("disconnected", s_wifiLastReason);
      break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.printf("[WIFI];event;connected;ms;%lu\n", (unsigned long)millis());
      wifiAssocLog("connected", 0);
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      s_wifiLastGotIpMs = millis();
      s_wifiGotIpCount++;
      s_wifiGotIpPending = true;
      Serial.printf("[WIFI];event;got_ip;ms;%lu\n", (unsigned long)millis());
      wifiAssocLog("got_ip", 0);
      break;
    case ARDUINO_EVENT_WIFI_STA_AUTHMODE_CHANGE:
      Serial.printf("[WIFI];event;authmode;%d;ms;%lu\n", (int)info.wifi_sta_authmode_change.new_mode, (unsigned long)millis());
      break;
    case ARDUINO_EVENT_WIFI_STA_START:
      s_wifiStaUp = true;
      break;
    case ARDUINO_EVENT_WIFI_STA_STOP:
      s_wifiStaUp = false;
      break;
    default:
      break;
  }
}

// F1: einmal, vor dem ersten WiFi.mode() -- persistent() wird nur beim
// Low-Level-Init gelesen (WiFiGeneric.cpp wifiLowLevelInit).
static void wifiInitOnce()
{
  if(s_wifiInitDone)
    return;
  s_wifiInitDone = true;
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);
  WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL);
  WiFi.onEvent(wifiEventLog);
}

static void wifiSetTxPower()
{
  if(meshcom_settings.node_wifi_power > 0)
    esp_wifi_set_max_tx_power((int8_t)meshcom_settings.node_wifi_power*4);
  else
    WiFi.setTxPower(WIFI_POWER_8_5dBm);
}

// F2: Verbindung nur ueber die SSID; AP-Wahl macht der Treiber. Konfiguration
// setzen, Sicherheitsrichtlinie anwenden, dann verbinden.
static void wifiBegin()
{
  WifiStall st("begin");

  WiFi.mode(WIFI_STA);
  wifiSetTxPower();

  // offenes WLAN: "empty" (neu) ODER "none" (Bestand, Flash-Default und Hilfetext) akzeptieren
  const char *pwd = meshcom_settings.node_pwd;
  if(strcmp(pwd, "empty") == 0 || strcmp(pwd, "none") == 0)
    pwd = NULL;

  printfdeb("[WIFI]...try connecting to SSID: %s \n", meshcom_settings.node_ssid);
  WiFi.begin(meshcom_settings.node_ssid, pwd, 0, NULL, false);   // Konfiguration, noch kein connect

  #if WIFI_SAE_POLICY == 1
  {
    esp_err_t rc = esp_wifi_disable_pmf_config(WIFI_IF_STA);
    Serial.printf("[WIFI];policy;pmf_off;rc;%d\n", (int)rc);
  }
  #elif WIFI_SAE_POLICY == 2
  {
    wifi_config_t c;
    if(esp_wifi_get_config(WIFI_IF_STA, &c) == ESP_OK)
    {
      c.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;
      esp_err_t rc = esp_wifi_set_config(WIFI_IF_STA, &c);
      Serial.printf("[WIFI];policy;sae_h2e_both;rc;%d\n", (int)rc);
    }
  }
  #else
  Serial.printf("[WIFI];policy;driver_default\n");
  #endif

  esp_wifi_connect();
}

// Asynchroner WLAN-Scan: nur noch Protokoll der hoerbaren APs (Felddiagnose),
// das Ergebnis geht nicht mehr in WiFi.begin().
static bool s_wifiScanPending = false;
// TM-16: Zaehler der Funk-Hochlaeufe seit dem Reset. Der erste laeuft ohne den
// reinen Diagnose-Scan (siehe startNetwork()), jeder weitere mit.
static uint16_t s_wifiBringupCount = 0;
static int  s_wifiScanPolls = 0;

static void wifiLogScan(int nrAps)
{
  for (int i = 0; i < nrAps; ++i)
  {
     if(strcmp(WiFi.SSID(i).c_str(), meshcom_settings.node_ssid) == 0)
     {
        uint8_t *bssid = WiFi.BSSID(i);
        printfdeb("[WIFI]...SSID: %s CHAN: %d RSSI: %d BSSID: %02X:%02X:%02X:%02X:%02X:%02X\n",
          WiFi.SSID(i).c_str(), (int) WiFi.channel(i), (int) WiFi.RSSI(i),
          bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5]);
     }
  }
  WiFi.scanDelete();
}

// F6: DNS ohne loopTask-Blockade. WiFi.hostByName() wartet bis zu 31 s auf
// dem aufrufenden Task; hier wird lwIP direkt gefragt (dieselbe Funktion, die
// hostByName() intern nutzt), das Ergebnis kommt per Callback vom tcpip-Task
// und wird aus der Hauptschleife angewendet (wifiDnsPoll). Literal-IPs sofort,
// jeder Name pro Boot nur einmal (Cache), bei Fehlschlag Wiederholung ueber
// sendMeshComHeartbeat().
struct WifiDns
{
  char name[64];
  IPAddress result;
  volatile bool pending;
  volatile bool done;
  bool applied;
  uint32_t t0;
};
static WifiDns s_dnsSrv = {{0}, IPAddress(0,0,0,0), false, false, false, 0};
static WifiDns s_dnsNtp = {{0}, IPAddress(0,0,0,0), false, false, false, 0};

static void wifiDnsCb(const char *name, const ip_addr_t *ipaddr, void *arg)
{
  WifiDns *d = (WifiDns*)arg;
  if(ipaddr != NULL && ipaddr->u_addr.ip4.addr != 0)
    d->result = IPAddress(ipaddr->u_addr.ip4.addr);
  else
    d->result = IPAddress(0,0,0,0);
  d->pending = false;
  d->done = true;
}

static void wifiDnsStart(WifiDns *d, const char *host)
{
  if(host == NULL || host[0] == 0)
    return;
  if(d->pending)
    return;

  bool same = (strncmp(d->name, host, sizeof(d->name)) == 0);
  if(!same)
  {
    snprintf(d->name, sizeof(d->name), "%s", host);
    d->result = IPAddress(0,0,0,0);
  }
  d->applied = false;
  d->done = false;

  IPAddress lit;
  if(lit.fromString(host))            // Literal: keine Aufloesung
  {
    d->result = lit;
    d->done = true;
    return;
  }
  if(same && (uint32_t)d->result != 0) // Cache: einmal pro Boot
  {
    d->done = true;
    return;
  }

  ip_addr_t addr;
  d->t0 = millis();
  err_t err;
  { WifiStall st("dns"); err = dns_gethostbyname(host, &addr, wifiDnsCb, d); }
  if(err == ERR_OK)
  {
    d->result = IPAddress(addr.u_addr.ip4.addr);
    d->done = true;
  }
  else if(err == ERR_INPROGRESS)
  {
    d->pending = true;
  }
  else
  {
    Serial.printf("[WIFI];dns;%s;err;%d\n", host, (int)err);
    d->done = true;
  }
}

// Aus der Hauptschleife: aufgeloeste Adressen anwenden.
void wifiDnsPoll()
{
  if(s_dnsSrv.done && !s_dnsSrv.applied)
  {
    s_dnsSrv.applied = true;
    node_hostip = s_dnsSrv.result;
    // TM-31 bench hook: after the server selection, so it is not overwritten
    if(bench_srvip != IPAddress(0,0,0,0))
    {
      node_hostip = bench_srvip;
      printfdeb("[WIFI]...BENCH srvip override -> %s\n", node_hostip.toString().c_str());
    }
    s_node_hostip = node_hostip.toString();
    Serial.printf("[WIFI];dns;%s;ip;%s;ms;%lu\n", s_dnsSrv.name, s_node_hostip.c_str(), (unsigned long)(millis() - s_dnsSrv.t0));
    if((uint32_t)node_hostip != 0 && hasIPaddress)
      sendMeshComHeartbeat();     // gateway activity (KEEP)
  }
  if(s_dnsNtp.done && !s_dnsNtp.applied)
  {
    s_dnsNtp.applied = true;
    Serial.printf("[WIFI];dns;%s;ip;%s;ms;%lu\n", s_dnsNtp.name, s_dnsNtp.result.toString().c_str(), (unsigned long)(millis() - s_dnsNtp.t0));
    if((uint32_t)s_dnsNtp.result != 0)
      timeClient.setPoolServerIP(s_dnsNtp.result);
  }
}

 bool startNetwork()
 {
  #if defined(HAS_ETHERNET)

      if(meshcom_settings.node_netmode == 1)
      {
          printlndeb("[NET] Ethernet mode");
          
          if(!meshcom_settings.node_hasIPaddress)
              neth.initethDHCP();

          return meshcom_settings.node_hasIPaddress;
      }

  #endif

  #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
  {
    if (!meshcom_settings.node_wifion)
    {
      printlndeb("[WIFI]...disabled by Settings (node_wifion=false)");
      return false;
    }

    printlndeb("[WIFI]...enabled by Settings (node_wifion=true)");
  }
  #endif

  if(meshcom_settings.node_netmode == 0 && meshcom_settings.node_hasIPaddress)
  {
    if (bDEBUG)
        printlndeb("[WIFI]...meshcom_settings.node_hasIPaddress=true");

    return false;
  }

  if(meshcom_settings.node_ssid[0] == 0x00 || is_equ(meshcom_settings.node_ssid, "none"))
  {
    return false;
  }

  wifiInitOnce();
  s_wifiRestartWanted = false;
  s_wifiDownSince = 0;
  s_wifiWdStage = 0;
  s_wifiBringupMs = millis();

  if (bWIFIAP)
  {
    if (bDEBUG)
        printlndeb("[WIFI]...bWIFIAP=true");

    { WifiStall st("disconnect"); WiFi.disconnect(true, true); }
    { WifiStall st("mode");       WiFi.mode(WIFI_OFF); }
    delay(1000);

    hasIPaddress=false;
    meshcom_settings.node_hasIPaddress = hasIPaddress;

    { WifiStall st("mode"); WiFi.mode(WIFI_AP); }
    WiFi.softAP(meshcom_settings.node_call);
    wifiSetTxPower();

    printfdeb("[WIFI]...AP mode ssid<%s> connected\n", meshcom_settings.node_call);

    startMeshComUDP();
    
    return true;
  }
  else
  {
    if (bDEBUG)
        printlndeb("[WIFI]...bWIFIAP=false");

    if(strcmp(meshcom_settings.node_ssid, "none") == 0)
    {
      printfdeb("[WIFI]...ST no ssid<%s> pwd<%s> not connected\n", meshcom_settings.node_ssid, maskSecret(meshcom_settings.node_pwd));
      return false;
    }
  }

  if (bDEBUG)
      printlndeb("[WIFI]...WiFi full radio reset");

  // Nicht blockieren (TM-20): Reset, asynchronen Scan starten, zurueck;
  // doWiFiConnect() (1-s-Takt der Hauptschleife) protokolliert das
  // Scanergebnis und ruft wifiBegin().
  { WifiStall st("disconnect"); WiFi.disconnect(true, true); }

  // Hostname hier statt in wifiInitOnce(): das laeuft nur einmal pro Boot
  // (s_wifiInitDone) und wuerde einem Rufzeichenwechsel hinterherhinken. Der
  // Core schreibt seinen statischen Hostname-Puffer bei jedem STA-Uebergang
  // neu (WiFiGeneric.cpp:1265, 2.x-Core), also ist es hier billig und aktuell --
  // wie der mDNS-Responder node_call bei jedem Webserver-Neustart neu liest.
  char host[32];
  if(makeDhcpHostname(host, sizeof(host), meshcom_settings.node_call))
    WiFi.setHostname(host);

  { WifiStall st("mode");       WiFi.mode(WIFI_OFF); WiFi.mode(WIFI_STA); }
  hasIPaddress=false;
  meshcom_settings.node_hasIPaddress = hasIPaddress;
  s_wifiGotIpPending = false;

  // TM-16 (Bootzeit): der Scan hier ist reine Diagnose. wifiLogScan() druckt nur
  // die passenden APs und wirft das Ergebnis weg; die AP-Auswahl macht seit
  // Welle W der Treiber selbst (wifiBegin() ruft WiFi.begin(ssid, pwd, 0, NULL,
  // false) -- Kanal 0, kein BSSID). Er kostet damit 3-5 s vor JEDEM begin(),
  // auch auf einem Knoten, bei dem nichts im Argen ist.
  //
  // Deshalb: beim ersten Hochlauf nach dem Reset ohne Scan direkt verbinden,
  // ab dem zweiten (Funk-Neustart, Watchdog -- also nachdem etwas schiefging)
  // wieder mit Scan. Die Felddiagnose bleibt genau dort erhalten, wo sie
  // gebraucht wird, und ein gesunder Knoten ist ~4-5 s frueher online.
  s_wifiBringupCount++;
  if(s_wifiBringupCount == 1)
  {
    Serial.printf("[WIFI];scan;skipped;first_bringup;ms;%lu\n", (unsigned long)millis());
    s_wifiScanPending = false;
    wifiBegin();
  }
  else
  {
    WiFi.scanDelete();
    { WifiStall st("scan"); WiFi.scanNetworks(true); }   // async, Ergebnis ueber WiFi.scanComplete()
    s_wifiScanPending = true;
    s_wifiScanPolls = 0;
  }

  iWlanWait = 1;

  return true;
}

// F3: got_ip des Treibers abholen -- aus doWiFiConnect() waehrend des
// Aufbaus und aus der Hauptschleife danach (auch fuer jedes spaetere
// Wiederverbinden). Liefert true, wenn UDP jetzt bereit ist.
bool wifiHarvestGotIp()
{
  if(!s_wifiGotIpPending)
    return false;
  s_wifiGotIpPending = false;

  if(WiFi.status() != WL_CONNECTED)
    return false;

  IPAddress ip = WiFi.localIP();
  if(hasIPaddress && ip == node_ip)
  {
    // Wiederverbinden mit derselben Adresse: Socket bleibt gueltig
    Serial.printf("[WIFI];harvest;same_ip;%s;ms;%lu\n", ip.toString().c_str(), (unsigned long)millis());
    s_wifiDownSince = 0;
    s_wifiWdStage = 0;
    return true;
  }

  printlndeb("");
  printlndeb("[WIFI]...connect OK");

  iWlanWait = 0;
  s_wifiScanPending = false;

  timeClient.begin();
  timeClient.setUpdateInterval(3600000); // Sets refresh interval to 1 hour (in ms)

  startMeshComUDP();

  return hasIPaddress;
}

bool doWiFiConnect()
{
  if(s_wifiScanPending)
  {
    int n = WiFi.scanComplete();
    if(n == WIFI_SCAN_RUNNING)
    {
      if(++s_wifiScanPolls <= 10)          // ein Scan dauert 3-5 s
      {
        printdeb("s");
        return false;
      }
      printlndeb("\n[WIFI]...scan timeout, connecting anyway");
      n = 0;
    }
    s_wifiScanPending = false;
    wifiLogScan(n < 0 ? 0 : n);
    wifiBegin();
    iWlanWait = 1;                         // Verbindungszaehler beginnt jetzt
    printdeb("[WIFI]...Wait connect ");
  }

  if(wifiHarvestGotIp())
    return true;

  printdeb(".");
  iWlanWait++;

  if(iWlanWait == 11)
    printfdeb("\n[WIFI]...ssid<%s> connection error\n", meshcom_settings.node_ssid);

  return false;
}

// 5-min-Pfad (esp32_main): Funk nur dann zyklisch neu starten, wenn der
// Treiber wirklich nichts mehr tut -- nicht, waehrend er noch verbindet.
bool wifiTrulyOffline()
{
  if(WiFi.status() == WL_CONNECTED)
    return false;
  if(s_wifiRestartWanted)
    return true;
  uint32_t now = millis();
  if(s_wifiLastEventMs == 0 || (uint32_t)(now - s_wifiLastEventMs) > WIFI_DRIVER_IDLE_MS)
    return true;
  uint32_t ref = (s_wifiLastGotIpMs > s_wifiBringupMs) ? s_wifiLastGotIpMs : s_wifiBringupMs;
  return (uint32_t)(now - ref) > WIFI_OFFLINE_MAX_MS;
}

void wifiRequestRestart()
{
  s_wifiRestartWanted = true;
}

// Bench-/Feldhaken: Verbindung treiberseitig trennen und neu waehlen lassen
// (dieselbe Aktion wie Watchdog-Stufe 1). Keine Konfigurationsaenderung.
void wifiDrop()
{
  if(!s_wifiStaUp || bWIFIAP)
  {
    Serial.println("[WIFI];drop;err;not in STA mode");
    return;
  }
  Serial.printf("[WIFI];drop;ms;%lu\n", (unsigned long)millis());
  { WifiStall st("disconnect"); WiFi.disconnect(false, false); }
  wifiBegin();
}

// Waehrend der Treiber scannt (Aufbau, Auto-Reconnect) blockiert jeder
// esp_wifi_*-Aufruf loopTask bis zum Scanende (gemessen: 2.9 s Loop-Luecke
// durch WiFi.getMode() im 1. Wurf). Deshalb hier nur Zustand aus Ereignissen
// und esp_wifi_sta_get_ap_info() nur im verbundenen Zustand.
void wifiLinkLog(const char *tag)
{
  unsigned long now = millis();
  if(s_wifiStaUp && WiFi.status() == WL_CONNECTED)
  {
    wifi_ap_record_t ap;
    if(esp_wifi_sta_get_ap_info(&ap) != ESP_OK)
      memset(&ap, 0, sizeof(ap));
    Serial.printf("[WIFI];%s;up;rssi;%d;bssid;%02X:%02X:%02X:%02X:%02X:%02X;chan;%d;age_s;%lu;got_ip_n;%lu;ip;%d;ms;%lu\n",
      tag, (int)ap.rssi,
      ap.bssid[0], ap.bssid[1], ap.bssid[2], ap.bssid[3], ap.bssid[4], ap.bssid[5],
      (int)ap.primary, (unsigned long)((now - s_wifiLastGotIpMs) / 1000),
      (unsigned long)s_wifiGotIpCount, hasIPaddress ? 1 : 0, now);
  }
  else
  {
    Serial.printf("[WIFI];%s;down;sta;%d;status;%d;down_s;%lu;last_reason;%d;got_ip_n;%lu;ip;%d;ms;%lu\n",
      tag, s_wifiStaUp ? 1 : 0, (int)WiFi.status(),
      (unsigned long)(s_wifiLastDiscMs ? (now - s_wifiLastDiscMs) / 1000 : 0),
      s_wifiLastReason, (unsigned long)s_wifiGotIpCount, hasIPaddress ? 1 : 0, now);
  }
}

// --wifistat
void wifiStat()
{
  wifiLinkLog("stat");
  Serial.printf("[WIFI];stat;ssid;%s;localip;%s;hostip;%s;iWlanWait;%d;wd_stage;%d;policy;%d;scan_pending;%d;bringups;%u;bringup_ms;%lu;last_got_ip_ms;%lu;last_disc_ms;%lu\n",
    meshcom_settings.node_ssid, WiFi.localIP().toString().c_str(), s_node_hostip.c_str(),
    iWlanWait, (int)s_wifiWdStage, (int)WIFI_SAE_POLICY, s_wifiScanPending ? 1 : 0,
    (unsigned)s_wifiBringupCount,
    (unsigned long)s_wifiBringupMs, (unsigned long)s_wifiLastGotIpMs, (unsigned long)s_wifiLastDiscMs);
}

// Alle 60 s aus der Hauptschleife
void wifiLinkHeartbeat()
{
  static unsigned long s_last = 0;
  if(s_last != 0 && (uint32_t)(millis() - s_last) < 60000)
    return;
  if(!s_wifiStaUp && !hasIPaddress)
    return;
  s_last = millis();
  wifiLinkLog("link");
}

// F4: Watchdog. Liefert false, solange die Verbindung fehlt; ifalseping == 0
// heisst fuer den Aufrufer (esp32_main): Funk-Reset ueber startNetwork().
bool checkWifiPing()
{
  if(bWIFIAP)
    return true;

  if(!hasIPaddress)
    return true;

  if(!s_wifiStaUp)
  {
    // Funk absichtlich aus (T-Deck WLAN-Schalter, --wifi off): kein Watchdog,
    // sonst wuerde Stufe 1 den Funk wieder einschalten. Bewusst kein
    // WiFi.getMode(): jeder esp_wifi_*-Aufruf wartet auf den WiFi-Task und
    // blockiert loopTask, solange der Treiber scannt (bis ~3 s).
    Udp.stop();
    hasIPaddress = false;
    meshcom_settings.node_hasIPaddress = hasIPaddress;
    s_wifiDownSince = 0;
    s_wifiWdStage = 0;
    return true;
  }

  if(WiFi.status() == WL_CONNECTED)
  {
    s_wifiDownSince = 0;
    s_wifiWdStage = 0;
    ifalseping = 5;
    if(bDEBUG && bDisplayCont)
      printfdeb("%s [WIFI]..WiFi connected\n", getTimeString().c_str());
    return true;
  }

  uint32_t now = millis();
  if(s_wifiDownSince == 0)
  {
    s_wifiDownSince = now;
    s_wifiWdStage = 0;
  }
  uint32_t down_s = (uint32_t)(now - s_wifiDownSince) / 1000;

  printfdeb("%s [WIFI]..WiFi not connected for %lu s (driver reconnecting, stage %d)\n", getTimeString().c_str(), (unsigned long)down_s, (int)s_wifiWdStage);

  if(s_wifiWdStage == 0 && down_s >= WIFI_WD_GRACE_S)
  {
    s_wifiWdStage = 1;
    Serial.printf("[WIFI];watchdog;reconnect;down_s;%lu;ms;%lu\n", (unsigned long)down_s, (unsigned long)now);
    { WifiStall st("disconnect"); WiFi.disconnect(false, false); }
    wifiBegin();
  }
  else if(s_wifiWdStage == 1 && down_s >= 2 * WIFI_WD_GRACE_S)
  {
    s_wifiWdStage = 2;
    Serial.printf("[WIFI];watchdog;reset;down_s;%lu;ms;%lu\n", (unsigned long)down_s, (unsigned long)now);
    Udp.stop();
    hasIPaddress = false;
    meshcom_settings.node_hasIPaddress = hasIPaddress;
    ifalseping = 0;       // Aufrufer: stopWebserver() + startNetwork()
    return false;
  }

  ifalseping = 5;         // Aufrufer prueft in 5 s wieder
  return false;
}

String udpUpdateTimeClient()
{
  // TM-35: ask for a refresh and return at once; the reply lands in getMeshComUDP()
  timeClient.requestNow();
  timeClient.loop();

  if(!timeClient.isTimeSet())
  {
    printlndeb("TimeClient no update possible --> CPU-Mode");
    return "none";
  }

  if(bDisplayInfo)
  {
    printdeb("TimeClient now (UTC): ");
    printlndeb(timeClient.getFormattedTime());
  }

  return timeClient.getFormattedTime();
}

String udpGetTimeClient()
{
  timeClient.loop();

  if(!timeClient.isTimeSet())
    return "none";

  return timeClient.getFormattedTime();
}

String udpGetDateClient()
{
  if(!timeClient.isTimeSet())
    return "none";

  return getDateTime(timeClient.getEpochTime());
}

void startMeshComUDP()
{
  if(bWIFIAP)
  {
    WiFi.softAPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0));

    node_ip = WiFi.softAPIP();
    
    if(meshcom_settings.node_wifi_power > 0)
      esp_wifi_set_max_tx_power((int8_t)meshcom_settings.node_wifi_power*4);  //parameter = dBm * 4
    else
      WiFi.setTxPower(WIFI_POWER_8_5dBm);
      
    snprintf(meshcom_settings.node_ip, sizeof(meshcom_settings.node_ip), "%i.%i.%i.%i", node_ip[0], node_ip[1], node_ip[2], node_ip[3]);
    snprintf(meshcom_settings.node_subnet, sizeof(meshcom_settings.node_subnet), "255.255.255.0");
  }
  else
  // Wifi IP-Addess static
  if(strlen(meshcom_settings.node_ownip) >= 7 && strlen(meshcom_settings.node_owngw) >= 7 && strlen(meshcom_settings.node_ownms) >= 7)
  {
    snprintf(meshcom_settings.node_ip, sizeof(meshcom_settings.node_ip), "%s", meshcom_settings.node_ownip);
    snprintf(meshcom_settings.node_gw, sizeof(meshcom_settings.node_gw), "%s", meshcom_settings.node_owngw);

    if(strlen(meshcom_settings.node_owndns) >= 7)
      snprintf(meshcom_settings.node_dns, sizeof(meshcom_settings.node_dns), "%s", meshcom_settings.node_owndns);
    else
      snprintf(meshcom_settings.node_dns, sizeof(meshcom_settings.node_dns), "%s", (char*)"8.8.8.8");

    snprintf(meshcom_settings.node_subnet, sizeof(meshcom_settings.node_subnet), "%s", meshcom_settings.node_ownms);

    // Set your Static IP address
    node_ip.fromString(meshcom_settings.node_ownip);
    // Set your Gateway IP address
    node_gw.fromString(meshcom_settings.node_owngw);
    // Set your Gateway IP mask
    node_ms.fromString(meshcom_settings.node_ownms);
    // Set your DNS IP
    if(strlen(meshcom_settings.node_owndns) >= 7)
      snprintf(meshcom_settings.node_dns, sizeof(meshcom_settings.node_dns), "%s", meshcom_settings.node_owndns);
    else
      snprintf(meshcom_settings.node_dns, sizeof(meshcom_settings.node_dns), "%s", (char*)"8.8.8.8");

    node_dns.fromString(meshcom_settings.node_dns);

    // Set your NTP IP
    if(strlen(meshcom_settings.node_ownntp) >= 7)
      snprintf(meshcom_settings.node_ntp, sizeof(meshcom_settings.node_ntp), "%s", meshcom_settings.node_ownntp);
    else
      snprintf(meshcom_settings.node_ntp, sizeof(meshcom_settings.node_ntp), "%s", (char*)"");

    // Configures static IP address
    if (!WiFi.config(node_ip, node_gw, node_ms, node_dns))
    {
      printlndeb("[Error] STA Failed to configure");
    }
  }
  else
  {
    node_ip = WiFi.localIP();

    snprintf(meshcom_settings.node_ip, sizeof(meshcom_settings.node_ip), "%i.%i.%i.%i", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);
    snprintf(meshcom_settings.node_gw, sizeof(meshcom_settings.node_gw), "%i.%i.%i.%i", WiFi.gatewayIP()[0], WiFi.gatewayIP()[1], WiFi.gatewayIP()[2], WiFi.gatewayIP()[3]);
    snprintf(meshcom_settings.node_dns, sizeof(meshcom_settings.node_dns), "%i.%i.%i.%i", WiFi.dnsIP()[0], WiFi.dnsIP()[1], WiFi.dnsIP()[2], WiFi.dnsIP()[3]);
    snprintf(meshcom_settings.node_subnet, sizeof(meshcom_settings.node_subnet), "%i.%i.%i.%i", WiFi.subnetMask()[0], WiFi.subnetMask()[1], WiFi.subnetMask()[2], WiFi.subnetMask()[3]);

    if(bDEBUG)
    {
      printf("[localIP] %i.%i.%i.%i", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);
    }
  }

  // update phone status
  if(isPhoneReady == 1)
  {
    commandAction((char*)"--wifiset", true);
  }

  s_node_ip = node_ip.toString();

  // no gateway activity
  if(!bWIFIAP)
  {
    Udp.begin(LOCAL_PORT);

    if(strcmp(s_node_ip.c_str(), "0.0.0.0") == 0)
    {
      hasIPaddress=false;
      printfdeb("[WIFI]..not connected for UDP port %d\n",  LOCAL_PORT);
    }
    else
    {
      hasIPaddress=true;
      ifalseping=5;

      printfdeb("[WIFI]...now listening at IP %s, UDP port %d\n",  s_node_ip.c_str(), LOCAL_PORT);
    }

    meshcom_settings.node_hasIPaddress = hasIPaddress;

    if(hasIPaddress)
    {
      ifalseping = 5;

      // F6: Zielnamen bestimmen; aufgeloest wird asynchron (wifiDnsPoll),
      // Literal-IPs sofort, ein Name pro Boot nur einmal.
      const char *srv_host = NULL;
      const char *srv_path = NULL;   // TM-39: "hamnet" or "inet", matches the printlndeb text below
      const char *ntp_host = NULL;
      IPAddress ntp_literal(0,0,0,0);

      if (node_ip[0] == 44 || meshcom_settings.node_hamnet_only == 1)
      {
        // Hamnet only not available for IT-Server, so we use Internet for IT-Server
        if(memcmp(meshcom_settings.node_gwsrv, "IT", 2) == 0)
        {
          printlndeb("[WIFI]...Internet (no HAMNET) UDP-DEST meshcom.dig-italia.it");
          srv_host = "meshcom.dig-italia.it";
          srv_path = "inet";
        }
        else
        if(memcmp(meshcom_settings.node_gwsrv, "DL", 2) == 0)
        {
          printlndeb("[WIFI]...Hamnet UDP-DEST meshcom.hamnet.cloud");
          srv_host = "meshcom.hamnet.cloud";
          srv_path = "hamnet";
        }
        else
        {
          printlndeb("[WIFI]...Hamnet UDP-DEST 44.143.8.143");
          srv_host = "44.143.8.143";
          srv_path = "hamnet";
        }

        if(strlen(meshcom_settings.node_ntp) >= 7)
        {
          printfdeb("[WIFI]...Internet NTP-DEST %s\n", meshcom_settings.node_ntp);
          ntp_host = meshcom_settings.node_ntp;
        }
        else if(node_ip[1] == 143)
        {
          printlndeb("[WIFI]...Hamnet NTP-DEST 44.143.0.9");
          ntp_literal = IPAddress(44, 143, 0, 9);
        }
        else
        {
          printlndeb("[WIFI]...Hamnet NTP-DEST 44.148.224.123");
          ntp_literal = IPAddress(44, 148, 224, 123);
        }
      }
      else
      {
        if(memcmp(meshcom_settings.node_gwsrv, "IT", 2) == 0)
        {
          printlndeb("[WIFI]...Internet UDP-DEST meshcom.dig-italia.it");
          srv_host = "meshcom.dig-italia.it";
          srv_path = "inet";
        }
        else
        {
          printlndeb("[WIFI]...Internet UDP-DEST meshcom.oevsv.at");
          srv_host = "meshcom.oevsv.at";
          srv_path = "inet";
        }

        if(strlen(meshcom_settings.node_ntp) >= 7)
        {
          printfdeb("[WIFI]...Internet NTP-DEST %s\n", meshcom_settings.node_ntp);
          ntp_host = meshcom_settings.node_ntp;
        }
        else
        {
          printlndeb("[WIFI]...Internet NTP-DEST pool.ntp.org");
          ntp_host = "pool.ntp.org";
        }
      }

      if(ntp_host == NULL)
        timeClient.setPoolServerIP(ntp_literal);

      // TM-39: raw & unconditional, once per (re)connect -- before this the
      // host is not chosen yet, so there is nothing to log.
      Serial.printf("[GW];srv;%.2s;host;%s;path;%s;ms;%lu\n",
                     meshcom_settings.node_gwsrv, srv_host, srv_path, (unsigned long)millis());

      wifiDnsStart(&s_dnsSrv, srv_host);
      if(ntp_host != NULL)
        wifiDnsStart(&s_dnsNtp, ntp_host);

      // Ergebnis anwenden (Literal/Cache sofort), sonst spaeter aus der Hauptschleife
      wifiDnsPoll();
    }
  }
  else
  {
    printdeb("[WIFIAP]...node_ip ");
    printlndeb(node_ip);
  
    hasIPaddress=true;
    meshcom_settings.node_hasIPaddress = hasIPaddress;
    ifalseping=5;
  }


}

void sendMeshComHeartbeat()
{
    // F7: der fruehere startNetwork()-Zweig fuer !hasIPaddress war tot (jeder
    // Aufrufer ist auf hasIPaddress gegated); Wiederverbinden macht F3/F4.
    if(!hasIPaddress)
      return;

    if((uint32_t)node_hostip == 0)
    {
      // DNS noch offen oder fehlgeschlagen: erneut anstossen, KEEP erst mit Adresse
      if(!s_dnsSrv.pending && s_dnsSrv.name[0])
        wifiDnsStart(&s_dnsSrv, s_dnsSrv.name);
      return;
    }

    if(bGATEWAY)
    {
      sendKEEP();
    }
}

void resetMeshComUDP()
{
  Udp.stop();

  WiFi.disconnect(true, true);

  hasIPaddress = false;
  meshcom_settings.node_hasIPaddress = hasIPaddress;
  iWlanWait = 0;
  web_timer = 0;
  wifiRequestRestart();

  printlndeb("[WIFI-DBG] resetMeshComUDP: WiFi disconnected, flags reset for reconnect");

  sendDisplayHead(false);
}

#endif

///////////////////////////////////////////////////////////////////////////////
// ESP32 & RAK

void addUdpOutBuffer(uint8_t* buffer, uint16_t len)
{
    if (len > UDP_TX_BUF_SIZE)
        len = UDP_TX_BUF_SIZE; // just for safety

    // CONC-16: addUdpOutBuffer() is reachable from OnRxDone via addNodeData()
    // (lora_functions.cpp) — the FreeRTOS timer-service task on nRF52,
    // priority 2, see C-01 — while sendMeshComUDP() drains the same ring
    // from the Main Loop task. Der Byte-Ring sperrt sich selbst.
    // WF-01: genau len Bytes, kein Byte hinter der Nutzlast des Aufrufers.
    int lost = bf_push(&udpOutRing, buffer, (uint8_t)len);

    // DR-21 (Sichtbarkeitsklausel, 2026-09-12 entschieden): der Ausgangsring
    // war der EINZIGE, dessen Ueberlauf stumm blieb. Der Byte-Ring meldet,
    // wie viele UNGELESENE Frames die Verdraengung getroffen hat.
    if(lost > 0 && bLORADEBUG)
        printfdeb("[MC-DBG] RING_OVERFLOW buf=udp lost=%i\n", lost);
}

void sendKEEP()
{
  int hb_buffer_size=0;
  uint8_t hb_buffer[UDP_TX_BUF_SIZE+50];

  grc_ids = "";
  
  for(int igrc=0; igrc<6; igrc++)
  {
      if(meshcom_settings.node_gcb[igrc] > 0)
      {
          grc_ids.concat(meshcom_settings.node_gcb[igrc]);
          grc_ids.concat(";");
      }
  }

  char keep_buffer[60];
  memset(keep_buffer, 0x00, sizeof(keep_buffer));

  // KEEPFFFFFFFFOE1KBC-124.34w20;232;262;0x00
  snprintf(keep_buffer, sizeof(keep_buffer), "KEEP%08X%-9.9s%-4.4s%-1.1s%s", _GW_ID, meshcom_settings.node_call,SOURCE_VERSION, SOURCE_VERSION_SUB, grc_ids.c_str());

  hb_buffer_size = strlen(keep_buffer)+1;
  memcpy(hb_buffer, keep_buffer, hb_buffer_size);
  
  // if sending fails via UDP.endpacket() for a maximum of counts reset UDP stack
  //also avoid UDP tx when UDP is getting a packet
  // add HB message to the ringbuffer
  if(bLORADEBUG)
  {
    printfdeb("[KEEP]...%s\n", keep_buffer);
  }

  addUdpOutBuffer(hb_buffer, hb_buffer_size);

  // TM-39: raw & unconditional. "ok" reflects the ring enqueue only --
  // addUdpOutBuffer() has no failure path -- not confirmed on-wire delivery;
  // that is tracked separately by --udpstat's tx/tx_fail counters.
  Serial.printf("[GW];keep;tx;ok;1;ms;%lu\n", (unsigned long)millis());
}

/**@brief Function to write our additional data into the UDP tx buffer
 * we add now Longname (up to 20), ID - 4, RSSI - 2, SNR - 1 and MODE BYTE - 1
 * MODE BYTE: LongSlow = 1, MediumSlow = 3
 * 8 byte offset = ID+RSSI+SNR
 */
void addNodeData(uint8_t msg_buffer[UDP_TX_BUF_SIZE], uint16_t size, int16_t rssi, int8_t snr)
{
  #ifdef ESP32
  
  if(!hasIPaddress)
    return;
    
  #endif

  int dt_buffer_size=0;
  uint8_t dt_buffer[UDP_TX_BUF_SIZE+50];

  char data_buffer[60];
  memset(data_buffer, 0x00, sizeof(data_buffer));

  // DATAFFFFFFFFOE1KBC-124.34w2-123-12303
  // 03...MOD
  snprintf(data_buffer, sizeof(data_buffer), "DATA%08X%-9.9s%-4.4s%-1.1s%4i%4i03", _GW_ID, meshcom_settings.node_call,SOURCE_VERSION, SOURCE_VERSION_SUB,rssi,(int)snr);

  dt_buffer_size = strlen(data_buffer);
  memcpy(dt_buffer, data_buffer, dt_buffer_size);
  memcpy(dt_buffer+dt_buffer_size, msg_buffer, size);
  
  addUdpOutBuffer(dt_buffer, dt_buffer_size+size);

  if(bLORADEBUG)
  {
    printfdeb("[DATA]...%s\n", data_buffer);
  }
}

//
///////////////////////////////////////////////////////////////////////////////
