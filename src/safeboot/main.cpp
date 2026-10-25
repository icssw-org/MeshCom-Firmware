// SPDX-License-Identifier: GPL-3.0-or-later
/*
* Copyright (C) 2023-2024 Mathieu Carbou
*/

#include "ElegantOTA.h"
#include "ota_state.h"
#include "fw_apply.h"
#include <WiFi.h>
#include <esp_wifi.h>

#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>

#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <esp_image_format.h>
#include <esp_rom_crc.h>
#include <miniz.h> // ROM tinfl_decompress (ESP32 and ESP32-S3)

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <Preferences.h>
#include "../configuration_global.h"
#include "../esp32/esp32_flash.h"
#include "../esp32/netif_mtu.h"
#include "safeboot_ver.h"
#include "safeboot_id.h"
#include "safeboot_log.h" // keep last: renames Serial on the S3

// AU-12: capability version marker. The image's esp_app_desc_t is blank, so the app finds the version by
// scanning this partition for the marker literal (safeboot_ver.h). Exactly one literal, kept by `used`.
static const char kSafebootMarker[] __attribute__((used)) = SAFEBOOT_MARKER;
static_assert(sizeof(SAFEBOOT_MARKER) - 1 == SAFEBOOT_MARKER_PREFIX_LEN + SAFEBOOT_MARKER_DIGITS,
              "SAFEBOOT_MARKER must be prefix + zero-padded digits");

#if SAFEBOOT_LOG_TEE
SafebootTee SafebootSerial;
#endif

#define TAG "SafeBoot"

// 0: validate ota_0 via esp_image_verify() (bootloader_support, linked in by
//    esp_ota_ops.c) without touching the boot pointer -- the normal path.
// 1: fallback for a toolchain where esp_image_verify() does not link --
//    validate by attempting esp_ota_set_boot_partition() itself and reading
//    its verdict (see checkAppImageValid()).
#ifndef SAFEBOOT_APP_VALID_VIA_SET_BOOT_PARTITION
#define SAFEBOOT_APP_VALID_VIA_SET_BOOT_PARTITION 0
#endif

const unsigned int port = 80;
AsyncWebServer webServer(port);
String hostname = "MeshCom-OTA";

extern Preferences preferences;
extern s_meshcom_settings meshcom_settings;

// ---------------------------------------------------------------------------
// OTA session state machine (docs/safeboot-ota-contract.md,
// src/safeboot/ota_state.h). Replaces the old scattered flags
// (updateInProgress, fallback_armed_at, last_ota_data_millis,
// reboot_after_cancel) plus ElegantOTAClass's own _updateGeneration /
// _ota_image_valid. Touched from both the async_tcp task (ElegantOTA.cpp's
// handlers) and the Arduino loop task (below) -- every call is wrapped in
// g_ota_mux, held only for the duration of one call (microseconds, no
// allocation inside OtaSession).
// ---------------------------------------------------------------------------
safeboot::OtaSession g_ota;
portMUX_TYPE g_ota_mux = portMUX_INITIALIZER_UNLOCKED;

void startMDNS();
bool setBootPartition_APP();
bool checkAppImageValid();

// Re-checks the ota_0 image (boot, and after every drained Abort action --
// docs/safeboot-ota-contract.md "Single app slot") and feeds the verdict
// into g_ota under the spinlock.
static void refreshAppValid() {
  bool valid = checkAppImageValid();
  portENTER_CRITICAL(&g_ota_mux);
  g_ota.setAppValid(valid);
  portEXIT_CRITICAL(&g_ota_mux);
}

// ---------------------------------------------------------------------------
// WiFi bookkeeping for the non-blocking join / AP-fallback / scan (contract
// decisions section + GET /ota/info). Written from the WiFi event task
// (safebootWifiEventLog) and read/written from loop() -- single-writer per
// field in practice (the event task only ever sets the *_pending/connected
// flags and the disconnect reason), so plain volatile bools are enough here,
// unlike the multi-writer OTA session above.
// ---------------------------------------------------------------------------
struct SafebootWifiState {
  bool sta_configured = false;     // SSID configured and WIFI-AP not forced
  bool ap_only = false;            // "no SSID / --wifiap on" path (mode=ap)
  volatile bool ap_up = false;     // AP interface active (ap_only, or ap_sta after 25s)
  volatile bool got_ip = false;
  volatile bool got_ip_pending = false; // set by the WiFi event, drained in loop()
  volatile int last_disconnect_reason = 0;
  uint32_t join_attempts = 0;
  bool wifi_retried_12s = false;
  unsigned long last_reconnect_attempt_ms = 0;
  unsigned long boot_ms = 0;
};
SafebootWifiState g_safeboot_wifi;

// Async scan cache (GET /ota/scan + /ota/info.scan). ~64 bytes/entry * 20 =
// a bit over 1 kB static, well inside the "modest" budget for a classic
// ESP32 safeboot image (item 5 of the brief).
struct ScanEntry {
  char ssid[33];
  char bssid[18]; // "AA:BB:CC:DD:EE:FF"
  int32_t rssi;
  int32_t channel;
  wifi_auth_mode_t auth;
  bool connected;
};
static const int SCAN_CACHE_MAX = 20;
static const int SCAN_CONSIDER_MAX = 64; // strongest 20 of up to this many results
struct ScanCache {
  ScanEntry entries[SCAN_CACHE_MAX];
  int count = 0;
  bool has_result = false;
  volatile bool in_progress = false;
  unsigned long finished_ms = 0;
};
ScanCache g_safeboot_scan;

// -----------------------------------------------------------------------
// Small helpers: JSON building (snprintf only, no ArduinoJson, no String
// concatenation loops -- item 3/5 of the brief) and the WiFi auth-mode name
// mapping used by both /ota/info's sta.auth and its scan.aps[].auth.
// -----------------------------------------------------------------------
static char g_json_buf[3072]; // shared by /ota/info and /ota/state: both run
                               // to completion on the single AsyncTCP task,
                               // never concurrently, so one static buffer is
                               // safe and saves RAM over one each.

static int appendf(char *buf, size_t bufsize, int offset, const char *fmt, ...) {
  if (offset < 0 || (size_t)offset >= bufsize) {
    return offset;
  }
  va_list args;
  va_start(args, fmt);
  int n = vsnprintf(buf + offset, bufsize - offset, fmt, args);
  va_end(args);
  if (n < 0) {
    return offset;
  }
  return offset + n;
}

// Escapes '"' and '\' for a JSON string body (contract: "escape \" and \\ in
// SSIDs"). Truncates rather than overruns if dst is too small.
static void jsonEscape(char *dst, size_t dst_size, const char *src) {
  size_t di = 0;
  if (dst_size == 0) return;
  for (size_t si = 0; src[si] != '\0' && di + 1 < dst_size; ++si) {
    char c = src[si];
    if ((c == '"' || c == '\\')) {
      if (di + 2 >= dst_size) break; // no room for the escape + the char
      dst[di++] = '\\';
    }
    dst[di++] = c;
  }
  dst[di] = '\0';
}

static void macToStr(const uint8_t mac[6], char *out /* size >= 18 */) {
  snprintf(out, 18, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// Contract: "auth strings follow wifi_auth_mode_t names without the
// WIFI_AUTH_ prefix" -- the enum values below match the ESP-IDF
// wifi_auth_mode_t exactly.
static const char *mapAuthName(wifi_auth_mode_t mode) {
  switch (mode) {
    case WIFI_AUTH_OPEN: return "OPEN";
    case WIFI_AUTH_WEP: return "WEP";
    case WIFI_AUTH_WPA_PSK: return "WPA_PSK";
    case WIFI_AUTH_WPA2_PSK: return "WPA2_PSK";
    case WIFI_AUTH_WPA_WPA2_PSK: return "WPA_WPA2_PSK";
    case WIFI_AUTH_WPA2_ENTERPRISE: return "WPA2_ENTERPRISE";
    case WIFI_AUTH_WPA3_PSK: return "WPA3_PSK";
    case WIFI_AUTH_WPA2_WPA3_PSK: return "WPA2_WPA3_PSK";
    case WIFI_AUTH_WAPI_PSK: return "WAPI_PSK";
    case WIFI_AUTH_OWE: return "OWE";
    case WIFI_AUTH_WPA3_ENT_192: return "WPA3_ENT_192";
    default: return "UNKNOWN";
  }
}

static void startScanAsync() {
  if (g_safeboot_scan.in_progress) {
    return;
  }
  g_safeboot_scan.in_progress = true;
  WiFi.scanNetworks(true /*async*/, false /*show hidden*/, false /*passive*/, 300 /*ms per channel*/);
}

// WiFi.onEvent() callback -- mirrors src/udp_functions.cpp wifiEventLog():
// only flags + a log line, runs on the WiFi event task, not loop().
static void safebootWifiEventLog(WiFiEvent_t event, WiFiEventInfo_t info) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      g_safeboot_wifi.last_disconnect_reason = (int)info.wifi_sta_disconnected.reason;
      g_safeboot_wifi.got_ip = false;
      Serial.printf("[SAFEBOOT];wifi;event;disconnected;reason;%d\n", g_safeboot_wifi.last_disconnect_reason);
      break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.println("[SAFEBOOT];wifi;event;connected;reason;0");
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      g_safeboot_wifi.got_ip = true;
      g_safeboot_wifi.got_ip_pending = true;
      netif_mtu::applyConfiguredMtu(meshcom_settings.node_ethmtu); // NMTU-01
      Serial.println("[SAFEBOOT];wifi;event;got_ip;reason;0");
      break;
    case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
      // NMTU-01: AP netif is certainly up once a client associated (AP_START is too early)
      netif_mtu::applyConfiguredMtu(meshcom_settings.node_ethmtu);
      break;
    default:
      break;
  }
}

void wifiConnect() {

  // read wlan credentials from flash
  init_flash();

  const char *ssid = meshcom_settings.node_ssid;
  const char *pass = meshcom_settings.node_pwd;
  bool bWEBSERVER = meshcom_settings.node_sset2 & 0x0040;
  bool bGATEWAY = meshcom_settings.node_sset & 0x1000;
  bool bWIFIAP = meshcom_settings.node_sset2 & 0x0080;

  Serial.printf("\nNVS Flash Settings:\n");
  Serial.printf("Callsign: %s\n", meshcom_settings.node_call);
  Serial.printf("Wifi SSID: %s\n", ssid);
  Serial.printf("Webserver: %d\n", bWEBSERVER);
  Serial.printf("Gateway: %d\n", bGATEWAY);
  Serial.printf("WIFI AP: %d\n", bWIFIAP);



  // Set the hostname from the callsign. If the callsign is not set, use the default hostname
  if (!isNodeUnconfigured(meshcom_settings.node_call))
  {
    hostname = meshcom_settings.node_call;
  }

  WiFi.onEvent(safebootWifiEventLog);

  // When there is no SSID or WIFI-AP is enabled, start AP (unchanged: "ap"
  // mode is not part of the non-blocking join/fallback logic below).
  if (strcmp(ssid, "none") == 0 || bWIFIAP)
  {
    g_safeboot_wifi.ap_only = true;
    g_safeboot_wifi.sta_configured = false;
    Serial.println("\nStarting Wifi AP");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(hostname);
    delay(300);
    g_safeboot_wifi.ap_up = true;
    Serial.printf("AP IP: %s\n", WiFi.softAPIP().toString().c_str());
    // start mDNS responder
    startMDNS();
    return;
  }

  g_safeboot_wifi.sta_configured = true;
  g_safeboot_wifi.boot_ms = millis();

   // TM-48: driver-managed join, same as production (udp_functions.cpp wifiInitOnce/wifiBegin)
   WiFi.persistent(false);
   WiFi.setAutoReconnect(true);
   WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);
   WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL);

   // Use the same DHCP hostname as the firmware/mDNS so the node shows up
   // under one name in OTA mode too.
   char host[32];
   if(makeDhcpHostname(host, sizeof(host), meshcom_settings.node_call))
     WiFi.setHostname(host);

   WiFi.mode(WIFI_STA);
   WiFi.disconnect(true);

   // Static IP settings
  IPAddress node_ip = IPAddress(0,0,0,0);
  IPAddress node_gw = IPAddress(0,0,0,0);
  IPAddress node_ms = IPAddress(0,0,0,0);
  IPAddress node_dns = IPAddress(0,0,0,0);

  if (strlen(meshcom_settings.node_ownip) >= 7 && strlen(meshcom_settings.node_owngw) >= 7 && strlen(meshcom_settings.node_ownms) >= 7 && bWIFIAP == false)
  {
    Serial.printf("Static IP settings:\n");
    Serial.printf("IP: %s\n", meshcom_settings.node_ownip);
    Serial.printf("GW: %s\n", meshcom_settings.node_owngw);
    Serial.printf("MS: %s\n", meshcom_settings.node_ownms);
    Serial.printf("DNS: %s\n", meshcom_settings.node_owndns);

    // Set your Static IP address
    node_ip.fromString(meshcom_settings.node_ownip);
    // Set your Gateway IP address
    node_gw.fromString(meshcom_settings.node_owngw);
    // Set your Gateway IP mask
    node_ms.fromString(meshcom_settings.node_ownms);
    // Set your DNS IP
    if (strlen(meshcom_settings.node_owndns) >= 7)
      node_dns.fromString(meshcom_settings.node_owndns);
    else
      node_dns.fromString("8.8.8.8");

    // Configures static IP address
    if (!WiFi.config(node_ip, node_gw, node_ms, node_dns))
    {
      Serial.println("[Error] STA Failed to configure static IP!");
    }
  }

   delay(500);

   // TM-48: config-only begin, let the driver pick the AP -- no own scan,
   // no BSSID/channel pin (see udp_functions.cpp wifiBegin() for the
   // production pattern this mirrors).
   // "empty" (production convention) and "none" (safeboot/flash-default
   // convention) both mean "open network".
   const char *wifi_pwd = pass;
   if (strcmp(wifi_pwd, "empty") == 0 || strcmp(wifi_pwd, "none") == 0)
     wifi_pwd = NULL;

   Serial.printf("-> try connecting to SSID: %s \n", ssid);
   WiFi.begin(ssid, wifi_pwd, 0, NULL, false); // configuration only, no connect yet

   {
     esp_err_t rc = esp_wifi_disable_pmf_config(WIFI_IF_STA);
     Serial.printf("[SAFEBOOT];wifi;pmf_off;rc;%d\n", (int)rc);
   }

   esp_wifi_connect();
   g_safeboot_wifi.join_attempts = 1;

   Serial.println("Connecting to WiFi (non-blocking)");
   // No blocking wait here: the web server and mDNS start right after this
   // returns (setup(), below); loop() drives the 12 s retry, the 25 s
   // ap_sta fallback, the periodic reconnect while that AP is up, and
   // starts mDNS once GOT_IP arrives (see the contract's WLAN decision).
 }



 // start mdns responder and set hostname and tcp service
 void startMDNS()
 {
   if (!MDNS.begin(hostname.c_str()))
   {
     Serial.println("Error setting up MDNS responder!");
   }
   Serial.println("mDNS responder started");
   if(MDNS.addService("http", "tcp", port))
   {
     Serial.println("mDNS http service added");
   }
   else
   {
     Serial.println("Error setting up mDNS service!");
   }
 }


 // Set partition to ota_0. Returns whether the switch actually succeeded --
 // on the single app slot (docs/safeboot-ota-contract.md "Single app slot")
 // this fails (ESP_ERR_OTA_VALIDATE_FAILED) when ota_0 holds a half-written
 // image, e.g. right after a kill-mid-upload abort; the caller must not
 // reboot in that case, or it loops straight back into safeboot.
 bool setBootPartition_APP()
 {
   const esp_partition_t *partition = esp_partition_find_first(esp_partition_type_t::ESP_PARTITION_TYPE_APP, esp_partition_subtype_t::ESP_PARTITION_SUBTYPE_APP_OTA_0, nullptr);
   if (!partition)
   {
     Serial.println("Error setting boot partition!");
     Serial.println("[SAFEBOOT];fallback;result;app_invalid;rc;-1");
     return false;
   }
   esp_err_t rc = esp_ota_set_boot_partition(partition);
   bool ok = (rc == ESP_OK);
   Serial.printf("[SAFEBOOT];fallback;result;%s;rc;%d\n", ok ? "ok" : "app_invalid", (int)rc);
   return ok;
 }

 // Validates the ota_0 image without switching the boot pointer (bench
 // finding, docs/safeboot-ota-contract.md "Single app slot"): called once at
 // boot and again after every Abort action this session drains, since an
 // abort that landed after chunks were written is exactly the case that
 // invalidates the single app slot. esp_image_verify() comes from
 // bootloader_support, which esp_ota_ops.c already links into every
 // arduino-esp32 app -- if that ever stops being true in this toolchain,
 // the fallback below (attempting the real esp_ota_set_boot_partition() and
 // reading its verdict, undoing it immediately) is exactly the check the
 // fallback-to-app path would perform anyway, just run early instead of at
 // reboot time.
 bool checkAppImageValid()
 {
   const esp_partition_t *partition = esp_partition_find_first(esp_partition_type_t::ESP_PARTITION_TYPE_APP, esp_partition_subtype_t::ESP_PARTITION_SUBTYPE_APP_OTA_0, nullptr);
   if (!partition)
   {
     Serial.println("[SAFEBOOT];app;image;invalid;rc;-1");
     return false;
   }

#if SAFEBOOT_APP_VALID_VIA_SET_BOOT_PARTITION
   // Fallback path: esp_image_verify() did not link. The boot-partition
   // switch validates the image as its own side effect (ESP_ERR_OTA_
   // VALIDATE_FAILED on a half-written image), so use that verdict directly
   // -- this is exactly what the fallback-to-app reboot would do anyway,
   // just performed here instead of at reboot time so the state machine can
   // stay quiet in the meantime.
   esp_err_t rc = esp_ota_set_boot_partition(partition);
   bool valid = (rc == ESP_OK);
#else
   esp_partition_pos_t pos;
   pos.offset = partition->address;
   pos.size = partition->size;
   esp_image_metadata_t data;
   esp_err_t rc = esp_image_verify(ESP_IMAGE_VERIFY_SILENT, &pos, &data);
   bool valid = (rc == ESP_OK);
#endif
   Serial.printf("[SAFEBOOT];app;image;%s;rc;%d\n", valid ? "valid" : "invalid", (int)rc);
   return valid;
 }


 // ElegantOTA Callbacks -- print points only now; onStart/onChunk/onFinalReceived/
 // onVerified/onWriteFailed/onDisconnect (driven from ElegantOTA.cpp) own the
 // actual g_ota bookkeeping (docs/safeboot-ota-contract.md).
 unsigned long ota_progress_millis = 0;

 void onOTAStart() {
   Serial.println("OTA update started!");
   Serial.println("[SAFEBOOT];ota;start");
 }

 void onOTAProgress(size_t current, size_t final) {
   // Log every 1 second
   if (millis() - ota_progress_millis > 1000) {
     ota_progress_millis = millis();
     Serial.printf("OTA Progress Current: %u bytes, Final: %u bytes\n", current, final);
   }
 }

 // Fired by ElegantOTA whenever it actually aborts a running Update session
 // (drained from a g_ota Abort action, see loop() below, or from
 // ElegantOTA.cpp's /ota/start stale-session drain).
 void onOTAAbort(const char *reason)
 {
   Serial.printf("[SAFEBOOT];ota;rearm;reason;%s\n", reason);
 }

 void onOTAEnd(bool success)
 {
   // The boot-partition switch is no longer done here: it happens only when
   // g_ota emits a SwitchPartition action (only ever queued from
   // onVerified(ok=true), see ota_state.h), drained in loop() below.
   if (success)
   {
     Serial.println("OTA update finished successfully!");
     Serial.println("[SAFEBOOT];ota;end;result;success");
   }
   else
   {
     Serial.println("There was an error during OTA update!");
     Serial.println("[SAFEBOOT];ota;end;result;error");
   }
 }


 // ---------------------------------------------------------------------
 // AU-07 (#1187): offline apply of a staged update.
 //
 // The app stores a compressed (.autoupdate, zlib) release at the END of ota_0
 // and an FWS2 record in NVS "fwstage"/"rec" (src/fw_update.h). Here, before
 // WiFi and the web server exist:
 //   CRC32 of the staged bytes == rec.crc32   (else: drop the record)
 //   erase ota_0 [0, roundup4K(ilen))         (never reaches the staged tail)
 //   inflate with the ROM tinfl into ota_0    (32 KB circular dictionary)
 //   esp_image_verify(ota_0), boot pointer -> ota_0, restart
 // A power loss after the erase leaves the record and the tail intact, so
 // the next boot retries; "fwstage"/"tries" bounds that to FW_APPLY_MAX_TRIES.
 // A deterministic failure (CRC, inflate, verify) drops the record and falls
 // through to the normal Safeboot AP/OTA mode (ota_0 invalid -> app_valid
 // false). No SHA-256 here by design: CRC + adler32 + esp_image_verify.
 // ---------------------------------------------------------------------
 #define FWSTAGE_NS "fwstage"
 #define FWSTAGE_KEY_REC "rec"
 #define FWSTAGE_KEY_TRIES "tries"
 #define FWSTAGE_KEY_LAST "last"
 #define FWAPPLY_IN_BUF 4096u

 static bool g_apply_failed = false; // replayed into g_ota after g_ota.begin()

 struct FwFlashRd {
   const esp_partition_t *part;
   uint32_t base;
   bool read(uint32_t pos, uint8_t *dst, size_t n) { return esp_partition_read(part, base + pos, dst, n) == ESP_OK; }
 };

 struct FwFlashWr {
   const esp_partition_t *part;
   bool write(uint32_t pos, const uint8_t *src, size_t n) {
     if (esp_partition_write(part, pos, src, n) != ESP_OK) return false;
     g_ota.onApplyProgress(millis(), pos + (uint32_t)n); // no web server yet: no lock needed
     delay(1); // let the idle task run between writes
     return true;
   }
 };

 // One tinfl_decompress() call: zlib header + adler32 are parsed and checked
 // by tinfl; the output goes to the circular dictionary (no NON_WRAPPING flag).
 struct FwTinflDec {
   tinfl_decompressor d;
   int step(const uint8_t *in, size_t *inSize, uint8_t *dictStart, uint8_t *outNext, size_t *outSize, bool more) {
     mz_uint32 flags = TINFL_FLAG_PARSE_ZLIB_HEADER | (more ? TINFL_FLAG_HAS_MORE_INPUT : 0);
     return (int)tinfl_decompress(&d, in, inSize, dictStart, outNext, outSize, flags);
   }
 };

 // Drops the record and the attempt counter. "last" (the app's reinstall
 // guard) stays, except after a deterministic apply failure (dropLast): then
 // the app may fetch that tag once more, so one flash bit error does not cost
 // the update until the next release.
 static void fwStageDrop(bool dropLast) {
   Preferences p;
   if (p.begin(FWSTAGE_NS, false)) {
     p.remove(FWSTAGE_KEY_REC);
     p.remove(FWSTAGE_KEY_TRIES);
     if (dropLast) p.remove(FWSTAGE_KEY_LAST);
     p.end();
   }
 }

 // nullptr = success. *keep tells the caller whether the record survives the
 // failure; *erased whether ota_0 was already touched (the app is then invalid).
 static const char *fwApplyRun(const esp_partition_t *ota0, const FwApplyPlan &plan, const FwStageRecord &rec, bool *keep, bool *erased, uint32_t *written) {
   *keep = true; // until a deterministic verdict
   *erased = false;
   *written = 0;
   const size_t memLen = sizeof(FwTinflDec) + 8 + TINFL_LZ_DICT_SIZE + FWAPPLY_IN_BUF;
   uint8_t *mem = (uint8_t *)malloc(memLen);
   if (!mem) return "nomem";
   FwTinflDec *dec = (FwTinflDec *)mem;
   uint8_t *dict = mem + ((sizeof(FwTinflDec) + 7) & ~(size_t)7);
   uint8_t *in = dict + TINFL_LZ_DICT_SIZE;
   const char *err = nullptr;

   FwFlashRd rd = {ota0, plan.srcOff};
   FwFlashWr wr = {ota0};

   // (a) CRC32 (zlib) over the staged bytes, chained from 0
   uint32_t crc = 0;
   for (uint32_t pos = 0; pos < plan.zlen;) {
     uint32_t n = plan.zlen - pos;
     if (n > FWAPPLY_IN_BUF) n = FWAPPLY_IN_BUF;
     if (!rd.read(pos, in, n)) { err = "read"; goto done; }
     crc = esp_rom_crc32_le(crc, in, n);
     pos += n;
     delay(0);
   }
   if (crc != rec.crc32) { *keep = false; err = "crc"; goto done; }

   // (b) erase [0, roundup4K(ilen)) in 64 KB steps; from here the old image is gone
   g_ota.onApplyProgress(millis(), 0);
   *erased = true; // even a failed first erase may have damaged the head
   for (uint32_t o = 0; o < plan.eraseLen;) {
     uint32_t c = plan.eraseLen - o;
     if (c > 0x10000u) c = 0x10000u;
     if (esp_partition_erase_range(ota0, o, c) != ESP_OK) { err = "erase"; goto done; }
     o += c;
     delay(1);
   }

   // (c) stream-inflate tail -> head
   {
     tinfl_init(&dec->d);
     FwInflateResult r = fwInflateStream(*dec, rd, wr, in, FWAPPLY_IN_BUF, dict, TINFL_LZ_DICT_SIZE, plan.zlen, plan.ilen, written);
     if (r != FWI_OK) {
       err = fwInflateReason(r);
       *keep = (r == FWI_READ || r == FWI_WRITE);
       goto done;
     }
   }

   // (d) esp_image_verify on ota_0 (also logs [SAFEBOOT];app;image;...)
   if (!checkAppImageValid()) { *keep = false; err = "verify"; goto done; }

 done:
   free(mem);
   return err;
 }

 static void applyStagedUpdate() {
   uint8_t buf[FW_STAGE_ENC_LEN];
   size_t n = 0;
   uint8_t tries = 0;
   {
     Preferences p;
     if (!p.begin(FWSTAGE_NS, true)) return; // namespace absent: nothing staged
     if (p.isKey(FWSTAGE_KEY_REC) && p.getBytesLength(FWSTAGE_KEY_REC) == FW_STAGE_ENC_LEN) {
       n = p.getBytes(FWSTAGE_KEY_REC, buf, sizeof(buf));
     } else if (p.isKey(FWSTAGE_KEY_REC)) {
       n = 1; // wrong size: falls into the decode failure below
     }
     tries = p.getUChar(FWSTAGE_KEY_TRIES, 0);
     p.end();
   }
   if (n == 0) return;

   const char *why = "ok";
   FwStageRecord rec;
   FwApplyPlan plan;
   const esp_partition_t *ota0 = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, nullptr);
   if (n != FW_STAGE_ENC_LEN || !fwRecordDecode(buf, n, rec)) {
     why = "record";
   } else if (!ota0) {
     why = "no_ota0";
   } else if (!fwApplyPlan(rec, ota0->size, plan, &why)) {
     // why set by the plan
   } else if (tries >= FW_APPLY_MAX_TRIES) {
     why = "tries";
   } else {
     why = nullptr;
   }
   if (why) {
     Serial.printf("[SAFEBOOT];apply;fail;%s\n", why);
     g_apply_failed = true;
     fwStageDrop(false); // plan / record problems: keep "last", no re-download loop
     return;
   }

   { // count the attempt before touching flash: a hang or power loss still counts
     Preferences p;
     if (p.begin(FWSTAGE_NS, false)) {
       p.putUChar(FWSTAGE_KEY_TRIES, (uint8_t)(tries + 1));
       p.end();
     }
   }

   unsigned long t0 = millis();
   Serial.printf("[SAFEBOOT];apply;start;tag;%s;zlen;%lu;ilen;%lu;try;%u\n", rec.tag, (unsigned long)plan.zlen, (unsigned long)plan.ilen, (unsigned)(tries + 1));
   g_ota.onApplyBegin(t0, plan.ilen);

   bool keep = false, erased = false;
   uint32_t written = 0;
   const char *err = fwApplyRun(ota0, plan, rec, &keep, &erased, &written);
   if (err) {
     Serial.printf("[SAFEBOOT];apply;fail;%s\n", err);
     g_ota.onApplyEnd(millis(), false);
     g_apply_failed = true;
     if (!keep) {
       fwStageDrop(true); // deterministic: the app may fetch this tag once more
     } else if (erased) {
       // The head is gone, the record and the tail are intact: retry on the
       // next boot without waiting for a power cycle (bounded by "tries").
       Serial.printf("[SAFEBOOT];apply;retry;restart;try;%u\n", (unsigned)(tries + 1));
       Serial.flush();
       delay(1000);
       ESP.restart();
     }
     // keep && !erased: ota_0 is untouched, continue as normal Safeboot; the record stays
     return;
   }

   // (e) boot pointer -> ota_0 (re-validates the image), record gone, restart
   if (esp_ota_set_boot_partition(ota0) != ESP_OK) {
     Serial.println("[SAFEBOOT];apply;fail;boot");
     g_ota.onApplyEnd(millis(), false);
     g_apply_failed = true;
     fwStageDrop(true);
     return;
   }
   fwStageDrop(false); // success: "last" stays
   g_ota.onApplyEnd(millis(), true);
   Serial.printf("[SAFEBOOT];apply;ok;bytes;%lu;ms;%lu\n", (unsigned long)written, (unsigned long)(millis() - t0));
   Serial.flush();
   delay(200);
   ESP.restart();
 }


// F4-2a: name the app image in ota_0 (marker at image offset 0x120). Printed once at boot and
// answered on request by the serial reader in loop() ("--id" / "--info").
static SbIdentity g_app_id;
static SbLineReader g_line_reader;

static void readAppIdentity() {
   const esp_partition_t *p = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);
   uint8_t head[SB_ID_HEAD_LEN];
   if (!p || esp_partition_read(p, 0, head, sizeof(head)) != ESP_OK)
      sbIdFail(&g_app_id, SB_ID_NO_PARTITION);
   else
      sbIdParse(head, sizeof(head), &g_app_id);
   char out[96];
   if (g_app_id.reason == SB_ID_OK) {
      sbIdLine(out, sizeof(out), g_app_id);
      Serial.println(out);
      Serial.printf("[SAFEBOOT];id;%s\n", g_app_id.env);
   } else
      Serial.printf("[SAFEBOOT];id;none;%s\n", sbIdReasonName(g_app_id.reason));
}

// Non-blocking: drains whatever the UART/USB holds, answers a complete "--id" / "--info" line.
static void serialCommands() {
   int guard = 64;
   while (guard-- > 0 && Serial.available() > 0) {
      int c = Serial.read();
      if (c < 0)
         break;
      SbCmd cmd = g_line_reader.feed((char)c);
      if (cmd != SB_CMD_NONE) {
         char out[256];
         sbRespond(cmd, g_app_id, meshcom_settings.node_call, out, sizeof(out));
         Serial.print(out);
      }
   }
}

 void setup() {

   // AU-07: tell the app this Safeboot understands staged records. The app
   // writes "hand" (and may set "nocap") before every handover to detect an
   // OLD Safeboot; clearing them here, unconditionally, ends that verdict.
   {
     Preferences p;
     if (p.begin(FWSTAGE_NS, false)) {
       p.remove("hand");
       p.remove("nocap");
       p.end();
     }
   }

   Serial.begin(115200);
   // whait for serial
   delay(1000);
   Serial.println("\n-----------------------------");
   Serial.println("OTA UDATE started");
   Serial.printf("[SAFEBOOT];ver;%d\n", SAFEBOOT_VERSION);
   readAppIdentity();

   // AU-07: apply a staged update (offline, before any network). Returns
   // only if there is nothing staged or the apply failed; on success it
   // restarts into the new app.
   applyStagedUpdate();

   // Connect to saved ssid or as fallback spawn an AP. Non-blocking for the
   // STA-join case: returns as soon as esp_wifi_connect() has been kicked
   // off, loop() drives the rest (contract WLAN decision).
   wifiConnect();

   // Start ElegantOTA
   ElegantOTA.clearAuth();
   ElegantOTA.setAutoReboot(true);
   ElegantOTA.begin(&webServer);
   // ElegantOTA callbacks
   ElegantOTA.onStart(onOTAStart);
   ElegantOTA.onProgress(onOTAProgress);
   ElegantOTA.onEnd(onOTAEnd);
   ElegantOTA.onAbort(onOTAAbort);

   // Start web server
   webServer.rewrite("/", "/update");
   webServer.onNotFound([](AsyncWebServerRequest* request) {
     request->redirect("/");
   });

   //endpoint for canceling the update. Only works if the update has not started yet
   webServer.on("/ota/cancel", HTTP_GET, [](AsyncWebServerRequest *request) {
     bool app_valid;
     portENTER_CRITICAL(&g_ota_mux);
     app_valid = g_ota.state().app_valid;
     portEXIT_CRITICAL(&g_ota_mux);
     if (!app_valid)
     {
       // Single app slot (docs/safeboot-ota-contract.md): a cancel would
       // just reboot into the same half-written image. Checked before the
       // in-progress check below.
       request->send(409, "text/plain", "app_invalid");
       return;
     }

     bool accepted;
     portENTER_CRITICAL(&g_ota_mux);
     accepted = g_ota.onCancelRequest(millis());
     portEXIT_CRITICAL(&g_ota_mux);
     if (!accepted)
     {
       request->send(400, "text/plain", "OTA update in progress. Cannot cancel.");
     }
     else
     {
       request->send(200, "text/plain", "OTA update canceled.");
     }
   });

   // GET /ota/info -- contract: docs/safeboot-ota-contract.md
   webServer.on("/ota/info", HTTP_GET, [](AsyncWebServerRequest *request) {
     char call_esc[80];
     jsonEscape(call_esc, sizeof(call_esc), hostname.c_str());

     bool sta_conn = g_safeboot_wifi.sta_configured && g_safeboot_wifi.got_ip && (WiFi.status() == WL_CONNECTED);

     char ssid_esc[80] = "";
     if (g_safeboot_wifi.sta_configured) {
       jsonEscape(ssid_esc, sizeof(ssid_esc), meshcom_settings.node_ssid);
     }

     char bssid_str[18] = "";
     char ip_str[40] = "";
     int32_t rssi = 0;
     int32_t channel = 0;
     const char *auth_name = "";
     if (sta_conn) {
       wifi_ap_record_t apinfo;
       if (esp_wifi_sta_get_ap_info(&apinfo) == ESP_OK) {
         macToStr(apinfo.bssid, bssid_str);
         rssi = apinfo.rssi;
         channel = apinfo.primary;
         auth_name = mapAuthName(apinfo.authmode);
       }
       strncpy(ip_str, WiFi.localIP().toString().c_str(), sizeof(ip_str) - 1);
     }

     const char *mode_str = g_safeboot_wifi.ap_only ? "ap" : (g_safeboot_wifi.ap_up ? "ap_sta" : "sta");
     // Default softAP IP even when the AP is not (yet) up -- informational,
     // matches what WiFi.softAP() would assign.
     String ap_ip = g_safeboot_wifi.ap_up ? WiFi.softAPIP().toString() : String("192.168.4.1");

     int off = 0;
     off = appendf(g_json_buf, sizeof(g_json_buf), off,
       "{\"call\":\"%s\",\"hostname\":\"%s\",\"mdns\":\"%s.local\",\"mode\":\"%s\",\"uptime_ms\":%lu,",
       call_esc, call_esc, call_esc, mode_str, (unsigned long)millis());

     off = appendf(g_json_buf, sizeof(g_json_buf), off,
       "\"sta\":{\"configured\":%s,\"ssid\":\"%s\",\"connected\":%s,\"bssid\":\"%s\",\"ip\":\"%s\","
       "\"rssi\":%ld,\"channel\":%ld,\"auth\":\"%s\",\"last_disconnect_reason\":%d,\"join_attempts\":%lu},",
       g_safeboot_wifi.sta_configured ? "true" : "false", ssid_esc, sta_conn ? "true" : "false",
       bssid_str, ip_str, (long)rssi, (long)channel, auth_name,
       g_safeboot_wifi.last_disconnect_reason, (unsigned long)g_safeboot_wifi.join_attempts);

     off = appendf(g_json_buf, sizeof(g_json_buf), off,
       "\"ap\":{\"active\":%s,\"ssid\":\"%s\",\"ip\":\"%s\"},",
       g_safeboot_wifi.ap_up ? "true" : "false", call_esc, ap_ip.c_str());

     uint32_t age_ms = g_safeboot_scan.has_result ? (uint32_t)(millis() - g_safeboot_scan.finished_ms) : 0;
     off = appendf(g_json_buf, sizeof(g_json_buf), off,
       "\"scan\":{\"age_ms\":%lu,\"in_progress\":%s,\"aps\":[",
       (unsigned long)age_ms, g_safeboot_scan.in_progress ? "true" : "false");

     for (int i = 0; i < g_safeboot_scan.count; ++i) {
       char ap_ssid_esc[64];
       jsonEscape(ap_ssid_esc, sizeof(ap_ssid_esc), g_safeboot_scan.entries[i].ssid);
       off = appendf(g_json_buf, sizeof(g_json_buf), off,
         "%s{\"ssid\":\"%s\",\"bssid\":\"%s\",\"rssi\":%ld,\"channel\":%ld,\"auth\":\"%s\",\"connected\":%s}",
         i == 0 ? "" : ",", ap_ssid_esc, g_safeboot_scan.entries[i].bssid,
         (long)g_safeboot_scan.entries[i].rssi, (long)g_safeboot_scan.entries[i].channel,
         mapAuthName(g_safeboot_scan.entries[i].auth), g_safeboot_scan.entries[i].connected ? "true" : "false");
     }
     off = appendf(g_json_buf, sizeof(g_json_buf), off, "]}}");

     AsyncWebServerResponse *response = request->beginResponse(200, "application/json", g_json_buf);
     response->addHeader("Cache-Control", "no-store");
     request->send(response);
   });

   // GET /ota/state -- contract: docs/safeboot-ota-contract.md
   webServer.on("/ota/state", HTTP_GET, [](AsyncWebServerRequest *request) {
     safeboot::OtaSession::Status st;
     portENTER_CRITICAL(&g_ota_mux);
     st = g_ota.state();
     portEXIT_CRITICAL(&g_ota_mux);

     int off = 0;
     off = appendf(g_json_buf, sizeof(g_json_buf), off,
       "{\"state\":\"%s\",\"reason\":\"%s\",\"generation\":%lu,\"received\":%lu,\"total\":%lu,"
       "\"image_valid\":%s,\"app_valid\":%s,\"fallback_in_ms\":%ld,\"uptime_ms\":%lu,"
       "\"heap_free\":%lu,\"heap_block\":%lu}",
       safeboot::OtaSession::stateName(st.state), safeboot::OtaSession::reasonName(st.reason),
       (unsigned long)st.generation, (unsigned long)st.received, (unsigned long)st.total,
       st.image_valid ? "true" : "false", st.app_valid ? "true" : "false",
       (long)st.fallback_in_ms, (unsigned long)millis(),
       (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getMaxAllocHeap());

     AsyncWebServerResponse *response = request->beginResponse(200, "application/json", g_json_buf);
     response->addHeader("Cache-Control", "no-store");
     request->send(response);
   });

   // GET /ota/scan -- contract: docs/safeboot-ota-contract.md
   webServer.on("/ota/scan", HTTP_GET, [](AsyncWebServerRequest *request) {
     bool busy;
     portENTER_CRITICAL(&g_ota_mux);
     safeboot::OtaSession::State st = g_ota.state().state;
     portEXIT_CRITICAL(&g_ota_mux);
     busy = (st == safeboot::OtaSession::State::Receiving || st == safeboot::OtaSession::State::Verifying ||
             st == safeboot::OtaSession::State::Applying);

     if (busy || g_safeboot_scan.in_progress) {
       request->send(409, "text/plain", "busy");
       return;
     }
     if (!g_safeboot_wifi.sta_configured) {
       request->send(409, "text/plain", "no_sta");
       return;
     }
     startScanAsync();
     request->send(200, "text/plain", "OK");
   });

   netif_mtu::applyConfiguredMtu(meshcom_settings.node_ethmtu); // NMTU-01: before the first accept
   webServer.begin();

   portENTER_CRITICAL(&g_ota_mux);
   g_ota.begin(millis());
   if (g_apply_failed) { // begin() reset the state: put the apply verdict back
     g_ota.onApplyBegin(millis(), 0);
     g_ota.onApplyEnd(millis(), false);
   }
   portEXIT_CRITICAL(&g_ota_mux);

   // Boot-time check of the ota_0 image (docs/safeboot-ota-contract.md
   // "Single app slot"): a prior kill-mid-upload can leave the single app
   // slot half-written, in which case the fallback-to-app timer must stay
   // suspended (g_ota.setAppValid(false)) instead of looping every 180 s.
   refreshAppValid();
 }

 void loop() {
   unsigned long now = millis();
   serialCommands();

   // -- Non-blocking STA join follow-up: 12 s retry, 25 s ap_sta fallback,
   // periodic reconnect while that AP is up, mDNS + a scan once GOT_IP
   // arrives (contract WLAN decision). The "ap"-only path (no SSID /
   // --wifiap on) never reaches here (sta_configured stays false). --
   if (g_safeboot_wifi.sta_configured) {
     long since_boot = (long)(now - g_safeboot_wifi.boot_ms);

     if (!g_safeboot_wifi.got_ip && !g_safeboot_wifi.wifi_retried_12s && since_boot >= 12000) {
       g_safeboot_wifi.wifi_retried_12s = true;
       Serial.println("[SAFEBOOT];wifi;retry;reason;no_connect_12s");
       esp_wifi_connect();
       g_safeboot_wifi.join_attempts++;
     }

     if (!g_safeboot_wifi.got_ip && !g_safeboot_wifi.ap_up && since_boot >= 25000) {
       Serial.println("[SAFEBOOT];wifi;ap_sta;reason;join_timeout_25s");
       WiFi.mode(WIFI_AP_STA);
       WiFi.softAP(hostname);
       g_safeboot_wifi.ap_up = true;
       g_safeboot_wifi.last_reconnect_attempt_ms = now;
       Serial.printf("AP IP: %s\n", WiFi.softAPIP().toString().c_str());
     }

     // Keep the STA retrying every 20 s once the ap_sta fallback is open --
     // auto-reconnect plus the one-shot 12 s kick above are enough before
     // that.
     if (g_safeboot_wifi.ap_up && !g_safeboot_wifi.got_ip && (long)(now - g_safeboot_wifi.last_reconnect_attempt_ms) >= 20000) {
       g_safeboot_wifi.last_reconnect_attempt_ms = now;
       esp_wifi_connect();
       g_safeboot_wifi.join_attempts++;
     }

     if (g_safeboot_wifi.got_ip_pending) {
       g_safeboot_wifi.got_ip_pending = false;
       Serial.print("IP Address: ");
       Serial.println(WiFi.localIP());
       startMDNS();
       startScanAsync();
     }
   }

   // -- Async WiFi scan poll (GET /ota/scan + /ota/info.scan) --
   if (g_safeboot_scan.in_progress) {
     int16_t n = WiFi.scanComplete();
     if (n == WIFI_SCAN_FAILED) {
       g_safeboot_scan.in_progress = false;
       WiFi.scanDelete();
     } else if (n >= 0) {
       int16_t consider = (n < SCAN_CONSIDER_MAX) ? n : SCAN_CONSIDER_MAX;
       int16_t idx[SCAN_CONSIDER_MAX];
       for (int16_t i = 0; i < consider; ++i) idx[i] = i;
       // Sort the considered entries by RSSI descending (contract: "scan.aps
       // sorted by RSSI descending"). `consider` is small (<=64); a plain
       // insertion sort is plenty for a one-off, non-hot-path event.
       for (int16_t i = 1; i < consider; ++i) {
         int16_t key = idx[i];
         int32_t key_rssi = WiFi.RSSI(key);
         int16_t j = i - 1;
         while (j >= 0 && WiFi.RSSI(idx[j]) < key_rssi) {
           idx[j + 1] = idx[j];
           --j;
         }
         idx[j + 1] = key;
       }

       bool have_sta_bssid = g_safeboot_wifi.got_ip && (WiFi.status() == WL_CONNECTED);
       uint8_t sta_bssid[6] = {0};
       if (have_sta_bssid) {
         const uint8_t *b = WiFi.BSSID();
         if (b != nullptr) memcpy(sta_bssid, b, 6);
       }

       int copy_n = (consider < SCAN_CACHE_MAX) ? consider : SCAN_CACHE_MAX;
       for (int i = 0; i < copy_n; ++i) {
         int16_t k = idx[i];
         ScanEntry &e = g_safeboot_scan.entries[i];
         strncpy(e.ssid, WiFi.SSID(k).c_str(), sizeof(e.ssid) - 1);
         e.ssid[sizeof(e.ssid) - 1] = '\0';
         const uint8_t *bssid = WiFi.BSSID(k);
         if (bssid != nullptr) {
           macToStr(bssid, e.bssid);
           e.connected = have_sta_bssid && (memcmp(bssid, sta_bssid, 6) == 0);
         } else {
           e.bssid[0] = '\0';
           e.connected = false;
         }
         e.rssi = WiFi.RSSI(k);
         e.channel = WiFi.channel(k);
         e.auth = WiFi.encryptionType(k);
       }
       g_safeboot_scan.count = copy_n;
       g_safeboot_scan.has_result = true;
       g_safeboot_scan.finished_ms = millis();
       g_safeboot_scan.in_progress = false;
       WiFi.scanDelete();
     }
     // WIFI_SCAN_RUNNING (-1): still scanning, nothing to do yet.
   }

   ElegantOTA.loop();

   portENTER_CRITICAL(&g_ota_mux);
   g_ota.tick(now);
   portEXIT_CRITICAL(&g_ota_mux);

   // Drain every action the state machine has queued since the last pass:
   // Abort -> the real Update.abort() (+ print + onAbort callback); by the
   // time control reaches here the ABORT(stale_session) case has usually
   // already been drained synchronously inside /ota/start (ElegantOTA.cpp)
   // -- this still catches write_failed/client_disconnected/stalled.
   // SwitchPartition -> the boot-partition switch, only ever queued from a
   // verified image. RebootToApp -> a timeout/cancel reboot is handled
   // here directly; a successful update's reboot is left to ElegantOTA's
   // own 2.5 s auto-reboot (ElegantOTA.loop() above), matching the existing
   // "let the HTTP response reach the client first" behaviour.
   for (;;) {
     safeboot::OtaSession::Action action;
     bool has;
     portENTER_CRITICAL(&g_ota_mux);
     has = g_ota.pop(action);
     portEXIT_CRITICAL(&g_ota_mux);
     if (!has) {
       break;
     }

     switch (action.type) {
       case safeboot::OtaSession::ActionType::Abort:
         ElegantOTA.abortActiveUpdate(safeboot::OtaSession::reasonName(action.reason));
         // An abort that landed after chunks were written into the single
         // app slot is exactly the case that can invalidate ota_0 (bench
         // finding, docs/safeboot-ota-contract.md "Single app slot") --
         // re-check now so the fallback timer stays suspended instead of
         // looping.
         refreshAppValid();
         break;

       case safeboot::OtaSession::ActionType::SwitchPartition:
         setBootPartition_APP();
         // onVerified(ok) already set app_valid; re-check so the flag
         // reflects the real image (demotes to false if the switch failed).
         refreshAppValid();
         break;

       case safeboot::OtaSession::ActionType::RebootToApp:
         if (action.reason == safeboot::OtaSession::Reason::None) {
           // Success path: ElegantOTA's own 2.5 s auto-reboot handles it
           // (its _reboot flag is set in the /ota/upload completion handler
           // once state().image_valid is true).
           break;
         }
         Serial.printf("[SAFEBOOT];fallback;reason;%s\n", safeboot::OtaSession::reasonName(action.reason));
         // Legacy line kept verbatim (contract: "so old tools still match").
         Serial.println("OTA Start Timeout. Rebooting to app partition.");
         // Unlike the success path, this reboot has no matching
         // SwitchPartition action (ota_state.h only ever queues that one
         // from onVerified(ok=true)) -- but the boot pointer was set to the
         // safeboot/factory partition explicitly on safeboot entry (see
         // command_functions.cpp), so it must be pointed back at the app
         // here, or ESP.restart() would boot straight back into safeboot.
         // Single app slot (bench finding): on a half-written ota_0 this
         // switch fails (ESP_ERR_OTA_VALIDATE_FAILED) -- only restart when
         // it actually succeeded, or this becomes the 180 s reboot loop
         // the fix is for. g_ota should already be quiet in that case
         // (app_valid is refreshed after every Abort, above) but this is
         // kept as its own guard rather than trusting that alone.
         if (!setBootPartition_APP()) {
           break;
         }
         delay(1000);
         ESP.restart();
         break;
     }
   }
 }
