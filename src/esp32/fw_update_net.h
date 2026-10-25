// fw_update_net.h -- network and staging layer of the firmware auto update (#1187, AU-04)
//
// One FreeRTOS task does everything that blocks (TLS handshake 2-5 s, the
// download, the flash erase and write, the read-back): the loop task only
// starts a job and polls a status snapshot. At most one job runs at a time.
//
//   CHECK     GET api.github.com/repos/<repo>/releases/latest, whole body
//             through two FwJsonScan (the .autoupdate asset and the raw .bin), then
//             fwShouldInstall(). Never downloads. availNewer without an asset or
//             digest stays true (notify), installable is then false and zlen 0.
//   DOWNLOAD  needs the last CHECK (availNewer + digest). fwStageLayout(), erase
//             the stage range at the END of ota_0, stream the .autoupdate into it
//             in 4 KB blocks, SHA-256 against the API digest (mandatory), CRC32
//             read-back, then the FWS2 record into NVS.
//
// Battery, phone, WiFi-up policy and the retry timer (fwAttemptFailed) are the
// caller's job (fw_update.h). Serial markers: [AU];check|refuse|dl|stage|fail|heap|ble|alloc|tls.
//
// Bench staging (INSTRUMENT_ENABLED builds only, no symbol and no string in a release image):
//   STAGELAN  the same stage path as DOWNLOAD (fwStageLayout, erase, write, read-back CRC,
//             FWS2 record) fed from a bench-LAN HTTP server over PLAIN http. The URL is
//             vetted by fwLanUrlParse() (private IPv4 only). The compressed length is the
//             Content-Length, the SHA-256 is computed and recorded (no API digest to compare).
//
// NVS namespace "fwstage": key "rec" = FWS2 record blob (FW_STAGE_ENC_LEN bytes,
// FwStageRecord::off is relative to the start of ota_0; crc32 is the zlib CRC-32
// of the staged compressed bytes), key "last" = tag string last staged (AU-D16).

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

// ---------------------------------------------------------------------------
// Bench-LAN URL vetting. Accepts exactly
// "http://A.B.C.D[:port]/path" with A.B.C.D in 10/8, 172.16/12 or 192.168/16. Everything
// else (https, hostnames, userinfo, public / loopback / link-local addresses, a missing
// path, control characters, spaces) is refused. Not guarded: it is plain C++.
// ---------------------------------------------------------------------------
struct FwLanUrl
{
    char host[16];   // dotted quad, canonical (no leading zeros)
    uint16_t port;   // 80 if the URL has none
    char path[160];  // starts with '/'
};

inline bool fwLanUrlParse(const char *url, FwLanUrl &out)
{
    out.host[0] = '\0';
    out.port = 0;
    out.path[0] = '\0';
    if (url == nullptr)
        return false;
    static const char kScheme[] = "http://";
    const char *p = url;
    for (size_t i = 0; i < sizeof(kScheme) - 1; i++)
        if (p[i] != kScheme[i])
            return false;
    p += sizeof(kScheme) - 1;

    uint32_t oct[4];
    for (int i = 0; i < 4; i++)
    {
        uint32_t v = 0;
        int n = 0;
        while (*p >= '0' && *p <= '9')
        {
            v = v * 10u + (uint32_t)(*p - '0');
            p++;
            if (++n > 3)
                return false;
        }
        if (n == 0 || v > 255u || (n > 1 && p[-n] == '0'))
            return false; // empty, out of range, or a leading zero (octal ambiguity)
        oct[i] = v;
        if (i < 3)
        {
            if (*p != '.')
                return false;
            p++;
        }
    }
    const bool priv = oct[0] == 10u || (oct[0] == 172u && oct[1] >= 16u && oct[1] <= 31u) ||
                      (oct[0] == 192u && oct[1] == 168u);
    if (!priv)
        return false;

    uint32_t port = 80u;
    if (*p == ':')
    {
        p++;
        port = 0;
        int n = 0;
        while (*p >= '0' && *p <= '9')
        {
            port = port * 10u + (uint32_t)(*p - '0');
            p++;
            if (++n > 5)
                return false;
        }
        if (n == 0 || port < 1u || port > 65535u)
            return false;
    }
    if (*p != '/')
        return false;
    size_t pl = 0;
    for (; p[pl] != '\0'; pl++)
    {
        const unsigned char c = (unsigned char)p[pl];
        if (c <= 0x20u || c >= 0x7Fu || c == '#' || c == '\\')
            return false;
        if (pl + 1 >= sizeof(out.path))
            return false;
    }
    for (size_t i = 0; i < pl; i++)
        out.path[i] = p[i];
    out.path[pl] = '\0';

    const unsigned n = (unsigned)snprintf(out.host, sizeof(out.host), "%u.%u.%u.%u",
                                                    (unsigned)oct[0], (unsigned)oct[1],
                                                    (unsigned)oct[2], (unsigned)oct[3]);
    if (n >= sizeof(out.host))
        return false;
    out.port = (uint16_t)port;
    return true;
}

#if defined(ESP32)

#include "sdkconfig.h" // CONFIG_IDF_TARGET_ESP32 (AU_BLE_PAUSE)
#include "../fw_update.h"

enum FwNetJob : uint8_t
{
    FWJ_NONE = 0,
    FWJ_CHECK,
    FWJ_DOWNLOAD,
    FWJ_STAGELAN   // INSTRUMENT_ENABLED builds only (fwNetStartLan)
};

enum FwNetState : uint8_t
{
    FWS_IDLE = 0,
    FWS_BUSY,
    FWS_DONE_OK,
    FWS_DONE_FAIL
};

struct FwNetStatus
{
    FwNetState state;
    FwNetJob job;          // the running or last finished job
    char availTag[24];     // tag of the newest release seen by the last CHECK, "" if none
    bool availNewer;       // fwShouldInstall() said yes (and not draft/prerelease)
    bool installable;      // the release carries the .autoupdate asset AND its sha256 digest
    uint32_t zlen, ilen;   // .autoupdate size (0 = not installable), raw .bin size (0 = none)
    char stagedTag[24];    // tag of the valid FWS2 record, "" if none
    bool staged;           // a valid FWS2 record exists in NVS
    char lastErr[48];      // short reason of the last failed job, "" otherwise
    uint32_t lastCheckEpoch; // time() at the last successful CHECK, 0 if none this boot
    uint32_t bytesDone;    // DOWNLOAD / STAGELAN progress (compressed bytes written); reset to 0 when a
                           // job starts and NOT cleared at its end (it then equals progLen on success). Only
                           // meaningful together with state == FWS_BUSY and job DOWNLOAD / STAGELAN: the
                           // caller derives pct from busy-only (progLen > 0), never from a finished job.
    uint32_t progLen;      // DOWNLOAD / STAGELAN progress denominator: the compressed length the running job
                           // stages (CHECK's zlen for DOWNLOAD, Content-Length for STAGELAN). 0 until the
                           // job has prepared the stage. Separate from zlen, which DOWNLOAD reads as the
                           // expected asset size. pct = fwProgressPct(bytesDone, progLen), busy-only.
    uint32_t stagedIlen;   // inflated image length of the staged FWS2 record (set at staging and when the
                           // record is loaded from NVS), 0 if nothing is staged. Not the last CHECK's ilen.
};

// False if a job runs, WiFi is not connected, or the task could not be created.
// `channel` is a FwChannel. The job runs in its own task (12 KB stack, prio 1,
// core 1) that deletes itself; the status moves BUSY -> DONE_OK / DONE_FAIL.
bool fwNetStart(FwNetJob job, uint8_t channel);

// INSTRUMENT_ENABLED builds only: start a STAGELAN job (see the header comment). `url` is
// vetted with fwLanUrlParse(), `ilen` is the expected inflated length (> 0), `tag` must
// parse with fwParseTag(). False if any of that fails, a job runs, or WiFi is down.
// Declared unconditionally (a prototype costs nothing); defined only with INSTRUMENT_ENABLED.
bool fwNetStartLan(const char *url, uint32_t ilen, const char *tag);

// Deliberate reboot into Safeboot, which applies the staged update (inflates the FWS2
// record's image into ota_0). The shared body of `--ota-update` and the AU handover; defined
// in command_functions.cpp. Shows the OTA screen, waits 2 s, sets the Safeboot partition as
// boot partition, clears the loop breadcrumb and restarts. Returns false (and does not
// restart) if there is no Safeboot partition; it does not return on success.
bool auRebootToSafeboot(void);

// Thread-safe snapshot (portMUX). Never blocks on the network.
void fwNetGetStatus(FwNetStatus &out);

// True while a phone holds the BLE link (connected or app ready): the same expression esp32BlePause()
// declines on and the lowheap_phone token keys on, so /austatus "phone" and the job agree.
bool fwNetPhoneHeld(void);

// Diagnostic heap numbers for /austatus (never a gate). Read-only, non-logging, takes no lock and
// does not install the TLS allocator. Same sources as the pre-TLS gate: *ifree = free internal
// 8-bit heap, *iblk = its largest free block. On a PSRAM-TLS build (BOARD_HAS_PSRAM and PSRAM
// found: mbedTLS runs on PSRAM) *iblk is instead the largest free PSRAM block, because that is
// the block the gate checks there. Either pointer may be null.
void fwNetHeapNow(uint32_t *ifree, uint32_t *iblk);

// INSTRUMENT_ENABLED builds only (bench hooks of `--update heapgate|forget|checknow`; declared
// unconditionally, defined only with INSTRUMENT_ENABLED, no symbol and no string in a release image).
//   fwNetSetHeapGateOverride  RAM only: sets the free-heap threshold of the pre-TLS gate (0 = default) to at
//                             least `bytes` (classic: total free; PSRAM-TLS: internal free), so a
//                             CHECK / DOWNLOAD is refused with lowheap. 0 = back to the build value.
//   fwNetForgetLast           removes NVS "fwstage"/"last" (the reinstall guard), so the tag can
//                             be offered again. True if the namespace opened.
// `--update checknow` needs no function here: the automatic check path is the FwTimer in
// esp32_main.cpp (fwTick), so the console hook sets that timer's nextCheckMs to "now".
void fwNetSetHeapGateOverride(uint32_t bytes);
bool fwNetForgetLast(void);

// A valid FWS2 record exists in NVS. Cached (read once, refreshed by DOWNLOAD),
// cheap enough for every loop pass.
bool fwNetStagedPending(void);

// ---------------------------------------------------------------------------
// BLE pause around CHECK / DOWNLOAD (AU-10, AU-D18). On the classic ESP32 the NimBLE stack
// (controller, HCI buffers, host pools) holds heap that the TLS handshake needs: W0 measured
// 58 KB free before TLS and a 5.4 KB minimum, and the P-384 chain verification then fails
// with -9984. AU_BLE_PAUSE=1 (default on the classic ESP32 only; -DAU_BLE_PAUSE=0/1 overrides)
// makes the job task stop the BLE stack when no phone is connected, run, and bring BLE back
// up exactly as at boot before the job reports DONE. The two functions are defined in
// esp32_main.cpp, which owns the BLE objects.
// ---------------------------------------------------------------------------
#ifndef AU_BLE_PAUSE
#if defined(CONFIG_IDF_TARGET_ESP32)
#define AU_BLE_PAUSE 1
#else
#define AU_BLE_PAUSE 0
#endif
#endif

// Stops advertising and the whole NimBLE stack (NimBLEDevice::deinit(true): the GATT
// objects are rebuilt by the resume). Returns true if the stack WAS stopped (the caller must
// call esp32BleResume()), false if nothing was done: no BLE, a phone is connected or ready,
// or BLE is already down. Prints [AU];ble;pause;heap;<free>;blk;<largest> or
// [AU];ble;pause;skip;<why>. Called from the job task, never from the loop task.
bool esp32BlePause(void);

// Re-initialises BLE exactly as at boot (stack, server, service, characteristics, callbacks,
// advertising). True if BLE is up afterwards (also if it was never paused). Prints
// [AU];ble;resume;ok|fail. Safe to call again after a failure.
bool esp32BleResume(void);

// BLE guard (spec 7, src/fw_update.h auBleGuardRequest): the pause with a phone linked. Stops advertising,
// disables advertiseOnDisconnect, NimBLEDevice::deinit(true) with the link up, then bleSessionResetAll().
// True if the stack was stopped. esp32BleResume() refuses while the guard is set; the loop resumes BLE when it
// releases the guard. Prints [AU];ble;pause;force;phone;<0|1>;heap;..;blk;... Job task only.
#if AU_BLE_PAUSE
bool esp32BleForcePause(void);
#endif

// True while the BLE stack is stopped (pause, force pause or a failed resume).
bool esp32BleDown(void);

// Spec 8.1 step 3 (job task). auShedAvail: what the node could stop now (FW_SHED_* bits of fw_update.h: bit 0 a net
// console session, bit 1 KISS TCP), read only. auShedHold: keep the services of mask stopped through the loop's own
// off paths (the loop does the stopping); mask 0 restores both. auShedSettled: the sockets of mask are closed
// (poll after auShedHold, cap FW_SHED_MAX_MS).
uint8_t auShedAvail(void);
void auShedHold(uint8_t mask);
bool auShedSettled(uint8_t mask);

// Reads and decodes the record from NVS. False if there is none or it is invalid.
// A record for another env, or whose tag is not newer than the running version, is
// stale: it is removed from NVS and false is returned.
bool fwNetLoadRecord(FwStageRecord &out);

// Old-Safeboot guard (AU-08). OTA and AU rewrite only ota_0, so a node updated over the air keeps
// its old Safeboot, which ignores the FWS2 record and boots the app again.
//   fwNetMarkHandover   NVS "fwstage"/"hand" = tag; MUST precede auRebootToSafeboot() on every
//                       handover (auto and --update apply). False if the write failed: do not hand over.
//   fwNetUnmarkHandover removes "hand" again when the reboot call returned (nothing happened).
//   fwNetSafebootOld    true while "nocap" is set. At the first call per boot: "hand" present and the
//                       record still valid -> Safeboot did not process it -> set "nocap", print
//                       [AU];refuse;old_safeboot; "hand" is consumed. A NEW Safeboot removes "hand" and
//                       "nocap" at boot. The automatic handover is suppressed while true; the
//                       operator's `--update apply` still tries.
bool fwNetMarkHandover(const char *tag);
void fwNetUnmarkHandover(void);
bool fwNetSafebootOld(void);

// Safeboot capability version (AU-12, src/safeboot/safeboot_ver.h). The Safeboot partition is scanned
// once per boot (first call) for its embedded version marker; the result is cached.
//   fwNetSafebootVersion  -1 = partition missing or unreadable, 0 = old (no marker, not AU-aware),
//                         1 = first AU-aware image (recognised by strings), N >= 2 = marker version.
//   fwNetSafebootCapable  true when the version is >= AU_SAFEBOOT_MIN and fwNetSafebootOld() is false
//                         (the handover backstop above still overrides a good scan). Auto update may
//                         only be switched on, and the automatic handover only runs, while this is true.
int fwNetSafebootVersion(void);
bool fwNetSafebootCapable(void);

// AU-16: the Safeboot cannot run behind a 2nd-stage bootloader that configured the flash cache in DIO mode
// (ESP32-S3 only; it reads NVS and ota_0 wrongly and destroys the settings).
//   fwNetBootloaderQio  true when SPI0 (flash cache) runs a quad read mode, or the target is not an ESP32-S3.
//                       Read once, cached. fwNetSafebootCapable() is false while this is false.
//   fwNetSpi0Ctrl       raw SPI_MEM_CTRL_REG(0) (0 on other targets), printed by --update status.
//   fwNetSafebootWhy    "ok", "old" (version / nocap) or "dio" (bootloader, wins over "old").
bool fwNetBootloaderQio(void);
uint32_t fwNetSpi0Ctrl(void);
const char *fwNetSafebootWhy(void);

// Removes the record and Safeboot's "tries" counter and clears the cached staged flag. Used
// when a handover cannot succeed (no Safeboot partition, boot partition not settable), so
// it is not retried at every boot (also removes "hand"). The "last" tag (reinstall guard) stays.
void fwNetClearRecord(void);

// NVS "fwstage"/"last"; "" if none. n must be >= 1.
void fwNetLastInstalledTag(char *out, size_t n);

#endif // ESP32
