// msgstore_settings.cpp -- persistence for the store node (mailbox) settings.
// See msgstore_settings.h for the contract and the T13 rationale.
//
// Compiled for every board (src_filter is +<*> for both the esp32 and the
// nrf52_base platformio.ini profiles), so msgstoreHardwareEligible() must
// work everywhere and the ENABLE_MSGSTORE guard below must leave nothing
// behind on an ineligible build -- in particular no "[MBOX]"/mailbox string
// literal, which the release gate string-scans for on the T-Beam image.
#include "msgstore_settings.h"

#include <Arduino.h>

// Pulls in the active variant's configuration.h -> configuration_global.h,
// which is where ENABLE_MSGSTORE, ESP32 and BOARD_RAK4630 come from
// (same include mheard_functions.h uses for the same reason).
#include <configuration.h>

bool msgstoreHardwareEligible(void)
{
#if defined(ENABLE_MSGSTORE)
    return true;
#else
    return false;
#endif
}

#if defined(ENABLE_MSGSTORE)

#include "msgstore_api.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>

#if defined(ESP32)

// ---- ESP32 (S3): own NVS keys, same "Credentials" namespace and
// begin/end-per-call pattern as src/esp32/esp32_flash.cpp:65,334 -- a
// separate Preferences handle, opened and closed here, not the flash
// module's global `preferences` object (own-file rule, stage 3 brief).
#include <Preferences.h>

static Preferences s_msgstore_prefs;

static void msgstoreLoadPlatform(uint8_t *mode, uint8_t *slots, uint16_t *hold_h, char *list, size_t list_n, uint8_t *notice)
{
    s_msgstore_prefs.begin("Credentials", false);

    *mode = s_msgstore_prefs.getUChar("store_mode", (uint8_t)MSGSTORE_OFF);
    *slots = s_msgstore_prefs.getUChar("store_slots", (uint8_t)MSGSTORE_SLOTS_DEFAULT);
    *hold_h = s_msgstore_prefs.getUShort("store_time", (uint16_t)MSGSTORE_HOLD_DEFAULT_H);
    *notice = s_msgstore_prefs.getUChar("store_notice", 1);   // stage 4: default on

    String strVar = s_msgstore_prefs.getString("store_list", "");
    snprintf(list, list_n, "%s", strVar.c_str());

    s_msgstore_prefs.end();
}

static void msgstoreSavePlatform(uint8_t mode, uint8_t slots, uint16_t hold_h, const char *list, uint8_t notice)
{
    s_msgstore_prefs.begin("Credentials", false);

    s_msgstore_prefs.putUChar("store_mode", mode);
    s_msgstore_prefs.putUChar("store_slots", slots);
    s_msgstore_prefs.putUShort("store_time", hold_h);
    s_msgstore_prefs.putString("store_list", String(list ? list : ""));
    s_msgstore_prefs.putUChar("store_notice", notice);

    s_msgstore_prefs.end();
}

#else // nRF52 (BOARD_RAK4630)

// ---- nRF52: no per-key store exists on this port (T13 recon), so the
// store settings get one small standalone LittleFS file, "/msgstore.cfg",
// versioned by a magic marker -- absent file or bad magic falls back to
// defaults, exactly like src/nrf52/nrf52_flash.cpp's flash_reset() path.
// Same includes and File/InternalFS handling as nrf52_flash.cpp:22-31,
// own File instance (not its global `lora_file`).
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
using namespace Adafruit_LittleFS_Namespace;

static const char kMsgStoreFileName[] = "/msgstore.cfg";

// 'M','B','X','1'/'2' packed as a little-endian uint32 -- the struct is
// written and read raw (memcpy-style), same as nrf52_flash.cpp does for the
// whole settings struct, so this only ever has to round-trip on the same
// CPU. v2 (stage 4) appends `notice` after the v1 fields so a v1 file is
// exactly sizeof(MsgStoreFileV1) and is never misread as v2.
#define MSGSTORE_FILE_MAGIC_V1 (((uint32_t)'M') | ((uint32_t)'B' << 8) | ((uint32_t)'X' << 16) | ((uint32_t)'1' << 24))
#define MSGSTORE_FILE_MAGIC_V2 (((uint32_t)'M') | ((uint32_t)'B' << 8) | ((uint32_t)'X' << 16) | ((uint32_t)'2' << 24))

struct MsgStoreFileV1
{
    uint32_t magic;
    uint8_t  mode;
    uint8_t  slots;
    uint16_t hold_h;
    char     list[MSGSTORE_LIST_MAX * MSGSTORE_CALL_MAX];
};

struct MsgStoreFileV2
{
    uint32_t magic;
    uint8_t  mode;
    uint8_t  slots;
    uint16_t hold_h;
    char     list[MSGSTORE_LIST_MAX * MSGSTORE_CALL_MAX];
    uint8_t  notice;   // stage 4: --storenotice, default 1 (on)
};

static File s_msgstore_file(InternalFS);

static void msgstoreLoadPlatform(uint8_t *mode, uint8_t *slots, uint16_t *hold_h, char *list, size_t list_n, uint8_t *notice)
{
    InternalFS.begin();   // idempotent -- init_flash() already mounted it at boot

    struct MsgStoreFileV2 rec;
    memset(&rec, 0, sizeof(rec));
    bool ok = false;
    bool v1 = false;

    s_msgstore_file.open(kMsgStoreFileName, FILE_O_READ);
    if (s_msgstore_file)
    {
        // Size before read, like nrf52_flash.cpp:171 -- close() below makes
        // it unqueryable, and a short/garbled file must not be trusted.
        uint32_t stored_size = s_msgstore_file.size();
        if (stored_size == sizeof(rec))
        {
            s_msgstore_file.read((uint8_t *)&rec, sizeof(rec));
            ok = (rec.magic == MSGSTORE_FILE_MAGIC_V2);
        }
        else if (stored_size == sizeof(struct MsgStoreFileV1))
        {
            struct MsgStoreFileV1 v1rec;
            memset(&v1rec, 0, sizeof(v1rec));
            s_msgstore_file.read((uint8_t *)&v1rec, sizeof(v1rec));
            if (v1rec.magic == MSGSTORE_FILE_MAGIC_V1)
            {
                rec.magic = v1rec.magic;
                rec.mode = v1rec.mode;
                rec.slots = v1rec.slots;
                rec.hold_h = v1rec.hold_h;
                memcpy(rec.list, v1rec.list, sizeof(rec.list));
                rec.notice = 1;   // v1 predates the setting -- default on
                ok = true;
                v1 = true;
            }
        }
        s_msgstore_file.close();
    }

    if (!ok)
    {
        *mode = (uint8_t)MSGSTORE_OFF;
        *slots = (uint8_t)MSGSTORE_SLOTS_DEFAULT;
        *hold_h = (uint16_t)MSGSTORE_HOLD_DEFAULT_H;
        *notice = 1;
        if (list_n > 0)
            list[0] = 0;
        return;
    }

    (void)v1;
    *mode = rec.mode;
    *slots = rec.slots;
    *hold_h = rec.hold_h;
    *notice = rec.notice;
    rec.list[sizeof(rec.list) - 1] = 0;   // raw read, no terminator guaranteed
    snprintf(list, list_n, "%s", rec.list);
}

static void msgstoreSavePlatform(uint8_t mode, uint8_t slots, uint16_t hold_h, const char *list, uint8_t notice)
{
    InternalFS.begin();

    struct MsgStoreFileV2 rec;
    memset(&rec, 0, sizeof(rec));
    rec.magic = MSGSTORE_FILE_MAGIC_V2;
    rec.mode = mode;
    rec.slots = slots;
    rec.hold_h = hold_h;
    snprintf(rec.list, sizeof(rec.list), "%s", list ? list : "");
    rec.notice = notice;

    // Read-compare-write, like nrf52_flash.cpp's save_settings() memcmp
    // guard -- skip the erase/write cycle when nothing actually changed.
    // Save always writes v2, so an on-disk v1 file never compares equal
    // here (size differs) and gets upgraded on the first save.
    struct MsgStoreFileV2 current;
    memset(&current, 0, sizeof(current));

    s_msgstore_file.open(kMsgStoreFileName, FILE_O_READ);
    if (s_msgstore_file)
    {
        if (s_msgstore_file.size() == sizeof(current))
            s_msgstore_file.read((uint8_t *)&current, sizeof(current));
        s_msgstore_file.close();
    }

    if (memcmp(&current, &rec, sizeof(rec)) == 0)
        return;

    InternalFS.remove(kMsgStoreFileName);
    if (s_msgstore_file.open(kMsgStoreFileName, FILE_O_WRITE))
    {
        s_msgstore_file.write((uint8_t *)&rec, sizeof(rec));
        s_msgstore_file.flush();
        s_msgstore_file.close();
    }
}

#endif // ESP32 / BOARD_RAK4630

void msgstoreSettingsLoad(void)
{
    uint8_t mode_raw = (uint8_t)MSGSTORE_OFF;
    uint8_t slots = MSGSTORE_SLOTS_DEFAULT;
    uint16_t hold_h = MSGSTORE_HOLD_DEFAULT_H;
    uint8_t notice_raw = 1;
    char list[MSGSTORE_LIST_MAX * MSGSTORE_CALL_MAX];
    list[0] = 0;

    msgstoreLoadPlatform(&mode_raw, &slots, &hold_h, list, sizeof(list), &notice_raw);

    enum MsgStoreMode mode = (mode_raw <= (uint8_t)MSGSTORE_HEARD) ? (enum MsgStoreMode)mode_raw : MSGSTORE_OFF;
    if (slots < 1) slots = 1;
    if (slots > MSGSTORE_SLOTS_MAX) slots = MSGSTORE_SLOTS_MAX;
    if (hold_h < 1) hold_h = 1;
    if (hold_h > MSGSTORE_HOLD_MAX_H) hold_h = MSGSTORE_HOLD_MAX_H;

    msgstoreConfigure(mode, slots, hold_h);
    msgstoreSetList(list);
    msgstoreSetNotice(notice_raw != 0);
}

void msgstoreSettingsSave(void)
{
    msgstoreSavePlatform((uint8_t)msgstoreMode(), msgstoreSlots(), msgstoreHoldHours(), msgstoreListCsv(),
        (uint8_t)(msgstoreNotice() ? 1 : 0));
}

#else // !ENABLE_MSGSTORE -- nothing to persist, nothing to load

void msgstoreSettingsLoad(void) {}
void msgstoreSettingsSave(void) {}

#endif
