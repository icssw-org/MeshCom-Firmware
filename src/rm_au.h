// rm_au.h -- bridge from the RM1 command "upd" to the firmware auto-update (AU, #1187). Arduino-free.
// Callers: src/rm_exec_read.cpp ("upd" read) and src/rm_exec_write.cpp ("upd check", "upd go <tag>").
// Implementations: src/esp32/rm_au_esp32.cpp on ESP32 (fw_update_net), src/rm_au_none.cpp everywhere
// else (supported = false, every call RM_AU_UNSUPPORTED).
#ifndef RM_AU_H
#define RM_AU_H

#include <stddef.h>
#include <stdint.h>

#define RM_AU_TAG_MAX 23 // longest tag, the AU buffers are char[24]

struct RmAuState
{
    bool supported;            // false: this build has no AU, every upd answers "err unsupported"
    uint8_t mode;              // node_autoupd: 0 off, 1 notify, 2 auto
    uint8_t chan;              // node_updchan: 0 prod, 1 dev
    bool newer;                // availNewer: the last check found a newer release
    char avail[RM_AU_TAG_MAX + 1];  // availTag, "" if none
    char staged[RM_AU_TAG_MAX + 1]; // stagedTag while a valid staged record exists, "" otherwise
    bool busy;                 // a check or download job is running
    bool goPending;            // a remote "upd go" waits for its download or for the handover checks
    int32_t ckMin;             // minutes since the last successful check, -1 if none this boot
    char err[48];              // lastErr, "" if none
};

void rmAuGetState(RmAuState &out);

enum RmAuResult
{
    RM_AU_STARTED,     // check: started; go: download started ("ok installing <tag>")
    RM_AU_APPLYING,    // go: tag already staged, the handover follows the reply ("ok applying <tag>")
    RM_AU_UNSUPPORTED, // no AU on this build
    RM_AU_BUSY,        // a check or download job is running
    RM_AU_NOWIFI,      // no WiFi STA address
    RM_AU_STALE,       // go: tag is neither the available nor the staged tag
    RM_AU_CURRENT,     // go: no newer release known
    RM_AU_NOASSET,     // go: the newer release carries no .autoupdate asset with digest
    RM_AU_SAFEBOOT,    // go: Safeboot too old to apply an update
    RM_AU_NOTREADY     // go: the auto path would not download now (no valid clock for TLS, a phone connected, or battery below the floor)
};

RmAuResult rmAuCheck(void);
RmAuResult rmAuGo(const char *tag);

#endif
