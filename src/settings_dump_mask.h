// settings_dump_mask.h -- masking of secret rows for the nRF52 `--dumpsettings` output (SEC-DUMP-01, #1291).
//
// Header-only and free of platform includes, so any native env can compile it. The key table mirrors the
// secret rows of the persisted schema (config_json.h); test_dump_secret_list_covers_schema_names guards
// against drift.

#pragma once

#include <stddef.h>
#include <string.h>

#include "mask_secret.h"

// Keys of the settings-store file whose value is a secret.
static const char *const kSettingsDumpSecretKeys[] = {
    "node_passwd",    // console / RM / KISS / AppLink password
    "node_webpwd",    // web GUI password
    "node_lpwd",      // WLAN password
    "node_pwd",       // MeshCom server password (legacy)
    "bt_code",        // BLE pairing code, 0 = no PIN
    "node_aprsispas", // APRS-IS passcode (not persisted on nRF52)
};

static inline bool settingsDumpKeyIsSecret(const char *key, size_t keyLen)
{
    if (key == NULL)
        return false;
    for (size_t i = 0; i < sizeof(kSettingsDumpSecretKeys) / sizeof(kSettingsDumpSecretKeys[0]); i++)
    {
        const char *k = kSettingsDumpSecretKeys[i];
        if (strlen(k) == keyLen && memcmp(k, key, keyLen) == 0)
            return true;
    }
    return false;
}

// One "key=value" line without the newline in, masked line out (NUL-terminated).
// Secret string: "key=***", or "key=" when unset (empty / leading space).
// bt_code: "0" stays "0", anything else "***".
// Non-secret line: copied unchanged. Returns the output length, 0 if outCap is too small.
static inline size_t settingsDumpMaskLine(const char *line, size_t len, char *out, size_t outCap)
{
    if (line == NULL || out == NULL || outCap == 0)
        return 0;

    size_t eq = 0;
    while (eq < len && line[eq] != '=')
        eq++;

    const bool secret = eq < len && settingsDumpKeyIsSecret(line, eq);
    if (!secret)
    {
        if (len + 1 > outCap)
            return 0;
        memcpy(out, line, len);
        out[len] = '\0';
        return len;
    }

    const char *value = line + eq + 1;
    const size_t valueLen = len - eq - 1;
    const char *shown;
    if (eq == 7 && memcmp(line, "bt_code", 7) == 0)
        shown = (valueLen == 1 && value[0] == '0') ? "0" : "***";
    else
    {
        const char first[2] = {valueLen > 0 ? value[0] : (char)0, 0};
        shown = maskSecret(first);
    }

    const size_t shownLen = strlen(shown);
    if (eq + 1 + shownLen + 1 > outCap)
        return 0;
    memcpy(out, line, eq + 1);
    memcpy(out + eq + 1, shown, shownLen + 1);
    return eq + 1 + shownLen;
}
