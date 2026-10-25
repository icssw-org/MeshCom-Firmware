// Board identity marker embedded in every firmware image (web flasher
// autodetect, WF-05).
// ESP32: lives at firmware.bin offset 0x120, right behind esp_app_desc_t.
// Little-endian, exactly 128 bytes.
#pragma once
#include <stdint.h>
#include <stddef.h>

#ifndef MC_ENV_NAME
#define MC_ENV_NAME "unknown"
#endif
#ifndef MC_BUILD_TAG
#define MC_BUILD_TAG ""
#endif

typedef struct {
    char     magic[8];     // "MCBOARD1", no NUL
    uint16_t format;       // 1
    uint16_t hwid;         // MODUL_HARDWARE at compile time
    char     env[32];      // MC_ENV_NAME, NUL-padded
    char     tag[24];      // MC_BUILD_TAG, NUL-padded, empty for local builds
    uint8_t  reserved[60]; // zero
} mc_board_id_t;

static_assert(sizeof(mc_board_id_t) == 128, "mc_board_id_t must be 128 bytes");
// Web flasher tools read these fields by offset: keep the layout fixed.
static_assert(offsetof(mc_board_id_t, format) == 8 && offsetof(mc_board_id_t, hwid) == 10 &&
                  offsetof(mc_board_id_t, env) == 12 && offsetof(mc_board_id_t, tag) == 44,
              "mc_board_id_t field offsets are fixed by the flasher parsers");

// Defined in mc_board_id.cpp on ESP32/nRF52 targets only (not in native tests).
extern const mc_board_id_t mc_board_id;

// Writes "MCBOARD:v1:<hwidRuntime>:<id->hwid>:<env>:<tag or (local)>" (no
// newline), always NUL-terminated when n > 0, never writes past n bytes.
// env/tag need no NUL (read with a bounded length). Returns snprintf-style
// length of the full line, or -1 on bad arguments.
int mcBoardIdLine(char *out, size_t n, int hwidRuntime, const mc_board_id_t *id);
