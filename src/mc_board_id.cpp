// F4-1a: see mc_board_id.h.
#include "mc_board_id.h"
#include <stdio.h>
#include <string.h>

int mcBoardIdLine(char *out, size_t n, int hwidRuntime, const mc_board_id_t *id)
{
    if (id == NULL || (out == NULL && n > 0))
        return -1;

    int envLen = 0;
    while (envLen < (int)sizeof(id->env) && id->env[envLen] != '\0')
        envLen++;
    int tagLen = 0;
    while (tagLen < (int)sizeof(id->tag) && id->tag[tagLen] != '\0')
        tagLen++;

    if (tagLen == 0)
        return snprintf(out, n, "MCBOARD:v1:%d:%u:%.*s:(local)", hwidRuntime, (unsigned)id->hwid, envLen, id->env);
    return snprintf(out, n, "MCBOARD:v1:%d:%u:%.*s:%.*s", hwidRuntime, (unsigned)id->hwid, envLen, id->env, tagLen, id->tag);
}

#if !defined(NATIVE_BUILD) && !defined(UNIT_TEST)

#include "configuration.h" // MODUL_HARDWARE (variant)
#ifndef MODUL_HARDWARE
#error "MODUL_HARDWARE must come from the variant configuration.h before mc_board_id.cpp"
#endif

static_assert(sizeof(MC_ENV_NAME) <= 32, "MC_ENV_NAME too long for mc_board_id_t.env");
static_assert(sizeof(MC_BUILD_TAG) <= 24, "MC_BUILD_TAG too long for mc_board_id_t.tag");

// `used` does not survive --gc-sections and the section is not KEEP'd: the
// object stays in the image because the --id handler reads it. If that ever
// goes, a web flasher that checks the marker refuses the image (no MCBOARD1 at 0x120).
#if defined(ESP_PLATFORM) || defined(ARDUINO_ARCH_ESP32)
#define MC_BOARD_ID_SECTION __attribute__((section(".rodata_custom_desc"), used))
#else
#define MC_BOARD_ID_SECTION __attribute__((used))
#endif

extern const mc_board_id_t mc_board_id;
MC_BOARD_ID_SECTION const mc_board_id_t mc_board_id = {
    {'M', 'C', 'B', 'O', 'A', 'R', 'D', '1'},
    1,
    (uint16_t)MODUL_HARDWARE,
    MC_ENV_NAME,
    MC_BUILD_TAG,
    {0}};

#endif
