#include <cstdint>

#include "configuration.h"
#include "country_profile.h"

// C5 carve-out of the country table out of lora_setchip.cpp; see
// country_profile.h. The case bodies are moved unchanged apart from the
// assignment target (meshcom_settings.node_* -> out.*) and `break;` becoming
// `return true;`.

// RF-08 (BACKLOG), resolved as "not a defect": country 5 ("868") has no real
// APRS/track sub-channel to assign, unlike every other code. 999 stands in for "none",
// and is safe *only* because it is out of every actually-shipped radio's
// tunable range, not because anything here gates it.
//
// track mode is reachable on a country-5 node: --track has no country guard
// (bDisplayTrack in loop_functions.cpp toggles unconditionally), so
// out.track_freq does reach meshcom_settings.node_track_freq
// (lora_setchip.cpp:207) and then lora_setchip_aprs()'s ESP32 branch, which
// reads it straight into `rf_freq` (lora_setchip.cpp:~456) and hands it to
// lora_setchip_new() -> radio.setFrequency(rf_freq). It stops there: every
// RadioLib driver this firmware actually instantiates rejects 999 MHz before
// any register write --
//   SX1278::setFrequency (SX127X)                 137 ..  525 MHz
//   SX1262::setFrequency (SX1262X/E22/V3/V4/...)   150 ..  960 MHz
//   SX1268::setFrequency (SX126X/E22)              410 ..  810 MHz
// (RadioLib RADIOLIB_CHECK_RANGE in each modules/SX12{6,7}x/*.cpp) -- 999 is
// above all three ceilings, so setFrequency() returns
// RADIOLIB_ERR_INVALID_FREQUENCY, lora_setchip_new() returns false
// (lora_setchip.cpp:~506-509), lora_setchip_aprs() propagates that, and the
// caller in lora_functions.cpp (~:1888) rolls the TX back -- nothing is ever
// written to an antenna. An EXTERNAL_RADIO board never even gets that far:
// lora_setchip_new()'s EXTERNAL_RADIO branch returns before touching rf_freq
// at all. The nRF52 path is unaffected regardless (lora_setchip.cpp:387
// hardcodes LORA_APRS_FREQUENCY and never reads node_track_freq).
//
// Confirmed 2026-09-12 against the RadioLib version vendored for every board
// this firmware ships (heltec_wifi_lora_32_V3 checked; SX1278/SX1262/SX1268
// share the same range macro across envs). If a future RadioLib version, a
// new chip family, or a change to the EXTERNAL_RADIO bridge removes that
// range check, this sentinel stops being safe -- re-derive the guard before
// touching this value, do not just trust the comment.
#define TRACK_FREQ_NONE_SENTINEL 999 // no APRS/track frequency defined for this region

// PRE-EXISTING DEFECT, carried over unchanged from the switch this table
// replaced -- recorded here 2026-09-16 rather than fixed, because changing an
// emitted RF value is a behaviour change that needs its own row and a bench
// proof, not a drive-by edit inside a DRY refactor.
//
// `track_freq` is ONE column shared by both platforms, while `freq` has
// separate esp32/nrf52 columns -- and the two platforms do not use the same
// unit. ESP32 stores MHz (test/golden/native/country-profile-esp32.txt:
// `track=433.774994`), the nRF52 stores Hz (country-profile-nrf52.txt:
// `track=433775008`). Every row gets that right because it uses the
// LORA_APRS_FREQUENCY macro, which is per-platform.
//
// Poland (code 15) does not: it is the one row with a bare literal,
// `434.855f`. On ESP32 that is correct. On the nRF52 it lands in a Hz column
// as 434.855 Hz -- below every RadioLib floor, so a Polish nRF52 node with
// --track on gets RADIOLIB_ERR_INVALID_FREQUENCY, exactly like the 999
// sentinel above. Fixing it means 434855000.0f on the nRF52 side, which needs
// track_freq split into two columns like freq already is.

// D3-05: the 14-case switch collapsed into a const lookup table. Every case
// assigned the same six fields (freq, bw, sf, cr, track_freq, preamble) and
// differed only in the numbers -- and, for freq/bw/cr, by platform, because
// those three are stored in different units on the two sides (OPT-D14, see
// country_profile.h). sf/track_freq/preamble are the same value on both
// platforms for every row here, so those get one column each; freq/bw/cr get
// an _esp32 and an _nrf52 column instead of a per-row `#if`.
//
// Case 7 (MAN) never reaches this table: countryProfile() returns false for
// it before the lookup, exactly as the switch's `case 7: return false;` did.
// The table itself must stay `static const` -- see the D3-05 report for the
// `nm`/`objdump` check that it lands in .rodata (flash), not .data (RAM).
struct CountryRfProfile
{
    int8_t code;
    float  freq_esp32;
    float  freq_nrf52;
    float  bw_esp32;
    float  bw_nrf52;
    int8_t cr_esp32;
    int8_t cr_nrf52;
    int8_t sf;
    float  track_freq;
    int8_t preamble;
};

static const CountryRfProfile kCountryRfProfiles[] = {
    // code  freq_esp32     freq_nrf52     bw_esp32  bw_nrf52  cr_esp32  cr_nrf52  sf        track_freq                 preamble
    {   1,   439.9125f,     439912500.f,   125.0f,   0.0f,     6,        1,        10,       (float)LORA_APRS_FREQUENCY,      8 }, // UK
    {   2,   (float)RF_FREQUENCY,  (float)RF_FREQUENCY,  125.0f,   0.0f,     6,        2,        10,       (float)LORA_APRS_FREQUENCY,      8 }, // ON
    {   4,   433.9250f,     433925000.f,   125.0f,   0.0f,     6,        2,        10,       (float)LORA_APRS_FREQUENCY,      8 }, // LA
    {   5,   869.525f,      869525000.f,   250.0f,   1.0f,     6,        2,        LORA_SF,  TRACK_FREQ_NONE_SENTINEL, 8 }, // 868
    {   6,   906.875f,      906875000.f,   250.0f,   1.0f,     6,        2,        LORA_SF,  (float)LORA_APRS_FREQUENCY,      8 }, // 915
    {   8,   (float)RF_FREQUENCY,  (float)RF_FREQUENCY,  250.0f,   1.0f,     6,        2,        LORA_SF,  (float)LORA_APRS_FREQUENCY,      8 }, // EU8 (preamble 8)
    {   9,   439.9125f,     439912500.f,   125.0f,   0.0f,     6,        1,        10,       (float)LORA_APRS_FREQUENCY,      8 }, // UK8
    {  10,   433.175f,      433175000.f,   250.0f,   1.0f,     6,        2,        11,       (float)LORA_APRS_FREQUENCY,      8 }, // US
    {  11,   435.775f,      435775000.f,   250.0f,   1.0f,     6,        2,        11,       (float)LORA_APRS_FREQUENCY,      8 }, // VR2
    {  12,   435.750f,      435750000.f,   250.0f,   1.0f,     6,        2,        11,       (float)LORA_APRS_FREQUENCY,      8 }, // 435
    {  13,   436.250f,      436250000.f,   250.0f,   1.0f,     6,        2,        11,       (float)LORA_APRS_FREQUENCY,      8 }, // 436
    {  14,   442.000f,      442000000.f,   250.0f,   1.0f,     6,        2,        11,       (float)LORA_APRS_FREQUENCY,      8 }, // 442
    {  15,   (float)RF_FREQUENCY,  (float)RF_FREQUENCY,  250.0f,   1.0f,     6,        2,        LORA_SF,  434.855f,                 8 }, // PL
};

// The switch's `default` (EU): whatever code has no row above -- 0, 3, 16,
// and anything else out of range. Unlike every row, its preamble is
// LORA_PREAMBLE_LENGTH, not the literal 8; that split is in the switch this
// replaces and test_table_matches_the_committed_baseline pins it (codes 0/3/16
// print preamble=32 in the stub configs, every table row prints 8).
static const CountryRfProfile kCountryRfProfileDefault = {
    0, (float)RF_FREQUENCY, (float)RF_FREQUENCY, 250.0f, 1.0f, 6, 2,
    LORA_SF, (float)LORA_APRS_FREQUENCY, LORA_PREAMBLE_LENGTH
};

bool countryProfile(int iCtry, CountryProfile &out)
{
    if (iCtry == 7)  // MAN ... manual
    {
        // Not a table entry: it validates what is already stored instead of
        // assigning literals, so the caller keeps it. Without this early
        // return, falling through to the default-row lookup below would
        // silently hand back the EU profile for country 7, turning manual
        // mode into EU.
        return false;
    }

    const CountryRfProfile *row = &kCountryRfProfileDefault;

    for (unsigned i = 0; i < sizeof(kCountryRfProfiles) / sizeof(kCountryRfProfiles[0]); i++)
    {
        if (kCountryRfProfiles[i].code == iCtry)
        {
            row = &kCountryRfProfiles[i];
            break;
        }
    }

    #if defined(BOARD_RAK4630) || defined(USE_HELTEC_T114) || defined(BOARD_T_ECHO)
        out.freq = row->freq_nrf52;
        out.bw = row->bw_nrf52;
        out.cr = row->cr_nrf52;
    #else
        out.freq = row->freq_esp32;
        out.bw = row->bw_esp32;
        out.cr = row->cr_esp32;
    #endif

    out.sf = row->sf;
    out.track_freq = row->track_freq;
    out.preamble = row->preamble;

    return true;
}
