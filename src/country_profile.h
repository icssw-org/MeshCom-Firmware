#pragma once

// C5 carve-out (DRY unification U6, audit row D3-05): the country table out of
// lora_setcountry().
//
// The switch was 14 near-identical blocks, each assigning the same six
// settings fields, doubled by a `#if BOARD_RAK4630 || USE_HELTEC_T114 ||
// BOARD_T_ECHO` because those boards store different units (see radio_units.h
// and OPT-D14). countryProfile() is the pure half: it reads nothing, writes
// only the struct, and depends on nothing but the per-variant configuration
// macros -- which is what makes test_country_twin possible, one native env per
// `#if` side, all 15 codes compared as tuples.
//
// The field types mirror the settings struct, and the values are in the
// platform's STORED unit, not a normalized one: freq is MHz on the SX127x path
// and Hz on the SX126x path, bw is kHz vs a bandwidth index, cr is a 4/N
// denominator vs an index. Normalizing them is OPT-D14's job and a separate
// decision; this carve preserves the split exactly as it is.
struct CountryProfile
{
    float freq;
    float bw;
    int   sf;
    int   cr;
    float track_freq;
    int   preamble;
};

// Fills `out` for a country code and returns true. Returns false for code 7
// (MAN, manual), which is not a table entry at all: it validates whatever is
// already stored rather than assigning literals, so it stays with the caller.
// Unknown codes fall to the EU default and return true, as the switch did.
bool countryProfile(int iCtry, CountryProfile &out);
