// settings_store: a standalone, host-testable codec for the D1-04 target
// architecture ("D1-04 target architecture: schema-driven settings",
// docs/BACKLOG.md, decided 2026-09-12).
//
// WHY THIS EXISTS: before W3 the nRF52 on-disk settings format WAS the C
// struct (nrf52_flash.cpp read/wrote sizeof(s_meshcom_settings) raw bytes), so
// any field reorder silently misread and any size change wiped the file. Since
// W3c that legacy blob is only READ, through the frozen s_ble_settings_v1
// layout, and migrated once. The target replaces it with a schema-driven,
// self-describing record format on both platforms, driven by the same field
// table config_json.cpp's X() macro already builds for JSON export/import.
//
// THIS FILE IS THE CODEC ONLY. It knows nothing about s_meshcom_settings,
// X(), NVS, or InternalFS -- no platform header may be included here or in
// settings_store.cpp, on purpose (see the platformio.ini env comment for
// native_settings_store). Wiring a real field-descriptor array generated
// from the X() table, and calling encode()/decode() from nrf52_flash.cpp /
// esp32_flash.cpp, is later wave work. This wave only has to prove the
// codec correct against a schema it invents for its own tests -- getting
// that wrong here is cheap; getting it wrong after it reaches a device costs
// every node its settings.
//
// ---------------------------------------------------------------------------
// RECORD FORMAT
// ---------------------------------------------------------------------------
//
// The on-disk/in-memory encoded form is a sequence of lines:
//
//   key=value\n
//
// one per field, in the order the caller's FieldDescriptor array lists them.
// No header, no length prefix, no framing beyond the newline. This is the
// STRING-KEYED encoding the sizing doc decided on
// (docs/opt07-nrf52-settings-store-sizing-20260912.md): 2 937 B worst case
// against 149 304 B free flash on the tightest nRF52 env, a ~50x margin --
// the tagged-record fallback is NOT built here because the decision already
// closed in its favour.
//
// Keys are caller-supplied C strings from the descriptor table (the X()
// table generates them in a later wave) and are NEVER escaped or
// re-interpreted by this codec: a key must not itself contain '=', '\n' or a
// NUL byte, which is a property of the descriptor table, not of any input
// this codec parses. A line is split on its FIRST '=' only, so a value MAY
// contain '=' freely with no escaping at all.
//
// ---------------------------------------------------------------------------
// THE FOUR READ BEHAVIOURS (each has a dedicated test in test_settings_store)
// ---------------------------------------------------------------------------
//
// 1. VALUE ESCAPING (a value containing '\n' or '='):
//    - '=' needs NO escaping. Splitting on the first '=' in the line is
//      unambiguous regardless of how many further '=' characters the value
//      holds, so CFG_STR values pass '=' through literally.
//    - '\n' cannot appear literally in a value -- it is the line terminator.
//      encode() escapes exactly three bytes in STRING values:
//        '\\' -> "\\\\"   (backslash, so escaping itself is unambiguous)
//        '\n' -> "\\n"
//        '\r' -> "\\r"    (defensive: keeps a stray CR from ever reaching
//                           the file byte-for-byte, though nothing in this
//                           codebase currently puts one in a settings field)
//      Every other byte, ASCII or not, is copied through unescaped -- this
//      is a byte-oriented codec, not a text/locale-aware one.
//      decode() reverses exactly those three sequences. A backslash
//      followed by anything else (including end-of-value) is malformed
//      (behaviour 4).
//
// 2. UNKNOWN KEY ON READ: IGNORED. A line whose key does not match any
//    descriptor in the caller-supplied array is skipped: it does not affect
//    any field, is not an error, and decoding continues with the next line.
//    Counted in DecodeStats::unknown_keys so a caller/test can observe it
//    happened, but it never causes DecodeStats to signal failure -- there is
//    no failure signal for decode() as a whole (see "no whole-decode
//    failure" below). This is what lets an old settings file, or one hand-
//    edited to remove a field, load cleanly on a newer schema that has since
//    dropped or renamed that key -- exactly the "unknown key ignored" half
//    of the D1-04 target's stated contract.
//
// 3. MISSING KEY ON READ: DEFAULT. decode() never zeroes or resets the
//    caller's state before applying fields -- it only OVERWRITES the bytes
//    of a field whose key it finds a valid line for. A field whose key never
//    appears in the input is left exactly as the caller had it before
//    calling decode(). The caller is expected to have already initialised
//    `state` to its defaults (the same discipline configImportJson already
//    follows: defaults come from struct initialisation, decode only patches
//    what the record actually specifies). This is what lets a new firmware
//    generation add a field: an old settings file that predates it simply
//    never mentions the key, and the field keeps whatever default the
//    caller's own initialisation gave it -- the "missing key takes its
//    default" half of the D1-04 target's stated contract.
//
// 4. MALFORMED LINE: SKIPPED, decoding continues. There is no whole-decode
//    failure return -- decode() always returns a DecodeStats, never an error
//    code, because a keyed store's whole reason to exist is that ONE bad or
//    unrecognised line must never take the rest of the file down with it
//    (contrast today's nRF52 loader, where a single sizeof() mismatch wipes
//    every field). A line is malformed, and counted in
//    DecodeStats::malformed_lines, in exactly these cases, and none other:
//      - the line contains no '=' at all (including an empty line -- two
//        consecutive '\n's, or a leading/trailing stray '\n', produce a
//        zero-length line, which has no '=' and is therefore malformed);
//      - the key matches a descriptor, but the value cannot be decoded for
//        that field's type: a numeric value with trailing garbage or that
//        does not fit the integer width and has no descriptor range to
//        clamp into (behaviour below), a bool value that is not exactly "0"
//        or "1", or a STRING escape sequence that is invalid or truncated;
//      - the key matches a CFG_STR-equivalent (STRING) descriptor, but the
//        unescaped value is longer than the field can hold. **This is
//        rejected outright, never truncated** -- silently truncating a
//        setting is a worse failure than leaving it at its default, because
//        it produces a plausible-looking but wrong value with no signal
//        that anything went wrong.
//    A malformed line never touches `state`: the field it would have set
//    (if its key even matched one) keeps its prior value, exactly as for a
//    missing key. Every other, well-formed line in the same input still
//    decodes normally -- malformed-ness is a per-line property, not a
//    whole-input one.
//
// ---------------------------------------------------------------------------
// TYPE COVERAGE
// ---------------------------------------------------------------------------
//
// Limited to the integer widths actually present in s_meshcom_settings
// (grepped 2026-09-12 across src/nrf52/WisBlock-API.h and
// src/esp32/esp32_flash.h: plain `int`, `char`/`uint8_t`, `unsigned
// int`/`uint32_t`, `float`, `double`, `bool`, fixed `char[N]` strings --
// nothing 16- or 64-bit). Extending FieldType to a width this struct never
// uses would be untested scope creep, not schema coverage:
//
//   I8, U8    -- int8_t / uint8_t (covers CFG_CHR's single `char` too --
//                a bare `char`'s signedness is platform-defined, so callers
//                with an unsigned single-char field describe it as U8)
//   I32, U32  -- int32_t / uint32_t (covers CFG_INT's plain `int` and
//                CFG_U32's `unsigned int`/`uint32_t`)
//   FLOAT     -- float  (CFG_FLT)
//   DOUBLE    -- double (CFG_DBL)
//   BOOL      -- bool   (CFG_BOOL)
//   STRING    -- fixed char[N] buffer (CFG_STR), N given by the
//                descriptor's `size`
//
// Range clamping (FieldDescriptor::has_range / min_value / max_value)
// applies to every numeric type (I8/U8/I32/U32/FLOAT/DOUBLE) and ONLY on
// decode: a value inside the descriptor's [min_value, max_value] decodes as
// itself, a value outside it is CLAMPED to the nearer bound rather than
// rejected. This is deliberately different from behaviour 4's string-too-
// long case: a range is the descriptor author's stated valid envelope for an
// otherwise well-formed number (e.g. a lat/lon field), so clamping into it
// is a correction; a string that does not fit its buffer is not a value at
// all, so there is nothing to clamp it to. encode() never clamps -- it
// serialises whatever `state` already holds, trusting the caller to have
// kept it valid by construction.
//
// When a numeric field has NO descriptor range, decode() still rejects a
// value that overflows the FIELD'S native width (e.g. "999" for an I8 field
// with no range) -- that is behaviour 4 (malformed), not a silent wraparound
// or truncation. A range, when present, is assumed to itself lie within the
// field's native width; this codec does not re-validate that assumption
// (docs/d1-04-settings-field-triage-20260912.md's field triage is where
// that would be caught, once a real descriptor table exists).
//
// ---------------------------------------------------------------------------
// THE FLOAT/DOUBLE GUARANTEE -- stated honestly, not claimed as "exact"
// ---------------------------------------------------------------------------
//
// encode() formats FLOAT with "%.9g" and DOUBLE with "%.17g" -- always at
// that fixed precision, never a shorter "looks clean" form. 9 and 17
// significant decimal digits are FLT_DECIMAL_DIG and DBL_DECIMAL_DIG: on a
// C99-conforming, correctly-rounding printf/strtod (both glibc's and
// macOS/BSD libc's are), that many significant digits is PROVEN sufficient
// to round-trip every distinct finite IEEE-754 binary32/binary64 value back
// to its exact original bit pattern -- not an approximation, a guarantee
// from the digit count itself. This is why the guarantee holds even for
// values a naive "%f" or "%.6g" loses: 1.0f/3.0f (0x3EAAAAAB) printed at 6
// significant digits round-trips to a DIFFERENT float than it started as;
// printed at 9 it round-trips to the same bit pattern every time. The
// round-trip test in test_settings_store checks exactly that value, plus
// DBL_MIN/DBL_MAX/FLT_MIN/FLT_MAX and -0.0 (whose sign bit "%g" preserves
// and strtod/strtof restores).
//
// What is NOT guaranteed, and deliberately untested here because it would
// be a false claim: locale independence. decode() parses through strtoll/
// strtoull/strtod, which read the process's current LC_NUMERIC decimal
// point. This codec assumes the "C" locale, exactly as the firmware runtime
// it will eventually be wired into has no locale support to change that
// assumption in the first place.
//
// NaN and +-Infinity: encode()/decode() round-trip them too (a C99 printf
// emits "nan"/"inf"/"-inf" for %g on those values and strtod parses them
// back), as a consequence of using the standard formatting/parsing
// functions rather than a special-case guard -- not a separately hand-built
// feature, so there is little to get wrong, but it is real coverage and one
// round-trip test exercises it.
//
// ---------------------------------------------------------------------------
// ENCODE CONTRACT
// ---------------------------------------------------------------------------
//
// encode() is all-or-nothing: it first computes the exact number of bytes
// the full encoded output needs, and only writes to `out` if that fits
// within `out_cap`. If it does not fit, encode() returns -1 and `out` is
// left completely untouched (no partial line is ever written) -- there is
// no truncation mode. On success it returns the number of bytes written,
// which does NOT include a trailing NUL (none is appended; `out` is treated
// as a byte buffer, not a C string, so a caller that wants to print or log
// it should NUL-terminate itself if `out_cap` has room, or use the returned
// length).
//
// encode() assumes every STRING field in `state` is already NUL-terminated
// within its declared buffer (every struct this will eventually be wired to
// maintains that invariant already). To stay memory-safe even if that
// invariant were ever violated by a caller bug, encode() never reads past
// `size` bytes of a STRING field -- but the text it then emits for that one
// field is undefined content (not a crash, just not meaningful), which is a
// caller bug to fix, not something this codec can repair.

#pragma once

#include <cstddef>
#include <cstdint>

namespace settings_store {

// The type an individual field is encoded/decoded as. See "TYPE COVERAGE"
// above for exactly why this list and no wider one.
enum class FieldType : uint8_t {
    I8,
    U8,
    I32,
    U32,
    FLOAT,
    DOUBLE,
    BOOL,
    STRING,
};

// One row of a caller-supplied schema. A real one, generated from
// config_json.cpp's X() table, is later-wave work -- this codec only
// consumes the array, it never builds one.
struct FieldDescriptor {
    // Key text for the "key=value" line. Never escaped or validated by this
    // codec: must not itself contain '=', '\n', or a NUL byte (a property of
    // the descriptor table). Must outlive every encode()/decode() call that
    // uses this descriptor (the codec keeps no copy).
    const char *key;

    FieldType type;

    // Byte offset of this field within the caller's `state` struct, e.g.
    // offsetof(MyState, my_field).
    size_t offset;

    // MEANINGFUL FOR STRING ONLY: the declared size of the char[N] buffer,
    // i.e. N including the NUL terminator slot (so N-1 is the maximum
    // content length). Ignored for every other type -- their encoded width
    // is implied by `type` itself, so a scalar descriptor may leave this 0.
    size_t size;

    // When true, decode() clamps an out-of-envelope numeric value into
    // [min_value, max_value] instead of rejecting it (see "THE FLOAT/DOUBLE
    // GUARANTEE" section above for why this differs from the string-too-
    // long case). Ignored for STRING and BOOL. When false, min_value/
    // max_value are ignored and a numeric value is instead rejected outright
    // if it overflows the field's native width.
    bool has_range;
    double min_value;
    double max_value;
};

// Counts of what decode() did, returned instead of a pass/fail code because
// a keyed store's whole point is that no single bad line invalidates the
// rest of the input (see "MALFORMED LINE" above). A caller/test reads these
// to confirm what happened; decode() itself never "fails".
struct DecodeStats {
    // Non-empty '\n'-delimited segments the input was split into (a
    // trailing '\n' does not itself start a further empty segment; see the
    // header comment above for the exact splitting rule).
    size_t lines_total = 0;

    // Lines that matched a known key and decoded successfully, i.e. fields
    // actually written into `state`.
    size_t fields_set = 0;

    // Lines with a well-formed "key=value" shape whose key matched no
    // descriptor. state is untouched for these.
    size_t unknown_keys = 0;

    // Lines that were not "key=value" at all, or whose value could not be
    // decoded for its field's type (see behaviour 4 above for the exact
    // list). state is untouched for these.
    size_t malformed_lines = 0;
};

// Encodes every field described by fields[0..field_count) by reading it out
// of *state, into `out` as "key=value\n" lines in descriptor order. See
// "ENCODE CONTRACT" above: all-or-nothing, returns the number of bytes
// written (>= 0) on success or -1 if out_cap is too small (and leaves `out`
// untouched in that case). `out` is not NUL-terminated.
long encode(const FieldDescriptor *fields, size_t field_count,
            const void *state, char *out, size_t out_cap);

// Decodes `in` (in_len bytes, need not be NUL-terminated and need not end in
// '\n') into *state: every field named by a well-formed, recognised,
// in-range-or-clampable "key=value" line is overwritten; every field whose
// key never appears, or whose line was malformed, is left exactly as it was
// found in *state on entry. See the four read behaviours above. Returns
// counts, never a failure code.
DecodeStats decode(const FieldDescriptor *fields, size_t field_count,
                    void *state, const char *in, size_t in_len);

} // namespace settings_store
