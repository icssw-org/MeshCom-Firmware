// Implementation of the settings_store codec. See settings_store.h for the
// full format contract -- this file only implements it, it does not restate
// it. No platform header (Arduino, nRF52 SDK, ESP-IDF, ...) may be included
// here: this codec must compile and run on the host, standalone, before
// either platform wires it in (see settings_store.h's top comment and the
// native_settings_store platformio.ini env).

#include "settings_store.h"

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace settings_store {

namespace {

// ---------------------------------------------------------------------------
// A tiny sink used by encode() so the exact same field-formatting code both
// counts the required output size (sink.buf == nullptr) and writes it
// (sink.buf != nullptr) -- see settings_store.h's "ENCODE CONTRACT": encode
// is all-or-nothing, so the size must be known before any byte is written.
// ---------------------------------------------------------------------------
struct Sink {
    char *buf;    // nullptr => counting pass only
    size_t cap;   // capacity of buf, meaningless when buf == nullptr
    size_t pos;   // bytes appended so far (this IS the encoded size once the
                  // counting pass completes)

    void put(char c) {
        if (buf != nullptr && pos < cap) {
            buf[pos] = c;
        }
        pos++;
    }

    void put(const char *s, size_t n) {
        for (size_t i = 0; i < n; i++) {
            put(s[i]);
        }
    }

    void put_cstr(const char *s) { put(s, strlen(s)); }
};

const FieldDescriptor *find_field(const FieldDescriptor *fields,
                                   size_t field_count, const char *key,
                                   size_t key_len) {
    for (size_t i = 0; i < field_count; i++) {
        const FieldDescriptor &f = fields[i];
        if (f.key != nullptr && strlen(f.key) == key_len &&
            memcmp(f.key, key, key_len) == 0) {
            return &f;
        }
    }
    return nullptr;
}

// ---------------------------------------------------------------------------
// Encoding one field's value into `sink`. Shared verbatim between the
// counting pass and the writing pass (see Sink above).
// ---------------------------------------------------------------------------

// The integer encoders convert by hand instead of calling snprintf, and that
// is not a style choice: the nRF52 links newlib-NANO, whose printf is built
// without _WANT_IO_LONG_LONG. It parses the first 'l', does not recognise the
// second as a modifier, and emits the rest of the conversion LITERALLY -- so
// "%lld" wrote the two characters `ld` into the settings file for every
// integer field, and "%llu" wrote `lu`. The write succeeded, the file was
// well formed, and the loss only surfaced one reboot later when decode()
// rejected `ld` and left those fields at their defaults. Floats are fine
// (%.9g/%.17g are supported and were observed correct on hardware); only the
// long-long conversions are missing. Hand conversion removes the dependency
// on a libc build option that no test on the host can see.
// test/golden/nano_printf_lint.py keeps every other nRF52-compiled file off
// the same rake.
void encode_u64(Sink &sink, unsigned long long v) {
    char tmp[24];                       // 20 digits of UINT64_MAX + slack
    size_t n = 0;
    do {
        tmp[n++] = (char)('0' + (int)(v % 10ULL));
        v /= 10ULL;
    } while (v != 0ULL);
    while (n > 0) {
        sink.put(tmp[--n]);
    }
}

void encode_i64(Sink &sink, long long v) {
    // Negated in unsigned space so LLONG_MIN does not overflow on the way.
    unsigned long long mag;
    if (v < 0) {
        sink.put('-');
        mag = (unsigned long long)(-(v + 1)) + 1ULL;
    } else {
        mag = (unsigned long long)v;
    }
    encode_u64(sink, mag);
}

// FLT_DECIMAL_DIG / DBL_DECIMAL_DIG significant digits -- see
// settings_store.h "THE FLOAT/DOUBLE GUARANTEE" for why 9 and 17 are the
// right, and sufficient, constants rather than a "looks clean" precision.
void encode_float(Sink &sink, float v) {
    char tmp[48];
    int n = snprintf(tmp, sizeof(tmp), "%.9g", (double)v);
    sink.put(tmp, (size_t)n);
}

void encode_double(Sink &sink, double v) {
    char tmp[48];
    int n = snprintf(tmp, sizeof(tmp), "%.17g", v);
    sink.put(tmp, (size_t)n);
}

void encode_bool(Sink &sink, bool v) { sink.put(v ? '1' : '0'); }

// Escapes exactly '\\', '\n', '\r' -- see settings_store.h behaviour 1.
void encode_string(Sink &sink, const char *field_ptr, size_t field_size) {
    size_t content_len = strnlen(field_ptr, field_size);
    for (size_t i = 0; i < content_len; i++) {
        char c = field_ptr[i];
        switch (c) {
            case '\\':
                sink.put('\\');
                sink.put('\\');
                break;
            case '\n':
                sink.put('\\');
                sink.put('n');
                break;
            case '\r':
                sink.put('\\');
                sink.put('r');
                break;
            default:
                sink.put(c);
                break;
        }
    }
}

void encode_field_value(Sink &sink, const FieldDescriptor &f,
                         const void *state) {
    const char *base = static_cast<const char *>(state) + f.offset;
    switch (f.type) {
        case FieldType::I8: {
            int8_t v;
            memcpy(&v, base, sizeof(v));
            encode_i64(sink, (long long)v);
            break;
        }
        case FieldType::U8: {
            uint8_t v;
            memcpy(&v, base, sizeof(v));
            encode_u64(sink, (unsigned long long)v);
            break;
        }
        case FieldType::I32: {
            int32_t v;
            memcpy(&v, base, sizeof(v));
            encode_i64(sink, (long long)v);
            break;
        }
        case FieldType::U32: {
            uint32_t v;
            memcpy(&v, base, sizeof(v));
            encode_u64(sink, (unsigned long long)v);
            break;
        }
        case FieldType::FLOAT: {
            float v;
            memcpy(&v, base, sizeof(v));
            encode_float(sink, v);
            break;
        }
        case FieldType::DOUBLE: {
            double v;
            memcpy(&v, base, sizeof(v));
            encode_double(sink, v);
            break;
        }
        case FieldType::BOOL: {
            bool v;
            memcpy(&v, base, sizeof(v));
            encode_bool(sink, v);
            break;
        }
        case FieldType::STRING: {
            encode_string(sink, base, f.size);
            break;
        }
    }
}

void encode_pass(Sink &sink, const FieldDescriptor *fields,
                  size_t field_count, const void *state) {
    for (size_t i = 0; i < field_count; i++) {
        const FieldDescriptor &f = fields[i];
        sink.put_cstr(f.key);
        sink.put('=');
        encode_field_value(sink, f, state);
        sink.put('\n');
    }
}

// ---------------------------------------------------------------------------
// Decoding
// ---------------------------------------------------------------------------

double clamp_range(double v, const FieldDescriptor &f) {
    if (!f.has_range) {
        return v;
    }
    if (v < f.min_value) {
        return f.min_value;
    }
    if (v > f.max_value) {
        return f.max_value;
    }
    return v;
}

// Longest legitimate numeric token this codec ever emits is the double
// worst case (sign + 17 significant digits in scientific form), well under
// 32 bytes; 63 leaves ample margin for a hand-edited file's stray padding
// while still rejecting truly bad input as too-long-to-be-a-number rather
// than reading past a fixed buffer.
constexpr size_t kNumTokenCap = 63;

bool copy_token(const char *value, size_t value_len, char (&tmp)[kNumTokenCap + 1]) {
    if (value_len > kNumTokenCap) {
        return false;
    }
    memcpy(tmp, value, value_len);
    tmp[value_len] = '\0';
    return true;
}

bool decode_signed(const char *value, size_t value_len, const FieldDescriptor &f,
                    long long native_min, long long native_max,
                    long long *out) {
    if (value_len == 0) {
        return false;
    }
    char tmp[kNumTokenCap + 1];
    if (!copy_token(value, value_len, tmp)) {
        return false;
    }
    errno = 0;
    char *end = nullptr;
    long long v = strtoll(tmp, &end, 10);
    if (end != tmp + value_len || errno == ERANGE) {
        return false;
    }
    if (f.has_range) {
        v = (long long)clamp_range((double)v, f);
    } else if (v < native_min || v > native_max) {
        return false;
    }
    *out = v;
    return true;
}

bool decode_unsigned(const char *value, size_t value_len, const FieldDescriptor &f,
                      unsigned long long native_max, unsigned long long *out) {
    if (value_len == 0 || value[0] == '-') {
        return false;
    }
    char tmp[kNumTokenCap + 1];
    if (!copy_token(value, value_len, tmp)) {
        return false;
    }
    errno = 0;
    char *end = nullptr;
    unsigned long long v = strtoull(tmp, &end, 10);
    if (end != tmp + value_len || errno == ERANGE) {
        return false;
    }
    if (f.has_range) {
        double clamped = clamp_range((double)v, f);
        v = (unsigned long long)clamped;
    } else if (v > native_max) {
        return false;
    }
    *out = v;
    return true;
}

bool decode_real(const char *value, size_t value_len, double *out) {
    if (value_len == 0) {
        return false;
    }
    char tmp[kNumTokenCap + 1];
    if (!copy_token(value, value_len, tmp)) {
        return false;
    }
    char *end = nullptr;
    double v = strtod(tmp, &end);
    if (end != tmp + value_len) {
        return false;
    }
    *out = v;
    return true;
}

bool decode_bool(const char *value, size_t value_len, bool *out) {
    if (value_len != 1) {
        return false;
    }
    if (value[0] == '0') {
        *out = false;
        return true;
    }
    if (value[0] == '1') {
        *out = true;
        return true;
    }
    return false;
}

// Validates + unescapes a STRING value into `out` (capacity `cap`, which is
// the declared char[N] buffer size, i.e. N -- room for content plus NUL).
// out may be nullptr for a dry-run length check. Returns false (rejects,
// per settings_store.h behaviour 4) on an invalid escape, a trailing lone
// backslash, or content that would not fit in cap-1 bytes -- never
// truncates. On success, when out != nullptr, the result is NUL-terminated.
bool unescape_string(const char *value, size_t value_len, size_t cap, char *out) {
    if (cap == 0) {
        return value_len == 0; // no room for even a NUL: only "" fits
    }
    size_t out_pos = 0;
    for (size_t i = 0; i < value_len; i++) {
        char c = value[i];
        char stored;
        if (c == '\\') {
            i++;
            if (i >= value_len) {
                return false; // trailing lone backslash
            }
            char esc = value[i];
            if (esc == '\\') {
                stored = '\\';
            } else if (esc == 'n') {
                stored = '\n';
            } else if (esc == 'r') {
                stored = '\r';
            } else {
                return false; // unrecognised escape
            }
        } else {
            stored = c;
        }
        if (out_pos >= cap - 1) {
            return false; // would not fit -- reject, never truncate
        }
        if (out != nullptr) {
            out[out_pos] = stored;
        }
        out_pos++;
    }
    if (out != nullptr) {
        out[out_pos] = '\0';
    }
    return true;
}

bool decode_field_value(const FieldDescriptor &f, const char *value,
                         size_t value_len, void *state) {
    char *base = static_cast<char *>(state) + f.offset;
    switch (f.type) {
        case FieldType::I8: {
            long long v;
            if (!decode_signed(value, value_len, f, -128LL, 127LL, &v)) return false;
            int8_t stored = (int8_t)v;
            memcpy(base, &stored, sizeof(stored));
            return true;
        }
        case FieldType::U8: {
            unsigned long long v;
            if (!decode_unsigned(value, value_len, f, 255ULL, &v)) return false;
            uint8_t stored = (uint8_t)v;
            memcpy(base, &stored, sizeof(stored));
            return true;
        }
        case FieldType::I32: {
            long long v;
            if (!decode_signed(value, value_len, f, -2147483648LL, 2147483647LL, &v)) return false;
            int32_t stored = (int32_t)v;
            memcpy(base, &stored, sizeof(stored));
            return true;
        }
        case FieldType::U32: {
            unsigned long long v;
            if (!decode_unsigned(value, value_len, f, 4294967295ULL, &v)) return false;
            uint32_t stored = (uint32_t)v;
            memcpy(base, &stored, sizeof(stored));
            return true;
        }
        case FieldType::FLOAT: {
            double v;
            if (!decode_real(value, value_len, &v)) return false;
            v = clamp_range(v, f);
            float stored = (float)v;
            memcpy(base, &stored, sizeof(stored));
            return true;
        }
        case FieldType::DOUBLE: {
            double v;
            if (!decode_real(value, value_len, &v)) return false;
            v = clamp_range(v, f);
            memcpy(base, &v, sizeof(v));
            return true;
        }
        case FieldType::BOOL: {
            bool v;
            if (!decode_bool(value, value_len, &v)) return false;
            memcpy(base, &v, sizeof(v));
            return true;
        }
        case FieldType::STRING: {
            // Dry-run first (out == nullptr): reject-without-mutating on any
            // failure, per settings_store.h's "malformed line never touches
            // state" guarantee.
            if (!unescape_string(value, value_len, f.size, nullptr)) return false;
            return unescape_string(value, value_len, f.size, base);
        }
    }
    return false;
}

void process_line(const FieldDescriptor *fields, size_t field_count,
                   void *state, const char *line, size_t line_len,
                   DecodeStats &stats) {
    size_t eq = 0;
    bool found_eq = false;
    for (size_t i = 0; i < line_len; i++) {
        if (line[i] == '=') {
            eq = i;
            found_eq = true;
            break;
        }
    }
    if (!found_eq) {
        stats.malformed_lines++;
        return;
    }
    const char *key = line;
    size_t key_len = eq;
    const char *value = line + eq + 1;
    size_t value_len = line_len - eq - 1;

    const FieldDescriptor *f = find_field(fields, field_count, key, key_len);
    if (f == nullptr) {
        stats.unknown_keys++;
        return;
    }
    if (decode_field_value(*f, value, value_len, state)) {
        stats.fields_set++;
    } else {
        stats.malformed_lines++;
    }
}

} // namespace

long encode(const FieldDescriptor *fields, size_t field_count,
            const void *state, char *out, size_t out_cap) {
    Sink counting{nullptr, 0, 0};
    encode_pass(counting, fields, field_count, state);
    if (counting.pos > out_cap) {
        return -1;
    }
    Sink writing{out, out_cap, 0};
    encode_pass(writing, fields, field_count, state);
    return (long)writing.pos;
}

DecodeStats decode(const FieldDescriptor *fields, size_t field_count,
                    void *state, const char *in, size_t in_len) {
    DecodeStats stats;
    size_t i = 0;
    while (i < in_len) {
        size_t j = i;
        while (j < in_len && in[j] != '\n') {
            j++;
        }
        size_t line_len = j - i;
        stats.lines_total++;
        process_line(fields, field_count, state, in + i, line_len, stats);
        i = (j < in_len) ? j + 1 : in_len;
    }
    return stats;
}

} // namespace settings_store
