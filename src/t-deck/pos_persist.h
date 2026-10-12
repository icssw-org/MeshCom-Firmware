#ifndef TDECK_POS_PERSIST_H
#define TDECK_POS_PERSIST_H

// Pure helpers for the T-Deck /pos.dat restore (TD-15). No Arduino includes, so
// the host test (test/test_pos_persist) can include this header directly.
//
// Record layout, 40 bytes per row, written by savePosPersistence():
//   10 B callsign   "%-10.10s"
//    6 B time       "%02i:%02i"
//   24 B position   "%.2lf%c/%.2lf%c/%i"  (lat, N/S, lon, E/W, altitude)
// The position field is the string tdeck_add_to_pos_view() puts into the POS
// table: latitude and longitude as magnitudes, the hemisphere in the letter.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>

// Copies at most `len` bytes of `src` into `out` (size `outsz`), stopping at the
// first NUL and dropping leading and trailing blanks. Always NUL-terminates.
static inline void tdeck_pos_trim(const char *src, size_t len, char *out, size_t outsz)
{
    if (outsz == 0)
        return;

    size_t n = 0;
    while (n < len && src[n] != '\0')
        n++;

    size_t b = 0;
    while (b < n && (src[b] == ' ' || src[b] == '\t' || src[b] == '\r' || src[b] == '\n'))
        b++;

    while (n > b && (src[n - 1] == ' ' || src[n - 1] == '\t' || src[n - 1] == '\r' || src[n - 1] == '\n'))
        n--;

    size_t m = n - b;
    if (m > outsz - 1)
        m = outsz - 1;

    memcpy(out, src + b, m);
    out[m] = '\0';
}

// Reads one unsigned decimal ("48.20", "7", "0.5") at *p and advances *p.
// strtod() is not used on the raw field: "16.37E/..." would let it look for an
// exponent after the hemisphere letter E.
static inline bool tdeck_pos_number(const char *&p, double &v)
{
    char num[16];
    size_t n = 0;
    bool dot = false;

    while (*p != '\0' && n < sizeof(num) - 1)
    {
        if (*p >= '0' && *p <= '9')
            num[n++] = *p++;
        else if (*p == '.' && !dot)
        {
            dot = true;
            num[n++] = *p++;
        }
        else
            break;
    }

    num[n] = '\0';
    if (n == 0 || (n == 1 && dot))
        return false;

    v = atof(num);
    return true;
}

// Parses the position field "48.20N/16.37E/200". The altitude is optional (a
// truncated or hand-edited row still yields a map point) and defaults to 0.
// lat/lon are returned as magnitudes, the sign lives in lat_c ('N'/'S') and
// lon_c ('E'/'W'), exactly as tdeck_add_pos_point() expects them. A leading
// minus sign on a number is tolerated and dropped. Returns false for anything
// that is not a plausible coordinate pair.
static inline bool tdeck_parse_pos_field(const char *field, double &lat, char &lat_c,
                                         double &lon, char &lon_c, int &alt)
{
    if (field == nullptr)
        return false;

    const char *p = field;
    while (*p == ' ')
        p++;

    double la = 0.0, lo = 0.0;

    if (*p == '-')
        p++;
    if (!tdeck_pos_number(p, la))
        return false;
    char lac = *p++;
    if (lac != 'N' && lac != 'S')
        return false;
    if (*p++ != '/')
        return false;

    if (*p == '-')
        p++;
    if (!tdeck_pos_number(p, lo))
        return false;
    char loc = *p++;
    if (loc != 'E' && loc != 'W')
        return false;

    if (la > 90.0 || lo > 180.0)
        return false;

    int al = 0;
    if (*p == '/')
    {
        p++;
        char *end = nullptr;
        long v = strtol(p, &end, 10);
        if (end != p)
            al = (int)v;
    }

    lat = la;
    lat_c = lac;
    lon = lo;
    lon_c = loc;
    alt = al;

    return true;
}

#endif // TDECK_POS_PERSIST_H
