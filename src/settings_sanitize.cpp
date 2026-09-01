#include "settings_sanitize.h"
#include "maxhop.h"

#include <stdio.h>
#include <string.h>

static const int   POWER_NOT_SET = -20;
static const float FLOAT_NOT_SET = 0.0f;

static bool float_is_finite(float x)
{
    return x == x && x <= 3.4e38f && x >= -3.4e38f;
}

static bool float_near(float a, float b)
{
    float d = a - b;
    return d < 0.01f && d > -0.01f;
}

static void report(sanitize_log_fn log, const char *field, const char *oldv, const char *newv)
{
    if (log != NULL)
        log(field, oldv, newv);
}

static void report_int(sanitize_log_fn log, const char *field, int oldv, int newv)
{
    char a[16], b[16];
    snprintf(a, sizeof(a), "%d", oldv);
    snprintf(b, sizeof(b), "%d", newv);
    report(log, field, a, b);
}

static void report_float(sanitize_log_fn log, const char *field, float oldv, float newv)
{
    char a[32], b[32];
    snprintf(a, sizeof(a), "%g", (double)oldv);
    snprintf(b, sizeof(b), "%g", (double)newv);
    report(log, field, a, b);
}

int sanitize_radio_params(RadioParams &p, const RadioLimits &lim, sanitize_log_fn log)
{
    int fixed = 0;

    /* power: sentinel or within [min, max] */
    if (p.power != POWER_NOT_SET && (p.power < lim.power_min || p.power > lim.power_max))
    {
        report_int(log, "node_power", p.power, POWER_NOT_SET);
        p.power = POWER_NOT_SET;
        fixed++;
    }

    /* frequency: 0 (default) or inside the plausible band edges */
    if (!float_is_finite(p.freq) || (p.freq != FLOAT_NOT_SET && (p.freq < lim.freq_min || p.freq > lim.freq_max)))
    {
        report_float(log, "node_freq", p.freq, FLOAT_NOT_SET);
        p.freq = FLOAT_NOT_SET;
        fixed++;
    }

    /* bandwidth */
    {
        bool ok;
        if (!float_is_finite(p.bw))
            ok = false;
        else if (lim.bw_style == 0)
            ok = float_near(p.bw, 0.0f) || float_near(p.bw, 125.0f) || float_near(p.bw, 250.0f) || float_near(p.bw, 500.0f);
        else
            ok = float_near(p.bw, 0.0f) || float_near(p.bw, 1.0f) || float_near(p.bw, 2.0f);
        if (!ok)
        {
            report_float(log, "node_bw", p.bw, FLOAT_NOT_SET);
            p.bw = FLOAT_NOT_SET;
            fixed++;
        }
    }

    /* spreading factor: 0 or 6..12 */
    if (p.sf != 0 && (p.sf < 6 || p.sf > 12))
    {
        report_int(log, "node_sf", p.sf, 0);
        p.sf = 0;
        fixed++;
    }

    /* coding rate: 0 or 5..8 (ESP32) / 0..4 (nRF52 index) */
    {
        bool ok = (p.cr == 0) || (lim.cr_style == 0 ? (p.cr >= 5 && p.cr <= 8) : (p.cr >= 1 && p.cr <= 4));
        if (!ok)
        {
            report_int(log, "node_cr", p.cr, 0);
            p.cr = 0;
            fixed++;
        }
    }

    /* country: index into strCountry[] */
    if (p.country < 0 || p.country >= lim.country_count)
    {
        report_int(log, "node_country", p.country, 0);
        p.country = 0;
        fixed++;
    }

    return fixed;
}

bool sanitize_max_hop_text(int &v, sanitize_log_fn log)
{
    int fixed = maxHopTextSanitize(v);
    if (fixed == v)
        return false;

    report_int(log, "max_hop_text", v, fixed);
    v = fixed;
    return true;
}

bool sanitize_cstring(char *s, size_t n)
{
    if (s == NULL || n == 0)
        return false;
    if (memchr(s, 0, n) != NULL)
        return false;
    s[n - 1] = 0;
    return true;
}
