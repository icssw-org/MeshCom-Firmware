// daylight.h -- sun position, day/night decision and the T-Deck backlight policy (TD-23).
// Pure, header-only, no Arduino includes, so the host env native_daylight can pin it.
//
// The T-Deck Plus has no light sensor, so the backlight's day/night switch runs on the sun's
// elevation at the node position. sun_elevation_deg() is the NOAA low-precision solar position
// (Meeus, "Astronomical Algorithms" ch. 25, as used by the NOAA solar calculator): declination,
// equation of time and hour angle; well inside +-0.5 deg between 1950 and 2100.
#ifndef DAYLIGHT_H_
#define DAYLIGHT_H_

#include <stdint.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Day/night threshold for the sun's centre, geometric elevation (no refraction). Civil twilight:
// the screen keeps its day level until it is actually getting dark. -0.833 would switch at the
// geometric sunset instead.
#define DAYLIGHT_THRESHOLD_DEG (-6.0)
// Half-width of the hysteresis band around the threshold, so dusk does not flap.
#define DAYLIGHT_HYSTERESIS_DEG 1.0

static inline double daylight_rad(double deg) { return deg * (M_PI / 180.0); }
static inline double daylight_deg(double rad) { return rad * (180.0 / M_PI); }

// Sun elevation above the horizon in degrees at the given UTC time and position (lat north
// positive, lon east positive). No refraction correction.
static inline double sun_elevation_deg(uint32_t unix_utc, double lat_deg, double lon_deg)
{
    const double jd = (double)unix_utc / 86400.0 + 2440587.5;
    const double t = (jd - 2451545.0) / 36525.0; // Julian centuries since J2000

    const double l0 = fmod(280.46646 + t * (36000.76983 + t * 0.0003032), 360.0);
    const double m = 357.52911 + t * (35999.05029 - 0.0001537 * t);
    const double e = 0.016708634 - t * (0.000042037 + 0.0000001267 * t);
    const double mr = daylight_rad(m);
    const double c = sin(mr) * (1.914602 - t * (0.004817 + 0.000014 * t)) +
                     sin(2 * mr) * (0.019993 - 0.000101 * t) + sin(3 * mr) * 0.000289;
    const double omega = daylight_rad(125.04 - 1934.136 * t);
    const double lambda = daylight_rad(l0 + c - 0.00569 - 0.00478 * sin(omega));
    const double eps0 = 23.0 + (26.0 + (21.448 - t * (46.815 + t * (0.00059 - t * 0.001813))) / 60.0) / 60.0;
    const double eps = daylight_rad(eps0 + 0.00256 * cos(omega));
    const double decl = asin(sin(eps) * sin(lambda));

    // Equation of time in minutes.
    const double y = tan(eps / 2) * tan(eps / 2);
    const double l0r = daylight_rad(l0);
    const double eot = 4.0 * daylight_deg(y * sin(2 * l0r) - 2 * e * sin(mr) + 4 * e * y * sin(mr) * cos(2 * l0r) -
                                          0.5 * y * y * sin(4 * l0r) - 1.25 * e * e * sin(2 * mr));

    const double utc_min = (double)(unix_utc % 86400u) / 60.0;
    double tst = fmod(utc_min + eot + 4.0 * lon_deg, 1440.0); // true solar time, minutes
    if (tst < 0)
        tst += 1440.0;
    const double ha = daylight_rad(tst / 4.0 - 180.0);

    const double latr = daylight_rad(lat_deg);
    double cz = sin(latr) * sin(decl) + cos(latr) * cos(decl) * cos(ha);
    if (cz > 1.0)
        cz = 1.0;
    if (cz < -1.0)
        cz = -1.0;
    return 90.0 - daylight_deg(acos(cz));
}

// Day/night from an elevation, with hysteresis: a day stays day down to threshold - band, a night
// stays night up to threshold + band.
static inline bool daylight_is_day_elev(double elev_deg, bool was_day)
{
    const double limit = was_day ? DAYLIGHT_THRESHOLD_DEG - DAYLIGHT_HYSTERESIS_DEG
                                 : DAYLIGHT_THRESHOLD_DEG + DAYLIGHT_HYSTERESIS_DEG;
    return elev_deg > limit;
}

static inline bool daylight_is_day(uint32_t unix_utc, double lat, double lon, bool was_day)
{
    return daylight_is_day_elev(sun_elevation_deg(unix_utc, lat, lon), was_day);
}

// The stored own position is usable: hemisphere letters set and not the 0/0 "no position" value
// (same rule as compute_locator_from_settings() on the T-Deck), magnitudes in range.
static inline bool daylight_position_valid(char lat_c, char lon_c, double lat, double lon)
{
    if ((lat_c != 'N' && lat_c != 'S') || (lon_c != 'E' && lon_c != 'W'))
        return false;
    if (!(lat >= -90.0 && lat <= 90.0) || !(lon >= -180.0 && lon <= 180.0))
        return false;
    return !(lat == 0.0 && lon == 0.0);
}

// ---- Backlight policy ----

#define BL_LEVEL_MIN 1
#define BL_LEVEL_MAX 16
#define BL_DAY_DEFAULT 16
#define BL_NIGHT_DEFAULT 4

// Which saved level is active. FALLBACK (auto on, but no trusted clock or no position) uses the
// day level and never switches; it is its own value only so the log can say why.
enum BlPeriod
{
    BL_PERIOD_DAY = 0,
    BL_PERIOD_NIGHT,
    BL_PERIOD_FALLBACK
};

static inline BlPeriod brightness_period(bool auto_on, bool clock_ok, bool pos_ok, bool is_day)
{
    if (!auto_on)
        return BL_PERIOD_DAY;
    if (!clock_ok || !pos_ok)
        return BL_PERIOD_FALLBACK;
    return is_day ? BL_PERIOD_DAY : BL_PERIOD_NIGHT;
}

static inline const char *brightness_period_name(BlPeriod p)
{
    return p == BL_PERIOD_NIGHT ? "night" : (p == BL_PERIOD_FALLBACK ? "fallback" : "day");
}

// 1..16 is a valid stored level; 0 would mean "panel off" and is never stored.
static inline bool brightness_level_valid(int level)
{
    return level >= BL_LEVEL_MIN && level <= BL_LEVEL_MAX;
}

// The saved level of the period, with the default standing in for an invalid stored value.
static inline uint8_t brightness_level_for(BlPeriod p, int day, int night)
{
    if (p == BL_PERIOD_NIGHT)
        return (uint8_t)(brightness_level_valid(night) ? night : BL_NIGHT_DEFAULT);
    return (uint8_t)(brightness_level_valid(day) ? day : BL_DAY_DEFAULT);
}

// SYM+B writes into the slot of the active period: the night slot at night, the day slot
// otherwise (auto off and fallback included).
static inline void brightness_store_level(BlPeriod p, uint8_t level, int *day, int *night)
{
    if (p == BL_PERIOD_NIGHT)
        *night = level;
    else
        *day = level;
}

// Display mode of the T-Deck setup page: DAY (Light) = auto off, fixed day level; AUTO = day/night
// switching; NIGHT (Dark) = auto off, fixed night level. Stored as node_bl_auto (bool) plus
// node_bl_fnight (forced night); auto wins over a stale night flag, so old stores read as before.
enum BlMode
{
    BL_MODE_DAY = 0,
    BL_MODE_AUTO = 1,
    BL_MODE_NIGHT = 2
};

// Exact lower case "auto" | "day" | "night"; anything else is -1.
static inline int brightness_mode_parse(const char *arg)
{
    if (arg == NULL)
        return -1;
    static const char *const names[3] = {"day", "auto", "night"};
    for (int i = 0; i < 3; i++)
    {
        const char *a = arg;
        const char *n = names[i];
        while (*a && *n && *a == *n)
        {
            a++;
            n++;
        }
        if (*a == 0 && *n == 0)
            return i;
    }
    return -1;
}

static inline const char *brightness_mode_name(int m)
{
    return m == BL_MODE_AUTO ? "auto" : (m == BL_MODE_NIGHT ? "night" : "day");
}

static inline int brightness_mode_resolve(bool auto_on, uint8_t forced_night)
{
    if (auto_on)
        return BL_MODE_AUTO;
    return forced_night ? BL_MODE_NIGHT : BL_MODE_DAY;
}

static inline void brightness_mode_flags(int mode, bool *auto_on, uint8_t *forced_night)
{
    *auto_on = (mode == BL_MODE_AUTO);
    *forced_night = (mode == BL_MODE_NIGHT) ? 1 : 0;
}

// Setup page segments Auto | Light | Dark map to modes AUTO, DAY, NIGHT.
static inline int brightness_mode_from_seg(unsigned seg)
{
    return seg == 1 ? BL_MODE_DAY : (seg == 2 ? BL_MODE_NIGHT : BL_MODE_AUTO);
}

static inline unsigned brightness_mode_to_seg(int mode)
{
    return mode == BL_MODE_DAY ? 1 : (mode == BL_MODE_NIGHT ? 2 : 0);
}

// The period whose level is applied: AUTO follows the sun period, DAY and NIGHT force their own.
static inline BlPeriod brightness_period_for_mode(int mode, BlPeriod sun)
{
    if (mode == BL_MODE_NIGHT)
        return BL_PERIOD_NIGHT;
    if (mode == BL_MODE_DAY)
        return BL_PERIOD_DAY;
    return sun;
}


// SYM+B step table, descending, wraps 1 -> 16. A level off the table steps to the next lower
// entry (10 -> 8, 3 -> 2); 0 (panel off) and anything above 16 step to 16.
static inline uint8_t brightness_next_step(uint8_t cur)
{
    static const uint8_t steps[] = {16, 12, 8, 4, 2, 1};
    if (cur <= 1 || cur > BL_LEVEL_MAX)
        return BL_LEVEL_MAX;
    for (unsigned i = 0; i < sizeof(steps); i++)
    {
        if (steps[i] < cur)
            return steps[i];
    }
    return BL_LEVEL_MAX;
}

#endif // DAYLIGHT_H_
