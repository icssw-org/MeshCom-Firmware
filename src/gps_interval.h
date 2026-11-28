#pragma once

// Position interval while GPS is switched on but gives no position: no module answers, or the module
// has no fix. The configured --postime applies, as with --gps off; without one the default interval.
// Before this, the fallback after ten empty polls was always POSINFO_INTERVAL (1800 s) and --postime
// was ignored (upstream issue #1327).
static inline unsigned int gpsNoFixInterval(int nodePostime, unsigned int defaultInterval)
{
    return nodePostime > 0 ? (unsigned int)nodePostime : defaultInterval;
}
