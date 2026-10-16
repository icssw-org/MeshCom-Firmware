// Pure logic for the T-Deck DM target prefill and reply button (no LVGL,
// no Arduino): which callsign goes into the compose "To" field.
#pragma once

#include <string>
#include <string.h>
#include <ctype.h>

// Max length of the dm_callsign textarea (lv_textarea_set_max_length).
#define DM_FIELD_MAX 9

static inline std::string dmTrim(const std::string &s)
{
    size_t b = 0, e = s.size();
    while (b < e && isspace((unsigned char)s[b])) b++;
    while (e > b && isspace((unsigned char)s[e - 1])) e--;
    return s.substr(b, e - b);
}

static inline bool dmEqualNoCase(const std::string &a, const std::string &b)
{
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); i++)
        if (tolower((unsigned char)a[i]) != tolower((unsigned char)b[i])) return false;
    return true;
}

// "SRC1,SRC2 -> DST" -> "SRC1": left of "->", first comma token, trimmed.
static inline std::string dmSenderFromHeader(const char *header)
{
    std::string left = header ? header : "";
    size_t arrow = left.find("->");
    if (arrow != std::string::npos) left = left.substr(0, arrow);
    size_t comma = left.find(',');
    if (comma != std::string::npos) left = left.substr(0, comma);
    return dmTrim(left);
}

// Conversation tab name -> value for the "To" field ("" = no target).
static inline std::string dmTargetForGroup(const char *group, const char *ownCall)
{
    std::string g = dmTrim(group ? group : "");
    if (g.empty() || g == "*" || g == "MSG") return "";
    if (g.size() > DM_FIELD_MAX) return "";
    // callsign or numeric group only (also rejects "booting..." system tab)
    for (size_t i = 0; i < g.size(); i++)
        if (!isalnum((unsigned char)g[i]) && g[i] != '-') return "";
    if (ownCall && dmEqualNoCase(g, dmTrim(ownCall))) return "";
    return g;
}

struct DmPrefill
{
    bool write;
    std::string value;
};

// Overwrite only if the user did not touch the field: empty, or still the
// value the prefill itself wrote last. lastAuto is reset by explicit writers.
static inline DmPrefill dmPrefillDecide(const char *field, const char *lastAuto,
                                        const char *activeGroup, const char *ownCall)
{
    std::string f = field ? field : "";
    std::string la = lastAuto ? lastAuto : "";
    DmPrefill r;
    r.value = dmTargetForGroup(activeGroup, ownCall);
    r.write = (f.empty() || (!la.empty() && f == la)) && f != r.value;
    return r;
}
