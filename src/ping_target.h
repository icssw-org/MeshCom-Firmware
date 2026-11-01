/**
 * Ping target rule: a ping target is a plain
 * callsign. No group, no special target ("*", "H", "HG", "TEST", ...), not the
 * node's own callsign. Only a node whose own call is the destination answers
 * {pong}, so everything else measures nothing and only draws gateway ACKs.
 *
 * One rule for every path: console/web setter, boot sanitize, nRF52 live BLE
 * write, sendPing() and manual {ping} text. Pure C++ without Arduino or the
 * Regexp library, so it links into every env that compiles settings_sanitize.cpp;
 * the callsign shape is the pattern of checkRegexCall() (regex_functions.cpp),
 * test_ping_target cross-checks both.
 */
#pragma once

#include <string.h>

// Callsign shape, same pattern as regex_call_match in regex_functions.cpp:
// ^[0-9A-Z]?[A-Z]?[0-9]+[A-Z][A-Z]?[A-Z]?[-]?[0-9]?[0-9]?$
// Tiny backtracking matcher over {class, min, max} items.
struct PingTargetItem { char cls; unsigned char mn, mx; };   // 'a' = 0-9A-Z, 'L' = A-Z, 'D' = 0-9, '-' = dash

static inline bool pingTargetClass(char cls, char c)
{
    switch (cls)
    {
        case 'a': return (c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z');
        case 'L': return c >= 'A' && c <= 'Z';
        case 'D': return c >= '0' && c <= '9';
        default:  return c == '-';
    }
}

static inline bool pingTargetShape(const char *s, const PingTargetItem *it, int n)
{
    if (n == 0)
        return *s == 0;
    for (int k = 0; k <= it->mx; k++)
    {
        // consume k chars of this item, then try the rest
        if (k > 0 && !pingTargetClass(it->cls, s[k - 1]))
            return false;                      // s[k-1] could not be consumed (also covers end of string)
        if (k >= it->mn && pingTargetShape(s + k, it + 1, n - 1))
            return true;
    }
    return false;
}

static inline bool pingTargetIsCallShape(const char *s)
{
    static const PingTargetItem items[] = {
        { 'a', 0, 1 }, { 'L', 0, 1 }, { 'D', 1, 20 }, { 'L', 1, 1 }, { 'L', 0, 1 },
        { 'L', 0, 1 }, { '-', 0, 1 }, { 'D', 0, 1 }, { 'D', 0, 1 } };
    return pingTargetShape(s, items, (int)(sizeof(items) / sizeof(items[0])));
}

enum PingTargetReason
{
    PING_TARGET_OK = 0,
    PING_TARGET_GROUP,
    PING_TARGET_SPECIAL,
    PING_TARGET_OWN,
    PING_TARGET_NOT_CALL
};

static inline const char *pingTargetReasonText(PingTargetReason r)
{
    switch (r)
    {
        case PING_TARGET_GROUP:   return "group";
        case PING_TARGET_SPECIAL: return "special target";
        case PING_TARGET_OWN:     return "own call";
        case PING_TARGET_NOT_CALL: return "not a callsign";
        default:                  return "ok";
    }
}

static inline PingTargetReason pingTargetCheck(const char *call, const char *own_call)
{
    if (call == NULL || call[0] == 0)
        return PING_TARGET_NOT_CALL;

    size_t n = strlen(call);

    // all digits: a group (CheckGroup() accepts a subset of this)
    bool all_digits = true;
    for (size_t i = 0; i < n; i++)
        if (call[i] < '0' || call[i] > '9')
        {
            all_digits = false;
            break;
        }
    if (all_digits)
        return PING_TARGET_GROUP;

    static const char *const special[] = { "*", "H", "HG", "BOT GATE", "TEST", "TESTER",
                                           "WLNK-1", "APRS2SOTA", "OE2YOTA-1" };
    for (size_t i = 0; i < sizeof(special) / sizeof(special[0]); i++)
        if (strcmp(call, special[i]) == 0)
            return PING_TARGET_SPECIAL;

    if (own_call != NULL && own_call[0] != 0 && strcmp(call, own_call) == 0)
        return PING_TARGET_OWN;

    if (!pingTargetIsCallShape(call))
        return PING_TARGET_NOT_CALL;

    // SSID: 1-2 digits, no leading zero (same rule as checkViaList())
    const char *dash = strchr(call, '-');
    if (dash != NULL)
    {
        size_t sl = strlen(dash + 1);
        if (sl < 1 || sl > 2)
            return PING_TARGET_NOT_CALL;
        if (sl == 2 && dash[1] == '0')
            return PING_TARGET_NOT_CALL;
    }

    return PING_TARGET_OK;
}

static inline bool pingTargetValid(const char *call, const char *own_call)
{
    return pingTargetCheck(call, own_call) == PING_TARGET_OK;
}
