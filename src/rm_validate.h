// rm_validate.h -- pure validators for the RM web GUI.
//
// Header-only, no Arduino, no allocation. One definition of "a password RM can use" and "a call RM
// can address", shared by the --passwd console command, the web Set field, the known-node slot save
// and the sender (rm_runtime.cpp). Reject, never truncate, never normalise: the caller folds a call
// to upper case BEFORE validating, and trims nothing out of a password.
//
// Why the password rule exists (verdict-protocol M-1): lora_functions.cpp ignores RM1 when
// node_passwd[0] == ' ', while the status page counted "any non-space" as set. A password such as
// " x" therefore showed "password set" but RM silently never ran. A leading space is rejected here.
#ifndef RM_VALIDATE_H
#define RM_VALIDATE_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define RM_PASSWD_MAX 14  // node_passwd is char[15], space padded

enum RmPasswdProblem : uint8_t
{
    RM_PW_OK = 0,
    RM_PW_EMPTY,          // null or length 0
    RM_PW_TOO_LONG,       // more than 14 bytes (UTF-8 counts in bytes)
    RM_PW_BAD_CHAR,       // outside printable ASCII 0x20..0x7E (control byte, DEL, UTF-8)
    RM_PW_LEADING_SPACE,  // first byte is a space (RM would be silently dead, see above)
    RM_PW_TRAILING_SPACE, // last byte is a space (every key derivation strips it: not what was typed)
    RM_PW_RESERVED        // exactly "none": --passwd none clears the password
};

// Checks the first len bytes of pw (len, not NUL, ends the password; pw may be null only for len 0).
inline RmPasswdProblem rmPasswordProblemN(const char *pw, size_t len)
{
    if (pw == nullptr || len == 0)
        return RM_PW_EMPTY;
    if (len > RM_PASSWD_MAX)
        return RM_PW_TOO_LONG;
    for (size_t i = 0; i < len; i++)
    {
        const unsigned char ch = (unsigned char)pw[i];
        if (ch < 0x20 || ch > 0x7E)
            return RM_PW_BAD_CHAR;
    }
    if (pw[0] == ' ')
        return RM_PW_LEADING_SPACE;
    if (pw[len - 1] == ' ')
        return RM_PW_TRAILING_SPACE;
    if (len == 4 && memcmp(pw, "none", 4) == 0)
        return RM_PW_RESERVED;
    return RM_PW_OK;
}

inline RmPasswdProblem rmPasswordProblem(const char *pw)
{
    return rmPasswordProblemN(pw, pw != nullptr ? strlen(pw) : 0);
}

// 1..14 bytes, printable ASCII (inner spaces allowed), no leading or trailing space, not "none".
inline bool rmValidatePassword(const char *pw)
{
    return rmPasswordProblem(pw) == RM_PW_OK;
}

inline bool rmValidatePasswordN(const char *pw, size_t len)
{
    return rmPasswordProblemN(pw, len) == RM_PW_OK;
}

// One plain sentence per problem (operator-facing, also for the console).
inline const char *rmPasswordProblemText(RmPasswdProblem p)
{
    switch (p)
    {
    case RM_PW_OK:             return "The password is fine.";
    case RM_PW_EMPTY:          return "Enter a password.";
    case RM_PW_TOO_LONG:       return "The password can be at most 14 characters.";
    case RM_PW_BAD_CHAR:       return "Use only plain letters, digits and symbols (no umlauts or special characters).";
    case RM_PW_LEADING_SPACE:  return "The password must not start with a space.";
    case RM_PW_TRAILING_SPACE: return "The password must not end with a space.";
    case RM_PW_RESERVED:       return "The word none is reserved: it clears the password. Choose another one.";
    }
    return "The password is not valid.";
}

#define RM_CALL_MAX 9  // call incl. SSID, e.g. "DK5EN-12" (8) or "OE1ABC-15" (9)

// CR-05: the ONE upper-casing of a typed call (ASCII a-z only, nothing else changes) into out, which must
// hold RM_CALL_MAX + 1 bytes. false for nullptr or a call longer than RM_CALL_MAX; out is then "". Never
// truncates: a truncated call would address (or forget) a DIFFERENT node. The caller reports "dst".
// Whether the result is a valid call is rmValidateCall()'s business.
inline bool rmFoldCall(const char *dst, char *out)
{
    out[0] = '\0';
    if (dst == nullptr)
        return false;
    size_t i = 0;
    for (; dst[i] != '\0'; i++)
    {
        if (i >= RM_CALL_MAX)
        {
            out[0] = '\0';
            return false;
        }
        const char ch = dst[i];
        out[i] = (ch >= 'a' && ch <= 'z') ? (char)(ch - 'a' + 'A') : ch;
    }
    out[i] = '\0';
    return true;
}

// A call with SSID, already upper case: 2..7 characters of [A-Z0-9], '-', SSID of 1 or 2 digits,
// 9 characters in total at most. Same character set as the sender (rm_runtime.cpp: [A-Z0-9-]) and
// the host tool.
// Does not compare with the own call; that is the caller's business. Lower case is rejected.
inline bool rmValidateCall(const char *call)
{
    if (call == nullptr)
        return false;
    size_t len = 0;
    while (call[len] != '\0' && len <= RM_CALL_MAX)
        len++;
    if (len > RM_CALL_MAX)
        return false;
    // find the last '-': everything before is the base, everything after the SSID
    size_t dash = len;
    for (size_t i = 0; i < len; i++)
        if (call[i] == '-')
            dash = i;
    if (dash == len)
        return false; // no SSID
    const size_t base = dash;
    const size_t ssid = len - dash - 1;
    if (base < 2 || ssid < 1 || ssid > 2)
        return false;
    for (size_t i = 0; i < base; i++)
    {
        const char ch = call[i];
        if (!((ch >= 'A' && ch <= 'Z') || (ch >= '0' && ch <= '9')))
            return false;
    }
    for (size_t i = dash + 1; i < len; i++)
        if (call[i] < '0' || call[i] > '9')
            return false;
    return true;
}

#endif // RM_VALIDATE_H
