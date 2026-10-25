// web_rm_parse.h -- pure parsing and validation of the Remote page POST bodies
//.
//
// Header-only, no Arduino, no allocation, C++11, only <stddef.h>/<string.h> and rm_validate.h.
// Three bodies: POST /rmpasswd, POST /rmnodes, POST /rmsend (dst or slot form).
//
// Rules shared by all three:
//   - The body is parsed IN PLACE: every returned `const char *` points into the caller's buffer,
//     which the caller wipes (rm_wipe) before it returns. Nothing is copied, nothing is static.
//   - application/x-www-form-urlencoded as the page sends it (encodeURIComponent): %XX is decoded,
//     '+' stays a literal '+', a '%' not followed by two hex digits stays a literal '%'.
//   - Rejected as `form`: a decoded control byte (< 0x20, 0x7F, so also %00), an empty pair, a pair
//     without '=', an empty key, an unknown key, a key given twice, a key that does not belong to the
//     chosen act/mode.
//   - Values are cut by no rule: an over-long value is rejected, never truncated.
//   - On error the result carries ONLY a static token (the closed set below); no input byte is ever
//     part of an error, and the value pointers of a failed parse point at a static empty string.
//
// Error tokens: "size" (body longer than RM_FORM_BODY_MAX), "form", "act", "slot", "call", "pw",
// "cmd" (cmd or args missing, over-long or not printable ASCII). The sentence of each token is the
// handler's `msg`, looked up in rmTokenTable (rm_sender_policy.h); test_rm_web_parse pins that every
// token has one.
#ifndef WEB_RM_PARSE_H
#define WEB_RM_PARSE_H

#include <stddef.h>
#include <string.h>

#include "../rm_validate.h"

#define RM_FORM_BODY_MAX 260 // the handlers' stack buffer is this + 1; dst form, every field %XX: 258
#define RM_FORM_CMD_MAX 15   // RmCmd::cmd[16]
#define RM_FORM_ARGS_MAX 39  // RmCmd::args[40] (RM_MAX_ARGS in remote_cmd.h; asserted in test_rm_web_parse)
#define RM_FORM_ACT_MAX 6    // "forget"
#define RM_FORM_SLOTS 3

#define RM_ERR_SIZE "size"
#define RM_ERR_FORM "form"
#define RM_ERR_ACT "act"
#define RM_ERR_SLOT "slot"
#define RM_ERR_CALL "call"
#define RM_ERR_PW "pw"
#define RM_ERR_CMD "cmd"

namespace rmparse
{

static inline const char *emptyStr() { return ""; }

static inline bool isHex(unsigned char c)
{
    return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
}

static inline unsigned hexVal(unsigned char c)
{
    return c <= '9' ? (unsigned)(c - '0') : (unsigned)((c | 0x20) - 'a' + 10);
}

// %XX in place ('+' stays literal); false on a control byte (also a decoded %00) or DEL.
static inline bool decode(char *s)
{
    char *w = s;
    for (const char *r = s; *r; r++)
    {
        unsigned char ch = (unsigned char)*r;
        if (ch == '%' && isHex((unsigned char)r[1]) && isHex((unsigned char)r[2]))
        {
            ch = (unsigned char)(hexVal((unsigned char)r[1]) * 16 + hexVal((unsigned char)r[2]));
            r += 2;
        }
        if (ch < 0x20 || ch == 0x7f)
            return false;
        *w++ = (char)ch;
    }
    *w = '\0';
    return true;
}

// Splits "k=v&k=v" in place. keys[0..nkeys) are the accepted names, vals[i] receives the decoded value
// of keys[i] (nullptr when absent). false = "form": empty body, empty pair, no '=', unknown or
// duplicate key, control byte. The body is modified (separators become NUL).
static inline bool split(char *body, const char *const *keys, size_t nkeys, char **vals)
{
    for (size_t i = 0; i < nkeys; i++)
        vals[i] = nullptr;
    if (body == nullptr || body[0] == '\0')
        return false;
    char *p = body;
    for (;;)
    {
        char *amp = strchr(p, '&');
        if (amp != nullptr)
            *amp = '\0';
        char *eq = strchr(p, '=');
        if (eq == nullptr || eq == p)
            return false; // pair without '=' or with an empty key (also an empty pair: "a=1&&b=2")
        *eq = '\0';
        char *val = eq + 1;
        size_t k = 0;
        while (k < nkeys && strcmp(p, keys[k]) != 0)
            k++;
        if (k == nkeys || vals[k] != nullptr)
            return false; // unknown or duplicate key
        if (!decode(val))
            return false;
        vals[k] = val;
        if (amp == nullptr)
            return true;
        p = amp + 1;
    }
}

static inline void lower(char *s)
{
    for (; *s; s++)
        if (*s >= 'A' && *s <= 'Z')
            *s = (char)(*s + ('a' - 'A'));
}

static inline void upper(char *s)
{
    for (; *s; s++)
        if (*s >= 'a' && *s <= 'z')
            *s = (char)(*s - ('a' - 'A'));
}

// exactly one digit '0'..'2'; anything else (empty, "3", "-1", "1a", "01", " 1") is no slot
static inline int slotOf(const char *s)
{
    if (s == nullptr || s[0] < '0' || s[0] >= (char)('0' + RM_FORM_SLOTS) || s[1] != '\0')
        return -1;
    return s[0] - '0';
}

// lower-case printable ASCII, no control byte; length 1..max (cmd) or 0..max (args)
static inline bool wireText(const char *s, size_t minLen, size_t maxLen)
{
    const size_t n = strlen(s);
    if (n < minLen || n > maxLen)
        return false;
    for (size_t i = 0; i < n; i++)
        if ((unsigned char)s[i] < 0x20 || (unsigned char)s[i] > 0x7e)
            return false;
    return true;
}

static inline bool bodyTooLong(const char *body)
{
    return body != nullptr && strlen(body) > (size_t)RM_FORM_BODY_MAX;
}

} // namespace rmparse

// ---------------------------------------------------------------------------------------------------
// POST /rmpasswd   act=set&pw=<pw>   |   act=clear

struct RmPwdReq
{
    const char *err; // nullptr = ok, else one static token
    bool clear;      // act=clear (pw is "")
    const char *pw;  // act=set: validated password inside the body buffer; else ""
};

static inline RmPwdReq rmParsePasswdBody(char *body)
{
    RmPwdReq r;
    r.err = nullptr;
    r.clear = false;
    r.pw = rmparse::emptyStr();
    if (rmparse::bodyTooLong(body))
    {
        r.err = RM_ERR_SIZE;
        return r;
    }
    static const char *const keys[] = {"act", "pw"};
    char *v[2];
    if (!rmparse::split(body, keys, 2, v))
    {
        r.err = RM_ERR_FORM;
        return r;
    }
    if (v[0] == nullptr || (strcmp(v[0], "set") != 0 && strcmp(v[0], "clear") != 0))
    {
        r.err = RM_ERR_ACT;
        return r;
    }
    if (strcmp(v[0], "clear") == 0)
    {
        if (v[1] != nullptr)
            r.err = RM_ERR_FORM; // a password has no place in a clear request
        else
            r.clear = true;
        return r;
    }
    if (v[1] == nullptr || !rmValidatePasswordN(v[1], strlen(v[1])))
    {
        r.err = RM_ERR_PW;
        return r;
    }
    r.pw = v[1];
    return r;
}

// ---------------------------------------------------------------------------------------------------
// POST /rmnodes   act=save&slot=0..2&call=<call>&pw=<pw>   |   act=del&slot=n   |   act=forget

enum RmNodesAct
{
    RMN_NONE = 0,
    RMN_SAVE,
    RMN_DEL,
    RMN_FORGET
};

struct RmNodesReq
{
    const char *err;  // nullptr = ok, else one static token
    RmNodesAct act;
    int slot;         // 0..2 for save/del, -1 for forget
    const char *call; // save: upper-cased, rmValidateCall() passed; else ""
    const char *pw;   // save: rmValidatePasswordN() passed; else ""
};

static inline RmNodesReq rmParseNodesBody(char *body)
{
    RmNodesReq r;
    r.err = nullptr;
    r.act = RMN_NONE;
    r.slot = -1;
    r.call = rmparse::emptyStr();
    r.pw = rmparse::emptyStr();
    if (rmparse::bodyTooLong(body))
    {
        r.err = RM_ERR_SIZE;
        return r;
    }
    static const char *const keys[] = {"act", "slot", "call", "pw"};
    char *v[4];
    if (!rmparse::split(body, keys, 4, v))
    {
        r.err = RM_ERR_FORM;
        return r;
    }
    RmNodesAct act = RMN_NONE;
    if (v[0] != nullptr)
    {
        if (strcmp(v[0], "save") == 0)
            act = RMN_SAVE;
        else if (strcmp(v[0], "del") == 0)
            act = RMN_DEL;
        else if (strcmp(v[0], "forget") == 0)
            act = RMN_FORGET;
    }
    if (act == RMN_NONE)
    {
        r.err = RM_ERR_ACT;
        return r;
    }
    // which keys belong to which act: anything else is a malformed request
    const bool wantSlot = act != RMN_FORGET;
    const bool wantCallPw = act == RMN_SAVE;
    if ((!wantSlot && v[1] != nullptr) || (!wantCallPw && (v[2] != nullptr || v[3] != nullptr)))
    {
        r.err = RM_ERR_FORM; // a key that does not belong to this act; a missing one gets its own token below
        return r;
    }
    if (wantSlot)
    {
        const int s = rmparse::slotOf(v[1]);
        if (s < 0)
        {
            r.err = RM_ERR_SLOT;
            return r;
        }
        r.slot = s;
    }
    if (wantCallPw)
    {
        if (v[2] == nullptr || strlen(v[2]) > RM_CALL_MAX)
        {
            r.err = RM_ERR_CALL;
            r.slot = -1;
            return r;
        }
        rmparse::upper(v[2]);
        if (!rmValidateCall(v[2]))
        {
            r.err = RM_ERR_CALL;
            r.slot = -1;
            return r;
        }
        if (v[3] == nullptr || !rmValidatePasswordN(v[3], strlen(v[3])))
        {
            r.err = RM_ERR_PW;
            r.slot = -1;
            return r;
        }
        r.call = v[2];
        r.pw = v[3];
    }
    r.act = act;
    return r;
}

// Case-insensitive call compare (the slot guard: a typed "dk5en-12" equals the saved "DK5EN-12").
static inline bool rmCallEq(const char *a, const char *b)
{
    if (a == nullptr || b == nullptr)
        return false;
    for (; *a != '\0' && *b != '\0'; a++, b++)
    {
        char x = *a, y = *b;
        if (x >= 'a' && x <= 'z')
            x = (char)(x - 'a' + 'A');
        if (y >= 'a' && y <= 'z')
            y = (char)(y - 'a' + 'A');
        if (x != y)
            return false;
    }
    return *a == *b;
}

// ---------------------------------------------------------------------------------------------------
// POST /rmsend   dst=<call>&pw=<pw>&cmd=<cmd>&args=<args>   |   slot=0..2&cmd=<cmd>&args=<args>
// (args optional in both; an optional force=1|0 may follow in both, absent = 0, any other value, a
// duplicate or an empty value is the "form" error). cmd is folded to lower case (the wire form,
// rmCommandAllowed() requires it); args reach the wire exactly as typed (names keep their capitals; the
// allowlist still refuses e.g. "gps ON"); dst is folded to upper case and must pass rmValidateCall(). The
// slot form takes an optional call=<CALL> (folded like dst): rmSendBySlot() refuses the send unless the
// saved slot holds exactly that call (page "Run again" guard); call with the dst form is a "form" error,
// an invalid call is the "call" error. The password of the dst
// form is NOT judged here: rmSendCommand() owns that rule (it strips trailing spaces first); it must
// only be present and free of control bytes. The allowlist is rmCommandAllowed()'s job.

struct RmSendReq
{
    const char *err;  // nullptr = ok, else one static token
    int slot;         // 0..2 slot form, -1 dst form
    const char *dst;  // dst form: upper case, rmValidateCall() passed; else ""
    const char *pw;   // dst form: as sent (non-empty); else ""
    const char *cmd;  // lower case, 1..15 printable bytes
    const char *args; // as typed, 0..39 printable bytes ("" when absent)
    bool force;       // optional field force=1 (true) | force=0 / absent (false)
    const char *call; // slot form guard: upper case, rmValidateCall() passed; "" when absent
};

static inline RmSendReq rmParseSendBody(char *body)
{
    RmSendReq r;
    r.err = nullptr;
    r.slot = -1;
    r.dst = rmparse::emptyStr();
    r.pw = rmparse::emptyStr();
    r.cmd = rmparse::emptyStr();
    r.args = rmparse::emptyStr();
    r.force = false;
    r.call = rmparse::emptyStr();
    if (rmparse::bodyTooLong(body))
    {
        r.err = RM_ERR_SIZE;
        return r;
    }
    static const char *const keys[] = {"slot", "dst", "pw", "cmd", "args", "force", "call"};
    char *v[7];
    if (!rmparse::split(body, keys, 7, v))
    {
        r.err = RM_ERR_FORM;
        return r;
    }
    if (v[5] != nullptr)
    {
        if (strcmp(v[5], "1") != 0 && strcmp(v[5], "0") != 0)
        {
            r.err = RM_ERR_FORM;
            return r;
        }
    }
    const bool slotForm = v[0] != nullptr;
    if (v[6] != nullptr && !slotForm)
    {
        // call is the slot form's guard; the dst form names its target itself
        r.err = RM_ERR_FORM;
        return r;
    }
    if (slotForm && (v[1] != nullptr || v[2] != nullptr))
    {
        // slot together with dst/pw: two forms in one request
        r.err = RM_ERR_FORM;
        return r;
    }
    if (slotForm)
    {
        const int s = rmparse::slotOf(v[0]);
        if (s < 0)
        {
            r.err = RM_ERR_SLOT;
            return r;
        }
        r.slot = s;
        if (v[6] != nullptr)
        {
            if (strlen(v[6]) > RM_CALL_MAX || (rmparse::upper(v[6]), !rmValidateCall(v[6])))
            {
                r.err = RM_ERR_CALL;
                r.slot = -1;
                return r;
            }
        }
    }
    else
    {
        if (v[1] == nullptr || strlen(v[1]) > RM_CALL_MAX)
        {
            r.err = RM_ERR_CALL;
            return r;
        }
        rmparse::upper(v[1]);
        if (!rmValidateCall(v[1]))
        {
            r.err = RM_ERR_CALL;
            return r;
        }
        if (v[2] == nullptr || v[2][0] == '\0')
        {
            r.err = RM_ERR_PW;
            return r;
        }
    }
    if (v[3] == nullptr || !rmparse::wireText(v[3], 1, RM_FORM_CMD_MAX))
    {
        r.err = RM_ERR_CMD;
        r.slot = -1;
        return r;
    }
    if (v[4] != nullptr && !rmparse::wireText(v[4], 0, RM_FORM_ARGS_MAX))
    {
        r.err = RM_ERR_CMD;
        r.slot = -1;
        return r;
    }
    rmparse::lower(v[3]); // cmd only: args stay as typed
    if (!slotForm)
    {
        r.dst = v[1];
        r.pw = v[2];
    }
    r.cmd = v[3];
    r.args = v[4] != nullptr ? v[4] : rmparse::emptyStr();
    r.force = v[5] != nullptr && v[5][0] == '1';
    if (v[6] != nullptr)
        r.call = v[6];
    return r;
}

#endif // WEB_RM_PARSE_H
