/**
 * @file        kbd_repeat.h
 * @brief       Pure state machine for T-Deck keyboard auto-repeat (raw I2C mode)
 * @license     MIT
 * @copyright   Copyright (c) 2025 ICSSW.org
 *
 * Header-only, no Arduino dependency: takes `now` (millis()) from the caller
 * and never touches I2C itself, so it builds and runs on the native test
 * target.
 */
#ifndef KBD_REPEAT_H
#define KBD_REPEAT_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define KBD_RAW_FRAME_LEN     5
#define KBD_RAW_TIMEOUT_MS    5000
#define KBD_RAW_PROBE_MAX     3
#define KBD_RAW_MAX_BITS      3      /* key + up to two modifiers */

enum kbd_raw_support { KBD_RAW_UNKNOWN = 0, KBD_RAW_YES, KBD_RAW_NO };

/* Why a hold poll ended -- so the caller's reason ladder does not have to
 * repeat the timeout arithmetic. */
enum kbd_hold { KBD_HOLD_ACTIVE = 0, KBD_HOLD_RELEASED, KBD_HOLD_TIMEOUT };

/* uint32_t members first, then the pointer, then the bytes: 20 B on the
 * 32-bit target, 24 B on a 64-bit host (checked by the native test). */
struct KbdRepeat {
    uint32_t since_ms;
    uint32_t key;          /* the remapped act_key being repeated */
    void    *focus;        /* lv_obj_t* focused at arm time (K5); NULL on the
                            * native target, which has no LVGL */
    uint8_t  mask[KBD_RAW_FRAME_LEN];
    uint8_t  support;      /* enum kbd_raw_support */
    uint8_t  probes;
    bool     active;
};

/* Physical matrix of the LilyGo T-Deck keyboard C3, transcribed from
 * examples/Keyboard_ESP32C3/Keyboard_ESP32C3.ino (setup(), keyboard[col][row]
 * and keyboard_symbol[col][row]).  Frame byte i is column i, bit r is row r.
 * `0` marks a cell that emits no character (SYM, ALT, shifts, Mic, Enter,
 * Backspace -- the last two are special-cased in kbdExpectedCell()). */
static const char KBD_BASE[5][7] = {
    { 'q', 'w',  0 , 'a',  0 , ' ',  0  },   /* col 0: q w SYM a ALT SPACE MIC   */
    { 'e', 's', 'd', 'p', 'x', 'z',  0  },   /* col 1: e s d p x z LSHIFT        */
    { 'r', 'g', 't',  0 , 'v', 'c', 'f' },   /* col 2: r g t RSHIFT v c f        */
    { 'u', 'h', 'y',  0 , 'b', 'n', 'j' },   /* col 3: u h y ENTER b n j         */
    { 'o', 'l', 'i',  0 , '$', 'm', 'k' },   /* col 4: o l i BACKSPACE $ m k     */
};

static const char KBD_SYM[5][7] = {
    { '#', '1',  0 , '*',  0 ,  0 , '0'  },  /* col 0 */
    { '2', '4', '5', '@', '8', '7',  0   },  /* col 1 */
    { '3', '/', '(',  0 , '?', '9', '6'  },  /* col 2 */
    { '_', ':', ')',  0 , '!', ',', ';'  },  /* col 3 */
    { '+', '"', '-',  0 ,  0 , '.', '\'' },  /* col 4 */
};

/* Maps a character byte as delivered by the keyboard MCU back to the matrix
 * cell that produced it, so an arm can verify that the key it is about to
 * repeat is the one actually held down.  Returns false for anything with no
 * physical cell (Enter, Alt combos, 0x00, non-ASCII). */
static inline bool kbdExpectedCell(uint8_t ch, uint8_t *col, uint8_t *row)
{
    if (ch == 0)
        return false;

    if (ch == 0x08)                  /* Backspace: no character in the table */
    {
        *col = 4; *row = 3;
        return true;
    }
    if (ch == ' ')
    {
        *col = 0; *row = 5;
        return true;
    }

    if (ch >= 'A' && ch <= 'Z')      /* shifted alpha sits on the base cell */
        ch = (uint8_t)(ch + ('a' - 'A'));

    for (uint8_t c = 0; c < 5; c++)
    {
        for (uint8_t r = 0; r < 7; r++)
        {
            if (KBD_BASE[c][r] != 0 && (uint8_t)KBD_BASE[c][r] == ch)
            {
                *col = c; *row = r;
                return true;
            }
        }
    }
    for (uint8_t c = 0; c < 5; c++)
    {
        for (uint8_t r = 0; r < 7; r++)
        {
            if (KBD_SYM[c][r] != 0 && (uint8_t)KBD_SYM[c][r] == ch)
            {
                *col = c; *row = r;
                return true;
            }
        }
    }
    return false;
}

/* Frame plausibility: 1..KBD_RAW_MAX_BITS bits set across all five bytes and
 * no byte above 0x7F -- only rows 0..6 exist, so bit 7 is never set by a
 * keyboard that understands raw mode.  0xFF bytes are what an old firmware's
 * idle-high SDA leaves behind, and are the caller's stand-in for a failed
 * I2C read. */
static inline bool kbdRawFrameValid(const uint8_t *f)
{
    int bits = 0;
    for (int i = 0; i < KBD_RAW_FRAME_LEN; i++)
    {
        if (f[i] > 0x7F)
            return false;
        for (uint8_t b = f[i]; b != 0; b >>= 1)
            bits += (b & 1);
    }
    return bits >= 1 && bits <= KBD_RAW_MAX_BITS;
}

/* First frame after a key: arm the window only when the frame is plausible
 * AND shows the very cell (col,row) that produced this key -- otherwise a
 * fast a->b transition would bind key 'a' to key 'b's bits and repeat 'a'
 * under 'b's finger.  Returns false when the frame disqualifies the arm.
 *
 * An all-zero frame does NOT count as a probe failure: the char can arrive
 * up to one LVGL poll plus one C3 matrix scan (roughly 65 ms) after the
 * physical press, so a quick tap is often already released by the time the
 * first raw frame is read, and a key-mode slave answers a 5-byte request
 * with five zeros as well.  Every other rejection (a byte above 0x7F, too
 * many bits, or the expected bit missing from a non-zero frame) is a real
 * probe failure; KBD_RAW_PROBE_MAX of them mark the keyboard unsupported
 * for this boot.  Once support is YES the verdict is final: later failures
 * only decline the arm (K3). */
static inline bool kbdRepeatArm(struct KbdRepeat *s, const uint8_t *f, uint32_t key,
                                uint8_t col, uint8_t row, uint32_t now)
{
    bool all_zero = true;
    for (int i = 0; i < KBD_RAW_FRAME_LEN; i++)
    {
        if (f[i] != 0)
        {
            all_zero = false;
            break;
        }
    }
    if (all_zero)
        return false;               /* released before the read -- not a probe */

    if (col < KBD_RAW_FRAME_LEN && row < 7
        && kbdRawFrameValid(f)
        && (f[col] & (uint8_t)(1u << row)) != 0)
    {
        memcpy(s->mask, f, KBD_RAW_FRAME_LEN);
        s->since_ms = now;
        s->key      = key;
        s->focus    = NULL;         /* caller fills this in after a successful arm */
        s->active   = true;
        s->support  = KBD_RAW_YES;
        s->probes   = 0;
        return true;
    }

    if (s->support != KBD_RAW_YES)
    {
        s->probes++;
        if (s->probes >= KBD_RAW_PROBE_MAX)
            s->support = KBD_RAW_NO;
    }
    return false;
}

/* Subsequent frames: KBD_HOLD_ACTIVE while every bit armed in `mask` is still
 * set in `f` (extra bits, e.g. a modifier added mid-hold, are accepted) and
 * the window has not timed out.  `now - since_ms` in uint32_t arithmetic is
 * the standard millis()-rollover-safe comparison. */
static inline enum kbd_hold kbdRepeatHold(struct KbdRepeat *s, const uint8_t *f, uint32_t now)
{
    if (!s->active)
        return KBD_HOLD_RELEASED;
    if ((uint32_t)(now - s->since_ms) >= KBD_RAW_TIMEOUT_MS)
        return KBD_HOLD_TIMEOUT;
    for (int i = 0; i < KBD_RAW_FRAME_LEN; i++)
    {
        if ((f[i] & s->mask[i]) != s->mask[i])
            return KBD_HOLD_RELEASED;
    }
    return KBD_HOLD_ACTIVE;
}

/* Ends the hold. `support`/`probes` are a per-boot verdict on the keyboard
 * firmware, not per-hold state -- left untouched here. */
static inline void kbdRepeatClear(struct KbdRepeat *s)
{
    s->active = false;
    memset(s->mask, 0, KBD_RAW_FRAME_LEN);
    s->key   = 0;
    s->focus = NULL;
}

#endif /* KBD_REPEAT_H */
