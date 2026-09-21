// byte_fifo: Umsetzung. Was die Datei tut und warum, steht in byte_fifo.h.

#include "byte_fifo.h"

#include <string.h>

#if defined(NRF52_SERIES) && !defined(NATIVE_BUILD)
#include <FreeRTOS.h>
#include <task.h>
#define BF_LOCK()   taskENTER_CRITICAL()
#define BF_UNLOCK() taskEXIT_CRITICAL()
#else
#define BF_LOCK()   ((void)0)
#define BF_UNLOCK() ((void)0)
#endif

// Umbruch am Ringende: eine Kopie, hoechstens zwei memcpy.
static void bf_write_at(byte_fifo_t *f, uint16_t pos, const uint8_t *src, uint16_t n)
{
    uint16_t first = (uint16_t)(f->cap - pos);
    if (first > n)
        first = n;
    memcpy(f->buf + pos, src, first);
    if (n > first)
        memcpy(f->buf, src + first, (size_t)(n - first));
}

static void bf_read_at(const byte_fifo_t *f, uint16_t pos, uint8_t *dst, uint16_t n)
{
    uint16_t first = (uint16_t)(f->cap - pos);
    if (first > n)
        first = n;
    memcpy(dst, f->buf + pos, first);
    if (n > first)
        memcpy(dst + first, f->buf, (size_t)(n - first));
}

static inline uint16_t bf_adv(const byte_fifo_t *f, uint16_t pos, uint16_t n)
{
    uint32_t p = (uint32_t)pos + n;
    if (p >= f->cap)
        p -= f->cap;
    return (uint16_t)p;
}

void bf_reset(byte_fifo_t *f)
{
    BF_LOCK();
    f->head = f->tail = f->oldest = 0;
    f->used = f->frames = f->unread = 0;
    f->tail_gen++;
    f->evict_gen++;
    BF_UNLOCK();
}

int bf_push2(byte_fifo_t *f, const uint8_t *a, uint8_t alen, const uint8_t *b, uint8_t blen)
{
    uint16_t len = (uint16_t)alen + blen;
    if (len == 0 || len > 255 || (uint16_t)(len + 1) > f->cap)
        return -1;

    int lost = 0;
    BF_LOCK();

    // Platz schaffen: aelteste Frames weg, bis len+1 Byte frei sind.
    uint16_t need = (uint16_t)(len + 1);
    while ((uint16_t)(f->cap - f->used) < need)
    {
        uint8_t l = f->buf[f->oldest];
        bool was_unread = (f->frames == f->unread);
        f->oldest = bf_adv(f, f->oldest, (uint16_t)(1 + l));
        f->used = (uint16_t)(f->used - (1 + l));
        f->frames--;
        if (was_unread)
        {
            f->tail = f->oldest;
            f->unread--;
            f->tail_gen++;
            lost++;
        }
        f->evict_gen++;
    }

    f->buf[f->head] = (uint8_t)len;
    uint16_t p = bf_adv(f, f->head, 1);
    if (alen)
    {
        bf_write_at(f, p, a, alen);
        p = bf_adv(f, p, alen);
    }
    if (blen)
        bf_write_at(f, p, b, blen);

    f->head = bf_adv(f, f->head, need);
    f->used = (uint16_t)(f->used + need);
    f->frames++;
    f->unread++;

    BF_UNLOCK();
    return lost;
}

uint8_t bf_peek(byte_fifo_t *f, uint8_t *out, uint16_t outmax)
{
    uint8_t l = 0;
    BF_LOCK();
    if (f->unread)
    {
        l = f->buf[f->tail];
        uint16_t n = (l < outmax) ? l : outmax;
        if (n)
            bf_read_at(f, bf_adv(f, f->tail, 1), out, n);
    }
    BF_UNLOCK();
    return l;
}

void bf_pop(byte_fifo_t *f)
{
    BF_LOCK();
    if (f->unread)
    {
        uint8_t l = f->buf[f->tail];
        f->tail = bf_adv(f, f->tail, (uint16_t)(1 + l));
        f->unread--;
        f->tail_gen++;
    }
    BF_UNLOCK();
}

void bf_iter_begin(const byte_fifo_t *f, bf_iter_t *it)
{
    // Unter Sperre: sonst kann zwischen den drei Lesezugriffen eine
    // Verdraengung liegen, und der Iterator startet mit einem pos von VOR
    // und einem gen von NACH der Verdraengung. bf_iter_next() haelt das
    // fuer gueltig, liest ein beliebiges Byte als Laenge und liefert Muell,
    // bis left aufgebraucht ist (in-bounds, aber sichtbar auf der
    // Web-Nachrichtenseite).
    BF_LOCK();
    it->pos = f->oldest;
    it->left = f->frames;
    it->gen = f->evict_gen;
    BF_UNLOCK();
}

uint8_t bf_iter_next(byte_fifo_t *f, bf_iter_t *it, uint8_t *out, uint16_t outmax)
{
    uint8_t l = 0;
    BF_LOCK();
    if (it->left && it->gen == f->evict_gen)
    {
        l = f->buf[it->pos];
        uint16_t n = (l < outmax) ? l : outmax;
        if (n)
            bf_read_at(f, bf_adv(f, it->pos, 1), out, n);
        it->pos = bf_adv(f, it->pos, (uint16_t)(1 + l));
        it->left--;
    }
    else
    {
        it->left = 0;
    }
    BF_UNLOCK();
    return l;
}
