// byte_fifo: Umsetzung. Was die Datei tut und warum, steht in byte_fifo.h.

#include "byte_fifo.h"

#include <string.h>

#if defined(NRF52_SERIES) && !defined(NATIVE_BUILD)
#include <FreeRTOS.h>
#include <task.h>
#define BF_LOCK()   taskENTER_CRITICAL()
#define BF_UNLOCK() taskEXIT_CRITICAL()
#elif defined(ARDUINO_ARCH_ESP32) && !defined(NATIVE_BUILD)
// ESP32: producers (loop task, LoRa task, web, BLE) and the AU job task (bf_detach, core 1
// prio 1 next to the loop task) share the rings. A spinlock critical section is enough: every
// section below is a few index updates and at most one 255 B memcpy (two with wrap), nothing
// inside allocates, logs, blocks or calls out.
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
static portMUX_TYPE s_bfMux = portMUX_INITIALIZER_UNLOCKED;
#define BF_LOCK()   portENTER_CRITICAL(&s_bfMux)
#define BF_UNLOCK() portEXIT_CRITICAL(&s_bfMux)
#elif defined(BF_TEST_LOCK_HOOK)
// Nur im Host-Test (env:native_byte_fifo): die Suite zaehlt mit, welche
// Funktion die Sperre nimmt.
void bf_test_lock(void);
void bf_test_unlock(void);
#define BF_LOCK()   bf_test_lock()
#define BF_UNLOCK() bf_test_unlock()
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
    if (len == 0 || len > 255)
        return -1;

    int lost = 0;
    BF_LOCK();

    // cap is read under the lock: bf_detach() on another task sets it to 0 (and buf to NULL)
    // under the same lock, so a push either completes on the old storage or drops.
    if ((uint16_t)(len + 1) > f->cap)
    {
        BF_UNLOCK();
        return -1;
    }

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
    if (f->cap && f->unread)
    {
        l = f->buf[f->tail];
        uint16_t n = (l < outmax) ? l : outmax;
        if (n)
            bf_read_at(f, bf_adv(f, f->tail, 1), out, n);
    }
    BF_UNLOCK();
    return l;
}

uint8_t bf_peek_gen(byte_fifo_t *f, uint8_t *out, uint16_t outmax, uint16_t *gen)
{
    uint8_t l = 0;
    BF_LOCK();
    if (f->cap && f->unread)
    {
        l = f->buf[f->tail];
        uint16_t n = (l < outmax) ? l : outmax;
        if (n)
            bf_read_at(f, bf_adv(f, f->tail, 1), out, n);
    }
    *gen = f->tail_gen;
    BF_UNLOCK();
    return l;
}

bool bf_pop_if(byte_fifo_t *f, uint16_t gen)
{
    bool popped = false;
    BF_LOCK();
    if (f->cap && f->unread && f->tail_gen == gen)
    {
        uint8_t l = f->buf[f->tail];
        f->tail = bf_adv(f, f->tail, (uint16_t)(1 + l));
        f->unread--;
        f->tail_gen++;
        popped = true;
    }
    BF_UNLOCK();
    return popped;
}

void bf_pop(byte_fifo_t *f)
{
    BF_LOCK();
    if (f->cap && f->unread)
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
    // Web-Nachrichtenseite). Aus #1157 (f1c5b14f) nachgezogen.
    BF_LOCK();
    it->pos = f->oldest;
    it->left = f->cap ? f->frames : 0;
    it->gen = f->evict_gen;
    BF_UNLOCK();
}

uint8_t bf_iter_next(byte_fifo_t *f, bf_iter_t *it, uint8_t *out, uint16_t outmax)
{
    uint8_t l = 0;
    BF_LOCK();
    if (f->cap && it->left && it->gen == f->evict_gen)
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

uint8_t *bf_detach(byte_fifo_t *f)
{
    BF_LOCK();
    uint8_t *old = f->buf;
    f->cap = 0;
    f->buf = NULL;
    f->head = f->tail = f->oldest = 0;
    f->used = f->frames = f->unread = 0;
    f->tail_gen++;
    f->evict_gen++;
    BF_UNLOCK();
    return old;
}

void bf_attach(byte_fifo_t *f, uint8_t *storage, uint16_t cap)
{
    BF_LOCK();
    f->head = f->tail = f->oldest = 0;
    f->used = f->frames = f->unread = 0;
    f->tail_gen++;
    f->evict_gen++;
    f->buf = storage;
    f->cap = cap;
    BF_UNLOCK();
}

bool bf_attach_all(byte_fifo_t *const *rings, const uint16_t *caps, uint8_t n, bf_alloc_fn alloc,
                   bf_free_fn release)
{
    uint8_t *mem[BF_ATTACH_MAX] = {0};
    if (n > BF_ATTACH_MAX)
        return false;

    bool ok = true;
    for (uint8_t i = 0; i < n; i++)
    {
        if (rings[i]->cap != 0)
            continue; // already attached
        mem[i] = (uint8_t *)alloc(caps[i]);
        if (mem[i] == NULL)
        {
            ok = false;
            break;
        }
    }
    if (!ok)
    {
        for (uint8_t i = 0; i < n; i++)
            if (mem[i])
                release(mem[i]); // all or nothing
        return false;
    }
    for (uint8_t i = 0; i < n; i++)
        if (mem[i])
            bf_attach(rings[i], mem[i], caps[i]);
    return true;
}

void bf_detach_free(byte_fifo_t *const *rings, uint8_t n, bf_free_fn release)
{
    for (uint8_t i = 0; i < n; i++)
    {
        uint8_t *old = bf_detach(rings[i]);
        if (old)
            release(old); // detach ran under the ring lock: no accessor holds old any more
    }
}
