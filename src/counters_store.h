/**
 * counters_store.h -- persistence for meshcom_settings.node_msgid, kept apart
 * from the settings store (D1-04 W3 step 4, operator decision 2026-09-13).
 *
 * node_msgid is the low 10 bits of every message id this node originates
 * (msgid_counter.h). It is state, not configuration: a settings restore, a
 * config import or a BLE settings write must never rewind it, and it reaches
 * flash once per kMsgIdPersistStep frames -- so it must not drag the whole
 * settings record (a ~4 kB keyed file on nRF52) to flash every time.
 *
 *   ESP32  Preferences namespace "Counters", key "node_msgid". First boot on
 *          this firmware falls back to the legacy "Credentials"/"node_msgid"
 *          key so a field node keeps its counter across the upgrade.
 *   nRF52  its own small file in InternalFS, written with the same
 *          temp-then-rename sequence as the settings file. First boot falls
 *          back to whatever init_flash() already loaded into
 *          meshcom_settings.node_msgid (the legacy blob carries it).
 *
 * Contract (both platforms):
 *   countersLoad()  -- reads the stored counter into meshcom_settings.node_msgid,
 *                      steps it with msgIdAfterLoad() and persists that
 *                      result BEFORE returning (msgid_counter.h explains why
 *                      the stepped value has to be on flash before the first
 *                      frame goes out). Called by init_flash() after the
 *                      settings load; never from a timer task.
 *   countersSave()  -- writes meshcom_settings.node_msgid. Called where
 *                      msgIdNeedsPersist() says so (loop_functions.cpp).
 *                      Returns false on a storage failure; the caller keeps
 *                      running, the next high-water mark retries.
 */
#ifndef COUNTERS_STORE_H
#define COUNTERS_STORE_H

#include <stdint.h>

void countersLoad();
bool countersSave();

/**
 * RM-05 (#1189): high-water mark of the authenticated remote-management counter (remote_cmd.h).
 * Same reasoning as node_msgid above, stricter: it is a replay defence, so it must never go DOWN
 * through a settings restore, a config import (it is not a settings field and never reaches the
 * config JSON) or a flash wipe of the settings record.
 *
 *   ESP32  Preferences namespace "Counters", key "rm_hwm" (UInt). The namespace survives
 *          clear_flash() on purpose, see esp32_flash.cpp.
 *   nRF52  two slot files /rm_hwm.a and /rm_hwm.b (decimal + LF), no rename: each save overwrites the
 *          slot with the smaller value, load = max (rename onto the file failed on DK5EN-90).
 *
 *   rmHwmLoad()     -- stored mark, 0 when absent or unreadable. Called once at boot (rmInit()).
 *   rmHwmSave(v)    -- writes v, false on a storage failure (the caller prints a marker, runs on
 *                      and retries with the next accepted command). Loop task only, never from
 *                      RX context.
 */
uint32_t rmHwmLoad();
bool rmHwmSave(uint32_t hwm);

/**
 * RM-09: counter of the OUTGOING side (rmSendCommand() in rm_runtime.h) -- the last counter this
 * node put into a sent RM1 command. Same storage and survival rules as the high-water mark:
 *
 *   ESP32  Preferences namespace "Counters", key "rm_snd" (UInt).
 *   nRF52  two slot files /rm_snd.a and /rm_snd.b, same no-rename scheme as rm_hwm.
 *
 *   rmSndLoad()     -- stored value, 0 when absent or unreadable. Called once at boot (rmInit()).
 *   rmSndSave(v)    -- false on a storage failure; rmSendCommand() then refuses to send. Loop task only.
 */
uint32_t rmSndLoad();
bool rmSndSave(uint32_t ctr);

#endif // COUNTERS_STORE_H
