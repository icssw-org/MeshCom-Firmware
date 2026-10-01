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

void countersLoad();
bool countersSave();

#endif // COUNTERS_STORE_H
