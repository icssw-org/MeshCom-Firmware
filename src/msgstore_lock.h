// msgstore_lock.h -- internal critical-section macros of src/msgstore.cpp
//.
// Include from msgstore.cpp only: the ESP32 branch defines a file-static
// spinlock.
//
// Why: the slot table is written from two tasks. On nRF52 OnRxDone runs in
// the LORA task while the UDP GATE handlers (server ingress) and
// msgstoreLoop() run in the loop task; ESP32 boards with a separate lora
// task (T5, T-Deck-Pro) have the same shape. A hook (msgstoreStore,
// msgstoreOnAck, msgstorePresence, msgstoreOnPeerDelivery) therefore must
// not interleave with another hook or with the read-modify-write sections of
// msgstoreLoop(). The `gen` field only covers a hook landing DURING
// env->deliver(); this lock covers the table itself.
//
// Rules for code between MSGSTORE_LOCK() and MSGSTORE_UNLOCK():
//   - short and bounded (a few scans of <= MSGSTORE_SLOTS_MAX slots plus the
//     MSGSTORE_TOMB_SLOTS tombstone ring, one memcpy);
//   - no printf/printfdeb/Serial, no malloc, no flash access;
//   - no call into MsgStoreEnv (now_ms, own_call, random_between, deliver,
//     notify, log, ...): read the clock and draw random numbers BEFORE the
//     lock, call deliver()/notify() on a snapshot AFTER it;
//   - no nesting (the macros are not paired by a depth counter on firmware).
//
// Platforms:
//   NRF52_SERIES (not host)   taskENTER_CRITICAL()/taskEXIT_CRITICAL(), the
//                             convention of byte_fifo.cpp / uptime_min.h.
//   ESP32 (not host)          portENTER_CRITICAL(&mux) spinlock.
//   NATIVE_BUILD (host test)  counting no-op: msgstoreTestLock*() in
//                             msgstore_api.h lets the host test prove
//                             lock/unlock pairing and that no env callback
//                             runs inside the lock.
//   anything else             no-op.
#pragma once

#if defined(NATIVE_BUILD)

void msgstoreLockEnterCounted(void);   // defined in msgstore.cpp
void msgstoreLockExitCounted(void);
#define MSGSTORE_LOCK()   msgstoreLockEnterCounted()
#define MSGSTORE_UNLOCK() msgstoreLockExitCounted()

#elif defined(NRF52_SERIES)

#include <FreeRTOS.h>
#include <task.h>
#define MSGSTORE_LOCK()   taskENTER_CRITICAL()
#define MSGSTORE_UNLOCK() taskEXIT_CRITICAL()

#elif defined(ESP32) || defined(ARDUINO_ARCH_ESP32)

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
static portMUX_TYPE s_msgstore_mux = portMUX_INITIALIZER_UNLOCKED;
#define MSGSTORE_LOCK()   portENTER_CRITICAL(&s_msgstore_mux)
#define MSGSTORE_UNLOCK() portEXIT_CRITICAL(&s_msgstore_mux)

#else

#define MSGSTORE_LOCK()   ((void)0)
#define MSGSTORE_UNLOCK() ((void)0)

#endif
