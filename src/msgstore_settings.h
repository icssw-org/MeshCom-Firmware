// msgstore_settings.h -- persistence for the store node (mailbox) settings.
//
// Owner C (docs/dm-stage3-wave-plan-20260914.md). T13
// (docs/dm-reliability-and-store-node-verdict-20260913.md:295): none of this
// touches struct s_meshcom_settings -- a FLASH_STRUCT_VERSION bump wipes
// every updating node's callsign/WLAN (src/configuration_global.h:102-108).
// Own NVS keys on ESP32 (same "Credentials" namespace esp32_flash.cpp uses,
// own Preferences handle), a small standalone LittleFS file on nRF52
// (BOARD_RAK4630) -- exactly the pattern documented at :113-118.
//
// Real bodies exist only where ENABLE_MSGSTORE is defined
// (configuration_global.h: ESP32-S3 and nRF52840 with the full buffer set).
// On every other board Load/Save are no-ops and Eligible() is false; callers
// never need their own #if for these three functions.
#pragma once

// True when this firmware was built with ENABLE_MSGSTORE (S3 / RAK4630).
// Compile-time fact, not a runtime probe -- safe to call from any board.
bool msgstoreHardwareEligible(void);

// Read the persisted store settings (clamped to the msgstore_api.h ranges,
// bad/missing values fall back to the documented defaults: off, 50 slots,
// 24 h, empty list) and apply them via msgstoreConfigure()/msgstoreSetList().
// Call once at boot, after msgstoreInit() (the platform mains' boot-time
// call site is applied by the orchestrator, not by this wave). No-op when
// !msgstoreHardwareEligible().
void msgstoreSettingsLoad(void);

// Persist the current core values (msgstoreMode()/Slots()/HoldHours()/
// ListCsv()) back to storage.
//
// nRF52: this writes to LittleFS, and LittleFS on this port is not safe to
// touch from a timer task (MEMORY.md: "no LittleFS access from a timer
// task"). msgstoreSettingsSave() is called only from commandAction(), which
// runs on the loop task -- keep it that way; never call it from an ISR, a
// FreeRTOS timer callback, or msgstoreLoop() itself.
void msgstoreSettingsSave(void);
