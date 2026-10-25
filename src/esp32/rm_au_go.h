// rm_au_go.h -- RAM-only state of a remote "upd go" (RM1), shared by rm_au_esp32.cpp (arms it) and
// auTick() in esp32_main.cpp (hands over). Lost on reboot by design.
#ifndef RM_AU_GO_H
#define RM_AU_GO_H

#if defined(ESP32)
#include <stdint.h>

#define RM_AU_GO_DELAY_MS 8000u // same as kRebootDelayMs (rm_runtime.cpp): the reply leaves first

// Arms the go for `tag`; the handover is not allowed before nowMs + RM_AU_GO_DELAY_MS.
void rmAuGoArm(const char *tag, uint32_t nowMs);
// Disarms (download failed, handover done or hopeless).
void rmAuGoDisarm(void);
bool rmAuGoArmed(void);
// Copies the armed tag (char[24]) and tells whether the not-before time has passed at nowMs.
// False if not armed (tag untouched).
bool rmAuGoGet(char *tagOut, bool *delayPassed, uint32_t nowMs);

// The auto path's two download gates, one definition for auTick() and rmAuGo(): a clock valid for TLS
// (NTP, or GPS / RTC / NTP has set it since boot) and a phone connected (isPhoneReady == 1).
// Defined in rm_au_esp32.cpp on every ESP32 build.
void rmAuAutoGates(bool *timeValid, bool *phoneConnected);

// True if a handover for `tag` was already attempted this boot (esp32_main.cpp, s_auHandoverTag): a
// second one is a no-op there, so a go for it must be refused or disarmed instead.
bool auHandoverAttempted(const char *tag);

#endif
#endif
