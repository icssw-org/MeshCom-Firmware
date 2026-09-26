#pragma once

// C3 carve-out (DRY unification U3): checkSerialCommand() used to sit in the
// two main translation units, which are the largest in the tree and will never
// compile on a host. That, not the Serial object, is what kept U3 off the
// native side -- the test plan's C3 row assumed a Serial stub was the only
// obstacle and that no carve was needed.
//
// Each platform keeps its own copy in its own file
// (esp32/serial_command_esp32.cpp, nrf52/serial_command_nrf52.cpp) rather than
// being merged here: merging is unification, and which of the two copies is
// right is a drift-matrix decision that has not been taken. What the carve
// buys is that each can be built into a native binary at all and fed the same
// byte scripts (test_serial_command_twin, N1).
//
// NOT one binary, unlike the U1/U2 twins: both copies define the same symbol,
// `void checkSerialCommand(void)`, so they collide at link. The twin is built
// twice (env native_serial_esp32 / native_serial_nrf52) and compared through
// committed per-side baselines -- the U6 country-twin arrangement. An earlier
// version of this comment claimed a single binary was possible; it is not.
//
// The two differ in exactly two ways today, both preserved:
//   - ESP32 also reads the net console (nRF52 has none)
//   - nRF52 holds the 600 B msg_buffer in BSS (N-22, a loop-task stack fix);
//     ESP32 still has it on the stack
void checkSerialCommand(void);
