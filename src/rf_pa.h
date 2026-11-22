// TBEAM1W-PWR-01 (#1301): one place that sets the SX126x/LLCC68 PA, used by the boot path
// (esp32_main.cpp) and the runtime path (lora_setchip.cpp) so the two cannot drift apart.
//
// The second argument of RadioLib's setOutputPower() means "optimize" on SX1262/SX1268/LLCC68
// (true = paOptTable per dBm, false = datasheet PA: duty cycle 0x04, hpMax 0x07) but "forceRfo" on
// SX127x (false = default). Passing false is therefore the unchanged behaviour on SX127x.
// T-Deck Pro keeps the table (593495c1).
#pragma once

#include <stdint.h>

#ifndef RF_PA_OPTIMIZE
#if defined(BOARD_T_DECK_PRO)
#define RF_PA_OPTIMIZE true
#else
#define RF_PA_OPTIMIZE false
#endif
#endif

template <typename Radio>
inline int16_t rfApplyOutputPower(Radio &r, int8_t power, bool optimize = RF_PA_OPTIMIZE)
{
    return r.setOutputPower(power, optimize);
}
