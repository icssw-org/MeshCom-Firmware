// TBEAM1W-PWR-01 (#1301): one place that sets the SX126x/LLCC68 PA, used by the boot path
// (esp32_main.cpp) and the runtime path (lora_setchip.cpp) so the two cannot drift apart.
//
// The second argument of RadioLib's setOutputPower() means "optimize" on SX1262/SX1268/LLCC68
// (true = paOptTable per dBm, false = datasheet PA: duty cycle 0x04, hpMax 0x07) but "forceRfo" on
// SX127x (false = default). Passing false is therefore the unchanged behaviour on SX127x.
// T-Deck Pro keeps the table (593495c1).
#pragma once

#include <stdint.h>

// T-Deck Pro pins RadioLib 7.1.2 (variants/t_deck_pro/platformio.ini): its SX1262 has only the
// one-argument setOutputPower(), which applies the paOptTable. Every other ESP32 board uses
// RadioLib 7.6.0 and gets the datasheet PA (optimize = false).

// table path: one-argument overload (T-Deck Pro, RadioLib 7.1.2)
template <typename Radio>
inline int16_t rfApplyOutputPowerTable(Radio &r, int8_t power)
{
    return r.setOutputPower(power);
}

// datasheet path: optimize = false on SX1262/SX1268/LLCC68, forceRfo = false on SX127x
template <typename Radio>
inline int16_t rfApplyOutputPowerDatasheet(Radio &r, int8_t power)
{
    return r.setOutputPower(power, false);
}

template <typename Radio>
inline int16_t rfApplyOutputPower(Radio &r, int8_t power)
{
#if defined(BOARD_T_DECK_PRO)
    return rfApplyOutputPowerTable(r, power);
#else
    return rfApplyOutputPowerDatasheet(r, power);
#endif
}
