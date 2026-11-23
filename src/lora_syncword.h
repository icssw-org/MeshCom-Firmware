// LoRa sync word: one-byte form (SX127x / RadioLib setSyncWord()) -> two-byte SX126x register form.
//
// SX127x store the sync word as one byte (RegSyncWord 0x39). SX126x use two registers
// (0x0740/0x0741), one per nibble, each nibble followed by the control nibble 0x4:
//   0xXY -> 0xX4Y4   (0x2b -> 0x24b4, 0x12 -> 0x1424, 0x34 -> 0x3444)
// This is the conversion RadioLib's SX126x::setSyncWord() applies on the ESP32 boards.
// SX126x-Arduino (nRF52 boards) writes Radio.SetCustomSyncWord() raw to 0x0740/0x0741,
// so the nRF52 code has to pass the converted value to stay on air with the ESP32 nodes.
#pragma once

#include <stdint.h>

static constexpr uint16_t loraSyncWordSx126x(uint8_t syncWord)
{
    return (uint16_t)(((syncWord & 0xF0) << 8) | 0x0400 | ((syncWord & 0x0F) << 4) | 0x04);
}

static_assert(loraSyncWordSx126x(0x2b) == 0x24b4, "MeshCom sync word conversion");
static_assert(loraSyncWordSx126x(0x12) == 0x1424, "LoRa APRS / private sync word conversion");
static_assert(loraSyncWordSx126x(0x34) == 0x3444, "LoRaWAN public sync word conversion");
