// WX-01 (upstream #999): LTR-390UV-01 raw counts -> UV index, header-only, no Arduino types.
//
// Source: LiteOn LTR-390UV-01 datasheet, UVS mode: sensitivity 2300 counts per UVI at
// gain 18x and 20-bit resolution (400 ms integration time). Other settings scale linearly:
//   sensitivity = 2300 * (gain / 18) * (integration_ms / 400)
// Integration time halves per resolution bit below 20 (20 bit = 400 ms, 19 = 200, 18 = 100,
// 17 = 50, 16 = 25 ms), so integration_ms / 400 = 2^(bits - 20).
//   UVI = raw / sensitivity * wfac
// Only 16..20 bit are accepted (other values return 0): 13 bit is 12.5 ms (factor 1/32 on this
// part), 14 and 15 bit do not exist.
// wfac is the window factor: 1.0 without cover glass, > 1 behind a window that attenuates UV.
#pragma once

#include <stdint.h>
#include <stdbool.h>

#define LTR390_I2C_ADDR       0x53
#define LTR390_REG_MAIN_CTRL  0x00
#define LTR390_REG_MEAS_RATE  0x04
#define LTR390_REG_GAIN       0x05
#define LTR390_REG_PART_ID    0x06
#define LTR390_REG_MAIN_STATUS 0x07
#define LTR390_REG_UVS_DATA   0x10   // 3 bytes, LSB first

#define LTR390_CTRL_UVS_ACTIVE 0x0A  // bit 3 UVS mode, bit 1 enable
#define LTR390_STATUS_DATA_READY 0x08 // bit 3
#define LTR390_GAIN_18X       0x04
#define LTR390_MEAS_20BIT_500MS 0x04 // resolution 20 bit (bits 6:4 = 0), rate 500 ms (bits 2:0 = 4)
#define LTR390_RES_20BIT      20
#define LTR390_GAIN_VALUE_18X 18
#define LTR390_SENS_UVI_18X_20BIT 2300.0f
#define LTR390_PART_ID_UPPER  0x0B

static inline uint32_t ltr390Raw20(uint8_t b0, uint8_t b1, uint8_t b2)
{
	return (uint32_t)b0 | ((uint32_t)b1 << 8) | ((uint32_t)(b2 & 0x0F) << 16);
}

static inline bool ltr390PartIdOk(uint8_t reg)
{
	return (reg >> 4) == LTR390_PART_ID_UPPER;
}

static inline bool ltr390DataReady(uint8_t status)
{
	return (status & LTR390_STATUS_DATA_READY) != 0;
}

static inline float ltr390Uvi(uint32_t raw, uint8_t gain, uint8_t resolution_bits, float wfac)
{
	if (gain == 0 || resolution_bits < 16 || resolution_bits > 20)
		return 0.0f;

	// sensitivity = 2300 * (gain / 18) * 2^(bits - 20)
	float sens = LTR390_SENS_UVI_18X_20BIT * ((float)gain / (float)LTR390_GAIN_VALUE_18X);
	for (uint8_t b = resolution_bits; b < LTR390_RES_20BIT; b++)
		sens *= 0.5f;

	return ((float)raw / sens) * wfac;
}
