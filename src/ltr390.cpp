// WX-01 (upstream #999): minimal LTR-390UV-01 UV driver. UVS mode, gain 18x, 20-bit, 500 ms.
#ifdef ENABLE_LTR390

#include <Arduino.h>
#include <Wire.h>

#include "configuration.h"
#include "loop_functions_extern.h"
#include "ltr390.h"
#include "ltr390_calc.h"

static bool bLtr390Found = false;
static bool bLtr390NotFoundLogged = false;
static float fLtr390Wfac = 1.0f;

static bool ltr390ReadRegs(uint8_t reg, uint8_t *buf, uint8_t len)
{
	Wire.beginTransmission(LTR390_I2C_ADDR);
	Wire.write(reg);
	if (Wire.endTransmission(true) != 0)
		return false;
	if (Wire.requestFrom((uint8_t)LTR390_I2C_ADDR, len) != len)
		return false;
	for (uint8_t i = 0; i < len; i++)
		buf[i] = Wire.read();
	return true;
}

static bool ltr390WriteReg(uint8_t reg, uint8_t val)
{
	Wire.beginTransmission(LTR390_I2C_ADDR);
	Wire.write(reg);
	Wire.write(val);
	return Wire.endTransmission(true) == 0;
}

bool ltr390Begin(void)
{
	uint8_t id = 0;

	bLtr390Found = false;

	if (ltr390ReadRegs(LTR390_REG_PART_ID, &id, 1) && ltr390PartIdOk(id)
		&& ltr390WriteReg(LTR390_REG_MEAS_RATE, LTR390_MEAS_20BIT_500MS)
		&& ltr390WriteReg(LTR390_REG_GAIN, LTR390_GAIN_18X)
		&& ltr390WriteReg(LTR390_REG_MAIN_CTRL, LTR390_CTRL_UVS_ACTIVE))
	{
		bLtr390Found = true;
		return true;
	}

	if (bWXDEBUG && !bLtr390NotFoundLogged)
		Serial.println("[LTR390] not found");
	bLtr390NotFoundLogged = true;

	return false;
}

void ltr390End(void)
{
	if (bLtr390Found)
		ltr390WriteReg(LTR390_REG_MAIN_CTRL, 0x00);   // standby
	bLtr390Found = false;
	bLtr390NotFoundLogged = false;
}

void ltr390SetWfac(float wfac)
{
	if (wfac > 0.0f)
		fLtr390Wfac = wfac;
}

bool ltr390Read(uint32_t *raw, float *uvi)
{
	uint8_t st = 0;
	uint8_t d[3];

	if (!bLtr390Found)
		return false;
	if (!ltr390ReadRegs(LTR390_REG_MAIN_STATUS, &st, 1) || !ltr390DataReady(st))
		return false;
	if (!ltr390ReadRegs(LTR390_REG_UVS_DATA, d, 3))
		return false;

	uint32_t r = ltr390Raw20(d[0], d[1], d[2]);
	float u = ltr390Uvi(r, LTR390_GAIN_VALUE_18X, LTR390_RES_20BIT, fLtr390Wfac);

	if (raw)
		*raw = r;
	if (uvi)
		*uvi = u;

	if (bWXDEBUG)
		Serial.printf("[LTR390] raw=%lu gain=18 res=20 uvi=%.1f\n", (unsigned long)r, u);

	return true;
}

bool ltr390Found(void)
{
	return bLtr390Found;
}

#endif // ENABLE_LTR390
