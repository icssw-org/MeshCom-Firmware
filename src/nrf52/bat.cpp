/**
 * @file bat.cpp
 * @author Bernd Giesecke (bernd.giesecke@rakwireless.com)
 * @brief Battery reading functions
 * @version 0.1
 * @date 2021-04-24
 *
 * @copyright Copyright (c) 2021
 *
 */
#include "WisBlock-API.h"

#if defined NRF52_SERIES || defined ESP32
/** Millivolts per LSB 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096 */
#define VBAT_MV_PER_LSB (0.73242188F)
/** Compensation factor for the VBAT divider */
#define VBAT_DIVIDER_COMP (1.73)
#endif
#ifdef ARDUINO_ARCH_RP2040
/** Millivolts per LSB 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096 */
#define VBAT_MV_PER_LSB (0.806F)
/** Compensation factor for the VBAT divider */
#define VBAT_DIVIDER_COMP (1.846F)
#endif

/** Real milli Volts per LSB including compensation */
#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)

/** Analog input for battery level */
uint32_t vbat_pin = WB_A0;

/**
 * @brief Initialize the battery analog input
 *
 */
void init_batt(void)
{
#if defined NRF52_SERIES
	// Set the analog reference to 3.0V (default = 3.6V)
	analogReference(AR_INTERNAL_3_0);
#endif

	// Set the resolution to 12-bit (0..4095)
	analogReadResolution(12); // Can be 8, 10, 12 or 14

#if defined NRF52_SERIES
	// Set the sampling time to 10us
	analogSampleTime(10);
#endif
}

/**
 * @brief Read the analog value from the battery analog pin
 * and convert it to milli volt
 *
 * @return float Battery level in milli volts 0 ... 4200
 */
float read_batt(void)
{
	float raw;

	// Get the raw 12-bit, 0..3000mV ADC value
	raw = analogRead(vbat_pin);

	// Convert the raw value to compensated mv, taking the resistor-
	// divider into account (providing the actual LIPO voltage)
	// ADC range is 0..3000mV and resolution is 12-bit (0..4095)
	return raw * REAL_VBAT_MV_PER_LSB;
}

/**
 * @brief Estimate the battery level in percentage
 * from milli volts
 *
 * @param mvolts Milli volts measured from analog pin
 * @return uint8_t Battery level as percentage (0 to 100)
 */
uint8_t mv_to_percent(float mvolts)
{
	if (mvolts < 3300)
		return 0;

	if (mvolts < 3600)
	{
		mvolts -= 3300;
		return mvolts / 30;
	}

	if (mvolts > 4200)
	{
		return 100;
	}

	mvolts -= 3600;
	return 10 + (mvolts * 0.15F); // thats mvolts /6.66666666
}

/**
 * @brief Read the battery level as value
 * between 0 and 254. This is used in LoRaWan status requests
 * as the battery level
 *
 * @return uint8_t Battery level as value between 0 and 254
 */
uint8_t get_lora_batt(void)
{
	uint16_t read_val = 0;
	for (int i = 0; i < 10; i++)
	{
		read_val += read_batt();
	}
	return (mv_to_percent(read_val / 10) * 2.54);
}