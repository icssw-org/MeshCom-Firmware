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
#include <Arduino.h>
//#include "WisBlock-API.h"

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
#if defined( NRF52_SERIES)
	uint32_t vbat_pin = WB_A0;
#elif defined(BOARD_TBEAM) || defined(BOARD_SX1268)
	uint32_t vbat_pin = 14;
#elif defined(BOARD_HELTEC)
	uint32_t vbat_pin = 35;
#else
	uint32_t vbat_pin = 35;
#endif

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
//	return 0.0;

	#if defined(NRF52_SERIES)
		analogReference(AR_INTERNAL_3_0);
	#endif

	float raw;

	// Get the raw 12-bit, 0..3000mV ADC value
	uint32_t irawd[10] = {0};
	uint32_t iraws = 0;
	for(int its=0;its<10;its++)
	{
		irawd[its] = analogRead(vbat_pin);
		iraws = iraws + irawd[its];
		//Serial.printf("BATT irawd[%i]:%i\n", its, irawd[its]);
		delay(20);
	}

	iraws = iraws / 10;

	uint32_t iraw=0;
	uint16_t irawc=0;
	for(int its=0;its<10;its++)
	{
		if((float)irawd[its] > (float(iraws) * .95) && (float)irawd[its] < ((float)iraws * 1.05) )
		{
			irawc++;
			iraw = iraw + irawd[its];
			//Serial.printf("BATT2 irawd[%i]:%i\n", its, irawd[its]);
		}
	}

	if(irawc == 0)
		iraw = iraws;
	else
		iraw = iraw / irawc;

	//Serial.printf("\nBATT iraw:%i\n", iraw);

	#if defined( NRF52_SERIES)
		// take it als mV
		raw = (float)iraw * REAL_VBAT_MV_PER_LSB;
	#elif defined(BOARD_TBEAM) || defined(BOARD_SX1268)
		raw = (((float)iraw * 11.75) / 4095.0) * 4.2 * 1000.0;
	#elif defined(BOARD_HELTEC)
		raw = (((float)iraw * VBAT_DIVIDER_COMP) / 4095.0) * 4.2 * 1000.0;
	#else
		raw = (((float)iraw * VBAT_DIVIDER_COMP) / 4095.0) * 4.2 * 1000.0;
	#endif

	//Serial.printf("FLOW raw:%.2f mV\n", raw);

	return raw;
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
