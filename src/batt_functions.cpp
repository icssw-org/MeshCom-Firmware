#include <Arduino.h>
#include <configuration.h>

#include <loop_functions.h>
#include <loop_functions_extern.h>

#include <esp_adc_cal.h>

float global_batt = 0;
int global_proz = 0;

unsigned long BattTimeWait = 0;
unsigned long BattTimeAPP = 0;

extern bool is_receiving;

#if defined(BOARD_RAK4630)

uint32_t vbat_pin = WB_A0;

#define NO_OF_SAMPLES   64          //Multisampling

#endif

#if defined(BOARD_T_ECHO)

//uint32_t vbat_pin = WB_A0;
uint32_t vbat_pin = 4;

#define NO_OF_SAMPLES   64          //Multisampling

#endif

#if defined(BOARD_E290)

uint32_t vbat_pin = BATTERY_PIN;

#define NO_OF_SAMPLES   64          //Multisampling

#endif

#if defined(BOARD_TLORA_OLV216)

uint32_t vbat_pin = BATTERY_PIN;

#endif

#if defined(BOARD_HELTEC_V3) || defined(BOARD_STICK_V3) || defined(BOARD_HELTEC_V4)

uint32_t vbat_pin = BATTERY_PIN;

#endif

#if defined(BOARD_HELTEC)

uint32_t vbat_pin = BATTERY_PIN;

#endif

#if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)

uint32_t vbat_pin = BATTERY_PIN;

#endif

#if defined(BOARD_RAK4630) || defined(BOARD_T_ECHO)
//nothing
#else

#include "esp_adc_cal.h"

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

#if !defined(BOARD_TRACKER) && !defined(BOARD_HELTEC)
//static
esp_adc_cal_characteristics_t adc_chars[sizeof(esp_adc_cal_characteristics_t)];

#if defined(CONFIG_IDF_TARGET_ESP32)

#if defined(BOARD_TLORA_OLV216)
//static const
adc_channel_t channel = ADC_CHANNEL_7;	 //GPIO35
#else
//static const
adc_channel_t channel = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2
#endif

//static const
adc_bits_width_t width = ADC_WIDTH_BIT_12;

#elif defined(CONFIG_IDF_TARGET_ESP32S2)
//static const 
adc_channel_t channel = ADC_CHANNEL_6;     // GPIO7 if ADC1, GPIO17 if ADC2
//static const
adc_bits_width_t width = ADC_WIDTH_BIT_13;
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
//static const
adc_channel_t channel = ADC_CHANNEL_6;
//static const
adc_bits_width_t width = ADC_WIDTH_BIT_12;

#endif

#endif

#if defined(BOARD_TBEAM) || defined(BOARD_SX1268)
//static const
adc_atten_t atten = ADC_ATTEN_DB_0;
//static const
adc_unit_t unit = ADC_UNIT_2;
#elif defined(BOARD_TRACKER)
#elif defined(BOARD_STICK_V3)
//static const
adc_atten_t atten = ADC_ATTEN_DB_0;
//static const
adc_unit_t unit = ADC_UNIT_1;
#elif defined(BOARD_TBEAM_1W)
adc_atten_t atten = ADC_ATTEN_DB_2_5;
adc_unit_t unit = ADC_UNIT_1;
#else
//static const
adc_atten_t atten = ADC_ATTEN_DB_0;
//static const
adc_unit_t unit = ADC_UNIT_1;
#endif


//static
void check_efuse(void)
{
	// NOT TESTED
#if defined(CONFIG_IDF_TARGET_ESP32)
    //Check if TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        Serial.printf("[EFUS]...Two Point: Supported\n");
    } else {
        Serial.printf("[EFUS]...Two Point: NOT supported\n");
    }
    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        Serial.printf("[EFUS]...Vref: Supported\n");
    } else {
        Serial.printf("[EFUS]...Vref: NOT supported\n");
    }
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        Serial.printf("[EFUS]...Two Point: Supported\n");
    } else {
        Serial.printf("[EFUS]...Cannot retrieve eFuse Two Point calibration values. Default calibration values will be used.\n");
    }
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
	//Check if TP is burned into eFuse
	if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
		Serial.printf("[EFUS]...Two Point: Supported\n");
	} else {
		Serial.printf("[EFUS]...Two Point: NOT supported\n");
	}
	//Check Vref is burned into eFuse
	if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
		Serial.printf("[EFUS]...Vref: Supported\n");
	} else {
		Serial.printf("[EFUS]...Vref: NOT supported\n");
	}
#else
#error "[EFUS]...This example is configured for ESP32/ESP32S2/ESP32S3."
#endif
}

//static
void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        Serial.printf("[ADC ]...Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        Serial.printf("[ADC ]...Characterized using eFuse Vref\n");
    } else {
        Serial.printf("[ADC ]...Characterized using Default Vref\n");
    }
}


#endif

#if defined(BOARD_E290)

void VextON(void)
{
	pinMode(VEXT_ENABLE_1,OUTPUT);
	digitalWrite(VEXT_ENABLE_1, HIGH);
	pinMode(VEXT_ENABLE_2,OUTPUT);
	digitalWrite(VEXT_ENABLE_2, HIGH);
}

void VextOFF(void)  // Vext default OFF
{
	pinMode(VEXT_ENABLE_1,OUTPUT);
	digitalWrite(VEXT_ENABLE_1, LOW);
	pinMode(VEXT_ENABLE_2,OUTPUT);
	digitalWrite(VEXT_ENABLE_2, LOW);
}
#endif

/**
 * @brief Initialize the battery analog input
 *
 */
void init_batt(void)
{
    Serial.println("[INIT]...init_batt");

// geht für HELTEC V3/V4 und für V3.2  wichtig für Display
#if defined(BOARD_HELTEC_V3) || defined(BOARD_STICK_V3) || defined(BOARD_HELTEC_V4)
	pinMode(36,OUTPUT);
	digitalWrite(36, LOW);

	#define ADC_CTRL_PIN 37
	#define BATTERY_SAMPLES 20

	pinMode(vbat_pin, INPUT);
	pinMode(ADC_CTRL_PIN, OUTPUT);

	analogReadResolution(12);
#endif

#if defined(NRF52_SERIES)
	// Set the resolution to 12-bit (0..4095)
	analogReadResolution(12); // Can be 8, 10, 12 or 14

	// Set the analog reference to 3.0V (default = 3.6V)
	analogReference(AR_INTERNAL_3_0);

	// Set the sampling time to 10us
	analogSampleTime(10);

#elif defined(BOARD_E290)

	VextON();

	analogReadResolution(12); // Can be 8, 10, 12 or 14

#elif defined(BOARD_E22_S3)
	analogSetAttenuation(ADC_0db);
	analogReadResolution(12);

#elif defined(BOARD_TBEAM_1W)
	analogSetAttenuation(ADC_11db); // bis ≈4,3V an GPIO
	analogReadResolution(12);

#elif defined(BOARD_TRACKER)

#elif defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
// NONE

#elif defined(BOARD_HELTEC)
	// Heltec V2: simple analogRead on GPIO 37
	pinMode(vbat_pin, INPUT);

#else
	//only for Test check_efuse();

    //Configure ADC
    if (unit == ADC_UNIT_1)
	{
        adc1_config_width(width);
        adc1_config_channel_atten((adc1_channel_t)channel, atten);
    }
	else
	{
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }

    //Characterize ADC
    //adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);
	print_char_val_type(val_type);

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
	//Serial.println("read_batt");

	is_receiving = true;

	float raw = 0.0;

	#if defined(NRF52_SERIES)

		analogReference(AR_INTERNAL_3_0);
		
		uint32_t adc_reading = 0;
		//Multisampling
		for (int i = 0; i < NO_OF_SAMPLES; i++)
		{
			int raw = analogRead(vbat_pin);
			adc_reading += raw;
		}

		adc_reading /= NO_OF_SAMPLES;

		raw = (float)adc_reading;

		//Serial.printf("Raw: %d\n", adc_reading);

	#elif defined(BOARD_TLORA_OLV216)

		pinMode(23, OUTPUT);
  		pinMode(vbat_pin, INPUT);
  
   		raw = (float)(analogRead(vbat_pin)) / 4095*2*3.3*1.1;
		
		// Serial.printf("ADC analog value = <%f>\n", raw);

	#elif defined(BOARD_E290)

   		uint16_t battery_levl = analogRead(vbat_pin);
		
		if(bDisplayCont)
		 Serial.printf("ADC analog value = <%i>\n", battery_levl);
	
		raw = (float)battery_levl;

	#elif defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)

        uint16_t adcValue = analogRead(vbat_pin);
        raw = adcValue;

	// TRACKER
	#elif defined(BOARD_TRACKER)
		#ifdef BATTERY_PIN
			int adc_value = analogRead(BATTERY_PIN);

			double voltage = (adc_value * 3.3 ) / 4095.0;
			
			double inputDivider = (1.0 / (390.0 + 100.0)) * 100.0;  // The voltage divider is a 390k + 100k resistor in series, 100k on the low side. 

			float milliVolt = ((voltage / inputDivider) + 0.285)*1000.0;
			
			is_receiving = false;
			
			return milliVolt; // Yes, this offset is excessive, but the ADC on the ESP32s3 is quite inaccurate and noisy. Adjust to own measurements.
		#else
			is_receiving = false;
			return (float)0.0;
		#endif
	#elif defined(BOARD_HELTEC_V3) || defined(BOARD_STICK_V3) || defined(BOARD_HELTEC_V4)

		// ADC resolution
		const int resolution = 12;
		const int adcMax = pow(2,resolution) - 1;
		const float adcMaxVoltage = 3.3;
		// On-board voltage divider
		const int R1 = 390;
		const int R2 = 100;
		// Calibration measurements
		const float measuredVoltage = 4.2;
		const float reportedVoltage = 4.095;
		// Calibration factor
		const float factor = (adcMaxVoltage / adcMax) * ((R1 + R2)/(float)R2) * (measuredVoltage / reportedVoltage);
		
		//V3.1 digitalWrite(ADC_CTRL_PIN,LOW);
		digitalWrite(ADC_CTRL_PIN, HIGH);

		delay(100);
		int analogValue = analogRead(vbat_pin);
		
		//V3.1 digitalWrite(ADC_CTRL_PIN, HIGH);
		digitalWrite(ADC_CTRL_PIN, LOW);

		float floatVoltage = factor * analogValue;
		uint16_t voltage = (int)(floatVoltage * 1000.0);

		if(bDEBUG && bDisplayInfo)
		{
			Serial.print("[readBatteryVoltage] ADC : ");
			Serial.println(analogValue);
			Serial.print("[readBatteryVoltage] Float : ");
			Serial.println(floatVoltage,3);
			Serial.print("[readBatteryVoltage] milliVolts : ");
			Serial.println(voltage);
		}

		raw = floatVoltage * 1000.0;

		#elif defined(BOARD_E22_S3) || defined(BOARD_TBEAM_1W)

		uint16_t analogValue = analogReadMilliVolts(BATTERY_PIN);

		raw = (float)analogValue * fBattFaktor  + BAT_VOL_COMPENSATION;

		if(bDisplayCont)
		{
			Serial.printf("%s [BATT]...reading: %u factor: %.4f voltage: %.2f mV\n", getTimeString().c_str(), analogValue, fBattFaktor, raw);
		}

	#elif defined(BOARD_HELTEC)
		// Heltec V2: GPIO 37 via 220K/100K voltage divider (3.2:1 ratio)
		analogReadResolution(12);

		uint32_t adc_reading = 0;
		for (int i = 0; i < 8; i++) {
			adc_reading += analogReadMilliVolts(vbat_pin);
		}
		adc_reading /= 8;

		// Multiply by voltage divider ratio: (220K + 100K) / 100K = 3.2
		raw = (float)adc_reading * 3.2;

	#else

	int adc_value = 0;
	int adc_reading = 0;

	//Multisampling
	for (int i = 0; i < NO_OF_SAMPLES; i++)
	{
		if (unit == ADC_UNIT_1)
			adc_value = adc1_get_raw((adc1_channel_t)channel);
		else
			adc2_get_raw((adc2_channel_t)channel, width, &adc_value);

		adc_reading += adc_value;
	}

	adc_reading /= NO_OF_SAMPLES;

	//Convert adc_reading to voltage in mV
	uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);

	raw = voltage;
	
	#endif

	// take it als mV
	#if defined(NRF52_SERIES)
		raw = raw * 1.25717;
	#elif defined(BOARD_TBEAM) || defined(BOARD_SX1268)
		raw = raw * 10.7687;
	#elif defined(BOARD_HELTEC)
		// all done - millivolts computed directly in read path
	#elif defined(BOARD_HELTEC_V3) || defined(BOARD_STICK_V3) || defined(BOARD_TRACKER) || defined(BOARD_HELTEC_V4)
		// all done
	#elif defined(BOARD_E22_S3) || defined(BOARD_TBEAM_1W)
		// all done
	#elif defined(BOARD_TLORA_OLV216)
		raw = raw * 1000.0; // convert to volt
	#elif defined(BOARD_E290)
		raw = raw * 4.13173653;
	#elif defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
        raw = raw * 1.7209; //1.66051;
	#else
		raw = raw * 24.80;
	#endif

	if(bDisplayCont)
	{
		Serial.printf("[readBatteryVoltage] raw %.2f mV\n", raw);
	}

	is_receiving = false;

	return raw;
}

/**
 * @brief Estimate the battery level in percentage
 * from milli volts
 *
 * @param mvolts Milli volts measured from analog pin
 * @return uint8_t Battery level as percentage (0 to 100)
 */
float max_batt = 4.125;

void setMaxBatt(float u_max_batt)
{
	max_batt = u_max_batt;
}

uint8_t mv_to_percent(float mvolts)
{
	if (mvolts < max_batt * 0.785F)	// 80% (3300)
		return 0;

	if (mvolts < max_batt * 0.857F)	// (3600)
	{
		mvolts -= max_batt * 0.785F;	// (3300)
		return mvolts / 30;
	}

	if (mvolts > max_batt)	// 4200
	{
		return 100;
	}

	mvolts -= max_batt * 0.857F;	// 3600

	uint8_t rproz = 10 + (mvolts * 0.15F);
	
	return rproz;
}