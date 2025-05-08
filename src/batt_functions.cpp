#include <Arduino.h>
#include <configuration.h>

#include <loop_functions_extern.h>

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

#if defined(BOARD_HELTEC_V3)

uint32_t vbat_pin = BATTERY_PIN;

#endif

#if defined(BOARD_RAK4630) || defined(BOARD_T_ECHO)
//nothing
#else

#include "esp_adc_cal.h"

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

//static
esp_adc_cal_characteristics_t adc_chars[sizeof(esp_adc_cal_characteristics_t)];

#if defined(SX1262_V3) || defined(SX1262_E290)
//static const
adc_channel_t channel = ADC_CHANNEL_0;     //GPIO34 if ADC1, GPIO14 if ADC2
//static const
adc_bits_width_t width = ADC_WIDTH_BIT_12;
#endif

#if CONFIG_IDF_TARGET_ESP32
#ifdef BOARD_TLORA_OLV216
//static const
adc_channel_t channel = ADC_CHANNEL_7;	 //GPIO35
#else
//static const
adc_channel_t channel = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2
#endif
//static const
adc_bits_width_t width = ADC_WIDTH_BIT_12;
#elif CONFIG_IDF_TARGET_ESP32S2
//static const
adc_channel_t channel = ADC_CHANNEL_6;     // GPIO7 if ADC1, GPIO17 if ADC2
//static const
adc_bits_width_t width = ADC_WIDTH_BIT_13;
#endif

#if defined(BOARD_TBEAM) || defined(BOARD_SX1268)
//static const
adc_atten_t atten = ADC_ATTEN_DB_0;
//static const
adc_unit_t unit = ADC_UNIT_2;
#elif defined(BOARD_HELTEC) || defined(BOARD_HELTEC_V3)
//static const
adc_atten_t atten = ADC_ATTEN_DB_0;
//static const
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
#if defined(SX1262_V3) || defined(SX1262_E290) || defined(SX126x_V3)
	// NOT TESTED
#elif CONFIG_IDF_TARGET_ESP32
    //Check if TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        Serial.printf("eFuse Two Point: Supported\n");
    } else {
        Serial.printf("eFuse Two Point: NOT supported\n");
    }
    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        Serial.printf("eFuse Vref: Supported\n");
    } else {
        Serial.printf("eFuse Vref: NOT supported\n");
    }
#elif CONFIG_IDF_TARGET_ESP32S2
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        Serial.printf("eFuse Two Point: Supported\n");
    } else {
        Serial.printf("Cannot retrieve eFuse Two Point calibration values. Default calibration values will be used.\n");
    }
#else
#error "This example is configured for ESP32/ESP32S2."
#endif
}

//static
void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        Serial.printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        Serial.printf("Characterized using eFuse Vref\n");
    } else {
        Serial.printf("Characterized using Default Vref\n");
    }
}


#endif

void VextON(void)
{
	pinMode(18,OUTPUT);
	digitalWrite(18, HIGH);
	pinMode(46,OUTPUT);
	digitalWrite(46, HIGH);
}

void VextOFF(void)  // Vext default OFF
{
	pinMode(18,OUTPUT);
	digitalWrite(18, LOW);
	pinMode(46,OUTPUT);
	digitalWrite(46, LOW);
}

/**
 * @brief Initialize the battery analog input
 *
 */
void init_batt(void)
{
    Serial.println("[INIT]...init_batt");

// getht für HELTEC V3 und für V3.2  wichtig für Display
#if defined(BOARD_HELTEC_V3)
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
    
	//only for test print_char_val_type(val_type);
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

	#elif defined(BOARD_HELTEC_V3)

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

		#elif defined(BOARD_E22_S3)

		uint16_t analogValue = analogReadMilliVolts(BATTERY_PIN);

		raw = (float)analogValue * fBattFaktor;

		if(bDisplayCont)
		{
			Serial.printf("[BATT]...reading: %lu factor: %.4f\n", analogValue, fBattFaktor);
			Serial.printf("[BATT]...voltage: %.2f mV\n", raw);
			delay(1000); 
		}

	#else

	int imax=1;
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
		raw = raw * 24.80;
	#elif defined(BOARD_HELTEC_V3)
		// al done
	#elif defined(BOARD_E22_S3)
		// al done
	#elif defined(BOARD_TLORA_OLV216)
		raw = raw * 1000.0; // convert to volt
	#elif defined(BOARD_E290)
		raw = raw * 4.13173653;
	#else
		raw = raw * 24.80;
	#endif

	if(bDisplayCont)
	{
		Serial.print("[readBatteryVoltage] raw mV : ");
		Serial.println(raw);
	}

	delay(50);

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
	
	return 10 + (mvolts * 0.15F); // thats mvolts /6.66666666
}