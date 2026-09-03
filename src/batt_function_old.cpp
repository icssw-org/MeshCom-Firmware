#include <Arduino.h>
#include <configuration.h>
#include "printfdeb_functions.h"
#include "batt_functions.h"

#ifndef USE_NEW_BATT

#include <loop_functions.h>
#include <loop_functions_extern.h>

#if !defined(NRF52_SERIES)
#include <esp_adc_cal.h>
#endif

float global_batt = 0;
int global_proz = 0;

unsigned long BattTimeWait = 0;
unsigned long BattTimeAPP = 0;

// Compile-Zeit-Fallback: bisheriges hartcodiertes Verhalten dieses Pfads (HIGH=Teiler ein,
// LOW=Teiler aus). Siehe batt_functions.h fuer die Begruendung der Probe.
batt_probe_t battProbeState = BATT_PROBE_ACTIVE_HIGH;

#if defined(BOARD_HELTEC_V3) || defined(BOARD_STICK_V3) || defined(BOARD_HELTEC_V4) || \
	(defined(NRF52_SERIES) && not defined(BOARD_HELTEC_T114) && not defined(BOARD_T_ECHO))
// max_batt is defined further down (near mv_to_percent(), in mV -- setMaxBatt() is always
// called with node_maxv*1000). Forward-declared here so read_batt()'s HELTEC_V3/STICK_V3/
// HELTEC_V4 branch (defined before max_batt in this file) can derive the detector's
// plausible band from it.
extern float max_batt;

// ----- BAT-01/BAT-02: no-battery detection -----
// Duplicated core, not shared: the canonical copy (with the full rationale -- floating
// divider node, why raw/unfiltered samples, why the band is relative to max_batt, why
// hysteresis) lives in batt_functions.h/.cpp (battDetectReset()/battDetectUpdate()). This
// file is the USE_NEW_BATT-undefined branch (see "#ifndef USE_NEW_BATT" above) and
// batt_functions.h declares that detector inside "#if defined(USE_NEW_BATT)", so it is not
// visible in this translation unit -- hence a minimal, algorithmically identical duplicate
// here rather than a shared include. Keep any change to the algorithm/thresholds in sync
// with the canonical copy. Scoped to HELTEC_V3/STICK_V3/HELTEC_V4 (BAT-01) and the plain
// NRF52_SERIES board -- i.e. RAK4631/WisBlock, the only nRF52 target that reaches this guard
// since BOARD_HELTEC_T114/BOARD_T_ECHO are excluded (BAT-02). Every other board in this file
// (TLORA_OLV216, TRACKER, T_CONNECT_PRO, ...) is untouched.
#define BATT_DETECT_MAX_DELTA_MV        250.0f   // see canonical copy for the measured-swing justification
#define BATT_DETECT_MIN_BAND_FACTOR     0.55f    // fraction of max_batt -- see canonical copy
#define BATT_DETECT_MAX_BAND_FACTOR     1.15f
#define BATT_DETECT_ABSENT_STREAK       6        // ~3s at the 500ms read_batt() cadence
#define BATT_DETECT_PRESENT_STREAK      10       // ~5s

typedef struct {
	bool  haveLast;
	float lastMv;
	int   implausibleStreak;
	int   plausibleStreak;
	bool  present;
} batt_detect_state_t;

static void battDetectReset(batt_detect_state_t *state)
{
	state->haveLast = false;
	state->lastMv = 0.0f;
	state->implausibleStreak = 0;
	state->plausibleStreak = 0;
	state->present = true;   // fail-safe: erst nach BATT_DETECT_ABSENT_STREAK unplausiblen Samples "false"
}

static bool battDetectUpdate(batt_detect_state_t *state, float rawMv, float minPlausibleMv, float maxPlausibleMv)
{
	bool implausible = (rawMv < minPlausibleMv) || (rawMv > maxPlausibleMv);

	if (state->haveLast)
	{
		float delta = state->lastMv - rawMv;
		if (delta < 0) { delta = -delta; }
		if (delta > BATT_DETECT_MAX_DELTA_MV) { implausible = true; }
	}

	state->lastMv = rawMv;
	state->haveLast = true;

	if (implausible)
	{
		state->implausibleStreak++;
		state->plausibleStreak = 0;
	}
	else
	{
		state->plausibleStreak++;
		state->implausibleStreak = 0;
	}

	if (state->present && state->implausibleStreak >= BATT_DETECT_ABSENT_STREAK)
		state->present = false;
	else if (!state->present && state->plausibleStreak >= BATT_DETECT_PRESENT_STREAK)
		state->present = true;

	return state->present;
}

// Produktions-Instanz (ein Zustand pro Node). read_batt() speist sie mit dem frischen,
// ungefilterten Sample (nur wenn tatsaechlich neu vom ADC gelesen wurde, nicht in den
// dividerArmed-Zwischenzyklen), battHardwarePresent() liest das Urteil.
static batt_detect_state_t battDetectState;
static bool battDetectStateInit = false;

static bool battDetectFeed(float rawMv, float minPlausibleMv, float maxPlausibleMv)
{
	if (!battDetectStateInit)
	{
		battDetectReset(&battDetectState);
		battDetectStateInit = true;
	}
	return battDetectUpdate(&battDetectState, rawMv, minPlausibleMv, maxPlausibleMv);
}

static bool battDetected(void)
{
	if (!battDetectStateInit) { return true; }   // fail-safe vor dem ersten echten Sample
	return battDetectState.present;
}
#endif

bool battHardwarePresent(void)
{
#if defined(BOARD_HELTEC_V3) || defined(BOARD_STICK_V3) || defined(BOARD_HELTEC_V4) || \
	(defined(NRF52_SERIES) && not defined(BOARD_HELTEC_T114) && not defined(BOARD_T_ECHO))
	// fail-safe: nur bei positiv erkanntem "kein Teiler" ODER positiv erkannter Abwesenheit
	// (Laufzeit-Detektion, BAT-01/BAT-02). battProbeState bleibt auf dem RAK4631-Pfad
	// permanent BATT_PROBE_ACTIVE_HIGH (nur die Heltec-Probe in init_batt() aendert ihn),
	// dort reduziert sich der Ausdruck also praktisch auf battDetected().
	return battProbeState != BATT_PROBE_NONE && battDetected();
#else
	return battProbeState != BATT_PROBE_NONE;   // fail-safe: nur bei positiv erkanntem "kein Teiler" false
#endif
}

#if defined(NRF52_SERIES)

#if defined(BOARD_HELTEC_T114)
uint32_t vbat_pin = 4;
#elif defined(BOARD_T_ECHO)
uint32_t vbat_pin = 4;
#else
uint32_t vbat_pin = WB_A0;
#endif

#define NO_OF_SAMPLES   64          //Multisampling

#endif

#if defined(BOARD_E290)

uint32_t vbat_pin = BATTERY_PIN;
#define NO_OF_SAMPLES   64          //Multisampling
#endif

#if defined(BOARD_WIRELESS_PAPER)
uint32_t vbat_pin = BATTERY_PIN;    // GPIO20, VBAT ueber 1:1-Teiler
#define ADC_CTRL_PIN ADC_CTRL_WP    // GPIO19, Mess-Freigabe (active LOW)
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

#if defined(BOARD_T3S3_V13)
uint32_t vbat_pin = ADC_PIN;
#endif

#if defined(BOARD_T_CONNECT_PRO)
uint32_t vbat_pin = ADC_PIN;
#endif

#if defined(NRF52_SERIES)
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
        printfdeb("[EFUS]...Two Point: Supported\n");
    } else {
        printfdeb("[EFUS]...Two Point: NOT supported\n");
    }
    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printfdeb("[EFUS]...Vref: Supported\n");
    } else {
        printfdeb("[EFUS]...Vref: NOT supported\n");
    }
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printfdeb("[EFUS]...Two Point: Supported\n");
    } else {
        printfdeb("[EFUS]...Cannot retrieve eFuse Two Point calibration values. Default calibration values will be used.\n");
    }
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
	//Check if TP is burned into eFuse
	if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
		printfdeb("[EFUS]...Two Point: Supported\n");
	} else {
		printfdeb("[EFUS]...Two Point: NOT supported\n");
	}
	//Check Vref is burned into eFuse
	if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
		printfdeb("[EFUS]...Vref: Supported\n");
	} else {
		printfdeb("[EFUS]...Vref: NOT supported\n");
	}
#else
#error "[EFUS]...This example is configured for ESP32/ESP32S2/ESP32S3."
#endif
}

//static
void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printfdeb("[ADC ]...Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printfdeb("[ADC ]...Characterized using eFuse Vref\n");
    } else {
        printfdeb("[ADC ]...Characterized using Default Vref\n");
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

#if defined(BOARD_HELTEC_V3) || defined(BOARD_STICK_V3) || defined(BOARD_HELTEC_V4)
// battProbeState startet bewusst NICHT auf BATT_PROBE_UNKNOWN (Compile-Zeit-Fallback oben),
// daher braucht das "einmalig ausfuehren"-Gating ein eigenes Flag statt eines Vergleichs
// gegen battProbeState.
static bool battProbeDone = false;

// Einmalige ADC_CTRL_PIN-Polaritaets-Probe (siehe Begruendung in batt_functions.h). Wird aus
// init_batt() aufgerufen, das battProbeDone-Flag sorgt dafuer, dass sie trotz mehrfachem
// init_batt()-Aufruf (z.B. nach --batt factor Befehl) nur einmal laeuft.
static void battProbeADCPolarity(uint32_t ctrlPin, uint32_t vbatPin)
{
	int countsHigh = 0;
	int countsLow  = 0;

	digitalWrite(ctrlPin, HIGH);
	delay(100);   // Teiler braucht ~100ms zum Einschwingen (wie im Heltec-Zweig von read_batt())
	for (int i = 0; i < 8; i++) { countsHigh += analogRead(vbatPin); }
	countsHigh /= 8;

	digitalWrite(ctrlPin, LOW);
	delay(100);
	for (int i = 0; i < 8; i++) { countsLow += analogRead(vbatPin); }
	countsLow /= 8;

	int probeDelta = countsHigh - countsLow;
	if (probeDelta < 0) probeDelta = -probeDelta;
	if (countsHigh >= BATT_PROBE_MIN_COUNTS && countsLow >= BATT_PROBE_MIN_COUNTS && probeDelta < BATT_PROBE_MIN_COUNTS)
	{
		// Beide Messungen plausibel und praktisch gleich: der Teiler liegt fest an, der
		// Steuerpin bewirkt nichts (z.B. Wireless Stick V3). Batteriehardware vorhanden,
		// Polaritaet ohne Bedeutung -> nicht als "kein Teiler" fehlinterpretieren.
		battProbeState = BATT_PROBE_ACTIVE_HIGH;
		digitalWrite(ctrlPin, LOW);
	}
	else if (countsHigh >= BATT_PROBE_MIN_COUNTS && countsHigh > countsLow)
	{
		battProbeState = BATT_PROBE_ACTIVE_HIGH;
		digitalWrite(ctrlPin, LOW);    // Ruhezustand: Teiler getrennt (Strom sparen)
	}
	else if (countsLow >= BATT_PROBE_MIN_COUNTS && countsLow > countsHigh)
	{
		battProbeState = BATT_PROBE_ACTIVE_LOW;
		digitalWrite(ctrlPin, HIGH);   // Ruhezustand: Teiler getrennt (Strom sparen)
	}
	else
	{
		battProbeState = BATT_PROBE_NONE;   // kein Teiler bestueckt -> keine Batteriehardware
	}

	printfdeb("[INIT]...ADC_CTRL_PIN probe: high=%d;low=%d;-> %s\n", countsHigh, countsLow,
		(battProbeState == BATT_PROBE_ACTIVE_HIGH && probeDelta < BATT_PROBE_MIN_COUNTS && countsLow >= BATT_PROBE_MIN_COUNTS) ? "fester Teiler (active HIGH)" :
		(battProbeState == BATT_PROBE_ACTIVE_HIGH) ? "active HIGH" :
		(battProbeState == BATT_PROBE_ACTIVE_LOW)  ? "active LOW"  : "keine Batteriehardware (kein Teiler)");
}
#endif

/**
 * @brief Initialize the battery analog input
 *
 */
void init_batt(void)
{
    printlndeb("[INIT]...init_batt");

// geht für HELTEC V3/V4 und für V3.2  wichtig für Display
#if defined(BOARD_HELTEC_V3) || defined(BOARD_STICK_V3) || defined(BOARD_HELTEC_V4)
	pinMode(36,OUTPUT);
	digitalWrite(36, LOW);

	#define ADC_CTRL_PIN 37

	pinMode(vbat_pin, INPUT);
	pinMode(ADC_CTRL_PIN, OUTPUT);

	analogReadResolution(12);

	if (!battProbeDone)
	{
		battProbeADCPolarity(ADC_CTRL_PIN, vbat_pin);
		battProbeDone = true;
	}
#endif

// geht für HELTEC V3/V4 und für V3.2  wichtig für Display
#if defined(BOARD_HELTEC_T114)

	#define ADC_CTRL_PIN 6

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

#elif defined(BOARD_WIRELESS_PAPER)

	// Heltec Wireless Paper: VBAT (GPIO20) ueber Control-Pin GPIO19 (active LOW) freigeben.
	pinMode(vbat_pin, INPUT);
	pinMode(ADC_CTRL_PIN, OUTPUT);
	digitalWrite(ADC_CTRL_PIN, HIGH);   // Ruhezustand: Teiler getrennt (Strom sparen)

	analogReadResolution(12);
	analogSetPinAttenuation(vbat_pin, ADC_11db);   // ~ DB_12: bis ~2,5V am Pin -> *2 deckt 4,2V LiPo ab

#elif defined(BOARD_E22_S3)
	analogSetAttenuation(ADC_0db);
	analogReadResolution(12);

#elif defined(BOARD_TBEAM_1W)
	analogSetAttenuation(ADC_11db); // bis ≈4,3V an GPIO
	analogReadResolution(12);

#elif defined(BOARD_T3S3_V13)
	analogSetAttenuation(ADC_11db);
    analogReadResolution(12);

#elif defined(BOARD_T_CONNECT_PRO)
	analogSetAttenuation(ADC_11db);
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
	//printlndeb("read_batt");

	float raw = 0.0;

	#if defined(NRF52_SERIES) && not defined(BOARD_HELTEC_T114) && not defined(BOARD_T_ECHO)

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

		//printfdeb("Raw: %d\n", adc_reading);

	#elif defined(BOARD_TLORA_OLV216)

		pinMode(23, OUTPUT);
  		pinMode(vbat_pin, INPUT);
  
   		raw = (float)(analogRead(vbat_pin)) / 4095*2*3.3*1.1;
		
		// printfdeb("ADC analog value = <%f>\n", raw);

	#elif defined(BOARD_T3S3_V13)

		// Spannung am Pin in Millivolt auslesen (kalibriert)
		uint32_t Vpin = analogReadMilliVolts(ADC_PIN);
		
		// Da 1:1 Teiler: V_bat = Vpin * 2
		float battery_voltage = (Vpin * 2.0);

		raw = battery_voltage;
		
	#elif defined(BOARD_T_CONNECT_PRO)

		// NO BAT PIN
		raw = 0.0;
		
	#elif defined(BOARD_E290)

   		uint16_t battery_levl = analogRead(vbat_pin);

		if(bDisplayCont)
		 printfdeb("ADC analog value = <%i>\n", battery_levl);

		raw = (float)battery_levl;

	#elif defined(BOARD_WIRELESS_PAPER)

		// Mess-Freigabe (active LOW): Teiler durchschalten, kurz setteln lassen
		digitalWrite(ADC_CTRL_PIN, LOW);
		delay(10);

		uint32_t umv = 0;
		for (int i = 0; i < 8; i++)
			umv += analogReadMilliVolts(vbat_pin);   // kalibriert, beruecksichtigt Atten/Vref
		umv /= 8;

		digitalWrite(ADC_CTRL_PIN, HIGH);            // Teiler wieder trennen (Strom sparen)

		// 1:1-Spannungsteiler -> reale Batteriespannung = Pin-Spannung * ADC_MULTIPLIER (=2)
		raw = (float)umv * ADC_MULTIPLIER;

		if(bDisplayCont)
			printfdeb("%s [BATT]...Wireless Paper pin_mV=%u -> %.0f mV\n", getTimeString().c_str(), umv, raw);

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
			
			return milliVolt; // Yes, this offset is excessive, but the ADC on the ESP32s3 is quite inaccurate and noisy. Adjust to own measurements.
		#else
			return (float)0.0;
		#endif

		#elif defined(BOARD_HELTEC_T114)
		int adcin = vbat_pin; 
		int adcvalue = 0;
		float mv_per_lsb = 3000.0F / 4096.0F;  // 12-bit ADC with 3.0V input range

		analogReference(AR_INTERNAL_3_0);
		analogReadResolution(12);
		pinMode(ADC_CTRL_PIN, OUTPUT);
		digitalWrite(ADC_CTRL_PIN, 1);
		
		delay(10);
		
		adcvalue = analogRead(adcin);
		digitalWrite(ADC_CTRL_PIN, 0);

		uint16_t voltage = (uint16_t)((float)adcvalue * mv_per_lsb * 4.9);

		raw = voltage;

		#elif defined(BOARD_T_ECHO)

		#define VBAT_MV_PER_LSB   (0.73242188F)   // 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096

		#ifdef NRF52840_XXAA
		#define VBAT_DIVIDER      (0.5F)          // 150K + 150K voltage divider on VBAT
		#define VBAT_DIVIDER_COMP (2.0F)          // Compensation factor for the VBAT divider
		#else
		#define VBAT_DIVIDER      (0.71275837F)   // 2M + 0.806M voltage divider on VBAT = (2M / (0.806M + 2M))
		#define VBAT_DIVIDER_COMP (1.403F)        // Compensation factor for the VBAT divider
		#endif

		#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)

		// Set the analog reference to 3.0V (default = 3.6V)
		analogReference(AR_INTERNAL_3_0);

		// Set the resolution to 12-bit (0..4095)
		analogReadResolution(12); // Can be 8, 10, 12 or 14

		// Let the ADC settle
		delay(1);

		// Get the raw 12-bit, 0..3000mV ADC value
		raw = analogRead(vbat_pin);

		// Set the ADC back to the default settings
		analogReference(AR_DEFAULT);
		analogReadResolution(10);

		// Convert the raw value to compensated mv, taking the resistor-
		// divider into account (providing the actual LIPO voltage)
		// ADC range is 0..3000mV and resolution is 12-bit (0..4095)
		raw =  raw * REAL_VBAT_MV_PER_LSB;

		#elif defined(BOARD_HELTEC_V3) || defined(BOARD_STICK_V3) || defined(BOARD_HELTEC_V4)

		// ADC resolution
		/* faktor fix defined configuration.h
		const int resolution = 12;
		const int adcMax = (1 << resolution) -1;
		const float adcMaxVoltage = 3.3;
		// On-board voltage divider
		const int R1 = 390;
		const int R2 = 100;
		// Calibration measurements
		const float measuredVoltage = 4.2;
		const float reportedVoltage = 4.095;
		// Calibration factor
		const float factor = (adcMaxVoltage / adcMax) * ((R1 + R2)/(float)R2) * (measuredVoltage / reportedVoltage);
		*/
		const float factor = ADC_MULTIPLIER;
		/**/

		// Divider needs ~100 ms to settle after being enabled. Das bisherige
		// blockierende delay(100) haelt dabei die komplette Hauptschleife an --
		// alle ~500 ms (die BattTimeWait-Kadenz), gemessen ~100,6 ms pro Aufruf.
		// Spread the settle across two read_batt() calls instead of blocking:
		// arm+enable on one call (return the previous cached value), read+disable
		// on the next call once >=100 ms has actually elapsed.
		static bool dividerArmed = false;
		static uint32_t dividerArmedAt = 0;
		static float cachedRaw = 0.0;

		if (!dividerArmed)
		{
			// Polaritaet kommt aus der einmaligen Probe in init_batt() (battProbeState);
			// Fallback (Probe noch nicht gelaufen / nichts gefunden) = active HIGH, bisheriges Verhalten.
			if (battProbeState == BATT_PROBE_ACTIVE_LOW)
				digitalWrite(ADC_CTRL_PIN, LOW);
			else
				digitalWrite(ADC_CTRL_PIN, HIGH);
			dividerArmedAt = millis();
			dividerArmed = true;
			raw = cachedRaw;
		}
		else if ((uint32_t)(millis() - dividerArmedAt) >= 100)
		{
			int analogValue = analogRead(vbat_pin);

			if (battProbeState == BATT_PROBE_ACTIVE_LOW)
				digitalWrite(ADC_CTRL_PIN, HIGH);   // Teiler wieder trennen (Strom sparen)
			else
				digitalWrite(ADC_CTRL_PIN, LOW);    // Teiler wieder trennen (Strom sparen)
			dividerArmed = false;

			float floatVoltage = factor * analogValue;
			uint16_t voltage = (int)(floatVoltage);

			if(bDEBUG && bDisplayCont)
			{
				printdeb("[readBatteryVoltage] ADC : ");
				printlndeb(analogValue);
				printdeb("[readBatteryVoltage] Float : ");
				printfdeb("%.3f\n", floatVoltage);
				printdeb("[readBatteryVoltage] milliVolts : ");
				printlndeb(voltage);
			}

			// BAT-01: Laufzeit-Erkennung "kein Akku" auf dem frischen Sample -- max_batt ist
			// hier bereits in mV (setMaxBatt() wird mit node_maxv*1000 aufgerufen), daher
			// keine weitere Skalierung noetig. Nur in diesem Zweig gefuettert (echter neuer
			// ADC-Read), nicht in den dividerArmed-Zwischenzyklen oben/unten.
			bool battPresentNow = battDetectFeed(floatVoltage,
				max_batt*BATT_DETECT_MIN_BAND_FACTOR, max_batt*BATT_DETECT_MAX_BAND_FACTOR);

			// dieselbe 0V/"USB"-Konvention wie ueberall sonst (loop_functions.cpp prueft
			// global_batt==0.0), statt eine zweite Anzeige-Fallunterscheidung einzufuehren.
			raw = battPresentNow ? floatVoltage : 0.0f;
			cachedRaw = raw;
		}
		else
		{
			raw = cachedRaw;
		}

		#elif defined(BOARD_TBEAM_1W)

		// T-Beam 1W uses 7.4V battery with voltage divider
		// ADC reads through divider - adjust multiplier based on actual divider ratio
		analogReadResolution(12);

		uint32_t uraw = 0;
		for (int i = 0; i < 8; i++) {
			uraw += analogRead(BATTERY_PIN);
		}
		
		uraw = uraw / 8;

		if(uraw < 850)	// not batt on
			uraw = 0;

		// Assuming voltage divider ratio from ADC_MULTIPLIER
		// 3.3V reference, 12-bit ADC (4095 max)
		raw = (float)((uraw * 3300 * ADC_MULTIPLIER) / 4095);

		if(bDisplayCont)
		{
			printfdeb("%s [BATT]...reading: %u factor: %.4f voltage: %.2f mV\n", getTimeString().c_str(), uraw, (float)((3300 * ADC_MULTIPLIER) / 4095), raw);
		}

		#elif defined(BOARD_E22_S3)

		uint16_t analogValue = analogReadMilliVolts(BATTERY_PIN);

		raw = (float)analogValue * fBattFaktor  + BAT_VOL_COMPENSATION;

		if(bDisplayCont)
		{
			printfdeb("%s [BATT]...reading: %u factor: %.4f voltage: %.2f mV\n", getTimeString().c_str(), analogValue, fBattFaktor, raw);
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
	#if defined(BOARD_HELTEC_T114)
		// all done - millivolts computed directly in read path
	#elif defined(BOARD_T_ECHO)
		// all done - millivolts computed directly in read path
	#elif defined(BOARD_T3S3_V13)
		// all done - millivolts computed directly in read path
	#elif defined(BOARD_T_CONNECT_PRO)
		// all done - millivolts computed directly in read path
	#elif defined(NRF52_SERIES)
		raw = raw * 1.25717;

		// BAT-02: same runtime "no battery" detection as the Heltec V3/V4 branch (BAT-01),
		// fed with this board's raw (unfiltered -- there is no inter-call smoothing on this
		// path, just the intra-call multisample average above) mV sample. max_batt is in mV
		// by the time this runs (nrf52_main.cpp calls setMaxBatt(node_maxv*1000) at boot when
		// node_maxv is configured), same units as the Heltec copy, so no rescale needed here.
		{
			bool battPresentNow = battDetectFeed(raw,
				max_batt*BATT_DETECT_MIN_BAND_FACTOR, max_batt*BATT_DETECT_MAX_BAND_FACTOR);

			// dieselbe 0V/"USB"-Konvention wie ueberall sonst (loop_functions.cpp prueft
			// global_batt==0.0), statt eine zweite Anzeige-Fallunterscheidung einzufuehren.
			if (!battPresentNow) { raw = 0.0f; }
		}
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
	#elif defined(BOARD_WIRELESS_PAPER)
		// all done - read_batt-Zweig liefert bereits Millivolt (analogReadMilliVolts * 2)
	#elif defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
        raw = raw * 1.7209; //1.66051;
	#else
		raw = raw * 24.80;
	#endif

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
#endif