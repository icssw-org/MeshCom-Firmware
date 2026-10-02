#include <Arduino.h>
#include <configuration.h>
#include "printfdeb_functions.h"
#include "batt_functions.h"
#include "batt_pipeline.h"
#include "instrument.h"   // TEMPORARY -- INSTRUMENT_ENABLED, see src/instrument.h

// Battery path of every board WITHOUT USE_NEW_BATT (the ones with USE_NEW_BATT live in
// batt_functions.cpp): Heltec V2/V3/V4, Wireless Stick, Wireless Tracker, Vision Master E290,
// T-Deck Pro, T-Connect Pro, T5, T-ETH-Elite, esp32-loraprs-ra01, the classic T-Beams (which
// take their value from the PMU and never call read_batt(), the file must only compile),
// RAK4631, Heltec T114 and T-Echo.
//
// Layout (campaign 2026-09-29, docs/archive/concept-battery-consolidation-20260923.md):
//   1. per-family RAW readers, each returns the battery-side mV of ONE sample
//   2. one shared tail (battFilter): BAT-01 detector on the RAW sample -> EMA -> < 1000 mV = 0
//   3. read_batt(): asks the sampling scheduler (batt_pipeline.h) what to do this tick
//      (NONE / ARM the divider / READ) and returns the FILTERED mV ("no reading" = 0)
// EMA, detector, percent curve and scheduler are the shared code in batt_pipeline.h.
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

// ----- board families of this file -----
// Divider with an enable pin on GPIO37 that the sampling scheduler switches (SWITCHED profile)
#if defined(BOARD_HELTEC_V3) || defined(BOARD_STICK_V3) || defined(BOARD_HELTEC_V4)
#define BATT_FAMILY_HELTEC_SWITCHED 1
#endif
// RAK4631 / WisBlock: the only nRF52 target that is neither T114 nor T-Echo
#if defined(NRF52_SERIES) && !defined(BOARD_HELTEC_T114) && !defined(BOARD_T_ECHO)
#define BATT_FAMILY_RAK 1
#endif
// Boards with a floating VBAT node without a cell: BAT-01 (Heltec) / BAT-02 (RAK) no-battery
// detection. Every other board of this file runs without the detector, as before.
#if defined(BATT_FAMILY_HELTEC_SWITCHED) || defined(BATT_FAMILY_RAK)
#define BATT_USE_DETECTOR 1
#endif
// Boards whose divider is switched by the scheduler: ARM every 30 s, READ + release >= 100 ms later
#if defined(BATT_FAMILY_HELTEC_SWITCHED) || defined(BOARD_HELTEC_T114)
#define BATT_SWITCHED_DIVIDER 1
#endif

#if defined(BATT_FAMILY_HELTEC_SWITCHED)
#ifndef ADC_CTRL_PIN
#define ADC_CTRL_PIN 37   // V4's configuration.h defines the same value
#endif
#elif defined(BOARD_HELTEC_T114)
#define ADC_CTRL_PIN 6
#endif

// max_batt is in mV: setMaxBatt() is always called with node_maxv*1000. It also derives the
// detector's plausible band and the percent curve, so it is defined before its first use.
float max_batt = 4125.0F;

void setMaxBatt(float u_max_batt)
{
	max_batt = u_max_batt;
}

// Defined with the filter state below: true once a sample has reported 0 mV ("no reading").
static bool battNoReading(void);

bool battHardwarePresent(void)
{
	// A reported "no reading" (0 mV = USB, no cell) counts as absent on every board, as in
	// batt_functions.cpp: mv_to_percent(0) is 0, and "/B=000" would claim "empty" instead of "none".
	if (battNoReading())
		return false;
#if defined(BATT_USE_DETECTOR)
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

#if defined(BATT_FAMILY_HELTEC_SWITCHED)
uint32_t vbat_pin = BATTERY_PIN;
#endif

#if defined(BOARD_HELTEC)
uint32_t vbat_pin = BATTERY_PIN;
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
// war faelschlich als Array mit sizeof(...)-Elementen deklariert (36 Kopien statt einer
// Instanz, ~1.3 KB DRAM) - bitte nicht wieder auf [sizeof(...)] "korrigieren"
esp_adc_cal_characteristics_t adc_chars;

#if defined(CONFIG_IDF_TARGET_ESP32)

//static const
adc_channel_t channel = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2

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

#if defined(BATT_FAMILY_HELTEC_SWITCHED)
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
	delay(100);   // Teiler braucht ~100ms zum Einschwingen (wie das Settle-Fenster des Schedulers)
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
#if defined(BATT_FAMILY_HELTEC_SWITCHED)
	pinMode(36,OUTPUT);
	digitalWrite(36, LOW);

	pinMode(vbat_pin, INPUT);
	pinMode(ADC_CTRL_PIN, OUTPUT);

	analogReadResolution(12);

	if (!battProbeDone)
	{
		battProbeADCPolarity(ADC_CTRL_PIN, vbat_pin);
		battProbeDone = true;
	}
#endif

#if defined(BOARD_HELTEC_T114)

	pinMode(vbat_pin, INPUT);
	pinMode(ADC_CTRL_PIN, OUTPUT);
	digitalWrite(ADC_CTRL_PIN, LOW);   // Ruhezustand: Teiler getrennt (das Freigeben macht jetzt der Scheduler)

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

#elif defined(BOARD_T_CONNECT_PRO)
	analogSetAttenuation(ADC_11db);
    analogReadResolution(12);

#elif defined(BOARD_TRACKER)

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
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, &adc_chars);
	print_char_val_type(val_type);

#endif

}

// =========================================================================
// 1. Per-family raw readers: ONE sample, battery-side mV (0.0 = no reading)
// =========================================================================
// A raw reader does no filtering and no scheduling; it runs only on a scheduler READ. Debug
// prints live in the readers, so they show once per real sample, not once per 100 ms tick.

#if defined(BATT_USE_DETECTOR)
// Spread (max - min, mV) of the read window of the latest raw sample. Only the Heltec switched
// reader fills it; every other family leaves it at NONE. battFilter() consumes it with the sample
// and resets it, so a stale value is never reused.
static float s_battWindowSpreadMv = BATT_DETECT_SPREAD_NONE;
#endif

#if defined(BATT_SWITCHED_DIVIDER)

// Switch the divider on (scheduler ARM). The release is part of the raw reader, right after
// the ADC read, so the divider is never left on by anything but a pending READ.
static void battDividerArm(void)
{
#if defined(BATT_FAMILY_HELTEC_SWITCHED)
	// Polaritaet kommt aus der einmaligen Probe in init_batt() (battProbeState);
	// Fallback (Probe noch nicht gelaufen / nichts gefunden) = active HIGH, bisheriges Verhalten.
	if (battProbeState == BATT_PROBE_ACTIVE_LOW)
		digitalWrite(ADC_CTRL_PIN, LOW);
	else
		digitalWrite(ADC_CTRL_PIN, HIGH);
#else   // BOARD_HELTEC_T114
	pinMode(ADC_CTRL_PIN, OUTPUT);
	digitalWrite(ADC_CTRL_PIN, 1);
#endif
}

#endif

#if defined(BATT_FAMILY_HELTEC_SWITCHED)

// Family "switched divider" (Heltec V3/V4, Wireless Stick): divider on GPIO37 was armed >= 100 ms
// ago by the scheduler; read, release, scale.
static float battRawHeltecSwitched(void)
{
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

	// First read = the reported value (calibration unchanged).
	int analogValue = analogRead(vbat_pin);

	// Read window, still with the divider armed: a floating VBAT node (no cell) swings 800-1300 mV
	// within these ~14 ms, a real cell stays within ~50 mV. The spread feeds the no-battery detector.
	float windowMv[BATT_DETECT_WINDOW_READS];
	windowMv[0] = factor * analogValue;
	for (int i = 1; i < BATT_DETECT_WINDOW_READS; i++)
	{
		delay(BATT_DETECT_WINDOW_STEP_MS);
		windowMv[i] = factor * analogRead(vbat_pin);
	}
	float windowSpreadMv = battWindowSpread(windowMv, BATT_DETECT_WINDOW_READS);
	s_battWindowSpreadMv = windowSpreadMv;

	if (battProbeState == BATT_PROBE_ACTIVE_LOW)
		digitalWrite(ADC_CTRL_PIN, HIGH);   // Teiler wieder trennen (Strom sparen)
	else
		digitalWrite(ADC_CTRL_PIN, LOW);    // Teiler wieder trennen (Strom sparen)

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
		printdeb("[readBatteryVoltage] window spread mV : ");
		printfdeb("%.0f\n", windowSpreadMv);
	}

	return floatVoltage;
}

#elif defined(BATT_FAMILY_RAK)

// Family nRF52 RAK4631 / WisBlock: 64x block average, reference set per READ.
static float battRawNrf52Rak(void)
{
	analogReference(AR_INTERNAL_3_0);

	uint32_t adc_reading = 0;
	//Multisampling
	for (int i = 0; i < NO_OF_SAMPLES; i++)
	{
		int raw = analogRead(vbat_pin);
		adc_reading += raw;
	}

	adc_reading /= NO_OF_SAMPLES;

	//printfdeb("Raw: %d\n", adc_reading);

	return (float)((float)adc_reading * 1.25717);
}

#elif defined(BOARD_HELTEC_T114)

// Family nRF52 T114: divider on ADC_CTRL_PIN was armed by the scheduler (replaces the old
// enable + delay(10)); read, release.
static float battRawNrf52T114(void)
{
	int adcin = vbat_pin;
	int adcvalue = 0;
	float mv_per_lsb = 3000.0F / 4096.0F;  // 12-bit ADC with 3.0V input range

	analogReference(AR_INTERNAL_3_0);
	analogReadResolution(12);

	adcvalue = analogRead(adcin);
	digitalWrite(ADC_CTRL_PIN, 0);

	uint16_t voltage = (uint16_t)((float)adcvalue * mv_per_lsb * 4.9);

	return (float)voltage;
}

#elif defined(BOARD_T_ECHO)

// Family nRF52 T-Echo: reference and resolution set per READ, restored to the defaults after.
static float battRawNrf52TEcho(void)
{
	#define VBAT_MV_PER_LSB   (0.73242188F)   // 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096

	#ifdef NRF52840_XXAA
	#define VBAT_DIVIDER      (0.5F)          // 150K + 150K voltage divider on VBAT
	#define VBAT_DIVIDER_COMP (2.0F)          // Compensation factor for the VBAT divider
	#else
	#define VBAT_DIVIDER      (0.71275837F)   // 2M + 0.806M voltage divider on VBAT = (2M / (0.806M + 2M))
	#define VBAT_DIVIDER_COMP (1.403F)        // Compensation factor for the VBAT divider
	#endif

	#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)

	float raw;

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
	return raw * REAL_VBAT_MV_PER_LSB;
}

#elif defined(BOARD_E290)

// Family Vision Master E290: one plain read, fixed divider (Vext stays on since init_batt()).
static float battRawE290(void)
{
	uint16_t battery_levl = analogRead(vbat_pin);

	if(bDisplayCont)
	 printfdeb("ADC analog value = <%i>\n", battery_levl);

	return (float)((float)battery_levl * 4.13173653);
}

#elif defined(BOARD_T_CONNECT_PRO)

// Family T-Connect Pro: no battery pin, "no reading".
static float battRawTConnectPro(void)
{
	// NO BAT PIN
	return 0.0F;
}

#elif defined(BOARD_TRACKER)

// Family Wireless Tracker: one read, 390k + 100k divider, empirical offset.
static float battRawTracker(void)
{
	#ifdef BATTERY_PIN
		int adc_value = analogRead(BATTERY_PIN);

		double voltage = (adc_value * 3.3 ) / 4095.0;

		double inputDivider = (1.0 / (390.0 + 100.0)) * 100.0;  // The voltage divider is a 390k + 100k resistor in series, 100k on the low side.

		float milliVolt = ((voltage / inputDivider) + 0.285)*1000.0;

		return milliVolt; // Yes, this offset is excessive, but the ADC on the ESP32s3 is quite inaccurate and noisy. Adjust to own measurements.
	#else
		return (float)0.0;
	#endif
}

#elif defined(BOARD_HELTEC)

// Family Heltec V2: GPIO 37 via 220K/100K voltage divider (3.2:1 ratio), 8x block average.
static float battRawHeltecV2(void)
{
	analogReadResolution(12);

	uint32_t adc_reading = 0;
	for (int i = 0; i < 8; i++) {
		adc_reading += analogReadMilliVolts(vbat_pin);
	}
	adc_reading /= 8;

	// Multiply by voltage divider ratio: (220K + 100K) / 100K = 3.2
	return (float)((float)adc_reading * 3.2);
}

#else

// Family classic ESP32 / ESP32-S3 on the esp_adc_cal path (T-Beam classic, esp32-loraprs-ra01,
// T-Deck Pro, T-ETH-Elite, T5, ...): 64x block average, calibrated to mV, board scale.
#if defined(BOARD_TBEAM) || defined(BOARD_SX1268)
#define BATT_ESP32_ADC_SCALE 10.7687
#else
#define BATT_ESP32_ADC_SCALE 24.80
#endif

static float battRawEsp32Cal(void)
{
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
	uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, &adc_chars);

	return (float)((float)voltage * BATT_ESP32_ADC_SCALE);
}

#endif

// Dispatch to the family reader of this build.
static float battReadRaw(void)
{
#if defined(BATT_FAMILY_HELTEC_SWITCHED)
	return battRawHeltecSwitched();
#elif defined(BATT_FAMILY_RAK)
	return battRawNrf52Rak();
#elif defined(BOARD_HELTEC_T114)
	return battRawNrf52T114();
#elif defined(BOARD_T_ECHO)
	return battRawNrf52TEcho();
#elif defined(BOARD_E290)
	return battRawE290();
#elif defined(BOARD_T_CONNECT_PRO)
	return battRawTConnectPro();
#elif defined(BOARD_TRACKER)
	return battRawTracker();
#elif defined(BOARD_HELTEC)
	return battRawHeltecV2();
#else
	return battRawEsp32Cal();
#endif
}

// =========================================================================
// 2. Shared tail: detector (RAW sample) -> EMA -> < 1000 mV = "no reading"
// =========================================================================

#define BATT_NO_READING_MV   1000.0F   // a filtered value below this reports 0 (USB / no cell)

static batt_ema_t s_battEma;
static bool       s_battEmaInit = false;
static float      s_battMv = 0.0F;      // last filtered value returned by read_batt(), 0 = no reading

static float battFilter(float rawMv, uint32_t now)
{
	if (!s_battEmaInit)
	{
		battEmaInit(&s_battEma, BATT_EMA_TAU_MS_DEFAULT);
		s_battEmaInit = true;
	}

	bool present = true;

#if defined(BATT_USE_DETECTOR)
	// BAT-01/BAT-02: runtime "no battery" detection on the RAW sample (an EMA would smooth the
	// floating-node signature away). max_batt is in mV (setMaxBatt(node_maxv*1000)).
	present = battDetectFeedSpread(rawMv, s_battWindowSpreadMv, max_batt*BATT_DETECT_MIN_BAND_FACTOR, max_batt*BATT_DETECT_MAX_BAND_FACTOR);
	s_battWindowSpreadMv = BATT_DETECT_SPREAD_NONE;   // consumed: never reuse a stale window
#endif

	float mv = 0.0F;
	if (present && rawMv > 0.0F)
	{
		mv = battEmaUpdate(&s_battEma, rawMv, now);
		if (mv < BATT_NO_READING_MV) { mv = 0.0F; }   // same 0 V / "USB" convention as everywhere else
	}
	else
	{
		// no reading (no cell, no sensor): drop the filter state so the next real sample
		// seeds it again instead of ramping up from the noise
		battEmaInit(&s_battEma, BATT_EMA_TAU_MS_DEFAULT);
	}

	s_battMv = mv;
	return mv;
}

// =========================================================================
// 3. Entry points
// =========================================================================

static batt_sched_t s_battSched;
static bool         s_battSchedInit = false;
static uint32_t     s_battSampleCount = 0;

static bool battNoReading(void)
{
	return s_battSampleCount > 0 && s_battMv <= 0.0F;
}

// Number of real samples (scheduler READs) taken so far. A caller ticking read_batt() every
// 100 ms compares this against its last value to print once per sample.
uint32_t battSampleCount(void)
{
	return s_battSampleCount;
}

/**
 * @brief Tick the battery sampler (call about every 100 ms). Reads the analog value only when the
 * sampling scheduler says READ, converts it to milli volt and filters it.
 *
 * @return float FILTERED battery level in milli volts 0 ... 4200, 0 = no reading / no battery.
 *         Between two samples the cached value.
 */
float read_batt(void)
{
	const uint32_t now = millis();

	if (!s_battSchedInit)
	{
#if defined(BATT_SWITCHED_DIVIDER)
		battSchedInit(&s_battSched, BATT_SCHED_PROFILE_SWITCHED);
#else
		battSchedInit(&s_battSched, BATT_SCHED_PROFILE_FIXED);
#endif
		s_battSchedInit = true;
	}

	switch (battSchedTick(&s_battSched, now))
	{
		case BATT_SCHED_ARM:
#if defined(BATT_SWITCHED_DIVIDER)
			battDividerArm();
#endif
			return s_battMv;

		case BATT_SCHED_READ:
			break;

		case BATT_SCHED_NONE:
		default:
			return s_battMv;
	}

	const float raw = battReadRaw();
	s_battSampleCount++;

	return battFilter(raw, now);
}

/**
 * @brief Estimate the battery level in percentage
 * from milli volts (one shared curve, batt_pipeline.h battPercent)
 *
 * @param mvolts Milli volts measured from analog pin
 * @return uint8_t Battery level as percentage (0 to 100)
 */
uint8_t mv_to_percent(float mvolts)
{
	if (!(mvolts > 0.0F))
		return 0;

	return battPercent(mvolts, max_batt);
}

// ---------------------------------------------------------------------------
// TEMPORARY bench measurement: --battprobe [n]  (INSTRUMENT_ENABLED builds only, removed with the
// rest of the instrument scaffolding). Raw ADC capture of the switched Heltec divider to show
// whether a cell is attached: the BAT-01 detector sees the divider only 100 ms every 30 s, and a
// floating charger output reads 4.2-4.7 V in that window. Measurement only: it does not feed
// battDetectFeed()/battFilter() and leaves the divider released at exit. Runs synchronously in the
// loop task (~7.5 s per cycle), so it feeds the loopTask WDT itself and waits with delay().
// ---------------------------------------------------------------------------
#if INSTRUMENT_ENABLED
#if defined(BATT_FAMILY_HELTEC_SWITCHED)

#include <esp_task_wdt.h>
#include <math.h>

#define BATTPROBE_BURST_N     151   // t = 0..300 ms every 2 ms
#define BATTPROBE_BURST_STEP_US 2000UL
#define BATTPROBE_PAIRS       8
#define BATTPROBE_LONG_N      31    // t = 0..3000 ms every 100 ms
#define BATTPROBE_PER_LINE    25    // values per printed line (printfdeb buffer is 300 B format / 600 B out)

static void battProbeRelease(void)
{
	if (battProbeState == BATT_PROBE_ACTIVE_LOW)
		digitalWrite(ADC_CTRL_PIN, HIGH);   // divider off
	else
		digitalWrite(ADC_CTRL_PIN, LOW);    // divider off
}

// delay() (yields) in slices of <= 100 ms, feeding the loopTask WDT between the slices
static void battProbeWait(uint32_t ms)
{
	while (ms > 0)
	{
		uint32_t s = (ms > 100) ? 100 : ms;
		delay(s);
		esp_task_wdt_reset();
		ms -= s;
	}
}

// Wait until micros()-t0 >= targetUs: delay(1) while >= 1.5 ms remain, a bounded sub-1.5 ms
// delayMicroseconds() for the tail so a 2 ms grid stays usable.
static void battProbeWaitUntilUs(uint32_t t0, uint32_t targetUs)
{
	for (;;)
	{
		uint32_t el = micros() - t0;
		if (el >= targetUs)
			return;
		uint32_t rem = targetUs - el;
		if (rem >= 1500UL)
			delay(1);
		else
			delayMicroseconds(rem);
		esp_task_wdt_reset();
	}
}

static void battProbePrintValues(const char *tag, int cycle, const uint16_t *v, int n)
{
	for (int i = 0; i < n; i += BATTPROBE_PER_LINE)
	{
		char buf[160];
		int p = 0;
		for (int j = i; j < n && j < i + BATTPROBE_PER_LINE; j++)
		{
			p += snprintf(buf + p, sizeof(buf) - p, "%s%u", (j > i) ? "," : "", (unsigned)v[j]);
			if (p >= (int)sizeof(buf) - 8)
				break;
		}
		printfdeb("[BATTPROBE]|%s|%d|%d|%s\n", tag, cycle, i, buf);
		esp_task_wdt_reset();
	}
}

// min|max|mean|stdev of v[from..n-1] in raw counts, then the same in mV (x ADC_MULTIPLIER)
static void battProbePrintSummary(const char *tag, int cycle, const uint16_t *v, int from, int n, const char *extra)
{
	if (from >= n)
		return;
	uint16_t mn = 0xFFFF, mx = 0;
	double sum = 0.0, sumsq = 0.0;
	int cnt = 0;
	for (int i = from; i < n; i++)
	{
		if (v[i] < mn) mn = v[i];
		if (v[i] > mx) mx = v[i];
		sum += v[i];
		sumsq += (double)v[i] * (double)v[i];
		cnt++;
	}
	double mean = sum / cnt;
	double var = sumsq / cnt - mean * mean;
	double sd = (var > 0.0) ? sqrt(var) : 0.0;
	const double f = ADC_MULTIPLIER;
	printfdeb("[BATTPROBE]|%s|%d|%u|%u|%.2f|%.2f|mV|%.0f|%.0f|%.1f|%.1f|n=%d%s\n", tag, cycle,
		(unsigned)mn, (unsigned)mx, mean, sd, mn * f, mx * f, mean * f, sd * f, cnt, extra);
}

void battProbeRun(int cycles)
{
	if (cycles < 1) cycles = 1;
	if (cycles > 10) cycles = 10;

	const char *pol = (battProbeState == BATT_PROBE_ACTIVE_LOW) ? "LOW" :
		(battProbeState == BATT_PROBE_NONE) ? "NONE" : "HIGH";   // UNKNOWN runs as HIGH, like the scheduler
	printfdeb("[BATTPROBE]|START|%d|polarity=%s|maxv_mV=%.0f|factor=%.4f\n", cycles, pol, max_batt, (double)ADC_MULTIPLIER);

	pinMode(vbat_pin, INPUT);
	pinMode(ADC_CTRL_PIN, OUTPUT);
	battProbeRelease();
	esp_task_wdt_reset();

	for (int c = 1; c <= cycles; c++)
	{
		// ---- 1. BURST: arm, read every 2 ms from t=0 to t=300 ms, release ----
		uint16_t burst[BATTPROBE_BURST_N];
		uint32_t lagMaxUs = 0, durUs = 0;
		esp_task_wdt_reset();
		battDividerArm();
		uint32_t t0 = micros();
		for (int i = 0; i < BATTPROBE_BURST_N; i++)
		{
			uint32_t target = (uint32_t)i * BATTPROBE_BURST_STEP_US;
			if (i > 0)
				battProbeWaitUntilUs(t0, target);
			burst[i] = (uint16_t)analogRead(vbat_pin);
			uint32_t el = micros() - t0;
			if (el - target > lagMaxUs) lagMaxUs = el - target;
			durUs = el;
		}
		battProbeRelease();
		battProbePrintValues("BURST", c, burst, BATTPROBE_BURST_N);
		char extra[48];
		snprintf(extra, sizeof(extra), "|dur_ms=%lu|maxlag_us=%lu", (unsigned long)(durUs / 1000UL), (unsigned long)lagMaxUs);
		battProbePrintSummary("BSUM", c, burst, 50, BATTPROBE_BURST_N, extra);   // samples from t >= 100 ms (index 50)

		// ---- 2. PAIRS: 8 windows 500 ms apart: arm, wait 100 ms, 4 reads 1 ms apart, release ----
		uint16_t pairs[BATTPROBE_PAIRS][4];
		for (int k = 0; k < BATTPROBE_PAIRS; k++)
		{
			uint32_t w0 = millis();
			battDividerArm();
			battProbeWait(100);
			for (int r = 0; r < 4; r++)
			{
				pairs[k][r] = (uint16_t)analogRead(vbat_pin);
				if (r < 3)
					delay(1);
			}
			battProbeRelease();
			uint32_t used = millis() - w0;
			if (used < 500UL)
				battProbeWait(500UL - used);
		}
		for (int k = 0; k < BATTPROBE_PAIRS; k++)
		{
			float mean = (pairs[k][0] + pairs[k][1] + pairs[k][2] + pairs[k][3]) / 4.0F;
			printfdeb("[BATTPROBE]|PAIR|%d|%d|%u,%u,%u,%u|%.0f\n", c, k, (unsigned)pairs[k][0], (unsigned)pairs[k][1],
				(unsigned)pairs[k][2], (unsigned)pairs[k][3], mean * (float)ADC_MULTIPLIER);
			esp_task_wdt_reset();
		}

		// ---- 3. LONG: arm, divider on for 3 s, read every 100 ms (31 samples), release ----
		uint16_t lng[BATTPROBE_LONG_N];
		battDividerArm();
		uint32_t l0 = millis();
		for (int i = 0; i < BATTPROBE_LONG_N; i++)
		{
			uint32_t target = (uint32_t)i * 100UL;
			uint32_t el = millis() - l0;
			if (i > 0 && el < target)
				battProbeWait(target - el);
			lng[i] = (uint16_t)analogRead(vbat_pin);
		}
		battProbeRelease();
		battProbePrintValues("LONG", c, lng, BATTPROBE_LONG_N);
		battProbePrintSummary("LSUM", c, lng, 1, BATTPROBE_LONG_N, "");   // samples from t >= 100 ms (index 1)
	}

	battProbeRelease();   // belt and braces: divider off at exit
	s_battSchedInit = false;   // a READ pending from before the probe would read the released divider; re-arm
	printfdeb("[BATTPROBE]|END\n");
}

#else   // instrument build on a board without the switched Heltec divider

void battProbeRun(int cycles)
{
	(void)cycles;
	printfdeb("[BATTPROBE]|unsupported\n");
}

#endif   // BATT_FAMILY_HELTEC_SWITCHED
#endif   // INSTRUMENT_ENABLED

#endif
