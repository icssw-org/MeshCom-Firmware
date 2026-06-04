/**
 * @file batt_functions.cpp
 * @author W.Zelinka (OE3WAS, https://github.com/karamo)
 * @brief 
 * @version 0.4
 * @date 2026-05-13
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#include <configuration.h>

#if defined(USE_NEW_BATT)

#include "batt_functions.h"

#ifdef USE_BATT

float max_batt = BAT_MAX_VOLTAGE;  //alt
float fBattMax = BAT_MAX_VOLTAGE;  //später extern

static bool firstReading = true;
float rawVoltage;
float BatVoltage;
static float filteredVoltage = 0.0f;
const float alpha = 0.1f;  // Glaettungsfaktor (0.05 = träger, 0.2 = schneller)
unsigned long batt_show_timer = 0;
int BATTshowtime;

// wird hier nicht verwendet, aber definiert, aber nicht freigegeben
float global_batt = 0;  // in mV
int global_proz = 0;
unsigned long BattTimeWait = 0;

//TODO: ev. weitere Definitionen für spezielle Boards ???
//...

#endif


void check_efuse(void) { 	// NOT TESTED, wird nicht benötigt
  Serial.printf("[INIT]...efuse not used\n");
}


#if defined(BOARD_E290) || defined(BOARD_WIRELESS_PAPER)
	void VextON(void)
	{
		#if defined(BOARD_WIRELESS_PAPER)
			pinMode(VEXT_ENABLE,OUTPUT);
			digitalWrite(VEXT_ENABLE, HIGH);
		#else
			pinMode(VEXT_ENABLE_1,OUTPUT);
			digitalWrite(VEXT_ENABLE_1, HIGH);
			pinMode(VEXT_ENABLE_2,OUTPUT);
			digitalWrite(VEXT_ENABLE_2, HIGH);
		#endif
	}

	void VextOFF(void)  // Vext default OFF
	{
		#if defined(BOARD_WIRELESS_PAPER)
			pinMode(VEXT_ENABLE,OUTPUT);
			digitalWrite(VEXT_ENABLE, LOW);
		#else
			pinMode(VEXT_ENABLE_1,OUTPUT);
			digitalWrite(VEXT_ENABLE_1, LOW);
			pinMode(VEXT_ENABLE_2,OUTPUT);
			digitalWrite(VEXT_ENABLE_2, LOW);
		#endif
	}
#endif



/**
 * @brief Initialize the battery analog input
 *
 */
void init_batt(void)
{
#ifdef USE_BATT
Serial.printf("[INIT]...init_batt\n");
	firstReading = true;

	// nach Änderung durch Befehl in command_functions.cpp muss init_batt() aufgerufen werden!
	BATTshowtime = (int)meshcom_settings.node_analog_batt_faktor / 1000;  // [--batt factor 99xxx.xxx]
	fBattFaktor = meshcom_settings.node_analog_batt_faktor - BATTshowtime*1000;  // [--batt factor x.xxx]
	if (fBattFaktor == 0.0) { fBattFaktor = 1.0; }
	if (BATTshowtime == 0) { BATTshowtime = 10; }
	fBattMax = meshcom_settings.node_maxv;  // [--maxv x.xxx]
	// -----

	//analogSetPinAttenuation(BAT_VOLT_PIN, ADC_11db);  // alternative Variante
	analogSetAttenuation(BAT_ATTEN);
	analogReadResolution(BAT_WIDTH);

	#if defined(ADC_CTRL_PIN)
		pinMode(ADC_CTRL_PIN, OUTPUT);
		//Heltec V3.1 --- hat keine eigene variants !?!?
		#if defined(BOARD_HELTEC_V31) || defined(BOARD_WIRELESS_PAPER)
			digitalWrite(ADC_CTRL_PIN,LOW);
		#else
			digitalWrite(ADC_CTRL_PIN, HIGH);
		#endif
	#endif

	#if defined(BOARD_TBEAM) || defined(BOARD_SX1262) || defined(BOARD_SX1268)
	// XPOWERS_CHIP_AXP192 via I2C
	#endif

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//=====================================================================================
#else
// hier sind noch alle Boards mit Spezialbehandlung,
// die noch eingearbeitet werden müssen, noch klären

	#if defined(BOARD_TLORA_OLV216)
		pinMode(23, OUTPUT);  // = LORA RESET gehört hier nicht her
	#endif

	// für Display am HELTEC V3/V4 und V3.2 --- gehört nicht hier her
	#if defined(BOARD_HELTEC_V3) || defined(BOARD_HELTEC_V4) || defined(BOARD_STICK_V3)
		pinMode(36,OUTPUT);
		digitalWrite(36, LOW);
	#endif

	// für Display u.a. hat nichts mit BATT zu tun --- --- gehört nicht hier her
	#if defined(BOARD_E290)
		VextON();
	#endif

	//---------------------------------------------------------------------------
	#if defined(BOARD_HELTEC_T114)
	//TODO: da das KEINE ESP32, ist eine Spezialbehandlung erforderlich !!!
		// ... analogReadMilliVolts(BAT_VOLT_PIN) ...
		mVBatt = (uint16_t)((float)adcvalue * 3.589);
	#endif

	//---------------------------------------------------------------------------
	#if defined(BOARD_T_ECHO)
	//TODO: da das KEINE ESP32, ist eine Spezialbehandlung erforderlich !!!
		#define VBAT_MV_PER_LSB   (0.73242188F)   // 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096
		#define VBAT_DIVIDER      (0.71275837F)   // 2M + 0.806M voltage divider on VBAT = (2M / (0.806M + 2M))
		#define VBAT_DIVIDER_COMP (1.403F)        // Compensation factor for the VBAT divider
		// Convert the raw value to compensated mv, taking the resistor-divider into account (providing the actual LIPO voltage)
		// ADC range is 0..3000mV and resolution is 12-bit (0..4095)
		mVBatt =  raw * VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB;
	#endif

#endif
}  // init_batt



/**
 * @brief Read the analog value from the battery analog pin
 * and convert it to milli volt
 *
 * @return float Battery level in milli volts 0 ... 4200
 */
float read_batt(void)
{
#ifdef USE_BATT

	char Buffer[100];

	#if defined(ADC_CTRL_PIN)
		//Heltec V3.1 --- hat keine eigene variants !?!?
		#if defined(BOARD_HELTEC_V31) || defined(BOARD_WIRELESS_PAPER)
			digitalWrite(ADC_CTRL_PIN,LOW);
		#else
			digitalWrite(ADC_CTRL_PIN, HIGH);
		#endif
		delay(100);
	#endif

	// fall nach Änderung durch Befehl in command_functions.cpp init_batt() aufgerufen wird,
	// kann dieser Abschnitt hier entfallen!
	BATTshowtime = (int)meshcom_settings.node_analog_batt_faktor / 1000;  // [--batt factor 99xxx.xxx]
	fBattFaktor = meshcom_settings.node_analog_batt_faktor - BATTshowtime*1000;  // [--batt factor x.xxx]
	if (fBattFaktor == 0.0) { fBattFaktor = 1.0; }
	if (BATTshowtime == 0) { BATTshowtime = 10; }  // default 10s
	fBattMax = meshcom_settings.node_maxv;  // [--maxv x.xxx]
	// ======

	#if defined(BOARD_HELTEC_T114) || defined(BOARD_T_ECHO) || defined(NRF52_SERIES)
		analogReference(AR_INTERNAL_3_0); // Set the analog reference to 3.0V (default = 3.6V)
		delay(5);
		analogSampleTime(10);	// Set the sampling time to 10us
	#endif

	// fBattFaktor = Parameter aus Flash
	// fBattMax = Parameter aus Flash
	// einfache Filterfunktion: exponentielle Glättung 1. Ordnung
	rawVoltage = (float)analogReadMilliVolts(BAT_VOLT_PIN)*BAT_MULTIPLIER/1000.0 * fBattFaktor + BAT_VOLT_OFFSET;
	if (firstReading) { filteredVoltage = rawVoltage; firstReading = false; }
	else { filteredVoltage = alpha * rawVoltage + (1.0f - alpha) * filteredVoltage; }

	#if defined(ADC_CTRL_PIN)
		//Heltec V3.1 --- hat keine eigene variants !?!?
		#if defined(BOARD_HELTEC_V31)
			digitalWrite(ADC_CTRL_PIN,HIGH);
		#else
			digitalWrite(ADC_CTRL_PIN, LOW);
		#endif
	#endif


	if ((batt_show_timer + (1000 * std::max(1,BATTshowtime))) < millis())  // 1 .. 99s
	{
		batt_show_timer = millis();

		snprintf(Buffer, sizeof(Buffer), "[BATT];%s; raw: ;%.3f; V max: ;%.2f; V fact: ;%.4f; filt: ;%.3f; V ;%.0f; %%",
			getTimeString().c_str(), rawVoltage, fBattMax, fBattFaktor, filteredVoltage, mv_to_percent(filteredVoltage*1000.0));
		
		// durchlaufe Array, bis Ende-Zeichen '\0' und '.' => ',' für deutsche CSV-Kompatibilität
		for (int i = 0; Buffer[i] != '\0'; i++) { if (Buffer[i] == '.') { Buffer[i] = ','; } }

		Serial.printf("%s\n",Buffer);
	}

	BatVoltage = filteredVoltage;

	// Board spezifische Modifikation
	#if defined(BOARD_TBEAM_1W)
		// T-Beam 1W uses 7.4V 2S-battery (max. 8.1V)
		// USB-Spannung kann nicht gemessen werden, nur die AKKU-Spannung
		if(BatVoltage < 5.0) { BatVoltage = 0; }  // USB
	#endif

	// falls die Akku-Spannung BAT_MIN_VOLTAGE erreicht wird, sollte ein --deepsleep erfolgen.
	// Dieses erlaubt es, nach einem händischen RESET zum Aufwecken noch kurz nachzusehen,
	// da sich der Akku auch etwas erholt.
	if (BatVoltage <= (BAT_MIN_VOLTAGE) && BatVoltage > 1.0)  // 6.5V für T-Beam 1W, 3.3V für andere Boards
	{
		// letzte Ausgabe erzwingen
		snprintf(Buffer, sizeof(Buffer), "[BATT];%s; raw: ;%.3f; V max: ;%.2f; V fact: ;%.4f; filt: ;%.3f; V ;%.0f; %%",
			getTimeString().c_str(), rawVoltage, fBattMax, fBattFaktor, filteredVoltage, mv_to_percent(filteredVoltage*1000.0));
		// Abschaltmeldung ausgeben
		Serial.print("[ERR]...low Voltage Accu > goto deepsleep\n");
		delay(1000); // für Ausgabe ermöglichen !!!

		#if defined(BOARD_T_ECHO)   // = NRF52
			digitalWrite(Power_On_Pin, LOW);
			//boardPWROff();  // nrf52_functions
		#else
			commandAction((char*)"--display off", isPhoneReady, false);
			commandAction((char*)"--deepsleep", isPhoneReady, false);
		#endif
	}

	// wenn keine AKKU am BATT PIN ist immmer 0V aber 100% ausgeben
	if(BatVoltage < 1.0) { BatVoltage = 0; }

	return BatVoltage*1000.0;  // [mV]

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//=====================================================================================
#else

	return 0.0;

#endif
}  // read_batt


//=====================================================================================

/**
 * @brief Set the Max Batt object
 * @todo genauso wie fBattFaktor behandeln und in main
 * 
 * @param u_max_batt [mV]
 */
void setMaxBatt(float u_max_batt)
{
#ifdef USE_BATT
	max_batt = u_max_batt/1000.0;
	fBattMax = u_max_batt/1000.0; // ev. nach main auslagern
#endif
}


/**
 * @brief Volt => Prozent Umrechnung über lineare Näherung
 * @note max_batt = Parameter aus Flash
 * 
 * @param mvolts [mV]
 * @return rproz
 */
float mv_to_percent(float mvolts)
{
#ifdef USE_BATT

	// USB - Versorgung
	if(mvolts < 1000.0) { return 100.0; }

	// fBattMax = Parameter aus Flash
  	float rproz = (mvolts/1000.0 - BAT_MIN_VOLTAGE)/(fBattMax - BAT_MIN_VOLTAGE) *100.0;
  	if (rproz > 100.0) { rproz = 100.0; }

	return round(rproz);

#else

	return 0.0;

#endif
}

#endif