/**
 * @file batt_functions.cpp
 * @author W.Zelinka (OE3WAS, https://github.com/karamo)
 * @brief
 * @version 0.6
 * @date 2026-06-09
 *
 * @copyright Copyright (c) 2026
 *
 */
#include "batt_functions.h"

#if defined(USE_NEW_BATT)

#ifdef USE_BATT

float max_batt = BAT_MAX_VOLTAGE;  //alt
float fBattMax = BAT_MAX_VOLTAGE;  //später extern

static bool firstReading = true;
float rawVoltage;
float BatVoltage;
static float filteredVoltage = 0.0f;
const float alpha = 0.05f;  // Glaettungsfaktor (0.05 = träger, 0.2 = schneller)
unsigned long batt_show_timer = 0;
int BATTshowtime;
#define CDcount 6
static int CountDown = CDcount;


// wird hier nicht verwendet, aber definiert, aber nicht freigegeben
float global_batt = 0;  // in mV
int global_proz = 0;
unsigned long BattTimeWait = 0;

//TODO: ev. weitere Definitionen für spezielle Boards ???
//...

#endif


void check_efuse(void)
{ 	// NOT TESTED, wird nicht benötigt
  printlndeb("[INIT]...efuse not used");
}


void VextON(void)
{
	#if defined(BOARD_WIRELESS_PAPER)
		pinMode(VEXT_ENABLE,OUTPUT);
		digitalWrite(VEXT_ENABLE, HIGH);
	#endif
	#if defined(BOARD_E290)
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
	#endif
	#if defined(BOARD_E290)
		pinMode(VEXT_ENABLE_1,OUTPUT);
		digitalWrite(VEXT_ENABLE_1, LOW);
		pinMode(VEXT_ENABLE_2,OUTPUT);
		digitalWrite(VEXT_ENABLE_2, LOW);
	#endif
}

void ADC_BATT_ON(void)
{
	#if defined(ADC_CTRL_PIN)
		pinMode(ADC_CTRL_PIN, OUTPUT);
		//Heltec V3.1 --- hat keine eigene variants !?!?
		#if defined(BOARD_HELTEC_V31) || defined(BOARD_WIRELESS_PAPER)
			digitalWrite(ADC_CTRL_PIN,LOW);   // active LOW: LOW = Teiler durchgeschaltet/messen
		#else
			digitalWrite(ADC_CTRL_PIN, HIGH);   // E213/E290: active HIGH (am Geraet verifiziert: LOW->0mV, HIGH->840mV)
		#endif
	#endif
}

void ADC_BATT_OFF(void)
{
	#if defined(ADC_CTRL_PIN)
		pinMode(ADC_CTRL_PIN, OUTPUT);
		//Heltec V3.1 --- hat keine eigene variants !?!?
		#if defined(BOARD_HELTEC_V31) || defined(BOARD_WIRELESS_PAPER)
			digitalWrite(ADC_CTRL_PIN,HIGH);
		#else
			digitalWrite(ADC_CTRL_PIN, LOW);   // E213/E290: active HIGH -> OFF = LOW
		#endif
	#endif
}


/**
 * @brief Initialize the battery analog input
 *
 */
void init_batt(void)
{
	#ifdef USE_BATT
		printlndeb("[INIT]...init_batt");
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

		ADC_BATT_ON();

		#if defined(BOARD_TBEAM) || defined(BOARD_SX1262) || defined(BOARD_SX1268)
		// XPOWERS_CHIP_AXP192 via I2C
		#endif

	#endif  // USE_BATT

	// allgemeine andere Aktionen

	#if defined(BOARD_E290)
		VextON();
	#endif

	// für Display am HELTEC V3/V4 und V3.2 --- gehört nicht unbedingt hier her
	#if defined(BOARD_HELTEC_V3) || defined(BOARD_HELTEC_V4) || defined(BOARD_STICK_V3)
		pinMode(36,OUTPUT);
		digitalWrite(36, LOW);
	#endif

	#if defined(BOARD_TLORA_OLV216)
		pinMode(23, OUTPUT);  // = LORA RESET - gehört nicht unbedingt hier her
	#endif

}  // init_batt



/**
 * @brief Read the analog value from the battery analog pin
 * and convert it to milli volt
 *
 * @return float Battery level in milli volts 0 ... 4200
 */
#if defined(BOARD_WIRELESS_PAPER)
// ----- "AKKU LOW"-Beobachtung (WP) -----
// Ringpuffer der letzten Spannungs-Rohwerte. read_batt() laeuft hier mit 2x/Sekunde -> 12 Werte = 6 s.
// bWpAkkuLow wird vor dem Low-Voltage-Deepsleep gesetzt; das WP-Display zeigt dann statt blank
// "AKKU LOW" + diese Werte (E-Ink haelt das Bild auch im Schlaf -> ablesbar). Die Hysterese
// (erst nach mehreren Low-Messungen schlafen) macht 0.6 selbst via CountDown.
// WP_VHIST_MAX ist zentral in batt_functions.h definiert (auch vom Anzeige-Aufrufer genutzt).
static float wpVHist[WP_VHIST_MAX];
static int   wpVHistCount = 0;
static int   wpVHistHead  = 0;
bool bWpAkkuLow = false;
static void wpPushVolt(float v)
{
    wpVHist[wpVHistHead] = v;
    wpVHistHead = (wpVHistHead + 1) % WP_VHIST_MAX;
    if(wpVHistCount < WP_VHIST_MAX) wpVHistCount++;
}
// Kopiert die letzten Werte NEUESTE ZUERST nach out[], liefert die Anzahl.
int wpBattHistory(float* out, int maxn)
{
    int n = (wpVHistCount < maxn) ? wpVHistCount : maxn;
    for(int i = 0; i < n; i++)
        out[i] = wpVHist[(wpVHistHead - 1 - i + 2 * WP_VHIST_MAX) % WP_VHIST_MAX];
    return n;
}
#endif

float read_batt(void)
{
	#ifdef USE_BATT

		// ist hier nicht redundant, da nach deepsleep ein sicherer Platz zum reaktivieren
		ADC_BATT_ON();

		// Messparameter aufbereiten
		// fBattFaktor = Parameter aus Flash
		// fBattMax = Parameter aus Flash
		BATTshowtime = (int)meshcom_settings.node_analog_batt_faktor / 1000;  // [--batt factor 99xxx.xxx]
		fBattFaktor = meshcom_settings.node_analog_batt_faktor - BATTshowtime*1000;  // [--batt factor x.xxx]
		if (fBattFaktor == 0.0) { fBattFaktor = 1.0; }
		if (BATTshowtime == 0) { BATTshowtime = 10; }  // default 10s
		fBattMax = meshcom_settings.node_maxv;  // [--maxv x.xxx]

		// Spezialbehandlung ungetestet
		#if defined(BOARD_HELTEC_T114) || defined(BOARD_T_ECHO) || defined(NRF52_SERIES)
			analogReference(AR_INTERNAL_3_0); // Set the analog reference to 3.0V (default = 3.6V)
			delay(5);
			analogSampleTime(10);	// Set the sampling time to 10us
		#endif

		// einfache Filterfunktion: exponentielle Glättung 1. Ordnung
		rawVoltage = (float)analogReadMilliVolts(BAT_VOLT_PIN)*BAT_MULTIPLIER/1000.0 * fBattFaktor + BAT_VOLT_OFFSET;
		if (firstReading) { filteredVoltage = fBattMax; } // verhindert deepsleep nach REBOOT
		else { filteredVoltage = alpha * rawVoltage + (1.0f - alpha) * filteredVoltage; }


		firstReading = false;

		#if defined(BOARD_WIRELESS_PAPER)
		wpPushVolt(rawVoltage);   // 2x/s -> letzte 10 Rohwerte fuer die "AKKU LOW"-Anzeige
		#endif

		if ((batt_show_timer + (1000 * std::max(1,BATTshowtime))) < millis())  // 1 .. 99s
		{
			batt_show_timer = millis();

			if(bDisplayCont)
			{
				bDEBUGLNG = true; // für den nächsten printfdeb language en/de aktivieren
				printfdeb("[BATT];%s;raw:;%.3f;V;max:;%.2f;V;fact:;%.4f;filt:;%.3f;V;%.0f;%%\n",
					getTimeString().c_str(), rawVoltage, fBattMax, fBattFaktor, filteredVoltage, mv_to_percent(filteredVoltage*1000.0));
			}
		}

		BatVoltage = filteredVoltage;

		// Board spezifische Modifikation
		#if defined(BOARD_E22)       // TODO: und auch die anderen E22 !!!
			if (BatVoltage < 3.0) { BatVoltage = 0; }	// ADC-Eingang nicht mit Versorgungsspannung verbunden
		#endif

		#if defined(BOARD_TBEAM_1W)
			// T-Beam 1W uses 7.4V 2S-battery (max. 8.1V)
			// USB-Spannung kann nicht gemessen werden, nur die AKKU-Spannung
			if(BatVoltage < 5.0) { BatVoltage = 0; }  // USB
		#endif

		// falls die Akku-Spannung BAT_MIN_VOLTAGE erreicht wird, soll ein --deepsleep erfolgen.
		// Dieses erlaubt es, nach einem händischen RESET zum Aufwecken noch kurz nachzusehen,
		// da sich der Akku auch etwas erholt.
		// E213: Akku-Messung am Geraet verifiziert 2026-06-23 (Faktor 4.9245, ADC_CTRL active HIGH):
		// Voll ~4.14 V, Leer-Cutoff ~3.26 V (unter Last; im Boot ~3.5 V = Last-Sag bei leerem LiPo).
		// Low-voltage-Deepsleep wieder scharf wie bei allen anderen Boards. BAT_MIN_VOLTAGE = 3.3 V
		// loest knapp vor dem 3.26-V-Cutoff aus; der firstReading-Seed (filteredVoltage = fBattMax)
		// verhindert den Boot-Deepsleep beim Laden.

		/* issue #1053 Had to remove battery, boot with USB, change max. Voltage from 4.2 to 8.2 
		if ((BatVoltage <= (BAT_MIN_VOLTAGE)) && (BatVoltage > 1.0))  // 6.5V für T-Beam 1W, 3.3V für andere Boards
		{
			CountDown--;
			if (CountDown == 0) {
				if(bDisplayCont)
				{
					bDEBUGLNG = true; // für den nächsten printfdeb language en/de aktivieren
					printfdeb("[BATT];%s;raw:;%.3f;V;max:;%.2f;V;fact:;%.4f;filt:;%.3f;V;%.0f;%%\n",
						getTimeString().c_str(), rawVoltage, fBattMax, fBattFaktor, filteredVoltage, mv_to_percent(filteredVoltage*1000.0));
				}

				// Abschaltmeldung ausgeben
				printlndeb("[ERR]...low Voltage Accu > goto deepsleep");

				delay(1000); // für Ausgabe ermöglichen !!!

				#if defined(BOARD_T_ECHO)   // = NRF52 --- ungetestet
					digitalWrite(Power_On_Pin, LOW);
					//boardPWROff();  // nrf52_functions
				#else
					ADC_BATT_OFF();
					// Andere Boards / Original: Display regulaer ausschalten (persistiert node_sset).
					commandAction((char*)"--display off", isPhoneReady, false);
					#if defined(BOARD_WIRELESS_PAPER)
					bWpAkkuLow = true;   // WP-Display zeigt "AKKU LOW" + letzte Werte statt blank
					#endif
					commandAction((char*)"--deepsleep", isPhoneReady, false);
				#endif
				// Node stopped
			}

		} else {
			CountDown = CDcount; // retrigger
		}
		*/

		// wenn keine AKKU am BATT PIN ist immmer 0V aber 100% ausgeben
		if(BatVoltage < 1.0) { BatVoltage = 0; }

		return BatVoltage*1000.0;  // [mV]

	//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	//=====================================================================================
	#else
		//---------------------------------------------------------------------------
		#if defined(BOARD_HELTEC_T114)
		//TODO: da das KEINE ESP32 ist, ist eine Spezialbehandlung erforderlich !!!
			// ... analogReadMilliVolts(BAT_VOLT_PIN) ...
			BatVoltage = rawVoltage * 3.589;
		#endif

		//---------------------------------------------------------------------------
		#if defined(BOARD_T_ECHO)
		//TODO: da das KEINE ESP32 ist, ist eine Spezialbehandlung erforderlich !!!
			#define VBAT_MV_PER_LSB   (0.73242188F)   // 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096
			#define VBAT_DIVIDER      (0.71275837F)   // 2M + 0.806M voltage divider on VBAT = (2M / (0.806M + 2M))
			#define VBAT_DIVIDER_COMP (1.403F)        // Compensation factor for the VBAT divider
			// Convert the raw value to compensated mv, taking the resistor-divider into account (providing the actual LIPO voltage)
			// ADC range is 0..3000mV and resolution is 12-bit (0..4095)
			BatVoltage =  rawVoltage * VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB;
		#endif
		//---------------------------------------------------------------------------

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
  if (rproz < 0.0) { rproz = 0.0; }
	return round(rproz);

#else

	return 0.0;

#endif
}

#endif  // USE_NEW_BATT
