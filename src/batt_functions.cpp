/**
 * @file batt_functions.cpp
 * @author W.Zelinka (OE3WAS, https://github.com/karamo)
 * @brief Battery path of the USE_NEW_BATT boards (E22 family, loraprs-e22, T-Beam 1W, T3-S3,
 *        T-Deck, T-Deck Plus, lora32-v21, E213, Wireless Paper) on the shared pipeline.
 * @version 0.7
 * @date 2026-09-29
 *
 * Wave 3 of the battery consolidation (docs/archive/concept-battery-consolidation-20260923.md,
 * contract in docs/ble-batt-campaign-20260929.md): EMA, settle rule, BAT-01 detector, percent
 * curve and the sampling scheduler come from src/batt_pipeline.h. Only the board specific part
 * stays here: reading one raw value, the ADC_CTRL polarity probe and the divider switch.
 *
 * @copyright Copyright (c) 2026
 *
 */
#include "batt_functions.h"

#if defined(USE_NEW_BATT)

#if defined(BAT_MAX_VOLTAGE)
	#define BATT_MAX_DEFAULT_V  ((float)(BAT_MAX_VOLTAGE))
#else
	#define BATT_MAX_DEFAULT_V  4.1f   // native host build (no variant configuration.h)
#endif

float max_batt = BATT_MAX_DEFAULT_V;  //alt
float fBattMax = BATT_MAX_DEFAULT_V;  //später extern [V]

#ifdef USE_BATT

unsigned long batt_show_timer = 0;
int BATTshowtime;

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


// Compile-Zeit-Fallback, bis die Probe (falls ADC_CTRL_PIN vorhanden) gelaufen ist bzw. auf
// Boards ohne ADC_CTRL_PIN dauerhaft: Wireless Paper ist als active LOW dokumentiert, alle
// anderen (E213/E290) als active HIGH ("am Geraet verifiziert").
#if defined(BOARD_WIRELESS_PAPER)
batt_probe_t battProbeState = BATT_PROBE_ACTIVE_LOW;
#else
batt_probe_t battProbeState = BATT_PROBE_ACTIVE_HIGH;
#endif


// ----- sample core: everything after "one raw value" (host testable) -----
// State of the one VBAT channel of this node. EMA/detector/percent logic is batt_pipeline.h;
// this block only wires it together. No Arduino calls, see battFeedSample() in batt_functions.h.
static batt_ema_t battEma;
static bool       battPipelineReady = false;
static bool       battWasAbsent     = false;   // detector said "absent" since the last EMA seed
static bool       battHaveSample    = false;   // at least one sample went through battFeedSample()
static float      battReportedMv    = 0.0f;    // last value read_batt() reported (0 = no reading)
static uint32_t   battSamples       = 0;       // real ADC samples, see battSampleCount()

void battPipelineReset(void)
{
	battEmaInit(&battEma, BATT_EMA_TAU_MS_DEFAULT);
	battDetectGlobalReset();
	battWasAbsent  = false;
	battHaveSample = false;
	battReportedMv = 0.0f;
	battPipelineReady = true;
}

uint32_t battSampleCount(void)
{
	return battSamples;
}

float battFeedSample(float rawMv, float maxMv, uint32_t nowMs)
{
	return battFeedSampleSpread(rawMv, BATT_DETECT_SPREAD_NONE, maxMv, nowMs);
}

float battFeedSampleSpread(float rawMv, float spreadMv, float maxMv, uint32_t nowMs)
{
	if (!battPipelineReady) { battPipelineReset(); }

	battSamples++;
	battHaveSample = true;

	// BAT-01: presence on the RAW sample. Plausible band relative to the pack maximum, so the
	// 2S packs (TBEAM_1W, E22) on this path are covered too (see batt_pipeline.h, section 3).
	// BAT-03: spreadMv (window spread, switched dividers) above the limit counts as implausible.
	const bool present = battDetectFeedSpread(rawMv, spreadMv,
		maxMv*BATT_DETECT_MIN_BAND_FACTOR, maxMv*BATT_DETECT_MAX_BAND_FACTOR);

	if (!present)
	{
		// Floating divider: samples are noise, keep them out of the EMA. The value comes back
		// as a fresh seed (with a restarted settle rule) once the detector sees a cell again.
		battWasAbsent  = true;
		battReportedMv = 0.0f;
		return 0.0f;
	}

	if (battWasAbsent)
	{
		battEmaInit(&battEma, BATT_EMA_TAU_MS_DEFAULT);
		battWasAbsent = false;
	}

	float mv = battEmaUpdate(&battEma, rawMv, nowMs);

	// "no reading" = 0 mV (USB / no cell): T-Deck header, "USB" on the displays, /B= suppression.
	if (mv < 1000.0f) { mv = 0.0f; }   // ADC input not connected to the supply

	// Board specific modifications
	#if defined(BOARD_E22)       // TODO: und auch die anderen E22 !!!
		if (mv < 3000.0f) { mv = 0.0f; }	// ADC-Eingang nicht mit Versorgungsspannung verbunden
	#endif

	#if defined(BOARD_TBEAM_1W)
		// T-Beam 1W uses 7.4V 2S-battery (max. 8.1V)
		// USB-Spannung kann nicht gemessen werden, nur die AKKU-Spannung
		if (mv < 5000.0f) { mv = 0.0f; }  // USB
	#endif

	battReportedMv = mv;
	return mv;
}

float battFilteredMv(void)
{
	return battEma.seeded ? battEma.value : 0.0f;
}

bool battSettled(void)
{
	return battPipelineReady && battEmaSettled(&battEma);
}

bool battLowVoltage(float thresholdMv)
{
	// Only through battEmaLowVoltage(): false until the EMA is settled (bad first sample, boot on
	// a sagging cell), 1 V floor keeps "no battery / USB only" out. Never while the detector says
	// "no battery" (floating pin, issue #1053).
	return battPipelineReady && battDetected() && battEmaLowVoltage(&battEma, thresholdMv, 1000.0f);
}


bool battHardwarePresent(void)
{
	// fail-safe: nur bei positiv erkanntem "kein Teiler" (Probe), positiv erkannter Abwesenheit
	// (Laufzeit-Detektion, BAT-01) oder einem gemeldeten "no reading" (0 mV = USB, kein Akku)
	// false. Vor dem ersten Sample bleibt es beim fail-safe "true". "no reading" muss hier
	// mitzaehlen: mv_to_percent(0) ist 0, und "/B=000" hiesse sonst "Akku leer" statt "kein Akku".
	return battProbeState != BATT_PROBE_NONE && battDetected()
		&& !(battHaveSample && battReportedMv <= 0.0f);
}

#if defined(ADC_CTRL_PIN)
// battProbeState startet bewusst NICHT auf BATT_PROBE_UNKNOWN (siehe oben), daher braucht das
// "einmalig ausfuehren"-Gating ein eigenes Flag statt eines Vergleichs gegen battProbeState.
// Nur hier deklariert: ohne ADC_CTRL_PIN gibt es keine Probe und das Flag waere ungenutzt.
static bool battProbeDone = false;

// Einmalige Polaritaets-Probe (siehe Begruendung in batt_functions.h). Wird lazy beim ersten
// ADC_BATT_ON() aufgerufen (also beim Boot, aus init_batt()) und danach nie wieder (battProbeDone).
static void battProbeADCPolarity(void)
{
	int countsHigh = 0;
	int countsLow  = 0;

	digitalWrite(ADC_CTRL_PIN, HIGH);
	delay(100);   // Teiler braucht ~100ms zum Einschwingen (wie an anderer Stelle bereits verwendet)
	for (int i = 0; i < 8; i++) { countsHigh += analogRead(BAT_VOLT_PIN); }
	countsHigh /= 8;

	digitalWrite(ADC_CTRL_PIN, LOW);
	delay(100);
	for (int i = 0; i < 8; i++) { countsLow += analogRead(BAT_VOLT_PIN); }
	countsLow /= 8;

	int probeDelta = countsHigh - countsLow;
	if (probeDelta < 0) probeDelta = -probeDelta;
	if (countsHigh >= BATT_PROBE_MIN_COUNTS && countsLow >= BATT_PROBE_MIN_COUNTS && probeDelta < BATT_PROBE_MIN_COUNTS)
	{
		// Beide Messungen plausibel und praktisch gleich: der Teiler liegt fest an, der
		// Steuerpin bewirkt nichts (z.B. Wireless Stick V3). Batteriehardware vorhanden,
		// Polaritaet ohne Bedeutung -> nicht als "kein Teiler" fehlinterpretieren.
		battProbeState = BATT_PROBE_ACTIVE_HIGH;
		digitalWrite(ADC_CTRL_PIN, LOW);
	}
	else if (countsHigh >= BATT_PROBE_MIN_COUNTS && countsHigh > countsLow)
	{
		battProbeState = BATT_PROBE_ACTIVE_HIGH;
		digitalWrite(ADC_CTRL_PIN, LOW);    // Ruhezustand: Teiler getrennt (Strom sparen)
	}
	else if (countsLow >= BATT_PROBE_MIN_COUNTS && countsLow > countsHigh)
	{
		battProbeState = BATT_PROBE_ACTIVE_LOW;
		digitalWrite(ADC_CTRL_PIN, HIGH);   // Ruhezustand: Teiler getrennt (Strom sparen)
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

#if defined(ADC_CTRL_PIN)
// BAT-01 Nebenbefund: verhindert ein woertliches delay() bei jedem read_batt()-Zyklus
// (siehe battDividerOn() unten) -- nur der tatsaechliche AUS->AN-Wechsel muss einschwingen.
static bool battDividerSettled = false;

// Teiler durchschalten. settle=true (ADC_BATT_ON(), Boot/Deepsleep-Aufwachen): blockiert
// beim AUS->AN-Wechsel kurz, damit der erste ADC-Read nicht waehrend des Einschwingens
// passiert. settle=false (Scheduler-ARM): der Scheduler wartet selbst BATT_SCHED_SETTLE_MS
// bis zum READ, kein delay() im Hot Path.
static void battDividerOn(bool settle)
{
	pinMode(ADC_CTRL_PIN, OUTPUT);

	if (!battProbeDone)
	{
		battProbeADCPolarity();   // einmalig: Polaritaet des Teiler-Schalters ermitteln
		battProbeDone = true;
	}

	if (battProbeState == BATT_PROBE_ACTIVE_LOW)
		digitalWrite(ADC_CTRL_PIN, LOW);    // active LOW: LOW = Teiler durchgeschaltet/messen (z.B. Wireless Paper)
	else
		digitalWrite(ADC_CTRL_PIN, HIGH);   // active HIGH (Default/Fallback): E213/E290 am Geraet verifiziert

	if (!battDividerSettled)
	{
		if (settle) { delay(20); }
		battDividerSettled = true;
	}
}
#endif

void ADC_BATT_ON(void)
{
	#if defined(ADC_CTRL_PIN)
		battDividerOn(true);
	#endif
}


void ADC_BATT_OFF(void)
{
	#if defined(ADC_CTRL_PIN)
		pinMode(ADC_CTRL_PIN, OUTPUT);

		if (battProbeState == BATT_PROBE_ACTIVE_LOW)
			digitalWrite(ADC_CTRL_PIN, HIGH);   // active LOW -> OFF = HIGH
		else
			digitalWrite(ADC_CTRL_PIN, LOW);    // active HIGH (Default/Fallback) -> OFF = LOW

		battDividerSettled = false;   // naechstes ADC_BATT_ON() ist wieder ein AUS->AN-Wechsel
	#endif
}

#if defined(BOARD_WIRELESS_PAPER)
// ----- "AKKU LOW"-Beobachtung (WP) -----
// Ringpuffer der letzten Spannungs-Rohwerte, einer pro echtem Sample (READ des geschalteten
// Teilers, alle 30 s) -> 12 Werte = 6 min.
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

// ----- USE_BATT: acquisition (one raw value) and scheduling -----
#ifdef USE_BATT

// Divider with an enable pin (E213, Wireless Paper, ADC_CTRL_PIN): SWITCHED profile, the scheduler
// arms the divider, waits for it to settle, reads once and releases it (ARM every 30 s). Every other
// board has the divider permanently connected: FIXED profile, one READ per second.
#if defined(ADC_CTRL_PIN)
	#define BATT_SCHED_PROFILE_BOARD  BATT_SCHED_PROFILE_SWITCHED
#else
	#define BATT_SCHED_PROFILE_BOARD  BATT_SCHED_PROFILE_FIXED
#endif

static batt_sched_t battSched;
static bool battSchedReady = false;

static void battSchedSetup(void)
{
	battSchedInit(&battSched, BATT_SCHED_PROFILE_BOARD);
	battSchedReady = true;
}

// Messparameter aufbereiten (nach Aenderung per Befehl in command_functions.cpp wirksam)
// fBattFaktor = Parameter aus Flash [--batt factor 99xxx.xxx]
// fBattMax    = Parameter aus Flash [--maxv x.xxx]
static void battLoadParams(void)
{
	BATTshowtime = (int)meshcom_settings.node_analog_batt_faktor / 1000;  // [--batt factor 99xxx.xxx]
	fBattFaktor = meshcom_settings.node_analog_batt_faktor - BATTshowtime*1000;  // [--batt factor x.xxx]
	if (fBattFaktor == 0.0) { fBattFaktor = 1.0; }
	if (BATTshowtime == 0) { BATTshowtime = 10; }  // default 10s
	fBattMax = meshcom_settings.node_maxv;  // [--maxv x.xxx]
}

#if defined(BATT_LOW_VOLTAGE_DEEPSLEEP)
// Low-voltage deep sleep, DISABLED since issue #1053 (had to remove the battery, boot on USB, change
// max. voltage from 4.2 to 8.2 -> floating pin read low -> deep sleep). Define
// BATT_LOW_VOLTAGE_DEEPSLEEP to re-enable. It now decides on the settled EMA only (battLowVoltage():
// battery present per BAT-01, settled = 3 tau and 8 samples, 1 V floor), not on a single sample.
// BAT_MIN_VOLTAGE: 6.5 V for T-Beam 1W, 3.3 V for the others. E213: Voll ~4.14 V, Leer-Cutoff ~3.26 V
// (unter Last), BAT_MIN_VOLTAGE = 3.3 V loest knapp davor aus (am Geraet verifiziert 2026-06-23).
static void battLowVoltageCheck(void)
{
	if (!battLowVoltage(BAT_MIN_VOLTAGE*1000.0f)) { return; }

	// Abschaltmeldung ausgeben
	printlndeb("[ERR]...low Voltage Accu > goto deepsleep");

	delay(1000); // für Ausgabe ermöglichen !!!

	ADC_BATT_OFF();
	// Display regulaer ausschalten (persistiert node_sset).
	commandAction((char*)"--display off", isPhoneReady, false);
	#if defined(BOARD_WIRELESS_PAPER)
	bWpAkkuLow = true;   // WP-Display zeigt "AKKU LOW" + letzte Werte statt blank
	#endif
	commandAction((char*)"--deepsleep", isPhoneReady, false);
	// Node stopped
}
#endif

// One real ADC sample: raw value incl. --batt factor and multiplier -> pipeline (detector on the
// raw value, EMA) -> cached value. Debug output only here, i.e. once per sample.
static void battTakeSample(uint32_t now)
{
	battLoadParams();

	const float rawVoltage = (float)analogReadMilliVolts(BAT_VOLT_PIN)*BAT_MULTIPLIER/1000.0 * fBattFaktor + BAT_VOLT_OFFSET;

	#if defined(ADC_CTRL_PIN)
		// BAT-03: without a cell the divider sits on the charger-output sawtooth, one read lands at a
		// random phase. Read a short window (first read = rawVoltage above), spread over all reads
		// goes to the detector. Window is taken BEFORE the divider is released.
		float windowMv[BATT_DETECT_WINDOW_READS];
		windowMv[0] = rawVoltage*1000.0f;
		for (int i = 1; i < BATT_DETECT_WINDOW_READS; i++)
		{
			delay(BATT_DETECT_WINDOW_STEP_MS);
			const float v = (float)analogReadMilliVolts(BAT_VOLT_PIN)*BAT_MULTIPLIER/1000.0 * fBattFaktor + BAT_VOLT_OFFSET;
			windowMv[i] = v*1000.0f;
		}
		const float windowSpreadMv = battWindowSpread(windowMv, BATT_DETECT_WINDOW_READS);

		ADC_BATT_OFF();   // SWITCHED: release the divider right after the reads (no drain through it)

		battFeedSampleSpread(rawVoltage*1000.0f, windowSpreadMv, fBattMax*1000.0f, now);
	#else
		battFeedSample(rawVoltage*1000.0f, fBattMax*1000.0f, now);
	#endif

	#if defined(BOARD_WIRELESS_PAPER)
	wpPushVolt(rawVoltage);   // letzte Rohwerte fuer die "AKKU LOW"-Anzeige
	#endif

	if ((uint32_t)(millis() - batt_show_timer) >= (uint32_t)(1000 * std::max(1,BATTshowtime)))  // 1 .. 99s
	{
		batt_show_timer = millis();

		if(bDisplayCont)
		{
			bDEBUGLNG = true; // für den nächsten printfdeb language en/de aktivieren
			#if defined(ADC_CTRL_PIN)
			printfdeb("[BATT];%s;raw:;%.3f;V;max:;%.2f;V;fact:;%.4f;filt:;%.3f;V;%.0f;%%;spread:;%.0f;mV\n",
				getTimeString().c_str(), rawVoltage, fBattMax, fBattFaktor, battFilteredMv()/1000.0f, mv_to_percent(battFilteredMv()), windowSpreadMv);
			#else
			printfdeb("[BATT];%s;raw:;%.3f;V;max:;%.2f;V;fact:;%.4f;filt:;%.3f;V;%.0f;%%\n",
				getTimeString().c_str(), rawVoltage, fBattMax, fBattFaktor, battFilteredMv()/1000.0f, mv_to_percent(battFilteredMv()));
			#endif
		}
	}

	#if defined(BATT_LOW_VOLTAGE_DEEPSLEEP)
	battLowVoltageCheck();
	#endif
}

#endif  // USE_BATT


/**
 * @brief Initialize the battery analog input
 *
 */
void init_batt(void)
{
	#ifdef USE_BATT
		printlndeb("[INIT]...init_batt");

		// nach Änderung durch Befehl in command_functions.cpp muss init_batt() aufgerufen werden!
		battLoadParams();
		battPipelineReset();   // EMA reseeds with the first sample, detector back to fail-safe "present"
		battSchedSetup();
		// -----

		//analogSetPinAttenuation(BAT_VOLT_PIN, ADC_11db);  // alternative Variante
		analogSetAttenuation(BAT_ATTEN);
		analogReadResolution(BAT_WIDTH);

		ADC_BATT_ON();   // runs the ADC_CTRL polarity probe once

		#if defined(ADC_CTRL_PIN)
		// SWITCHED divider: first sample now, so read_batt() has a value from the start and does not
		// report "no reading" between the first ARM and its READ (~100 ms). ADC_BATT_ON() above only
		// waits 20 ms; the scheduler's settle time is used here as well.
		delay(BATT_SCHED_SETTLE_MS);
		battTakeSample(millis());   // releases the divider again
		#endif

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
 * @brief Battery level, filtered, in milli volts. Called from the main loop about every 100 ms
 * (loopAction_battCheck); the scheduler decides when a real sample is taken (FIXED: every 1 s,
 * SWITCHED: ARM every 30 s, READ ~100 ms later), all other calls return the cached value.
 *
 * @return float filtered battery level in mV (0 ... 4200 / 8400); 0 = no reading (USB / no cell)
 */
float read_batt(void)
{
	#ifdef USE_BATT

		if (!battSchedReady) { battSchedSetup(); }

		const uint32_t now = millis();

		switch (battSchedTick(&battSched, now))
		{
			case BATT_SCHED_ARM:
				#if defined(ADC_CTRL_PIN)
					battDividerOn(false);   // scheduler waits for the divider to settle
				#endif
				break;

			case BATT_SCHED_READ:
				battTakeSample(now);
				break;

			default:   // BATT_SCHED_NONE: cached value
				break;
		}

		return battReportedMv;   // [mV], 0 = no reading

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
#else
	(void)u_max_batt;
#endif
}


/**
 * @brief Volt => Prozent, one curve for 1S and 2S (batt_pipeline.h battPercent())
 * @note fBattMax = Parameter aus Flash [V]. 0 mV = "no reading" (USB / no cell) -> 0.
 *       Callers show "USB" for global_batt == 0, the percent is not shown then.
 *
 * @param mvolts [mV]
 * @return percent 0 ... 100
 */
float mv_to_percent(float mvolts)
{
	if (mvolts < 1000.0f) { return 0.0f; }   // no reading
	return (float)battPercent(mvolts, fBattMax*1000.0f);
}

#endif  // USE_NEW_BATT
