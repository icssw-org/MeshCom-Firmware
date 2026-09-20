#include "regex_functions.h"

#include <Regexp.h>

// match state object
MatchState regex_call;

//char* regex_call_match = (char*)"^[A-Z]{1,2}|[0-9][A-Z])[0-9][A-Z]{1,3}(-[0-9]{1,2}|$";
char* regex_call_match = (char*)"^[0-9A-Z]?[A-Z]?[0-9]+[A-Z][A-Z]?[A-Z]?[%-]?[0-9]?[0-9]?$";

int ret_call;

bool checkRegexCall(String callsign)
{
	if(callsign.length() < 1)
		return false;

	// not legal
	// DE ... Germany SWL-Callsign
	if(callsign.compareTo("DE") == 0)
		return false;

	// legal
	if(callsign.compareTo("*") == 0) // TOALL message
		return true;
	
	if(callsign.compareTo("H") == 0)	// HEY Message
		return true;

	if(callsign.compareTo("HG") == 0)	// HEY Message from Gateway
		return true;

	if(callsign.compareTo("BOT GATE") == 0)
		return true;

	if(callsign.compareTo("TEST") == 0)
		return true;

	if(callsign.compareTo("TESTER") == 0)
		return true;

	if(callsign.compareTo("WLNK-1") == 0)
		return true;

	if(callsign.compareTo("APRS2SOTA") == 0)
		return true;

	if(callsign.compareTo("OE2YOTA-1") == 0)
		return true;

	regex_call.Target((char*)callsign.c_str());

	if(regex_call.Match(regex_call_match) <= 0)
		return false;

	return true;
}

// Ersatz-SSID, wenn keine angegeben ist. In APRS bedeutet ein Rufzeichen ohne
// SSID dasselbe wie "-0" ("no SSID represents a zero SSID"), beides laesst den
// Knoten im Netz aber nicht eindeutig werden. 99 ist die im MeshCom-Netz
// eingebuergerte Sammel-SSID; sie ist APRS-IS-konform (ein bis zwei
// alphanumerische Zeichen), nicht AX.25-konform (dort nur 0-15) -- MeshCom ist
// kein AX.25-Netz. Die Werkseinstellung des Rufzeichens steht daneben in
// configuration_global.h.
#define OWN_CALL_DEFAULT_SSID 99

bool normalizeOwnCall(String &callsign)
{
	int iDash = callsign.indexOf('-');

	String sBase = (iDash < 0) ? callsign : callsign.substring(0, iDash);
	String sSSID = (iDash < 0) ? String("") : callsign.substring(iDash + 1);

	// Nur gewoehnliche Rufzeichen anfassen. APRS verlangt eine Basis von
	// mindestens drei Zeichen, AX.25 laesst hoechstens sechs zu, und ein
	// Rufzeichen traegt immer mindestens eine Ziffer und einen Buchstaben.
	// Damit bleiben die Sondermarken, die checkRegexCall() durchlaesst --
	// "*", "H", "HG", "TEST", "TESTER", "WLNK-1", "APRS2SOTA", "OE2YOTA-1" --
	// von selbst unberuehrt, ohne sie hier noch einmal aufzuzaehlen.
	if(sBase.length() < 3 || sBase.length() > 6)
		return true;

	bool bDigit = false;
	bool bAlpha = false;

	for(unsigned int ic = 0; ic < sBase.length(); ic++)
	{
		char cc = sBase.charAt(ic);

		if(cc >= '0' && cc <= '9')
			bDigit = true;
		else
		if(cc >= 'A' && cc <= 'Z')
			bAlpha = true;
		else
			return true;
	}

	if(!bDigit || !bAlpha)
		return true;

	// Die SSID ist eine Zahl, kein Text: "-01" und "-1" bezeichnen dieselbe
	// Station, kanonisch ist die Form ohne fuehrende Null.
	int iSSID = OWN_CALL_DEFAULT_SSID;

	if(sSSID.length() > 0)
	{
		for(unsigned int ic = 0; ic < sSSID.length(); ic++)
		{
			char cc = sSSID.charAt(ic);

			if(cc < '0' || cc > '9')
				return true;
		}

		iSSID = sSSID.toInt();

		if(iSSID == 0)
			iSSID = OWN_CALL_DEFAULT_SSID;
	}

	String sOut = sBase + "-" + String(iSSID);

	// APRS-IS begrenzt Rufzeichen samt SSID auf neun Zeichen, und genau so
	// gross ist meshcom_settings.node_call (neun Zeichen plus Nullbyte). Was
	// nicht hineinpasst, wird zurueckgewiesen statt stillschweigend
	// abgeschnitten.
	if(sOut.length() > 9)
		return false;

	callsign = sOut;

	return true;
}
