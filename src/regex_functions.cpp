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

	regex_call.Target((char*)callsign.c_str());

	if(regex_call.Match(regex_call_match) <= 0)
		return false;

	return true;
}
