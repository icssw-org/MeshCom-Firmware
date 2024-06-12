#include "regex_functions.h"

#include <Regexp.h>

// match state object
MatchState regex_call;

//char* regex_call_match = (char*)"^[A-Z]{1,2}|[0-9][A-Z])[0-9][A-Z]{1,3}(-[0-9]{1,2}|$";
char* regex_call_match = (char*)"^[0-9A-Z]?[A-Z]?[0-9]+[A-Z][A-Z]?[A-Z]?[%-]?[0-9]?[0-9]?$";

int ret_call;

bool checkRegexCall(String callsign)
{
	if(callsign == "*")
		return true;
	
	if(callsign == "BOT GATE")
		return true;

	if(callsign == "TEST")
		return true;

	if(callsign == "TESTER")
		return true;

	if(callsign == "WLNK-1")
		return true;

	if(callsign == "APRS2SOTA")
		return true;

	regex_call.Target((char*)callsign.c_str());

	if(regex_call.Match(regex_call_match) <= 0)
		return false;

	return true;
}
