// posTagIsNan() -- NaN guard for the position beacon's optional APRS tags
// (/P=, /H=, /T=, /O=, /F=, /Q=, /G=, /C= in PositionToAPRS(),
// src/loop_functions.cpp). Each tag is formatted with snprintf("/X=%.1f",
// value) into its own buffer; when the source value is NaN, snprintf writes
// "nan" (or "-nan" on some libc/printf implementations for a signalling
// NaN) after the "/X=" key. The guard used to compare every buffer against
// the SAME hard-coded "/P=nan" template regardless of which tag it actually
// held, so a NaN in humidity/temp/temp2/qfe/qnh/gas/co2 went on air as
// "/H=nan" -- upstream issue: NaN guards test cpress instead of the buffer
// they just wrote.
//
// posTagIsNan() takes the buffer that was just written and checks only the
// value after the 3-char key ("/X="), so each call site can be handed its
// own buffer and get a correct answer independent of the tag letter.
// Arduino-free so the contract is testable on the host.
#ifndef _POS_TAG_NAN_H_
#define _POS_TAG_NAN_H_

#include <string.h>

static inline bool posTagIsNan(const char *tag)
{
    if(tag == NULL || strlen(tag) < 3)
        return false;

    const char *value = tag + 3; // skip "/X="
    return strcmp(value, "nan") == 0 || strcmp(value, "-nan") == 0;
}

#endif // _POS_TAG_NAN_H_
