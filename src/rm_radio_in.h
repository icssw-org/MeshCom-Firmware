// rm_radio_in.h -- stored radio settings to the engineering units of the RM `radio` reply.
//
// node_freq / node_bw / node_cr hold different units on the two platforms (radio_units.h): the nRF52
// SX126x path stores Hz, a bandwidth index and a coding-rate index. Bench 2026-10-06: the RAK4631
// answered `f=999.999 ... cr=2` because the executor passed the stored values through. Pure (explicit
// `indexed` flag), so both platform sides run in one native test.
#ifndef RM_RADIO_IN_H
#define RM_RADIO_IN_H

#include "radio_units.h"
#include "rm_format.h"

inline RmRadioIn rmRadioInFromStored(float freq, int sf, int cr, float bw, int pCur, int pMin, int pMax, bool indexed)
{
    RmRadioIn in;
    in.freqMHz = radioFreqStoredToMhz(freq, indexed);
    in.sf = sf;
    in.cr = radioCrStoredToDenom(cr, indexed);
    in.bwKHz = radioBwStoredToKhz(bw, indexed);
    in.pCur = pCur;
    in.pMin = pMin;
    in.pMax = pMax;
    return in;
}

#endif
