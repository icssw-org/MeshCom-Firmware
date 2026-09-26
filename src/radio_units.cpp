#include "radio_units.h"

// See radio_units.h for the unit table and why these are pure.

float radioBwStoredToKhz(float stored, bool indexed)
{
    if(!indexed)
        return stored;

    if(stored == 0)
        return 125.0f;
    if(stored == 1)
        return 250.0f;
    if(stored == 2)
        return 500.0f;

    return stored;
}

float radioBwKhzToStored(float khz, bool indexed)
{
    if(!indexed)
        return khz;

    if(khz == 125.0f)
        return 0;
    if(khz == 250.0f)
        return 1;
    if(khz == 500.0f)
        return 2;

    return khz;
}

int radioCrStoredToDenom(int stored, bool indexed)
{
    if(!indexed)
        return stored;

    if(stored >= 1 && stored <= 4)
        return stored + 4;

    return stored;
}

int radioCrDenomToStored(int denom, bool indexed)
{
    if(!indexed)
        return denom;

    if(denom >= 5 && denom <= 8)
        return denom - 4;

    return denom;
}

float radioFreqStoredToMhz(float stored, bool indexed)
{
    if(!indexed)
        return stored;

    return stored / 1000000.0f;
}

float radioFreqMhzToStored(float mhz, bool indexed)
{
    if(!indexed)
        return mhz;

    return mhz * 1000000.0f;
}
