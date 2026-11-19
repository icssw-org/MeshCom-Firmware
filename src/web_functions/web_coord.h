// P-0124 -- Read-back-Vergleich fuer setlat/setlon (src/web_functions/web_setup.cpp).
// Header-only, damit der Test auf dem Host laeuft (env:native_web_coord).
//
// Der Vergleich bleibt in double. Ein float-Vergleich scheitert fuer fast jede
// Koordinate: 49.997 wird als float zu 49.9970016, der Test schlaegt fehl und der
// Web-Dialog meldet "Value could not be set", obwohl der Wert gespeichert ist.
// --setlat/--setlon speichern Sued- und Westwerte als Betrag plus 'S'/'W', deshalb
// wird der getippte Wert ebenfalls als Betrag verglichen.
#pragma once

#include <math.h>
#include <stdlib.h>

static inline bool webCoordMatches(double stored, const char *typed){
    return fabs(stored - fabs(atof(typed))) < 1e-7;
}
