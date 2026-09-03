#pragma once

// Format-Vorverarbeitung fuer printfdeb().
//
// printfdeb() schreibt den Format-String nicht direkt an vsnprintf, sondern
// baut ihn vorher um: im Nicht-CSV-Modus verschwinden Semikolons (sie trennen
// im CSV-Modus die Spalten), und '%%' muss diesen Durchlauf unbeschadet
// ueberstehen.
//
// Warum als eigener Header und nicht inline in printfdeb_functions.cpp: nur so
// laesst sich der Umbau ohne Arduino nativ pruefen (test/test_printfdeb_format)
// -- dieselbe Trennung wie bei isPlausibleAckFrame() in ack_functions.h.

#include <stddef.h>

/**
 * @brief Baut den Format-String fuer vsnprintf um.
 *
 * @param uformat  Eingabe-Format, NUL-terminiert.
 * @param nformat  Ziel-Puffer, wird NUL-terminiert.
 * @param nsize    Groesse von @p nformat in Byte.
 * @param csv      true = CSV-Modus, Semikolons bleiben stehen.
 * @return Anzahl geschriebener Zeichen ohne die abschliessende NUL.
 */
static inline size_t printfdebRewriteFormat(const char *uformat, char *nformat, size_t nsize, bool csv)
{
    size_t inn = 0;

    if(nformat == NULL || nsize == 0)
        return 0;

    nformat[0] = 0x00;

    if(uformat == NULL)
        return 0;

    for(size_t in = 0; uformat[in] != 0x00; in++)
    {
        // '%%' ist die Escape-Sequenz fuer ein einzelnes Prozentzeichen und muss
        // als GENAU zwei Zeichen durchgereicht werden -- danach ist das zweite
        // '%' mitverbraucht.
        //
        // Frueher schrieb dieser Zweig die zwei Zeichen und fiel anschliessend in
        // die allgemeine Kopie, die dasselbe '%' noch einmal anhaengte; der
        // naechste Schleifendurchlauf fuegte das zweite '%' erneut hinzu. Aus
        // '%%' wurden '%%%%', und vsnprintf machte daraus zwei Prozentzeichen --
        // sichtbar in jeder Zeile, die ein Prozentzeichen ausgab
        // ("util=18%%", "BATT 100 %%").
        if(uformat[in] == '%' && uformat[in+1] == '%')
        {
            if(inn + 2 < nsize)
            {
                nformat[inn++] = '%';
                nformat[inn++] = '%';
            }

            in++;
            continue;
        }

        if(!csv && uformat[in] == ';')
        {
            // in > 0 absichern: bei fuehrendem ';' laege uformat[in-1] vor dem
            // Puffer.
            if((in > 0 && uformat[in-1] == ' ') || uformat[in+1] == ' ')
                continue;

            if(inn + 1 < nsize)
            {
                nformat[inn++] = ' ';
                continue;
            }
        }

        if(inn + 1 < nsize)
            nformat[inn++] = uformat[in];
    }

    nformat[inn] = 0x00;

    return inn;
}
