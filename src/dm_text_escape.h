#ifndef _DM_TEXT_ESCAPE_H_
#define _DM_TEXT_ESCAPE_H_

// P15: Arduino-freier Kern der Klammer-Escape-Ausnahme fuer sendMessage()
// (nativ testbar, test/test_dm_text_escape). sendMessage() ersetzt bei einer
// DM jedes '{' im Text durch '(' (Kommentar dort: "A '{' inside the user
// text breaks the receiver's NNN parse"). {ping} und {SET} sind aber keine
// Fliesstext-Nachrichten, sondern eigene Tags:
//   * ein {ping} wuerde zu (ping}{NNN -- kein Ping mehr, die Gegenstelle
//     antwortet nie mit {pong}, und nichts stoppt eine Wiederholung.
//   * ein {SET}n;m; wuerde zu (SET}n;m; -- das Remote-Hop-Limit-Kommando
//     (sendDisplayText(), startsWith("{SET}")) feuert nie.
// dmTextEscapeFrom() liefert den Index, AB DEM escaped werden muss: 0 im
// Normalfall (unveraendertes Verhalten), sonst die Laenge des erkannten
// Tags -- ein '{' NACH dem Tag bricht weiterhin den NNN-Parse des
// Empfaengers (aprsmsg.msg_payload.indexOf("{", 1)) und wird wie bisher
// escaped.
//
// Nur ein EXAKTES fuehrendes Tag zaehlt ("{pingx"/"{SETX" treffen nicht --
// strncmp() vergleicht das schliessende '}' mit).

#include <stddef.h>
#include <string.h>

inline size_t dmTextEscapeFrom(const char *text)
{
    if(text == NULL)
        return 0;

    static const char PING_TAG[] = "{ping}";
    static const char SET_TAG[]  = "{SET}";
    const size_t pingLen = sizeof(PING_TAG) - 1;
    const size_t setLen  = sizeof(SET_TAG) - 1;

    if(strncmp(text, PING_TAG, pingLen) == 0)
        return pingLen;

    if(strncmp(text, SET_TAG, setLen) == 0)
        return setLen;

    return 0;
}

#endif // _DM_TEXT_ESCAPE_H_
