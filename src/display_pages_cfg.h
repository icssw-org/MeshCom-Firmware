// Welche Boards halten die Langtext-Seiten (pageLastTextLong1/2)?
//
// Die Langtext-Seiten (Kopfzeile + bis zu 200 Zeichen je gespeicherter
// Nachricht) fuellen nur Boards mit TFT oder E-Paper: alle drei
// Schreibstellen in sendDisplayText() und die Leser (onebutton_functions.cpp,
// wpShowStoredMessage) liegen in deren #if-Bloecken. Auf OLED-Boards
// (Heltec V3, T-Beam, TLORA, E22, RAK) waren die 1350 Byte immer leer.
//
// HAS_LONG_PAGE_TEXT ist die Vereinigung aller Guards, hinter denen
// geschrieben oder gelesen wird. Ein neues Board mit Langtext ergaenzt DIESE
// Zeile. Die Board-Makros (HAS_TFT, HAS_EPAPER, WP_DISP) kommen aus
// configuration.h, das vor diesem Header eingezogen sein muss -- fehlt es,
// gibt es die Felder nicht, und der Compiler sagt es beim Board, das sie
// braucht ("... was not declared in this scope").
//
// Dieser Header zieht configuration.h bewusst NICHT selbst ein. Geprueft am
// 21.09.2026: nur loop_functions.cpp und onebutton_functions.cpp fassen die
// Langtext-Felder an, beide haben configuration.h vorher. Ein eigener
// Include hier landete ueber loop_functions_extern.h in rund zwanzig
// Uebersetzungseinheiten, die configuration.h heute nicht sehen -- das ist
// mehr Risiko als der Fall, den es abfangen soll, und dieser Fall bricht
// beim Uebersetzen, nicht still zur Laufzeit. Wer kuenftig eine dritte
// Datei an die Felder laesst, zieht dort configuration.h zuerst ein.
#pragma once

#if defined(WP_DISP) || defined(HAS_EPAPER) || defined(HAS_TFT) || defined(HAS_TFT_114) || \
    defined(BOARD_T_DECK_PRO) || defined(BOARD_TRACKER) || defined(BOARD_HELTEC_T114) || \
    defined(BOARD_T_CONNECT_PRO) || defined(BOARD_T5_EPAPER)
#define HAS_LONG_PAGE_TEXT 1
#endif
