#pragma once

// R1-02 Schritt 1: die Rahmung einer Ringzelle fuer das Telefon, EINMAL.
//
// sendToPhone() und sendComToPhone() (src/phone_commands.cpp) trugen dieselbe
// dreiarmige Fallunterscheidung zweimal aus -- 0x91 (MHeard), 0x44 (JSON),
// sonst Text mit vorangestelltem 0x40. Bis auf EINEN Unterschied Zeichen fuer
// Zeichen gleich, und dieser Unterschied ist ein Defekt, kein Zweck:
//
//   sendToPhone()     Text-Arm kopiert blelen Bytes
//   sendComToPhone()  Text-Arm kopiert blelen-1 Bytes  <-- eins zu wenig
//
// Der Com-Text-Arm ist unerreichbar: BLEComToPhoneBuff hat genau zwei
// Erzeuger -- sendBleJsonRegister() (command_functions.cpp:133-140) und der
// MHeard-Versand (mheard_functions.cpp:758-764) -- und BEIDE setzen
// buffer[0] = 0x44, nehmen also immer den JSON-Arm. Geschrieben wird der Ring
// sonst nirgends (einzige Schreibstelle: addBLEComToOutBuffer(),
// loop_functions.cpp). Der 0x91-Arm dort ist aus demselben Grund tot.
//
// Massgeblich ist deshalb die Fassung aus sendToPhone(): dort haengen die 58
// Aufrufer von addBLECommandBack() dran, und der Text-Arm laeuft dort
// tatsaechlich. Beide Seiten rufen jetzt diese eine Funktion.
//
// ACHTUNG, DIE SENDELAENGE GEHOERT NICHT HIERHER. Beide Aufrufer senden
// anschliessend `blelen + 2` Bytes -- und zwar in ALLEN drei Armen, obwohl
// jeder Arm eine ANDERE Anzahl Bytes schreibt (blelen-1, blelen, blelen+1).
// Gesendet werden also je nach Arm 3, 2 oder 1 Byte mehr, als hier gefuellt
// wurden; sie kommen aus dem nullinitialisierten Zielpuffer. Genau daher der
// Kommentar "why do we need to add 2 bytes??" im Original. Das ist die
// Wiregestalt, auf die das Telefon eingestellt ist, und sie bleibt unberuehrt.
//
// Deshalb gibt diese Funktion KEINE Laenge zurueck, sondern nur "gerahmt/
// nicht gerahmt": gaebe sie eine Laenge zurueck, waere es eine Einladung,
// damit zu senden -- und das wuerde jedes BLE-Telegramm um 1 bis 3 Byte
// verkuerzen. Die Sendelaenge bleibt beim Aufrufer, unveraendert.
//
// Arduino-frei, damit der Rahmen auf dem Host pruefbar ist
// (test/test_ble_phone_frame, env native_ble_phone_frame). Bis hierher gab es
// fuer diese Umformung KEINEN einzigen ausfuehrbaren Test -- sie ist nur ueber
// BLE beobachtbar, und dafuer braucht es tools/bench/ble_golden.py und ein
// Telefon.

#include <stdint.h>
#include <string.h>

#define BLE_PHONE_TYPE_MHEARD 0x91u   // MHeard-Liste: Typbyte wird verworfen
#define BLE_PHONE_TYPE_JSON   0x44u   // 'D' -- JSON, unveraendert durchgereicht
#define BLE_PHONE_TAG_TEXT    0x40u   // Text/Position: Tag wird vorangestellt

/**
 * @brief Rahmt eine Ringzelle in den Sendepuffer fuer das Telefon.
 *
 * @param payload  Nutzlast der Zelle, also ab Offset 1 (das Laengenbyte in
 *                 [0] gehoert NICHT dazu). payload[0] ist das Typbyte.
 * @param blelen   Laenge laut Laengenbyte der Zelle.
 * @param out      Zielpuffer, vom Aufrufer nullinitialisiert.
 * @param out_size Groesse von out.
 *
 * @return true wenn gerahmt wurde. false heisst NICHT SENDEN -- entweder
 *         blelen == 0 (N-04-Rest: blelen-1 liefe sonst auf 255 ueber und
 *         kopierte weit hinter die Nutzlast) oder das Ergebnis passt nicht
 *         in out.
 */
static inline bool blePhoneFrame(const uint8_t *payload, uint8_t blelen,
                                 uint8_t *out, size_t out_size)
{
    if (payload == 0 || out == 0 || blelen == 0)
        return false;

    if (payload[0] == BLE_PHONE_TYPE_MHEARD)
    {
        uint8_t n = (uint8_t)(blelen - 1);
        if ((size_t)n > out_size)
            return false;
        memcpy(out, payload, n);
        return true;
    }

    if (payload[0] == BLE_PHONE_TYPE_JSON)
    {
        if ((size_t)blelen > out_size)
            return false;
        memcpy(out, payload, blelen);
        return true;
    }

    // Text und Position: 0x40 davor, Nutzlast vollstaendig dahinter.
    if ((size_t)blelen + 1u > out_size)
        return false;
    out[0] = BLE_PHONE_TAG_TEXT;
    memcpy(out + 1, payload, blelen);
    return true;
}
