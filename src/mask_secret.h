#pragma once

// Maskierung von Passwoertern in der Klartextausgabe der Firmware.
//
// Die Ausgabe laeuft ueber printfdeb() und damit nicht nur auf die serielle
// Schnittstelle, sondern auch auf die Netzkonsole (Port 2323, net_console.h).
// Die steht ohne gesetztes node_passwd ohne jede Authentisierung offen: ein
// `nc <node> 2323` gefolgt von `--info` lieferte bis hierher das WLAN-PSK, das
// Webserver-Passwort und das Konsolenpasswort im Klartext an jeden im selben
// Netz. Am laufenden Knoten nachgestellt (DK5EN-98, 25.08.2026).
//
// Sichtbar bleibt, OB ein Passwort gesetzt ist -- das ist die Information, um
// derentwillen die Zeile in --info steht. Die Settings-JSON an die App
// (nsetdoc["WSPWD"]) bleibt unberuehrt: sie muss den Wert tragen, damit die App
// ihn anzeigen und aendern kann.

#include <stddef.h>

/**
 * @brief Ersetzt ein gesetztes Passwort durch "***".
 *
 * @param secret  Passwortpuffer, darf NULL sein.
 * @return "***" wenn gesetzt, sonst "" -- nie NULL, direkt an %s uebergebbar.
 */
static inline const char *maskSecret(const char *secret)
{
    if(secret == NULL)
        return "";

    // node_passwd wird mit "%-14.14s" gespeichert, ist also mit Leerzeichen
    // aufgefuellt; ein fuehrendes Leerzeichen heisst "nicht gesetzt" -- dieselbe
    // Pruefung wie bei hasPasswd im --passwd-Zweig (command_functions.cpp).
    if(secret[0] == 0x00 || secret[0] == ' ')
        return "";

    return "***";
}
