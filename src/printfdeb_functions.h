#pragma once

#include <Arduino.h>

int printlndeb(const char *buff);
int printdeb(const char *buff);

int printlndeb(int iVar);

int printdeb(int iVar);
int printdeb(unsigned int iVar);
int printdeb(short iVar);
int printdeb(float fVar);
int printdeb(char c);
int printdeb(unsigned char c);

int printlndeb(String str);
int printdeb(String str);

int printfdeb(const char *format, ...);

/**
 * @brief Bytes, die printfdeb() seit dem Start verworfen hat (kumulativ).
 *
 * Auf nRF52 verwirft printfdeb() Ausgabe, wenn der USB-CDC-Sendepuffer voll
 * bleibt (N-21: sonst blockiert write() bei totem Endpoint endlos). Auf allen
 * anderen Plattformen gibt es diesen Pfad nicht, der Zaehler ist dort 0.
 *
 * Wichtig fuer den Rohframe-Mitschnitt: eine Logdatei wird genau dann
 * loechrig, wenn der Kanal ausgelastet ist -- also in den Lagen, um
 * derentwillen man mitschneidet. Ohne diese Zahl behauptet ein Mitschnitt
 * Vollstaendigkeit, die er nicht hat.
 */
unsigned long printfdebDroppedBytes(void);
