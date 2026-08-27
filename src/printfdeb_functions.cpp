// Functions: printfdeb() replaces the printf()
// This function, allowing for different formatting options.
//
// printf() is converted to printfdeb() to maintain functionality
// and passing data to the NETCONSOLE.
//
// Author: Kurt Baumann (OE1KBC), 06/2026
//
// Syntax:
// int printfdeb(const char *format [,argument] ...)    --> Serial.printf(format, argument)
// int printdeb(const char *text)                       --> Serial.print(text)
// int printlndeb(const char *text)                     --> Serial.println(text)
//
// Commands:
// --debug csv/man
// --debug de/en
//
// damit ein printfdeb() auch die Commands --debug de/en umsetzt
// muss vor jedem printfdeb() die Variable
// bDEBUGLNG = true
// gesetzt werden

#include <Arduino.h>

static auto& s_hwSerial = Serial;

#include "printfdeb_functions.h"
#include "printfdeb_format.h"

#include "loop_functions.h"
#include "loop_functions_extern.h"

#include <stdio.h>
#include <stdarg.h>

#ifdef ESP32
#include "net_console.h"
#endif

#if defined(NRF52_SERIES)
// N-21: Bei totem Bulk-IN-Endpoint (DTR bleibt gesetzt, EP0 lebt) blockiert
// Adafruit_USBD_CDC::write() endlos (while(remain && connected) { yield(); }) —
// das friert den Task ein, der zuerst druckt (Loop-Task oder ueber OnRxDone
// sogar den Timer-Task). Deshalb: kurz auf FIFO-Platz warten (max. 20 ms),
// dann verwerfen statt blockieren. g_cdcTxDropped zaehlt verworfene Bytes
// fuer die [CDC]-Telemetrie in nrf52_main.cpp.
volatile unsigned long g_cdcTxDropped = 0;

// true = Platz vorhanden, Ausgabe darf raus; false = verworfen (gezaehlt).
static bool cdcReady(size_t need)
{
    if (!(bool)Serial)
        return true;   // nicht verbunden: write() verwirft selbst, blockiert nie
    if (need > 128) need = 128;
    uint32_t t0 = millis();
    while ((size_t)Serial.availableForWrite() < need)
    {
        if (millis() - t0 > 20)
        {
            g_cdcTxDropped += need;
            return false;
        }
        yield();
    }
    return true;
}
#else
static inline bool cdcReady(size_t) { return true; }
#endif

unsigned long printfdebDroppedBytes(void)
{
#if defined(NRF52_SERIES)
    return g_cdcTxDropped;
#else
    return 0;   // kein verwerfender Pfad -- cdcReady() ist hier immer true
#endif
}

int printfdeb(const char *uformat, ...)
{
    char nformat[300];

    printfdebRewriteFormat(uformat, nformat, sizeof(nformat), bDEBUGCSV);

    char loc_buf[600];
    char * temp = loc_buf;
    va_list arg;
    va_list copy;
    va_start(arg, uformat);
    va_copy(copy, arg);
    int len = vsnprintf(temp, sizeof(loc_buf), nformat, copy);
    va_end(copy);
    if(len < 0) {
        va_end(arg);
        return 0;
    }
    if(len >= (int)sizeof(loc_buf)){  // comparation of same sign type for the compiler
        temp = (char*) malloc(len+1);
        if(temp == NULL) {
            va_end(arg);
            return 0;
        }
        len = vsnprintf(temp, len+1, nformat, arg);
    }
    va_end(arg);

    // durchlaufe Array, bis Ende-Zeichen '\0' und '.' => ',' für deutsche CSV-Kompatibilität
    if(!bDEBUGEN && bDEBUGCSV && bDEBUGLNG)
        for (int i = 0; temp[i] != '\0'; i++) { if (temp[i] == '.') { temp[i] = ','; } }

    bDEBUGLNG = false; // wieder deaktivieren

    // SEC-02: temp enthaelt bereits substituierten Text — u.a. ueber Funk empfangene
    // Nutzdaten, die per %s eingesetzt wurden. Diese duerfen NICHT erneut als
    // Format-String geparst werden (%s/%n ohne passende varargs -> Stack-Lesezugriffe
    // bzw. Schreibzugriff).
    //
    // Serial.printf("%s", temp) waere dagegen ebenfalls sicher, formatiert den fertigen
    // String aber ein zweites Mal: Print::printf nutzt intern nur 64 Byte Stack und
    // malloc()t fuer alles Darueber — also bei fast jeder Log-Zeile. Dieser Heap-Churn
    // liess NimBLE (MSYS1_BLOCK_COUNT=4) keine mbufs mehr fuer den Verbindungsaufbau
    // finden: der Node beantwortete CONNECT_IND nicht mehr, der Central brach den
    // Verbindungsaufbau mit 0x3e ab. Direkt schreiben ist sicher UND allokationsfrei.
#if defined(NRF52_SERIES)
    // N-21: nie mehr an write() uebergeben, als der TX-FIFO frei hat — sonst
    // blockiert der Aufruf bei totem Endpoint endlos (s. cdcReady()-Kommentar).
    {
        size_t off = 0;
        uint32_t tw0 = millis();
        while (off < (size_t)len && (bool)Serial)
        {
            size_t canw = (size_t)Serial.availableForWrite();
            if (canw == 0)
            {
                if (millis() - tw0 > 20)
                {
                    g_cdcTxDropped += (unsigned long)((size_t)len - off);
                    break;
                }
                yield();
                continue;
            }
            size_t n = (size_t)len - off;
            if (n > canw) n = canw;
            Serial.write((const uint8_t*)temp + off, n);
            off += n;
        }
    }
#else
    Serial.write((const uint8_t*)temp, (size_t)len);
#endif

    if(temp != loc_buf){
        free(temp);
    }
    return len;
}

int printlndeb(const char *buff)
{
    if (!cdcReady(strlen(buff) + 2)) return 0;
    int len = Serial.println(buff);
    return len;
}

int printdeb(const char *buff)
{
    if (!cdcReady(strlen(buff))) return 0;
    int len = Serial.print(buff);
    return len;
}

int printlndeb(int iVar)
{
    if (!cdcReady(14)) return 0;
    int len = Serial.println(iVar);
    return len;
}

int printdeb(int iVar)
{
    if (!cdcReady(12)) return 0;
    int len = Serial.print(iVar);
    return len;
}

int printfdeb(unsigned int iVar)
{
    if (!cdcReady(12)) return 0;
    int len = Serial.print(iVar);
    return len;
}

int printfdeb(short iVar)
{
    if (!cdcReady(8)) return 0;
    int len = Serial.print(iVar);
    return len;
}

int printdeb(float fVar)
{
    if (!cdcReady(16)) return 0;
    int len = Serial.print(fVar);
    return len;
}

int printdeb(char c)
{
    if (!cdcReady(1)) return 0;
    int len = Serial.print(c);
    return len;
}

int printdeb(unsigned char c)
{
    if (!cdcReady(4)) return 0;
    int len = Serial.print(c);
    return len;
}

int printlndeb(String str)
{
    if (!cdcReady(str.length() + 2)) return 0;
    int len = Serial.println(str);
    return len;
}

int printdeb(String str)
{
    if (!cdcReady(str.length())) return 0;
    int len = Serial.print(str);
    return len;
}
