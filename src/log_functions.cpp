/**
 * @file WZ_dbLOG.cpp
 * @author W.Zelinka (OE3WAS, https://github.com/karamo)
 * @brief 
 * @version 0.1
 * @date 2026-04-09
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#include "Arduino.h"

#include <stdio.h>
#include <stdarg.h> // Erforderlich für va_list

#include "log_functions.h"

// 1. Variante: nur einen formatierten String ausgeben & die Serial unterscheiden
//    z.B. dbLOG("Sats: %d", numSats);
void dbLOG(const char * fmt, ...) {
  char buf[256]; // Puffergröße bei Bedarf anpassen
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  #if defined(ESP32S3) && (ARDUINO_USB_CDC_ON_BOOT == 1)
    USBSerial.print(buf);
  #else
    Serial.print(buf);
  #endif
}


// 2. Variante: einen formatierten String ausgeben & die Serial unterscheiden
//    + abhängig von einer Bedingung (bool)
//    z.B. dbCLOG(on_GPS,"Sats: %d", numSats);
void dbCLOG(bool condition, const char * fmt, ...) {
  char buf[256]; // Puffergröße bei Bedarf anpassen
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  if(condition) {
    #if defined(ESP32S3) && (ARDUINO_USB_CDC_ON_BOOT == 1)
      USBSerial.print(buf);
    #else
      Serial.print(buf);
    #endif
  }
}

// 3. Variante: einen formatierten String ausgeben & die Serial unterscheiden
//    + abhängig von einem LOG-Level (int)
//    z.B. dbLLOG(GPSDEBUG, 2, "Sats: %d", numSats);
void dbLLOG(int dbgVAR, int Level, const char * fmt, ...) {
  char buf[256]; // Puffergröße bei Bedarf anpassen
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  if(dbgVAR >= Level) {
    #if defined(ESP32S3) && (ARDUINO_USB_CDC_ON_BOOT == 1)
      USBSerial.print(buf);
    #else
      Serial.print(buf);
    #endif
  }
}



/*
const char * fmt: Akzeptiert nun direkt String-Literale wie "[GPS]" oder Format-Strings.
va_list & vsnprintf: Ermöglicht es der Funktion, wie ein echtes printf zu arbeiten
  z.B. dbLOG("Sats: %d", numSats);
Serial.print(buf): Da der String bereits im Puffer fertig formatiert wurde,
 nutzt man print statt printf, um eine erneute (und riskante) Formatierung zu vermeiden. 
*/