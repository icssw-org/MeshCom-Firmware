#include <Arduino.h>

#include "configuration.h"
#include "loop_functions_extern.h"
#include "loop_functions.h"
#include "command_functions.h"

#include "onebutton_functions.h"

#if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
#include <lvgl.h>
#include <t-deck/tdeck_main.h>
#include <t-deck/lv_obj_functions.h>
#endif 

#if defined(HAS_TFT) || defined(HAS_TFT_114)
#include "tft_display_functions.h"
#endif

#if defined(BOARD_T_ECHO)
#include "nrf52_functions.h"
#endif

#if defined(WP_DISP)
// Anzahl der per 1x-Klick durchblaetterbaren letzten Nachrichten (Wireless Paper + E213,
// gemeinsamer 2.13"-Display-Pfad): die letzten 4 (original-kompatibel).
#define WP_MSG_BROWSE_MAX 4
#endif

void singleClick()
{
  if(bDisplayCont)
    Serial.println("SingleClick");

    #ifdef BOARD_T5_EPAPER
        return;
    #endif

  // no oneclick on TRACK=on
  if(!bDisplayTrack)
  {
    if(bDisplayCont)
        Serial.printf("BUTTON single press last:%i pointer:%i lines:%i\n", pageLastPointer, pagePointer, pageLastLineAnz[pagePointer]);

    #if defined(WP_DISP)
    // ===== Wireless Paper + E213: die letzten WP_MSG_BROWSE_MAX Nachrichten + Status in einer LOOP =====
    // Eigene Logik statt des Magic-Status-Slots: der konnte haengenbleiben, wenn die NEUESTE
    // Nachricht zufaellig im Status-Slot landete (Original "verhaspelte" sich nach ~3). Die WP hat
    // zudem keinen pageHold-Timeout (sendDisplayTime laeuft hier nicht), daher wird der Loop hier
    // explizit geschlossen. Rotation: neueste -> ... -> Fensterrand -> Status -> neueste -> ...
    // Das Browse-Fenster ist auf WP_MSG_BROWSE_MAX (=4, vorbereitet fuer 9) begrenzt - der Ring
    // (PAGE_MAX=10) koennte mehr, aus Kompatibilitaet zeigen wir aber nur die letzten 4.
    {
      static bool wpStatusStep = false;                 // naechster Klick zeigt die Status-Seite
      int wpNewest = pageLastPointer - 1;               // Slot der neuesten Nachricht
      if(wpNewest < 0) wpNewest += PAGE_MAX;

      // KEINE Nachricht im Ringpuffer: Original/PR -> Statusschirm; Preview -> "No Message".
      if(pageLastLineAnz[wpNewest] == 0)
      {
        sendDisplayHead(true);
        wpStatusStep = false;
        pagePointer  = wpNewest;
        return;
      }

      // Status-Schritt (nach dem Fensterrand/der aeltesten) -> Statusschirm, dann Loop
      if(wpStatusStep)
      {
        sendDisplayHead(true);
        wpStatusStep = false;
        pagePointer  = wpNewest;
        return;
      }

      if(pageLastLineAnz[pagePointer] == 0)             // Sicherheit: aktueller Slot leer -> neueste
        pagePointer = wpNewest;

      // Anzahl gespeicherter Nachrichten (zusammenhaengend ab der neuesten)
      int wpN = 0;
      {
        int s = wpNewest;
        for(int n = 0; n < PAGE_MAX; n++)
        {
          if(pageLastLineAnz[s] == 0) break;
          wpN++;
          s--; if(s < 0) s += PAGE_MAX;
        }
      }
      // sichtbares Fenster = min(vorhanden, WP_MSG_BROWSE_MAX)
      int wpShow  = (wpN < WP_MSG_BROWSE_MAX) ? wpN : WP_MSG_BROWSE_MAX;
      // Schritte unterhalb der neuesten (0 = neueste); ausserhalb Fenster -> auf neueste zuruecksetzen
      int wpSteps = (wpNewest - pagePointer + PAGE_MAX) % PAGE_MAX;
      if(wpSteps >= wpShow) { pagePointer = wpNewest; wpSteps = 0; }

      // Index oben rechts: neueste = wpShow (max 4) ... Fensterrand = 1; 0 = Home/Status.
      int wpRank = wpShow - wpSteps;

      // Nachricht anzeigen (Helper in loop_functions.cpp; kompaktes Layout + Voll-Refresh).
      wpShowStoredMessage(pagePointer, wpRank);

      // naechste (aeltere) bestimmen; am Fensterrand (wpSteps+1 >= wpShow) ODER aelteste erreicht
      // (naechster Slot leer) -> naechster Klick zeigt Status, danach wieder bei der neuesten.
      int wpNext = pagePointer - 1;
      if(wpNext < 0) wpNext += PAGE_MAX;
      if((wpSteps + 1) >= wpShow || pageLastLineAnz[wpNext] == 0)
        wpStatusStep = true;
      else
        pagePointer = wpNext;

      return;
    }
    #endif

    if(pagePointer == 5)
    {
      sendDisplayHead(true);
      return;
    }
      
    if(pageLastLineAnz[pagePointer] == 0)
      pagePointer = pageLastPointer-1;

    bDisplayIsOff=false;

    pageLineAnz = pageLastLineAnz[pagePointer];

    for(int its=0;its<pageLineAnz;its++)
    {
        // Save last Text (init)
        pageLine[its][0] = pageLastLine[pagePointer][its][0];
        pageLine[its][1] = pageLastLine[pagePointer][its][1];
        pageLine[its][2] = pageLastLine[pagePointer][its][2];
        memcpy(pageText[its], pageLastText[pagePointer][its], 25);
        
        if(its == 0)
        {
            for(int iss=0; iss < 20; iss++)
            {
                if(pageText[its][iss] == 0x00)
                    pageText[its][iss] = 0x20;
            }

            #if !defined(BOARD_E290) && !defined(BOARD_TRACKER) && !defined(BOARD_HELTEC_T114) && !defined(BOARD_T_ECHO) && !defined (BOARD_T_DECK) && !defined (BOARD_T_DECK_PLUS) && !defined (BOARD_T5_EPAPER) && !defined (BOARD_T_DECK_PRO)
            int ispos=18;
            #else
            int ispos=16;
            #endif
            pageText[its][ispos] = 0x20;
            pageText[its][ispos+1] = (pagePointer + 1) | 0x30;
            pageText[its][ispos+2] = 0x00;
        }
    }

    #if defined(BOARD_E290) || defined(BOARD_WIRELESS_PAPER)
        iDisplayType=9;
    #else
        iDisplayType=0;
    #endif

    #if defined(HAS_TFT) || defined(HAS_TFT_114)
      displayTFT(pageLastTextLong1[pagePointer], pageLastTextLong2[pagePointer]);
    #else
      strncpy(pageTextLong1, pageLastTextLong1[pagePointer], sizeof(pageTextLong1));
      if(bDisplayCont && strlen(pageTextLong1) > 0)
        Serial.println(pageTextLong1);

      strncpy(pageTextLong2, pageLastTextLong2[pagePointer], sizeof(pageTextLong2));
      if(bDisplayCont && strlen(pageTextLong2) > 0)
        Serial.println(pageTextLong2);

      sendDisplay1306(false, true, 0, 0, (char*)"#N");
    #endif

    pagePointer--;
    if(pagePointer < 0)
        pagePointer=PAGE_MAX-1;

    pageHold=5;
  }
}

void doubleClick()
{

  if(bDisplayCont)
    Serial.println("DoubleClick");

  #ifdef BOARD_T5_EPAPER
      return;
  #endif

  #if defined(BOARD_HELTEC_T114) or defined(BOARD_T_ECHO)
    if(!bDisplayOff)
    {
        commandAction((char*)"--display off", isPhoneReady, false);
    }
    else
    {
        commandAction((char*)"--display on", isPhoneReady, false);
    }
  #else
  if(bDisplayTrack)
      commandAction((char*)"--sendtrack", false);
  else
      commandAction((char*)"--sendpos", false);
  #endif
}

void tripleClick()
{
  if(bDisplayCont)
    Serial.println("TripleClick");

  #ifdef BOARD_T5_EPAPER
    return;
  #endif

  bDisplayTrack=!bDisplayTrack;

  bDisplayIsOff=false;

  if(bDisplayTrack)
      commandAction((char*)"--track on", false);
  else
      commandAction((char*)"--track off", false);

  bOneButton = true;

  sendDisplayHead(false);
}

void PressLong()
{
  if(bDisplayCont)
    Serial.println("LongPress Stop");

  bShowHead=false;

  #if defined(WP_DISP)
      // Long Press auf Wireless Paper + E213 -> Deepsleep (Strom sparen im Akkubetrieb). #992
      // "--display off" ist auf E-Ink SINNLOS (bistabiles Panel zieht statisch keinen Strom);
      // der --deepsleep-Pfad loescht stattdessen das Panel sichtbar (wpShowDeepSleep).
      commandAction((char*)"--deepsleep", isPhoneReady, false);
  #elif defined(BOARD_E290)
      sendDisplayMainline();
      E290DisplayUpdate();
  #else
      pageHold=0;

      #if defined(BOARD_TRACKER)
        Serial.println("[BOARD_TRACKER]... GO to deepsleep");
        commandAction((char*)"--deepsleep", isPhoneReady, false);
      #elif defined(BOARD_HELTEC_V2)
        Serial.println("[BOARD_HELTEC_V2]... GO to deepsleep");
        commandAction((char*)"--deepsleep", isPhoneReady, false);
      #elif defined(BOARD_HELTEC_V3)
        Serial.println("[BOARD_HELTEC_V3]... GO to deepsleep");
        commandAction((char*)"--deepsleep", isPhoneReady, false);
      #elif defined(BOARD_HELTEC_V4)
        Serial.println("[BOARD_HELTEC_V4]... GO to deepsleep");
        commandAction((char*)"--deepsleep", isPhoneReady, false);
      #elif defined(BOARD_HELTEC_T114)
        Serial.println("[BOARD_HELTEC_T114]... GO to deepsleep");
        commandAction((char*)"--deepsleep", isPhoneReady, false);
      #elif defined(BOARD_TLORA_OLV216)
        Serial.println("[BOARD_TLORA_OLV216]... GO to deepsleep");
        bDisplayOff=true;
        bDisplayIsOff=true;
        sendDisplayHead(false);
        commandAction((char*)"--deepsleep", isPhoneReady, false);
      #elif defined(BOARD_T_ECHO)
        Serial.println("[BOARD_T_ECHO]... PWR OFF");
        boardPWROff();
      #else
        if(!bDisplayOff)
        {
            commandAction((char*)"--display off", isPhoneReady, false);
        }
        else
        {
            commandAction((char*)"--display on", isPhoneReady, false);
        }
      #endif

  #endif
}

// setup code here, to run once:
void init_onebutton()
{
  if(bButtonCheck)
  {
    Serial.printf("[OBUT]...One Button GPIO(%i) started\n", iButtonPin);

/* Initialize a new OneButton instance for a button connected to digital
*  pin and GND, which is active low [2] and uses the internal pull-up resistor.
*/
    #if (defined(BOARD_E290) || defined(BOARD_WIRELESS_PAPER) || defined(BOARD_E213))
      // btn.setup( /*GPIO=*/ iButtonPin, /*active LOW=*/ true, /*pull-up=*/ false);
      btn.setup( /*GPIO=*/ iButtonPin, INPUT, /*active LOW=*/ true);
    #else
      // btn.setup( /*GPIO=*/ iButtonPin, /*active LOW=*/ true, /*pull-up=*/ true);
      btn.setup( /*GPIO=*/ iButtonPin, INPUT_PULLUP, /*active LOW=*/ true);
    #endif

    btn.attachClick(singleClick);         // Single Click event attachment
    btn.attachDoubleClick(doubleClick);   // Double Click event attachment
    btn.attachMultiClick(tripleClick);    // Multi Click event attachement
    btn.attachLongPressStart(PressLong);  // LongPress Click event attachement
  }
  else
  {
      Serial.printf("[OBUT]...One Button GPIO(%i) not activated\n", iButtonPin);
  }
}
// setup

// End
