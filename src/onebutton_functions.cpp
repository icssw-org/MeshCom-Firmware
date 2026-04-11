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
            pageText[its][19] = pagePointer | 0x30;
            pageText[its][20] = 0x00;
        }
    }

    #ifdef BOARD_E290
        iDisplayType=9;
    #else
        iDisplayType=0;
    #endif

    #if defined(HAS_TFT) || defined(HAS_TFT_114)
      displayTFT(pageLastTextLong1[pagePointer], pageLastTextLong2[pagePointer]);
    #else
      strcpy(pageTextLong1, pageLastTextLong1[pagePointer]);
      if(bDisplayCont && strlen(pageTextLong1) > 0)
        Serial.println(pageTextLong1);

      strcpy(pageTextLong2, pageLastTextLong2[pagePointer]);
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

  if(bDisplayTrack)
      commandAction((char*)"--sendtrack", false);
  else
      commandAction((char*)"--sendpos", false);
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

  #ifdef BOARD_E290
      sendDisplayMainline();
      E290DisplayUpdate();
  #else
      pageHold=0;

      #if defined(BOARD_TRACKER)
        Serial.println("[BOARD_TRACKER]... GO to deepsleep");
        commandAction((char*)"--deepsleep", isPhoneReady, false);
      #elif defined(BOARD_HELTEC_V2)
        Serial.println("[BOARD_HELTEC_V3]... GO to deepsleep");
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
    #if defined (BOARD_E290)
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
