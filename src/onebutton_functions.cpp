#include <Arduino.h>

#include "configuration.h"
#include "loop_functions_extern.h"
#include "loop_functions.h"
#include "command_functions.h"
#include "gfx_charmap.h"

#include "onebutton_functions.h"

#if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
#include <lvgl.h>
#include <t-deck/tdeck_main.h>
#include <t-deck/lv_obj_functions.h>
#endif 

#if defined(HAS_TFT) || defined(HAS_TFT_114)
#include "tft_display_functions.h"
#endif

#if defined(BOARD_HELTEC_T114)
#include "t114_gps_page.h"
#endif

#if defined(BOARD_T_ECHO)
#include "nrf52_functions.h"
#endif

#include "page_ring.h"   // DISP-PAGE-01/03: click walk over the message ring; WP_MSG_BROWSE_MAX (the last 4)

#if defined(WP_DISP)
extern bool wpArrived;     // loop_functions.cpp: a message arrived since the last click (T114-DARK-01)
#endif

void singleClick()
{
  if(bDisplayCont)
    Serial.println("SingleClick");

    #ifdef BOARD_T5_EPAPER
        return;
    #endif

  #if defined(BOARD_HELTEC_T114)
  // The T114 draws the page below via displayTFT() directly, which never touches
  // the backlight: with the display off (--display off / double click) the click
  // would paint onto a dark panel. This must run before the early returns below
  // (sendDisplayHead(true) keeps the panel dark while bDisplayIsOff is set) and
  // before the TRACK check, so a click can also wake the display in track mode.
  // Switch the backlight on and, if the display is stored off, arm the same
  // off-timer a received message uses.
  // Every click restarts the 30 s off-timer while the display is stored off (also when it is already lit).
  if(bDisplayOff)
      DisplayOffWait = millis() + (30 * 1000);

  if(bDisplayIsOff)
  {
      bDisplayIsOff=false;
      digitalWrite(PIN_TFT_LEDA_CTL, LOW);   // TFT ON

      if(bDisplayTrack)
      {
          bOneButton = true;   // redraw the track page; the click only wakes the display
          return;
      }

      if(t114GpsPageActive())
      {
          t114GpsPageRedraw();   // the click only wakes the display
          return;
      }
  }
  #endif

  #if defined(BOARD_HELTEC_T114)
  // Track mode keeps the track page, the click only toggles the graphic GPS page.
  if(bDisplayTrack)
  {
      if(t114GpsPageActive())
      {
          t114GpsPageClose();
          pageHold = 0;
          pagePointer = pageLastPointer - 1;
          if(pagePointer < 0)
              pagePointer = PAGE_MAX - 1;
          bOneButton = true;
          sendDisplayHead(true);   // back to the track page
      }
      else
      {
          t114GpsPageOpen();
      }
      return;
  }
  #endif

  // no oneclick on TRACK=on
  if(!bDisplayTrack)
  {
    if(bDisplayCont)
        Serial.printf("BUTTON single press last:%i pointer:%i lines:%i\n", pageLastPointer, pagePointer, pageLastLineAnz[pagePointer]);

    #if defined(BOARD_HELTEC_T114)
    {
      // Page order: stored messages -> status page -> GPS page -> status page (home) ...
      static unsigned long t114StatusAt = 0;   // millis() when the status page was shown by a click

      if(t114GpsPageActive())
      {
        // click on the GPS page: leave it, back to the status page; the next click starts again
        t114GpsPageClose();
        t114StatusAt = 0;
        pageHold = 0;
        pagePointer = pageLastPointer - 1;
        if(pagePointer < 0)
          pagePointer = PAGE_MAX - 1;
        sendDisplayHead(true);
        return;
      }

      // End of the stored messages (an empty slot): the status page
      // first, a click while it is shown opens the GPS page.
      if(pageLastLineAnz[pagePointer] == 0)
      {
        if(t114StatusAt != 0 && (millis() - t114StatusAt) < 6000)
        {
          t114StatusAt = 0;
          t114GpsPageOpen();
          return;
        }
        t114StatusAt = millis();
        sendDisplayHead(true);
        pageHold = 5;
        return;
      }

      t114StatusAt = 0;
    }
    #endif

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

      // DISP-PAGE-03: die Klick-Logik (Status-Schritt, Fenster, Rang) steckt in page_ring.h
      // (wpBrowseClick); liveNewest = die frisch eingetroffene Nachricht ist schon auf dem Panel,
      // der erste Klick geht daher zur naechstaelteren.
      WpBrowseState wpState = { wpStatusStep, wpLiveNewest, pagePointer, wpArrived };
      const WpBrowseResult wpRes = wpBrowseClick(pageLastLineAnz, PAGE_MAX, pageLastPointer, WP_MSG_BROWSE_MAX, wpState);

      if(bDisplayCont)
          Serial.printf("[WPBROWSE];slot;%d;rank;%d;live;%d\n", wpRes.status ? -1 : wpRes.slot, wpRes.rank, wpLiveNewest ? 1 : 0);

      wpStatusStep = wpRes.next.statusStep;
      pagePointer  = wpRes.next.pointer;
      wpLiveNewest = false;
      wpArrived    = false;

      if(wpRes.status)
      {
          sendDisplayHead(true);
          return;
      }

      // Nachricht anzeigen (Helper in loop_functions.cpp; kompaktes Layout + Voll-Refresh).
      wpShowStoredMessage(wpRes.slot, wpRes.rank);

      return;
    }
    #endif

    // DISP-PAGE-01: click order newest -> ... -> oldest -> status page -> newest, on every board; the
    // walk ends at the first empty slot of the ring (page_ring.h), not at the absolute slot 5. Number
    // top right = rank among the stored messages (newest = highest, like E290 and Wireless Paper).
    // An empty ring or an out-of-range pointer (boot) is the status page, never an index below 0.
    const PageClickStep clickStep = pageClickStep(pageLastLineAnz, PAGE_MAX, pageLastPointer, pagePointer);
    if(clickStep.kind == PageClickKind::Status)
    {
        pagePointer = clickStep.nextPointer;
        sendDisplayHead(true);
        return;
    }

    pagePointer = clickStep.slot;
    int pageNo = clickStep.rank;   // number shown top right

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
            pageText[its][ispos+1] = pageNo | 0x30;
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
      #if defined(HAS_LONG_PAGE_TEXT)
      strncpy(pageTextLong1, pageLastTextLong1[pagePointer], sizeof(pageTextLong1));
      if(bDisplayCont && strlen(pageTextLong1) > 0)
        Serial.println(pageTextLong1);

      strncpy(pageTextLong2, pageLastTextLong2[pagePointer], sizeof(pageTextLong2));
      if(bDisplayCont && strlen(pageTextLong2) > 0)
      {
        #if defined(BOARD_E290) || defined(WP_DISP)
        // the page buffer holds GFX font codes (gfx_charmap.h): print readable UTF-8
        char echoUtf8[sizeof(pageTextLong2) * 2 + 1];
        gfx_font_to_utf8(pageTextLong2, strnlen(pageTextLong2, sizeof(pageTextLong2)), echoUtf8, sizeof(echoUtf8));
        Serial.println(echoUtf8);
        #else
        Serial.println(pageTextLong2);
        #endif
      }
      #endif

      sendDisplay1306(false, true, 0, 0, (char*)"#N");
    #endif

    pagePointer = clickStep.nextPointer;

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
    // Follow what the panel actually shows (bDisplayIsOff), not the stored setting
    // (bDisplayOff): after a single click woke a stored-off display, bDisplayOff is
    // still set and the double click would have turned it on permanently.
    if(!bDisplayIsOff)
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

  #if defined(BOARD_HELTEC_T114)
  digitalWrite(PIN_TFT_LEDA_CTL, LOW);   // TFT ON: sendDisplayHead() below can return early and leave the panel dark
  t114GpsPageClose();                    // the track page replaces the GPS page
  pageHold = 0;
  #endif

  if(bDisplayTrack)
  {
      #if defined (ENABLE_GPS) or defined(BOARD_RAK4630) or defined(BOARD_HELTEC_T114) or defined(BOARD_T_ECHO)
      commandAction((char*)"--gps on", false);
      #endif
      commandAction((char*)"--track on", false);
  }
  else
  {
      #if defined (ENABLE_GPS) or defined(BOARD_RAK4630) or defined(BOARD_HELTEC_T114) or defined(BOARD_T_ECHO)
      commandAction((char*)"--gps off", false);
      #endif
      commandAction((char*)"--track off", false);
  }

  bOneButton = true;

  sendDisplayHead(false);
}

#if defined(BOARD_TBEAM) || defined(BOARD_SX1262) || defined(BOARD_SX1268) || defined(BOARD_TBEAM_V3) || defined(BOARD_TBEAM_1W)
// All five T-Beam envs (ttgo_tbeam, ttgo_tbeam_SX1262/SX1268, ttgo_tbeam_supreme, LilyGo_T-Beam-1W).
#define MC_TBEAM_VERYLONG_SLEEP 1
#define TBEAM_SLEEP_PRESS_MS 3000
static bool bTBeamDisplayOffBeforeLong = false;
static bool bTBeamDisplayIsOffBeforeLong = false;
static bool bTBeamPosDisplayBeforeLong = false;
static int uTBeamSsetBeforeLong = 0;
static bool bTBeamSleepFired = false;
void save_settings(void);
void TBeamDuringLongPress();
#endif

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
        // Visible feedback: otherwise the display just goes dark and a long press by
        // accident looks like a crash. The sleep sequence switches the TFT off afterwards.
        digitalWrite(PIN_TFT_LEDA_CTL, LOW);   // TFT ON (it may have been off)
        displayTFT("DEEP SLEEP", "Press button to wake");
        delay(1500);
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
        #if defined(MC_TBEAM_VERYLONG_SLEEP)
        // Display state before this press: a press held on to deep sleep restores it (TBeamDuringLongPress).
        bTBeamDisplayOffBeforeLong = bDisplayOff;
        bTBeamDisplayIsOffBeforeLong = bDisplayIsOff;
        bTBeamPosDisplayBeforeLong = bPosDisplay;
        uTBeamSsetBeforeLong = meshcom_settings.node_sset;
        bTBeamSleepFired = false;
        #endif
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

#if defined(MC_TBEAM_VERYLONG_SLEEP)
// T-Beam: the long press (0.8 s) keeps toggling the display; held for TBEAM_SLEEP_PRESS_MS
// it puts the node into deep sleep. The display toggle of that same press is undone first,
// because bDisplayOff is persisted and would otherwise survive the wake. The ext1 wake on
// the button is armed after the release (esp32WaitButtonRelease(), esp32_sleep.cpp).
void TBeamDuringLongPress()
{
  if(bTBeamSleepFired || btn.getPressedMs() < TBEAM_SLEEP_PRESS_MS)
    return;
  bTBeamSleepFired = true;
  Serial.println("[BOARD_TBEAM]... very long press, GO to deepsleep");
  // Restore the pre-press display state directly: the display-on command would also force
  // posdisplay on (node_sset 0x0004) and change the user's setting.
  if(bDisplayOff != bTBeamDisplayOffBeforeLong)
  {
    bDisplayOff = bTBeamDisplayOffBeforeLong;
    bDisplayIsOff = bTBeamDisplayIsOffBeforeLong;
    bPosDisplay = bTBeamPosDisplayBeforeLong;
    meshcom_settings.node_sset = uTBeamSsetBeforeLong;
    save_settings();
  }
  commandAction((char*)"--deepsleep", isPhoneReady, false);
}
#endif

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
    #if defined(MC_TBEAM_VERYLONG_SLEEP)
    btn.attachDuringLongPress(TBeamDuringLongPress);   // held 3 s -> deep sleep
    #endif
  }
  else
  {
      Serial.printf("[OBUT]...One Button GPIO(%i) not activated\n", iButtonPin);
  }
}

// BTN-OFF-01 (#1294): "--button off" detaches every handler init_onebutton() attached (OneButton
// ignores a null callback), so the button stops acting at once instead of at the next reboot.
// btn.tick() stays unguarded at its call sites, and the INSTRUMENT "--btn" hook still reaches
// the handlers directly. Keep this list in step with init_onebutton().
void deinit_onebutton()
{
    btn.attachClick(nullptr);
    btn.attachDoubleClick(nullptr);
    btn.attachMultiClick(nullptr);
    btn.attachLongPressStart(nullptr);
    #if defined(MC_TBEAM_VERYLONG_SLEEP)
    btn.attachDuringLongPress(nullptr);
    #endif
    btn.reset();   // drop a half-finished gesture

    Serial.printf("[OBUT]...One Button GPIO(%i) detached\n", iButtonPin);
}
// setup

// End
