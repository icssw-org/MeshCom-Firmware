// (C) 2023 OE1KBC Kurt Baumann, OE1KFR Rainer 
// (C) 2016, 2017, 2018, 2018, 2019, 2020 OE1KBC Kurt Baumann
//
// 20230326: Version 4.00: START

#include <Arduino.h>
#include <configuration.h>

#if defined(ESP32)

#include <loop_functions.h>
#include <loop_functions_extern.h>
#include <lora_setchip.h>

#include "printfdeb_functions.h"

#include "esp32_flash.h"

#if defined(BOARD_T5_EPAPER)
// extra source
#elif defined BOARD_TRACKER
#elif defined(BOARD_T_DECK)
#elif defined(BOARD_T_DECK_PLUS)
#elif defined(BOARD_T_DECK_PRO)
#elif defined(BOARD_T_CONNECT_PRO)
#elif (defined(BOARD_E290) || defined(BOARD_WIRELESS_PAPER) || defined(BOARD_E213))
#include "heltec-eink-modules.h"

#if defined(BOARD_E290)
extern EInkDisplay_VisionMasterE290 epaper_display;
#elif (defined(BOARD_WIRELESS_PAPER) || defined(BOARD_E213))
// Display-Treiber wird zur Laufzeit per Chip-ID gewaehlt -> Zeiger statt festem Objekt.
extern BaseDisplay* g_epaper_display;
extern const char*  g_wp_panel_name;
#define epaper_display (*g_epaper_display)
#endif

#include "Fonts/FreeSans9pt7b.h"
#include "Fonts/FreeSansBold10pt7b.h"
#include "Fonts/FreeSansBold12pt7b.h"
#include "Fonts/FreeSans12pt7b.h"
#include "Fonts/FreeSans18pt7b.h"

#else
    #include <U8g2lib.h>
    extern U8G2 *u8g2;
    extern U8G2 u8g2_1;
    extern U8G2 u8g2_2;
#endif   

#if (defined(BOARD_WIRELESS_PAPER) || defined(BOARD_E213))
// Liest die Controller-Chip-ID des E-Ink-Panels per Software-SPI (Bit-Bang) aus.
// Methode wie im Heltec-Factory-Test (Wireless_Paper_E0213A367_FactoryTest):
// Reset -> Befehl 0x2F -> Datenleitung (SDI/MOSI) auf Eingang -> 8 Bit zuruecklesen.
// Unterscheidet E0213A367 (V1.1.1/V1.2) von LCMEN2R13EFC1 (V1.1). Pins = aktive Plattform.
// Alte Chip-ID-Probe (Heltec-Factory-Test, Befehl 0x2F). Durch detectEinkPanelIsE0213()
// (BUSY-Polaritaet, nicht-invasiv) abgeloest, aber als Referenz/Fallback behalten.
static uint8_t __attribute__((unused)) detectEinkChipId()
{
    // E-Ink mit Strom versorgen (VEXT = GPIO45, active LOW)
    pinMode(PIN_PCB_VEXT, OUTPUT);
    digitalWrite(PIN_PCB_VEXT, VEXT_ACTIVE);
    delay(100);

    pinMode(DEFAULT_CLK, OUTPUT);       // SCLK = 3
    pinMode(PIN_DISPLAY_DC, OUTPUT);    // DC   = 5
    pinMode(PIN_DISPLAY_CS, OUTPUT);    // CS   = 4
    pinMode(PIN_DISPLAY_RST, OUTPUT);   // RES  = 6

    digitalWrite(PIN_DISPLAY_RST, LOW);  delay(20);
    digitalWrite(PIN_DISPLAY_RST, HIGH); delay(20);

    digitalWrite(PIN_DISPLAY_DC, LOW);
    digitalWrite(PIN_DISPLAY_CS, LOW);

    // Befehl 0x2F (Read Chip-ID) MSB-first heraustakten
    uint8_t cmd = 0x2F;
    pinMode(DEFAULT_SDI, OUTPUT);       // MOSI/SDI = 2
    digitalWrite(DEFAULT_CLK, LOW);
    for (int i = 0; i < 8; i++)
    {
        digitalWrite(DEFAULT_SDI, (cmd & 0x80) ? HIGH : LOW);
        cmd <<= 1;
        digitalWrite(DEFAULT_CLK, HIGH); delayMicroseconds(1);
        digitalWrite(DEFAULT_CLK, LOW);  delayMicroseconds(1);
    }
    delay(10);

    // Antwort des Controllers ueber dieselbe Datenleitung einlesen
    digitalWrite(PIN_DISPLAY_DC, HIGH);
    pinMode(DEFAULT_SDI, INPUT_PULLUP);

    uint8_t chipId = 0;
    for (int8_t b = 7; b >= 0; b--)
    {
        digitalWrite(DEFAULT_CLK, LOW);  delayMicroseconds(1);
        digitalWrite(DEFAULT_CLK, HIGH); delayMicroseconds(1);
        if (digitalRead(DEFAULT_SDI)) chipId |= (1 << b);
    }
    digitalWrite(PIN_DISPLAY_CS, HIGH);

    return chipId;
}

// Panel-Auto-Erkennung ueber die (invertierte) BUSY-Polaritaet der beiden 2.13"-Controller.
// NICHT-INVASIV: kein SPI-Befehl (anders als detectEinkChipId()/0x2F, der den SSD1680 in einen
// Dauer-BUSY-Zustand bringt -> wait()-Timeout -> Hang). Nach einem reinen Hardware-Reset-PULS und
// kurzer Recovery treibt der Controller den BUSY-Pin auf seinen statischen IDLE-Pegel; dieser ist
// zwischen den Controllern invertiert - bewiesen durch die wait()-Logik der Treiber:
//   - LCMEN2R13EFC1:                wait() == "while(BUSY==LOW)"  -> IDLE = BUSY HIGH
//   - E0213A367 (SSD1680, Base):    wait() == "while(BUSY==HIGH)" -> IDLE = BUSY LOW
// WICHTIG: das gilt fuer den IDLE-Pegel NACH der Recovery. Das fluechtige Busy-Fenster direkt
// nach dem Reset ist genau umgekehrt und timing-kritisch -> bewusst NICHT verwendet. Ebenso wird
// RST nur GEPULST (nicht gehalten): im gehaltenen Reset ist der BUSY-Ausgang undefiniert.
// Rueckgabe: true = E0213A367 (SSD1680), false = LCMEN2R13EFC1.
static bool detectEinkPanelIsE0213()
{
    // E-Ink mit Strom versorgen (VEXT board-spezifisch: WP GPIO45/LOW, E213 GPIO18/HIGH)
    pinMode(PIN_PCB_VEXT, OUTPUT);
    digitalWrite(PIN_PCB_VEXT, VEXT_ACTIVE);
    delay(100);

    pinMode(PIN_DISPLAY_RST, OUTPUT);
    pinMode(PIN_DISPLAY_BUSY, INPUT);

    // Hardware-Reset-PULS (LOW -> HIGH), danach Recovery abwarten
    digitalWrite(PIN_DISPLAY_RST, LOW);  delay(10);
    digitalWrite(PIN_DISPLAY_RST, HIGH);
    delay(100);

    int busy = digitalRead(PIN_DISPLAY_BUSY);
    printfdeb("[INIT]...2.13\" E-Ink BUSY-Probe: idle-Pegel=%s -> %s\n",
              busy == HIGH ? "HIGH" : "LOW",
              busy == LOW  ? "E0213A367" : "LCMEN2R13EFC1");
    return (busy == LOW);   // IDLE LOW = SSD1680/E0213A367 ; IDLE HIGH = LCMEN2R13EFC1
}
#endif

void initDisplay()
{
#if defined(WP_DISP)
    // Gemeinsame Panel-Auto-Erkennung fuer Wireless Paper (V1.1 LCMEN / V1.1.1+V1.2 E0213) UND
    // Vision Master E213. Nicht-invasiv ueber die invertierte BUSY-Polaritaet (siehe
    // detectEinkPanelIsE0213()). Loest die alte 0x2F-Chip-ID-Probe ab: die war auf dem E213
    // unzuverlaessig (waehlte faelschlich LCMEN -> Schlieren) und konnte den SSD1680 aufhaengen.
    if (detectEinkPanelIsE0213())
    {
        g_epaper_display = new EInkDisplay_WirelessPaperV1_2();   // E0213A367-BW (SSD1680)
        g_wp_panel_name  = "E0213A367";
    }
    else
    {
        g_epaper_display = new EInkDisplay_WirelessPaperV1_1();   // LCMEN2R13EFC1 (V1.1)
        g_wp_panel_name  = "LCMEN2R13EFC1";
    }
    printfdeb("[INIT]...2.13\" E-Ink Panel -> %s\n", g_wp_panel_name);
#endif

#if ! (defined(BOARD_E290) || defined(BOARD_WIRELESS_PAPER) || defined(BOARD_E213)) && !defined(BOARD_T_DECK) && !defined(BOARD_T_DECK_PLUS) && !defined(BOARD_TRACKER) && !defined(BOARD_T5_EPAPER) && !defined(BOARD_T_DECK_PRO) && !defined(BOARD_T_CONNECT_PRO)

    printlndeb("[INIT]...Auto detecting display:");
        
    int idtype = esp32_isSSD1306(0x3C);

    // SSD1306 .... idtype 2   u8g2_1
    // SH1106 ..... idtype 1   u8g2_2

    u8g2 = NULL;

    if(idtype < 0)
    {
        bDisplayOff = true;
        return;
    }

    if (idtype == 1)
    {
        u8g2 = &u8g2_1;
    }
    else
    {
        u8g2 = &u8g2_2;
    }

    u8g2->begin();

    u8g2->setContrast(meshcom_settings.node_contrast);

#endif

}

void startDisplay(char line1[20], char line2[20], char line3[20])
{
    #if (defined(BOARD_E290) || defined(BOARD_WIRELESS_PAPER) || defined(BOARD_E213))

    char cvers[20];

    snprintf(cvers, sizeof(cvers), "%s%-1.1s <%s>", SOURCE_VERSION, SOURCE_VERSION_SUB, getCountry(meshcom_settings.node_country).c_str());

    epaper_display.clear();
    epaper_display.fastmodeOn();

    epaper_display.landscape();

    epaper_display.setRotation(90); // top/down (270);

    // Start-Screen im Original-E290-Layout - nur der Board-Name ist board-spezifisch.
    // Der erkannte Panel-Controller wird zusaetzlich als DEBUG-Zeile am Terminal
    // ausgegeben (siehe initDisplay()).
    epaper_display.fillCircle(10, 10, 10, BLACK);
    #if (defined(BOARD_WIRELESS_PAPER) || defined(BOARD_E213))
    // WP / E213 (250px schmal): 10pt-Fettschrift. Titel "MeshCom <version> <land>" so weit nach RECHTS
    // (Richtung Original-Position x=20) setzen, wie er GARANTIERT in eine Zeile passt: Breite
    // messen, x = min(20, 250 - Breite - 4). So nie Umbruch, aber maximal rechts. E290: 12pt @ x20.
    {
      char wtitle[40];
      snprintf(wtitle, sizeof(wtitle), "MeshCom %s", cvers);
      int16_t bx, by; uint16_t bw, bh;
      epaper_display.setFont( &FreeSansBold10pt7b );
      epaper_display.getTextBounds(wtitle, 0, 50, &bx, &by, &bw, &bh);
      int tx = 250 - (int)bw - 4;
      if(tx > 20) tx = 20;
      if(tx < 0)  tx = 0;
      epaper_display.setCursor(tx, 50);
      epaper_display.println(wtitle);
    }
    #else
    epaper_display.setFont( &FreeSansBold12pt7b );
    epaper_display.setCursor(20, 50);
    epaper_display.printf("MeshCom %s\n", cvers);
    #endif
    #if (defined(BOARD_WIRELESS_PAPER) || defined(BOARD_E213))
    // WP / E213 (beide 2.13", 250px): die unteren 3 Zeilen (Board-Name, @BY-Zeile, Rufzeichen)
    // horizontal ZENTRIEREN. Textbreite per getTextBounds messen, x = (Panelbreite 250 - Breite) / 2.
    {
      const int WPW = 250;
      int16_t bx, by; uint16_t bw, bh;
      #if defined(BOARD_E213)
      const char *boardName = "Heltec Vision Master E213";
      // laenger als "Heltec PaperW" -> kleinere 9pt-Schrift, sonst passt es nicht auf 250px
      epaper_display.setFont( &FreeSans9pt7b );
      #else
      const char *boardName = "Heltec PaperW";
      epaper_display.setFont( &FreeSans12pt7b );
      #endif

      epaper_display.getTextBounds(boardName, 0, 80, &bx, &by, &bw, &bh);
      epaper_display.setCursor((WPW - (int)bw) / 2, 80);
      epaper_display.println(boardName);

      epaper_display.setFont( &FreeSans9pt7b );
      epaper_display.getTextBounds(line2, 0, 100, &bx, &by, &bw, &bh);
      epaper_display.setCursor((WPW - (int)bw) / 2, 100);
      epaper_display.println(line2);

      epaper_display.getTextBounds(line3, 0, 120, &bx, &by, &bw, &bh);
      epaper_display.setCursor((WPW - (int)bw) / 2, 120);
      epaper_display.println(line3);

      epaper_display.setCursor(30, 18);          // "...starting now" bleibt oben
      epaper_display.println(line1);
    }
    #else
    epaper_display.setCursor(65, 80);
    epaper_display.setFont( &FreeSans12pt7b );
    epaper_display.println("HELTEC E290");
    epaper_display.setFont( &FreeSans9pt7b );
    epaper_display.setCursor(30, 18);
    epaper_display.println(line1);
    epaper_display.setCursor(80, 100);
    epaper_display.println(line2);
    epaper_display.setCursor(65, 120);
    epaper_display.println(line3);
    #endif

    epaper_display.update();

    #elif defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS) || defined(BOARD_TRACKER) || defined (BOARD_T5_EPAPER) || defined(BOARD_T_DECK_PRO) || defined(BOARD_T_CONNECT_PRO)
    // do nothing
    #else

    char cvers[20];

    if(u8g2 == NULL)
        return;

    u8g2->clearDisplay();
    u8g2->firstPage();

    do
    {
        
        #if defined (BOARD_TRACKER)
        //TODO
        #elif defined (BOARD_STICK_V3)
            u8g2->setFont(u8g2_font_6x10_tf);
            u8g2->drawStr(36, 42, "MeshCom 4");
            snprintf(cvers, sizeof(cvers), "%s/%s %s", SOURCE_VERSION, SOURCE_VERSION_SUB, getCountry(meshcom_settings.node_country).c_str());
            u8g2->drawStr(36, 52, cvers);
            u8g2->drawStr(36, 62, "icssw.org");
        #else
            u8g2->setFont(u8g2_font_10x20_mf);
            u8g2->drawStr(5, 16, "MeshCom 4.0");
            u8g2->setFont(u8g2_font_6x10_mf);
            snprintf(cvers, sizeof(cvers), "FW %s/%s <%s>", SOURCE_VERSION, SOURCE_VERSION_SUB, getCountry(meshcom_settings.node_country).c_str());
            u8g2->drawStr(5, 30, cvers);
            u8g2->drawStr(5, 40, line1);
            u8g2->drawStr(5, 50, line2);
            u8g2->drawStr(5, 60, line3);
        #endif
    } while (u8g2->nextPage());

    #endif
}

#endif