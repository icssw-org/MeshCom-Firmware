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

#include "esp32_flash.h"

#if defined(BOARD_T5_EPAPER)
// extra source
#elif defined BOARD_TRACKER
#elif defined(BOARD_T_DECK)
#elif defined(BOARD_T_DECK_PLUS)
#elif defined(BOARD_T_DECK_PRO)
#elif defined(BOARD_T_CONNECT_PRO)
#elif (defined(BOARD_E290) || defined(BOARD_WIRELESS_PAPER))
#include "heltec-eink-modules.h"

#if defined(BOARD_E290)
extern EInkDisplay_VisionMasterE290 epaper_display;
#elif defined(BOARD_WIRELESS_PAPER)
// Display-Treiber wird zur Laufzeit per Chip-ID gewaehlt -> Zeiger statt festem Objekt.
extern BaseDisplay* g_epaper_display;
extern const char*  g_wp_panel_name;
#define epaper_display (*g_epaper_display)
#endif

#include "Fonts/FreeSans9pt7b.h"
#include "Fonts/FreeSansBold12pt7b.h"
#include "Fonts/FreeSans12pt7b.h"
#include "Fonts/FreeSans18pt7b.h"

#else
    #include <U8g2lib.h>
    extern U8G2 *u8g2;
    extern U8G2 u8g2_1;
    extern U8G2 u8g2_2;
#endif   

#if defined(BOARD_WIRELESS_PAPER)
// Liest die Controller-Chip-ID des E-Ink-Panels per Software-SPI (Bit-Bang) aus.
// Methode wie im Heltec-Factory-Test (Wireless_Paper_E0213A367_FactoryTest):
// Reset -> Befehl 0x2F -> Datenleitung (SDI/MOSI) auf Eingang -> 8 Bit zuruecklesen.
// Unterscheidet E0213A367 (V1.1.1/V1.2) von LCMEN2R13EFC1 (V1.1). Pins = WirelessPaper-Plattform.
static uint8_t detectEinkChipId()
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
#endif

void initDisplay()
{
#if defined(BOARD_WIRELESS_PAPER)
    // HW-Version anhand der Panel-Chip-ID bestimmen und passenden Treiber instanziieren.
    uint8_t chipId = detectEinkChipId();
    if ((chipId & 0x03) != 0x01)
    {
        g_epaper_display = new EInkDisplay_WirelessPaperV1_1();   // LCMEN2R13EFC1 (V1.1)
        g_wp_panel_name  = "LCMEN2R13EFC1";
    }
    else
    {
        g_epaper_display = new EInkDisplay_WirelessPaperV1_2();   // E0213A367 (V1.0 / V1.1.1 / V1.2)
        g_wp_panel_name  = "E0213A367";
    }
    Serial.printf("[INIT]...Wireless Paper E-Ink chipId=0x%02X -> %s\n", chipId, g_wp_panel_name);
#endif

#if ! (defined(BOARD_E290) || defined(BOARD_WIRELESS_PAPER)) && !defined(BOARD_T_DECK) && !defined(BOARD_T_DECK_PLUS) && !defined(BOARD_TRACKER) && !defined(BOARD_T5_EPAPER) && !defined(BOARD_T_DECK_PRO) && !defined(BOARD_T_CONNECT_PRO)

    Serial.println(F("[INIT]...Auto detecting display:"));
        
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
    #if (defined(BOARD_E290) || defined(BOARD_WIRELESS_PAPER))

    char cvers[20];

    snprintf(cvers, sizeof(cvers), "%s/%-1.1s <%s>", SOURCE_VERSION, SOURCE_VERSION_SUB, getCountry(meshcom_settings.node_country).c_str());

    epaper_display.clear();
    epaper_display.fastmodeOn();

    epaper_display.landscape();

    epaper_display.setRotation(270);

    epaper_display.fillCircle(10, 10,
        10,                             // Radius: 10px
        BLACK                           // Color: black
        );

    epaper_display.setFont( &FreeSansBold12pt7b );
    epaper_display.setCursor(20, 50);
    epaper_display.printf("MeshCom %s\n", cvers);
    #if defined(BOARD_WIRELESS_PAPER)
    epaper_display.setFont( &FreeSans9pt7b );      // 9pt, damit der lange Name in die Zeile passt
    epaper_display.setCursor(20, 72);
    epaper_display.println("Heltec Wireless Paper");
    epaper_display.setCursor(20, 90);
    epaper_display.printf("Panel: %s", g_wp_panel_name);   // zur Laufzeit erkannter Controller
    #else
    epaper_display.setCursor(65, 80);
    epaper_display.setFont( &FreeSans12pt7b );
    epaper_display.println("HELTEC E290");
    #endif

    epaper_display.setFont( &FreeSans9pt7b );
    epaper_display.setCursor(30, 18);
    epaper_display.println(line1);
    epaper_display.setCursor(80, 100);
    epaper_display.println(line2);
    epaper_display.setCursor(65, 120);
    epaper_display.println(line3);

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