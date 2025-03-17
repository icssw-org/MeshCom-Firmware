// (C) 2023 OE1KBC Kurt Baumann, OE1KFR Rainer 
// (C) 2016, 2017, 2018, 2018, 2019, 2020 OE1KBC Kurt Baumann
//
// 20230326: Version 4.00: START

#include <Arduino.h>
#include <configuration.h>

#ifdef ESP32

#include <loop_functions.h>
#include <lora_setchip.h>

#include "esp32_flash.h"

#ifdef BOARD_E290
#include "heltec-eink-modules.h"

extern EInkDisplay_VisionMasterE290 e290_display;

#include "Fonts/FreeSans9pt7b.h"
#include "Fonts/FreeSansBold12pt7b.h"
#include "Fonts/FreeSans12pt7b.h"
#include "Fonts/FreeSans18pt7b.h"

#endif

#ifdef BOARD_E290
#else

#include <U8g2lib.h>

extern U8G2 *u8g2;
extern U8G2 u8g2_1;
extern U8G2 u8g2_2;

#endif

#include "esp32_functions.h"

void initDisplay()
{
#ifndef BOARD_E290
    Serial.println(F("[INIT]...Auto detecting display:"));
        
    int idtype = esp32_isSSD1306(0x3C);

    u8g2 = NULL;

    if(idtype < 0)
        return;

    if (idtype == 1)
    { //Address of the display to be checked
        u8g2 = &u8g2_2;
    }
    else
    {
        u8g2 = &u8g2_1;
    }

    u8g2->begin();

#endif

}

void startDisplay(char line1[20], char line2[20], char line3[20])
{
    char cvers[20];

    #ifdef BOARD_E290

    sprintf(cvers, "%s/%-1.1s <%s>", SOURCE_VERSION, SOURCE_VERSION_SUB, getCountry(meshcom_settings.node_country).c_str());

    e290_display.clear();
    e290_display.fastmodeOn();

    e290_display.landscape();

    e290_display.setRotation(270);

    e290_display.fillCircle(10, 10,
        10,                             // Radius: 10px
        BLACK                           // Color: black
        );

    e290_display.setFont( &FreeSansBold12pt7b );
    e290_display.setCursor(20, 50);
    e290_display.printf("MeshCom %s\n", cvers);
    e290_display.setCursor(65, 80);
    e290_display.setFont( &FreeSans12pt7b );
    e290_display.println("HELTEC E290");

    e290_display.setFont( &FreeSans9pt7b );
    e290_display.setCursor(30, 18);
    e290_display.println(line1);
    e290_display.setCursor(80, 100);
    e290_display.println(line2);
    e290_display.setCursor(65, 120);
    e290_display.println(line3);

    e290_display.update();

    #else

    if(u8g2 == NULL)
        return;

    u8g2->clearDisplay();
    u8g2->setFont(u8g2_font_6x10_mf);
    u8g2->firstPage();
    do
    {
        u8g2->setFont(u8g2_font_10x20_mf);
        u8g2->drawStr(5, 15, "MeshCom 4.0");
        u8g2->setFont(u8g2_font_6x10_mf);
        sprintf(cvers, "FW %s/%s <%s>", SOURCE_VERSION, SOURCE_VERSION_SUB, getCountry(meshcom_settings.node_country).c_str());
        u8g2->drawStr(5, 25, cvers);
        u8g2->drawStr(5, 35, line1);
        u8g2->drawStr(5, 45, line2);
        u8g2->drawStr(5, 55, line3);
    } while (u8g2->nextPage());

    #endif
}

#endif
