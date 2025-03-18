// (C) 2023 OE1KBC Kurt Baumann, OE1KFR Rainer 
// (C) 2016, 2017, 2018, 2018, 2019, 2020 OE1KBC Kurt Baumann
//
// 20230326: Version 4.00: START

#include <Arduino.h>
#include <configuration.h>

#include <loop_functions.h>
#include <loop_functions_extern.h>
#include <lora_setchip.h>

#include <U8g2lib.h>

#include <nrf52_functions.h>

extern U8G2 *u8g2;
extern U8G2 u8g2_1;
extern U8G2 u8g2_2;

void initDisplay()
{
    Serial.println(F("[INIT]...Auto detecting display:"));
        
    int idtype = esp32_isSSD1306(0x3C);

    u8g2 = NULL;

    if(idtype < 0)
    {
        bDisplayOff = true;
        return;
    }

    if (idtype == 1)
    { //Address of the display to be checked
        u8g2 = &u8g2_2;
    }
    else
    {
        u8g2 = &u8g2_1;
    }

    u8g2->begin();
}

void startDisplay(char line1[20], char line2[20], char line3[20])
{
    char cvers[20];

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
}
