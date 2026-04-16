// (C) 2023 OE1KBC Kurt Baumann, OE1KFR Rainer 
// (C) 2016, 2017, 2018, 2018, 2019, 2020 OE1KBC Kurt Baumann
//
// 20230326: Version 4.00: START

#include <Arduino.h>
#include <configuration.h>

#include <loop_functions.h>
#include <loop_functions_extern.h>
#include <lora_setchip.h>

#include <nrf52_functions.h>

#if !defined(BOARD_E290) && !defined(BOARD_TRACKER) && !defined(BOARD_HELTEC_T114) && !defined(BOARD_T_ECHO) && !defined (BOARD_T_DECK) && !defined (BOARD_T_DECK_PLUS) && !defined (BOARD_T5_EPAPER) && !defined (BOARD_T_DECK_PRO)

#include <U8g2lib.h>

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
#endif

#if defined(BOARD_T_ECHO)

#include <SPI.h>
#include <Wire.h>

#include "t_echo_utilities.h"
#include "t_echo_images.h"

//Libraries for E-paper Display
#include <GxEPD.h>
#include <GxDEPG0150BN/GxDEPG0150BN.h>  // 1.54" b/w
#include GxEPD_BitmapExamples
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeMonoBold12pt7b.h>
#include <Fonts/FreeMonoBold18pt7b.h>
#include <Fonts/FreeMonoBold24pt7b.h>
#include <GxIO/GxIO_SPI/GxIO_SPI.h>
#include <GxIO/GxIO.h>

extern SPIClass dispPort;

extern GxIO_Class io;
extern GxEPD_Class epaper_display;

void enableBacklight(bool en)
{
    digitalWrite(ePaper_Backlight, en);
}

void LilyGo_logo(void)
{
    epaper_display.setRotation(2);
    epaper_display.fillScreen(GxEPD_WHITE);

    epaper_display.drawExampleBitmap(BitmapCallSign, 0, 0, GxEPD_WIDTH, GxEPD_HEIGHT, GxEPD_BLACK);
    epaper_display.update();
}

void Veille_logo(void)
{
    epaper_display.setRotation(0);
    epaper_display.fillScreen(GxEPD_WHITE);
    epaper_display.drawExampleBitmap(T_Echo_OFF, 0, 0, GxEPD_WIDTH, GxEPD_HEIGHT, GxEPD_BLACK);
    epaper_display.setRotation(3);
    epaper_display.updateWindow(0, 0, GxEPD_WIDTH, GxEPD_HEIGHT, false);
}

void Batterie_Vide_logo(void)
{
    epaper_display.setRotation(0);
    epaper_display.fillScreen(GxEPD_WHITE);
    epaper_display.drawExampleBitmap(Batterie_Vide, 0, 0, GxEPD_WIDTH, GxEPD_HEIGHT, GxEPD_BLACK);
    epaper_display.setRotation(3);
    epaper_display.updateWindow(0, 0, GxEPD_WIDTH, GxEPD_HEIGHT, false);
}

void MeshCom_Image(void)
{
    epaper_display.setRotation(0);
    epaper_display.fillScreen(GxEPD_WHITE);
    epaper_display.drawExampleBitmap(MeshCom_BitMap, 0, 0, GxEPD_WIDTH, GxEPD_HEIGHT, GxEPD_BLACK);
    epaper_display.setRotation(3);
    epaper_display.updateWindow(0, 0, GxEPD_WIDTH, GxEPD_HEIGHT, false);
}


void boardInit()
{
    dispPort.begin();

    uint32_t reset_reason;
    sd_power_reset_reason_get(&reset_reason);
    SerialMon.print("sd_power_reset_reason_get:");
    SerialMon.println(reset_reason, HEX);

    pinMode(Power_Enable_Pin, OUTPUT);
    digitalWrite(Power_Enable_Pin, HIGH);
    //Pwr_en = true;

    pinMode(Power_On_Pin, OUTPUT);
    digitalWrite(Power_On_Pin, HIGH);
    //Pwr_on = true;

    pinMode(ePaper_Backlight, OUTPUT);

    pinMode(GreenLed_Pin, OUTPUT);
    pinMode(RedLed_Pin, OUTPUT);
    pinMode(BlueLed_Pin, OUTPUT);

    pinMode(UserButton_Pin, INPUT_PULLUP);
    pinMode(Touch_Pin, INPUT_PULLUP);
}

void initDisplay()
{
    Serial.println("[DISP]...board init");

    boardInit();

    Serial.println("[DISP]...setup Display");

    epaper_display.update();
    delay(500);

    epaper_display.init(/*115200*/); // enable diagnostic output on Serial
    epaper_display.setRotation(0);
    epaper_display.fillScreen(GxEPD_WHITE);
    epaper_display.setTextColor(GxEPD_BLACK);
    epaper_display.setFont(&FreeMonoBold9pt7b);


    MeshCom_Image();
    delay(3000);

    enableBacklight(false);

    epaper_display.update();
}

void startDisplay(char line1[20], char line2[20], char line3[20])
{
    char cvers[20];

    sprintf(cvers, "%s/%-1.1s" /* <%s>*/, SOURCE_VERSION, SOURCE_VERSION_SUB); //, getCountry(meshcom_settings.node_country).c_str());

    Serial.println("[DISP]...start");

    epaper_display.fillScreen(GxEPD_WHITE);
    epaper_display.setCursor(5, 20);
    epaper_display.printf("MeshCom %s\n", cvers);

    epaper_display.setCursor(5, 60);
    epaper_display.printf("%s\n", line1);

    epaper_display.setCursor(5, 100);
    epaper_display.printf("%s\n", line2);

    epaper_display.setCursor(5, 120);
    epaper_display.printf("%s\n", line3);

    epaper_display.update();
}

#endif
