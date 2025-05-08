/**
 * @file        tdeck_main.cpp
 * @brief       general functions for T-Deck
 * @author      Ing. Jakob Gurnhofer (OE3GJC)
 * @author      Ing. Kurt Baumann (OE1KBC)
 * @license     MIT
 * @copyright   Copyright (c) 2025 ICSSW.org
 * @date        2025-03-24
 */
#include <configuration.h>
#include <debugconf.h>
#include "tdeck_main.h"
#include "tdeck_extern.h"
#include <esp32/esp32_flash.h>

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <Wire.h>
#include <SD.h>

#include <AceButton.h>
using namespace ace_button;
#include "event_functions.h"
#include <TFT_eSPI.h>
#include <TouchDrvGT911.hpp>
#include <lvgl.h>
#include "lv_obj_functions.h"
#include "lv_obj_functions_extern.h"
#include <loop_functions_extern.h>

#define I2S_CH I2S_NUM_1

TFT_eSPI            tft;
TouchDrvGT911       touch;

AceButton           button;
SemaphoreHandle_t   xSemaphore = NULL;

bool kbDected = false;

void addMessage(const char*);
bool checkKb();
void scanDevices(TwoWire*);
bool setupCoder();
void setupLvgl();
bool setupSD();

// LVGL functions
static void disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p);
static uint32_t keypad_get_key(void);
static void keypad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data);
static void mouse_read(lv_indev_drv_t *indev, lv_indev_data_t *data);
static void touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data);

LV_IMG_DECLARE(mouse_cursor_icon); 

/**
 * initialization of T-Deck hardware
 */
void initTDeck()
{
    char buf[256];
    bool ret = false;

    Serial.println("[INIT]...initTDeck");

    //! The board peripheral power control pin needs to be set to HIGH when using the peripheral
    pinMode(TDECK_POWERON, OUTPUT);
    digitalWrite(TDECK_POWERON, HIGH);

    //! Set CS on all SPI buses to high level during initialization
    pinMode(LORA_CS, OUTPUT);
    pinMode(TDECK_TFT_CS, OUTPUT);
    pinMode(TDECK_SDCARD_CS, OUTPUT);
    digitalWrite(LORA_CS, HIGH);
    digitalWrite(TDECK_TFT_CS, HIGH);
    digitalWrite(TDECK_SDCARD_CS, HIGH);

    //! Start SPI bus for SD
    //SPI.end();
    pinMode(TDECK_SPI_MISO, INPUT_PULLUP);
    SPI.begin(TDECK_SPI_SCK, TDECK_SPI_MISO, TDECK_SPI_MOSI);

    //! trackball
    pinMode(TDECK_BOOT_PIN, INPUT_PULLUP);
    pinMode(TDECK_TBOX_G01, INPUT_PULLUP);
    pinMode(TDECK_TBOX_G02, INPUT_PULLUP);
    pinMode(TDECK_TBOX_G03, INPUT_PULLUP);
    pinMode(TDECK_TBOX_G04, INPUT_PULLUP);

    button.init();
    ButtonConfig *buttonConfig = button.getButtonConfig();
    buttonConfig->setEventHandler(handleTrackballPressEvent);
    buttonConfig->setFeature(ButtonConfig::kFeatureClick);
    buttonConfig->setFeature(ButtonConfig::kFeatureLongPress);

    //! Add mutex to allow multitasking access
    xSemaphore = xSemaphoreCreateBinary();
    assert(xSemaphore);
    xSemaphoreGive( xSemaphore );

    //! TFT
    #if TFT_BL !=  TDECK_BL_PIN
    #error "Not using the already configured T-Deck file, please remove <Arduino/libraries/TFT_eSPI> and replace with <lib/TFT_eSPI>, please do not click the upgrade library button when opening sketches in ArduinoIDE versions 2.0 and above, otherwise the original configuration file will be replaced !!!"
    #endif
    tft.begin();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    
    //! Touch
    Wire.begin(I2C_SDA, I2C_SCL);
    //scanDevices(&Wire);
    touch.setPins(-1, TDECK_TOUCH_INT);

    bTouchDected = touch.begin(Wire);
    if (bTouchDected)
    {
        touch.setMaxCoordinates(320, 240);
        touch.setSwapXY(true);
        touch.setMirrorXY(false, true);
    }

    //! Keyboard
    kbDected = checkKb();

    //! Screen initialisation
    setupLvgl();
    setDisplayLayout(lv_scr_act());

    //! Map initialization
    init_map();

    Serial.print("[INIT]...MeshCom ");
    Serial.print(SOURCE_VERSION);
    Serial.println(SOURCE_VERSION_SUB);
    snprintf(buf, 256, "MeshCom %s%s\n", SOURCE_VERSION, SOURCE_VERSION_SUB);
    addMessage(buf);

    Serial.print("[INIT]...Touch: ");
    Serial.println(bTouchDected == true ? "OK" : "ERROR");
    snprintf(buf, 256, "%s: %s\n", "Touch", bTouchDected == true ? "OK" : "ERROR");
    addMessage(buf);

    ret = setupSD();
    Serial.print("[INIT]...SDCard: ");
    Serial.println(ret == true ? "OK" : "ERROR");
    snprintf(buf, 256, "%s: %s\n", "SDCard", ret == true ? "OK" : "ERROR");
    addMessage(buf);

    Serial.print("[INIT]...Keyboard: ");
    Serial.println(kbDected == true ? "OK" : "ERROR");
    snprintf(buf, 256, "%s: %s\n", "Keyboard", kbDected == true ? "OK" : "ERROR");
    addMessage(buf);
    
    // play_start_sound();

    // SET Map
    set_map(meshcom_settings.node_map);


    pinMode(TDECK_BOOT_PIN, INPUT);

    while (!digitalRead(TDECK_BOOT_PIN))
    {
        Serial.println("BOOT HAS PRESSED");
        delay(500);
    }

    tdeck_refresh_SET_view();

    lv_textarea_set_text(text_ta, "");

    lv_table_set_cell_value(position_ta, 0, 0, (char*)"Call");
    lv_table_set_cell_value(position_ta, 0, 1, (char*)"Time");
    lv_table_set_cell_value(position_ta, 0, 2, (char*)"Position");

    posrow = 1;

    lv_tabview_set_act(tv, 0, LV_ANIM_OFF);
}

/**
 * initializes SD card
 */
bool setupSD()
{
    if (SD.begin(TDECK_SDCARD_CS, SPI, 800000U)) {
        uint8_t cardType = SD.cardType();
        if (cardType == CARD_NONE) {
            Serial.println("No SD_MMC card attached");
            return false;
        } else {
            Serial.print("SD_MMC Card Type: ");
            if (cardType == CARD_MMC) {
                Serial.println("MMC");
            } else if (cardType == CARD_SD) {
                Serial.println("SDSC");
            } else if (cardType == CARD_SDHC) {
                Serial.println("SDHC");
            } else {
                Serial.println("UNKNOWN");
            }
            uint32_t cardSize = SD.cardSize() / (1024 * 1024);
            uint32_t cardTotal = SD.totalBytes() / (1024 * 1024);
            uint32_t cardUsed = SD.usedBytes() / (1024 * 1024);
            Serial.printf("SD Card Size: %lu MB\n", cardSize);
            Serial.printf("Total space: %lu MB\n",  cardTotal);
            //Serial.printf("Used space: %lu MB\n",   cardUsed);
            return true;
        }
    }
    return false;
}

/**
 * add a message to msg textarea
 */
void addMessage(const char *str)
{
    lv_textarea_add_text(text_ta, str);
    uint32_t run = millis() + 200;
    while (millis() < run)
    {
        lv_task_handler();
        delay(5);
    }
}

/**
 * check for keyboard
 */
bool checkKb()
{
    Wire.requestFrom(0x55, 1);
    return Wire.read() != -1;
}

/**
 * scan for i2c devices
 */
void scanDevices(TwoWire *w)
{
    uint8_t err, addr;
    int nDevices = 0;
    uint32_t start = 0;

    for (addr = 1; addr < 127; addr++)
    {
        start = millis();

        w->beginTransmission(addr); 
        delay(2);
        err = w->endTransmission();

        if (err == 0)
        {
            nDevices++;
            Serial.print("I2C device found at address 0x");
            if (addr < 16)
            {
                Serial.print("0");
            }
            Serial.print(addr, HEX);
            Serial.println(" !");

        } else if (err == 4)
        {
            Serial.print("Unknow error at address 0x");
            if (addr < 16)
            {
                Serial.print("0");
            }
            Serial.println(addr, HEX);
        }
    }

    if (nDevices == 0)
    {
        Serial.println("No I2C devices found\n");
    }
}

/**
 * LVGL initialisation
 */
void setupLvgl()
{
    static lv_disp_draw_buf_t draw_buf;

#ifndef BOARD_HAS_PSRAM
#define LVGL_BUFFER_SIZE    ( TFT_HEIGHT * 100 )
    static lv_color_t buf[ LVGL_BUFFER_SIZE ];
#else
#define LVGL_BUFFER_SIZE    (TFT_WIDTH * TFT_HEIGHT * sizeof(lv_color_t))
    static lv_color_t *buf = (lv_color_t *)ps_malloc(LVGL_BUFFER_SIZE);
    if (!buf) {
        Serial.println("menory alloc failed!");
        delay(5000);
        assert(buf);
    }
#endif


    String LVGL_Arduino = "T-DECK based on Arduino ";
    LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

    Serial.println( LVGL_Arduino );
    Serial.println( "using LVGL_Arduino_Display_LIB" );

    lv_init();

    lv_group_set_default(lv_group_create());

    lv_disp_draw_buf_init( &draw_buf, buf, NULL, LVGL_BUFFER_SIZE );

    /*Initialize the display*/
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init( &disp_drv );

    /*Change the following line to your display resolution*/
    disp_drv.hor_res = TFT_HEIGHT;
    disp_drv.ver_res = TFT_WIDTH;
    disp_drv.flush_cb = disp_flush;
    disp_drv.draw_buf = &draw_buf;
#ifdef BOARD_HAS_PSRAM
    disp_drv.full_refresh = 1;
#endif
    lv_disp_drv_register( &disp_drv );

    /*Initialize the  input device driver*/

    /*Register a touchscreen input device*/
    if (bTouchDected)
    {
        static lv_indev_drv_t indev_touchpad;
        lv_indev_drv_init( &indev_touchpad );
        indev_touchpad.type = LV_INDEV_TYPE_POINTER;
        indev_touchpad.read_cb = touchpad_read;
        touch_indev = lv_indev_drv_register( &indev_touchpad );
    }

    /*Register a mouse input device*/
    static lv_indev_drv_t indev_mouse;
    lv_indev_drv_init( &indev_mouse );
    indev_mouse.type = LV_INDEV_TYPE_POINTER;
    indev_mouse.read_cb = mouse_read;
    mouse_indev = lv_indev_drv_register( &indev_mouse );
    //lv_indev_set_group(mouse_indev, lv_group_get_default());
    lv_indev_set_group(mouse_indev, lv_group_get_default());

    lv_obj_t *cursor_obj;
    cursor_obj = lv_img_create(lv_scr_act());         /*Create an image object for the cursor */
    lv_img_set_src(cursor_obj, &mouse_cursor_icon);   /*Set the image source*/
    lv_indev_set_cursor(mouse_indev, cursor_obj);           /*Connect the image  object to the driver*/

    if (kbDected)
    {
        Serial.println("Keyboard registered");
        /*Register a keypad input device*/
        static lv_indev_drv_t indev_keypad;
        lv_indev_drv_init(&indev_keypad);
        indev_keypad.type = LV_INDEV_TYPE_KEYPAD;
        indev_keypad.read_cb = keypad_read;
        kb_indev = lv_indev_drv_register(&indev_keypad);
        lv_indev_set_group(kb_indev, lv_group_get_default());
    }
}

// !!! LVGL !!!
// !!! LVGL !!!
// !!! LVGL !!!

/**
 * 
 */
static void disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p )
{
    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );

    if ( xSemaphoreTake( xSemaphore, portMAX_DELAY ) == pdTRUE ) {
        tft.startWrite();
        tft.setAddrWindow( area->x1, area->y1, w, h );
        tft.pushColors( ( uint16_t * )&color_p->full, w * h, false );
        tft.endWrite();
        lv_disp_flush_ready( disp );
        
        xSemaphoreGive( xSemaphore );
    }
}

/**
 * Read key value from esp32c3
 */
static uint32_t keypad_get_key(void)
{
    char key_ch = 0;
    Wire.requestFrom(0x55, 1);
    while (Wire.available() > 0) {
        key_ch = Wire.read();

        /*
        if (key_ch != (char)0x00) {
            if (playHandle) {
                vTaskResume(playHandle);
            }
        }
        */

    }

    return key_ch;
}


/**
 * Will be called by the library to read the mouse
 */
static void keypad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data)
{
    static uint32_t last_key = 0;
    uint32_t act_key ;
    act_key = keypad_get_key();
    if (act_key != 0)
    {
        if(iKeyBoardType == 2)
        {
            if(act_key >= 0x61 && act_key <= 0x7a)
                act_key = act_key & 0x5F;
        }
        else
        if(iKeyBoardType == 3)
        {
            if(act_key == 0x77)
                act_key = 0x31;
            else
            if(act_key == 0x65)
                act_key = 0x32;
            else
            if(act_key == 0x72)
                act_key = 0x33;
            else
            if(act_key == 0x73)
                act_key = 0x34;
            else
            if(act_key == 0x64)
                act_key = 0x35;
            else
            if(act_key == 0x66)
                act_key = 0x36;
            else
            if(act_key == 0x7a)
                act_key = 0x37;
            else
            if(act_key == 0x78)
                act_key = 0x38;
            else
            if(act_key == 0x63)
                act_key = 0x39;
            else
            // 0 gibt keine Taste
            if(act_key == 0x76)
                act_key = 0x30;
            else
            if(act_key == 0x6e)
                act_key = 0x2c;
            else
            if(act_key == 0x6d)
                act_key = 0x2e;
        }
    
        if(bDEBUG)
            Serial.printf("Key pressed : iKeyBoardType:%i 0x%x\n", iKeyBoardType, act_key);

        if(!meshcom_settings.node_keyboardlock)
            tft_on();

        bool bSPEC=false;

        if(lv_tabview_get_tab_act(tv) != 1 && lv_tabview_get_tab_act(tv) != 6)
        {
            // spezial Keys
            if(act_key == 0x22) // SYM + L
            {
                meshcom_settings.node_backlightlock = !meshcom_settings.node_backlightlock;
                if(!meshcom_settings.node_backlightlock)
                    tft_off();

                bSPEC=true;
            }

            if(act_key == 0x27) // SYM + K
            {
                meshcom_settings.node_keyboardlock = !meshcom_settings.node_keyboardlock;

                if(meshcom_settings.node_keyboardlock)
                    tft_off();
                else
                    tft_on();

                bSPEC=true;
            }

            if(act_key == 0x2b) // SYM + O
            {
                if (meshcom_settings.node_map > 0)
                    meshcom_settings.node_map--;
                
                set_map(meshcom_settings.node_map);

                bSPEC=true;
            }

            if(act_key == 0x2d) // SYM + I
            {
                if (meshcom_settings.node_map < MAX_MAP-1)
                    meshcom_settings.node_map++;

                set_map(meshcom_settings.node_map);

                bSPEC=true;
            }

            if ((act_key == 0x21) && (!meshcom_settings.node_keyboardlock)) // SYM + B
            {
                cycleBrightness();
                bSPEC=true;
            }

            if ((act_key == 0x2e) && (!meshcom_settings.node_keyboardlock)) // SYM + M
            {
                meshcom_settings.node_mute = !meshcom_settings.node_mute;
            }
        }

        last_key = act_key;

        if(bSPEC || meshcom_settings.node_keyboardlock)
        {
            data->state = LV_INDEV_STATE_REL;
        }
        else
        {
            data->state = LV_INDEV_STATE_PR;
        }
    }
    else
    {
        data->state = LV_INDEV_STATE_REL;
    }
    
    data->key = last_key;
}

/**
 * read trackball input
 */
static void mouse_read(lv_indev_drv_t *indev, lv_indev_data_t *data)
{
    static  int16_t last_x;
    static int16_t last_y;
    bool left_button_down = false;
    const uint8_t dir_pins[5] = {TDECK_TBOX_G02,
                                 TDECK_TBOX_G01,
                                 TDECK_TBOX_G04,
                                 TDECK_TBOX_G03,
                                 TDECK_BOOT_PIN
                                };
    static bool last_dir[5];

    uint8_t pos = 10;
    for (int i = 0; i < 5; i++) {
        bool dir = digitalRead(dir_pins[i]);
        if (dir != last_dir[i])
        {
            if (!meshcom_settings.node_keyboardlock)
                tft_on();

            last_dir[i] = dir;
            switch (i) {
            case 0:
                if (last_x < (lv_disp_get_hor_res(NULL) - mouse_cursor_icon.header.w)) {
                    last_x += pos;
                }
                break;
            case 1:
                if (last_y > mouse_cursor_icon.header.h) {
                    last_y -= pos;
                }
                break;
            case 2:
                if (last_x > mouse_cursor_icon.header.w) {
                    last_x -= pos;
                }
                break;
            case 3:
                if (last_y < (lv_disp_get_ver_res(NULL) - mouse_cursor_icon.header.h)) {
                    last_y += pos;
                }
                break;
            case 4:
                left_button_down = true;
                break;
            default:
                break;
            }
        }
    }
    // Serial.printf("indev:X:%04d  Y:%04d \n", last_x, last_y);
    /*Store the collected data*/
    data->point.x = last_x;
    data->point.y = last_y;
    data->state = left_button_down ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
}

/**
 * Read the touchpad
 */
static void touchpad_read( lv_indev_drv_t *indev_driver, lv_indev_data_t *data )
{
    static int16_t x[5], y[5];

    data->state =  LV_INDEV_STATE_REL;
    if (touch.isPressed())
    {
        uint8_t touched = touch.getPoint(x, y, 1);
        if (!meshcom_settings.node_keyboardlock)
        {
            tft_on();
            if (touched > 0)
            {
                data->state = LV_INDEV_STATE_PR;
                data->point.x = x[0];
                data->point.y = y[0];
            }
        }
    }
}