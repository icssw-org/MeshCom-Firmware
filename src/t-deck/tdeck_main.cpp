/**
 * @file        tdeck_main.cpp
 * @brief       general functions for T-Deck
 * @author      Ing. Jakob Gurnhofer (OE3GJC)
 * @author      Ing. Kurt Baumann (OE1KBC)
 * @author      Ralph Weich (DD5RW)
 * @license     MIT
 * @copyright   Copyright (c) 2025 ICSSW.org
 * @date        2025-12-03
 */
#include <configuration.h>
#include <debugconf.h>
#include "tdeck_main.h"
#include "tdeck_extern.h"
#include "tdeck_helpers.h"
#include "instrument.h"     // TEMPORARY -- measurement scaffolding, see src/instrument.h
#include <esp32/esp32_flash.h>
#include <mheard_functions.h>
#include <time_functions.h>

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <Wire.h>
#include <SD.h>
#include <string.h>
#include "tdeck_sdmap.h"
#include <gps_functions.h>

#include <AceButton.h>
using namespace ace_button;
#include "event_functions.h"
#include <TFT_eSPI.h>
#include <TouchDrvGT911.hpp>
#include <lvgl.h>
#include "lv_obj_functions.h"
#include "lv_obj_functions_extern.h"
#include "tdeck_debug.h"
#include <soc/spi_struct.h>
#include <loop_functions_extern.h>
#include <batt_functions.h>

#if defined(ENABLE_AUDIO)
#include <esp32/esp32_audio.h>
#endif

#define I2S_CH I2S_NUM_1

TFT_eSPI            tft;
TouchDrvGT911       touch;
static const int    TDECK_TOUCH_INIT_TRIES = 5;   // TM-33 (a): begin() retries, 100 ms apart

AceButton           button;
SemaphoreHandle_t   xSemaphore = NULL;

bool kbDected = false;
static bool s_lvglDrawBufPsram = false;

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

static lv_obj_t *trackball_cursor_obj = NULL;
static uint32_t trackball_cursor_visible_until_ms = 0;
static const uint32_t TRACKBALL_CURSOR_SHOW_TIME_MS = 750;

// TM-18: Trackball-Flanken im Interrupt zaehlen. mouse_read() laeuft alle
// 10 ms und verglich bisher nur den Pegel mit dem letzten Pegel: zwei
// Flanken innerhalb eines Intervalls ergaben keinen Schritt, drei einen.
// Im Flankenmodus (Default) verbraucht mouse_read() die gezaehlten Flanken,
// hoechstens BALL_MAX_STEPS_PER_READ pro Achse und Aufruf; der Rest bleibt
// stehen und wird im naechsten Aufruf abgearbeitet. `--balledge on/off`
// schaltet zum Vergleich auf den alten Pegelvergleich zurueck; die Zaehler
// laufen in beiden Modi (`--balledges`).
#define BALL_MAX_STEPS_PER_READ 8
static volatile uint32_t s_ball_edges[4] = {0, 0, 0, 0};       // right, up, left, down (Reihenfolge dir_pins)
static volatile uint32_t s_ball_edges_total[4] = {0, 0, 0, 0};
static uint32_t s_ball_events_total = 0;
static bool s_ball_edge_mode = true;
static portMUX_TYPE s_ball_mux = portMUX_INITIALIZER_UNLOCKED;

static void IRAM_ATTR ball_isr(void *arg)
{
    int i = (int)(intptr_t)arg;
    s_ball_edges[i]++;
    s_ball_edges_total[i]++;
}

extern "C" void tdeck_dbg_balledge(bool on)
{
    s_ball_edge_mode = on;
    Serial.printf("[BALLEDGE];mode;%s\n", on ? "edge" : "level");
}

extern "C" void tdeck_dbg_balledges(bool reset)
{
    Serial.printf("[BALLEDGE];mode;%s;edges;r;%lu;u;%lu;l;%lu;d;%lu;pending;%lu;%lu;%lu;%lu;events;%lu\n",
                  s_ball_edge_mode ? "edge" : "level",
                  (unsigned long)s_ball_edges_total[0], (unsigned long)s_ball_edges_total[1],
                  (unsigned long)s_ball_edges_total[2], (unsigned long)s_ball_edges_total[3],
                  (unsigned long)s_ball_edges[0], (unsigned long)s_ball_edges[1],
                  (unsigned long)s_ball_edges[2], (unsigned long)s_ball_edges[3],
                  (unsigned long)s_ball_events_total);
    if (reset)
    {
        portENTER_CRITICAL(&s_ball_mux);
        for (int i = 0; i < 4; i++) { s_ball_edges[i] = 0; s_ball_edges_total[i] = 0; }
        s_ball_events_total = 0;
        portEXIT_CRITICAL(&s_ball_mux);
    }
}


/**
 * initialization of T-Deck hardware
 */
void initTDeck()
{
    setCpuFrequencyMhz(160);

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
    // TM-18: jede Flanke zaehlen, nicht nur den Pegel alle 10 ms vergleichen
    attachInterruptArg(TDECK_TBOX_G02, ball_isr, (void *)(intptr_t)0, CHANGE);   // right
    attachInterruptArg(TDECK_TBOX_G01, ball_isr, (void *)(intptr_t)1, CHANGE);   // up
    attachInterruptArg(TDECK_TBOX_G04, ball_isr, (void *)(intptr_t)2, CHANGE);   // left
    attachInterruptArg(TDECK_TBOX_G03, ball_isr, (void *)(intptr_t)3, CHANGE);   // down

    button.init();
    ButtonConfig *buttonConfig = button.getButtonConfig();
    buttonConfig->setEventHandler(handleTrackballPressEvent);
    buttonConfig->setFeature(ButtonConfig::kFeatureClick);
    buttonConfig->setFeature(ButtonConfig::kFeatureLongPress);

    //! Add mutex to allow multitasking access
    // Mutex, kein binaeres Semaphor: der Audio-Task (Prio 3) haelt den Bus
    // waehrend SD-Zugriffen, loopTask (Prio 1) waehrend disp_flush() --
    // Prioritaetsvererbung verhindert, dass ein wartender Audio-Task den
    // Flush verhungern laesst.
    xSemaphore = xSemaphoreCreateMutex();
    assert(xSemaphore);

    //! TFT
    #if TFT_BL !=  TDECK_BL_PIN
    #error "Not using the already configured T-Deck file, please remove <Arduino/libraries/TFT_eSPI> and replace with <lib/TFT_eSPI>, please do not click the upgrade library button when opening sketches in ArduinoIDE versions 2.0 and above, otherwise the original configuration file will be replaced !!!"
    #endif
    tft.begin();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    // tft.begin() returns void -- the library gives no failure signal to check.
    Serial.println("[INIT]...TFT: OK");

    //! Touch
    Wire.begin(I2C_SDA, I2C_SCL);
    //scanDevices(&Wire);
    touch.setPins(-1, TDECK_TOUCH_INT);

    // TM-33 (a) / upstream #64 ("touch: failed" at boot, then dead for the
    // session): the GT911 has no reset line on the T-Deck, and a controller
    // that does not answer on I2C yet after power-on (weak battery, slow rail)
    // failed the single begin() for good. begin() itself probes both addresses
    // back-to-back; what was missing is a retry spaced in time. Only a panel
    // that really is absent pays the full wait.
    int touch_tries = 0;
    do
    {
        touch_tries++;
        bTouchDected = touch.begin(Wire);
        if (!bTouchDected && touch_tries < TDECK_TOUCH_INIT_TRIES)
            delay(100);
    } while (!bTouchDected && touch_tries < TDECK_TOUCH_INIT_TRIES);
    if (touch_tries > 1)
        Serial.printf("[INIT]...touch: %s after %d attempts\n", bTouchDected ? "OK" : "ERROR", touch_tries);
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

    init_map();

    Serial.print("[INIT]...MeshCom ");
    Serial.print(SOURCE_VERSION);
    Serial.println(SOURCE_VERSION_SUB);

    Serial.print("[INIT]...Touch: ");
    Serial.println(bTouchDected == true ? "OK" : "ERROR");

    bSDDected = setupSD();
    Serial.print("[INIT]...SDCard: ");
    Serial.println(bSDDected == true ? "OK" : "ERROR");

    if(bSDDected)
    {
        loadMHeardPersistence();
        loadPathPersistence();
        loadTimePersistence();
    }

    Serial.print("[INIT]...Keyboard: ");
    Serial.println(kbDected == true ? "OK" : "ERROR");

    // SET Map
    set_map(meshcom_settings.node_map);

    pinMode(TDECK_BOOT_PIN, INPUT);

    while (!digitalRead(TDECK_BOOT_PIN))
    {
        Serial.println("BOOT HAS PRESSED");
        delay(500);
    }

    tdeck_refresh_SET_view();

    lv_table_set_cell_value(position_ta, 0, 0, (char*)"Call");
    lv_table_set_cell_value(position_ta, 0, 1, (char*)"Time");
    lv_table_set_cell_value(position_ta, 0, 2, (char*)"Position");

    posrow = 1;

    loadPosPersistence();

    lv_tabview_set_act(tv, 0, LV_ANIM_OFF);

    char buf[40];
    snprintf(buf, 40, "MeshCom %s%s", SOURCE_VERSION, SOURCE_VERSION_SUB);
    addMessage(buf);

    snprintf(buf, 40, "(build: %s / %s)", __DATE__, __TIME__);
    addMessage(buf);

    Serial.printf("[BOOT];init;sd;%d;touch;%d;touch_tries;%d;kb;%d;psram_buf;%d;t_ms;%lu\n",
                  bSDDected ? 1 : 0,
                  bTouchDected ? 1 : 0, touch_tries,
                  kbDected ? 1 : 0,
                  s_lvglDrawBufPsram ? 1 : 0,
                  (unsigned long)millis());
}

void startAudio()
{
    // Nur einreihen; ob die Datei existiert, entscheidet der Audio-Task
    // (Ersatz: CW-Startkennung). Die Startsequenz wartet nicht auf den Ton.
    Serial.printf("[BOOT];audio;queued;%s\n", meshcom_settings.node_audio_start.c_str());
    audio_play_file_or_cw(meshcom_settings.node_audio_start.c_str(), 12, AUDIO_CW_START);
}

/**
 * initializes SD card
 */
bool setupSD()
{
    if (SD.begin(TDECK_SDCARD_CS, SPI, 20000000U)) {   // was 800000U; 40 MHz measured identical (read path ~1.3 MB/s, decode dominates)
        uint8_t cardType = SD.cardType();
        if (cardType == CARD_NONE) {
            Serial.println("[INIT]...No SD_MMC card attached");
            return false;
        } else {
            Serial.print("[INIT]...SD_MMC Card Type: ");
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
            //not used uint32_t cardUsed = SD.usedBytes() / (1024 * 1024);
            Serial.printf("[INIT]...SD Card Size: %u MB\n", cardSize);
            Serial.printf("[INIT]...Total space: %u MB\n",  cardTotal);
            //Serial.printf("Used space: %lu MB\n",   cardUsed);
            sdmap_init();

            for (int i = 0; i < SDMAP_SET_COUNT; i++)
            {
                if (i < sdmap_get_set_count())
                    strMaps[i] = String(sdmap_get_set_name(i));
                else
                    strMaps[i] = "-";
            }
            
            if (meshcom_settings.node_map >= sdmap_get_set_count())
            {
                Serial.printf("[ SDMAP ]...Gespeicherte Kartenauswahl (%d) nicht gefunden, falle zurueck auf Set 0\n", meshcom_settings.node_map);
                meshcom_settings.node_map = 0;
            }
            lv_dropdown_set_options(dropdown_mapselect, getMapDropbox().c_str());
            return true;
        }
    }
    Serial.println("[INIT]...SD.begin() failed (no card detected or mount error)");
    return false;
}

/**
 * add a message to msg textarea
 */
void addMessage(const char *str)
{
    Serial.printf("[BOOT];msg;%s\n", str);
    tdeck_add_system_message(str);
    // LVGL kurz laufen lassen, damit die Blase sichtbar wird -- nicht 2 s
    // warten: sechs Startmeldungen kosteten 12 s Bootzeit (TM-15), und die
    // Meldungen bleiben ohnehin als System-Blasen in der Nachrichtenliste.
    uint32_t run = millis() + 100;
    while ((int32_t)(millis() - run) < 0)
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
    //not unsed uint32_t start = 0;

    for (addr = 1; addr < 127; addr++)
    {
        //not used start = millis();

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

    #define LVGL_BUFFER_SIZE    (TFT_WIDTH * TFT_HEIGHT * sizeof(lv_color_t))
    static lv_color_t *buf = (lv_color_t *)ps_malloc(LVGL_BUFFER_SIZE);
    if (buf == NULL)
    {
        Serial.println("[INIT]...LVGL draw buffer: PSRAM allocation failed, falling back to internal RAM");
        // Fallback to internal RAM if PSRAM fails
        buf = (lv_color_t *)malloc(LVGL_BUFFER_SIZE);
        if (buf == NULL)
        {
            Serial.println("[INIT]...LVGL draw buffer: FAIL (internal RAM allocation failed too)");
        }
        else
        {
            Serial.println("[INIT]...LVGL draw buffer: internal");
        }
    }
    else
    {
        s_lvglDrawBufPsram = true;
        Serial.println("[INIT]...LVGL draw buffer: PSRAM");
    }


    String LVGL_Arduino = "[INIT]...T-DECK based on Arduino ";
    LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

    Serial.println( LVGL_Arduino );
    Serial.println( "[INIT]...using LVGL_Arduino_Display_LIB" );

    lv_init();

    lv_group_set_default(lv_group_create());

    // G07: the 4th parameter is size_in_px_cnt, not a byte count
    // (lv_hal_disp.h:223). LVGL_BUFFER_SIZE is TFT_WIDTH*TFT_HEIGHT*sizeof(lv_color_t),
    // i.e. twice the pixel count. Harmless while full_refresh=1 and buf2==NULL keep
    // the partial-render paths unreachable; with partial refresh it is a ~150 KB overflow.
    lv_disp_draw_buf_init( &draw_buf, buf, NULL, TFT_WIDTH * TFT_HEIGHT );

    /*Initialize the display*/
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init( &disp_drv );

    /*Change the following line to your display resolution*/
    disp_drv.hor_res = TFT_HEIGHT;
    disp_drv.ver_res = TFT_WIDTH;
    disp_drv.flush_cb = disp_flush;
    disp_drv.draw_buf = &draw_buf;
    disp_drv.full_refresh = 0;   // EXPERIMENT: partial refresh, no other changes
    disp_drv.monitor_cb = tdeck_dbg_monitor_cb;
    disp_drv.render_start_cb = tdeck_dbg_render_start_cb;
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
    else
    {
        Serial.println("[INIT]...Touch indev: SKIPPED (not detected)");
    }

    /*Register a mouse input device*/
    static lv_indev_drv_t indev_mouse;
    lv_indev_drv_init( &indev_mouse );
    indev_mouse.type = LV_INDEV_TYPE_POINTER;
    indev_mouse.read_cb = mouse_read;
    mouse_indev = lv_indev_drv_register( &indev_mouse );
    //lv_indev_set_group(mouse_indev, lv_group_get_default());
    lv_indev_set_group(mouse_indev, lv_group_get_default());

    trackball_cursor_obj = lv_img_create(lv_scr_act());
    lv_img_set_src(trackball_cursor_obj, &mouse_cursor_icon);
    lv_obj_add_flag(trackball_cursor_obj, LV_OBJ_FLAG_HIDDEN);
    trackball_cursor_visible_until_ms = 0;
    lv_indev_set_cursor(mouse_indev, trackball_cursor_obj);

    if (kbDected)
    {
        Serial.println("[INIT]...Keyboard registered");
        /*Register a keypad input device*/
        static lv_indev_drv_t indev_keypad;
        lv_indev_drv_init(&indev_keypad);
        indev_keypad.type = LV_INDEV_TYPE_KEYPAD;
        indev_keypad.read_cb = keypad_read;
        kb_indev = lv_indev_drv_register(&indev_keypad);
        lv_indev_set_group(kb_indev, lv_group_get_default());
    }
    else
    {
        Serial.println("[INIT]...Keyboard registered: SKIPPED (not detected)");
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

    // TM-41: --disptest owns the panel while it runs. Report the area as done
    // (LVGL must not stall) but push nothing -- the whole screen is repainted
    // from lv_obj_invalidate() when the test hands the panel back.
    if(tdeck_dbg_disptest_running()) { lv_disp_flush_ready( disp ); return; }

    if ( xSemaphoreTake( xSemaphore, portMAX_DELAY ) == pdTRUE ) {
        INSTR_T0(_instr_flush_t0);          // TEMPORARY -- see src/instrument.h
        // EXPERIMENT (flushfix): no other task may run while the transfer is on the wire.
        if(tdeck_dbg_redrawlog_enabled() && w == (uint32_t)TFT_HEIGHT && h == (uint32_t)TFT_WIDTH) {
            // EXPERIMENT: bus state right before a full-screen transfer.
            Serial.printf("[BUS];sd_cs;%d;lora_cs;%d;tft_cs;%d;tft_dc;%d;spi2_clock;%08lx;spi2_user;%08lx;spi2_ctrl;%08lx\n",
                          digitalRead(TDECK_SDCARD_CS), digitalRead(LORA_CS), digitalRead(TDECK_TFT_CS), digitalRead(TDECK_TFT_DC),
                          (unsigned long)GPSPI2.clock.val, (unsigned long)GPSPI2.user.val, (unsigned long)GPSPI2.ctrl.val);
        }
        // TM-07: pre-transfer snapshot -- must run BEFORE the flushfix NOP below,
        // which re-arms the registers and would mask a foreign user's state.
        // Prints [SPITRACE];clobber;... when a register differs from the state
        // the previous flush left behind.
        if(tdeck_dbg_spitrace_enabled()) {
            tdeck_dbg_spitrace_preflush();
        }
        // EXPERIMENT (flushfix): one throw-away display transaction before the real one,
        // so that the SPI peripheral state left behind by another bus user (SD card at
        // 800 kHz) is re-armed before the frame goes out.
        if(tdeck_dbg_flushfix_enabled()) {
            tft.startWrite();
            tft.writecommand(0x00);         // NOP
            tft.endWrite();
        }
        tft.startWrite();
        tft.setAddrWindow( area->x1, area->y1, w, h );
        tft.pushColors( ( uint16_t * )&color_p->full, w * h, false );
        tft.endWrite();
        INSTR_FLUSH(_instr_flush_t0);       // TEMPORARY
        // TM-07: post-transfer bus-register snapshot + SD/LoRa user counts
        // since the previous flush, see tdeck_debug.h for the line format.
        if(tdeck_dbg_spitrace_enabled()) {
            tdeck_dbg_spitrace_flush();
        }
        if(tdeck_dbg_framedump_armed() && w == (uint32_t)TFT_HEIGHT && h == (uint32_t)TFT_WIDTH) {
            // One-shot ASCII dump of the full frame: every 4th row, every 4th column.
            // ' ' dark, '.' dim, '+' mid, '#' bright (luminance from RGB565, byte-swapped).
            tdeck_dbg_framedump_arm(false);
            const uint8_t * pb = (const uint8_t *)&color_p->full;
            Serial.printf("[FRAME];begin;w;%lu;h;%lu;step;4\n", (unsigned long)w, (unsigned long)h);
            char line[TFT_HEIGHT / 4 + 8];
            for(uint32_t y = 0; y < h; y += 4) {
                int o = 0;
                for(uint32_t x = 0; x < w; x += 4) {
                    uint32_t i = (y * w + x) * 2;
                    uint16_t raw = ((uint16_t)pb[i] << 8) | pb[i + 1];
                    uint32_t r = (raw >> 11) & 0x1f, g = (raw >> 5) & 0x3f, b = raw & 0x1f;
                    uint32_t lum = r * 2 + g + b * 2;            /* 0..188 */
                    line[o++] = lum < 30 ? ' ' : lum < 80 ? '.' : lum < 140 ? '+' : '#';
                }
                line[o] = 0;
                Serial.printf("[FRAME];%03lu;%s\n", (unsigned long)y, line);
            }
            Serial.println("[FRAME];end");
        }
        if(tdeck_dbg_redrawlog_enabled()) {
            // CRC32 of the flushed pixels: tells whether two flushes carried the same content.
            uint32_t crc = 0xFFFFFFFFu, crcm = 0xFFFFFFFFu;
            const uint8_t * pb = (const uint8_t *)&color_p->full;
            for(uint32_t row = 0; row < h; row++) {
                int y = area->y1 + (int)row;
                bool in_msg = (y >= 40 && y <= 193);           // message-list rows only
                const uint8_t * pr = pb + row * w * 2;
                for(uint32_t n = 0; n < w * 2; n++) {
                    uint8_t b = pr[n];
                    crc ^= b;
                    for(int k = 0; k < 8; k++) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
                    if(in_msg) {
                        crcm ^= b;
                        for(int k = 0; k < 8; k++) crcm = (crcm >> 1) ^ (0xEDB88320u & (0u - (crcm & 1u)));
                    }
                }
            }
            Serial.printf("[FLUSH];ms;%lu;area;%d;%d;%d;%d;px;%lu;sleeping;%d;bl;%u;crc;%08lx;crcm;%08lx\n",
                          (unsigned long)millis(), (int)area->x1, (int)area->y1, (int)area->x2, (int)area->y2,
                          (unsigned long)(w * h), tft_is_sleeping ? 1 : 0, (unsigned)current_brightness_level,
                          (unsigned long)(crc ^ 0xFFFFFFFFu), (unsigned long)(crcm ^ 0xFFFFFFFFu));
        }
        lv_disp_flush_ready( disp );

        xSemaphoreGive( xSemaphore );
    }
}

// EXPERIMENT: transfer-reliability probe. Pushes the current frame and its inverted
// copy alternately, `n` times, 500 ms apart. A lost transfer shows as a skipped toggle.
void tdeck_dbg_blink(int n)
{
    lv_disp_t * d = lv_disp_get_default();
    if(d == NULL || d->driver == NULL || d->driver->draw_buf == NULL) { Serial.println("[BLINK];err;nodisp"); return; }
    uint16_t * src = (uint16_t *)d->driver->draw_buf->buf1;
    static uint16_t * scratch = NULL;
    const uint32_t npx = TFT_WIDTH * TFT_HEIGHT;
    if(scratch == NULL) scratch = (uint16_t *)ps_malloc(npx * 2);
    if(src == NULL || scratch == NULL) { Serial.println("[BLINK];err;nobuf"); return; }
    for(uint32_t i = 0; i < npx; i++) scratch[i] = src[i] ^ 0xFFFF;
    for(int k = 0; k < n; k++) {
        uint16_t * p = (k & 1) ? src : scratch;
        uint32_t t0 = micros();
        if ( xSemaphoreTake( xSemaphore, portMAX_DELAY ) == pdTRUE ) {
            tft.startWrite();
            tft.setAddrWindow( 0, 0, TFT_HEIGHT, TFT_WIDTH );
            tft.pushColors( p, npx, false );
            tft.endWrite();
            xSemaphoreGive( xSemaphore );
        }
        Serial.printf("[BLINK];%d;%s;us;%lu\n", k, (k & 1) ? "normal" : "inverted", (unsigned long)(micros() - t0));
        delay(500);
    }
    Serial.println("[BLINK];done");
}

// EXPERIMENT: push the current LVGL draw buffer to the panel again without rendering.
void tdeck_dbg_reflush(void)
{
    lv_disp_t * d = lv_disp_get_default();
    if(d == NULL || d->driver == NULL || d->driver->draw_buf == NULL) { Serial.println("[REFLUSH];err;nodisp"); return; }
    lv_color_t * b = (lv_color_t *)d->driver->draw_buf->buf1;
    if(b == NULL) { Serial.println("[REFLUSH];err;nobuf"); return; }
    if ( xSemaphoreTake( xSemaphore, portMAX_DELAY ) == pdTRUE ) {
        tft.startWrite();
        tft.setAddrWindow( 0, 0, TFT_HEIGHT, TFT_WIDTH );
        tft.pushColors( ( uint16_t * )&b->full, TFT_WIDTH * TFT_HEIGHT, false );
        tft.endWrite();
        xSemaphoreGive( xSemaphore );
    }
    Serial.printf("[REFLUSH];ok;px;%lu\n", (unsigned long)(TFT_WIDTH * TFT_HEIGHT));
}

/**
 * Read key value from esp32c3
 */
// Bench-Harness: eingereihte Tasten und Trackball-Schritte (tdeck_debug.h).
#define DBG_KEY_RING 32
static volatile uint32_t s_dbg_key_ring[DBG_KEY_RING];
static volatile uint8_t  s_dbg_key_head = 0, s_dbg_key_tail = 0;
static volatile int      s_dbg_ball_pending[5] = {0, 0, 0, 0, 0};

extern "C" bool tdeck_dbg_inject_key(uint32_t code)
{
    uint8_t next = (uint8_t)((s_dbg_key_head + 1) % DBG_KEY_RING);
    if (next == s_dbg_key_tail)
        return false;                       // Ring voll
    s_dbg_key_ring[s_dbg_key_head] = code;
    s_dbg_key_head = next;
    return true;
}

extern "C" void tdeck_dbg_inject_ball(int dir, int n)
{
    if (dir < 0 || dir >= 5)
        return;
    if (dir < 4 && s_ball_edge_mode)
    {
        portENTER_CRITICAL(&s_ball_mux);
        s_ball_edges[dir] += n;
        s_ball_edges_total[dir] += n;
        portEXIT_CRITICAL(&s_ball_mux);
    }
    else
        s_dbg_ball_pending[dir] += n;
}

// Bench-Harness (TM-19): synthetic touch injection, consumed by
// touchpad_read() before it polls the GT911 hardware. tap/down/up match the
// --touch serial command (tdeck_debug.h).
enum { TDECK_TOUCH_TAP = 0, TDECK_TOUCH_DOWN, TDECK_TOUCH_UP };
#define TDECK_TOUCH_QUEUE 4
struct tdeck_touch_cmd_t { uint8_t op; int16_t x, y; uint16_t dur_ms; };
static volatile tdeck_touch_cmd_t s_touch_queue[TDECK_TOUCH_QUEUE];
static volatile uint8_t           s_touch_q_head = 0, s_touch_q_tail = 0;

// Injected press state consumed by touchpad_read(); s_touch_inj_auto_release
// distinguishes a tap's timed release from a down/up held press.
static bool     s_touch_inj_active = false;
static bool     s_touch_inj_auto_release = false;
static int16_t  s_touch_inj_x = 0, s_touch_inj_y = 0;
static uint32_t s_touch_inj_release_ms = 0;

extern "C" bool tdeck_touch_inject(const char *subcmd, int x, int y, int dur_ms)
{
    uint8_t op;
    if      (strcmp(subcmd, "tap")  == 0) op = TDECK_TOUCH_TAP;
    else if (strcmp(subcmd, "down") == 0) op = TDECK_TOUCH_DOWN;
    else if (strcmp(subcmd, "up")   == 0) op = TDECK_TOUCH_UP;
    else { Serial.println("[TOUCH];err;usage"); return false; }

    if (op != TDECK_TOUCH_UP && (x < 0 || x >= 320 || y < 0 || y >= 240))
    {
        Serial.println("[TOUCH];err;range");
        return false;
    }

    uint8_t next = (uint8_t)((s_touch_q_head + 1) % TDECK_TOUCH_QUEUE);
    if (next == s_touch_q_tail)
    {
        Serial.println("[TOUCH];err;full");
        return false;                        // Ring voll
    }

    // Field-by-field: a volatile struct has no implicit (volatile-qualified)
    // copy assignment/constructor, so `queue[i] = cmd;` does not compile.
    s_touch_queue[s_touch_q_head].op     = op;
    s_touch_queue[s_touch_q_head].x      = (int16_t)x;
    s_touch_queue[s_touch_q_head].y      = (int16_t)y;
    s_touch_queue[s_touch_q_head].dur_ms = (uint16_t)((op == TDECK_TOUCH_TAP) ? ((dur_ms > 0) ? dur_ms : 50) : 0);
    s_touch_q_head = next;
    return true;
}

static uint32_t keypad_get_key(void)
{
    char key_ch = 0;
    if (s_dbg_key_tail != s_dbg_key_head)   // eingereihte Taste zuerst
    {
        uint32_t code = s_dbg_key_ring[s_dbg_key_tail];
        s_dbg_key_tail = (uint8_t)((s_dbg_key_tail + 1) % DBG_KEY_RING);
        Serial.printf("[KEY];%02lx;ms;%lu;src;inject\n", (unsigned long)code, (unsigned long)millis());
        return code;
    }
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

    if (key_ch != 0)
        Serial.printf("[KEY];%02x;ms;%lu;src;kbd\n", (unsigned)(uint8_t)key_ch, (unsigned long)millis());
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
        bool bSPEC=false;

        // Map-context keys (TD-08 fix): dispatched on the raw key, before the
        // iKeyBoardType remaps below, so the symbol-input mode (type 4) can no
        // longer eat them the way it already ate g/h. Map tab only (tab_act
        // == 3) -- there is no text field there, so intercepting these seven
        // keys ahead of the remap cannot affect typing on any other tab.
        if(lv_tabview_get_tab_act(tv) == 3)
        {
            switch (act_key)
            {
                case 0x67: case 0x47: // g / G -> zoom in
                    tdeck_map_zoom(1);
                    bSPEC=true;
                    break;
                case 0x68: case 0x48: // h / H -> zoom out
                    tdeck_map_zoom(-1);
                    bSPEC=true;
                    break;
                case 0x69: case 0x49: // i / I -> pan up
                    tdeck_map_pan(0, -(sdmap_view_h() / 4));
                    bSPEC=true;
                    break;
                case 0x6a: case 0x4a: // j / J -> pan left
                    tdeck_map_pan(-(sdmap_view_w() / 4), 0);
                    bSPEC=true;
                    break;
                case 0x6b: case 0x4b: // k / K -> pan down
                    tdeck_map_pan(0, sdmap_view_h() / 4);
                    bSPEC=true;
                    break;
                case 0x6c: case 0x4c: // l / L -> pan right
                    tdeck_map_pan(sdmap_view_w() / 4, 0);
                    bSPEC=true;
                    break;
                case 0x6f: case 0x4f: // o / O -> recenter on own position
                    tdeck_map_recenter();
                    bSPEC=true;
                    break;
                default:
                    break;
            }
        }

        if(bSPEC)
        {
            // Consumed above -- skip the remap and the rest of the special-key
            // chain below for this key.
        }
        else
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
        else if(iKeyBoardType == 4)
        {
            if(act_key >= 0x61 && act_key <= 0x7a)
            {
                // Index 0..25 corresponds to 'a'..'z'.
                static const char sym_map[26] = {
                    '*', // a  -> keyboard_symbol[0][3]
                    '!', // b  -> keyboard_symbol[3][4]
                    '9', // c  -> keyboard_symbol[2][5]
                    '5', // d  -> keyboard_symbol[1][2]
                    '2', // e  -> keyboard_symbol[1][0]
                    '6', // f  -> keyboard_symbol[2][6]
                    '/', // g  -> keyboard_symbol[2][1]
                    ':', // h  -> keyboard_symbol[3][1]
                    '-', // i  -> keyboard_symbol[4][2]
                    ';', // j  -> keyboard_symbol[3][6]
                    '\'',// k  -> keyboard_symbol[4][6] (apostrophe)
                    '"',// l  -> keyboard_symbol[4][1]
                    '.', // m  -> keyboard_symbol[4][5]
                    ',', // n  -> keyboard_symbol[3][5]
                    '+', // o  -> keyboard_symbol[4][0]
                    '@', // p  -> keyboard_symbol[1][3]
                    '#', // q  -> keyboard_symbol[0][0]
                    '3', // r  -> keyboard_symbol[2][0]
                    '4', // s  -> keyboard_symbol[1][1]
                    '(', // t  -> keyboard_symbol[2][2]
                    '_', // u  -> keyboard_symbol[3][0]
                    '?', // v  -> keyboard_symbol[2][4]
                    '1', // w  -> keyboard_symbol[0][1]
                    '8', // x  -> keyboard_symbol[1][4]
                    ')', // y  -> keyboard_symbol[3][2]
                    '7'  // z  -> keyboard_symbol[1][5]
                };

                act_key = (uint32_t)sym_map[act_key - 0x61];
            }
            else
            {
                // keep dot/comma mapping too
                if(act_key == 0x6d)
                    act_key = 0x2e;
                else if(act_key == 0x6e)
                    act_key = 0x2c;
            }
        }
    
        

        if(!meshcom_settings.node_keyboardlock)
            tft_on();

        // NOTE: bSPEC is declared above, before the map-key dispatch (TD-08) --
        // do not redeclare it here, or a map key already consumed above would
        // be reset to "not special".

        if(lv_tabview_get_tab_act(tv) != 1 && lv_tabview_get_tab_act(tv) != 7)
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

            // g/h/i/j/k/l/o on the map tab are dispatched earlier, ahead of the
            // SYM remap -- see the switch at the top of this function (TD-08).

            if(act_key == 0x2b) // SYM + O -> Zoom raus (wie Touch-Button)
            {
                tdeck_map_zoom(-1);
                bSPEC=true;
            }

            if(act_key == 0x2d) // SYM + I -> Zoom rein (wie Touch-Button)
            {
                tdeck_map_zoom(1);
                bSPEC=true;
            }

            if ((act_key == 0x21) && (!meshcom_settings.node_keyboardlock)) // SYM + B
            {
                cycleBrightness();
                bSPEC=true;
            }

            if ((act_key == 0x2e) && (!meshcom_settings.node_keyboardlock)) // SYM + M
            {
                #if defined(ENABLE_AUDIO)
                audio_set_mute(!meshcom_settings.node_mute);
                #else
                meshcom_settings.node_mute = !meshcom_settings.node_mute;
                #endif
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
    bool activity_detected = false;
    const uint8_t dir_pins[5] = {TDECK_TBOX_G02,
                                 TDECK_TBOX_G01,
                                 TDECK_TBOX_G04,
                                 TDECK_TBOX_G03,
                                 TDECK_BOOT_PIN
                                };
    static bool last_dir[5];
    static bool last_dir_initialized = false;

    if(!last_dir_initialized)
    {
        for(int i = 0; i < 5; ++i)
            last_dir[i] = digitalRead(dir_pins[i]);
        last_dir_initialized = true;
    }

    uint8_t pos = 10;
    int steps_this_read = 0;
    for (int i = 0; i < 5; i++) {
        bool dir = digitalRead(dir_pins[i]);
        int steps = 0;
        if (i < 4 && s_ball_edge_mode)
        {
            // Flankenmodus: gezaehlte Flanken verbrauchen, Pegel nur mitfuehren
            // Alle gezaehlten Flanken abholen; mehr als BALL_MAX_STEPS_PER_READ
            // pro Aufruf werden verworfen, nicht nachgeholt: sonst laeuft der
            // Cursor nach einem Stillstand der Schleife (z.B. WLAN-Scan) noch
            // sekundenlang die waehrenddessen gerollten Flanken ab.
            portENTER_CRITICAL(&s_ball_mux);
            uint32_t n = s_ball_edges[i];
            s_ball_edges[i] = 0;
            portEXIT_CRITICAL(&s_ball_mux);
            if (n > BALL_MAX_STEPS_PER_READ) n = BALL_MAX_STEPS_PER_READ;
            steps = (int)n;
            last_dir[i] = dir;
        }
        else
        {
            // Pegelvergleich (alter Weg, Taste immer so)
            // Bench-Harness: ein eingereihter Schritt wirkt wie eine Flanke
            if (s_dbg_ball_pending[i] > 0)
            {
                s_dbg_ball_pending[i]--;
                dir = !last_dir[i];
            }
            if (dir != last_dir[i])
            {
                last_dir[i] = dir;
                steps = 1;
            }
        }
        for (int k = 0; k < steps; k++)
        {
            if (!meshcom_settings.node_keyboardlock)
                tft_on();
            s_ball_events_total++;
            steps_this_read++;
            switch (i) {
            case 0:
                if (last_x < (lv_disp_get_hor_res(NULL) - mouse_cursor_icon.header.w)) {
                    last_x += pos;
                    activity_detected = true;
                }
                break;
            case 1:
                if (last_y > mouse_cursor_icon.header.h) {
                    last_y -= pos;
                    activity_detected = true;
                }
                break;
            case 2:
                if (last_x > mouse_cursor_icon.header.w) {
                    last_x -= pos;
                    activity_detected = true;
                }
                break;
            case 3:
                if (last_y < (lv_disp_get_ver_res(NULL) - mouse_cursor_icon.header.h)) {
                    last_y += pos;
                    activity_detected = true;
                }
                break;
            case 4:
                left_button_down = true;
                activity_detected = true;
                break;
            default:
                break;
            }
        }
    }

    const uint32_t now_ms = millis();
    if(activity_detected)
    {
        Serial.printf("[BALL];x;%d;y;%d;btn;%d;steps;%d;ms;%lu\n", (int)last_x, (int)last_y,
                      left_button_down ? 1 : 0, steps_this_read, (unsigned long)now_ms);
        trackball_cursor_visible_until_ms = now_ms + TRACKBALL_CURSOR_SHOW_TIME_MS;
        if(trackball_cursor_obj != NULL)
            lv_obj_clear_flag(trackball_cursor_obj, LV_OBJ_FLAG_HIDDEN);
    }
    else if(trackball_cursor_obj != NULL && now_ms > trackball_cursor_visible_until_ms)
    {
        lv_obj_add_flag(trackball_cursor_obj, LV_OBJ_FLAG_HIDDEN);
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

    // TM-07: cheap SD/LoRa bus-user proxy, sampled once per indev tick.
    // No-op unless --spitrace is armed (tdeck_debug.cpp).
    tdeck_dbg_spitrace_poll();

    // TM-19: drain queued synthetic touch commands (--touch), then let any
    // active injected press/hold override the real GT911 poll below.
    while (s_touch_q_tail != s_touch_q_head)
    {
        // Field-by-field: see tdeck_touch_inject() for why a volatile struct
        // cannot be copy-constructed as a whole.
        uint8_t  op      = s_touch_queue[s_touch_q_tail].op;
        int16_t  cmd_x   = s_touch_queue[s_touch_q_tail].x;
        int16_t  cmd_y   = s_touch_queue[s_touch_q_tail].y;
        uint16_t dur_ms  = s_touch_queue[s_touch_q_tail].dur_ms;
        s_touch_q_tail = (uint8_t)((s_touch_q_tail + 1) % TDECK_TOUCH_QUEUE);
        switch (op)
        {
        case TDECK_TOUCH_TAP:
            s_touch_inj_active = true;
            s_touch_inj_auto_release = true;
            s_touch_inj_x = cmd_x;
            s_touch_inj_y = cmd_y;
            s_touch_inj_release_ms = millis() + dur_ms;
            Serial.printf("[TOUCH];inj;tap;x;%d;y;%d\n", (int)cmd_x, (int)cmd_y);
            break;
        case TDECK_TOUCH_DOWN:
            s_touch_inj_active = true;
            s_touch_inj_auto_release = false;
            s_touch_inj_x = cmd_x;
            s_touch_inj_y = cmd_y;
            Serial.printf("[TOUCH];inj;down;x;%d;y;%d\n", (int)cmd_x, (int)cmd_y);
            break;
        case TDECK_TOUCH_UP:
            s_touch_inj_active = false;
            Serial.printf("[TOUCH];inj;up;x;%d;y;%d\n", (int)s_touch_inj_x, (int)s_touch_inj_y);
            break;
        default:
            break;
        }
    }
    if (s_touch_inj_active && s_touch_inj_auto_release &&
        (int32_t)(millis() - s_touch_inj_release_ms) >= 0)
    {
        s_touch_inj_active = false;              // tap's timed release
    }
    if (s_touch_inj_active)
    {
        data->state = LV_INDEV_STATE_PR;
        data->point.x = s_touch_inj_x;
        data->point.y = s_touch_inj_y;
        return;
    }

    data->state =  LV_INDEV_STATE_REL;
    if (touch.isPressed())
    {
        uint8_t touched = touch.getPoint(x, y, 1);
        if (!meshcom_settings.node_keyboardlock)
        {
            if(current_brightness_level == 0)
                tft_on();
            else
                tdeck_tft_timer = millis();

            if (touched > 0)
            {
                data->state = LV_INDEV_STATE_PR;
                data->point.x = x[0];
                data->point.y = y[0];
            }
        }
    }
}

/**
 * adds initialization messages to message text area
 */
void tdeck_addMessage(bool bSuccess)
{
    char buf[50];

    snprintf(buf, 50, "%s: %s", "Touch", bTouchDected == true ? "OK" : "ERROR");
    addMessage(buf);

    snprintf(buf, 50, "%s: %s", "SDCard", bSDDected == true ? "OK" : "ERROR");
    addMessage(buf);

    snprintf(buf, 50, "%s: %s", "Keyboard", kbDected == true ? "OK" : "ERROR");
    addMessage(buf);

    snprintf(buf, 50, "%s: %s", "Radio", bSuccess == true ? "OK" : "ERROR");
    addMessage(buf);
}

/**
 * clears the message text area
 */
void tdeck_clear_text_ta()
{
    tdeck_reset_msg_tabs();
}

