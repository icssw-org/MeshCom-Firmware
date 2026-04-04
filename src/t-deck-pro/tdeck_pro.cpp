/**
 * @file      test_touchpad.h
 * @author    ShallowGreen123
 * @license   MIT
 * @copyright Copyright (c) 2023  Shenzhen Xin Yuan Electronic Technology Co., Ltd
 * @date      2024-05-27
 *
 */


#include <Arduino.h>
#include "utilities.h"
#include <GxEPD2_BW.h>
#include <TouchDrvCSTXXX.hpp>
#include <TinyGPS++.h>
#include "lvgl.h"
#include "ui_deckpro.h"
#include "ui_deckpro_port.h"
#include <Fonts/FreeMonoBold9pt7b.h>
#include "tdeck_pro.h"
#include "peripheral.h"

#include "loop_functions.h"
#include "loop_functions_extern.h"
#include "lora_setchip.h"
#include <configuration.h>
#include <clock.h>

#include "TinyGPSPlus.h"

// TinyGPS
TinyGPSPlus tinyGPSPlus;

TinyGsm modem(SerialAT);
TaskHandle_t a7682_handle;

XPowersPPM PPM;
BQ27220 bq27220;
Audio audio;

TouchDrvCSTXXX touch;
GxEPD2_BW<GxEPD2_310_GDEQ031T10, GxEPD2_310_GDEQ031T10::HEIGHT> display(GxEPD2_310_GDEQ031T10(BOARD_EPD_CS, BOARD_EPD_DC, BOARD_EPD_RST, BOARD_EPD_BUSY)); // GDEQ031T10 240x320, UC8253, (no inking, backside mark KEGMO 3100)

uint8_t *decodebuffer = NULL;
lv_timer_t *flush_timer = NULL;
int disp_refr_mode = DISP_REFR_MODE_PART;

const char HelloWorld1[] = "   T-Deck-Pro\n\n   MeshCom 4.0\n";
const char HelloWorld2[] = "   @ by icssw.org\n   OE1KBC & OE1KFR";

bool peri_init_st[E_PERI_NUM_MAX] = {0};

String strOldLine = "";


/*********************************************************************************
 *                              STATIC PROTOTYPES
 * *******************************************************************************/
static bool ink_screen_init()
{
    // SPI.begin(BOARD_SPI_SCK, -1, BOARD_SPI_MOSI, BOARD_EPD_CS);
    display.init(115200, true, 2, false);
    //Serial.println("helloWorld");
    display.setRotation(0);
    display.setFont(&FreeMonoBold9pt7b);
    if (display.epd2.WIDTH < 104) display.setFont(0);
    display.setTextColor(GxEPD_BLACK);
    // center bounding box by transposition of origin:
    uint16_t x = 0;
    uint16_t y = 100;
    display.setFullWindow();
    display.firstPage();
    do
    {
        display.fillScreen(GxEPD_WHITE);
        display.setCursor(x, y);
        display.print(HelloWorld1);
        display.print("   FW ");
        display.print(SOURCE_VERSION);
        display.print("/");
        display.print(SOURCE_VERSION_SUB);
        display.print(" <");
        display.print(getCountry(meshcom_settings.node_country).c_str());
        display.print(">\n\n");
        display.print(HelloWorld2);
    }
    while (display.nextPage());
    display.hibernate();

    bTDECKDEBUG = false;

    return true;
}

static void flush_timer_cb(lv_timer_t *t)
{
    static int idx = 0;
    lv_disp_t *disp = lv_disp_get_default();
    if(disp->rendering_in_progress == false)
    {
        lv_coord_t w = LV_HOR_RES;
        lv_coord_t h = LV_VER_RES;

        if(disp_refr_mode == DISP_REFR_MODE_PART) {
            display.setPartialWindow(0, 0, w, h);
        } else if(disp_refr_mode == DISP_REFR_MODE_FULL){
            display.setFullWindow();
        }

        display.firstPage();
        do {
            display.drawInvertedBitmap(0, 0, decodebuffer, w, h - 3, GxEPD_BLACK);
        }
        while (display.nextPage());
        display.hibernate();
        
        if(bTDECKDEBUG)
            Serial.printf("flush_timer_cb:%d, %s\n", idx++, (disp_refr_mode == 0 ?"full":"part"));

        disp_refr_mode = DISP_REFR_MODE_PART;
        lv_timer_pause(flush_timer);
    }
}

static void disp_render_start_cb(struct _lv_disp_drv_t * disp_drv)
{
    if(flush_timer == NULL)
    {
        flush_timer = lv_timer_create(flush_timer_cb, 3000, NULL);
    }
    else
    {
        lv_timer_resume(flush_timer);
    }
}

static void disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
    uint32_t w = (area->x2 - area->x1);
    uint32_t h = (area->y2 - area->y1);

    uint16_t epd_idx = 0;

    union flush_buf_pixel pixel;

    for(int i = 0; i < w * h; i += 8) {
        pixel.bit.b1 = (color_p + i + 7)->full;
        pixel.bit.b2 = (color_p + i + 6)->full;
        pixel.bit.b3 = (color_p + i + 5)->full;
        pixel.bit.b4 = (color_p + i + 4)->full;
        pixel.bit.b5 = (color_p + i + 3)->full;
        pixel.bit.b6 = (color_p + i + 2)->full;
        pixel.bit.b7 = (color_p + i + 1)->full;
        pixel.bit.b8 = (color_p + i + 0)->full;
        decodebuffer[epd_idx] = pixel.full;
        epd_idx++;
    }

    // Serial.printf("x1=%d, y1=%d, x2=%d, y2=%d\n", area->x1, area->y1, area->x2, area->y2);

    /*IMPORTANT!!!
     *Inform the graphics library that you are ready with the flushing*/
    lv_disp_flush_ready(disp_drv);
}

static void touchpad_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data)
{
    if(meshcom_settings.node_keyboardlock)
        return;
        
    static lv_coord_t last_x = 0;
    static lv_coord_t last_y = 0;

    uint8_t touched = touch.getPoint(&last_x, &last_y, 1);
    if(touched) {
        data->state = LV_INDEV_STATE_PR;

        if(bTDECKDEBUG)
            Serial.printf("x = %d, y = %d\n", last_x, last_y);
    }
    else
    {
        data->state = LV_INDEV_STATE_REL;
    }
    /*Set the last pressed coordinates*/
    data->point.x = last_x;
    data->point.y = last_y;
}

static void lvgl_init(void)
{
    lv_init();

    static lv_disp_draw_buf_t draw_buf_dsc_1;
    lv_color_t *buf_1 = (lv_color_t *)ps_calloc(sizeof(lv_color_t), DISP_BUF_SIZE);
    lv_color_t *buf_2 = (lv_color_t *)ps_calloc(sizeof(lv_color_t), DISP_BUF_SIZE);
    lv_disp_draw_buf_init(&draw_buf_dsc_1, buf_1, buf_2, LCD_HOR_SIZE * LCD_VER_SIZE);
    decodebuffer = (uint8_t *)ps_calloc(sizeof(uint8_t), DISP_BUF_SIZE);
    // lv_disp_draw_buf_init(&draw_buf, lv_disp_buf_p, NULL, DISP_BUF_SIZE);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_HOR_SIZE;
    disp_drv.ver_res = LCD_VER_SIZE;
    disp_drv.flush_cb = disp_flush;
    disp_drv.render_start_cb = disp_render_start_cb;
    disp_drv.draw_buf = &draw_buf_dsc_1;
    // disp_drv.rounder_cb = display_driver_rounder_cb;
    disp_drv.full_refresh = 1;

    lv_disp_drv_register(&disp_drv);

    /*------------------
     * Touchpad
     * -----------------*/
    /*Register a touchpad input device*/
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = touchpad_read;
    lv_indev_drv_register(&indev_drv);
}

static bool bq25896_init(void)
{
    // BQ25896 --- 0x6B
    Wire.beginTransmission(BOARD_I2C_ADDR_BQ25896);
    if (Wire.endTransmission() == 0)
    {
        // battery_25896.begin();
        PPM.init(Wire, BOARD_I2C_SDA, BOARD_I2C_SCL, BOARD_I2C_ADDR_BQ25896);
        // Set the minimum operating voltage. Below this voltage, the PPM will protect
        PPM.setSysPowerDownVoltage(3300);

        // Set input current limit, default is 500mA
        PPM.setInputCurrentLimit(3250);

        Serial.printf("getInputCurrentLimit: %d mA\n",PPM.getInputCurrentLimit());

        // Disable current limit pin
        PPM.disableCurrentLimitPin();

        // Set the charging target voltage, Range:3840 ~ 4608mV ,step:16 mV
        PPM.setChargeTargetVoltage(4208);

        // Set the precharge current , Range: 64mA ~ 1024mA ,step:64mA
        PPM.setPrechargeCurr(64);

        // The premise is that Limit Pin is disabled, or it will only follow the maximum charging current set by Limi tPin.
        // Set the charging current , Range:0~5056mA ,step:64mA
        PPM.setChargerConstantCurr(832);

        // Get the set charging current
        PPM.getChargerConstantCurr();
        Serial.printf("getChargerConstantCurr: %d mA\n",PPM.getChargerConstantCurr());

        PPM.enableADCMeasure();

        PPM.enableCharge();

        // Turn off charging function
        // If USB is used as the only power input, it is best to turn off the charging function,
        // otherwise the VSYS power supply will have a sawtooth wave, affecting the discharge output capability.
        // PPM.disableCharge();


        // The OTG function needs to enable OTG, and set the OTG control pin to HIGH
        // After OTG is enabled, if an external power supply is plugged in, OTG will be turned off

        PPM.enableOTG();
        PPM.disableOTG();
        // pinMode(OTG_ENABLE_PIN, OUTPUT);
        // digitalWrite(OTG_ENABLE_PIN, HIGH);

        return true;
    }
    return false;
}

static bool bq27220_init(void)
{
    bool ret = bq27220.init();
    // if(ret) 
    //     bq27220.reset();
    return ret;
}

static bool sd_care_init(void)
{
    if(!SD.begin(BOARD_SD_CS)){
        Serial.println("[SD CARD] Card Mount Failed");
        return false;
    }

    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);

    uint64_t totalSize = SD.totalBytes() / (1024 * 1024);
    Serial.printf("SD Card Total: %lluMB\n", totalSize);

    uint64_t usedSize = SD.usedBytes() / (1024 * 1024);
    Serial.printf("SD Card Used: %lluMB\n", usedSize);
    return true;
}

static void a7682_task(void *param)
{
    vTaskSuspend(a7682_handle);
    while (1)
    {
        while (SerialAT.available())
        {
            SerialMon.write(SerialAT.read());
        }
        while (SerialMon.available())
        {
            SerialAT.write(SerialMon.read());
        }
        delay(1);
    }
}

static bool A7682E_init(void)
{
    /*
    Serial.println("Place your board outside to catch satelite signal");

    // Set module baud rate and UART pins
    SerialAT.begin(115200, SERIAL_8N1, BOARD_A7682E_TXD, BOARD_A7682E_RXD);

    Serial.println("Start modem...");

    // power on
    digitalWrite(BOARD_A7682E_PWRKEY, LOW);
    delay(10);
    digitalWrite(BOARD_A7682E_PWRKEY, HIGH);
    delay(50);
    digitalWrite(BOARD_A7682E_PWRKEY, LOW);
    delay(10);

    int retry_cnt = 5;
    int retry = 0;
    while (!modem.testAT(1000)) {
        Serial.println(".");
        if (retry++ > retry_cnt) {
            digitalWrite(BOARD_A7682E_PWRKEY, LOW);
            delay(100);
            digitalWrite(BOARD_A7682E_PWRKEY, HIGH);
            delay(1000);
            digitalWrite(BOARD_A7682E_PWRKEY, LOW);

            Serial.println("[A7682E] Init Fail");
            break;
        }
    }
    
    Serial.println();
    delay(200);

    xTaskCreate(a7682_task, "a7682_handle", 1024 * 3, NULL, A7682E_PRIORITY, &a7682_handle);

    return (retry < retry_cnt);
    */
    return false;
}

static bool pcm5102a_init(void)
{
    bool ret = audio.setPinout(BOARD_I2S_BCLK, BOARD_I2S_LRC, BOARD_I2S_DOUT);

    if (ret == false) 
        Serial.printf("[%d] Execution error\n", __LINE__);

    audio.setVolume(21); // 0...21

    pinMode(BOARD_6609_EN, OUTPUT);
    digitalWrite(BOARD_6609_EN, HIGH);

    // audio_paly_flag = audio.connecttoFS(SD, "/voice_time/BBIBBI.mp3");

    return true;
}

static void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing spiffs directory: %s\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("- failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println(" - not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.path(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("\tSIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

void initTDeck_pro()
{
    // LORA、SD、EPD use the same SPI, in order to avoid mutual influence;
    // before powering on, all CS signals should be pulled high and in an unselected state;
    pinMode(BOARD_EPD_CS, OUTPUT); 
    digitalWrite(BOARD_EPD_CS, HIGH);
    pinMode(BOARD_SD_CS, OUTPUT); 
    digitalWrite(BOARD_SD_CS, HIGH);
    pinMode(BOARD_LORA_CS, OUTPUT); 
    digitalWrite(BOARD_LORA_CS, HIGH);

    Serial.begin(115200);

    // IO
    pinMode(BOARD_KEYBOARD_LED, OUTPUT);
    pinMode(BOARD_MOTOR_PIN, OUTPUT);
    pinMode(BOARD_6609_EN, OUTPUT);         // enable 7682 module
    pinMode(BOARD_LORA_EN, OUTPUT);         // enable LORA module
    pinMode(BOARD_GPS_EN, OUTPUT);          // enable GPS module
    pinMode(BOARD_1V8_EN, OUTPUT);          // enable gyroscope module
    pinMode(BOARD_A7682E_PWRKEY, OUTPUT); 
    digitalWrite(BOARD_KEYBOARD_LED, LOW);
    digitalWrite(BOARD_MOTOR_PIN, LOW);
    digitalWrite(BOARD_6609_EN, HIGH);
    digitalWrite(BOARD_LORA_EN, HIGH);
    digitalWrite(BOARD_GPS_EN, HIGH);
    digitalWrite(BOARD_1V8_EN, LOW);    // KBC GYRO OFF
    digitalWrite(BOARD_A7682E_PWRKEY, LOW); // KBC A7682E OFF

    // i2c devices
    byte error, address;
    int nDevices = 0;
    Wire.begin(BOARD_I2C_SDA, BOARD_I2C_SCL);
    Serial.printf("------------- I2C ------------- \n");
    for(address = 0x01; address < 0x7F; address++){
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if(error == 0){ // 0: success.
            nDevices++;
            if(address == BOARD_I2C_ADDR_TOUCH){
                // flag_Touch_init = true;
                Serial.printf("[0x%x] TOUCH find!\n", address);
            } else if (address == BOARD_I2C_ADDR_LTR_553ALS) {
                Serial.printf("[0x%x] LTR_553ALS find!\n", address);
            } else if (address == BOARD_I2C_ADDR_GYROSCOPDE) {
                Serial.printf("[0x%x] GYROSCOPDE find!\n", address);
            } else if (address == BOARD_I2C_ADDR_KEYBOARD) {
                Serial.printf("[0x%x] KEYBOARD find!\n", address);
            } else if (address == BOARD_I2C_ADDR_BQ27220) {
                Serial.printf("[0x%x] BQ27220 find!\n", address);
            } else if (address == BOARD_I2C_ADDR_BQ25896) {
                Serial.printf("[0x%x] BQ25896 find!\n", address);
            }
        }
    }

    Serial.printf("------------- SPIFFS ------------- \n");

    if(!SPIFFS.begin(true)){
        Serial.println("SPIFFS Mount Failed");
        return;
    }

    listDir(SPIFFS, "/", 0);

    Serial.println("------------- PERI ------------- ");

    // SPI
    SPI.begin(BOARD_SPI_SCK, BOARD_SPI_MISO, BOARD_SPI_MOSI);

    // init peripheral
    touch.setPins(BOARD_TOUCH_RST, BOARD_TOUCH_INT);
    peri_init_st[E_PERI_INK_SCREEN] = ink_screen_init();
    peri_init_st[E_PERI_LORA]       = lora_init();
    peri_init_st[E_PERI_TOUCH]      = touch.begin(Wire, BOARD_I2C_ADDR_TOUCH, BOARD_TOUCH_SDA, BOARD_TOUCH_SCL);
    peri_init_st[E_PERI_KYEPAD]     = keypad_init(BOARD_I2C_ADDR_KEYBOARD);
    peri_init_st[E_PERI_BQ25896]    = bq25896_init();
    peri_init_st[E_PERI_BQ27220]    = bq27220_init();
    peri_init_st[E_PERI_SD]         = sd_care_init();
    peri_init_st[E_PERI_GPS]        = gps_init();
    peri_init_st[E_PERI_BHI260AP]   = BHI260AP_init();
    peri_init_st[E_PERI_LTR_553ALS] = LTR553_init();
    peri_init_st[E_PERI_A7682E]     = A7682E_init();

    /*
    if(peri_init_st[E_PERI_A7682E] == false)
    {
        peri_init_st[E_PERI_PCM5102A] = pcm5102a_init();
    }
    */

    ui_setting_set_a7682_status(false);
    ui_setting_set_gyro_status(false);

    lvgl_init();

    ui_deckpro_entry();

    disp_full_refr();
}

void loopTDeck_pro()
{
    lv_task_handler();
    
    keypad_loop();

    /*
    if(peri_init_st[E_PERI_PCM5102A] == true) 
    {
        audio.loop();
    }
    */
}

/*********************************************************************************
 *                              GLOBAL PROTOTYPES
 * *******************************************************************************/
void disp_full_refr(void)
{
    disp_refr_mode = DISP_REFR_MODE_FULL;
}

void TDeck_pro_lora_disp(String strHead, String strText)
{
    //Serial.printf("TDeck_pro_lora_disp <%s><%s>\n", strHead.c_str(), strText.c_str());

    if(strOldLine.length() > 0)
    {
        strOldLine = "----- " + getDateString().substring(5, 11) + " " + getTimeString() + " -----\n" + strHead + "\n" + strText + "\n" + strOldLine;
    }
    else
    {
        strOldLine = "----- " + getDateString().substring(5, 11) + " " + getTimeString() + " -----\n" + strHead + "\n" + strText;
    }

    if(strOldLine.length() > 600)
        strOldLine = strOldLine.substring(0, 400);

    ui_lora_disp(strOldLine);

    // make noise
    if(!meshcom_settings.node_mute)
    {
        digitalWrite(BOARD_MOTOR_PIN, HIGH);
        delay(100);
        digitalWrite(BOARD_MOTOR_PIN, LOW);
    }

}

void TDeck_pro_mheard_disp()
{
    ui_mheard_disp();
}

void tdeck_refresh_track_view()
{
    ui_track_disp(false);
}

void tdeck_send_track_view()
{
    ui_track_disp(true);
}

void tdeck_set_gps(bool bGPS)
{
    if(bGPS)
        ui_gps_task_resume();
    else
        ui_gps_task_suspend();
}

unsigned int tdeck_get_gps()
{
    double lat      = 0; // Latitude
    double lon      = 0; // Longitude
    double speed    = 0; // Speed over ground
    double alt      = 0; // Altitude
    int hdop        = 0; // hdop
    uint32_t   vsat     = 0; // Visible Satellites
    uint16_t   year     = 0; // 
    uint8_t   month    = 0; // 
    uint8_t   day      = 0; // 
    uint8_t   hour     = 0; // 
    uint8_t   min      = 0; // 
    uint8_t   sec      = 0; // 
    uint8_t   fix      = 0; //

    ui_gps_get_coord(&lat, &lon, &alt);
    ui_gps_get_data(&year, &month, &day);
    ui_gps_get_time(&hour, &min, &sec);
    ui_gps_get_satellites(&vsat, &hdop);
    ui_gps_get_speed(&speed);
    ui_gps_get_fix(&fix);

    if(year > 2023)
    {
        MyClock.setCurrentTime(meshcom_settings.node_utcoff, year, month, day, (uint16_t)hour, (uint16_t)min, (uint16_t)sec);
        snprintf(cTimeSource, sizeof(cTimeSource), (char*)"GPS");
    }
        
    meshcom_settings.node_date_year = MyClock.Year();
    meshcom_settings.node_date_month = MyClock.Month();
    meshcom_settings.node_date_day = MyClock.Day();

    meshcom_settings.node_date_hour = MyClock.Hour();
    meshcom_settings.node_date_minute = MyClock.Minute();
    meshcom_settings.node_date_second = MyClock.Second();

    if(fix == 0)
    {
        posinfo_fix = false;

        posinfo_satcount = 0;
        posinfo_hdop = 0;
        posinfo_direction = 0;
        posinfo_distance = 0;
        posinfo_age = 0;

        if(bGPSDEBUG)
            Serial.println(F("INVALID"));

        return POSINFO_INTERVAL;
    }
    else
    {
        posinfo_fix = true;
        double dlat, dlon;

        dlat = cround4abs(lat);
        dlon = cround4abs(lon);

        if(lat < 0)
        {
            meshcom_settings.node_lat = lat * (-1);
            meshcom_settings.node_lat_c = 'S';
        }

        if(lat > 0)
        {
            meshcom_settings.node_lat = lat;
            meshcom_settings.node_lat_c = 'N';
        }
        
        if(lon < 0)
        {
            meshcom_settings.node_lon = lon * (-1);
            meshcom_settings.node_lon_c = 'W';
        }

        if(lon > 0)
        {
            meshcom_settings.node_lon = lon;
            meshcom_settings.node_lon_c = 'E';
        }

        if(alt > 0)
            meshcom_settings.node_alt = (int)(alt + 0.5);

        posinfo_satcount = vsat;
        posinfo_hdop = hdop;
        

        posinfo_age = 0;
    
        if(bGPSDEBUG)
            Serial.println(F("VALID"));

        return setSMartBeaconing(dlat, dlon);
    }
}