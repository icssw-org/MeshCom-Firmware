// (C) 2023 OE1KBC Kurt Baumann, OE1KFR Rainer 
//
// This code is not for distribution and may not be published!
// Vervielfältigung und Veröffentlichung des Codes sind nicht gestattet!
// Lizenz: kein Open Source.
// zugelassen ausschließlich für OE1KBC, OE1KFR
// alle anderen Finger weg !
// 20230326: Version 4.00: START

#include <Arduino.h>
#include <SPI.h>
#include <SX126x-RAK4630.h>
#include <debugconf.h>
#include <time.h>

#include <TinyGPSplus.h>

#include "Adafruit_SHTC3.h"

#include "Adafruit_LPS2X.h"

// MeshCom Common (ers32/nrf52) Funktions
#include <loop_functions.h>
#include <loop_functions_extern.h>
#include <command_functions.h>
#include <aprs_functions.h>
#include <batt_functions.h>
#include <lora_functions.h>
#include <phone_commands.h>
#include <mheard_functions.h>
#include <clock.h>

#include <bmx280.h>

#include <SparkFun_Ublox_Arduino_Library.h>
SFE_UBLOX_GPS myGPS;

// UBLOX Modul auf Factory-Set zurück setzen
// derzeit nur im esp32_gps realisiert
// nrf52 fehlt diese Funktion noch
// Diese Variablen werden durch das Command --gps reset gesetzt und solten im GPS Modul abgehandelt werden
int state = 0; // only for gps reset
bool bMitHardReset = false;  // only for gps reset

/*
    RAK4631 PIN DEFINITIONS

    static const uint8_t WB_IO1 = 17;	   // SLOT_A SLOT_B
    static const uint8_t WB_IO2 = 34;	   // SLOT_A SLOT_B
    static const uint8_t WB_IO3 = 21;	   // SLOT_C
    static const uint8_t WB_IO4 = 4;	   // SLOT_C
    static const uint8_t WB_IO5 = 9;	   // SLOT_D
    static const uint8_t WB_IO6 = 10;	   // SLOT_D
    static const uint8_t WB_SW1 = 33;	   // IO_SLOT
    static const uint8_t WB_A0 = 5;		   // IO_SLOT
    static const uint8_t WB_A1 = 31;	   // IO_SLOT
    static const uint8_t WB_I2C1_SDA = 13; // SENSOR_SLOT IO_SLOT
    static const uint8_t WB_I2C1_SCL = 14; // SENSOR_SLOT IO_SLOT
    static const uint8_t WB_I2C2_SDA = 24; // IO_SLOT
    static const uint8_t WB_I2C2_SCL = 25; // IO_SLOT
    static const uint8_t WB_SPI_CS = 26;   // IO_SLOT
    static const uint8_t WB_SPI_CLK = 3;   // IO_SLOT
    static const uint8_t WB_SPI_MISO = 29; // IO_SLOT
    static const uint8_t WB_SPI_MOSI = 30; // IO_SLOT

    SPI
    static const uint8_t SS = 26;
    static const uint8_t MOSI = PIN_SPI_MOSI;
    static const uint8_t MISO = PIN_SPI_MISO;
    static const uint8_t SCK = PIN_SPI_SCK;

    // QSPI Pins
    #define PIN_QSPI_SCK 3	// 19
    #define PIN_QSPI_CS 26	// 17
    #define PIN_QSPI_IO0 30 // 20
    #define PIN_QSPI_IO1 29 // 21
    #define PIN_QSPI_IO2 28 // 22
    #define PIN_QSPI_IO3 2	// 23

    @note RAK5005-O GPIO mapping to RAK4631 GPIO ports
   RAK5005-O <->  nRF52840
   IO1       <->  P0.17 (Arduino GPIO number 17)
   IO2       <->  P1.02 (Arduino GPIO number 34)
   IO3       <->  P0.21 (Arduino GPIO number 21)
   IO4       <->  P0.04 (Arduino GPIO number 4)
   IO5       <->  P0.09 (Arduino GPIO number 9)
   IO6       <->  P0.10 (Arduino GPIO number 10)
   SW1       <->  P0.01 (Arduino GPIO number 1)
   A0        <->  P0.04/AIN2 (Arduino Analog A2
   A1        <->  P0.31/AIN7 (Arduino Analog A7
   SPI_CS    <->  P0.26 (Arduino GPIO number 26)

    */

/*
Sync Word Setting in MeshCom
    MeshCom Syc Word is 0x2b

    Output of the LoRa Sync Word Register 0x0740 in MeshCom:

    Set radio: power level=22
    Sync Word 1st byte = 24
    Sync Word 2nd byte = b4

    In our Library it gets set at sx126x.h / radio.cpp
    Define: sx126x.h line 109:
    #define LORA_MAC_PUBLIC_SYNCWORD 0x242b

    Radio.SetPublicNetwork(true); needs to be called, so syncword gets new set in radio.cpp line: 1183 in
    void RadioSetPublicNetwork(bool enable)
    Method
*/

//NVIC_SystemReset(); resets the device

asm(".global _scanf_float");
asm(".global _printf_float");

// LoRa Events and Buffers
static RadioEvents_t RadioEvents;

bool g_meshcom_initialized;
bool init_flash_done=false;

bool bPosFirst = true;

/**
 * BLE Spec
 * Messages to and from the phone need to have flag if it is a Text, Pos Msg or it is a configuration
 * Text/Pos flag: 0x40
 * Config Flag: 0x80
 * 
 * Config Message Parameters: Callsign, Lat, Lon, altitude
 * Text/Pos Msg Format:
 * LENGTH 2B - FLAG 1B - MSG xB
 * Config Format:
 * LENGTH 2B - FLAG 1B - LENCALL 1B - Callsign - LAT 4B(Float) - LON 4B(Float) - ALT 4B(INT)
 * 
 * Hello Message from Phone is: "XXaaYYzz"
 * Config of this device is sent to phone after hello msg from phone
*/

// Textmessage buffer from phone, hasMsgFromPhone flag indicates new message
extern char textbuff_phone [MAX_MSG_LEN_PHONE];
extern uint8_t txt_msg_len_phone;

extern bool ble_busy_flag;    // flag to signal bluetooth uart is active

//variables and helper functions
uint8_t err_cnt_udp_tx = 0;    // counter on errors sending message via UDP

String strText="";

// TinyGPS
TinyGPSPlus tinyGPSPlus;

int direction_S_N = 0;  //0--S, 1--N
int direction_E_W = 0;  //0--E, 1--W

unsigned int getGPS(void);

// TEMP/HUM
Adafruit_SHTC3 shtc3 = Adafruit_SHTC3();

void getTEMP(void);

Adafruit_LPS22 g_lps22hb;

void getPRESSURE(void);

#define POWER_ENABLE   WB_IO2

#define LEFT_BUTTON    WB_IO3
#define MIDDLE_BUTTON  WB_IO5
#define RIGHT_BUTTON   WB_IO6

uint8_t gKeyNum = 0; // which button is pressed
int iGPSCount=0;

// left button interrupt handle function
void interruptHandle1()
{
  if(gKeyNum == 0)
  {
    gKeyNum = 1;
  }
}

// middle button interrupt handle function
void interruptHandle2()
{
  if(gKeyNum == 0)
  {
    gKeyNum = 2;
  }
}

// right button interrupt handle function
void interruptHandle3()
{
  if(gKeyNum == 0)
  {
    gKeyNum = 3;
  }
}

extern U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2;

// Prototypes
void blinkLED();                                     // blink GREEN
void blinkLED2();                                    // blink BLUE

void checkSerialCommand(void);


// Client basic variables
uint8_t dmac[6];

/** Set the device name, max length is 10 characters */
    char g_ble_dev_name[10] = "CL";

void getMacAddr(uint8_t *dmac)
{
    const uint8_t *src = (const uint8_t *)NRF_FICR->DEVICEADDR;
    dmac[5] = src[0];
    dmac[4] = src[1];
    dmac[3] = src[2];
    dmac[2] = src[3];
    dmac[1] = src[4];
    dmac[0] = src[5]; // | 0xc0; // MSB high two bits get set elsewhere in the bluetooth stack
}

void nrf52setup()
{
#if defined NRF52_SERIES || defined ESP32
	// Create the task event semaphore
	g_task_sem = xSemaphoreCreateBinary();
	// Initialize semaphore
	xSemaphoreGive(g_task_sem);
#endif

     // LEDs
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);

    // clear the buffers
    for (int i = 0; i < uint8_t(sizeof(RcvBuffer)); i++)
    {
        RcvBuffer[i] = 0x00;
    }

    for(int ib=0; ib<MAX_RING; ib++)
    {
        memset(own_msg_id[ib], 0x00, 5);
    }

    //clear ringbuffer
    for(int i=0; i<MAX_RING; i++)
    {
        memset(ringBufferLoraRX[i], 0, 4);
    }

    // Initialize mheard list
    initMheard();

	// Initialize battery reading
	init_batt();

	// Get LoRa parameter
	init_flash();

    meshcom_settings.node_date_hour = 0;
    meshcom_settings.node_date_minute = 0;
    meshcom_settings.node_date_second = 0;
    meshcom_settings.node_date_hundredths = 0;

    meshcom_settings.node_date_year = 0;
    meshcom_settings.node_date_month = 0;
    meshcom_settings.node_date_day = 0;

    bDisplayVolt = meshcom_settings.node_sset & 0x0001;
    bDisplayOff = meshcom_settings.node_sset & 0x0002;
    bPosDisplay = meshcom_settings.node_sset & 0x0004;
    bDEBUG = meshcom_settings.node_sset & 0x0008;
    bButtonCheck = meshcom_settings.node_sset & 0x0010;
    bDisplayTrack = meshcom_settings.node_sset & 0x0020;
    bGPSON =  meshcom_settings.node_sset & 0x0040;
    bBMPON =  meshcom_settings.node_sset & 0x0080;
    bBMEON =  meshcom_settings.node_sset & 0x0100;
    bSHORTPATH = meshcom_settings.node_sset & 0x0400;
    bLORADEBUG = meshcom_settings.node_sset & 0x0200;
    bGATEWAY =  meshcom_settings.node_sset & 0x1000;
    bEXTUDP =  meshcom_settings.node_sset & 0x2000;
    bEXTSER =  meshcom_settings.node_sset & 0x4000;

    global_batt = 4200.0;

    posinfo_interval = POSINFO_INTERVAL;

    if(meshcom_settings.node_maxv > 0)
    {
        setMaxBatt(meshcom_settings.node_maxv * 1000.0F);
    
        global_batt = meshcom_settings.node_maxv * 1000.0F;
    }

    //  Initialize the LoRa Module
    lora_rak4630_init();

    getMacAddr(dmac);

    _GW_ID = dmac[0] | (dmac[1] << 8) | (dmac[2] << 16) | (dmac[3] << 24);

    // BLE ID
    meshcom_settings.node_device_eui[0]=0x4D;   // M
    meshcom_settings.node_device_eui[1]=0x43;   // C
    for(int ieui=0; ieui<6; ieui++)
    {
        meshcom_settings.node_device_eui[ieui+2] = dmac[ieui];
    }


    //  Initialize the Serial Port for debug output
    time_t timeout = millis();
    Serial.begin(MONITOR_SPEED);
    while (!Serial)
    {
        if ((millis() - timeout) < 2000)
        {
            delay(100);
        }
        else
        {
            break;
        }
    }


    ////////////////////////////////////////////////////////////////////
    // Initialize time
	bool boResult;
	
	// initialize clock
	boResult = MyClock.Init();
	Serial.printf("Initialize clock: %s\n", (boResult) ? "ok" : "FAILED");

    DisplayTimeWait=0;
    //
    ////////////////////////////////////////////////////////////////////

    //gps init
    pinMode(WB_IO2, OUTPUT);
    digitalWrite(WB_IO2, 0);
    delay(1000);
    digitalWrite(WB_IO2, 1);
    delay(1000);
    
    Serial.println("=====================================");

    Serial.println("GPS: trying 38400 baud");
    
    Serial1.begin(38400);
    Serial1.setTimeout(500);
    while (!Serial1);

    if(Serial1)
    {
        if (myGPS.begin(Serial1))
        {
            Serial.println("GPS: connected at 38400 baud");
        }
        else
        {
            Serial1.end();

            delay(100);
            Serial.println("GPS: trying 9600 baud");

            Serial1.begin(9600);
            Serial1.setTimeout(500);
            while (!Serial1);

            if(Serial1)
            {
                if (myGPS.begin(Serial1))
                {
                    Serial.println("GPS: connected at 9600 baud");
                }
                else
                {
                    Serial.println("GPS: speed not found");
                }
            }
            else
                Serial.println("GPS: not connected");
        }
    }
    else
        Serial.println("GPS: not connected");

    delay(100);

    #if defined(ENABLE_BMX280)
        setupBMX280();
    #endif

    // Try to initialize!
    #if defined(LPS33)
    
    Serial.println("Adafruit LPS33 check");

    if (!g_lps22hb.begin_I2C(0x5d)) 
    {
        Serial.println("Failed to find LPS33 chip");
        //while (1) 
        { 
          //  delay(10); 
        }
    }
    else
    {
        Serial.println("LPS33 sensor found");

        g_lps22hb.setDataRate(LPS22_RATE_10_HZ);

        /*
        Serial.print("Data rate set to: ");

        switch (g_lps22hb.getDataRate()) 
        {
            case LPS22_RATE_ONE_SHOT: Serial.println("One Shot / Power Down"); 
                break;
            case LPS22_RATE_1_HZ: Serial.println("1 Hz"); 
                break;
            case LPS22_RATE_10_HZ: Serial.println("10 Hz"); 
                break;
            case LPS22_RATE_25_HZ: Serial.println("25 Hz"); 
                break;
            case LPS22_RATE_50_HZ: Serial.println("50 Hz"); 
                break;
        }
        */
    }
    #endif // LPS33

    #if defined(SHT3)

    Serial.println("Adafruit SHTC3 check");
    if (!shtc3.begin()) {
        Serial.println("Couldn't find SHTC3");
        while (1) delay(1);
    }
    Serial.println("SHTC3 sensor found");

    #endif // SHT3

    //////////////////////////////////////////////////////
    // BLE INIT

    g_enable_ble=true;

#if defined NRF52_SERIES || defined ESP32
	if (g_enable_ble)
	{
		// Init BLE
		init_ble();

        Serial.println("Adafruit BLE init");
	}
	else
	{
		// BLE is not activated, switch off blue LED
		digitalWrite(LED_BLUE, LOW);
	}

	// Take the semaphore so the loop will go to sleep until an event happens
	xSemaphoreTake(g_task_sem, 10);
#endif
#ifdef ARDUINO_ARCH_RP2040
	// RAK11310 does not have BLE, switch off blue LED
	digitalWrite(LED_BLUE, LOW);
#endif


    Wire.begin();
    u8g2.begin();

    u8g2.clearDisplay();
    u8g2.setFont(u8g2_font_6x10_mf);
    u8g2.firstPage();
    do
    {
        u8g2.setFont(u8g2_font_10x20_mf);
        u8g2.drawStr(5, 20, "MeshCom 4.0");
        u8g2.setFont(u8g2_font_6x10_mf);
        char cvers[10];
        sprintf(cvers, "FW %s%s", SOURCE_TYPE, SOURCE_VERSION);
        u8g2.drawStr(5, 30, cvers);
        u8g2.drawStr(5, 40, "by icssw.org");
        u8g2.drawStr(5, 50, "OE1KFR, OE1KBC");
        u8g2.drawStr(5, 60, "...starting now");
    } while (u8g2.nextPage());

    // reset GPS-Time parameter
    meshcom_settings.node_date_hour = 0;
    meshcom_settings.node_date_minute = 0;
    meshcom_settings.node_date_second = 0;
    meshcom_settings.node_date_hundredths = 0;

    Serial.println("CLIENT STARTED");

    //  Set the LoRa Callback Functions
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;
    //RadioEvents.PreAmpDetect = OnPreambleDetect;
    RadioEvents.HeaderDetect = OnHeaderDetect;
    

    //  Initialize the LoRa Transceiver
    Radio.Init(&RadioEvents);

    // Sets the Syncword new that we can set the MESHCOM SWORD
    DEBUG_MSG("RADIO", "Setting new LoRa Sync Word");
    Radio.SetPublicNetwork(true);

    //  Set the LoRa Frequency
    Radio.SetChannel(RF_FREQUENCY);

    //  Configure the LoRa Transceiver for receiving messages
    Radio.SetRxConfig(
        MODEM_LORA,
        LORA_BANDWIDTH,
        LORA_SPREADING_FACTOR,
        LORA_CODINGRATE,
        0, //  AFC bandwidth: Unused with LoRa
        LORA_PREAMBLE_LENGTH,
        LORA_SYMBOL_TIMEOUT,
        LORA_FIX_LENGTH_PAYLOAD_ON,
        0,    //  Fixed payload length: N/A
        true, //  CRC enabled
        0,    //  Frequency hopping disabled
        0,    //  Hop period: N/A
        LORA_IQ_INVERSION_ON,
        true //  Continuous receive mode
    );

    // Set Radio TX configuration
    Radio.SetTxConfig(
        MODEM_LORA,
        TX_OUTPUT_POWER,
        0, // fsk only
        LORA_BANDWIDTH,
        LORA_SPREADING_FACTOR,
        LORA_CODINGRATE,
        LORA_PREAMBLE_LENGTH,
        LORA_FIX_LENGTH_PAYLOAD_ON,
        true, // CRC ON
        0,    // fsk only frequ hop
        0,    // fsk only frequ hop period
        LORA_IQ_INVERSION_ON,
        TX_TIMEOUT_VALUE);

    //  Start receiving LoRa packets
    DEBUG_MSG("RADIO", "Starting RX MODE");
    Radio.Rx(RX_TIMEOUT_VALUE);

    // set left button interrupt
    //pinMode(LEFT_BUTTON, INPUT);
    //attachInterrupt(LEFT_BUTTON, interruptHandle1, FALLING);

    // set middle button interrupt
    pinMode(MIDDLE_BUTTON, INPUT);
    attachInterrupt(MIDDLE_BUTTON, interruptHandle2, FALLING);

    // set right button interrupt
    //pinMode(RIGHT_BUTTON, INPUT);
    //attachInterrupt(RIGHT_BUTTON, interruptHandle3, FALLING);

    sd_power_mode_set(NRF_POWER_MODE_CONSTLAT);

    Serial.println("=====================================");

    delay(100);
}

void nrf52loop()
{
	//Clock::EEvent eEvent;
	
	// check clock event
	//eEvent = MyClock.CheckEvent();

	MyClock.CheckEvent();
	
    meshcom_settings.node_date_year = MyClock.Year();
    meshcom_settings.node_date_month = MyClock.Month();
    meshcom_settings.node_date_day = MyClock.Day();

    meshcom_settings.node_date_hour = MyClock.Hour();
    meshcom_settings.node_date_minute = MyClock.Minute();
    meshcom_settings.node_date_second = MyClock.Second();

   	//digitalWrite(LED_GREEN, LOW);
   	//digitalWrite(LED_BLUE, LOW);

    // check if we have messages in ringbuffer to send
    //Serial.printf("is_receiving:%i tx_is_active:%i iWrite:%i iRead:%i \n", is_receiving, tx_is_active, iWrite, iRead);
    
    if(is_receiving == false && tx_is_active == false)
    {
        // channel is free
        // nothing was detected
        // do not print anything, it just spams the console
        if (iWrite != iRead)
        {
            // save transmission state between loops
            doTX();
        }
    }

    // check if message from phone to send
    if(hasMsgFromPhone)
    {
        sendMessage(textbuff_phone, txt_msg_len_phone);

        hasMsgFromPhone = false;
    }

    if(gKeyNum == 1)
    {
        //getTEMP();

        //getPRESSURE();

        gKeyNum = 0;
    }

    if(gKeyNum == 2)
    {
        // GPS FIX
        iGPSCount++;

        if(iGPSCount > 10)
        {
            unsigned int igps = getGPS();

            if(bDEBUG)
            {
                Serial.printf("\nGPS: <posinterval:%i> <direction:%i> LAT:%lf LON:%lf %02d-%02d-%02d %02d:%02d:%02d\n", igps, posinfo_direction, tinyGPSPlus.location.lat(), tinyGPSPlus.location.lng(), tinyGPSPlus.date.year(), tinyGPSPlus.date.month(), tinyGPSPlus.date.day(), tinyGPSPlus.time.hour(), tinyGPSPlus.time.minute(), tinyGPSPlus.time.second());
                //Serial.printf("INT: LAT:%lf LON:%lf %i-%02i-%02i %02i:%02i:%02i\n", meshcom_settings.node_lat, meshcom_settings.node_lon, meshcom_settings.node_date_year, meshcom_settings.node_date_month,  meshcom_settings.node_date_day,
                //meshcom_settings.node_date_hour, meshcom_settings.node_date_minute, meshcom_settings.node_date_second );
            }

            if(igps > 0)
                posinfo_interval = igps;
            else
            {
                no_gps_reset_counter++;
                if(no_gps_reset_counter > 10)
                {
                    posinfo_interval = POSINFO_INTERVAL;
                    no_gps_reset_counter = 0;
                    posinfo_fix = false;
                    posinfo_satcount = 0;
                    posinfo_hdop = 0;
                }
            }

            iGPSCount=0;
        }

        gKeyNum = 0;
    }

    if(gKeyNum == 3)
    {
        Serial.println("Right button pressed");

        gKeyNum = 0;
    }

    // check if we have messages for BLE to send
    if (isPhoneReady == 1 && (toPhoneWrite != toPhoneRead))
    {
        sendToPhone();   
    }

    // posinfo
    //Serial.printf("posinfo_timer:%ld posinfo_interval:%ld timer:%ld millis:%ld\n", posinfo_timer, posinfo_interval, (posinfo_timer + (posinfo_interval * 1000)), millis());

    if (((posinfo_timer + (posinfo_interval * 1000)) < millis()) || (millis() > 10000 && millis() < 30000 && bPosFirst) || posinfo_shot)
    {
        bPosFirst = false;
        posinfo_shot=false;
        
        sendPosition(posinfo_interval, meshcom_settings.node_lat, meshcom_settings.node_lat_c, meshcom_settings.node_lon, meshcom_settings.node_lon_c, meshcom_settings.node_alt, meshcom_settings.node_press, meshcom_settings.node_hum, meshcom_settings.node_temp, meshcom_settings.node_press_alt, meshcom_settings.node_press_asl);

        posinfo_last_lat=posinfo_lat;
        posinfo_last_lon=posinfo_lon;
        posinfo_last_direction=posinfo_direction;


        #if defined(LPS33)
            sendWeather(meshcom_settings.node_lat, meshcom_settings.node_lat_c, meshcom_settings.node_lon, meshcom_settings.node_lon_c, meshcom_settings.node_alt,
             meshcom_settings.node_temp, meshcom_settings.node_hum, meshcom_settings.node_press);
        #endif

        posinfo_timer = millis();
    }

    #if defined(LPS33)

    // TEMP/HUM
    if (((temphum_timer + TEMPHUM_INTERVAL) < millis()))
    {
        getTEMP();

        temphum_timer = millis();
    }

    #endif

    #if defined(SHT3)

    // DRUCK
    if (((druck_timer + DRUCK_INTERVAL) < millis()))
    {
        getPRESSURE();

        druck_timer = millis();
    }

    #endif

    mainStartTimeLoop();

    if(DisplayOffWait > 0)
    {
        if (millis() > DisplayOffWait)
        {
            DisplayOffWait = 0;
            bDisplayOff=true;
        }
    }
    
    // rebootAuto
    if(rebootAuto > 0)
    {
        if (millis() > rebootAuto)
        {
            rebootAuto = 0;

            #ifdef ESP32
                ESP.restart();
            #endif
        }
    }

    
    checkButtonState();

    checkSerialCommand();

    if(BattTimeWait == 0)
        BattTimeWait = millis() - 31000;

    if ((BattTimeWait + 30000) < millis())
    {
        if (tx_is_active == false && is_receiving == false)
        {
            global_batt = read_batt();

            BattTimeWait = millis();
        }
    }

    if (isPhoneReady == 1)
    {
        if(BattTimeAPP == 0)
            BattTimeAPP = millis() - 180000;


        if ((BattTimeAPP + 180000) < millis())  // 60*3 sec
        {
            char cbatt[15];
            sprintf(cbatt, "--BAT %4.2f %03i", global_batt/1000.0, mv_to_percent(global_batt));

            addBLECommandBack(cbatt);
            
            BattTimeAPP = millis();
        }
    }
    else
        BattTimeAPP=0;

    if(BMXTimeWait == 0)
        BMXTimeWait = millis() - 10000;

    if ((BMXTimeWait + 30000) < millis())   // 30 sec
    {
        // read BMX Sensor
        #if defined(ENABLE_BMX280)
            if(loopBMX280())
            {
                meshcom_settings.node_temp = getTemp();
                meshcom_settings.node_hum = getHum();  //BMP280 - not supported
                meshcom_settings.node_press = getPress();

                BMXTimeWait = millis();
            }

        #endif
    }

    //  We are on FreeRTOS, give other tasks a chance to run
    delay(100);
    
    yield();
}

/** @brief Function to check if the modem detected a preamble
 */
void blinkLED()
{
    digitalWrite(LED_GREEN, HIGH);
    delay(5);
    digitalWrite(LED_GREEN, LOW);
}

void blinkLED2()
{
    digitalWrite(LED_BLUE, HIGH);
    delay(20);
    digitalWrite(LED_BLUE, LOW);
}


/**@brief Function for handling a LoRa tx timer timeout event.
 */
void getTEMP(void)
{
    sensors_event_t humidity, temp;

    shtc3.getEvent(&humidity, &temp); // populate temp and humidity objects with fresh data

    /*
    Serial.print("Temperature: "); Serial.print(temp.temperature); Serial.println(" degrees C");
    Serial.print("Humidity: "); Serial.print(humidity.relative_humidity); Serial.println("% rH");
    */
    meshcom_settings.node_temp = temp.temperature;
    meshcom_settings.node_hum = humidity.relative_humidity;
}

/**@brief Function for handling a LoRa tx timer timeout event.
 */
void getPRESSURE(void)
{
    sensors_event_t temp;
    sensors_event_t pressure;
    
    g_lps22hb.getEvent(&pressure, &temp);

    /*
    Serial.print("Temperature: ");Serial.print(temp.temperature);Serial.println(" degrees C");
    Serial.print("Pressure: ");Serial.print(pressure.pressure);Serial.println(" hPa");
    Serial.println("");
    */
    
    double home_alt=meshcom_settings.node_alt;    // Höhe des Standorts
    double temperature_gradient = 0.0065;           // Standard-Temperaturgradient
    double temperatureK = temp.temperature + 273.15;     // Temperatur in Kelvin
    
    // barometrische Höhenformel
    meshcom_settings.node_press = pressure.pressure * pow((temperatureK / (temperatureK + home_alt * temperature_gradient)) , -5.255);
}

/**@brief Function for analytical direction.
 */
void direction_parse(String tmp)
{
    if (tmp.indexOf(",E,") != -1)
    {
        direction_E_W = 0;
    }
    else
    {
        direction_E_W = 1;
    }
    
    if (tmp.indexOf(",S,") != -1)
    {
        direction_S_N = 0;
    }
    else
    {
        direction_S_N = 1;
    }
}

/**@brief Function for handling a LoRa tx timer timeout event.
 */
unsigned int getGPS(void)
{ 
    if(bDEBUG)
        Serial.println("-----------check GPS-----------");

    String tmp_data = "";

    bool newData = false;
  
    // For one second we parse GPS data and report some key values
    for (unsigned long start = millis(); millis() - start < 1000;)
    {
      while (Serial1.available())
      {
        char c = Serial1.read();
        
        if(bDEBUG)
            Serial.write(c);

        tmp_data += c;

        if (tinyGPSPlus.encode(c))// Did a new valid sentence come in?
          newData = true;
      }
    }

    if (newData)
    {
        direction_parse(tmp_data);
        double dlat, dlon;
        
        dlat = tinyGPSPlus.location.lat();
        dlon = tinyGPSPlus.location.lng();

        meshcom_settings.node_lat = cround4(dlat);
        meshcom_settings.node_lon = cround4(dlon);

        if(direction_S_N == 0)
        {
        meshcom_settings.node_lat_c = 'S';
        }
        else
        {
        meshcom_settings.node_lat_c = 'N';
        }

        if(direction_E_W == 0)
        {
        meshcom_settings.node_lon_c = 'E';
        }
        else
        {
        meshcom_settings.node_lon_c = 'W';
        }

        meshcom_settings.node_alt = ((meshcom_settings.node_alt * 10) + (int)tinyGPSPlus.altitude.meters()) / 11;

        MyClock.setCurrentTime(true, tinyGPSPlus.date.year(), tinyGPSPlus.date.month(), tinyGPSPlus.date.day(), tinyGPSPlus.time.hour(), tinyGPSPlus.time.minute(), tinyGPSPlus.time.second());

        posinfo_satcount = tinyGPSPlus.satellites.value();
        posinfo_hdop = tinyGPSPlus.hdop.value();
        posinfo_fix = true;

        return setSMartBeaconing(dlat, dlon);
    }

    return 0;   // no GPS
}

void checkSerialCommand(void)
{
    //  Check Serial connected
    if(!Serial)
    {
        DEBUG_MSG("SERIAL", "not connected");
        return;
    }

 	//check if we got from the serial input
    if(Serial.available() > 0)
    {
        char rd = Serial.read();

        Serial.print(rd);

        strText += rd;

        if(strText.startsWith(":") || strText.startsWith("-"))
        {
            if(strText.endsWith("\n") || strText.endsWith("\r"))
            {
                strText.trim();
                strcpy(msg_text, strText.c_str());

                int inext=0;
                char msg_buffer[300];
                for(int itx=0; itx<(int)strText.length(); itx++)
                {
                    if(msg_text[itx] == 0x08 || msg_text[itx] == 0x7F)
                    {
                        inext--;
                        if(inext < 0)
                            inext=0;
                            
                        msg_buffer[inext+1]=0x00;
                    }
                    else
                    {
                        msg_buffer[inext]=msg_text[itx];
                        msg_buffer[inext+1]=0x00;
                        inext++;
                    }
                }

                if(strText.startsWith(":"))
                    sendMessage(msg_buffer+1, inext-1);

                if(strText.startsWith("-"))
                    commandAction(msg_buffer, inext, false);

                strText="";
            }
        }
        else
        {
            if(!strText.startsWith("\n") && !strText.startsWith("\r"))
            {
                printf("MSG:%02X", rd);
                printf("..not sent\n");
            }
            strText="";
        }
    }
}

void addNodeData(uint8_t msg_buffer[300], uint16_t size, int16_t rssi, int8_t snr)
{
    Serial.println("ERROR - Gateway not supported RAK Modules");
}
