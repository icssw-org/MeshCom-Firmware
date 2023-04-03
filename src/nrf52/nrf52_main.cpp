// (C) 2023 OE1KBC Kurt Baumann, OE1KFR Rainer 
// (C) 2016, 2017, 2018, 2018, 2019, 2020 OE1KBC Kurt Baumann
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

#include <TinyGPS.h>
#include <drivers/Adafruit_SSD1680.h>
#include <WisBlock_EPaper_Images.h>

#include "Adafruit_SHTC3.h"

#include "Adafruit_LPS2X.h"

// MeshCom Common (ers32/nrf52) Funktions
#include <loop_functions.h>
#include <loop_functions_extern.h>
#include <command_functions.h>
#include <aprs_functions.h>
#include <batt_functions.h>
#include <lora_functions.h>

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

    SX126x-Arduino library for RAK4630:
    _hwConfig.CHIP_TYPE = SX1262;		   // Chip type, SX1261 or SX1262
    _hwConfig.PIN_LORA_RESET = 38;		   // LORA RESET
    _hwConfig.PIN_LORA_NSS = 42;		   // LORA SPI CS
    _hwConfig.PIN_LORA_SCLK = 43;		   // LORA SPI CLK
    _hwConfig.PIN_LORA_MISO = 45;		   // LORA SPI MISO
    _hwConfig.PIN_LORA_DIO_1 = 47;		   // LORA DIO_1
    _hwConfig.PIN_LORA_BUSY = 46;		   // LORA SPI BUSY
    _hwConfig.PIN_LORA_MOSI = 44;		   // LORA SPI MOSI
    _hwConfig.RADIO_TXEN = 39;			   // LORA ANTENNA TX ENABLE (e.g. eByte E22 module)
    _hwConfig.RADIO_RXEN = 37;			   // LORA ANTENNA RX ENABLE (e.g. eByte E22 module)
    _hwConfig.USE_DIO2_ANT_SWITCH = true;  // LORA DIO2 controls antenna
    _hwConfig.USE_DIO3_TCXO = true;		   // LORA DIO3 controls oscillator voltage (e.g. eByte E22 module)
    _hwConfig.USE_DIO3_ANT_SWITCH = false; // LORA DIO3 controls antenna (e.g. Insight SIP ISP4520 module)
    _hwConfig.USE_RXEN_ANT_PWR = true;	   // RXEN is used as power for antenna switch

    */

/*
#define I2C_SCL_RAK1901 NRF_GPIO_PIN_MAP(0,14)
#define I2C_SDA_RAK1901 NRF_GPIO_PIN_MAP(0,13)

#define I2C_SCL_RAK12011 NRF_GPIO_PIN_MAP(0,14)
#define I2C_SDA_RAK12011 NRF_GPIO_PIN_MAP(0,13)
#define INT_RAK12011 NRF_GPIO_PIN_MAP(1,02)
*/

/*
Sync Word Setting in Meshtastic
    Meshtastic Syc Word is 0x2b

    Output of the LoRa Sync Word Register 0x0740 in Meshtastic:

    ??:??:?? 1 Set radio: final power level=22
    SYNC WORD SET!
    Sync Word 1st byte = 24
    Sync Word 2nd byte = b4
    ??:??:?? 1 SX126x init result 0
    ??:??:?? 1 Current limit set to 140.000000

    In our Library it gets set at sx126x.h / radio.cpp
    Define: sx126x.h line 109:
    #define LORA_MAC_PUBLIC_SYNCWORD 0x242b

    Radio.SetPublicNetwork(true); needs to be called, so syncword gets new set in radio.cpp line: 1183 in
    void RadioSetPublicNetwork(bool enable)
    Method
*/

//NVIC_SystemReset(); resets the device

// Lora callback Function declarations
void OnTxDone(void);
void OnTxTimeout(void);
void OnPreambleDetect(void);


asm(".global _scanf_float");
asm(".global _printf_float");

// LoRa Events and Buffers
static RadioEvents_t RadioEvents;

uint8_t lora_tx_buffer[UDP_TX_BUF_SIZE];  // lora tx buffer

bool g_meshcom_initialized;
bool init_flash_done=false;


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
char textbuff_phone [MAX_MSG_LEN_PHONE] = {0};
uint8_t txt_msg_len_phone = 0;

//variables and helper functions
int sendlng = 0;              // lora tx message length
uint8_t preamble_cnt = 0;     // stores how often a preamble detect is thrown
bool tx_is_active = false;    // avoids calling doTX() on each main iteration when we are already in TX mode
uint8_t err_cnt_udp_tx = 0;    // counter on errors sending message via UDP
bool ble_busy_flag = false;    // flag to signal bluetooth uart is active

// timers
unsigned long posinfo_timer = 0;      // we check periodically to send GPS
unsigned long temphum_timer = 0;      // we check periodically get TEMP/HUM
unsigned long druck_timer = 0;        // we check periodically get AIRPRESURE

String strText="";

// TinyGPS
TinyGPS gps;

int direction_S_N = 0;  //0--S, 1--N
int direction_E_W = 0;  //0--E, 1--W

void getGPS(void);

// TEMP/HUM
Adafruit_SHTC3 shtc3 = Adafruit_SHTC3();

void getTEMP(void);

Adafruit_LPS22 g_lps22hb;

void getPRESSURE(void);

// EPaper SSD1680
typedef struct  DEPG {
   int  width;
   int  height;
   int  position1_x;
   int  position1_y;  
   int  position2_x;
   int  position2_y;  
   int  position3_x;
   int  position3_y; 
   int  position4_x;
   int  position4_y;   
} DEPG;

//DEPG  DEPG_HP = {250,122,40,20,40,30,40,50,90,40};  //use DEPG0213BNS800F41HP as default
DEPG  DEPG_HP = {212,104,30,15,30,25,30,45,80,30};  //  this is for DEPG0213BNS800F42HP

#define POWER_ENABLE   WB_IO2

#define EPD_MOSI     MOSI
#define EPD_MISO     -1 // not used
#define EPD_SCK      SCK
#define EPD_CS       SS
#define EPD_DC       WB_IO1
#define SRAM_CS      -1 // not used
#define EPD_RESET    -1 // not used
#define EPD_BUSY     WB_IO4
#define LEFT_BUTTON    WB_IO3
#define MIDDLE_BUTTON  WB_IO5
#define RIGHT_BUTTON   WB_IO6

// 2.13" EPD with SSD1680
Adafruit_SSD1680 display_1680(DEPG_HP.width, DEPG_HP.height, EPD_MOSI,
                         EPD_SCK, EPD_DC, EPD_RESET,
                         EPD_CS, SRAM_CS, EPD_MISO,
                         EPD_BUSY);

void testdrawtext(int16_t x, int16_t y, char *text, uint16_t text_color, uint32_t text_size);

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

/**
   @brief Write a text on the display
   @param x x position to start
   @param y y position to start
   @param text text to write
   @param text_color color of text
   @param text_size size of text
*/
void testdrawtext(int16_t x, int16_t y, char *text, uint16_t text_color, uint32_t text_size)
{
  display_1680.setCursor(x, y);
  display_1680.setTextColor(text_color);
  display_1680.setTextSize(text_size);
  display_1680.setTextWrap(true);
  display_1680.print(text);
}

extern U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2;

bool bInitDisplay = false;

// Prototypes
void blinkLED();                                     // blink GREEN
void blinkLED2();                                    // blink BLUE
void doTX();                                         // LoraTX function
int CallToAPRS(char msg_type, uint8_t msg_buffer[MAX_MSG_LEN_PHONE]);
void sendMessage(char *buffer, int len);
int PositionToAPRS(uint8_t msg_buffer[MAX_MSG_LEN_PHONE], bool bConvPos, bool bWeather, double lat, char lat_c, double lon, char lon_c, int alt, int batt);
void sendPosition(double lat, char lat_c, double lon, char lon_c, int alt, int batt);
void sendWeather(double lat, char lat_c, double lon, char lon_c, int alt, float temp, float hum, float press);
void sendWX(char *buffer, float temp, float hum, float press);
void checkSerialCommand(void);
void sendToPhone();


// Client basic variables
    uint8_t dmac[6];

/** Set the device name, max length is 10 characters */
    char g_ble_dev_name[10] = "RAK-CL40";

void getMacAddr(uint8_t *dmac)
{
    const uint8_t *src = (const uint8_t *)NRF_FICR->DEVICEADDR;
    dmac[5] = src[0];
    dmac[4] = src[1];
    dmac[3] = src[2];
    dmac[2] = src[3];
    dmac[1] = src[4];
    dmac[0] = src[5] | 0xc0; // MSB high two bits get set elsewhere in the bluetooth stack
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

    for(int ib=0; ib<MAX_RING_UDP_OUT; ib++)
    {
        memset(RcvBuffer_before[ib], 0x00, 4);
    }

    //clear ringbuffer
    for(int i=0; i<MAX_RING_UDP_OUT; i++)
    {
        memset(ringBufferLoraRX[i], 0, 4);
    }

	// Initialize battery reading
	init_batt();

	// Get LoRa parameter
	init_flash();

    getMacAddr(dmac);

    _GW_ID = dmac[0] | (dmac[1] << 8) | (dmac[2] << 16) | (dmac[3] << 24);

    // BLE ID
    meshcom_settings.node_device_eui[0]=0x4D;   // M
    meshcom_settings.node_device_eui[1]=0x43;   // C
    for(int ieui=0; ieui<6; ieui++)
    {
        meshcom_settings.node_device_eui[ieui+2] = dmac[ieui];
    }


    //  Initialize the LoRa Module
    lora_rak4630_init();

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


    //gps init
    
    pinMode(WB_IO2, OUTPUT);
    digitalWrite(WB_IO2, 0);
    delay(1000);
    digitalWrite(WB_IO2, 1);
    delay(1000);
    
    Serial1.begin(9600);
    Serial1.setTimeout(500);
    while (!Serial1);


    Serial.println("=====================================");
    Serial.println("GPS UART init ok!");

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
    RadioEvents.PreAmpDetect = OnPreambleDetect;
    

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
    if(!bInitDisplay)
    {
        sendDisplayHead(mv_to_percent(read_batt()));

        bInitDisplay=true;
    }

   	//digitalWrite(LED_GREEN, LOW);
   	//digitalWrite(LED_BLUE, LOW);

    // check if we have messages in ringbuffer to send
    if (iWrite != iRead)
    {
        if(cmd_counter <= 0)
        {
            if (tx_is_active == false && is_receiving == false)
                doTX();
        }
        else
            cmd_counter--;
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
            getGPS();

           	//digitalWrite(LED_GREEN, HIGH);

            iGPSCount=0;
        }

        gKeyNum = 0;
    }

    if(gKeyNum == 3)
    {
        Serial.println("Right button pressed");

        gKeyNum = 0;
    }

    checkSerialCommand();

    // check if we have messages for BLE to send
    if (toPhoneWrite != toPhoneRead)
    {
        if(isPhoneReady == 1)
            sendToPhone();   
    }

    // posinfo
    if (((posinfo_timer + POSINFO_INTERVAL * 1000) < millis()))
    {
        sendPosition(meshcom_settings.node_lat, meshcom_settings.node_lat_c, meshcom_settings.node_lon, meshcom_settings.node_lon_c, meshcom_settings.node_alt, (int)mv_to_percent(read_batt()));

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

    //  We are on FreeRTOS, give other tasks a chance to run
    delay(100);
    
    yield();
}


/**@brief Function to be executed on Radio Tx Done event
 */
void OnTxDone(void)
{
    // DEBUG_MSG("RADIO", "OnTxDone");
    cmd_counter=WAIT_AFTER_TXDONE;
    Radio.Rx(RX_TIMEOUT_VALUE);
    tx_is_active = false;
    
    //digitalWrite(LED_BLUE, LOW);
}

/**@brief Function to be executed on Radio Tx Timeout event
 */
void OnTxTimeout(void)
{
    // DEBUG_MSG("RADIO", "OnTxTimeout");
    tx_is_active = false;
    
    //digitalWrite(LED_BLUE, LOW);
    
    Radio.Rx(RX_TIMEOUT_VALUE);
}

/**@brief fires when a preamble is detected 
 * currently not used!
 */
void OnPreambleDetect(void)
{
    //till_header_time = millis();
    preamble_cnt++;

    if(preamble_cnt >= 2){

        //DEBUG_MSG("RADIO", "Preamble detected");
        preamble_cnt = 0;
    } 
}

/**@brief our Lora TX sequence
 */

void doTX()
{
    tx_is_active = true;

    if (iWrite != iRead && iWrite < MAX_RING)
    {
        //int irs=iRead;

        sendlng = ringBuffer[iRead][0];
        memcpy(lora_tx_buffer, ringBuffer[iRead] + 1, sendlng);

        // we can now tx the message
        if (TX_ENABLE == 1)
        {
            //digitalWrite(LED_BLUE, HIGH);

            // print tx buffer
            //printBuffer(neth.lora_tx_buffer_eth, neth.lora_tx_msg_len);

            iRead++;
            if (iRead >= MAX_RING)
                iRead = 0;

            Radio.Send(lora_tx_buffer, sendlng);

            cmd_counter = WAIT_TX;

            if (iWrite == iRead)
            {
                DEBUG_MSG_VAL("RADIO", iRead,  "TX (LAST) :");
            }
            else
            {
                DEBUG_MSG_VAL("RADIO", iRead, "TX :");
            }

            Serial.println("");
        }
        else
        {
            DEBUG_MSG("RADIO", "TX DISABLED");
        }
    }
}

/**
 * @brief Method to send incoming LoRa messages to BLE connected device
 * 
*/
void sendToPhone()
{
    if(ble_busy_flag)
        return;

    ble_busy_flag = true;

    // we need to insert the first byte text msg flag
    uint8_t toPhoneBuff [MAX_MSG_LEN_PHONE] = {0};

    uint16_t blelen = BLEtoPhoneBuff[toPhoneRead][0];   //len ist um ein byte zu kurz

    toPhoneBuff[0] = 0x40;

    memcpy(toPhoneBuff+1, BLEtoPhoneBuff[toPhoneRead]+1, blelen);

if(g_ble_uart_is_connected && isPhoneReady == 1)
{

#if BLE_TEST > 0
    int tlen=0;
    for(int i=9; i<blelen+3; i++)
    {
        if(toPhoneBuff[i] == 0x00)
            break;
        
        toPhoneBuff[i-9]=toPhoneBuff[i];
        toPhoneBuff[i-8]=0x0a;
        toPhoneBuff[i-7]=0x00;
        tlen++;
    }
    tlen++;
    tlen++;

    g_ble_uart.write(toPhoneBuff, tlen);
#else
    g_ble_uart.write(toPhoneBuff, blelen + 2);
#endif

}
    toPhoneRead++;
    if (toPhoneRead >= MAX_RING)
        toPhoneRead = 0;

if(g_ble_uart_is_connected && isPhoneReady == 1)
{
    if (toPhoneWrite == toPhoneRead)
    {
        DEBUG_MSG_VAL("BLE", toPhoneRead,"TX (LAST) :");
    }
    else
    {
        DEBUG_MSG_VAL("BLE", toPhoneRead,"TX :");
    }

    Serial.println("");
}
    ble_busy_flag = false;
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
void getGPS(void)
{ 
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

        if (gps.encode(c))// Did a new valid sentence come in?
          newData = true;
      }
    }

    if (newData)
    {
        direction_parse(tmp_data);
        float flat, flon;
        
        unsigned long age;  
        gps.f_get_position(&flat, &flon, &age);
        //flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat;
        //flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon;

        meshcom_settings.node_lat = flat;
        meshcom_settings.node_lon = flon;

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

        meshcom_settings.node_alt = ((meshcom_settings.node_alt * 10) + (int)gps.f_altitude()) / 11;

        unsigned long date, time;
        gps.get_datetime(&date, &time, &meshcom_settings.node_age);

        meshcom_settings.node_date_year = date % 100;
        meshcom_settings.node_date_year += meshcom_settings.node_date_year > 80 ? 1900 : 2000;
        meshcom_settings.node_date_month = (date / 100) % 100;        
        meshcom_settings.node_date_day = date / 10000;

        meshcom_settings.node_date_hour = time / 1000000;
        meshcom_settings.node_date_minute = (time / 10000) % 100;
        meshcom_settings.node_date_second = (time / 100) % 100;
        meshcom_settings.node_date_hundredths = time % 100;

        if(bDEBUG)
            Serial.printf("\nTime: %ld", time);
    }

    if(bDEBUG)
        Serial.println();
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
                    sendMessage(msg_buffer, inext);

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
