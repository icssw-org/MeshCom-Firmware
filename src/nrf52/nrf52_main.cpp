#include <Arduino.h>
#include <SPI.h>
#include <configuration.h>
#include <SX126x-RAK4630.h>
#include <debugconf.h>
#include <time.h>

#include <WisBlock-API.h>
#include <TinyGPS.h>
#include <drivers\Adafruit_SSD1680.h>
#include <WisBlock_EPaper_Images.h>
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

// NVIC_SystemReset(); resets the device

// Lora callback Function declarations
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void OnRxTimeout(void);
void OnRxError(void);
void OnTxDone(void);
void OnTxTimeout(void);
void OnPreambleDetect(void);
void OnHeaderDetect(void);


asm(".global _scanf_float");
asm(".global _printf_float");

// LoRa Events and Buffers
static RadioEvents_t RadioEvents;
static uint8_t RcvBuffer[UDP_TX_BUF_SIZE];
static uint8_t RcvBuffer_before[UDP_TX_BUF_SIZE];
static uint8_t txBuffer[UDP_TX_BUF_SIZE]; // we need an extra buffer for udp tx, as we add other stuff (ID, RSSI, SNR, MODE)
uint8_t lora_tx_buffer[UDP_TX_BUF_SIZE];  // lora tx buffer
static uint8_t ringBufferUDPout[MAX_RING_UDP_OUT][UDP_TX_BUF_SIZE]; //Ringbuffer for UDP TX from LoRa RX, first byte is length
uint8_t udpWrite = 0;   // counter for ringbuffer
uint8_t udpRead = 0;    // counter for ringbuffer

s_meshcom_settings g_meshcom_settings;
bool g_meshcom_initialized;
bool init_flash_done=false;

// RINGBUFFER for incoming UDP lora packets for lora TX
unsigned char ringBuffer[MAX_RING][UDP_TX_BUF_SIZE];
int iWrite=0;
int iRead=0;

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

// BLE Ringbuffer to phone
unsigned char BLEtoPhoneBuff[MAX_RING][UDP_TX_BUF_SIZE];
int toPhoneWrite=0;
int toPhoneRead=0;


bool bDEBUG=false;

//variables and helper functions
int sendlng = 0;              // lora tx message length
uint8_t cmd_counter = 2;      // ticker dependant on main cycle delay time
void print_radioStatus(void); // prints the current Radio Status
uint8_t preamble_cnt = 0;     // stores how often a preamble detect is thrown
bool tx_is_active = false;    // avoids calling doTX() on each main iteration when we are already in TX mode
bool is_receiving = false;  // flag to store we are receiving a lora packet. triggered by header detect not preamble
uint8_t err_cnt_udp_tx = 0;    // counter on errors sending message via UDP
bool ble_busy_flag = false;    // flag to signal bluetooth uart is active
uint8_t isPhoneReady = 0;      // flag we receive from phone when itis ready to receive data

// timers
unsigned long hb_time = 0;            // heartbeat timer
unsigned long dhcp_timer = 0;         // dhcp refresh timer
unsigned long ntp_timer = 0;          // dhcp refresh timer
unsigned long chk_udp_conn_timer = 0; // we check periodically if we have received a HB from server
unsigned long posinfo_timer = 0;      // we check periodically to send GPS
//unsigned long till_header_time = 0;    // stores till a header is detected after preamble detect
unsigned int posinfo_first = 0;
unsigned int msg_counter = 0;
char msg_text[MAX_MSG_LEN_PHONE];

String strText="";

// TinyGPS
TinyGPS gps;

String tmp_data = "";
int direction_S_N = 0;  //0--S, 1--N
int direction_E_W = 0;  //0--E, 1--W

void getGPS(void);

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
Adafruit_SSD1680 display(DEPG_HP.width, DEPG_HP.height, EPD_MOSI,
                         EPD_SCK, EPD_DC, EPD_RESET,
                         EPD_CS, SRAM_CS, EPD_MISO,
                         EPD_BUSY);

void testdrawtext(int16_t x, int16_t y, char *text, uint16_t text_color, uint32_t text_size);

uint8_t gKeyNum = 0; // which button is pressed
int iGPSCount=0;
int iBLECount=0;

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
  display.setCursor(x, y);
  display.setTextColor(text_color);
  display.setTextSize(text_size);
  display.setTextWrap(true);
  display.print(text);
}

// Prototypes
bool is_new_packet(uint16_t size);                         // switch if we have a packet received we never saw before RcvBuffer[12] changes, rest is same
void addNodeData(uint16_t size, int16_t rssi, int8_t snr); // add additional data we need and send the udp packet
void blinkLED();
void sendHeartbeat();                                // heartbeat to server
void doTX();                                         // LoraTX function
void addUdpOutBuffer(int msg_id, uint8_t *buffer, uint16_t len); // function adds outgoing udp messages in the udp_out_ringbuffer
void printBuffer(uint8_t *buffer, int len);
void printBuffer_ascii(uint8_t *buffer, int len);
void sendMessage(char *buffer, int len);
void sendPosition(double lat, char lat_c, double lon, char lon_c, int alt, int batt);
void commandAction(char *buffer, int len, bool ble);
void addBLEOutBuffer(uint8_t *buffer, uint16_t len);
void sendToPhone();


// Client basic variables
    uint8_t dmac[6];
    unsigned int _GW_ID = 0x4B424332; // ID of our Node

/**
 * BLE
*/

/** Set the device name, max length is 10 characters */
    char g_ble_dev_name[10] = "RAK-TEST";

/**
 * @brief Initialize LoRa HW and LoRaWan MAC layer
 *
 * @return int8_t result
 *  0 => OK
 * -1 => SX126x HW init failure
 * -2 => LoRaWan MAC initialization failure
 * -3 => Subband selection failure
 */
int8_t init_meshcom(void)
{
#ifdef ESP32
	pinMode(WB_IO2, OUTPUT);
	digitalWrite(WB_IO2, HIGH);
	delay(500);
#endif

	// Initialize LoRa chip.
	if (api_init_lora() != 0)
	{
		API_LOG("LORA", "Failed to initialize SX1262");
		return -1;
	}

	// Setup the EUIs and Keys
    /*KBC
	lmh_setDevEui(g_lorawan_settings.node_device_eui);
	lmh_setAppEui(g_lorawan_settings.node_app_eui);
	lmh_setAppKey(g_lorawan_settings.node_app_key);
	lmh_setNwkSKey(g_lorawan_settings.node_nws_key);
	lmh_setAppSKey(g_lorawan_settings.node_apps_key);
	lmh_setDevAddr(g_lorawan_settings.node_dev_addr);

	// Setup the LoRaWan init structure
	lora_param_init.adr_enable = g_lorawan_settings.adr_enabled;
	lora_param_init.tx_data_rate = g_lorawan_settings.data_rate;
	lora_param_init.enable_public_network = g_lorawan_settings.public_network;
	lora_param_init.nb_trials = g_lorawan_settings.join_trials;
	lora_param_init.tx_power = g_lorawan_settings.tx_power;
	lora_param_init.duty_cycle = g_lorawan_settings.duty_cycle_enabled;
   
	API_LOG("LORA", "Initialize LoRaWAN for region %s", region_names[g_lorawan_settings.lora_region]);
	// Initialize LoRaWan
	if (lmh_init(&lora_callbacks, lora_param_init, g_lorawan_settings.otaa_enabled, (eDeviceClass)g_lorawan_settings.lora_class, (LoRaMacRegion_t)g_lorawan_settings.lora_region) != 0)
	{
		API_LOG("LORA", "Failed to initialize LoRaWAN");
		return -2;
	}

	// For some regions we might need to define the sub band the gateway is listening to
	// This must be called AFTER lmh_init()
	if (!lmh_setSubBandChannels(g_lorawan_settings.subband_channels))
	{
		API_LOG("LORA", "lmh_setSubBandChannels failed. Wrong sub band requested?");
		return -3;
	}

	API_LOG("LORA", "Begin timer");
	// Initialize the app timer
	api_timer_init();

	API_LOG("LORA", "Start Join");
	// Start Join process
	lmh_join();

    */
	g_meshcom_initialized = true;
	return 0;
}

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
        RcvBuffer[i] = RcvBuffer_before[i] = 0x00;
    }

    //clear ringbuffer
    for(int i=0; i<MAX_RING_UDP_OUT; i++)
    {
        memset(ringBufferUDPout[i], 0, UDP_TX_BUF_SIZE);
    }

	// Initialize battery reading
	init_batt();

	// Get LoRa parameter
	init_flash();

    getMacAddr(dmac);

    _GW_ID = dmac[0] | (dmac[1] << 8) | (dmac[2] << 16) | (dmac[3] << 24);


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
    while (!Serial1);
    Serial.println("gps uart init ok!");

    //ble init    
    g_enable_ble=true;

#if defined NRF52_SERIES || defined ESP32
	if (g_enable_ble)
	{
		API_LOG("API", "Init BLE");
		// Init BLE
		init_ble();
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


    Serial.println("=====================================");
    Serial.println("CLIENT STARTED");
    Serial.println("=====================================");

    //  Set the LoRa Callback Functions
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;
    //RadioEvents.PreAmpDetect = OnPreambleDetect;
    //NICHT BENÖTIGTRadioEvents.HeaderDetect = OnHeaderDetect;
    

    //  Initialize the LoRa Transceiver
    Radio.Init(&RadioEvents);

    // Sets the Syncowrd new that we can set the MESHTASTIC SWORD
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
    pinMode(LEFT_BUTTON, INPUT);
    attachInterrupt(LEFT_BUTTON, interruptHandle1, FALLING);

    // set middle button interrupt
    pinMode(MIDDLE_BUTTON, INPUT);
    attachInterrupt(MIDDLE_BUTTON, interruptHandle2, FALLING);

    // set right button interrupt
    pinMode(RIGHT_BUTTON, INPUT);
    attachInterrupt(RIGHT_BUTTON, interruptHandle3, FALLING);

    delay(100);
}

void nrf52loop()
{
   	digitalWrite(LED_GREEN, LOW);
   	digitalWrite(LED_BLUE, LOW);

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

    if(gKeyNum == 1)
    {
        Serial.println("Left button pressed");
        display.clearBuffer();
        display.drawBitmap(DEPG_HP.position2_x, DEPG_HP.position2_y, rak_img, 150, 56, EPD_BLACK);
        display.display(true);
        gKeyNum = 0;
    }

    if(gKeyNum == 2)
    {
        // GPS FIX
        iGPSCount++;

        if(iGPSCount > 10)
        {
            getGPS();

           	digitalWrite(LED_GREEN, HIGH);

            iGPSCount=0;
        }
        /*
        Serial.println("Middle button pressed");
        display.clearBuffer();
        testdrawtext(DEPG_HP.position3_x, DEPG_HP.position3_y, (char*)"IoT Made Easy", (uint16_t)EPD_BLACK, (uint32_t)2);
        display.display(true);
        */
        gKeyNum = 0;
    }

    if(gKeyNum == 3)
    {
        Serial.println("Right button pressed");
        display.clearBuffer();
        display.drawBitmap(DEPG_HP.position4_x, DEPG_HP.position4_y, lora_img, 60, 40, EPD_BLACK);
        display.display(true);
        gKeyNum = 0;
    }

  	//check if we got from the serial input
    if(Serial.available() > 0)
    {
        char rd = Serial.read();

        Serial.print(rd);

        strText += rd;

        //str.trim();                 // removes CR, NL, Whitespace at end

        if (strText.startsWith("reset"))
        {
            strText="";
            NVIC_SystemReset();     // resets the device
        }
        else
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

    // check if we have messages for BLE to send
    if(g_ble_uart_is_connected && !ble_busy_flag && isPhoneReady == 1)
    {
        if (toPhoneWrite != toPhoneRead)
        {
            sendToPhone();   
        }

        iBLECount++;
        if(iBLECount > 10)
        {
            iBLECount=0;
            digitalWrite(LED_BLUE, HIGH);
        }

    }

    // posinfo
    if (((posinfo_timer + POSINFO_INTERVAL * 1000) < millis()) || posinfo_first == 1)
    {
        sendPosition(g_meshcom_settings.node_lat, g_meshcom_settings.node_lat_c, g_meshcom_settings.node_lon, g_meshcom_settings.node_lon_c, g_meshcom_settings.node_alt, mv_to_percent(read_batt()));
        posinfo_first=2;

        save_settings();    // Position ins Flash schreiben

        posinfo_timer = millis();
    }

    //  We are on FreeRTOS, give other tasks a chance to run
    delay(100);
    yield();
}



/** @brief Function to be executed on Radio Rx Done event
 */
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    //unsigned long diff_rx = millis() - till_header_time;
    //DEBUG_MSG_VAL("RADIO", diff_rx, "Time Preamble to RxDone");

    memcpy(RcvBuffer, payload, size);

    if (is_new_packet(size) || (SEE_ALL_PACKETS == 1))
    {
		// :|0x11223344|0x05|OE1KBC|>*:Hallo Mike, ich versuche eine APRS Meldung\0x00

        // print which message type we got
        uint8_t msg_type_b_lora = payload[0];

        switch (msg_type_b_lora)
        {

            case 0x3A: DEBUG_MSG("RADIO", "Received Textmessage"); break;
            case 0x21: DEBUG_MSG("RADIO", "Received PosInfo"); break;
            case 0x05: DEBUG_MSG("RADIO", "Routing APP"); break;
            case 0x06: DEBUG_MSG("RADIO", "Admin APP"); break;
            case 0x20: DEBUG_MSG("RADIO", "Reply APP"); break;
            case 0x42: DEBUG_MSG("RADIO", "Rangetest APP"); break;
            case 0x43: DEBUG_MSG("RADIO", "Environmental APP"); break;
            default:
                DEBUG_MSG("RADIO", "Received unknown");
                if(bDEBUG)
                    printBuffer(RcvBuffer, size);
                break;
        }

        // txtmessage, position
        if(msg_type_b_lora == 0x3A || msg_type_b_lora == 0x21)
        {
            // print ascii of message
            printBuffer_ascii(RcvBuffer, size);

            // we add now Longname (up to 20), ID - 4, RSSI - 2, SNR - 1 and MODE BYTE - 1
            // MODE BYTE: LongSlow = 1, MediumSlow = 3
            // and send the UDP packet (done in the method)

            // we only send the packet via UDP if we have no collision with UDP rx
            // und wenn MSG nicht von einem anderen Gateway empfangen wurde welches es bereits vopm Server bekommen hat
            int msg_id = (RcvBuffer[4]<<24) | (RcvBuffer[3]<<16) | (RcvBuffer[2]<<8) | RcvBuffer[1];
            int msg_hop = RcvBuffer[5] & 0x07;
            int msg_hop_pos = 5;
            bool msg_server =false;
            if(RcvBuffer[5] & 0x80)
                msg_server=true;

            int lora_msg_len = size; // size ist uint16_t !
            if (lora_msg_len > UDP_TX_BUF_SIZE)
            lora_msg_len = UDP_TX_BUF_SIZE; // zur Sicherheit

            if(bDEBUG)
                printf("msg_id: %04X msg_len: %i payload[%i]=%i via=%d\n", msg_id, lora_msg_len, msg_hop_pos, msg_hop, msg_server);

            //if(!msg_server) // Message kommt von User
            {
                // Wiederaussendung via LORA
                // Ringbuffer filling

                bool bMsg=false;

                for(int iop=0;iop<MAX_RING_UDP_OUT;iop++)
                {
                    int ring_msg_id = (ringBufferUDPout[iop][3]<<24) | (ringBufferUDPout[iop][2]<<16) | (ringBufferUDPout[iop][1]<<8) | ringBufferUDPout[iop][0];

                    if(ring_msg_id != 0 && bDEBUG)
                        printf("ring_msg_id:%08X msg_id:%08X\n", ring_msg_id, msg_id);

                    if(ring_msg_id == msg_id)
                    {
                        bMsg=true;

                        break;
                    }
                }

                if(!bMsg)
                {
                    if(msg_hop > 0 && msg_hop_pos > 0)
                        payload[msg_hop_pos]=msg_hop-1;

                    if ((msg_type_b_lora == 0x3A || msg_type_b_lora == 0x21))
                    {
                        // add rcvMsg to forward to LoRa TX
                        addNodeData(size, rssi, snr);

                        // add rcvMsg to BLE out Buff
                        // size message is int -> uint16_t buffer size
                        if(isPhoneReady == 1)
                            addBLEOutBuffer(RcvBuffer, size);

                        DEBUG_MSG("RADIO", "Packet sent to -->");
                    }
                    else
                    {
                        //DEBUG_MSG("RADIO", "Packet not sent to -->");
                    }
                } 
            }   
        }
        else
        {
            // print hex of message
            if(bDEBUG)
                printBuffer(RcvBuffer, size);
        }

        if(bDEBUG)
            Serial.println("");

        // store received message to compare later on
        memcpy(RcvBuffer_before, RcvBuffer, UDP_TX_BUF_SIZE);
        // set buffer to 0
        memset(RcvBuffer, 0, UDP_TX_BUF_SIZE);

        blinkLED();
    }
    else
        DEBUG_MSG("RADIO", "Packet discarded, already seen it!");

    cmd_counter = WAIT_AFTER_RX;
    is_receiving = false;

    Radio.Rx(RX_TIMEOUT_VALUE);

}

/**@brief Function to be executed on Radio Rx Timeout event
 */
void OnRxTimeout(void)
{
    // DEBUG_MSG("RADIO", "OnRxTimeout");
    Radio.Rx(RX_TIMEOUT_VALUE);
    is_receiving = false;
}

/**@brief Function to be executed on Radio Rx Error event
 */

void OnRxError(void)
{
    cmd_counter = WAIT_AFTER_RX;
    //DEBUG_MSG("RADIO", "RX ERROR");
    Radio.Rx(RX_TIMEOUT_VALUE);
    is_receiving = false;
}

/**@brief Function to be executed on Radio Tx Done event
 */
void OnTxDone(void)
{
    // DEBUG_MSG("RADIO", "OnTxDone");
    cmd_counter=WAIT_AFTER_TXDONE;
    Radio.Rx(RX_TIMEOUT_VALUE);
    tx_is_active = false;
    digitalWrite(LED_BLUE, LOW);
}

/**@brief Function to be executed on Radio Tx Timeout event
 */
void OnTxTimeout(void)
{
    // DEBUG_MSG("RADIO", "OnTxTimeout");
    tx_is_active = false;
    digitalWrite(LED_BLUE, LOW);
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
            digitalWrite(LED_BLUE, HIGH);

            // print tx buffer
            //printBuffer(neth.lora_tx_buffer_eth, neth.lora_tx_msg_len);

            iRead++;
            if (iRead >= MAX_RING)
                iRead = 0;

            Radio.Send(lora_tx_buffer, sendlng);

            cmd_counter = WAIT_TX;

            if (iWrite == iRead)
            {
                DEBUG_MSG("RADIO",  "TX (LAST) :");
            }
            else
            {
                DEBUG_MSG("RADIO", "TX :");
            }

            Serial.println("");
        }
        else
        {
            DEBUG_MSG("RADIO", "TX DISABLED");
        }
    }
}


/**@brief Function to check if we have a Lora packet already received
 */
bool is_new_packet(uint16_t size)
{
    for (int i = 0; i < size; i++)
    {
        if (i != 12)
        {
            if (RcvBuffer[i] != RcvBuffer_before[i])
            {
                return true;
            }
        }
    }
    return false;
}

/**@brief Function to write our additional data into the UDP tx buffer
 * we add now Longname (up to 20), ID - 4, RSSI - 2, SNR - 1 and MODE BYTE - 1
 * MODE BYTE: LongSlow = 1, MediumSlow = 3
 * 8 byte offset = ID+RSSI+SNR
 */
void addNodeData(uint16_t size, int16_t rssi, int8_t snr)
{
    // copying the contents of the
    // string to char array
    uint8_t offset = 8 + strlen(g_meshcom_settings.node_call) + 1; // offset for the payload written into tx udp buffer. We add 0x00 after Longanme

    if (strlen(g_meshcom_settings.node_call) <= LONGNAME_MAXLEN)
    {
        memcpy(txBuffer, g_meshcom_settings.node_call, strlen(g_meshcom_settings.node_call));
        txBuffer[strlen(g_meshcom_settings.node_call)] = 0x00; // we add a trailing 0x00 to mark the end of longname
    }
    else
    {
        DEBUG_MSG("ERROR", "LongName is too long!");
    }

    uint8_t offset_params = strlen(g_meshcom_settings.node_call) + 1;
    memcpy(&txBuffer[offset_params], &_GW_ID, sizeof(_GW_ID));
    memcpy(&txBuffer[offset_params + 4], &rssi, sizeof(rssi));
    txBuffer[offset_params + 6] = snr;
    txBuffer[offset_params + 7] = 0x03; // manually set to 0x03 because we are on MediumSlow per default

    // now copy the rcvbuffer into txbuffer
    if ((size + 8 + offset) < UDP_TX_BUF_SIZE)
    {
        for (int i = 0; i < size; i++)
        {
            txBuffer[i + offset] = RcvBuffer[i];
        }
        // add it to the outgoing udp buffer
        // TODO change txBuffer with rinbuffer
        //DEBUG_MSG("UDP", "UDP out Buffer");
        //neth.printBuffer(txBuffer, (size + offset));

        int msg_id = (RcvBuffer[4]<<24) | (RcvBuffer[3]<<16) | (RcvBuffer[2]<<8) | RcvBuffer[1];

        addUdpOutBuffer(msg_id, txBuffer, (size + offset));
    }
    else
    {
        DEBUG_MSG("ERROR", "Exceeding Buffer length!");
    }

}

/**@brief Function adding messages into outgoing UDP ringbuffer
 * 
 */
void addUdpOutBuffer(int msg_id, uint8_t *buffer, uint16_t len)
{
    if (len > UDP_TX_BUF_SIZE)
        len = UDP_TX_BUF_SIZE; // just for safety

    // byte 0-3 msg_id
    ringBufferUDPout[udpWrite][3] = msg_id >> 24;
    ringBufferUDPout[udpWrite][2] = msg_id >> 16;
    ringBufferUDPout[udpWrite][1] = msg_id >> 8;
    ringBufferUDPout[udpWrite][0] = msg_id;

    // byte 4 one byte is always the message length
    ringBufferUDPout[udpWrite][4] = len;
    memcpy(ringBufferUDPout[udpWrite] + 4 + 1, buffer, len);

    if(bDEBUG)
    {
        Serial.printf("Out Ringbuffer added element: %u\n", udpWrite);
        //DEBUG_MSG_VAL("UDP", udpWrite, "UDP Ringbuf added El.:");
        printBuffer(ringBufferUDPout[udpWrite], len);
    }

    udpWrite++;
    if (udpWrite >= MAX_RING_UDP_OUT) // if the buffer is full we start at index 0 -> take care of overwriting!
        udpWrite = 0;
}

/** @brief Function adding messages into outgoing BLE ringbuffer
 * BLE to PHONE Buffer
 */
void addBLEOutBuffer(uint8_t *buffer, uint16_t len)
{
    if (len > UDP_TX_BUF_SIZE)
        len = UDP_TX_BUF_SIZE; // just for safety

    //first two bytes are always the message length
    BLEtoPhoneBuff[toPhoneWrite][0] = len;
    memcpy(BLEtoPhoneBuff[toPhoneWrite] + 1, buffer, len);

    if(bDEBUG)
    {
        Serial.printf("BLEtoPhone RingBuff added element: %u\n", toPhoneWrite);
        //DEBUG_MSG_VAL("UDP", udpWrite, "UDP Ringbuf added El.:");
        printBuffer(BLEtoPhoneBuff[toPhoneWrite], len + 1);
    }

    toPhoneWrite++;
    if (toPhoneWrite >= MAX_RING_UDP_OUT) // if the buffer is full we start at index 0 -> take care of overwriting!
        toPhoneWrite = 0;
}

/**
 * @brief Method to send incoming LoRa messages to BLE connected device
 * 
*/
void sendToPhone()
{
    ble_busy_flag = true;

    // we need to insert the first byte text msg flag
    uint8_t toPhoneBuff [MAX_MSG_LEN_PHONE] = {0};

    uint16_t blelen = BLEtoPhoneBuff[toPhoneRead][0] + 1;   //len ist um ein byte zu kurz

    toPhoneBuff[0] = blelen;
    toPhoneBuff[1] = blelen >> 8;
    toPhoneBuff[2] = 0x40;

    memcpy(toPhoneBuff+3, BLEtoPhoneBuff[toPhoneRead]+1, blelen);

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
    g_ble_uart.write(toPhoneBuff, blelen + 3);
#endif

    toPhoneRead++;
    if (toPhoneRead >= MAX_RING)
        toPhoneRead = 0;

    if (toPhoneWrite == toPhoneRead)
    {
        DEBUG_MSG_VAL("BLE", toPhoneRead,"TX (LAST) :");
    }
    else
    {
        DEBUG_MSG_VAL("BLE", toPhoneRead,"TX :");
    }

    Serial.println("");

    ble_busy_flag = false;
}



/** @brief Method to print our buffers
 */
void printBuffer(uint8_t *buffer, int len)
{
  for (int i = 0; i < len; i++)
  {
    Serial.printf("%02X ", buffer[i]);
  }
  Serial.println("");
}

/** @brief Method to print our buffers
 */
void printBuffer_ascii(uint8_t *buffer, int len)
{
  int i=0;

  Serial.printf("%03i ", len);

  Serial.printf("%c !", buffer[0]);
  for (i = 4; i > 0; i--)
  {
    Serial.printf("%02X", buffer[i]);
  }
  Serial.printf(" %02X ", buffer[5]);

  int ineg=2;
  if(buffer[len-7] == 0x00)
    ineg=6;

  for (i = 6; i < len-ineg; i++)
  {
    if(buffer[i] != 0x00)
      Serial.printf("%c", buffer[i]);
  }

  Serial.printf(" %02X", buffer[len-ineg]);
  Serial.printf("%02X", buffer[(len-ineg)+1]);

  Serial.println("");
}

/** @brief Function to check if the modem detected a preamble
 */
void blinkLED()
{
    digitalWrite(LED_GREEN, HIGH);
    delay(2);
    digitalWrite(LED_GREEN, LOW);
}

void blinkLED2()
{
    digitalWrite(LED_BLUE, HIGH);
    delay(2);
    digitalWrite(LED_BLUE, LOW);
}


/** @brief Function to get the current Radio Status
 * Radio status.[RF_IDLE, RF_RX_RUNNING, RF_TX_RUNNING]
 */

void print_radioStatus()
{
    if(Radio.GetStatus() == RF_IDLE) Serial.println("RF_IDLE");
    if(Radio.GetStatus() == RF_RX_RUNNING) Serial.println("RF_RX_RUNNING");
    if(Radio.GetStatus() == RF_TX_RUNNING) Serial.println("RF_TX_RUNNING");
    if(Radio.GetStatus() == RF_CAD) Serial.println("RF_CAD");

} 

void sendMessage(char *msg_text, int len)
{
    uint8_t msg_buffer[300];
    char msg_start[100];

    // :|0x11223344|0x05|OE1KBC|>*:Hallo Mike, ich versuche eine APRS Meldung\0x00

    msg_buffer[0]=':';
    
    msg_counter=millis();

    msg_buffer[1]=msg_counter & 0xff;
    msg_buffer[2]=(msg_counter >> 8) & 0xff;
    msg_buffer[3]=(msg_counter >> 16) & 0xff;
    msg_buffer[4]=(msg_counter >> 24) & 0xff;

    msg_buffer[5]=0x05; //max hop

    sprintf(msg_start, "%s>*", g_meshcom_settings.node_call);

    memcpy(msg_buffer+6, msg_start, strlen(g_meshcom_settings.node_call)+2);

    int inext=6+2+strlen(g_meshcom_settings.node_call);

    memcpy(msg_buffer+inext, msg_text, len);

    inext=inext+len;
    
    /*
    msg_buffer[inext]='/';
    inext++;

    sprintf(msg_start, "%03d%%", mv_to_percent(read_batt()));
    memcpy(msg_buffer+inext, msg_start, 4);

    inext+=4;
    */

    msg_buffer[inext]=0x00;
    inext++;

    int FCS_SUMME=0;
    for(int ifcs=0; ifcs<inext; ifcs++)
    {
        FCS_SUMME += msg_buffer[ifcs];
    }
    
    // FCS
    msg_buffer[inext] = (FCS_SUMME >> 8) & 0xFF;
    inext++;
    msg_buffer[inext] = FCS_SUMME & 0xFF;
    inext++;

    // _GW_ID   nur für 2.0 -> 4.0
    msg_buffer[inext] = (_GW_ID) & 0xFF;
    inext++;
    msg_buffer[inext] = (_GW_ID >> 8) & 0xFF;
    inext++;
    msg_buffer[inext] = (_GW_ID >> 16) & 0xFF;
    inext++;
    msg_buffer[inext] = (_GW_ID >> 24) & 0xFF;
    inext++;

    if(bDEBUG)
    {
        printBuffer(msg_buffer, inext);
        Serial.println("");
    }

    ringBuffer[iWrite][0]=inext;
    memcpy(ringBuffer[iWrite]+1, msg_buffer, inext);
    
    iWrite++;
    if(iWrite > MAX_RING)
        iWrite=0;

    DEBUG_MSG("RADIO", "Packet sent to -->");
}

void sendPosition(double lat, char lat_c, double lon, char lon_c, int alt, int batt)
{
    if(lat == 0 or lon == 0)
        return;

    uint8_t msg_buffer[MAX_MSG_LEN_PHONE];
    char msg_start[100];

    // :|0x11223344|0x05|OE1KBC|>*:Hallo Mike, ich versuche eine APRS Meldung\0x00

    msg_buffer[0]='!';
    
    msg_counter=millis();

    msg_buffer[1]=msg_counter & 0xff;
    msg_buffer[2]=(msg_counter >> 8) & 0xff;
    msg_buffer[3]=(msg_counter >> 16) & 0xff;
    msg_buffer[4]=(msg_counter >> 24) & 0xff;

    msg_buffer[5]=0x05; //max hop

    sprintf(msg_start, "%s>*", g_meshcom_settings.node_call);

    memcpy(msg_buffer+6, msg_start, strlen(g_meshcom_settings.node_call)+2);

    int inext=6+2+strlen(g_meshcom_settings.node_call);

    sprintf(msg_start, "!%07.2lf%c%c%08.2lf%c%c %i /A=%i", lat*100.0, lat_c, g_meshcom_settings.node_symid, lon*100.0, lon_c, g_meshcom_settings.node_symcd, batt, alt);

    memcpy(msg_buffer+inext, msg_start, strlen(msg_start));

    inext=inext+strlen(msg_start);
    
    msg_buffer[inext]=0x00;
    inext++;

    int FCS_SUMME=0;
    for(int ifcs=0; ifcs<inext; ifcs++)
    {
        FCS_SUMME += msg_buffer[ifcs];
    }
    
    // FCS
    msg_buffer[inext] = (FCS_SUMME >> 8) & 0xFF;
    inext++;
    msg_buffer[inext] = FCS_SUMME & 0xFF;
    inext++;

    // _GW_ID   nur für 2.0 -> 4.0
    msg_buffer[inext] = (_GW_ID) & 0xFF;
    inext++;
    msg_buffer[inext] = (_GW_ID >> 8) & 0xFF;
    inext++;
    msg_buffer[inext] = (_GW_ID >> 16) & 0xFF;
    inext++;
    msg_buffer[inext] = (_GW_ID >> 24) & 0xFF;
    inext++;

    if(bDEBUG)
    {
        printBuffer_ascii(msg_buffer, inext);
    }

    ringBuffer[iWrite][0]=inext;
    memcpy(ringBuffer[iWrite]+1, msg_buffer, inext);
    
    iWrite++;
    if(iWrite > MAX_RING)
        iWrite=0;

    DEBUG_MSG("RADIO", "Position sent to -->");
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
    bool newData = false;
  
    // For one second we parse GPS data and report some key values
    for (unsigned long start = millis(); millis() - start < 1000;)
    {
      while (Serial1.available())
      {
        char c = Serial1.read();
        //Serial.write(c); // uncomment this line if you want to see the GPS data flowing
        tmp_data += c;
        if (gps.encode(c))// Did a new valid sentence come in?
          newData = true;
      }
    }

    direction_parse(tmp_data);
    tmp_data = "";
    float flat, flon;
    
    if (newData)
    {
        unsigned long age;  
        gps.f_get_position(&flat, &flon, &age);
        flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat;
        flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon;

        g_meshcom_settings.node_lat = flat;
        g_meshcom_settings.node_lon = flon;

        if(direction_S_N == 0)
        {
        g_meshcom_settings.node_lat_c = 'S';
        }
        else
        {
        g_meshcom_settings.node_lat_c = 'N';
        }

        if(direction_E_W == 0)
        {
        g_meshcom_settings.node_lon_c = 'E';
        }
        else
        {
        g_meshcom_settings.node_lon_c = 'W';
        }

        g_meshcom_settings.node_alt = (int)gps.f_altitude();

        unsigned long date, time;
        gps.get_datetime(&date, &time, &g_meshcom_settings.node_age);

        g_meshcom_settings.node_date_year = date % 100;
        g_meshcom_settings.node_date_year += g_meshcom_settings.node_date_year > 80 ? 1900 : 2000;
        g_meshcom_settings.node_date_month = (date / 100) % 100;        
        g_meshcom_settings.node_date_day = date / 10000;

        g_meshcom_settings.node_date_hour = time / 1000000;
        g_meshcom_settings.node_date_minute = (time / 10000) % 100;
        g_meshcom_settings.node_date_second = (time / 100) % 100;
        g_meshcom_settings.node_date_hundredths = time % 100;

        //printf("Time: %ld\n", time);
    }
}

void commandAction(char *msg_text, int len, bool ble)
{
    char print_buff[500];

    // -info
    // -set-owner

    char _owner_c[MAX_CALL_LEN];
    double fVar=0.0;
    int iVar;
    String sVar;

    // copying the contents of the
    // string to char array

    bool bInfo=false;
    bool bPos=false;

    if(memcmp(msg_text, "-info", 5) == 0)
    {
        bInfo=true;
    }
    else
    if(memcmp(msg_text, "-pos", 4) == 0)
    {
        bPos=true;
    }
    else
    if(memcmp(msg_text, "-gps", 4) == 0)
    {
        getGPS();
        return;
    }
    else
    if(memcmp(msg_text, "-send-pos", 9) == 0)
    {
        sendPosition(g_meshcom_settings.node_lat, g_meshcom_settings.node_lat_c, g_meshcom_settings.node_lon, g_meshcom_settings.node_lon_c, g_meshcom_settings.node_alt, mv_to_percent(read_batt()));
        return;
    }
    else
    if(memcmp(msg_text, "-set_owner ", 11) == 0)
    {
        sprintf(_owner_c, "%s", msg_text+11);
        sVar = _owner_c;
        sVar.toUpperCase();
        sprintf(g_meshcom_settings.node_call, "%s", sVar.c_str());

        sprintf(_owner_c, "XXX40");

        for(int its=11; its<21; its++)
        {
            if(msg_text[its] == '-')
            {
                memcpy(_owner_c, msg_text+its-3, 3);
                _owner_c[3]=0x34;
                _owner_c[4]=0x30;
                _owner_c[5]=0x00;
                break;
            }
        }

        sVar = _owner_c;
        sVar.toUpperCase();
        sprintf(g_meshcom_settings.node_short, "%s", sVar.c_str());

        save_settings();

        init_ble_name();

        bInfo=true;
    }
    else
    if(memcmp(msg_text, "-set_lat ", 9) == 0)
    {
        sprintf(_owner_c, "%s", msg_text+9);
        sscanf(_owner_c, "%lf", &fVar);

        //printf("_owner_c:%s fVar:%f\n", _owner_c, fVar);

        g_meshcom_settings.node_lat=fVar;

        save_settings();

        bPos=true;
    }
    else
    if(memcmp(msg_text, "-set_lon ", 9) == 0)
    {
        sprintf(_owner_c, "%s", msg_text+9);
        sscanf(_owner_c, "%lf", &fVar);

        g_meshcom_settings.node_lon=fVar;

        save_settings();

        bPos=true;
    }
    else
    if(memcmp(msg_text, "-set_alt ", 9) == 0)
    {
        sprintf(_owner_c, "%s", msg_text+9);
        sscanf(_owner_c, "%d", &iVar);

        g_meshcom_settings.node_alt=iVar;

        save_settings();

        bPos=true;
    }

    if(bInfo)
    {
        sprintf(print_buff, "MeshCom 4.0 Client\n...Call:  <%s>\n...Short: <%s>\n...ID %08X\n...MAC %02X %02X %02X %02X %02X %02X\n...BATT %.2f mV\n...PBATT %d %%\n...TIME %li ms\n",
                g_meshcom_settings.node_call, g_meshcom_settings.node_short, _GW_ID, dmac[0], dmac[1], dmac[2], dmac[3], dmac[4], dmac[5], read_batt(), mv_to_percent(read_batt()), millis());
        if(ble)
        {
    		g_ble_uart.write(print_buff, strlen(print_buff));
        }
        else
        {
            printf("\n%s", print_buff);
        }
    }
    else
    if(bPos)
    {
        sprintf(print_buff, "MeshCom 4.0 Client\n...LAT: %.6lf %c\n...LON: %.6lf %c\n...ALT: %i\n...DATE: %i.%02i.%02i %02i:%02i:%02i UTC\n",
         g_meshcom_settings.node_lat, g_meshcom_settings.node_lat_c, g_meshcom_settings.node_lon, g_meshcom_settings.node_lon_c, g_meshcom_settings.node_alt,
         g_meshcom_settings.node_date_year, g_meshcom_settings.node_date_month, g_meshcom_settings.node_date_day,
         g_meshcom_settings.node_date_hour, g_meshcom_settings.node_date_minute, g_meshcom_settings.node_date_second);

        if(ble)
        {
    		g_ble_uart.write(print_buff, strlen(print_buff));
        }
        else
        {
            printf("\n%s", print_buff);
        }
    }
    else
        printf("\nMeshCom 4.0 Client\n...wrong command %s\n", msg_text);
}