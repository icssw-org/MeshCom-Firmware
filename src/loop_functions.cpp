#include "Arduino.h"

#include "loop_functions.h"
#include "command_functions.h"

#include "clock.h"

#include "batt_functions.h"
#include "udp_functions.h"
#include "extudp_functions.h"
#include "configuration.h"

#include "TinyGPSPlus.h"

#include <Wire.h> 

#if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
#include <lvgl.h>
#include <t-deck/tdeck_main.h>
#include <t-deck/lv_obj_functions.h>
#endif 

// TinyGPS
extern TinyGPSPlus tinyGPSPLus;

int BOARD_HARDWARE = MODUL_HARDWARE;
bool bUSER_BOARD_LED = false;

bool bRadio=false;

bool bLED_RED=false;
bool bLED_BLUE=false;
bool bLED_GREEN=false;
bool bLED_ORANGE=false;
bool bLED_CLEAR=true;

extern unsigned long rebootAuto;

int iWlanWait = 0;

extern float global_batt;
extern int global_proz;

bool bSetLoRaAPRS = false;

bool bDEBUG = false;
bool bLORADEBUG = false;
bool bBLEDEBUG = false;
bool bWXDEBUG = false;
bool bIODEBUG = false;

bool bPosDisplay = true;
bool bDisplayOff = false;
bool bDisplayIsOff = false;
bool bDisplayVolt = false;
bool bDisplayInfo = false;
bool bDisplayCont = false;
bool bDisplayRetx = false;
unsigned long DisplayOffWait = 0;
bool bDisplayTrack = false;
bool bGPSON = false;
bool bBMPON = false;
bool bBMP3ON = false;
bool bAHT20ON = false;
bool bBMEON = false;
bool bBME680ON = false;
bool bMCU811ON = false;
bool bINA226ON = false;
bool bRTCON = false;
bool bSMALLDISPLAY = false;
bool bSOFTSERON = false;
bool bNoMSGtoALL = false;

bool bTCA9548A = false;

bool bONEWIRE = false;

bool bLPS33 = false;

bool bme680_found = false;
bool bmx_found = false;
bool bmp3_found = false;
bool aht20_found =false;
bool mcu811_found = false;
bool one_found = false;
bool ina226_found = false;

bool bGATEWAY = false;
bool bGATEWAY_NOPOS = false;
bool bMESH = false;
bool bWEBSERVER = false;
bool bWIFIAP = false;
bool bEXTUDP = false;

bool bSHORTPATH = false;
bool bGPSDEBUG = false;
bool bSOFTSERDEBUG = false;

bool bBOOSTEDGAIN = false;

bool bBLElong = false;

int iDisplayType = 0;
int DisplayTimeWait = 0;

bool bWaitButton_Released = false;
bool bButtonCheck = false;
bool bcheckBottonRun = false;
uint8_t iButtonPin = 0;

bool bAnalogCheck = false;
float fAnalogValue = 0.0;
bool bAnalogFilter = false;

float fBattFaktor = 0.0;

int iInitDisplay = 0;
int iDisplayChange = 0;

unsigned long lastHeardTime = 0;
unsigned long posfixinterall = 0;

char cTimeSource[10];

char cBLEName[50]={0};

// SOFTSER global variables
String strSOFTSER_BUF = "";
bool bSOFTSER_APP = false;
int lastSOFTSER_MINUTE = -1;
int softserFunktion=0;

String strSOFTSERAPP_ID = "";    // ID der Messstelle
String strSOFTSERAPP_NAME = "";  // Name der Messstelle

// TELEMTRY global variables
int iNextTelemetry=0;
String strTelemetry="";

// common variables
char msg_text[MAX_MSG_LEN_PHONE * 2] = {0};

unsigned int _GW_ID = 0x12345678; // ID of our Node

#ifdef BOARD_E290
#include "heltec-eink-modules.h"

EInkDisplay_VisionMasterE290 e290_display;

#include "Fonts/FreeMonoBold9pt7b.h"
#include "Fonts/FreeMonoBold12pt7b.h"

#include "Fonts/FreeSans9pt7b.h"
#include "Fonts/FreeSans12pt7b.h"

int dzeile[6] = {16, 41, 61, 81, 101, 121};

#else

#ifdef BOARD_TBEAM_V3
int dzeile[6] = {11, 24, 34, 44, 54, 64};
#else
int dzeile[6] = {8, 21, 31, 41, 51, 61};
#endif

U8G2 *u8g2;

#if defined(BOARD_HELTEC)
    U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2_2(U8G2_R0, 16, 15, 4);
    U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2_1(U8G2_R0, 16, 15, 4);
#elif defined(BOARD_HELTEC_V3)
    U8G2_SH1106_128X64_NONAME_1_SW_I2C u8g2_2(U8G2_R0, 18, 17, 21);
    U8G2_SSD1306_128X64_NONAME_1_SW_I2C u8g2_1(U8G2_R0, 18, 17, 21);
#elif defined(BOARD_RAK4630)
    U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2_2(U8G2_R0);  //RESET CLOCK DATA
    U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2_1(U8G2_R0);  //RESET CLOCK DATA
#elif defined(BOARD_TBEAM_V3)
    U8G2_SH1106_128X64_NONAME_1_SW_I2C u8g2_1(U8G2_R0, 18, 17, U8X8_PIN_NONE);
    U8G2_SSD1306_128X64_NONAME_1_SW_I2C u8g2_2(U8G2_R0, 18, 17, U8X8_PIN_NONE);
#else
    U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2_2(U8G2_R0);
    U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2_1(U8G2_R0);
#endif

#endif

unsigned int msg_counter = 0;

// Buffer to hold temp info-frames
uint8_t RcvBuffer[UDP_TX_BUF_SIZE * 2] = {0};

// RINGBUFFER to hold own msg_id
uint8_t own_msg_id[MAX_RING][5] = {0};
int iWriteOwn=0;

// RINGBUFFER for incoming UDP lora packets for lora TX
unsigned char ringBuffer[MAX_RING][UDP_TX_BUF_SIZE+5] = {0};
int iWrite=0;
int iRead=0;
int iRetransmit=-1;

// RINGBUFFER for incomming LoRa RX msg_id
uint8_t ringBufferLoraRX[MAX_RING][5] = {0};
uint8_t loraWrite = 0;   // counter for ringbuffer

// RINGBUFFER RAW LoRa RX
unsigned char ringbufferRAWLoraRX[MAX_LOG][UDP_TX_BUF_SIZE+5] = {0};
int RAWLoRaWrite=0;
int RAWLoRaRead=0;

// RINGBUFFER for outgoing UDP lora packets for lora TX
uint8_t ringBufferUDPout[MAX_RING_UDP][UDP_TX_BUF_SIZE+20];
uint8_t udpWrite=0;
uint8_t udpRead=0;

// RINGBUFFER BLE to phone
unsigned char BLEtoPhoneBuff[MAX_RING][MAX_MSG_LEN_PHONE+5] = {0};
int toPhoneWrite=0;
int toPhoneRead=0;

// RINGBUFFER BLE Commandos to phone
unsigned char BLEComToPhoneBuff[MAX_RING][MAX_MSG_LEN_PHONE+5] = {0};
int ComToPhoneWrite=0;
int ComToPhoneRead=0;

bool hasMsgFromPhone = false;

// LoRa RX/TX sequence control
int cmd_counter = 0;      // ticker dependant on main cycle delay time
bool is_receiving = false;  // flag to store we are receiving a lora packet.
bool tx_is_active = false;  // flag to store we are transmitting  a lora packet.
bool tx_waiting = false;

int isPhoneReady = 0;      // flag we receive from phone when itis ready to receive data

// APP Time OK
bool bPhoneTimeValid = false;

// NTP Date/Time valid
bool bNTPDateTimeValid = false;

// GPS SmartBeaconing variables
unsigned long posinfo_interval = POSINFO_INTERVAL; // check interval
int posinfo_distance = 0;
double posinfo_direction = 0.0;
int posinfo_distance_ring[10] = {0};
int posinfo_ring_write = 0;
double posinfo_lat = 0.0;
double posinfo_lon = 0.0;
double posinfo_last_lat = 0.0;
double posinfo_last_lon = 0.0;
double posinfo_last_direction = 0.0;
uint32_t posinfo_satcount = 0;
int posinfo_hdop = 0;
bool posinfo_fix = false;
bool posinfo_shot=false;
bool pos_shot = false;
bool wx_shot = false;
int no_gps_reset_counter = 0;

// Loop timers
unsigned long posinfo_timer = 0;    // we check periodically to send GPS
unsigned long heyinfo_timer = 0;    // we check periodically to send HEY
unsigned long telemetry_timer = 0;  // we check periodically to send TELEMETRY
unsigned long temphum_timer = 0;    // we check periodically get TEMP/HUM
unsigned long druck_timer = 0;      // we check periodically get AIRPRESURE
unsigned long hb_timer = 0;
unsigned long web_timer = 0;

// Function that gets current epoch time
unsigned long getUnixClock()
{
	struct tm timeinfo;


    timeinfo.tm_year = meshcom_settings.node_date_year - 1900;
    timeinfo.tm_mon = meshcom_settings.node_date_month - 1;
    timeinfo.tm_mday = meshcom_settings.node_date_day;

    timeinfo.tm_hour = meshcom_settings.node_date_hour;
    timeinfo.tm_min = meshcom_settings.node_date_minute;
    timeinfo.tm_sec = meshcom_settings.node_date_second;

    unsigned long ut = (unsigned long)mktime(&timeinfo);
    unsigned long ot = (unsigned long)(meshcom_settings.node_utcoff * 3600.0); // utcoff in sec

    //Serial.printf("Date: %i.%i.%i %i:%i:%i %lu %f %lu\n",timeinfo.tm_year, timeinfo.tm_mon, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, ut, meshcom_settings.node_utcoff, ot);


	return ut - ot;
}

/** @brief Function adding messages into outgoing BLE ringbuffer
 * BLE to PHONE Buffer
 */
void addBLEOutBuffer(uint8_t *buffer, uint16_t len)
{
    if (len > UDP_TX_BUF_SIZE)
        len = UDP_TX_BUF_SIZE-4; // just for safety

    //first two bytes are always the message length
    memcpy(BLEtoPhoneBuff[toPhoneWrite] + 1, buffer, len);

    if(buffer[0] != 'D')
    {
        unsigned long unix_time = getUnixClock();
        
        //Serial.printf("UNIX TME:%lu\n", unix_time);

        uint8_t tbuffer[5];
        tbuffer[0] = (unix_time >> 24) & 0xFF;
        tbuffer[1] = (unix_time >> 16) & 0xFF;
        tbuffer[2] = (unix_time >> 8) & 0xFF;
        tbuffer[3] = (unix_time) & 0xFF;
        memcpy(BLEtoPhoneBuff[toPhoneWrite] + len + 1, tbuffer, 4);

        BLEtoPhoneBuff[toPhoneWrite][0] = len + 4;
    }
    else
        BLEtoPhoneBuff[toPhoneWrite][0] = len;

    if(bBLEDEBUG)
    {
        Serial.printf("<%02X>BLEtoPhone RingBuff added len=%i to element: %u\n", buffer[0], len, toPhoneWrite);
        printBuffer(BLEtoPhoneBuff[toPhoneWrite], len + 1 + 4);
    }

    toPhoneWrite++;
    
    //Serial.printf("toPhoneWrite:%i\n", toPhoneWrite);

    if (toPhoneWrite >= MAX_RING) // if the buffer is full we start at index 0 -> take care of overwriting!
        toPhoneWrite = 0;
}

/** @brief Function adding messages into outgoing BLE ringbuffer
 * BLE to PHONE Buffer
 */
void addBLEComToOutBuffer(uint8_t *buffer, uint16_t len)
{
    if (len > UDP_TX_BUF_SIZE)
        len = UDP_TX_BUF_SIZE-1; // just for safety

    //first two bytes are always the message length
    BLEComToPhoneBuff[ComToPhoneWrite][0] = len;
    memcpy(BLEComToPhoneBuff[ComToPhoneWrite] + 1, buffer, len);

    if(bBLEDEBUG)
    {
        Serial.printf("<%s> BLEComToPhone RingBuff added len=%i to element: %u\n", buffer, len, ComToPhoneWrite);
        //printBuffer(BLEComToPhoneBuff[ComToPhoneWrite], len + 1);
    }

    ComToPhoneWrite++;
    
    //Serial.printf("toPhoneWrite:%i\n", toPhoneWrite);

    if (ComToPhoneWrite >= MAX_RING) // if the buffer is full we start at index 0 -> take care of overwriting!
        ComToPhoneWrite = 0;
}

void addBLECommandBack(char text[UDP_TX_BUF_SIZE])
{
    uint8_t msg_buffer[MAX_MSG_LEN_PHONE];

    struct aprsMessage aprsmsg;

    initAPRS(aprsmsg, ':');

    aprsmsg.msg_len = 0;
    aprsmsg.payload_type = ':';
    aprsmsg.msg_id = millis();
    aprsmsg.msg_destination_call="*";
    aprsmsg.msg_source_path="response";
    aprsmsg.msg_payload=text;

    aprsmsg.msg_app_offline = true; // Rückmeldungen niemals annoucen

    encodeAPRS(msg_buffer, aprsmsg);

    addBLEOutBuffer(msg_buffer, aprsmsg.msg_len);
}

/**@brief Function adding messages into outgoing UDP ringbuffer
 * 
 */
void addLoraRxBuffer(unsigned int msg_id, bool bserver)
{
    // byte 0-3 msg_id
    ringBufferLoraRX[loraWrite][3] = msg_id >> 24;
    ringBufferLoraRX[loraWrite][2] = msg_id >> 16;
    ringBufferLoraRX[loraWrite][1] = msg_id >> 8;
    ringBufferLoraRX[loraWrite][0] = msg_id;

    if(bserver)
        ringBufferLoraRX[loraWrite][4] = 1;
    else
        ringBufferLoraRX[loraWrite][4] = 0;

    loraWrite++;
    if (loraWrite >= MAX_RING) // if the buffer is full we start at index 0 -> take care of overwriting!
        loraWrite = 0;
}

int checkOwnRx(uint8_t compBuffer[4])
{
    for(int ilo=0; ilo<MAX_RING; ilo++)
    {
        if(memcmp(ringBufferLoraRX[ilo], compBuffer, 4) == 0)
            return ilo;
    }

    return -1;
}

bool checkServerRx(uint8_t compBuffer[4])
{
    for(int ilo=0; ilo<MAX_RING; ilo++)
    {
        if(memcmp(ringBufferLoraRX[ilo], compBuffer, 4) == 0)
        {
            // MSG wurde von einem anderen GW gesendet
            if(ringBufferLoraRX[ilo][4] == 1)
                return true;

            break;
        }
    }

    return false;
}

int checkOwnTx(unsigned int msg_id)
{
    if(msg_id == 0)
        return -1;

    unsigned int own_id=0;

    for(int ilo=0; ilo<MAX_RING; ilo++)
    {
        own_id = own_msg_id[ilo][0] | (own_msg_id[ilo][1] << 8) | (own_msg_id[ilo][2] << 16) | (own_msg_id[ilo][3] << 24);

        if(own_id == msg_id)
        {
            if(bDisplayInfo)
            {
                Serial.printf("%s checkOwnTx:%08X own_msg_id:%08X <%02X%02X%02X%02X> %02X\n", getTimeString().c_str(), msg_id, own_id, own_msg_id[ilo][3], own_msg_id[ilo][2], own_msg_id[ilo][1], own_msg_id[ilo][0], own_msg_id[ilo][4]);
            }

            return ilo;
        }
    }

    return -1;
}

void insertOwnTx(unsigned int msg_id)
{
    if(msg_id == 0)
        return;

        // byte 0-3 msg_id
    own_msg_id[iWriteOwn][3] = msg_id >> 24;
    own_msg_id[iWriteOwn][2] = msg_id >> 16;
    own_msg_id[iWriteOwn][1] = msg_id >> 8;
    own_msg_id[iWriteOwn][0] = msg_id;

    own_msg_id[iWriteOwn][4]=0x00;

    if(bDisplayInfo)
    {
        Serial.printf("%s Insert own_msg_id:%08X <%02X%02X%02X%02X>\n", getTimeString().c_str(), msg_id, own_msg_id[iWriteOwn][3], own_msg_id[iWriteOwn][2], own_msg_id[iWriteOwn][1], own_msg_id[iWriteOwn][0]);
    }

    iWriteOwn++;
    if(iWriteOwn >= MAX_RING)
        iWriteOwn=0;
}

int pageLine[7][3] = {0};
char pageText[7][25] = {0};
char pageTextLong1[25] = {0};
char pageTextLong2[200] = {0};
int pageLineAnz=0;

#define PAGE_MAX 6

int pageLastLine[PAGE_MAX][7][3] = {0};
char pageLastText[PAGE_MAX][7][25] = {0};
char pageLastTextLong1[PAGE_MAX][25] = {0};
char pageLastTextLong2[PAGE_MAX][200] = {0};
int pageLastLineAnz[PAGE_MAX] = {0};
int pageLastPointer=0;
int pagePointer=0;
int pageHold=PAGE_MAX-1;

bool bSetDisplay = false;

// detect oled-display type
// see https://github.com/olikraus/u8g2/discussions/2088
int esp32_isSSD1306(int address)
{
    byte buffer[1];

    #ifdef BOARD_HELTEC_V3
        return false;
    #endif
        
    TwoWire *w = NULL;

    w = &Wire;

    w->beginTransmission(address);
    w->write(0x00);
    w->endTransmission(false);
    w->requestFrom(address, 1);

    bool bFound=false;

    if (w->available() > 0)
    {
        w->readBytes(buffer, 1);
        bFound=true;
    }
    w->endTransmission();

    // no display found
    if(!bFound)
    {
        Serial.println("[INIT]...Display not found");
        return -1;
    }

    Serial.printf("[INIT]...Display type: 0x%02X\n", buffer[0]);

    // 0x00 == T-BEAM 1.3" 1306
    // 0x03 == T-BEAM 0.9"
    // 0x28 == E22 1.3" 1306
    // 0x07 == T-LORA 0.9! type 1
    // 0x07 == T-LORA 0.9" type 2
    // 0x09 == HELTEC V3 type 1
    // 0x3F == HELTEC V3 type 2
    // 0x16 == T-BEAM 1.3" 1306
    // 0x48 == T-BEAM 1.3" SUPREME SH1106

    // check 1.3"
    if((buffer[0] & 0x04f) == 0x08 || (buffer[0] & 0x03f) == 0x00 || (buffer[0] & 0x3f) == 0x16)
    {
        Serial.println(F("[INIT]...OLED Display is SSD1306"));
        return 1;
    }

    // state 0,9"
    Serial.println(F("[INIT]...OLED Display is SH1106"));
    return 0;
}

void sendDisplay1306(bool bClear, bool bTransfer, int x, int y, char *text)
{
    #ifndef BOARD_E290
        if(u8g2 == NULL)
            return;
    #endif

	if(bClear || (x == 0 && y== 0) || (x == 0 && memcmp(text, "#F", 2) == 0))
    {
        #ifdef BOARD_E290
            e290_display.clearMemory();

        	if(memcmp(text, "#F", 2) == 0)
                e290_display.clear();
            else
                e290_display.clearMemory();

            e290_display.fastmodeOn();
            
            e290_display.setFont(&FreeMonoBold12pt7b);
        #else
            u8g2->setFont(u8g2_font_6x10_mf);
        #endif

    	if(bClear)
            pageLineAnz=0;
    }

    bool bNeu=true;

	if(memcmp(text, "#N", 2) == 0)
    {
        bNeu=false;
    }
    else
	if(memcmp(text, "#C", 2) == 0)
    {
        bNeu=false;
    }
    else
    {
        if(pageLineAnz < 7)
        {
            //Serial.printf("pageLineAnz:%i text:%s\n", pageLineAnz, text);

            pageLine[pageLineAnz][0] = x;
            pageLine[pageLineAnz][1] = y;
            pageLine[pageLineAnz][2] = 20;
            memcpy(pageText[pageLineAnz], text, 25);
            pageLineAnz++;
            if(pageLineAnz > 7)
                pageLineAnz = 7;
        }
    }

    if(bTransfer)
    {
        //Serial.println("Transfer");
        #ifdef BOARD_E290
            if(pageLineAnz > 0)
            {
                for(int its=0;its<pageLineAnz;its++)
                {
                    // Save last Text (init)
                    if(iDisplayType == 0 && bNeu)
                    {
                        pageLastLineAnz[pageLastPointer] = pageLineAnz;
                        pageLastLine[pageLastPointer][its][0] = pageLine[its][0];
                        pageLastLine[pageLastPointer][its][1] = pageLine[its][1];
                        pageLastLine[pageLastPointer][its][2] = pageLine[its][2];
                        memcpy(pageLastText[pageLastPointer][its], pageText[its], 25);
                    }

                    char ptext[30] = {0};
                    pageText[its][pageLine[its][2]] = 0x00;
                    
                    if(memcmp(pageText[its], "#S", 2) == 0 || memcmp(pageText[its], "#F", 2) == 0)  // #F fastmode off
                    {
                        // only transfer
                        if(!bNeu)
                        {
                            e290_display.setCursor(0, dzeile[1]);
                            e290_display.setFont(&FreeSans9pt7b);

                            e290_display.println(pageTextLong1);

                            e290_display.setCursor(0, dzeile[2]);
                            e290_display.println(pageTextLong2);
                        }
                    }
                    else
                    if(memcmp(pageText[its], "#L", 2) == 0)
                    {
                        e290_display.drawLine(0, 22, 320, 22, BLACK);

                        //u8g2->drawHLine(pageLine[its][0], pageLine[its][1], 120);
                    }
                    else
                    {
                        snprintf(ptext, sizeof(ptext), "%s", pageText[its]);
                        if(pageLine[its][1] >= 0)
                        {
                            e290_display.setCursor(pageLine[its][0], pageLine[its][1]);
                            e290_display.println(ptext);

                            //u8g2->drawUTF8(pageLine[its][0], pageLine[its][1], ptext);
                        }
                    }
                }

            	if(memcmp(text, "#S", 2) != 0 && memcmp(text, "#F", 2) != 0)
                    e290_display.update();
            }
            
        #else
        
        u8g2->firstPage();
        do
        {
            if(pageLineAnz > 0)
            {
                for(int its=0;its<pageLineAnz;its++)
                {
                    // Save last Text (init)
                    if(iDisplayType == 0 && bNeu)
                    {
                        pageLastLineAnz[pageLastPointer] = pageLineAnz;
                        pageLastLine[pageLastPointer][its][0] = pageLine[its][0];
                        pageLastLine[pageLastPointer][its][1] = pageLine[its][1];
                        pageLastLine[pageLastPointer][its][2] = pageLine[its][2];
                        memcpy(pageLastText[pageLastPointer][its], pageText[its], 25);
                    }

                    char ptext[30] = {0};
                    pageText[its][pageLine[its][2]] = 0x00;
                    
                    if(memcmp(pageText[its], "#L", 2) == 0)
                    {
                        u8g2->drawHLine(pageLine[its][0], pageLine[its][1], 120);
                    }
                    else
                    {
                        snprintf(ptext, sizeof(ptext), "%s", pageText[its]);
                        if(pageLine[its][1] >= 0)
                            u8g2->drawUTF8(pageLine[its][0], pageLine[its][1], ptext);
                    }
                }

            }

        } while (u8g2->nextPage());
        
        #endif

        if(iDisplayType == 0 && bNeu)
        {
            pagePointer=pageLastPointer;

            pageLastPointer++;
            if(pageLastPointer >= PAGE_MAX)
                pageLastPointer=0;

            pageLastLineAnz[pageLastPointer] = 0;   // nächsten Ringplatz frei machen
        }
    }
}

void sendDisplayHead(bool bInit)
{
    if((bSetDisplay || pageHold > 0) && !bInit)
        return;

    bSetDisplay=true;

    if(bDisplayIsOff)
    {
        sendDisplay1306(true, true, 0, 0, (char*)"#C");
        bSetDisplay=false;
        return;
    }

    iDisplayType=9;

    char print_text[500];

    sendDisplayMainline();

    snprintf(print_text, sizeof(print_text), "CALL : %s", meshcom_settings.node_call);
    sendDisplay1306(false, false, 3, dzeile[1], print_text);

    snprintf(print_text, sizeof(print_text), "MAC  : %08X", _GW_ID);
    sendDisplay1306(false, false, 3, dzeile[2], print_text);

    snprintf(print_text, sizeof(print_text), "BLE-C: %06i", meshcom_settings.bt_code);
    sendDisplay1306(false, false, 3, dzeile[3], print_text);

    if(bWIFIAP)
        snprintf(print_text, sizeof(print_text), "AP   : %-13.13s", meshcom_settings.node_call);
    else
        snprintf(print_text, sizeof(print_text), "SSID : %-15.15s", meshcom_settings.node_ssid);

    sendDisplay1306(false, false, 3, dzeile[4], print_text);

    snprintf(print_text, sizeof(print_text), "IP%s%s %s", (bGATEWAY?"G":""), (bWEBSERVER?"W":""), meshcom_settings.node_ip);
    sendDisplay1306(false, true, 3, dzeile[5], print_text);

    bSetDisplay=false;
}

void sendDisplayTrack()
{
    if(bSetDisplay)
        return;

    bSetDisplay=true;

    if(bDisplayIsOff)
    {
        sendDisplay1306(true, true, 0, 0, (char*)"#C");
        bSetDisplay=false;
        return;
    }

    iDisplayType=9;

    char print_text[500];

    sendDisplayMainline();

    snprintf(print_text, sizeof(print_text), "LAT : %.4lf %c  %s", meshcom_settings.node_lat, meshcom_settings.node_lat_c, (posinfo_fix?"fix":""));
    sendDisplay1306(false, false, 3, dzeile[1], print_text);

    snprintf(print_text, sizeof(print_text), "LON : %.4lf %c %4i", meshcom_settings.node_lon, meshcom_settings.node_lon_c, (int)posinfo_satcount);
    sendDisplay1306(false, false, 3, dzeile[2], print_text);

    snprintf(print_text, sizeof(print_text), "RATE: %5i sec %4i", (int)posinfo_interval, posinfo_hdop);
    sendDisplay1306(false, false, 3, dzeile[3], print_text);

    snprintf(print_text, sizeof(print_text), "DIST: %5i m", posinfo_distance);
    sendDisplay1306(false, false, 3, dzeile[4], print_text);

    snprintf(print_text, sizeof(print_text), "DIR :old%3i° new%3i°", (int)posinfo_last_direction, (int)posinfo_direction);
    sendDisplay1306(false, true, 3, dzeile[5], print_text);


    bSetDisplay=false;
}

void sendDisplayWX()
{
    if(bSetDisplay)
        return;

    bSetDisplay=true;

    if(bDisplayIsOff)
    {
        sendDisplay1306(true, true, 0, 0, (char*)"#C");
        bSetDisplay=false;
        return;
    }

    iDisplayType=9;

    char print_text[500];

    sendDisplayMainline();

    snprintf(print_text, sizeof(print_text), "TEMP : %.1f °C", meshcom_settings.node_temp);
    sendDisplay1306(false, false, 3, dzeile[1], print_text);

    snprintf(print_text, sizeof(print_text), "HUM  : %.1f %%", meshcom_settings.node_hum);
    sendDisplay1306(false, false, 3, dzeile[2], print_text);

    snprintf(print_text, sizeof(print_text), "QFE  : %.1f hPa", meshcom_settings.node_press);
    sendDisplay1306(false, false, 3, dzeile[3], print_text);

    snprintf(print_text, sizeof(print_text), "QNH  : %.1f hPa", meshcom_settings.node_press_asl);
    sendDisplay1306(false, false, 3, dzeile[4], print_text);

    snprintf(print_text, sizeof(print_text), "ALT:%5im / %5im", meshcom_settings.node_alt, meshcom_settings.node_press_alt);
    sendDisplay1306(false, true, 3, dzeile[5], print_text);


    bSetDisplay=false;
}

void sendDisplayTime()
{
    // Button Page 5 Sekunden halten
    pageHold--;
    if(pageHold < 0)
    {
        pageHold=0;
        pagePointer=pageLastPointer-1;
        if(pagePointer < 0)
            pagePointer=PAGE_MAX-1;
    }

    #ifndef BOARD_E290
        if(u8g2 == NULL)
            return;
    #endif

    #ifdef BOARD_E290
        return;
    #endif

    //TEST ONLY Serial.printf("Time bDisplayOff:%i iDisplayType:%i bSetDisplay:%i\n", bDisplayOff, iDisplayType, bSetDisplay);

    if(bDisplayIsOff)
        return;

    if(iDisplayType == 0)
        return;

    if(bSetDisplay)
        return;

    bSetDisplay = true;

    char print_text[500];
    char cbatt[6];

    if(bDisplayVolt)
        snprintf(cbatt, sizeof(cbatt), "%4.2fV", global_batt/1000.0);
    else
        snprintf(cbatt, sizeof(cbatt), "%4d%%", global_proz);

 #if defined(XPOWERS_CHIP_AXP192)
    if(global_batt == 0.0)
        snprintf(cbatt, sizeof(cbatt), "  USB");
 #endif

 #if defined(XPOWERS_CHIP_AXP2101)
    if(global_batt == 0.0)
        snprintf(cbatt, sizeof(cbatt), "  USB");
 #endif

    snprintf(print_text, sizeof(print_text), "%-4.4s%-1.1s %02i:%02i:%02i %-5.5s", SOURCE_VERSION, SOURCE_VERSION_SUB, meshcom_settings.node_date_hour, meshcom_settings.node_date_minute, meshcom_settings.node_date_second, cbatt);

    memcpy(pageText[0], print_text, 20);
    pageLine[0][0] = 3;
    pageLine[0][1] = dzeile[0];

    #ifndef BOARD_E290
        sendDisplay1306(false, true, 3, dzeile[0], print_text);
    #endif

    bSetDisplay = false;
}

void sendDisplayMainline()
{
    char print_text[500];
    char cbatt[6];
    char nodetype[5];


    if(bDisplayVolt)
        snprintf(cbatt, sizeof(cbatt), "%4.2fV", global_batt/1000.0);
    else
        snprintf(cbatt, sizeof(cbatt), "%4d%%", global_proz);

 #if defined(XPOWERS_CHIP_AXP192)
    if(global_batt == 0.0)
        snprintf(cbatt, sizeof(cbatt), " USB");
 #endif

 #if defined(XPOWERS_CHIP_AXP2101)
    if(global_batt == 0.0)
        snprintf(cbatt, sizeof(cbatt), " USB");
 #endif

 #if defined(BOARD_E290)
    if(global_batt > 4300.0)
        snprintf(cbatt, sizeof(cbatt), " USB");
 #endif

    if(meshcom_settings.node_date_hour == 0 && meshcom_settings.node_date_minute == 0 && meshcom_settings.node_date_second == 0)
    {
        snprintf(print_text, sizeof(print_text), "%-1.1s %-4.4s%-1.1s         %-5.5s", nodetype, SOURCE_VERSION, SOURCE_VERSION_SUB, cbatt);
    }
    else
    {
        snprintf(print_text, sizeof(print_text), "%-4.4s%-1.1s %02i:%02i:%02i %-5.5s", SOURCE_VERSION, SOURCE_VERSION_SUB, meshcom_settings.node_date_hour, meshcom_settings.node_date_minute, meshcom_settings.node_date_second, cbatt);
    }

    sendDisplay1306(true, false, 3, dzeile[0], print_text);
    sendDisplay1306(false, false, 3, dzeile[0]+3, (char*)"#L");
}

void mainStartTimeLoop()
{
    /////////////////////////////////////////////////////////////////////////////////////////////
    // Start-Loop & Time-Loop

    //if(iInitDisplay > 0)
    //    Serial.printf("iInitDisplay %i meshcom_settings.node_date_second %i DisplayTimeWait %i\n", iInitDisplay, meshcom_settings.node_date_second, DisplayTimeWait);

    if(iInitDisplay < 4)
    {
        if(meshcom_settings.node_date_second != DisplayTimeWait)
        {
            iInitDisplay++;

            DisplayTimeWait = meshcom_settings.node_date_second;
        }
    }
    else
    if(iInitDisplay < 8)
    {
        if(iInitDisplay == 4)
        {
            bDisplayIsOff = false;

            sendDisplayHead(true);

            bDisplayIsOff = bDisplayOff;

            DisplayTimeWait=0;
        }

        if(meshcom_settings.node_date_second != DisplayTimeWait)
        {
            iInitDisplay++;

            DisplayTimeWait = meshcom_settings.node_date_second;
        }
    }
    else
        if(iInitDisplay == 8)
        {
            sendDisplayHead(true);

            DisplayTimeWait=0;

            iInitDisplay = 9;
        }
        else
        {
            if (meshcom_settings.node_date_second != DisplayTimeWait)
            {
                if(bBMEON || bBMPON)
                {
                    iDisplayChange++;
                    if(iDisplayChange > 9)
                        iDisplayChange=1;
                }

                if(bDisplayTrack)
                {
                    if(iDisplayChange > 5)
                        sendDisplayWX(); // Show WX
                    else
                        sendDisplayTrack(); // Show Track
                }
                else
                {
                    sendDisplayTime(); // Time only
                }

                #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
                tdeck_update_time_label();
                #endif

                DisplayTimeWait = meshcom_settings.node_date_second;
            }
            else
            {
                bPosDisplay=true;
            }
        }
    // End Start-Loop & Time-Loop
    /////////////////////////////////////////////////////////////////////////////////////////////
}


void sendDisplayText(struct aprsMessage &aprsmsg, int16_t rssi, int8_t snr)
{
    // Fernwirken
    // {MCP}xA0yaazppppp
    // ppppp ... password (PWLFD)
    // S1 ... A0 switch A0-7 B0-7
    // aa ... ON or OF

    if(aprsmsg.msg_payload.startsWith("{MCP}") || aprsmsg.msg_payload.startsWith("{mcp}"))
    {
        char cset[30];
        memset(cset, 0x00, sizeof(cset));

        snprintf(cset, sizeof(cset), "%s", aprsmsg.msg_payload.c_str());
        char cpasswd[6];
        memcpy(cpasswd, cset+5+2+2+3, 5);

        char clfd[10];
        memset(clfd, 0x00, sizeof(clfd));
        snprintf(clfd, sizeof(cset), "%03i", (int)(aprsmsg.msg_id & 0x3FF));

        // check pwd
        bool bpass=true;
        for(int ip=0;ip<5;ip++)
        {
            if(cpasswd[ip] != 0x00 && bpass)
            {
                bool bp=false;
                for(int ic=0;ic<15;ic++)
                {
                    if(meshcom_settings.node_passwd[ic] == cpasswd[ip])
                        bp=true;
                }

                bpass = bp;
            }
        }

        if(bpass)
        {
            if(!(cset[5] == clfd[0] && cset[8] == clfd[1] && cset[11] == clfd[2]))
            {
                bpass=false;
                Serial.printf("[MCP] wrong lfd:%s\n", clfd);
            }
        }

        if(bpass)
        {
            int iswitch = 0;
            bool bON = false;
            if(cset[6] == 'A' || cset[6] == 'B' || cset[6] == 'a' || cset[6] == 'b')
            {
                iswitch=cset[7]-0x30;
                if(iswitch >= 0 and iswitch < 8)
                {
                    if(memcmp(cset+9, "ON", 2) == 0 || memcmp(cset+9, "on", 2) == 0)
                        bON = true;

                    Serial.printf("[MCP] key:%-5.5s command: %c%i %s\n", cpasswd, cset[6], iswitch, (bON?"on":"off"));

                    char cBefehl[30];
                    if(bON)
                        snprintf(cBefehl, sizeof(cBefehl), "--setout %c%i off", cset[6], iswitch);
                    else
                        snprintf(cBefehl, sizeof(cBefehl), "--setout %c%i on", cset[6], iswitch);

                    commandAction(cBefehl, false);
                }
                else
                {
                    Serial.println("[MCP] wrong switch number");
                }
            }
            else
            {
                Serial.println("[MCP] no command recognized");
            }
        }
        else
        {
            Serial.println("[MCP] wrong keyword");
        }

        return;
    }
    else
    if(aprsmsg.msg_payload.startsWith("{SET}") > 0)
    {
        char cset[30];
        snprintf(cset, sizeof(cset), "%s", aprsmsg.msg_payload.c_str());
        sscanf(cset+5, "%d;%d;", &meshcom_settings.max_hop_text, &meshcom_settings.max_hop_pos);

        return;
    }
    else
    if(aprsmsg.msg_payload.startsWith("{CET}") > 0)
    {
        // CET Meldungen nur annehmen wenn nichr GPS, RTC oder Handyverbindung vorhanden ist
        if(!bRTCON && !posinfo_fix && !bNTPDateTimeValid) // !!!! erst aktivieren wenn PHone regelmässig zeit liefert  && !bPhoneTimeValid)
        {
            uint16_t Year=2000;
            uint16_t Month=1;
            uint16_t Day=1;
            uint16_t Hour=0;
            uint16_t Minute=0;
            uint16_t Second=0;

            // {CET}2025-01-31 07:47:40
            Year = (uint16_t)aprsmsg.msg_payload.substring(5, 9).toInt();
            Month = (uint16_t)aprsmsg.msg_payload.substring(10, 12).toInt();
            Day = (uint16_t)aprsmsg.msg_payload.substring(13, 15).toInt();

            Hour = (uint16_t)aprsmsg.msg_payload.substring(16, 18).toInt();
            Minute = (uint16_t)aprsmsg.msg_payload.substring(19, 21).toInt();
            Second = (uint16_t)aprsmsg.msg_payload.substring(22, 24).toInt();

            if(Year > 2023)
            {
                MyClock.setCurrentTime(meshcom_settings.node_utcoff, Year, Month, Day, Hour, Minute, Second);
                snprintf(cTimeSource, sizeof(cTimeSource), (char*)"{CET}");
            }
        }

        return;
    }
    else
    {
        if(!bDisplayVolt)
            bPosDisplay=false;
    }

    if(bSetDisplay || pageHold > 0)
        return;

    bSetDisplay=true;

    ///////////////////////////////////////////////////////////
    // text immer 30 sec stehenlassen und Display immer ON
    DisplayOffWait = millis() + (30 * 1000); // 30 seconds
    bDisplayIsOff=false;
    //
    ///////////////////////////////////////////////////////////
    
    iDisplayType = 0;

    sendDisplayMainline();

    #ifdef BOARD_E290

    sendDisplay1306(false, true, 0, dzeile[0], (char*)"#F");    // not fastmode for CET display

    e290_display.setCursor(0, dzeile[1]);
    e290_display.setFont(&FreeSans9pt7b);

    String strPath = "M* <" + aprsmsg.msg_source_call + ">";
    // DM
    if(CheckGroup(aprsmsg.msg_destination_path))
    {
        strPath = "GM" + aprsmsg.msg_destination_path + " <" + aprsmsg.msg_source_call + ">";
    }
    else
        if(aprsmsg.msg_destination_path != "*")
        {
            strPath = "DM <" + aprsmsg.msg_source_call + ">";
        }

    if(strPath.length() < (20-4))
        snprintf(msg_text, sizeof(msg_text), "%s <%i>", strPath.c_str(), rssi);
    else
        snprintf(msg_text, sizeof(msg_text), "%s", strPath.c_str());

    msg_text[20]=0x00;
    e290_display.println(msg_text);

    e290_display.setCursor(0, dzeile[2]);

    String strAscii = "";//aprsmsg.msg_payload;

    strAscii = utf8ascii(aprsmsg.msg_payload);

    e290_display.println(strAscii);

    strcpy(pageLastTextLong1[pagePointer], msg_text);
    strcpy(pageLastTextLong2[pagePointer], strAscii.c_str());

    e290_display.update();

    #elif defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
    
    tdeck_add_MSG(aprsmsg);
    
    #else
    
    int izeile=0;
    unsigned int itxt=0;

    bool bClear=true;

    char line_text[21];
    char words[100][21]={0};
    unsigned int iwords=0;
    int ipos=0;

    String strPath = aprsmsg.msg_source_path;
    // DM
    if(CheckGroup(aprsmsg.msg_destination_path))
    {
        strPath = "GM" + aprsmsg.msg_destination_path + " <" + aprsmsg.msg_source_call + ">";
    }
    else
        if(aprsmsg.msg_destination_path != "*")
        {
            strPath = "DM <" + aprsmsg.msg_source_call + ">";
        }

    if(aprsmsg.msg_source_path.length() < (20-5))
        snprintf(msg_text, sizeof(msg_text), "%s <%i>", strPath.c_str(), rssi);
    else
        snprintf(msg_text, sizeof(msg_text), "%s", strPath.c_str());

    msg_text[20]=0x00;
    sendDisplay1306(bClear, false, 3, dzeile[izeile], msg_text);

    izeile++;
    bClear=false;

    for(itxt=0; itxt<aprsmsg.msg_payload.length(); itxt++)
    {
        words[iwords][ipos]=aprsmsg.msg_payload.charAt(itxt);

        if(words[iwords][ipos] == ' ')
        {
            words[iwords][ipos]=0x00;

            if(ipos == 0)
            {
                words[iwords][ipos]=0x20;
                words[iwords][ipos+1]=0x00;
            }
            iwords++;
            if(iwords >= 100)
                iwords=99;
            ipos=0;
        }
        else
        if(ipos > 19)
        {
            words[iwords][20]=0x00;
            iwords++;
            if(iwords >= 100)
                iwords=99;
            words[iwords][0]=aprsmsg.msg_payload.charAt(itxt);
            ipos=1;
        }
        else
            ipos++;
    }

    words[iwords][ipos]=0x00;
    iwords++;
    if(iwords >= 100)
    iwords=99;

    memset(line_text, 0x00, 21);
    strcat(line_text, words[0]);

    bool bEnd=false;

    for(itxt=1; itxt<iwords; itxt++)
    {
        if((strlen(line_text) + strlen(words[itxt])) > 19)
        {
            line_text[20]=0x00;
            snprintf(msg_text,sizeof(msg_text), "%s", line_text);

            if(izeile > 5)
                bEnd=true;

            if(bEnd && itxt < iwords)
                snprintf(msg_text, sizeof(msg_text), "%-17.17s...", line_text);

            msg_text[20]=0x00;
            sendDisplay1306(bClear, bEnd, 3, dzeile[izeile], msg_text);

            izeile++;
            
            memset(line_text, 0x00, 21);

            if(izeile > 5)
            {
                break;
            }

            if(itxt+1 == iwords)
                bEnd=true;

            bClear=false;

            strcat(line_text, words[itxt]);

        }
        else
        {
            strcat(line_text, " ");
            strcat(line_text, words[itxt]);
        }
    }

    if(strlen(line_text) > 0)
    {
        if(izeile > 5)
            izeile=5;

        bEnd=true;
        line_text[20]=0x00;
        snprintf(msg_text, sizeof(msg_text), "%s", line_text);
        //Serial.printf("1306-02:%s len:%i izeile:%i\n", msg_text, strlen(msg_text), izeile);
        msg_text[20]=0x00;
        sendDisplay1306(bClear, bEnd, 3, dzeile[izeile], msg_text);
    }

    #endif

    bSetDisplay=false;

}

void init_loop_function()
{
    posinfo_last_direction = 0.0;
    posinfo_satcount = 0;
    posinfo_hdop = 0;
    posinfo_fix = false;
}

// ANALOG
void initAnalogPin()
{
    #if defined (ANALOG_PIN)

    if(bAnalogCheck)
    {
        int ANAGPIO = meshcom_settings.node_analog_pin;
        if(meshcom_settings.node_analog_pin <= 0 || meshcom_settings.node_analog_pin >= 99)
        {
            ANAGPIO = ANALOG_PIN;
            meshcom_settings.node_analog_pin = ANALOG_PIN;
            save_settings();
        }

        pinMode(ANAGPIO, INPUT);

        if(bDEBUG && bDisplayInfo)
        {
            Serial.printf("%s [ANALOG]...GPIO%i SET\n", getTimeString().c_str(), ANAGPIO);
        }
    }
    
    #endif
}

void checkAnalogValue()
{
    #if defined (ANALOG_PIN)

    if(bAnalogCheck)
    {
        int ANAGPIO = meshcom_settings.node_analog_pin;
        if(meshcom_settings.node_analog_pin <= 0 || meshcom_settings.node_analog_pin >= 99)
            ANAGPIO = ANALOG_PIN;

        #if defined(BOARD_E22_S3)
        float raw = (float)(analogReadMilliVolts(ANAGPIO)); // some ESP32 have ADC facto5ry-cal
        #else
        float raw = (float)(analogRead(ANAGPIO));
        #endif

        fAnalogValue = raw  * meshcom_settings.node_analog_faktor;
        
        if(bDEBUG && bDisplayInfo)
        {
            Serial.printf("%s [ANALOG]...GPIO%i %.0f * %.4f = %.2f\n", getTimeString().c_str(), ANAGPIO, raw, meshcom_settings.node_analog_faktor, fAnalogValue);
        }
    }
    else
    {
        fAnalogValue = 0.0;
    }

    #endif
}

// BUTTON
void initButtonPin()
{
    #if defined (BUTTON_PIN)

    bcheckBottonRun = false;

    if(bButtonCheck)
    {
        #if defined (BOARD_E290)
            pinMode(iButtonPin, INPUT); // pullup placed on hardware
        #elif defined (BOARD_RAK4630)
            pinMode(BUTTON_PIN, INPUT_PULLUP);
        #else
            pinMode(iButtonPin, INPUT_PULLUP);
        #endif
    }
    
    #endif
}

int checkButtoExtraLong = 0;
int checkButtonTime = 0;
int iPress = 0;
bool bPressed=false;
bool bShowHead=false;

void checkButtonState()
{
    #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
    
    button.check();
    lv_task_handler();

    #elif defined(BUTTON_PIN)

    if(bcheckBottonRun)
        return;
   
    bcheckBottonRun = true;

    if(bButtonCheck)
    {
        #if defined (BOARD_RAK4630)
            if(digitalRead(BUTTON_PIN) == LOW)
        #else
            if(digitalRead(iButtonPin) == LOW)
        #endif
        {
            /* kann wieder raus das der GPIO-PIN PULLUP ist
            checkButtoExtraLong++;
            if(checkButtoExtraLong > 100)
            {
                checkButtoExtraLong=0;
                
                bButtonCheck=false;
                meshcom_settings.node_sset = meshcom_settings.node_sset & 0x7FEF;
                save_settings();
                
                Serial.println("BUTTON not connected (set BUTTON to off)");
                bcheckBottonRun = false;
                return;
            }
            */

            //if(bDEBUG)
            //    Serial.printf("Button Pressed pageLastPointer:%i pageLastLineAnz[%i]:%i Track:%i\n", pageLastPointer, pagePointer, pageLastLineAnz[pagePointer], bDisplayTrack);

            if(!bPressed)
            {
                bPressed = true;

                if(iPress < 3)
                    iPress++;

                checkButtonTime = 30;

                if(bDisplayCont)
                    Serial.printf("1:checkButtonTime:%i iPress:%i\n", checkButtonTime, iPress);
            }

            bcheckBottonRun = false;
            return;
        }
        else
        {

            checkButtoExtraLong = 0;

            bPressed = false;

            checkButtonTime--;

            if(checkButtonTime < 0)
            {
                if(iPress == 3)
                {
                    if(bDisplayCont)
                        Serial.println("BUTTON triple press");

                    bDisplayTrack=!bDisplayTrack;

                    bDisplayIsOff=false;

                    if(bDisplayTrack)
                        commandAction((char*)"--track on", false);
                    else
                        commandAction((char*)"--track off", false);

                    sendDisplayHead(false);

                }
                else
                if(iPress == 2)
                {
                    if(bDisplayCont)
                        Serial.println("BUTTON double press");

                    if(bDisplayTrack)
                        commandAction((char*)"--sendtrack", false);
                    else
                        commandAction((char*)"--sendpos", false);
                }
                else
                if(iPress == 1 && !bDisplayTrack)
                {
                    if(bDisplayCont)
                        Serial.printf("BUTTON singel press %i %i\n", pageLastLineAnz[pagePointer], bDisplayTrack);

                    if(pageLastLineAnz[pagePointer] == 0 || bShowHead)
                    {
                        if(!bShowHead)
                        {
                            pagePointer = pageLastPointer - 1;
                            if(pagePointer < 0)
                                pagePointer = PAGE_MAX - 1;

                            if(bDisplayCont)
                                Serial.printf("BUTTON singel press bShowHead %i bDisplayIsOff:%i\n", pagePointer, bDisplayIsOff);

                            sendDisplayHead(true);
                            bShowHead=true;
                        }
                        else
                        {
                            bShowHead=false;

                            #ifdef BOARD_E290
                                sendDisplayMainline();
                                e290_display.update();
                            #else
                                pageHold=0;
                                bDisplayOff=!bDisplayOff;

                                if(bDisplayOff)
                                {
                                    commandAction((char*)"--display off", isPhoneReady, false);
                                }
                                else
                                {
                                    commandAction((char*)"--display on", isPhoneReady, false);
                                }

                            #endif
                        }
                    }
                    else
                    {
                        bDisplayIsOff=false;

                        pageLineAnz = pageLastLineAnz[pagePointer];
                        for(int its=0;its<pageLineAnz;its++)
                        {
                            // Save last Text (init)
                            pageLine[its][0] = pageLastLine[pagePointer][its][0];
                            pageLine[its][1] = pageLastLine[pagePointer][its][1];
                            pageLine[its][2] = pageLastLine[pagePointer][its][2];
                            memcpy(pageText[its], pageLastText[pagePointer][its], 25);
                            if(its == 0)
                            {
                                for(int iss=0; iss < 20; iss++)
                                {
                                    if(pageText[its][iss] == 0x00)
                                        pageText[its][iss] = 0x20;
                                }
                                pageText[its][19] = pagePointer | 0x30;
                                pageText[its][20] = 0x00;
                            }
                        }

                        #ifdef BOARD_E290
                            iDisplayType=9;
                        #else
                            iDisplayType=0;
                        #endif

                        strcpy(pageTextLong1, pageLastTextLong1[pagePointer]);
                        strcpy(pageTextLong2, pageLastTextLong2[pagePointer]);

                        sendDisplay1306(false, true, 0, 0, (char*)"#N");

                        pagePointer--;
                        if(pagePointer < 0)
                            pagePointer=PAGE_MAX-1;

                        pageHold=5;
                    }
                }

                checkButtonTime = 0;

                iPress = 0;
            }
        }
    }

    bcheckBottonRun = false;
    #endif
}

void sendDisplayPosition(struct aprsMessage &aprsmsg, int16_t rssi, int8_t snr)
{
    //Serial.printf("bPosDisplay:%i bSetDisplay:%i pageHold:%i bDisplayTrack:%i rssi:%d snr:%d\n", bPosDisplay, bSetDisplay, pageHold, bDisplayTrack, rssi, snr);

    if(!bPosDisplay)
        return;

    // wenn Textmeldung angezeigt wird dann warten bis Display off
    if(DisplayOffWait > 0)
        return;

    if(bSetDisplay || pageHold > 0 || bDisplayTrack)
        return;

    bSetDisplay=true;

    if(bDisplayIsOff)
    {
        sendDisplay1306(true, true, 0, 0, (char*)"#C");
        bSetDisplay=false;
        return;
    }


    iDisplayType=1;

    char print_text[500];
    int ipt=0;

    int izeile=1;

    unsigned int itxt=0;
    int istarttext=0;

    double lat=0.0;
    double lon=0.0;

    float d_dir_to = 0;
    int dir_to=0;
    int dist_to=0;

    aprsPosition aprspos;

    initAPRSPOS(aprspos);

    decodeAPRSPOS(aprsmsg.msg_payload, aprspos);

    // Display Distance, Direction
    lat = conv_coord_to_dec(aprspos.lat);
    lon = conv_coord_to_dec(aprspos.lon);

    d_dir_to = tinyGPSPLus.courseTo(meshcom_settings.node_lat, meshcom_settings.node_lon, lat, lon);
    dir_to = d_dir_to;

    dist_to = tinyGPSPLus.distanceBetween(lat, lon, meshcom_settings.node_lat, meshcom_settings.node_lon)/1000.0;

    sendDisplayMainline();

    #ifdef BOARD_E290
        sendDisplay1306(false, true, 0, dzeile[0], (char*)"#S");
    #endif

    snprintf(msg_text, sizeof(msg_text), "%s<>%s", aprsmsg.msg_source_call.c_str(), aprsmsg.msg_source_last.c_str());


    msg_text[20]=0x00;
    sendDisplay1306(false, false, 3, dzeile[izeile], msg_text);

    izeile=izeile+1;

    ipt=0;

    for(itxt=0; itxt<aprsmsg.msg_payload.length(); itxt++)
    {
        if((aprsmsg.msg_payload.charAt(itxt) == 'N' || aprsmsg.msg_payload.charAt(itxt) == 'S'))
        {
            print_text[ipt]=0x00;

            sscanf(print_text, "%lf", &lat);

            snprintf(msg_text, sizeof(msg_text), "LAT: %s%c%5ikm", print_text, aprsmsg.msg_payload.charAt(itxt), dist_to);
            msg_text[20]=0x00;
            sendDisplay1306(false, false, 3, dzeile[izeile], msg_text);

            istarttext=itxt+2;
            izeile=izeile+1;
            break;
        }
        else
        {
            print_text[ipt]=aprsmsg.msg_payload.charAt(itxt);
            ipt++;
        }
    }

    ipt=0;

    for(itxt=istarttext; itxt<aprsmsg.msg_payload.length(); itxt++)
    {
        if((aprsmsg.msg_payload.charAt(itxt) == 'W' || aprsmsg.msg_payload.charAt(itxt) == 'E'))
        {
            print_text[ipt]=0x00;

            sscanf(print_text, "%lf", &lon);

            char cdir_to[3];
            float von_dir_to[10]     = {0.0, 22.5, 67.5, 112.5, 157.5, 202.5, 247.5, 292.5, 337.5, 360.0};
            char von_c_dir_to[10][4] = {"",  "N",  "NE",  "E",  "SE",  "S",   "SW",   "W",   "NW",  "N"} ;

            for(int idir=0; idir<9; idir++)
            {
                if(dir_to >= von_dir_to[idir] && dir_to <= von_dir_to[idir+1])
                    snprintf(cdir_to, sizeof(cdir_to), "%s", von_c_dir_to[idir+1]);
            }

            snprintf(msg_text, sizeof(msg_text), "LON:%s%c%5i%s", print_text, aprsmsg.msg_payload.charAt(itxt), dir_to, cdir_to);
            msg_text[20]=0x00;
            sendDisplay1306(false, false, 3, dzeile[izeile], msg_text);

            istarttext=itxt+3;
            izeile=izeile+1;
            break;
        }
        else
        {
            print_text[ipt]=aprsmsg.msg_payload.charAt(itxt);
            ipt++;
        }
    }

    int bat = 0;
    int alt = 0;

    char decode_text[20];

    memset(decode_text, 0x00, sizeof(decode_text));
    ipt=0;

    // check Batt
    for(itxt=istarttext; itxt<=aprsmsg.msg_payload.length(); itxt++)
    {
        if(aprsmsg.msg_payload.charAt(itxt) == '/' && aprsmsg.msg_payload.charAt(itxt+1) == 'B' && aprsmsg.msg_payload.charAt(itxt+2) == '=')
        {
            for(unsigned int id=itxt+3;id<=aprsmsg.msg_payload.length();id++)
            {
                // ENDE
                if(aprsmsg.msg_payload.charAt(id) == '/' || aprsmsg.msg_payload.charAt(id) == ' ' || id == aprsmsg.msg_payload.length())
                {
                    sscanf(decode_text, "%d", &bat);
                    break;
                }

                decode_text[ipt]=aprsmsg.msg_payload.charAt(id);
                ipt++;
            }

            break;
        }
    }

    memset(decode_text, 0x00, sizeof(decode_text));
    ipt=0;

    #ifdef BOARD_E290

    // dir_to
    e290_display.drawCircle(275, 107,
        20,                             // Radius: 10px
        BLACK                           // Color: black
        );

    // dir_to
    e290_display.drawCircle(275, 107,
        19,                             // Radius: 10px
        BLACK                           // Color: black
        );

    DrawDirection(d_dir_to, 275, 107, 20);
    DrawDirection(d_dir_to, 274, 108, 20);


    #endif

    for(itxt=istarttext; itxt<=aprsmsg.msg_payload.length(); itxt++)
    {
        if(aprsmsg.msg_payload.charAt(itxt) == '/' && aprsmsg.msg_payload.charAt(itxt+1) == 'A' && aprsmsg.msg_payload.charAt(itxt+2) == '=')
        {
            for(unsigned int id=itxt+3;id<=aprsmsg.msg_payload.length();id++)
            {
                // ENDE
                if(aprsmsg.msg_payload.charAt(id) == '/' || aprsmsg.msg_payload.charAt(id) == ' ' || id == aprsmsg.msg_payload.length())
                {
                    sscanf(decode_text, "%d", &alt);

                    if(aprsmsg.msg_source_fw_version > 13)
                        alt = (int)((float)alt * 0.3048);

                    #ifdef BOARD_E290
                        DrawRssi(3, 117, rssi);

                        snprintf(msg_text, sizeof(msg_text), "ALT:%im", alt);
                        msg_text[20]=0x00;
                        sendDisplay1306(false, true, 3, dzeile[izeile], msg_text);
                    #else
                        snprintf(msg_text, sizeof(msg_text), "ALT:%im rssi:%i", alt, rssi);
                        msg_text[20]=0x00;
                        sendDisplay1306(false, true, 3, dzeile[izeile], msg_text);
                    #endif

                    break;
                }

                decode_text[ipt]=aprsmsg.msg_payload.charAt(id);
                ipt++;
            }

            break;
        }
    }
    
    bSetDisplay=false;

}

#ifdef BOARD_E290
void DrawDirection(float angle, int cx, int cy, int radius)
{
    float displacedAngle = angle - 90;
    int x = cx + ((float)cos(degreesToRadians(displacedAngle)) * (radius - 3)); //convert angle to radians for x and y coordinates
    int y = cy + ((float)sin(degreesToRadians(displacedAngle)) * (radius - 3));
    
    e290_display.drawLine(cx, cy, x, y, BLACK); //draw a line from center point back to the point
}

double degreesToRadians(double degrees)
{
    return (degrees * PI) / 180;
}

void DrawRssi(int cx, int cy, int16_t rssi)
{
    int irssi = (110 + rssi) * 2;
    
    //Serial.printf("rssi:%i irssi:%i\n", rssi, irssi);

    e290_display.fillRect(cx + 60, cy, irssi, 5, BLACK);

    e290_display.setCursor(cx, cy + 10);
    e290_display.printf("%i", rssi);
}
#endif

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

void printAsciiBuffer(uint8_t *buffer, int len)
{
    if(buffer[0] != 0x21 && buffer[0] != 0x3A && buffer[0] != 0x40 && buffer[0] != 0x41)
    {
        Serial.printf("LoRa starting with 0x%02X and %02X%02X%02X ... no decode\n", buffer[0], buffer[1], buffer[2], buffer[3]);
        return;
    }

    for (int i = 0; i < len; i++)
    {
        if(buffer[i] == 0x00)
            Serial.printf("#");
        else
            Serial.printf("%c", buffer[i]);
    }

    Serial.println("");
}

String getDateString()
{
    char currDate[11] = {0};
    snprintf(currDate, sizeof(currDate), "%04i-%02i-%02i", meshcom_settings.node_date_year, meshcom_settings.node_date_month, meshcom_settings.node_date_day);

    return (String)currDate;
}

String getTimeString()
{
    char currTime[10] = {0};
    snprintf(currTime, sizeof(currTime), "%02i:%02i:%02i", meshcom_settings.node_date_hour, meshcom_settings.node_date_minute, meshcom_settings.node_date_second);

    return (String)currTime;
}

String charBuffer_aprs(char *msgSource, struct aprsMessage &aprsmsg)
{
    char internal_message[UDP_TX_BUF_SIZE];

    int ilpayload=aprsmsg.msg_payload.length();
    if(ilpayload > 60)
        ilpayload=60;

    //snprintf(internal_message, sizeof(internal_message), "%s %s:%08X %02X %i %i %i HW:%02i CS:%04X FW:%02i:%c LH:%02X %s>%s %c%s",  msgSource, getTimeString().c_str(),
    snprintf(internal_message, sizeof(internal_message), "%s %s:%08X %1u %i%i%i %01X/%1u LH:%02X %s>%s %c%s",  getTimeString().c_str(), msgSource,
        aprsmsg.msg_id, aprsmsg.max_hop,aprsmsg.msg_server, aprsmsg.msg_track, aprsmsg.msg_mesh, (aprsmsg.msg_source_mod>>4), (aprsmsg.msg_source_mod & 0xf), aprsmsg.msg_last_hw,
        //aprsmsg.msg_source_hw, aprsmsg.msg_fcs, aprsmsg.msg_source_fw_version, aprsmsg.msg_source_fw_sub_version, aprsmsg.msg_last_hw,
        aprsmsg.msg_source_path.c_str(), aprsmsg.msg_destination_path.c_str(),
        aprsmsg.payload_type, aprsmsg.msg_payload.substring(0, ilpayload).c_str());

    
    internal_message[UDP_TX_BUF_SIZE-1]=0x00;
    
    return (String)internal_message;
}

void printBuffer_aprs(char *msgSource, struct aprsMessage &aprsmsg)
{
    Serial.printf("%s %s: %03i %c x%08X H%02X S%i T%i M%02X %s>%s%c%s HW:%02i MOD:%01X/%01i FCS:%04X FW:%02i:%c LH:%02X", getTimeString().c_str(), msgSource, aprsmsg.msg_len, aprsmsg.payload_type, aprsmsg.msg_id, aprsmsg.max_hop,
        aprsmsg.msg_server, aprsmsg.msg_track, aprsmsg.msg_mesh, aprsmsg.msg_source_path.c_str(), aprsmsg.msg_destination_path.c_str(), aprsmsg.payload_type, aprsmsg.msg_payload.c_str(),
        aprsmsg.msg_source_hw, (aprsmsg.msg_source_mod>>4), (aprsmsg.msg_source_mod & 0xf), aprsmsg.msg_fcs, aprsmsg.msg_source_fw_version, aprsmsg.msg_source_fw_sub_version, aprsmsg.msg_last_hw);
}

void printBuffer_ack(char *msgSource, uint8_t payload[UDP_TX_BUF_SIZE+10], int8_t size)
{
    if(size == 7)
        Serial.printf("%s %s: %02X %02X%02X%02X%02X %02X %02X", getTimeString().c_str(), msgSource, payload[0], payload[4], payload[3], payload[2], payload[1], payload[5], payload[6]);
    else
        Serial.printf("%s %s: %02X %02X%02X%02X%02X %02X %02X%02X%02X%02X %02X %02X", getTimeString().c_str(), msgSource, payload[0], payload[4], payload[3], payload[2], payload[1], payload[5], payload[9], payload[8], payload[7], payload[6], payload[10], payload[11]);
}


///////////////////////////////////////////////////////////////////////////
// APRS Meldungen

void sendMessage(char *msg_text, int len)
{
    if(len < 1 || len > 160)
    {
        Serial.printf("sendMessage wrong text length:%i\n", len);
        return;
    }

    if(memcmp(msg_text, "-", 1) == 0)
    {
        if(bDisplayInfo)
            Serial.printf("COMMAND:%s\n", msg_text);

        commandAction(msg_text, false);
        return;
    }

    uint8_t ispos = 0;

    if(msg_text[0] == ':')
    {
        ispos=1;
    }

    String strDestinationCall = "*";
    String strMsg = msg_text+ispos;
    
    bool bDM=false;

    if(strMsg.charAt(0) == '{')
    {
        int iCall = strMsg.indexOf('}');
        if(iCall != -1 && iCall < 11)   // {OE1KBC-99}Textmessage or Group-Call {2321}
        {
            strDestinationCall = strMsg.substring(1, iCall);
            strDestinationCall.toUpperCase();
            strDestinationCall.trim();
            strMsg = strMsg.substring(iCall+1);

            if(strMsg.startsWith("{mcp}") || strMsg.startsWith("{MCP}")) // Fernwirken
            {
                char cId[4] = {0};
                snprintf(cId, sizeof(cId), "%03i", meshcom_settings.node_msgid);
                String newMsg = "{mcp}";
                newMsg.concat(cId[0]);
                newMsg.concat(strMsg.substring(5, 7).c_str());
                newMsg.concat(cId[1]);
                newMsg.concat(strMsg.substring(7, 9).c_str());
                newMsg.concat(cId[2]);
                newMsg.concat(strMsg.substring(9).c_str());

                strMsg = newMsg;
            }
            else
                if(CheckGroup(strDestinationCall) == 0 && strDestinationCall != "*" && strDestinationCall != "WLNK-1" && strDestinationCall != "APRS2SOTA") // no Group Call or WLNK-1 Call
                    bDM=true;
        }

        // check no message to own-call
        if(strDestinationCall.compareTo(meshcom_settings.node_call) == 0)
        {
            Serial.println("[ERROR]...DM to own-all not allowed");
            return;
        }
    }

    uint8_t msg_buffer[MAX_MSG_LEN_PHONE];

    struct aprsMessage aprsmsg;

    initAPRS(aprsmsg, ':');

    aprsmsg.msg_len = 0;

    // MSG ID zusammen setzen    
    aprsmsg.msg_id = ((_GW_ID & 0x3FFFFF) << 10) | (meshcom_settings.node_msgid & 0x3FF);   // MAC-address + 3FF = 1023 max in real only 0-999
    
    aprsmsg.msg_source_path = meshcom_settings.node_call;
    aprsmsg.msg_destination_path = strDestinationCall;  //Later FW insert PATH from HEY! collecting
    aprsmsg.msg_payload = strMsg;

    
    // ACK add request only DM Calls
    if(bDM)
    {
        char cAckId[4] = {0};
        snprintf(cAckId, sizeof(cAckId), "%03i", meshcom_settings.node_msgid);
        aprsmsg.msg_payload = strMsg + "{" + String(cAckId);
    }

    meshcom_settings.node_msgid++;
    if(meshcom_settings.node_msgid > 999)
        meshcom_settings.node_msgid=0;

    // Flash rewrite
    save_settings();

    encodeAPRS(msg_buffer, aprsmsg);

    if(bDisplayInfo)
    {
        printBuffer_aprs((char*)"NEW-TXT", aprsmsg);
        Serial.println();
    }

    // An APP als Anzeige retour senden
    if(hasMsgFromPhone)
    {
        addBLEOutBuffer(msg_buffer, aprsmsg.msg_len);

        if(bGATEWAY && meshcom_settings.node_hasIPaddress)
        {
            // set Info message send and Server reached, not on DM
            if(!bDM && (aprsmsg.msg_destination_path == "*" || CheckGroup(strDestinationCall)))
            {
                uint8_t print_buff[8];

                print_buff[0]=0x41;
                print_buff[1]=aprsmsg.msg_id & 0xFF;
                print_buff[2]=(aprsmsg.msg_id >> 8) & 0xFF;
                print_buff[3]=(aprsmsg.msg_id >> 16) & 0xFF;
                print_buff[4]=(aprsmsg.msg_id >> 24) & 0xFF;
                print_buff[5]=0x01;     // 0x01 ... server reached
                print_buff[6]=0x00;     // msg always 0x00 at the end
                
                addBLEOutBuffer(print_buff, (uint16_t)7);
            }
        }

    }

    #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
    tdeck_add_MSG(aprsmsg);
    #endif
    
    // store last message to compare later on
    insertOwnTx(aprsmsg.msg_id);

    if(bGATEWAY && meshcom_settings.node_hasIPaddress)
        addLoraRxBuffer(aprsmsg.msg_id, true);
    else
        addLoraRxBuffer(aprsmsg.msg_id, false);

    // Master RingBuffer for transmission
    // local messages send to LoRa TX
    ringBuffer[iWrite][0]=aprsmsg.msg_len;
    memcpy(ringBuffer[iWrite]+2, msg_buffer, aprsmsg.msg_len);
    
    if (ringBuffer[iWrite][2] == 0x3A) // only Messages
    {
        if(aprsmsg.msg_payload.startsWith("{") > 0)
            ringBuffer[iWrite][1] = 0xFF; // retransmission Status ...0xFF no retransmission on {CET} & Co.
        else
            ringBuffer[iWrite][1] = 0x00; // retransmission Status ...0xFF no retransmission
    }
    else
    {
        ringBuffer[iWrite][1] = 0xFF; // retransmission Status ...0xFF no retransmission
    }   

    if(bDisplayRetx)
    {
        unsigned int ring_msg_id = (ringBuffer[iWrite][6]<<24) | (ringBuffer[iWrite][5]<<16) | (ringBuffer[iWrite][4]<<8) | ringBuffer[iWrite][3];
        Serial.printf("einfügen retid:%i status:%02X lng;%02X msg-id: %c-%08X\n", iWrite, ringBuffer[iWrite][1], ringBuffer[iWrite][0], ringBuffer[iWrite][2], ring_msg_id);
    }

    iWrite++;
    if(iWrite >= MAX_RING)
        iWrite=0;
    
    if(bGATEWAY && meshcom_settings.node_hasIPaddress)
    {
	    // UDP out
		addNodeData(msg_buffer, aprsmsg.msg_len, 0, 0);
    }

    // Extern Server
    if(bEXTUDP)
        sendExtern(true, (char*)"node", msg_buffer, aprsmsg.msg_len);
}

String PositionToAPRS(bool bConvPos, bool bWeather, bool bFuss, double plat, char lat_c, double plon, char lon_c, int alt,  float press, float hum, float temp, float temp2, float gasres, float co2, int qfe, float qnh)
{
    double lat=plat;
    if(plat < 0.0)
        lat = plat * -1.0;

    double lon=plon;
    if(plon < 0.0)
        lon = plon * -1.0;


    if(lat == 0.0 or lon == 0.0)
    {
        DEBUG_MSG("APRS", "Error PositionToAPRS");
        return "";
    }

    char msg_start[150] = {0};

    // :|0x11223344|0x05|OE1KBC|>*:Hallo Mike, ich versuche eine APRS Meldung\0x00
    // 09:30:28 RX-LoRa: 105 ! xAE48E347 05 1 0 9V1LH-1,OE1KBC-12>*!0122.64N/10356.51E#/B=005/A=000272/P=1005.1/H=42.5/T=29.4/Q=1005.7/R=232;2321; HW:04 MOD:03 FCS:15DC FW:17 LH:09

	double slat = 100.0;
    slat = lat*slat;
	double slon = 100.0;
    slon=lon*slon;
	
    if(bConvPos)
    {
        double slatr=60.0;
        double slonr=60.0;
        
        slat = (int)lat;
        slatr = (lat - slat) * slatr;
        slat = (slat * 100.) + slatr;
        
        slon = (int)lon;
        slonr = (lon - slon) * slonr;
        slon = (slon * 100.) + slonr;
    }

    if(lon_c != 'W' && lon_c != 'E')
        lon_c = 'E';

    if(lat_c != 'N' && lat_c != 'S')
        lat_c = 'N';

    if(bWeather)
        snprintf(msg_start, sizeof(msg_start), "%02i%02i%02iz%07.2lf%c%c%08.2lf%c_", meshcom_settings.node_date_day, meshcom_settings.node_date_hour, meshcom_settings.node_date_minute, slat, lat_c, meshcom_settings.node_symid, slon, lon_c);
    else
    {
        char cversion[5]={0};

        char catxt[50]={0};
        char cname[50]={0};
        char cbatt[15]={0};
        char calt[15]={0};
        char cpress[15]={0};
        char chum[15]={0};
        char ctemp[15]={0};
        char ctemp2[15]={0};
        char cqfe[15]={0};
        char cqnh[15]={0};
        char cgasres[15]={0};
        char cco2[15]={0};
        char cgrc[50]={0};

        char csfpegel[15]={0};
        char csfpegel2[15]={0};
        char csftemp[15]={0};
        char csfbatt[15]={0};

        char cinaU[15]={0};
        char cinaI[15]={0};
        
        if(strcmp(meshcom_settings.node_atxt, "none") != 0 && meshcom_settings.node_atxt[0] != 0x00)
        {
            snprintf(catxt,  sizeof(catxt), "%s", meshcom_settings.node_atxt);
        }

        if(strcmp(meshcom_settings.node_name, "none") != 0 && meshcom_settings.node_name[0] != 0x00)
        {
            snprintf(cname,  sizeof(cname), "#%s", meshcom_settings.node_name);
        }

        // send INA226 Werte abwechselnd zu normal Pos
        //////////
        // TEST
        //bINA226ON=true;
        //meshcom_settings.node_vbus=meshcom_settings.node_vbus+1.0;
        //meshcom_settings.node_vcurrent=meshcom_settings.node_vcurrent+1.0;
        //////////

        if(strlen(meshcom_settings.node_parm) != 0 && strcmp(meshcom_settings.node_parm, "none") != 0)
        {
            // keine automatischen Werte
        }
        else
        if(bINA226ON)
        {
            snprintf(cversion, sizeof(cversion), "%s", "/V=5");
            if(alt > 0)
            {
                // auf Fuss umrechnen
                if(bFuss)
                    snprintf(calt, sizeof(calt), "/A=%06i", conv_fuss(alt));
                else
                    snprintf(calt, sizeof(calt), "/A=%05i", alt);
            }
            snprintf(cbatt, sizeof(cbatt), "/B=%i", global_proz);
            snprintf(cinaU, sizeof(cinaU), "/U=%.2f", meshcom_settings.node_vbus);
            snprintf(cinaI, sizeof(cinaI), "/I=%.1f", meshcom_settings.node_vcurrent);

            //////////
            // TEST
            //bINA226ON=false;
            //////////
        }
        else
        {
            if(global_proz > 0)
            {
                snprintf(cbatt, sizeof(cbatt), "/B=%03d", global_proz);
            }

            if(alt > 0)
            {
                // auf Fuss umrechnen
                if(bFuss)
                    snprintf(calt, sizeof(calt), "/A=%06i", conv_fuss(alt));
                else
                    snprintf(calt, sizeof(calt), "/A=%05i", alt);
            }

            if(press > 0)
            {
                snprintf(cpress, sizeof(cpress), "/P=%.1f", press);
                if(memcmp(cpress, "/P=nan", 6) == 0)
                    return "";
            }

            if(hum > 0)
            {
                snprintf(chum, sizeof(chum), "/H=%.1f", hum);
                if(memcmp(cpress, "/H=nan", 6) == 0)
                    return "";
            }

            if(temp != 0)
            {
                snprintf(ctemp, sizeof(ctemp), "/T=%.1f", temp);
                if(memcmp(cpress, "/T=nan", 6) == 0)
                    return "";
            }

            if(temp2 != 0)
            {
                snprintf(ctemp2, sizeof(ctemp2), "/O=%.1f", temp2);
                if(memcmp(cpress, "/O=nan", 6) == 0)
                    return "";
            }

            if(qfe > 0)
            {
                snprintf(cqfe, sizeof(cqfe), "/F=%i", qfe);
                if(memcmp(cpress, "/F=nan", 6) == 0)
                    return "";
            }

            if(qnh > 0 && !bMCU811ON && !bBME680ON)
            {
                snprintf(cqnh, sizeof(cqnh), "/Q=%.1f", qnh);
                if(memcmp(cpress, "/Q=nan", 6) == 0)
                    return "";
            }

            if(gasres > 0 && bBME680ON)
            {
                snprintf(cversion, sizeof(cversion), "%s", "/V=3");

                snprintf(cgasres, sizeof(cgasres), "/G=%.1f", gasres);
                if(memcmp(cpress, "/G=nan", 6) == 0)
                    return "";
            }

            if(co2 > 0 && bMCU811ON)
            {
                snprintf(cversion, sizeof(cversion),  "%s", "/V=2");

                snprintf(cco2, sizeof(cco2), "/C=%.0f", co2);
                if(memcmp(cpress, "/C=nan", 6) == 0)
                    return "";
            }
        }

        /////////////////////////////////////////////////////////////////
        // send Group-Call settings zu MesCom-Server
        String strGRC="";

        char cGC[8];
        for(int igrc=0;igrc<6;igrc++)
        {
            if(meshcom_settings.node_gcb[igrc] > 0 && meshcom_settings.node_gcb[igrc] < 100000)
            {
                snprintf(cGC, sizeof(cGC), "%i;", meshcom_settings.node_gcb[igrc]);
                strGRC.concat(cGC);
            }
        }

        if(strGRC.length() > 0)
        {
            snprintf(cgrc, sizeof(cgrc), "/R=%s", strGRC.c_str());
        }
        //
        /////////////////////////////////////////////////////////////////

        snprintf(msg_start, sizeof(msg_start), "%07.2lf%c%c%08.2lf%c%c%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s", slat, lat_c, meshcom_settings.node_symid, slon, lon_c, meshcom_settings.node_symcd, catxt, cname, cbatt, calt, cpress, chum, ctemp, ctemp2, cqfe, cqnh, cgasres, cco2, cgrc, csfpegel, csfpegel2, csftemp, csfbatt, cversion, cinaU, cinaI);
    }

    return String(msg_start);
}

void sendPosition(unsigned int intervall, double lat, char lat_c, double lon, char lon_c, int alt, float press, float hum, float temp, float temp2, float gasres, float co2, int qfe, float qnh)
{
    uint8_t msg_buffer[MAX_MSG_LEN_PHONE];

    bool bSendViaAPRS = bDisplayTrack;
    bool bSendViaMesh = !bDisplayTrack;

    if(lastHeardTime + 15000 < millis() && (intervall == POSINFO_INTERVAL || intervall == 0)) // wenn die letzte gehörte LoRa-Nachricht < 5sec dann auch via MeshCom
    {
        bSendViaMesh = true;

        // Zeitmessung
        posfixinterall = millis();
    }

    if(((posfixinterall + (POSINFO_INTERVAL * 1000)) < millis()))
    {
        bSendViaMesh = true;

        // Zeitmessung
        posfixinterall = millis();
    }

    #ifdef BOARD_TLORA_OLV216
        bSendViaAPRS = false;
    #endif

    // set default
    // Symbol Table / \ 0-9 A-Z  (compressed a-z)
    bool bSymbolTable = false;
    if(meshcom_settings.node_symid == '/' || meshcom_settings.node_symid != '\'')
        bSymbolTable = true;
    else
    if(meshcom_settings.node_symid >= '0' && meshcom_settings.node_symid <= '9')
        bSymbolTable = true;
    else
    if(meshcom_settings.node_symid >= 'A' && meshcom_settings.node_symid <= 'Z')
        bSymbolTable = true;

    bool bSymbolCode = false;
    if(meshcom_settings.node_symcd >= '!' && meshcom_settings.node_symcd <= '}')
        bSymbolCode = true;

    if(!bSymbolTable || !bSymbolCode)   // set default
    {
        meshcom_settings.node_symid = '/';
        meshcom_settings.node_symcd = '#';

        save_settings();
    }

    if(bSendViaAPRS)
    {
        //int ilng = encodeLoRaAPRS(msg_buffer, meshcom_settings.node_call, lat, lat_c, lon, lon_c, alt);
        int ilng = encodeLoRaAPRScompressed(msg_buffer, meshcom_settings.node_call, lat, lat_c, lon, lon_c, alt);

        if(bDisplayInfo)
        {
            Serial.printf("%s [LO-APRS]...%s\n", getTimeString().c_str(), msg_buffer+3);
        }

        // local LoRa-APRS position-messages send to LoRa TX
        ringBuffer[iWrite][0]=ilng;
        ringBuffer[iWrite][1]=0xFF;    // Status byte for retransmission 0xFF no retransmission
        memcpy(ringBuffer[iWrite]+2, msg_buffer, ilng);
        iWrite++;
        if(iWrite >= MAX_RING)
            iWrite=0;

        // An APP als Anzeige retour senden
        if(isPhoneReady == 1 || bGATEWAY)
        {
            struct aprsMessage aprsmsg;

            initAPRS(aprsmsg, '!');

            aprsmsg.msg_len = 0;

            // MSG ID not used in APP

            if(intervall != POSINFO_INTERVAL)
                aprsmsg.msg_track=true;

            aprsmsg.msg_source_path = meshcom_settings.node_call;
            
            if(strlen(meshcom_settings.node_lora_call) != 0 && strcmp(meshcom_settings.node_lora_call, "none") != 0)
            {
                aprsmsg.msg_source_path = meshcom_settings.node_lora_call;
            }

            aprsmsg.msg_destination_path = "*";
            aprsmsg.msg_payload = PositionToAPRS(true, false, true, lat, lat_c, lon, lon_c, alt, press, hum, temp, temp2, gasres, co2, qfe, qnh);
            
            if(aprsmsg.msg_payload == "")
                return;

            encodeAPRS(msg_buffer, aprsmsg);
            
            if(isPhoneReady == 1)
            {
                addBLEOutBuffer(msg_buffer, aprsmsg.msg_len);
            }

            // Sonderfall für langstrecken track - LoRa-Track mit Sonderrufzeichen bringt auch MeshCom-Track via Gateway
            if(bGATEWAY)
            {
                if(bDisplayInfo)
                {
                    Serial.printf("%s [NEW-UDP]...%s\n", getTimeString().c_str(), msg_buffer+3);
                }

                // UDP out
                addNodeData(msg_buffer, aprsmsg.msg_len, 0, 0);
            }
        }

    }
    
    if(bSendViaMesh)
    {
        struct aprsMessage aprsmsg;

        initAPRS(aprsmsg, '!');

        aprsmsg.msg_len = 0;

        // MSG ID zusammen setzen    
        aprsmsg.msg_id = ((_GW_ID & 0x3FFFFF) << 10) | (meshcom_settings.node_msgid & 0x3FF);

        if(intervall != POSINFO_INTERVAL)
            aprsmsg.msg_track=true;

        aprsmsg.msg_source_path = meshcom_settings.node_call;
        aprsmsg.msg_destination_path = "*";
        aprsmsg.msg_payload = PositionToAPRS(true, false, true, lat, lat_c, lon, lon_c, alt, press, hum, temp, temp2, gasres, co2, qfe, qnh);
        
        if(aprsmsg.msg_payload == "")
            return;

        meshcom_settings.node_msgid++;
        if(meshcom_settings.node_msgid > 999)
            meshcom_settings.node_msgid=0;
            
        // Flash rewrite
        save_settings();

        encodeAPRS(msg_buffer, aprsmsg);

        if(bDisplayInfo)
        {
            printBuffer_aprs((char*)"NEW-POS", aprsmsg);
            Serial.println();
        }

        // local position-messages send to LoRa TX
        ringBuffer[iWrite][0]=aprsmsg.msg_len;
        ringBuffer[iWrite][1]=0xFF;    // Status byte for retransmission 0xFF no retransmission
        memcpy(ringBuffer[iWrite]+2, msg_buffer, aprsmsg.msg_len);
        iWrite++;
        if(iWrite >= MAX_RING)
            iWrite=0;

        if(bGATEWAY)
        {
            // UDP out
            addNodeData(msg_buffer, aprsmsg.msg_len, 0, 0);
        }
        
        // store last message to compare later on
        memcpy(own_msg_id[iWriteOwn], msg_buffer+1, 4);
        own_msg_id[iWriteOwn][4]=0x00;
        iWriteOwn++;
        if(iWriteOwn >= MAX_RING)
            iWriteOwn=0;

        // An APP als Anzeige retour senden
        if(isPhoneReady == 1)
        {
            addBLEOutBuffer(msg_buffer, aprsmsg.msg_len);
        }

        // Extern Server
        if(bEXTUDP)
            sendExtern(true, (char*)"node", msg_buffer, aprsmsg.msg_len);
    }

}

void sendAPPPosition(double lat, char lat_c, double lon, char lon_c, float temp2)
{
    uint8_t msg_buffer[MAX_MSG_LEN_PHONE];

    struct aprsMessage aprsmsg;

    initAPRS(aprsmsg, '!');

    aprsmsg.msg_len = 0;

    // MSG ID zusammen setzen    
    aprsmsg.msg_id = ((_GW_ID & 0x3FFFFF) << 10) | (meshcom_settings.node_msgid & 0x3FF);

    aprsmsg.msg_source_path = meshcom_settings.node_call;
    aprsmsg.msg_destination_path = "*";
    aprsmsg.msg_payload = PositionToAPRS(true, false, true, lat, lat_c, lon, lon_c, 0, 0, 0, 0, temp2, 0, 0, 0, 0);
    
    if(aprsmsg.msg_payload == "")
        return;

    meshcom_settings.node_msgid++;
    if(meshcom_settings.node_msgid > 999)
        meshcom_settings.node_msgid=0;
        
    // Flash rewrite
    save_settings();

    encodeAPRS(msg_buffer, aprsmsg);

    if(bDisplayInfo)
    {
        printBuffer_aprs((char*)"NEW-POS", aprsmsg);
        Serial.println();
    }

    // local position-messages send to LoRa TX
    ringBuffer[iWrite][0]=aprsmsg.msg_len;
    ringBuffer[iWrite][1]=0xFF;    // Status byte for retransmission 0xFF no retransmission
    memcpy(ringBuffer[iWrite]+2, msg_buffer, aprsmsg.msg_len);
    iWrite++;
    if(iWrite >= MAX_RING)
        iWrite=0;

    if(bGATEWAY)
    {
        // UDP out
        addNodeData(msg_buffer, aprsmsg.msg_len, 0, 0);
    }
    
    // store last message to compare later on
    memcpy(own_msg_id[iWriteOwn], msg_buffer+1, 4);
    own_msg_id[iWriteOwn][4]=0x00;
    iWriteOwn++;
    if(iWriteOwn >= MAX_RING)
        iWriteOwn=0;

}

void SendAckMessage(String dest_call, unsigned int iAckId)
{
    struct aprsMessage aprsmsg;

    initAPRS(aprsmsg, ':');

    aprsmsg.msg_len = 0;

    // MSG ID zusammen setzen    
    aprsmsg.msg_id = ((_GW_ID & 0x3FFFFF) << 10) | (meshcom_settings.node_msgid & 0x3FF);
    
    aprsmsg.msg_source_path = meshcom_settings.node_call;   // own Call
    aprsmsg.msg_destination_path = dest_call;
    aprsmsg.msg_destination_call = dest_call;

    char cackmsg[20];
    if(strcmp(dest_call.c_str(), "WLNK-1") == 0)
        snprintf(cackmsg, sizeof(cackmsg), "ack%04i", iAckId);
    else
        snprintf(cackmsg, sizeof(cackmsg), "%-9.9s:ack%03i", dest_call.c_str(), iAckId);
    aprsmsg.msg_payload = cackmsg;

    meshcom_settings.node_msgid++;
    if(meshcom_settings.node_msgid > 999)
        meshcom_settings.node_msgid=0;

    // Flash rewrite
    save_settings();

    uint8_t msg_buffer[MAX_MSG_LEN_PHONE];
    
    // store last message to compare later on
    insertOwnTx(aprsmsg.msg_id);

    if(bGATEWAY && meshcom_settings.node_hasIPaddress)
        addLoraRxBuffer(aprsmsg.msg_id, true);
    else
        addLoraRxBuffer(aprsmsg.msg_id, false);

    encodeAPRS(msg_buffer, aprsmsg);

    // ACK-Message send to LoRa TX
    if(bDisplayInfo)
    {
        printBuffer_aprs((char*)"NEW-ACK", aprsmsg);
        Serial.println();
    }

    ringBuffer[iWrite][0]=aprsmsg.msg_len;
    ringBuffer[iWrite][1]=0xFF;    // ACK-Status byte 0xFF for no retransmission
    memcpy(ringBuffer[iWrite]+2, msg_buffer, aprsmsg.msg_len);
    iWrite++;
    if(iWrite >= MAX_RING)
        iWrite=0;
    
    if(bGATEWAY && meshcom_settings.node_hasIPaddress)
    {
		// UDP out
        if(aprsmsg.msg_destination_call != meshcom_settings.node_call)
        {
            addNodeData(msg_buffer, aprsmsg.msg_len, 0, 0);
        }
    }
}

// Send Hey-Message
void sendHey()
{
    uint8_t msg_buffer[MAX_MSG_LEN_PHONE];

    struct aprsMessage aprsmsg;

    initAPRS(aprsmsg, '@');

    aprsmsg.msg_len = 0;

    // MSG ID zusammen setzen    
    aprsmsg.msg_id = ((_GW_ID & 0x3FFFFF) << 10) | (meshcom_settings.node_msgid & 0x3FF);   // MAC-address + 3FF = 1023 max rela only 0-999
    
    aprsmsg.msg_source_path = meshcom_settings.node_call;
    
    if(bGATEWAY)
        aprsmsg.msg_destination_path = "HG";
    else
        aprsmsg.msg_destination_path = "H";

    aprsmsg.msg_payload = "R";
   
    meshcom_settings.node_msgid++;
    if(meshcom_settings.node_msgid > 999)
        meshcom_settings.node_msgid=0;

    // Flash rewrite
    save_settings();

    encodeAPRS(msg_buffer, aprsmsg);

    if(bDisplayInfo)
    {
        printBuffer_aprs((char*)"NEW-HEY", aprsmsg);
        Serial.println();
    }

    // store last message to compare later on
    insertOwnTx(aprsmsg.msg_id);

    if(bGATEWAY)
    {
	    // UDP out
		addNodeData(msg_buffer, aprsmsg.msg_len, 0, 0);
    }

    // and also to LoRa
    {
        // Master RingBuffer for transmission
        // local messages send to LoRa TX
        ringBuffer[iWrite][0] = aprsmsg.msg_len;
        ringBuffer[iWrite][1] = 0xFF; // retransmission Status ...0xFF no retransmission
        memcpy(ringBuffer[iWrite]+2, msg_buffer, aprsmsg.msg_len);

        iWrite++;
        if(iWrite >= MAX_RING)
            iWrite=0;
    }
}

// Telemetry with own Parameters
void sendTelemetry(int ID)
{
    String stationCall = meshcom_settings.node_call;

    if(ID == 1)
    {
        if(strlen(meshcom_settings.node_parm_1) == 0 || strcmp(meshcom_settings.node_parm_1, "none") == 0)
            return;
    
        int iipos=0;
        iipos = strSOFTSERAPP_ID.length() - 9;
        if(iipos < 0)
            iipos=0;
        stationCall = strSOFTSERAPP_ID.substring(iipos);    // APRS max. 9 chars
    }
    else
    {
        if(strlen(meshcom_settings.node_parm) == 0 || strcmp(meshcom_settings.node_parm, "none") == 0)
            return;
    }

    if(strlen(meshcom_settings.node_unit) == 0 || strcmp(meshcom_settings.node_unit, "none") == 0)
        return;

    if(strlen(meshcom_settings.node_values) == 0 || strcmp(meshcom_settings.node_values, "none") == 0)
        return;

    uint8_t msg_buffer[MAX_MSG_LEN_PHONE];

    struct aprsMessage aprsmsg;

    initAPRS(aprsmsg, ':');

    aprsmsg.msg_len = 0;

    // MSG ID zusammen setzen    
    aprsmsg.msg_id = ((_GW_ID & 0x3FFFFF) << 10) | (meshcom_settings.node_msgid & 0x3FF);   // MAC-address + 3FF = 1023 max rela only 0-999
    
    aprsmsg.msg_source_path = meshcom_settings.node_call;
    
    aprsmsg.msg_destination_path = "100001";

    if(iNextTelemetry == 0)
    {
        if(ID == 1)
            snprintf(msg_text, sizeof(msg_text), "%-9.9s:PARM.%s", stationCall.c_str(), meshcom_settings.node_parm_1);
        else
            snprintf(msg_text, sizeof(msg_text), "%-9.9s:PARM.%s", stationCall.c_str(), meshcom_settings.node_parm);

        iNextTelemetry=1;
    }
    else
    if(iNextTelemetry == 1)
    {
        if(strlen(meshcom_settings.node_unit) == 0 || strcmp(meshcom_settings.node_unit, "none") == 0)
            return;

        snprintf(msg_text, sizeof(msg_text), "%-9.9s:UNIT.%s", stationCall.c_str(), meshcom_settings.node_unit);

        iNextTelemetry=2;
    }
    else
    if(iNextTelemetry == 2)
    {
        if(strlen(meshcom_settings.node_eqns) == 0 || strcmp(meshcom_settings.node_eqns, "none") == 0)
            strTelemetry = "0,1,0,0,1,0,0,1,0,0,1,0,0,1,0";
        else
            strTelemetry=meshcom_settings.node_eqns;
        
        if(count_char(strTelemetry, ',') != 14)
            strTelemetry = "0,1,0,0,1,0,0,1,0,0,1,0,0,1,0";

        snprintf(msg_text, sizeof(msg_text), "%-9.9s:EQNS.%s", stationCall.c_str(), strTelemetry.c_str());

        iNextTelemetry=3;
    }
    else
    if(iNextTelemetry == 3)
    {
        memset(msg_text, 0x00, sizeof(msg_text));

        if(strSOFTSERAPP_NAME.length() > 0 && ID == 1)
        {
            snprintf(msg_text, sizeof(msg_text), "%-9.9s:BITS.00000000%s", stationCall.c_str(), strSOFTSERAPP_NAME.c_str());
        }

        iNextTelemetry=4;
    }
    else
    // Values to APRS.FI
    if(iNextTelemetry >= 4)
    {
        char cv[20];
        snprintf(cv, sizeof(cv), "%-9.9s:T#%03i", stationCall.c_str(), meshcom_settings.node_msgid);

        strTelemetry = cv;

        String strValue="";
        int ivcount=0;

        if(memcmp(meshcom_settings.node_values, "T:", 2) == 0)
        {
            strValue = meshcom_settings.node_values;
            
            strTelemetry.concat(",");
            strTelemetry.concat(strValue.substring(2));

            strTelemetry.concat(",0,0,00000000,");  // + zwei Messwerte
            strTelemetry.concat(meshcom_settings.node_parm_t);
            strTelemetry.concat(",");
            strTelemetry.concat(meshcom_settings.node_parm_id);

            memset(meshcom_settings.node_parm_1, 0x00, sizeof(meshcom_settings.node_parm_1));
            memset(meshcom_settings.node_unit, 0x00, sizeof(meshcom_settings.node_unit));
            memset(meshcom_settings.node_values, 0x00, sizeof(meshcom_settings.node_values));
            memset(meshcom_settings.node_parm_t, 0x00, sizeof(meshcom_settings.node_parm_t));
            memset(meshcom_settings.node_parm_id, 0x00, sizeof(meshcom_settings.node_parm_id));

        }
        else
        {
            for(int ival=0;ival<=(int)strlen(meshcom_settings.node_values); ival++)
            {
                if(meshcom_settings.node_values[ival] == ',' || meshcom_settings.node_values[ival] == 0x00)
                {
                    // max. 5 values
                    ivcount++;
                    if(ivcount > 5)
                        break;

                    strTelemetry.concat(",");

                    strValue.trim();

                    if(strValue == "press")
                    {
                        snprintf(cv, sizeof(cv), "%.1f", meshcom_settings.node_press);
                        strTelemetry.concat(cv);
                    }
                    else
                    if(strValue == "hum")
                    {
                        snprintf(cv, sizeof(cv), "%.1f", meshcom_settings.node_hum);
                        strTelemetry.concat(cv);
                    }
                    else
                    if(strValue == "temp")
                    {
                        snprintf(cv, sizeof(cv), "%.1f", meshcom_settings.node_temp);
                        strTelemetry.concat(cv);
                    }
                    else
                    if(strValue == "onewire")
                    {
                        snprintf(cv, sizeof(cv), "%.1f", meshcom_settings.node_temp2);
                        strTelemetry.concat(cv);
                    }
                    else
                    if(strValue == "gasres")
                    {
                        snprintf(cv, sizeof(cv), "%.1f", meshcom_settings.node_gas_res);
                        strTelemetry.concat(cv);
                    }
                    else
                    if(strValue == "co2")
                    {
                        snprintf(cv, sizeof(cv), "%.1f", meshcom_settings.node_co2);
                        strTelemetry.concat(cv);
                    }
                    else
                    if(strValue == "batt")
                    {
                        snprintf(cv, sizeof(cv), "%.2f", global_batt/1000.);
                        strTelemetry.concat(cv);
                    }
                    else
                    if(strValue == "proz")
                    {
                        snprintf(cv, sizeof(cv), "%i", global_proz);
                        strTelemetry.concat(cv);
                    }
                    else
                    if(strValue == "vbus")
                    {
                        snprintf(cv, sizeof(cv), "%.2f", meshcom_settings.node_vbus);
                        strTelemetry.concat(cv);
                    }
                    else
                    if(strValue == "vcurrent")
                    {
                        snprintf(cv, sizeof(cv), "%.1f", meshcom_settings.node_vcurrent);
                        strTelemetry.concat(cv);
                    }
                    else
                    if(strValue == "qnh")
                    {
                        snprintf(cv, sizeof(cv), "%.1f", meshcom_settings.node_press_asl);
                        strTelemetry.concat(cv);
                    }

                    strValue="";
                }
                else
                {
                    snprintf(cv, sizeof(cv), "%c", meshcom_settings.node_values[ival]);

                    strValue.concat(cv);
                }
            }
        }

        
        snprintf(msg_text, sizeof(msg_text), "%s", strTelemetry.c_str());

        iNextTelemetry++;
        // Alle 10 Werte - PARM, UNIT. EQNS. BITS. neuerlich senden
        if(iNextTelemetry > 13)
            iNextTelemetry=0;
    }

    if(strlen(msg_text) > 0)
    {
        aprsmsg.msg_payload = msg_text;
        
        meshcom_settings.node_msgid++;
        if(meshcom_settings.node_msgid > 999)
            meshcom_settings.node_msgid=0;

        // Flash rewrite
        save_settings();

        encodeAPRS(msg_buffer, aprsmsg);

        if(bDisplayInfo)
        {
            printBuffer_aprs((char*)"NEW-TMY", aprsmsg);
            Serial.println();
        }

        // store last message to compare later on
        insertOwnTx(aprsmsg.msg_id);

        // if GATEWAY only to Server
        if(bGATEWAY)
        {
            // UDP out
            addNodeData(msg_buffer, aprsmsg.msg_len, 0, 0);
        }
        else
        {
            // Master RingBuffer for transmission
            // local messages send to LoRa TX
            ringBuffer[iWrite][0] = aprsmsg.msg_len;
            ringBuffer[iWrite][1] = 0xFF; // retransmission Status ...0xFF no retransmission
            memcpy(ringBuffer[iWrite]+2, msg_buffer, aprsmsg.msg_len);

            iWrite++;
            if(iWrite >= MAX_RING)
                iWrite=0;
        }
    }
}

int GetHeadingDifference(int heading1, int heading2)
{   
    int  difference = heading1 - heading2;
    if(difference < 0)
        difference = difference * -1;
        
    if (difference > 180)
        return 360 - difference;
    
    return difference;
}

unsigned int setSMartBeaconing(double dlat, double dlon)
{
    extern TinyGPSPlus tinyGPSPLus;

    unsigned int gps_send_rate = POSINFO_INTERVAL;  // seconds

    if(posinfo_lat == 0.0)
        posinfo_lat = dlat;
    if(posinfo_lon == 0.0)
        posinfo_lon = dlon;


    posinfo_distance_ring[posinfo_ring_write] = tinyGPSPLus.distanceBetween(posinfo_lat, posinfo_lon, dlat, dlon);    // meters
    posinfo_direction = tinyGPSPLus.courseTo(posinfo_last_lat, posinfo_last_lon, dlat, dlon);    // Grad

    posinfo_ring_write++;
    if(posinfo_ring_write > 9)
    {
        posinfo_ring_write=0;
    }

    posinfo_distance=0;
    for(int ir=0;ir<10;ir++)
    {
        posinfo_distance += (int)posinfo_distance_ring[ir];
    }

    posinfo_lat = dlat;
    posinfo_lon = dlon;

    // get gps distance every 100 seconds
    // gps_send_rate 30 minutes default
    // bDisplayTrack = true Smartbeaconing used
    if(posinfo_distance < 100 || !bDisplayTrack)  // seit letzter gemeldeter position
    {
        if(meshcom_settings.node_postime > 0)
            gps_send_rate = meshcom_settings.node_postime;
        else
            gps_send_rate = POSINFO_INTERVAL;
    }
    else
    if(posinfo_distance < 200)  // zu fuss > 3 km/h  < 8 km/h
        gps_send_rate = 30; // seconds
    else
    if(posinfo_distance < 800)  // rad < 40 km/h
        gps_send_rate = 60; // seconds
    else
    if(posinfo_distance < 3000)  // auto stadt < 80 km/h
        gps_send_rate = 120; // seconds
    else
        gps_send_rate = 180; // auto > 80 km/h

    int direction_diff=0;

    if(posinfo_distance > 150 && bDisplayTrack)  // meter
    {
        direction_diff=GetHeadingDifference((int)posinfo_last_direction, (int)posinfo_direction);

        if(direction_diff > 15)
        {
            posinfo_shot=true;
        }
    }

    if(posinfo_last_lat == 0.0 && posinfo_last_lon == 0.0)
        posinfo_shot=true;

    if(posinfo_shot)
    {
        if(bGPSDEBUG)
        {
            Serial.printf("%s [POSINFO]... one-shot set - direction_diff:%i last_lat:%.1lf last_lon:%.1lf\n", getTimeString().c_str(), direction_diff, posinfo_last_lat, posinfo_last_lon);
        }
    }

    return gps_send_rate;
}

String convertCallToShort(char callsign[10])
{
    String sVar = "XXX";

    for(int its=0; its<10; its++)
    {
        if(callsign[its] == '-' || callsign[its] == 0x00)    // SSID Trennung oder ohne SSID
        {
            sVar.setCharAt(0, callsign[its-3]);
            sVar.setCharAt(1, callsign[its-2]);
            sVar.setCharAt(2, callsign[its-1]);
            sVar = sVar + "40";
            break;
        }
    }

    sVar.toUpperCase();

    return sVar;
}

double cround4(double dvar)
{
    char cvar[20];
    snprintf(cvar, sizeof(cvar), "%.4lf", dvar);
    double rvar;
    sscanf(cvar, "%lf", &rvar);

    return rvar;
}

double cround4abs(double dvar)
{
    char cvar[20];
    snprintf(cvar, sizeof(cvar), "%.4lf", dvar);
    double rvar;
    sscanf(cvar, "%lf", &rvar);

    rvar = abs(rvar);

    return rvar;
}

int conv_fuss(int alt_meter)
{
    double fuss = alt_meter * 10;
    fuss = fuss * 3.28084;
    int ifuss = fuss + 5;
    return ifuss / 10;
}

int conv_meter(int alt_fuss)
{
    return (int)((float)alt_fuss * 0.3048);
}

bool checkMesh()
{
    return bMESH;
}

// ****** UTF8-Decoder: convert UTF8-string to extended ASCII *******
byte c1;  // Last character buffer

// Convert a single Character from UTF8 to Extended ASCII
// Return "0" if a byte has to be ignored
byte utf8ascii(byte ascii)
{
    if ( ascii<128 )   // Standard ASCII-set 0..0x7F handling  
    {   c1=0;
        return( ascii );
    }

    // get previous input
    byte last = c1;   // get last char
    c1=ascii;         // remember actual character

    switch (last)     // conversion depending on first UTF8-character
    {
        case 0xC3:
            if(ascii == 0xA4)   // ä
                return  (0x94);
            else
            if(ascii == 0xB6)   // ö
                return  (0x92);
            else
            if(ascii == 0xBC)   // ü
                return  (0x81);
            else
            if(ascii == 0x84)   // Ä
                return  (0x8E);
            else
            if(ascii == 0x96)   // Ö
                return  (0x99);
            else
            if(ascii == 0x9C)   // Ü
                return  (0x9A);
            else
            if(ascii == 0x9F)   // ß
                return  (0xE1);
            else
                return (0);
            break;
        case 0xC2:
            if(ascii == 0xB0)   // °
                return  (0xF8);
            break;
        case 0x82:
            if(ascii==0xAC)
                return(0x80);       // special case Euro-symbol
    }

    return  (0);                                     // otherwise: return zero, if character has to be ignored
}

// convert String object from UTF8 String to Extended ASCII
String utf8ascii(String s)
{      
        String r="";
        char c;
        for (int i=0; i<(int)s.length(); i++)
        {
                c = utf8ascii(s.charAt(i));
                if (c!=0)
                {
                    if(c == 0x94)
                        r+="ae";
                    else
                    if(c == 0x92)
                        r+="oe";
                    else
                    if(c == 0x81)
                        r+="ue";
                    else
                    if(c == 0x8E)
                        r+="Ae";
                    else
                    if(c == 0x99)
                        r+="Oe";
                    else
                    if(c == 0x9A)
                        r+="Ue";
                    else
                    if(c == 0xE1)
                        r+="ss";
                    else
                    if(c == 0x80)
                        r+="*";
                    else
                       r+=c;
                }
        }
        return r;
}

// In Place conversion UTF8-string to Extended ASCII (ASCII is shorter!)
void utf8ascii(char* s)
{      
        int k=0;
        char c;
        for (int i=0; i<(int)strlen(s); i++)
        {
                c = utf8ascii(s[i]);
                if (c!=0)
                        s[k++]=c;
        }
        s[k]=0;
}

String getTimeZone()
{
    if(meshcom_settings.node_utcoff == 0)
        return "UTC";

    return "LT";
}

int count_char(String s, char c)
{
    int count = 0;
  
    for (int i = 0; i < (int)s.length(); i++)
      if (s.charAt(i) == c) count++;
  
    return count;
}