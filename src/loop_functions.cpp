#include "Arduino.h"

#include "loop_functions.h"
#include "command_functions.h"

#include "clock.h"

#include "batt_functions.h"
#include "udp_functions.h"
#include "configuration.h"

#include "TinyGPSplus.h"

// TinyGPS
extern TinyGPSPlus tinyGPSPLus;

extern unsigned long rebootAuto;

extern float global_batt;
extern int global_proz;

bool bSetLoRaAPRS = false;

bool bDEBUG = false;
bool bLORADEBUG = false;
bool bBLEDEBUG = false;
bool bWXDEBUG = false;

bool bPosDisplay = true;
bool bDisplayOff = false;
bool bDisplayVolt = false;
bool bDisplayInfo = false;
unsigned long DisplayOffWait = 0;
bool bDisplayTrack = false;
bool bGPSON = false;
bool bBMPON = false;
bool bBMEON = false;

bool bONEWIRE = false;

bool bLPS33 = false;

bool bGATEWAY = false;
bool bMESH = false;
bool bEXTUDP = false;
bool bEXTSER = false;

bool bSHORTPATH = false;
bool bGPSDEBUG = false;

bool bBLElong = false;

int iDisplayType = 0;
int DisplayTimeWait = 0;

bool bWaitButton_Released = false;
bool bButtonCheck = false;

int iInitDisplay = 0;
int iDisplayChange = 0;

unsigned long lastHeardTime = 0;

// common variables
char msg_text[MAX_MSG_LEN_PHONE * 2] = {0};

unsigned int _GW_ID = 0x12345678; // ID of our Node

#if defined(BOARD_HELTEC)
    U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, 16, 15, 4);
#elif defined(BOARD_HELTEC_V3)
    U8G2_SSD1306_128X64_NONAME_1_SW_I2C u8g2(U8G2_R0, 18, 17, 21);
#elif defined(BOARD_RAK4630)
    U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);  //RESET CLOCK DATA
#else
    U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);  //RESET CLOCK DATA
#endif

unsigned int msg_counter = 0;

uint8_t RcvBuffer[UDP_TX_BUF_SIZE * 2] = {0};

// nur eigene msg_id
uint8_t own_msg_id[MAX_RING][5] = {0};
int iWriteOwn=0;

// RINGBUFFER for incoming UDP lora packets for lora TX
unsigned char ringBuffer[MAX_RING][UDP_TX_BUF_SIZE] = {0};
int iWrite=0;
int iRead=0;

bool hasMsgFromPhone = false;

// BLE Ringbuffer to phone
unsigned char BLEtoPhoneBuff[MAX_RING][MAX_MSG_LEN_PHONE] = {0};
int toPhoneWrite=0;
int toPhoneRead=0;

uint8_t ringBufferLoraRX[MAX_RING][4] = {0}; //Ringbuffer for UDP TX from LoRa RX, first byte is length
uint8_t loraWrite = 0;   // counter for ringbuffer

// LoRa RX/TX sequence control
int cmd_counter = 0;      // ticker dependant on main cycle delay time
bool is_receiving = false;  // flag to store we are receiving a lora packet.
bool tx_is_active = false;  // flag to store we are transmitting  a lora packet.
bool tx_waiting = false;

uint8_t isPhoneReady = 0;      // flag we receive from phone when itis ready to receive data

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
bool pos_shot=false;
bool wx_shot=false;
int no_gps_reset_counter = 0;

// Loop timers
unsigned long posinfo_timer = 0;    // we check periodically to send GPS
unsigned long temphum_timer = 0;    // we check periodically get TEMP/HUM
unsigned long druck_timer = 0;      // we check periodically get AIRPRESURE
unsigned long hb_timer = 0;         // we check periodically get AIRPRESURE

/** @brief Function adding messages into outgoing BLE ringbuffer
 * BLE to PHONE Buffer
 */
void addBLEOutBuffer(uint8_t *buffer, uint16_t len)
{
    if (len > UDP_TX_BUF_SIZE)
        len = UDP_TX_BUF_SIZE-1; // just for safety

    //first two bytes are always the message length
    BLEtoPhoneBuff[toPhoneWrite][0] = len;
    memcpy(BLEtoPhoneBuff[toPhoneWrite] + 1, buffer, len);

    if(bDEBUG)
    {
        Serial.printf("<%02X>BLEtoPhone RingBuff added len=%i to element: %u\n", buffer[0], len, toPhoneWrite);
        printBuffer(BLEtoPhoneBuff[toPhoneWrite], len + 1);
    }

    toPhoneWrite++;
    
    //Serial.printf("toPhoneWrite:%i\n", toPhoneWrite);

    if (toPhoneWrite >= MAX_RING) // if the buffer is full we start at index 0 -> take care of overwriting!
        toPhoneWrite = 0;
}

void addBLECommandBack(char text[UDP_TX_BUF_SIZE])
{
    uint8_t msg_buffer[MAX_MSG_LEN_PHONE];

    struct aprsMessage aprsmsg;

    initAPRS(aprsmsg);

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
void addLoraRxBuffer(unsigned int msg_id)
{
    // byte 0-3 msg_id
    ringBufferLoraRX[loraWrite][3] = msg_id >> 24;
    ringBufferLoraRX[loraWrite][2] = msg_id >> 16;
    ringBufferLoraRX[loraWrite][1] = msg_id >> 8;
    ringBufferLoraRX[loraWrite][0] = msg_id;

    /*
    if(bDEBUG)
    {
        Serial.printf("LoraRX Ringbuffer added element: %u %02X%02X%02X%02X", loraWrite, ringBufferLoraRX[loraWrite][3], ringBufferLoraRX[loraWrite][2], ringBufferLoraRX[loraWrite][1], ringBufferLoraRX[loraWrite][0]);
    }
    */

    loraWrite++;

    //Serial.printf("loraWrite:%i\n", loraWrite);

    if (loraWrite >= MAX_RING) // if the buffer is full we start at index 0 -> take care of overwriting!
        loraWrite = 0;
}

int pageLine[7][3] = {0};
char pageText[7][25] = {0};
int pageLineAnz=0;

#define PAGE_MAX 6

int pageLastLine[PAGE_MAX][7][3] = {0};
char pageLastText[PAGE_MAX][7][25] = {0};
int pageLastLineAnz[PAGE_MAX] = {0};
int pageLastPointer=0;
int pagePointer=0;
int pageHold=PAGE_MAX-1;

bool bSetDisplay = false;

void sendDisplay1306(bool bClear, bool bTransfer, int x, int y, char *text)
{
	if(bClear)
    {
        u8g2.setFont(u8g2_font_6x10_mf);

        //u8g2.clearDisplay();
        //u8g2.clearBuffer();					// clear the internal memory

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
        if(pageLineAnz < 7 && strlen(text) < 25)
        {
            //Serial.printf("pageLineAnz:%i text:%s\n", pageLineAnz, text);

            pageLine[pageLineAnz][0] = x;
            pageLine[pageLineAnz][1] = y;
            pageLine[pageLineAnz][2] = 20;
            memcpy(pageText[pageLineAnz], text, 25);
            pageLineAnz++;
        }
        //u8g2.drawUTF8(x, y, text);
    }

    if(bTransfer)
    {
        //Serial.println("Transfer");
        u8g2.firstPage();
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
                        u8g2.drawHLine(pageLine[its][0], pageLine[its][1], 120);
                    }
                    else
                    {
                        sprintf(ptext, "%s", pageText[its]);
                        if(pageLine[its][1] >= 0)
                            u8g2.drawUTF8(pageLine[its][0], pageLine[its][1], ptext);
                    }
                }

            }

        } while (u8g2.nextPage());

        if(iDisplayType == 0 && bNeu)
        {
            pagePointer=pageLastPointer;

            pageLastPointer++;
            if(pageLastPointer > PAGE_MAX-1)
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

    if(bDisplayOff)
    {
        sendDisplay1306(true, true, 0, 0, (char*)"#C");
        bSetDisplay=false;
        return;
    }

    iDisplayType=9;

    char print_text[500];

    sendDisplayMainline();

    sprintf(print_text, "Call:  %s", meshcom_settings.node_call);
    sendDisplay1306(false, false, 3, 22, print_text);

    sprintf(print_text, "Short: %s", meshcom_settings.node_short);
    sendDisplay1306(false, false, 3, 32, print_text);

    sprintf(print_text, "MAC:   %08X", _GW_ID);
    sendDisplay1306(false, false, 3, 42, print_text);

    sprintf(print_text, "Modul: %i", MODUL_HARDWARE);
    sendDisplay1306(false, false, 3, 52, print_text);

    sprintf(print_text, "ssid:  %-15.15s", meshcom_settings.node_ssid);
    sendDisplay1306(false, true, 3, 62, print_text);

    bSetDisplay=false;
}

void sendDisplayTrack()
{
    if(bSetDisplay)
        return;

    bSetDisplay=true;

    if(bDisplayOff)
    {
        sendDisplay1306(true, true, 0, 0, (char*)"#C");
        bSetDisplay=false;
        return;
    }

    iDisplayType=9;

    char print_text[500];

    sendDisplayMainline();

    sprintf(print_text, "LAT : %.4lf %c  %s", meshcom_settings.node_lat, meshcom_settings.node_lat_c, (posinfo_fix?"fix":""));
    sendDisplay1306(false, false, 3, 22, print_text);

    sprintf(print_text, "LON : %.4lf %c %4i", meshcom_settings.node_lon, meshcom_settings.node_lon_c, (int)posinfo_satcount);
    sendDisplay1306(false, false, 3, 32, print_text);

    sprintf(print_text, "RATE: %5i sec %4i", (int)posinfo_interval, posinfo_hdop);
    sendDisplay1306(false, false, 3, 42, print_text);

    sprintf(print_text, "DIST: %5i m", posinfo_distance);
    sendDisplay1306(false, false, 3, 52, print_text);

    sprintf(print_text, "DIR :old%3i° new%3i°", (int)posinfo_last_direction, (int)posinfo_direction);
    sendDisplay1306(false, true, 3, 62, print_text);


    bSetDisplay=false;
}

void sendDisplayWX()
{
    if(bSetDisplay)
        return;

    bSetDisplay=true;

    if(bDisplayOff)
    {
        sendDisplay1306(true, true, 0, 0, (char*)"#C");
        bSetDisplay=false;
        return;
    }

    iDisplayType=9;

    char print_text[500];

    sendDisplayMainline();

    sprintf(print_text, "TEMP : %.1f °C", meshcom_settings.node_temp);
    sendDisplay1306(false, false, 3, 22, print_text);

    sprintf(print_text, "HUM  : %.1f %%", meshcom_settings.node_hum);
    sendDisplay1306(false, false, 3, 32, print_text);

    sprintf(print_text, "QFE  : %.1f hPa", meshcom_settings.node_press);
    sendDisplay1306(false, false, 3, 42, print_text);

    sprintf(print_text, "QNH  : %.1f hPa", meshcom_settings.node_press_asl);
    sendDisplay1306(false, false, 3, 52, print_text);

    sprintf(print_text, "ALT:%5im / %5im", meshcom_settings.node_alt, meshcom_settings.node_press_alt);
    sendDisplay1306(false, true, 3, 62, print_text);


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

    if(bDisplayOff)
        return;

    if(iDisplayType == 0)
        return;

    if(bSetDisplay)
        return;

    bSetDisplay = true;

    char print_text[500];
    char cbatt[5];

    char nodetype[5];

    sprintf(nodetype, "%s", SOURCE_TYPE);
    if(bGATEWAY)
        sprintf(nodetype, "G");

    if(bDisplayVolt)
        sprintf(cbatt, "%4.2f", global_batt/1000.0);
    else
        sprintf(cbatt, "%3d%%", global_proz);

 #if defined(XPOWERS_CHIP_AXP192)
    if(global_batt == 0.0)
        sprintf(cbatt, " USB");
 #endif

 #if defined(XPOWERS_CHIP_AXP2101)
    if(global_batt == 0.0)
        sprintf(cbatt, " USB");
 #endif

    sprintf(print_text, "%-1.1s%-4.4s%-1.1s %02i:%02i:%02i %-4.4s", nodetype, SOURCE_VERSION, SOURCE_VERSION_SUB, meshcom_settings.node_date_hour, meshcom_settings.node_date_minute, meshcom_settings.node_date_second, cbatt);

    memcpy(pageText[0], print_text, 20);

    #ifdef BOARD_HELTEC_V3
    u8g2.firstPage();
    u8g2.drawStr(pageLine[0][0], pageLine[0][1], print_text);
    u8g2.nextPage();
    #else
    u8g2.setCursor(pageLine[0][0], pageLine[0][1]);
    u8g2.print(print_text);
    u8g2.sendBuffer();
    #endif

    bSetDisplay = false;
}

void sendDisplayMainline()
{
    char print_text[500];
    char cbatt[5];
    char nodetype[5];

    sprintf(nodetype, "%s", SOURCE_TYPE);
    if(bGATEWAY)
        sprintf(nodetype, "G");

    if(bDisplayVolt)
        sprintf(cbatt, "%4.2f", global_batt/1000.0);
    else
        sprintf(cbatt, "%3d%%", global_proz);

 #if defined(XPOWERS_CHIP_AXP192)
    if(global_batt == 0.0)
        sprintf(cbatt, " USB");
 #endif

 #if defined(XPOWERS_CHIP_AXP2101)
    if(global_batt == 0.0)
        sprintf(cbatt, " USB");
 #endif

    if(meshcom_settings.node_date_hour == 0 && meshcom_settings.node_date_minute == 0 && meshcom_settings.node_date_second == 0)
    {
        sprintf(print_text, "%-1.1s %-4.4s%-1.1s         %-4.4s", nodetype, SOURCE_VERSION, SOURCE_VERSION_SUB, cbatt);
    }
    else
    {
        sprintf(print_text, "%-1.1s%-4.4s%-1.1s %02i:%02i:%02i %-4.4s", nodetype, SOURCE_VERSION, SOURCE_VERSION_SUB, meshcom_settings.node_date_hour, meshcom_settings.node_date_minute, meshcom_settings.node_date_second, cbatt);
    }

    sendDisplay1306(true, false, 3, 10, print_text);
    sendDisplay1306(false, false, 3, 12, (char*)"#L");
}

void mainStartTimeLoop()
{
    /////////////////////////////////////////////////////////////////////////////////////////////
    // Start-Loop & Time-Loop
    if(iInitDisplay < 4)
    {
        //Serial.printf("iInitDisplay %i meshcom_settings.node_date_second %i DisplayTimeWait %i\n", iInitDisplay, meshcom_settings.node_date_second, DisplayTimeWait);

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
            bool bsDisplayOff = bDisplayOff;

            bDisplayOff = false;

            sendDisplayHead(true);

            bDisplayOff = bsDisplayOff;

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
                    sendDisplayTime(); // Time only

                DisplayTimeWait = meshcom_settings.node_date_second;
            }
        }
    // End Start-Loop & Time-Loop
    /////////////////////////////////////////////////////////////////////////////////////////////
}


void sendDisplayText(struct aprsMessage &aprsmsg, int16_t rssi, int8_t snr)
{
    if(aprsmsg.msg_payload.startsWith("{CET}") > 0)
    {
        char csetTime[30];
        sprintf(csetTime, "%s", aprsmsg.msg_payload.c_str());

        int Year;
        int Month;
        int Day;
        int Hour;
        int Minute;
        int Second;

        sscanf(csetTime+5, "%d-%d-%d %d:%d:%d", &Year, &Month, &Day, &Hour, &Minute, &Second);
        
        MyClock.setCurrentTime(false, Year, Month, Day, Hour, Minute, Second);

        if(bDisplayInfo)
        {
            Serial.println("");
            Serial.print(getTimeString());
            Serial.print(" TIMESET: Time set ");
        }

        bPosDisplay=true;

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

    if(bDisplayOff)
    {
        DisplayOffWait=millis() + (30 * 1000); // seconds
        bDisplayOff=false;
    }
    
    iDisplayType = 0;

    int izeile=12;
    unsigned int itxt=0;

    bool bClear=true;

    char line_text[21];
    char words[100][21]={0};
    unsigned int iwords=0;
    int ipos=0;

    String strPath = aprsmsg.msg_source_path;
    // DM
    if(aprsmsg.msg_destination_path != "*")
    {
        strPath = aprsmsg.msg_source_call + ">" + aprsmsg.msg_destination_call;
    }

    if(aprsmsg.msg_source_path.length() < (20-7))
        sprintf(msg_text, "%s <%i>", strPath.c_str(), rssi);
    else
        sprintf(msg_text, "%s", strPath.c_str());

    msg_text[20]=0x00;
    sendDisplay1306(bClear, false, 3, izeile, msg_text);

    izeile=23;
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
            ipos=0;
        }
        else
        if(ipos > 19)
        {
            words[iwords][20]=0x00;
            iwords++;
            words[iwords][0]=aprsmsg.msg_payload.charAt(itxt);
            ipos=1;
        }
        else
            ipos++;
    }

    words[iwords][ipos]=0x00;
    iwords++;

    memset(line_text, 0x00, 21);
    strcat(line_text, words[0]);

    bool bEnd=false;

    for(itxt=1; itxt<iwords; itxt++)
    {
        if((strlen(line_text) + strlen(words[itxt])) > 19)
        {
            line_text[20]=0x00;
            sprintf(msg_text, "%s", line_text);

            if(izeile > 60)
                bEnd=true;

            if(bEnd && itxt < iwords)
                sprintf(msg_text, "%-17.17s...", line_text);

            msg_text[20]=0x00;
            sendDisplay1306(bClear, bEnd, 3, izeile, msg_text);

            izeile=izeile+10;
            
            memset(line_text, 0x00, 21);

            if(izeile > 63)
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
        if(izeile > 61)
            izeile=61;

        bEnd=true;
        line_text[20]=0x00;
        sprintf(msg_text, "%s", line_text);
        //Serial.printf("1306-02:%s len:%i izeile:%i\n", msg_text, strlen(msg_text), izeile);
        msg_text[20]=0x00;
        sendDisplay1306(bClear, bEnd, 3, izeile, msg_text);
    }

    bSetDisplay=false;

}

void init_loop_function()
{
    posinfo_last_direction = 0.0;
    posinfo_satcount = 0;
    posinfo_hdop = 0;
    posinfo_fix = false;
}

// BUTTON
void initButtonPin()
{
    #ifdef BUTTON_PIN
        pinMode(BUTTON_PIN, INPUT_PULLUP);
    #endif
}

int checkButtoExtraLong = 0;
int checkButtonTime = 0;
int iPress = 0;
bool bPressed=false;

void checkButtonState()
{
    #ifdef BUTTON_PIN
        if(bButtonCheck)
        {
            if(digitalRead(BUTTON_PIN) == 0)
            {
                checkButtoExtraLong++;
                if(checkButtoExtraLong > 80)
                {
                    checkButtoExtraLong=0;
                    bButtonCheck=false;
                    meshcom_settings.node_sset = meshcom_settings.node_sset & 0x7FEF;
                    save_settings();
                    Serial.println("BUTTON not connected (set BUTTON to off)");
                    return;
                }

                if(bDEBUG)
                    Serial.printf("Button Pressed pageLastPointer:%i pageLastLineAnz[%i]:%i Track:%i\n", pageLastPointer, pagePointer, pageLastLineAnz[pagePointer], bDisplayTrack);

                if(!bPressed)
                {
                    bPressed = true;

                    iPress++;

                    if(iPress == 1)
                        checkButtonTime = 8;

                    if(bDEBUG)
                        Serial.printf("checkButtonTime:%i iPress:%i\n", checkButtonTime, iPress);
                }

                return;
            }
            else
            {
                checkButtoExtraLong = 0;

                bPressed = false;

                checkButtonTime--;
                if(checkButtonTime < 0)
                {
                    checkButtonTime = 0;

                    if(iPress == 2)
                    {
                        if(bDEBUG)
                            Serial.println("BUTTON double press");

                        if(bDisplayTrack)
                            commandAction((char*)"--sendtrack", false);
                        else
                            commandAction((char*)"--sendpos", false);

                    }
                    else
                    if(iPress == 1)
                    {
                        if(bDEBUG)
                            Serial.println("BUTTON singel press");

                        if(pageLastLineAnz[pagePointer] == 0 || bDisplayTrack)
                        {
                            bDisplayTrack =false;
                            
                            addBLECommandBack((char*)"--track off");

                            pageHold=0;

                            pagePointer = pageLastPointer - 1;
                            if(pagePointer < 0)
                                pagePointer = PAGE_MAX-1;

                            bDisplayOff=!bDisplayOff;

                            sendDisplayHead(false);

                        }
                        else
                        {
                            bDisplayOff=false;

                            for(int its=0;its<pageLastLineAnz[pagePointer];its++)
                            {
                                // Save last Text (init)
                                pageLineAnz = pageLastLineAnz[pagePointer];
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

                            iDisplayType=0;

                            sendDisplay1306(false, true, 0, 0, (char*)"#N");

                            pagePointer--;
                            if(pagePointer < 0)
                                pagePointer=PAGE_MAX-1;

                            pageHold=5;
                        }
                    }

                    iPress = 0;
                }
            }
        }


    #endif
}

void sendDisplayPosition(struct aprsMessage &aprsmsg, int16_t rssi, int8_t snr)
{
    if(!bPosDisplay)
        return;

    if(bSetDisplay || pageHold > 0 || bDisplayTrack)
        return;

    bSetDisplay=true;

    if(bDisplayOff)
    {
        sendDisplay1306(true, true, 0, 0, (char*)"#C");
        bSetDisplay=false;
        return;
    }

    iDisplayType=1;

    char print_text[500];
    int ipt=0;

    int izeile=23;
    unsigned int itxt=0;
    int istarttext=0;

    double lat=0.0;
    double lon=0.0;

    int dir_to=0;
    int dist_to=0;

    aprsPosition aprspos;

    initAPRSPOS(aprspos);

    decodeAPRSPOS(aprsmsg.msg_payload, aprspos);

    // Display Distance, Direction
    lat = conv_coord_to_dec(aprspos.lat);
    lon = conv_coord_to_dec(aprspos.lon);

    dir_to = tinyGPSPLus.courseTo(meshcom_settings.node_lat, meshcom_settings.node_lon, lat, lon);

    dist_to = tinyGPSPLus.distanceBetween(lat, lon, meshcom_settings.node_lat, meshcom_settings.node_lon)/1000.0;

    sendDisplayMainline();

    sprintf(msg_text, "%s<>%s", aprsmsg.msg_source_call.c_str(), aprsmsg.msg_source_last.c_str());

    msg_text[20]=0x00;
    sendDisplay1306(false, false, 3, izeile, msg_text);

    izeile=izeile+12;

    ipt=0;

    for(itxt=0; itxt<aprsmsg.msg_payload.length(); itxt++)
    {
        if((aprsmsg.msg_payload.charAt(itxt) == 'N' || aprsmsg.msg_payload.charAt(itxt) == 'S'))
        {
            print_text[ipt]=0x00;

            sscanf(print_text, "%lf", &lat);

            sprintf(msg_text, "LAT: %s%c%5i°", print_text, aprsmsg.msg_payload.charAt(itxt), dir_to);
            msg_text[20]=0x00;
            sendDisplay1306(false, false, 3, izeile, msg_text);

            istarttext=itxt+2;
            izeile=izeile+12;
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

            sprintf(msg_text, "LON:%s%c%5ikm", print_text, aprsmsg.msg_payload.charAt(itxt), dist_to);
            msg_text[20]=0x00;
            sendDisplay1306(false, false, 3, izeile, msg_text);

            istarttext=itxt+3;
            izeile=izeile+10;
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

    // check Altitute
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

                    sprintf(msg_text, "ALT:%im rssi:%i", alt, rssi);
                    msg_text[20]=0x00;
                    sendDisplay1306(false, true, 3, izeile, msg_text);
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
    sprintf(currDate, "%04i-%02i-%02i", meshcom_settings.node_date_year, meshcom_settings.node_date_month, meshcom_settings.node_date_day);

    return (String)currDate;
}

String getTimeString()
{
    char currTime[10] = {0};
    sprintf(currTime, "%02i:%02i:%02i", meshcom_settings.node_date_hour, meshcom_settings.node_date_minute, meshcom_settings.node_date_second);

    return (String)currTime;
}

void printBuffer_aprs(char *msgSource, struct aprsMessage &aprsmsg)
{
    Serial.print(getTimeString());
    Serial.printf(" %s: %03i %c x%08X %02X %i %i %s>%s%c%s HW:%02i MOD:%02i FCS:%04X FW:%02X LH:%02X", msgSource, aprsmsg.msg_len, aprsmsg.payload_type, aprsmsg.msg_id, aprsmsg.max_hop,
        aprsmsg.msg_server, aprsmsg.msg_track, aprsmsg.msg_source_path.c_str(), aprsmsg.msg_destination_path.c_str(), aprsmsg.payload_type, aprsmsg.msg_payload.c_str(),
        aprsmsg.msg_source_hw, aprsmsg.msg_source_mod, aprsmsg.msg_fcs, aprsmsg.msg_source_fw_version, aprsmsg.msg_last_hw);
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

    if(memcmp(msg_text, "--", 1) == 0)
    {
        if(bDisplayInfo)
            Serial.printf("COMMAND:%s\n", msg_text);

        commandAction(msg_text, true);
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
        if(iCall != -1 && iCall < 11)   // {OE1KBC-99}Textmessage
        {
            strDestinationCall = strMsg.substring(1, iCall);
            strDestinationCall.toUpperCase();
            strDestinationCall.trim();
            strMsg = strMsg.substring(iCall+1);

            bDM=true;
        }
    }

    uint8_t msg_buffer[MAX_MSG_LEN_PHONE];

    struct aprsMessage aprsmsg;

    initAPRS(aprsmsg);

    aprsmsg.msg_len = 0;

    // MSG ID zusammen setzen    
    aprsmsg.msg_id = ((_GW_ID & 0x3FFFFF) << 10) | (meshcom_settings.node_msgid & 0x3FF);
    
    aprsmsg.payload_type = ':';
    aprsmsg.msg_source_path = meshcom_settings.node_call;
    aprsmsg.msg_destination_path = strDestinationCall;
    aprsmsg.msg_payload = strMsg;

    
    // ACK request anhängen
    if(bDM)
    {
        char cAckId[4] = {0};
        sprintf(cAckId, "%03i", meshcom_settings.node_msgid);
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

        if(bGATEWAY)
        {
            // gleich Wolke mit Hackerl setzen
            if(aprsmsg.msg_destination_path == "*")
            {
                uint8_t print_buff[8];

                print_buff[0]=0x41;
                print_buff[1]=aprsmsg.msg_id & 0xFF;
                print_buff[2]=(aprsmsg.msg_id >> 8) & 0xFF;
                print_buff[3]=(aprsmsg.msg_id >> 16) & 0xFF;
                print_buff[4]=(aprsmsg.msg_id >> 24) & 0xFF;
                print_buff[5]=0x01;     // switch ack GW / Node currently fixed to 0x00 
                print_buff[6]=0x00;     // msg always 0x00 at the end
                
                addBLEOutBuffer(print_buff, (uint16_t)7);
            }
        }
    }

    ringBuffer[iWrite][0]=aprsmsg.msg_len;
    memcpy(ringBuffer[iWrite]+1, msg_buffer, aprsmsg.msg_len);
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

    #ifdef ESP32
        // Extern Server
        if(bEXTUDP)
            sendExtern(true, (char*)"node", msg_buffer, aprsmsg.msg_len);

        if(bEXTSER)
            sendExtern(false, (char*)"node", msg_buffer, aprsmsg.msg_len);
    #endif
}

String PositionToAPRS(bool bConvPos, bool bWeather, bool bFuss, double lat, char lat_c, double lon, char lon_c, int alt,  float press, float hum, float temp, float temp2, int qfe, float qnh)
{
    if(lat == 0 or lon == 0)
    {
        DEBUG_MSG("APRS", "Error PositionToAPRS");
        return "";
    }

    char msg_start[100] = {0};

    // :|0x11223344|0x05|OE1KBC|>*:Hallo Mike, ich versuche eine APRS Meldung\0x00
    // 09:30:28 RX-LoRa: 105 ! xAE48E347 05 1 0 9V1LH-1,OE1KBC-12>*!0122.64N/10356.51E#/B=005/A=000272/P=1005.1/H=42.5/T=29.4/Q=1005.7 HW:04 MOD:03 FCS:15DC FW:17 LH:09
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
        sprintf(msg_start, "%02i%02i%02iz%07.2lf%c%c%08.2lf%c_", meshcom_settings.node_date_day, meshcom_settings.node_date_hour, meshcom_settings.node_date_minute, slat, lat_c, meshcom_settings.node_symid, slon, lon_c);
    else
    {
        char catxt[50]={0};
        char cbatt[15]={0};
        char calt[15]={0};
        char cpress[15]={0};
        char chum[15]={0};
        char ctemp[15]={0};
        char ctemp2[15]={0};
        char cqfe[15]={0};
        char cqnh[15]={0};

        if(strcmp(meshcom_settings.node_atxt, "none") != 0 && meshcom_settings.node_atxt[0] != 0x00)
        {
            sprintf(catxt, "%s ", meshcom_settings.node_atxt);
        }

        if(global_proz > 0)
        {
            sprintf(cbatt, "/B=%03d", global_proz);
        }

        if(alt > 0)
        {
            // auf Fuss umrechnen
            if(bFuss)
                sprintf(calt, "/A=%06i", conv_fuss(alt));
            else
                sprintf(calt, "/A=%05i", alt);
        }

        if(press > 0)
        {
            sprintf(cpress, "/P=%.1f", press);
        }

        if(hum > 0)
        {
            sprintf(chum, "/H=%.1f", hum);
        }

        if(temp != 0)
        {
            sprintf(ctemp, "/T=%.1f", temp);
        }

        if(temp2 != 0)
        {
            sprintf(ctemp2, "/O=%.1f", temp2);
        }

        if(qfe > 0)
        {
            sprintf(cqfe, "/F=%i", qfe);
        }

        if(qnh > 0)
        {
            sprintf(cqnh, "/Q=%.1f", qnh);
        }

        sprintf(msg_start, "%07.2lf%c%c%08.2lf%c%c%s%s%s%s%s%s%s%s%s", slat, lat_c, meshcom_settings.node_symid, slon, lon_c, meshcom_settings.node_symcd, catxt, cbatt, calt, cpress, chum, ctemp, ctemp2, cqfe, cqnh);
    }

    return String(msg_start);
}

void sendPosition(unsigned int intervall, double lat, char lat_c, double lon, char lon_c, int alt, float press, float hum, float temp, float temp2, int qfe, float qnh)
{
    uint8_t msg_buffer[MAX_MSG_LEN_PHONE];

    bool bSendViaAPRS = bDisplayTrack;

    if(posinfo_interval == 1800)    // wenn TRACK auf 30min steht
        bSendViaAPRS = false;

    if(lastHeardTime + 15000 < millis()) // wenn die letzte gehörte LoRa-Nachricht < 5sec dann auf jeden Fall über LoRaAPRS probieren
        bSendViaAPRS = true;

    #ifdef BOARD_TLORA_OLV216
        bSendViaAPRS = false;
    #endif

    // set default
    if(meshcom_settings.node_symid != '/' && meshcom_settings.node_symid != '\'')
    {
        meshcom_settings.node_symid = '/';
        meshcom_settings.node_symcd = '#';
    }
    else
    {
        if(meshcom_settings.node_symcd < '!' || meshcom_settings.node_symcd > '}')
            meshcom_settings.node_symcd = '#';
    }

    if(bSendViaAPRS)
    {
        //int ilng = encodeLoRaAPRS(msg_buffer, meshcom_settings.node_call, lat, lat_c, lon, lon_c, alt);
        int ilng = encodeLoRaAPRScompressed(msg_buffer, meshcom_settings.node_call, lat, lat_c, lon, lon_c, alt);

        if(bDisplayInfo)
        {
            Serial.print(getTimeString());
            Serial.printf(" LO-APRS:%s\n", msg_buffer+3);
        }

        ringBuffer[iWrite][0]=ilng;
        memcpy(ringBuffer[iWrite]+1, msg_buffer, ilng);
        iWrite++;
        if(iWrite >= MAX_RING)
            iWrite=0;
    }
    else
    {
        struct aprsMessage aprsmsg;

        initAPRS(aprsmsg);

        aprsmsg.msg_len = 0;

        // MSG ID zusammen setzen    
        aprsmsg.msg_id = ((_GW_ID & 0x3FFFFF) << 10) | (meshcom_settings.node_msgid & 0x3FF);

        aprsmsg.payload_type = '!';
        
        if(intervall != POSINFO_INTERVAL)
            aprsmsg.msg_track=true;

        aprsmsg.msg_source_path = meshcom_settings.node_call;
        aprsmsg.msg_destination_path = "*";
        aprsmsg.msg_payload = PositionToAPRS(true, false, true, lat, lat_c, lon, lon_c, alt, press, hum, temp, temp2, qfe, qnh);
        
        if(aprsmsg.msg_payload == "")
            return;

        meshcom_settings.node_msgid++;
        if(meshcom_settings.node_msgid > 255)
            meshcom_settings.node_msgid=0;
        // Flash rewrite
        save_settings();

        encodeAPRS(msg_buffer, aprsmsg);

        if(bDisplayInfo)
        {
            printBuffer_aprs((char*)"NEW-POS", aprsmsg);
            Serial.println();
        }

        ringBuffer[iWrite][0]=aprsmsg.msg_len;
        memcpy(ringBuffer[iWrite]+1, msg_buffer, aprsmsg.msg_len);
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

        #ifdef ESP32
            // Extern Server
            if(bEXTUDP)
                sendExtern(true, (char*)"node", msg_buffer, aprsmsg.msg_len);

            if(bEXTSER)
                sendExtern(false, (char*)"node", msg_buffer, aprsmsg.msg_len);
        #endif
    }

}

void SendAckMessage(String dest_call, unsigned int iAckId)
{
    struct aprsMessage aprsmsg;

    initAPRS(aprsmsg);

    aprsmsg.msg_len = 0;

    // MSG ID zusammen setzen    
    aprsmsg.msg_id = ((_GW_ID & 0x3FFFFF) << 10) | (meshcom_settings.node_msgid & 0x3FF);
    
    aprsmsg.payload_type = ':';
    aprsmsg.msg_source_path = meshcom_settings.node_call;
    aprsmsg.msg_destination_path = dest_call;

    char cackmsg[20];
    sprintf(cackmsg, "%-9.9s:ack%03i", dest_call.c_str(), iAckId);
    aprsmsg.msg_payload = cackmsg;

    meshcom_settings.node_msgid++;
    if(meshcom_settings.node_msgid > 999)
        meshcom_settings.node_msgid=0;
    // Flash rewrite
    save_settings();

    uint8_t msg_buffer[MAX_MSG_LEN_PHONE];
    
    encodeAPRS(msg_buffer, aprsmsg);

    if(bDisplayInfo)
    {
        printBuffer_aprs((char*)"NEW-ACK", aprsmsg);
        Serial.println();
    }

    ringBuffer[iWrite][0]=aprsmsg.msg_len;
    memcpy(ringBuffer[iWrite]+1, msg_buffer, aprsmsg.msg_len);
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
        gps_send_rate = POSINFO_INTERVAL;
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
            Serial.print(getTimeString());
            Serial.printf(" POSINFO one-shot set - direction_diff:%i last_lat:%.1lf last_lon:%.1lf\n", direction_diff, posinfo_last_lat, posinfo_last_lon);
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
    sprintf(cvar, "%.4lf", dvar);
    double rvar;
    sscanf(cvar, "%lf", &rvar);

    return rvar;
}

int conv_fuss(int alt_meter)
{
    double fuss = alt_meter * 10;
    fuss = fuss * 3.28084;
    int ifuss = fuss + 5;
    return ifuss / 10;
}

