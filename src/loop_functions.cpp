#include "Arduino.h"

#include "loop_functions.h"
#include "command_functions.h"

#include "clock.h"

#include "batt_functions.h"
#include "udp_functions.h"
#include "configuration.h"

extern unsigned long rebootAuto;

extern float global_batt;

bool bDEBUG = false;
bool bPosDisplay = true;
bool bDisplayOff = false;
bool bDisplayVolt = false;
bool bDisplayInfo = true;
unsigned long DisplayOffWait = 0;

int iDisplayType = 0;
int DisplayTimeWait = 0;

bool bButton_Press = false;
bool bWaitButton_Released = false;

// common variables
char msg_text[MAX_MSG_LEN_PHONE] = {0};

unsigned int _GW_ID = 0x12345678; // ID of our Node

#if defined(BOARD_HELTEC)
    U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, 16, 15, 4);
#elif defined(BOARD_HELTEC_V3)
    U8G2_SSD1306_128X64_NONAME_1_SW_I2C u8g2(U8G2_R0, 18, 17, 21);
#else
    U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);  //RESET CLOCK DATA
#endif

#if defined(ESP8266) || defined(ESP32)
#else
    s_meshcom_settings meshcom_settings;
#endif

unsigned int msg_counter = 0;

uint8_t RcvBuffer[UDP_TX_BUF_SIZE] = {0};

// nur eigene msg_id
uint8_t own_msg_id[MAX_RING][5] = {0};
int iWriteOwn=0;

// RINGBUFFER for incoming UDP lora packets for lora TX
unsigned char ringBuffer[MAX_RING][UDP_TX_BUF_SIZE] = {0};
int iWrite=0;
int iRead=0;

bool hasMsgFromPhone = false;

// BLE Ringbuffer to phone
unsigned char BLEtoPhoneBuff[MAX_RING][UDP_TX_BUF_SIZE] = {0};
int toPhoneWrite=0;
int toPhoneRead=0;

uint8_t ringBufferLoraRX[MAX_RING_UDP_OUT][4] = {0}; //Ringbuffer for UDP TX from LoRa RX, first byte is length
uint8_t loraWrite = 0;   // counter for ringbuffer

// LoRa RX/TX sequence control
int cmd_counter = 2;      // ticker dependant on main cycle delay time
bool is_receiving = false;  // flag to store we are receiving a lora packet.
bool tx_is_active = false;  // flag to store we are transmitting  a lora packet.

uint8_t isPhoneReady = 0;      // flag we receive from phone when itis ready to receive data

// timers
unsigned long posinfo_timer = 0;      // we check periodically to send GPS
unsigned long temphum_timer = 0;      // we check periodically get TEMP/HUM
unsigned long druck_timer = 0;        // we check periodically get AIRPRESURE

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
        Serial.printf("BLEtoPhone RingBuff added element: %u\n", toPhoneWrite);

    if(bDEBUG)
    {
        printBuffer(BLEtoPhoneBuff[toPhoneWrite], len + 1);
    }

    toPhoneWrite++;
    if (toPhoneWrite >= MAX_RING_UDP_OUT) // if the buffer is full we start at index 0 -> take care of overwriting!
        toPhoneWrite = 0;
}

void addBLECommandBack(char text[100])
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

    if(bDEBUG)
    {
        Serial.printf("LoraRX Ringbuffer added element: %u\n", loraWrite);
        printBuffer(ringBufferLoraRX[loraWrite], 4);
    }

    loraWrite++;
    if (loraWrite >= MAX_RING_UDP_OUT) // if the buffer is full we start at index 0 -> take care of overwriting!
        loraWrite = 0;
}

int pageLine[10][3] = {0};
char pageText[10][30] = {0};
int pageLineAnz=0;
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

	if(memcmp(text, "#C", 2) == 0)
    {
    }
    else
    {
        if(pageLineAnz < 10 && strlen(text) < 30)
        {
            //Serial.printf("pageLineAnz:%i text:%s\n", pageLineAnz, text);

            pageLine[pageLineAnz][0] = x;
            pageLine[pageLineAnz][1] = y;
            pageLine[pageLineAnz][2] = 20;
            memcpy(pageText[pageLineAnz], text, 20);
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
    }
}

void sendDisplayHead()
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

    sendDisplayMainline(); // no extra Display

    sprintf(print_text, "Call:  %s", meshcom_settings.node_call);
    sendDisplay1306(false, false, 3, 23, print_text);

    sprintf(print_text, "Short: %s", meshcom_settings.node_short);
    sendDisplay1306(false, false, 3, 33, print_text);

    sprintf(print_text, "MAC:   %08X", _GW_ID);
    sendDisplay1306(false, false, 3, 43, print_text);

    sprintf(print_text, "Modul: %i", MODUL_HARDWARE);
    sendDisplay1306(false, false, 3, 53, print_text);

    sprintf(print_text, "ssid:  %-15.15s", meshcom_settings.node_ssid);
    sendDisplay1306(false, true, 3, 63, print_text);

    bSetDisplay=false;
}

void sendDisplayTime()
{
    if(bDisplayOff)
        return;

    if(iDisplayType == 0)
        return;

    if(bSetDisplay)
        return;

    bSetDisplay = true;

    char print_text[500];
    char cbatt[5];

    if(bDisplayVolt)
        sprintf(cbatt, "%4.2f", global_batt/1000.0);
    else
        sprintf(cbatt, "%3d%%", mv_to_percent(global_batt));

    sprintf(print_text, "%-2.2s%-4.4s %02i:%02i:%02i %-4.4s", SOURCE_TYPE, SOURCE_VERSION, meshcom_settings.node_date_hour, meshcom_settings.node_date_minute, meshcom_settings.node_date_second, cbatt);

    memcpy(pageText[0], print_text, 20);

    u8g2.setCursor(pageLine[0][0], pageLine[0][1]);
    u8g2.print(print_text);
    u8g2.sendBuffer();

    bSetDisplay = false;
}

void sendDisplayMainline()
{
    char print_text[500];
    char cbatt[5];

    if(bDisplayVolt)
        sprintf(cbatt, "%4.2f", global_batt/1000.0);
    else
        sprintf(cbatt, "%3d%%", mv_to_percent(global_batt));

    if(meshcom_settings.node_date_hour == 0 && meshcom_settings.node_date_minute == 0 && meshcom_settings.node_date_second == 0)
    {
        sprintf(print_text, "%-2.2s %-4.4s         %-4.4s", SOURCE_TYPE, SOURCE_VERSION, cbatt);
    }
    else
    {
        sprintf(print_text, "%-2.2s%-4.4s %02i:%02i:%02i %-4.4s", SOURCE_TYPE, SOURCE_VERSION, meshcom_settings.node_date_hour, meshcom_settings.node_date_minute, meshcom_settings.node_date_second, cbatt);
    }

    sendDisplay1306(true, false, 3, 10, print_text);
    sendDisplay1306(false, true, 3, 12, (char*)"#L");
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
        
        MyClock.setCurrentTime(Year, Month, Day, Hour, Minute, Second);

        Serial.println("");
        Serial.print(getTimeString());
        Serial.print(" TIMESET: Time set ");

        bPosDisplay=true;

        return;
    }
    else
    {
        if(!bDisplayVolt)
            bPosDisplay=false;
    }

    if(bSetDisplay)
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

// BUTTON
void initButtonPin()
{
    #ifdef BUTTON_PIN
        pinMode(BUTTON_PIN, INPUT);
    #endif
}

void checkButtonState()
{
    #ifdef BUTTON_PIN
        if(digitalRead(BUTTON_PIN) == 0)
        {
            bButton_Press = true;
            return;
        }
        else
        if(bButton_Press == true)
        {
            bDisplayOff=!bDisplayOff;
            //Serial.printf("Display:%i\n", bDisplayOff);
            if(!bDisplayOff)
                sendDisplayHead();

            bButton_Press = false;
        }
    #endif
}

void sendDisplayPosition(struct aprsMessage &aprsmsg, int16_t rssi, int8_t snr)
{
    if(!bPosDisplay)
        return;

    if(bSetDisplay)
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


    bool bClear=false;

    sendDisplayMainline(); // no extra Display

    sprintf(msg_text, "%s", aprsmsg.msg_source_path.c_str());

    msg_text[20]=0x00;
    sendDisplay1306(bClear, false, 3, izeile, msg_text);

    izeile=izeile+12;
    bClear=false;

    ipt=0;

    for(itxt=0; itxt<aprsmsg.msg_payload.length(); itxt++)
    {
        if((aprsmsg.msg_payload.charAt(itxt) == 'N' || aprsmsg.msg_payload.charAt(itxt) == 'S'))
        {
            print_text[ipt]=0x00;

            sprintf(msg_text, "LAT:  %s %c", print_text, aprsmsg.msg_payload.charAt(itxt));
            msg_text[20]=0x00;
            sendDisplay1306(bClear, false, 3, izeile, msg_text);

            istarttext=itxt+2;
            izeile=izeile+12;
            bClear=false;
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

            sprintf(msg_text, "LON: %s %c", print_text, aprsmsg.msg_payload.charAt(itxt));
            msg_text[20]=0x00;
            sendDisplay1306(bClear, false, 3, izeile, msg_text);

            istarttext=itxt+3;
            izeile=izeile+10;
            bClear=false;
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

                    if(aprsmsg.msg_fw_version > 13)
                        alt = (int)((float)alt * 0.3048);

                    sprintf(msg_text, "ALT: %im rssi:%i", alt, rssi);
                    msg_text[20]=0x00;
                    sendDisplay1306(bClear, true, 3, izeile, msg_text);
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
    Serial.printf(" %s: %03i %c x%08X %02X %i %s>%s%c%s HW:%02i MOD:%02i FCS:%04X V:%02X", msgSource, aprsmsg.msg_len, aprsmsg.payload_type, aprsmsg.msg_id, aprsmsg.max_hop, aprsmsg.msg_server, aprsmsg.msg_source_path.c_str(),
        aprsmsg.msg_destination_path.c_str(), aprsmsg.payload_type, aprsmsg.msg_payload.c_str(), aprsmsg.msg_source_hw, aprsmsg.msg_source_mod, aprsmsg.msg_fcs, aprsmsg.msg_fw_version);
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
        commandAction(msg_text, len, true);
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
            strMsg = strMsg.substring(iCall+1);

            bDM=true;
        }
    }

    uint8_t msg_buffer[MAX_MSG_LEN_PHONE];

    struct aprsMessage aprsmsg;

    initAPRS(aprsmsg);

    aprsmsg.msg_len = 0;

    // MSG ID zusammen setzen    
    unsigned int iAckId = millis();
    aprsmsg.msg_id = ((_GW_ID & 0xFFFF) << 16) | (iAckId & 0xFFFF);
    
    aprsmsg.payload_type = ':';
    aprsmsg.max_hop = 5;
    aprsmsg.msg_server = false;
    aprsmsg.msg_source_path = meshcom_settings.node_call;
    aprsmsg.msg_destination_path = strDestinationCall;
    aprsmsg.msg_payload = strMsg;

    
    // ACK request anhängen
    if(bDM)
    {
        unsigned int iAckId = millis();
        aprsmsg.msg_id = ((_GW_ID & 0xFFFF) << 16) | (iAckId & 0xFF);

        iAckId = aprsmsg.msg_id & 0xFF;
        char cAckId[4] = {0};
        sprintf(cAckId, "%03i", iAckId);
        aprsmsg.msg_payload = strMsg + "{" + String(cAckId);
    }

    encodeAPRS(msg_buffer, aprsmsg);

    printBuffer_aprs((char*)"NEW-TXT", aprsmsg);
    Serial.println();

    // An APP als Anzeige retour senden
    if(hasMsgFromPhone)
    {
        addBLEOutBuffer(msg_buffer, aprsmsg.msg_len);

        #if defined GATEWAY_TYPE
            // gleich Wolke mit Hackler setzen
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
        #endif
    }

    ringBuffer[iWrite][0]=aprsmsg.msg_len;
    memcpy(ringBuffer[iWrite]+1, msg_buffer, aprsmsg.msg_len);
    iWrite++;
    if(iWrite >= MAX_RING)
        iWrite=0;
    
    #if defined GATEWAY_TYPE
		// UDP out
		addNodeData(msg_buffer, aprsmsg.msg_len, 0, 0);
    #endif

    // store last message to compare later on
    memcpy(own_msg_id[iWriteOwn], msg_buffer+1, 4);
    own_msg_id[iWriteOwn][4]=0x00;
    iWriteOwn++;
    if(iWriteOwn >= MAX_RING)
        iWriteOwn=0;
}

String PositionToAPRS(bool bConvPos, bool bWeather, double lat, char lat_c, double lon, char lon_c, int alt)
{
    if(lat == 0 or lon == 0)
    {
        DEBUG_MSG("APRS", "Error PositionToAPRS");
        return "";
    }

    char msg_start[100] = {0};

    // :|0x11223344|0x05|OE1KBC|>*:Hallo Mike, ich versuche eine APRS Meldung\0x00

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
        int proz = mv_to_percent(global_batt);
        
        char cbatt[8]={0};
        char calt[15]={0};

        if(proz > 0)
            sprintf(cbatt, " /B=%03i", proz);

        if(alt > 0)
        {
            // auf Fuss umrechnen
            sprintf(calt, " /A=%06i", conv_fuss(alt));
        }

        sprintf(msg_start, "%07.2lf%c%c%08.2lf%c%c%s%s", slat, lat_c, meshcom_settings.node_symid, slon, lon_c, meshcom_settings.node_symcd, cbatt, calt);
    }

    return String(msg_start);
}

void sendPosition(double lat, char lat_c, double lon, char lon_c, int alt)
{
    uint8_t msg_buffer[MAX_MSG_LEN_PHONE];

    struct aprsMessage aprsmsg;

    initAPRS(aprsmsg);

    aprsmsg.msg_len = 0;

    // MSG ID zusammen setzen    
    unsigned int iAckId = millis();
    aprsmsg.msg_id = ((_GW_ID & 0xFFFF) << 16) | (iAckId & 0xFFFF);

    aprsmsg.payload_type = '!';
    aprsmsg.max_hop = 5;
    aprsmsg.msg_server = false;
    aprsmsg.msg_source_path = meshcom_settings.node_call;
    aprsmsg.msg_destination_path = "*";
    aprsmsg.msg_payload = PositionToAPRS(true, false, lat, lat_c, lon, lon_c, alt);
    
    if(aprsmsg.msg_payload == "")
        return;

    encodeAPRS(msg_buffer, aprsmsg);

    printBuffer_aprs((char*)"NEW-POS", aprsmsg);
    Serial.println();

    ringBuffer[iWrite][0]=aprsmsg.msg_len;
    memcpy(ringBuffer[iWrite]+1, msg_buffer, aprsmsg.msg_len);
    iWrite++;
    if(iWrite >= MAX_RING)
        iWrite=0;

    #if defined GATEWAY_TYPE
		// UDP out
		addNodeData(msg_buffer, aprsmsg.msg_len, 0, 0);
    #endif
    
    // store last message to compare later on
    memcpy(own_msg_id[iWriteOwn], msg_buffer+1, 4);
    own_msg_id[iWriteOwn][4]=0x00;
    iWriteOwn++;
    if(iWriteOwn >= MAX_RING)
        iWriteOwn=0;

}

void sendWeather(double lat, char lat_c, double lon, char lon_c, int alt, float temp, float hum, float press)
{
    // @141324z4812.06N/01555.87E_270/...g...t...r000p000P...h..b10257Weather in Neulengbach

    uint8_t msg_buffer[MAX_MSG_LEN_PHONE];

    struct aprsMessage aprsmsg;

    initAPRS(aprsmsg);

    aprsmsg.msg_len = 0;

    // MSG ID zusammen setzen    
    unsigned int iAckId = millis();
    aprsmsg.msg_id = ((_GW_ID & 0xFFFF) << 16) | (iAckId & 0xFFFF);

    aprsmsg.payload_type = '@';
    aprsmsg.max_hop = 5;
    aprsmsg.msg_server = false;
    aprsmsg.msg_source_path = meshcom_settings.node_call;
    aprsmsg.msg_destination_path = "*";
    aprsmsg.msg_payload = PositionToAPRS(true, false, lat, lat_c, lon, lon_c, alt);
    
    encodeAPRS(msg_buffer, aprsmsg);

    printBuffer_aprs((char*)"NEW-WX ", aprsmsg);
    Serial.println();

    ringBuffer[iWrite][0]=aprsmsg.msg_len;

    memcpy(ringBuffer[iWrite]+1, msg_buffer, aprsmsg.msg_len);
    iWrite++;
    if(iWrite >= MAX_RING)
        iWrite=0;
    
    #if defined GATEWAY_TYPE
		// UDP out
		addNodeData(msg_buffer, aprsmsg.msg_len, 0, 0);
    #endif

    // store last message to compare later on
    memcpy(own_msg_id[iWriteOwn], msg_buffer+1, 4);
    own_msg_id[iWriteOwn][4]=0x00;
    iWriteOwn++;
    if(iWriteOwn >= MAX_RING)
        iWriteOwn=0;
}


void sendWX(char* text, float temp, float hum, float press)
{
    char msg_wx[200];

    //sprintf(msg_wx, "%s, %.1f °C, %.1f hPa, %i %%", text, temp, press, (int)(hum));
    sprintf(msg_wx, "%s, %.1f hPa, hum %i %%", text, press, (int)(hum));

    sendMessage(msg_wx, strlen(msg_wx));
}

void SendAckMessage(String dest_call, unsigned int iAckId)
{
    struct aprsMessage aprsmsg;

    initAPRS(aprsmsg);

    aprsmsg.msg_len = 0;

    // MSG ID zusammen setzen    
    aprsmsg.msg_id = ((_GW_ID & 0xFFFF) << 16) | (iAckId & 0xFF);
    
    aprsmsg.payload_type = ':';
    aprsmsg.max_hop = 5;
    aprsmsg.msg_server = false;
    aprsmsg.msg_source_path = meshcom_settings.node_call;
    aprsmsg.msg_destination_path = dest_call;

    char cackmsg[20];
    sprintf(cackmsg, "%-9.9s:ack%03i", dest_call.c_str(), iAckId);
    aprsmsg.msg_payload = cackmsg;

    uint8_t msg_buffer[MAX_MSG_LEN_PHONE];
    
    encodeAPRS(msg_buffer, aprsmsg);

    printBuffer_aprs((char*)"NEW-ACK", aprsmsg);

    ringBuffer[iWrite][0]=aprsmsg.msg_len;
    memcpy(ringBuffer[iWrite]+1, msg_buffer, aprsmsg.msg_len);
    iWrite++;
    if(iWrite >= MAX_RING)
        iWrite=0;
    
    #if defined GATEWAY_TYPE
		// UDP out
		addNodeData(msg_buffer, aprsmsg.msg_len, 0, 0);
    #endif

    // store last message to compare later on
    memcpy(own_msg_id[iWriteOwn], msg_buffer+1, 4);
    own_msg_id[iWriteOwn][4]=0x00;
    iWriteOwn++;
    if(iWriteOwn >= MAX_RING)
        iWriteOwn=0;
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

double cround4(float dvar)
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