#include "Arduino.h"

#include "loop_functions.h"
#include "command_functions.h"

bool bDEBUG = false;
bool bPosDisplay = true;
bool bDisplayOff = false;

// common variables
char msg_text[MAX_MSG_LEN_PHONE] = {0};

unsigned int _GW_ID = 0x12345678; // ID of our Node

#ifdef BOARD_HELTEC
    U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, 16, 15, 4);
#else
    U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);
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
uint8_t udpWrite = 0;   // counter for ringbuffer
uint8_t udpRead = 0;    // counter for ringbuffer

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

/**@brief Function adding messages into outgoing UDP ringbuffer
 * 
 */
void addLoraRxBuffer(unsigned int msg_id)
{
    // byte 0-3 msg_id
    ringBufferLoraRX[udpWrite][3] = msg_id >> 24;
    ringBufferLoraRX[udpWrite][2] = msg_id >> 16;
    ringBufferLoraRX[udpWrite][1] = msg_id >> 8;
    ringBufferLoraRX[udpWrite][0] = msg_id;

    if(bDEBUG)
    {
        Serial.printf("LoraRX Ringbuffer added element: %u\n", udpWrite);
        printBuffer(ringBufferLoraRX[udpWrite], 4);
    }

    udpWrite++;
    if (udpWrite >= MAX_RING_UDP_OUT) // if the buffer is full we start at index 0 -> take care of overwriting!
        udpWrite = 0;
}

void sendDisplay1306(bool bClear, bool bTransfer, int x, int y, char *text)
{
	// display bme680 sensor data on OLED
	if(bClear)
    {
        u8g2.clearBuffer();					// clear the internal memory
    }
	
    u8g2.setFont(u8g2_font_6x10_mf);    // u8g2_font_ncenB10_tr); // choose a suitable font

	if(memcmp(text, "#C", 2) == 0)
    {
        // only clear
    }
    else
	if(memcmp(text, "#L", 2) == 0)
    {
        //Serial.println("line");
    	u8g2.drawHLine(3, 16, 120);
    }
    else
    {
        u8g2.drawUTF8(x, y, text);
    }

    if(bTransfer)
    {
        //Serial.println("Transfer");
	    u8g2.sendBuffer(); // transfer internal memory to the display
    }
}

void sendDisplayHead(int batt)
{
    if(bDisplayOff)
    {
        sendDisplay1306(true, true, 0, 0, (char*)"#C");
        return;
    }

    char print_text[500];

    sprintf(print_text, "MC %-4.4s         %3d%%", SOURCE_VERSION, batt);
    sendDisplay1306(true, false, 3, 11, print_text);
    sendDisplay1306(false, false, 3, 9, (char*)"#L");

    sprintf(print_text, "Call:  %s", meshcom_settings.node_call);
    sendDisplay1306(false, false, 3, 27, print_text);

    sprintf(print_text, "Short: %s", meshcom_settings.node_short);
    sendDisplay1306(false, false, 3, 39, print_text);

    sprintf(print_text, "MAC:   %08X", _GW_ID);
    sendDisplay1306(false, false, 3, 51, print_text);

    sprintf(print_text, "Modul: %i", MODUL_HARDWARE);
    sendDisplay1306(false, true, 3, 63, print_text);
}

void sendDisplayMainline(int batt)
{
    if(bDisplayOff)
    {
        sendDisplay1306(true, true, 0, 0, (char*)"#C");
        return;
    }

    char print_text[500];

    if(meshcom_settings.node_date_hour == 0 && meshcom_settings.node_date_minute == 0 && meshcom_settings.node_date_second == 0)
    {
        sprintf(print_text, "MC %-4.4s         %3d%%", SOURCE_VERSION, batt);
    }
    else
    {
        sprintf(print_text, "MC%-4.4s %02i:%02i:%02i %3d%%", SOURCE_VERSION, meshcom_settings.node_date_hour, meshcom_settings.node_date_minute, meshcom_settings.node_date_second, batt); // (int)mv_to_percent(read_batt()));
    }

    sendDisplay1306(true, false, 3, 11, print_text);
    sendDisplay1306(false, true, 3, 9, (char*)"#L");
}

void sendDisplayText(struct aprsMessage &aprsmsg, int16_t rssi, int8_t snr)
{
    if(bDisplayOff)
    {
        bDisplayOff=false;
    }

    bPosDisplay=false;
    
    int izeile=13;
    unsigned int itxt=0;

    bool bClear=true;

    char line_text[21];
    char words[100][21]={0};
    unsigned int iwords=0;
    int ipos=0;

    if(aprsmsg.msg_source_path.length() < (20-7))
        sprintf(msg_text, "%s <%i>", aprsmsg.msg_source_path.c_str(), rssi);
    else
        sprintf(msg_text, "%s", aprsmsg.msg_source_path.c_str());

    msg_text[20]=0x00;
    sendDisplay1306(bClear, false, 3, izeile, msg_text);

    izeile=izeile+12;
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

            izeile=izeile+12;
            
            memset(line_text, 0x00, 21);

            if(izeile > 61)
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
}

void sendDisplayPosition(struct aprsMessage &aprsmsg, int16_t rssi, int8_t snr, int batt)
{
    if(bDisplayOff)
    {
        sendDisplay1306(true, true, 0, 0, (char*)"#C");
        return;
    }

    if(!bPosDisplay)
        return;

    char print_text[500];
    int ipt=0;

    int izeile=27;
    unsigned int itxt=0;
    int istarttext=0;


    bool bClear=false;

    sendDisplayMainline(batt);

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
        if(aprsmsg.msg_payload.charAt(itxt) == '/')
        {
            print_text[ipt]=0x00;

            //sprintf(msg_text, "BAT: %s", print_text);
            //msg_text[20]=0x00;
            //sendDisplay1306(bClear, false, 3, izeile, msg_text);

            istarttext=itxt+3;
            //izeile=izeile+12;
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
        if(itxt == aprsmsg.msg_payload.length()-1)
        {
            print_text[ipt]=aprsmsg.msg_payload.charAt(itxt);
            print_text[ipt+1]=0x00;

            sprintf(msg_text, "ALT: %sm rssi:%i", print_text, rssi);
            msg_text[20]=0x00;
            sendDisplay1306(bClear, true, 3, izeile, msg_text);

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

void printBuffer_aprs(char *msgSource, struct aprsMessage &aprsmsg)
{
    Serial.printf("%s: %03i %c x%08X %02X %i %s>%s%c%s HW:%02i MOD:%02i FCS:%04X V:%02X", msgSource, aprsmsg.msg_len, aprsmsg.payload_type, aprsmsg.msg_id, aprsmsg.max_hop, aprsmsg.msg_server, aprsmsg.msg_source_path.c_str(),
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

    if(memcmp(msg_text, "--", 2) == 0)
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
    
    if(strMsg.charAt(0) == '{')
    {
        int iCall = strMsg.indexOf('}');
        if(iCall != -1 && iCall < 11)   // {OE1KBC-99}Textmessage
        {
            strDestinationCall = strMsg.substring(1, iCall);
            strDestinationCall.toUpperCase();
            strMsg = strMsg.substring(iCall+1);
        }
    }

    uint8_t msg_buffer[MAX_MSG_LEN_PHONE];

    struct aprsMessage aprsmsg;

    initAPRS(aprsmsg);

    aprsmsg.msg_len = 0;
    aprsmsg.msg_id = millis();
    aprsmsg.payload_type = ':';
    aprsmsg.max_hop = 5;
    aprsmsg.msg_server = false;
    aprsmsg.msg_source_path = meshcom_settings.node_call;
    aprsmsg.msg_destination_path = strDestinationCall;
    aprsmsg.msg_payload = strMsg;

    encodeAPRS(msg_buffer, aprsmsg);

    printBuffer_aprs((char*)"TX-POS ", aprsmsg);
    Serial.println();

    // An APP als Anzeige retour senden
    if(hasMsgFromPhone)
    {
        addBLEOutBuffer(msg_buffer, aprsmsg.msg_len);
    }

    ringBuffer[iWrite][0]=aprsmsg.msg_len;
    memcpy(ringBuffer[iWrite]+1, msg_buffer, aprsmsg.msg_len);
    iWrite++;
    if(iWrite >= MAX_RING)
        iWrite=0;
    
    // store last message to compare later on
    memcpy(own_msg_id[iWriteOwn], msg_buffer+1, 4);
    own_msg_id[iWriteOwn][4]=0x00;
    iWriteOwn++;
    if(iWriteOwn >= MAX_RING)
        iWriteOwn=0;
}

String PositionToAPRS(bool bConvPos, bool bWeather, double lat, char lat_c, double lon, char lon_c, int alt, int batt)
{
    if(lat == 0 or lon == 0)
    {
        DEBUG_MSG("APRS", "Error PositionToAPRS");
        return "";
    }

    char msg_start[100];

    // :|0x11223344|0x05|OE1KBC|>*:Hallo Mike, ich versuche eine APRS Meldung\0x00

	double slat=lat*100.0;
	double slon=lon*100.0;
	
    if(bConvPos)
    {
        double slatr=0;
        double slonr=0;
        
        slat = (int)lat;
        slatr = (lat - slat)*60.;
        slat = (slat * 100.) + slatr;
        
        slon = (int)lon;
        slonr = (lon - slon)*60.;
        slon = (slon * 100.) + slonr;
    }

    if(bWeather)
        sprintf(msg_start, "%02i%02i%02iz%07.2lf%c%c%08.2lf%c_", meshcom_settings.node_date_day, meshcom_settings.node_date_hour, meshcom_settings.node_date_minute, slat, lat_c, meshcom_settings.node_symid, slon, lon_c);
    else
    {
        char cbatt[5]={0};
        char calt[11]={0};

        if(batt > 0)
            sprintf(cbatt, " %i", batt);

        if(alt > 0)
            sprintf(calt, " /A=%i", alt);

        sprintf(msg_start, "%07.2lf%c%c%08.2lf%c%c%s%s", slat, lat_c, meshcom_settings.node_symid, slon, lon_c, meshcom_settings.node_symcd, cbatt, calt);
    }

    
    return msg_start;
}

void sendPosition(double lat, char lat_c, double lon, char lon_c, int alt, int batt)
{
    uint8_t msg_buffer[MAX_MSG_LEN_PHONE];

    struct aprsMessage aprsmsg;

    initAPRS(aprsmsg);

    aprsmsg.msg_len = 0;
    aprsmsg.msg_id = millis();
    aprsmsg.payload_type = '!';
    aprsmsg.max_hop = 5;
    aprsmsg.msg_server = false;
    aprsmsg.msg_source_path = meshcom_settings.node_call;
    aprsmsg.msg_destination_path = "*";
    aprsmsg.msg_payload = PositionToAPRS(true, false, lat, lat_c, lon, lon_c, alt, batt);
    
    if(aprsmsg.msg_payload == "")
        return;

    encodeAPRS(msg_buffer, aprsmsg);

    printBuffer_aprs((char*)"TX-POS>", aprsmsg);
    Serial.println();

    ringBuffer[iWrite][0]=aprsmsg.msg_len;
    memcpy(ringBuffer[iWrite]+1, msg_buffer, aprsmsg.msg_len);
    iWrite++;
    if(iWrite >= MAX_RING)
        iWrite=0;

    
    // store last message to compare later on
    memcpy(own_msg_id[iWriteOwn], msg_buffer+1, 4);
    own_msg_id[iWriteOwn][4]=0x00;
    iWriteOwn++;
    if(iWriteOwn >= MAX_RING)
        iWriteOwn=0;

}

void sendWeather(double lat, char lat_c, double lon, char lon_c, int alt, float temp, float hum, float press, int batt)
{
    // @141324z4812.06N/01555.87E_270/...g...t...r000p000P...h..b10257Weather in Neulengbach

    uint8_t msg_buffer[MAX_MSG_LEN_PHONE];

    struct aprsMessage aprsmsg;

    initAPRS(aprsmsg);

    aprsmsg.msg_len = 0;
    aprsmsg.msg_id = millis();
    aprsmsg.payload_type = '@';
    aprsmsg.max_hop = 5;
    aprsmsg.msg_server = false;
    aprsmsg.msg_source_path = meshcom_settings.node_call;
    aprsmsg.msg_destination_path = "*";
    aprsmsg.msg_payload = PositionToAPRS(true, false, lat, lat_c, lon, lon_c, alt, batt);
    
    encodeAPRS(msg_buffer, aprsmsg);

    printBuffer_aprs((char*)"TX-WX >", aprsmsg);
    Serial.println();

    ringBuffer[iWrite][0]=aprsmsg.msg_len;

    memcpy(ringBuffer[iWrite]+1, msg_buffer, aprsmsg.msg_len);
    iWrite++;
    if(iWrite >= MAX_RING)
        iWrite=0;
    
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
