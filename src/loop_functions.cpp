#include "loop_functions.h"

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

uint8_t RcvBuffer[UDP_TX_BUF_SIZE];
uint8_t RcvBuffer_before[MAX_RING_UDP_OUT][4];

// RINGBUFFER for incoming UDP lora packets for lora TX
unsigned char ringBuffer[MAX_RING][UDP_TX_BUF_SIZE];
int iWrite=0;
int iRead=0;

bool hasMsgFromPhone = false;

// BLE Ringbuffer to phone
unsigned char BLEtoPhoneBuff[MAX_RING][UDP_TX_BUF_SIZE];
int toPhoneWrite=0;
int toPhoneRead=0;

uint8_t ringBufferUDPout[MAX_RING_UDP_OUT][UDP_TX_BUF_SIZE]; //Ringbuffer for UDP TX from LoRa RX, first byte is length
uint8_t udpWrite = 0;   // counter for ringbuffer
uint8_t udpRead = 0;    // counter for ringbuffer

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


void sendDisplay1306(bool bClear, bool bTransfer, int x, int y, char *text)
{
	// display bme680 sensor data on OLED
	if(bClear)
    {
        u8g2.clearBuffer();					// clear the internal memory
    }
	
    u8g2.setFont(u8g2_font_6x10_mf);    // u8g2_font_ncenB10_tr); // choose a suitable font

	if(memcmp(text, "L", 1) == 0)
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

void sendDisplayHead()
{

    char print_text[500];

    sprintf(print_text, "MC 4.0          %3d%%", (int)mv_to_percent(read_batt()));

    sendDisplay1306(true, false, 3, 11, print_text);
    sendDisplay1306(false, false, 3, 9, (char*)"L");

    sprintf(msg_text, "Call:  %s", meshcom_settings.node_call);
    sendDisplay1306(false, false, 3, 27, msg_text);

    sprintf(msg_text, "Short: %s", meshcom_settings.node_short);
    sendDisplay1306(false, false, 3, 40, msg_text);

    sprintf(msg_text, "MAC:   %08X", _GW_ID);
    sendDisplay1306(false, false, 3, 52, msg_text);

    sprintf(msg_text, "Modul: %i", MODUL_HARDWARE);
    sendDisplay1306(false, true, 3, 63, msg_text);
}

void sendDisplayMainline()
{
    sprintf(msg_text, "MC 4.0 %02i:%02i:%02i %3d%%", meshcom_settings.node_date_hour, meshcom_settings.node_date_minute, meshcom_settings.node_date_second, 100); // (int)mv_to_percent(read_batt()));

    sendDisplay1306(false, true, 3, 11, msg_text);
}

void sendDisplayText(uint8_t text[300], int size, int16_t rssi, int8_t snr)
{
    int izeile=13;
    int itxt=0;

    bool bClear=true;

    int istarttext=0;

    // Check Source-Call
    for(itxt=0; itxt<size; itxt++)
    {
        if(text[itxt] == ':' && itxt < 20)
        {
            text[itxt]=0x00;
            if(rssi != 0 && itxt < (20-7))
                sprintf(msg_text, "%s <%i>", text, rssi);
            else
                sprintf(msg_text, "%s", text);
            sendDisplay1306(bClear, true, 3, izeile, msg_text);
            izeile=izeile+12;
            istarttext=itxt+1;
            bClear=false;
            break;
        }
    }


    char line_text[20];
    char words[100][20];
    int iwords=0;
    int ipos=0;

    for(itxt=istarttext; itxt<size; itxt++)
    {
        words[iwords][ipos]=text[itxt];
        if(text[itxt] == ' ' || text[itxt] == '-')
        {
            words[iwords][ipos]=0x00;
            iwords++;
            ipos=0;
        }
        else
        if(ipos > 18)
        {
            words[iwords][19]=0x00;
            iwords++;
            words[iwords][0]=text[itxt];
            ipos=1;
        }
        else
            ipos++;
    }

    words[iwords][ipos]=0x00;
    iwords++;

    memset(line_text, 0x00, 20);
    strcat(line_text, words[0]);

    bool bEnd=false;

    for(itxt=1; itxt<iwords; itxt++)
    {
        if((strlen(line_text) + strlen(words[itxt])) > 18)
        {
            line_text[19]=0x00;
            sprintf(msg_text, "%s", line_text);

            if(izeile > 60)
                bEnd=true;

            if(bEnd && itxt < iwords)
                sprintf(msg_text, "%-17.17s...", line_text);

            sendDisplay1306(bClear, bEnd, 3, izeile, msg_text);
            izeile=izeile+12;
            
            memset(line_text, 0x00, 20);

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
        line_text[19]=0x00;
        sprintf(msg_text, "%s", line_text);
        //Serial.printf("1306-02:%s len:%i izeile:%i\n", msg_text, strlen(msg_text), izeile);
        sendDisplay1306(bClear, bEnd, 3, izeile, msg_text);
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

/** @brief Method to print our buffers
 */
void printBuffer_ascii(uint8_t *buffer, int len)
{
  int i=0;

  Serial.printf("%03i ", len);

  Serial.printf("%c x", buffer[0]);
  for (i = 4; i > 0; i--)
  {
    Serial.printf("%02X", buffer[i]);
  }
  Serial.printf(" %02X ", buffer[5]);

  int ineg=0;

  for (i = 6; i < len; i++)
  {
    if(buffer[i] == 0x00)
    {
        ineg=i;
        break;
    }

    Serial.printf("%c", buffer[i]);
  }

  Serial.printf(" %02X", buffer[ineg]);
  Serial.printf("%02X", buffer[ineg+1]);

  Serial.println("");
}

///////////////////////////////////////////////////////////////////////////
// APRS Meldungen

int CallToAPRS(char msg_type, uint8_t msg_buffer[MAX_MSG_LEN_PHONE])
{
    char msg_start[100];

    // :|0x11223344|0x05|OE1KBC|>*:Hallo Mike, ich versuche eine APRS Meldung\0x00

    msg_buffer[0]=msg_type;
    
    msg_counter=millis();

    msg_buffer[1]=msg_counter & 0xff;
    msg_buffer[2]=(msg_counter >> 8) & 0xff;
    msg_buffer[3]=(msg_counter >> 16) & 0xff;
    msg_buffer[4]=(msg_counter >> 24) & 0xff;

    msg_buffer[5]=0x05; //max hop

    sprintf(msg_start, "%s>*", meshcom_settings.node_call);

    memcpy(msg_buffer+6, msg_start, strlen(meshcom_settings.node_call)+2);

    int inext=6+2+strlen(meshcom_settings.node_call);

    msg_buffer[inext] = msg_type;
    inext++;

    return inext;
}

void sendMessage(char *msg_text, int len)
{
    uint8_t msg_buffer[MAX_MSG_LEN_PHONE];

    int inext = CallToAPRS(':', msg_buffer);
        
    if(inext == 0)
        return;

    if(memcmp(msg_text, ":", 1) == 0)
    {
        memcpy(msg_buffer+inext, msg_text+1, len-1);
        inext=inext+len-1;
    }
    else
    {
        memcpy(msg_buffer+inext, msg_text, len);
        inext=inext+len;
    }
   
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

    msg_buffer[inext] = MODUL_HARDWARE;
    inext++;

    // An APP als Anzeige retour senden
    if(hasMsgFromPhone)
    {
        addBLEOutBuffer(msg_buffer, inext);
    }

    if(bDEBUG)
    {
        printBuffer(msg_buffer, inext);
        Serial.println("");
    }

    if(inext > UDP_TX_BUF_SIZE)
        inext = UDP_TX_BUF_SIZE;

    ringBuffer[iWrite][0]=inext;
    memcpy(ringBuffer[iWrite]+1, msg_buffer, inext);
    
    // store last message to compare later on
    memcpy(RcvBuffer_before[iWrite], msg_buffer+1, inext);

    iWrite++;
    if(iWrite >= MAX_RING)
        iWrite=0;

}

int PositionToAPRS(uint8_t msg_buffer[MAX_MSG_LEN_PHONE], bool bConvPos, bool bWeather, double lat, char lat_c, double lon, char lon_c, int alt, int batt)
{
    if(lat == 0 or lon == 0)
    {
        DEBUG_MSG("APRS", "Error PositionToAPRS");
        return 0;
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
        sprintf(msg_start, "%07.2lf%c%c%08.2lf%c%c %i /A=%i", slat, lat_c, meshcom_settings.node_symid, slon, lon_c, meshcom_settings.node_symcd, batt, alt);

    memcpy(msg_buffer, msg_start, strlen(msg_start));

    int inext=strlen(msg_start);
    
    return inext;
}

void sendPosition(double lat, char lat_c, double lon, char lon_c, int alt, int batt)
{
    uint8_t msg_buffer[MAX_MSG_LEN_PHONE];

    int inext = CallToAPRS('!', msg_buffer);

    int inext_pos = PositionToAPRS(msg_buffer+inext, true, false, lat, lat_c, lon, lon_c, alt, batt);
        
    if(inext_pos == 0)
        return;

    inext = inext + inext_pos;

    msg_buffer[inext] = 0x00;
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

    msg_buffer[inext] = MODUL_HARDWARE;
    inext++;

    if(bDEBUG)
    {
        printBuffer_ascii(msg_buffer, inext);
    }

    if(inext > UDP_TX_BUF_SIZE)
        inext = UDP_TX_BUF_SIZE;

    ringBuffer[iWrite][0]=inext;

    memcpy(ringBuffer[iWrite]+1, msg_buffer, inext);
    
    // store last message to compare later on
    memcpy(RcvBuffer_before[iWrite], msg_buffer+1, inext);

    iWrite++;
    if(iWrite >= MAX_RING)
        iWrite=0;

}

void sendWeather(double lat, char lat_c, double lon, char lon_c, int alt, float temp, float hum, float press)
{
    // @141324z4812.06N/01555.87E_270/...g...t...r000p000P...h..b10257Weather in Neulengbach

    uint8_t msg_buffer[MAX_MSG_LEN_PHONE];

    int inext = CallToAPRS('@', msg_buffer);

    int inext_pos = PositionToAPRS(msg_buffer+inext, true, true, lat, lat_c, lon, lon_c, 0, 0);
        
    if(inext_pos == 0)
        return;

    inext = inext + inext_pos;

    char msg_start[100];

    sprintf(msg_start, ".../...g...t%03ir...p...P...h%02ib%05ixMESH", (int)((temp*1.8)+32.0), (int)(hum), (int)(press*10.0));

    memcpy(msg_buffer+inext, msg_start, strlen(msg_start));

    inext=inext+strlen(msg_start);

    msg_buffer[inext] = 0x00;
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

    msg_buffer[inext] = MODUL_HARDWARE;
    inext++;

    if(bDEBUG)
    {
        printBuffer_ascii(msg_buffer, inext);
    }

    ringBuffer[iWrite][0]=inext;
    memcpy(ringBuffer[iWrite]+1, msg_buffer, inext);
    
    // store last message to compare later on
    memcpy(RcvBuffer_before[iWrite], msg_buffer+1, inext);

    iWrite++;
    if(iWrite >= MAX_RING)
        iWrite=0;
}


void sendWX(char* text, float temp, float hum, float press)
{
    char msg_wx[200];

    //sprintf(msg_wx, "%s, %.1f °C, %.1f hPa, %i %%", text, temp, press, (int)(hum));
    sprintf(msg_wx, "%s, %.1f hPa, hum %i %%", text, press, (int)(hum));

    msg_wx[0] = ':';

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
