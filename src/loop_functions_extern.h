// (C) 2023 OE1KBC Kurt Baumann, OE1KFR Rainer 
// (C) 2016, 2017, 2018, 2018, 2019, 2020 OE1KBC Kurt Baumann
//
// This code is not for distribution and may not be published!
// Vervielfältigung und Veröffentlichung des Codes sind nicht gestattet!
// Lizenz: kein Open Source.
// zugelassen ausschließlich für OE1KBC, OE1KFR
// alle anderen Finger weg !
// 20230326: Version 4.00: START

extern int BOARD_HARDWARE;

extern unsigned long rebootAuto;

extern int iWlanWait;

extern bool bSetLoRaAPRS;

extern bool bDEBUG;
extern bool bLORADEBUG;
extern bool bBLEDEBUG;
extern bool bWXDEBUG;
extern bool bIODEBUG;

extern bool bPosDisplay;
extern bool bDisplayOff;
extern bool bDisplayVolt;
extern bool bDisplayInfo;
extern bool bDisplayCont;
extern bool bDisplayRetx;
extern unsigned long DisplayOffWait;
extern int DisplayTimeWait;
extern unsigned long BattTimeWait;
extern unsigned long BattTimeAPP;
extern unsigned long BMXTimeWait;
extern unsigned long MCU811TimeWait;
extern unsigned long INA226TimeWait;
extern bool bButtonCheck;
extern uint8_t iButtonPin;

extern bool bAnalogCheck;
extern float fAnalogValue;

extern bool bDisplayTrack;
extern bool bGPSON;
extern bool bBMPON;
extern bool bBMEON;
extern bool bBME680ON;
extern bool bMCU811ON;
extern bool bINA226ON;
extern bool bRTCON;
extern bool bSMALLDISPLAY;
extern bool bSOFTSERON;
extern bool bNoMSGtoALL;

extern bool bTCA9548A;
extern bool bMCP23017;

extern bool bONEWIRE;

extern bool bLPS33;

extern bool bme680_found;
extern bool bmx_found;
extern bool mcu811_found;
extern bool one_found;

extern bool bSHORTPATH;
extern bool bGPSDEBUG;
extern bool bSOFTSERDEBUG;

extern bool bBLElong;

extern bool bBOOSTEDGAIN;

extern bool bGATEWAY;
extern bool bGATEWAY_NOPOS;
extern bool bMESH;
extern bool bWEBSERVER;
extern bool bWIFIAP;
extern bool bEXTUDP;

extern float fBaseAltidude;
extern float fBasePress;

extern unsigned long onewireTimeWait;
extern unsigned long lastHeardTime;

extern unsigned long track_to_meshcom_timer;

// common variables
extern char msg_text[MAX_MSG_LEN_PHONE * 2];

extern unsigned int _GW_ID; // ID of our Node

extern unsigned int msg_counter;

extern uint8_t RcvBuffer[UDP_TX_BUF_SIZE * 2];

extern uint8_t own_msg_id[MAX_RING][5];

// TELEMTRY global variables
extern int iNextTelemetry;
extern String strTelemetry;

// RINGBUFFER for incoming UDP lora packets for lora TX
extern unsigned char ringBuffer[MAX_RING][UDP_TX_BUF_SIZE+5];
extern int iWrite;
extern int iRead;
extern int iRetransmit;

extern unsigned char ringbufferRAWLoraRX[MAX_LOG][UDP_TX_BUF_SIZE+5];
extern int RAWLoRaWrite;
extern int RAWLoRaRead;

// RINGBUFFER for outgoing UDP lora packets for lora TX
extern uint8_t ringBufferUDPout[MAX_RING_UDP][UDP_TX_BUF_SIZE+20];
extern uint8_t udpWrite;
extern uint8_t udpRead;

extern bool hasMsgFromPhone;

// BLE Ringbuffer to phone
extern unsigned char BLEtoPhoneBuff[MAX_RING][MAX_MSG_LEN_PHONE+5];
extern int toPhoneWrite;
extern int toPhoneRead;

// BLE Commands Ringbuffer to phone
extern unsigned char BLEComToPhoneBuff[MAX_RING][MAX_MSG_LEN_PHONE+5];
extern int ComToPhoneWrite;
extern int ComToPhoneRead;

extern uint8_t ringBufferLoraRX[MAX_RING][5]; //Ringbuffer for UDP TX from LoRa RX, first byte is length
extern uint8_t loraWrite;   // counter for ringbuffer

extern int cmd_counter; // ticker dependant on main cycle delay time
extern bool is_receiving;   // flag to store we are receiving a lora packet.
extern bool tx_is_active;   // flag to store we are transmitting  a lora packet.
extern bool tx_waiting;

extern int isPhoneReady;      // flag we receive from phone when itis ready to receive data
extern bool bPhoneTimeValid;
extern bool bNTPDateTimeValid;

// timers
extern unsigned long posinfo_interval; // check interval
extern unsigned long posfixinterall;
extern int posinfo_distance;
extern double posinfo_direction;
extern int posinfo_distance_ring[10];
extern int posinfo_ring_write;
extern double posinfo_lat;
extern double posinfo_lon;
extern double posinfo_last_lat;
extern double posinfo_last_lon;
extern double posinfo_last_direction;
extern uint32_t posinfo_satcount;
extern int posinfo_hdop;
extern bool posinfo_fix;
extern bool posinfo_shot;
extern bool pos_shot;
extern bool wx_shot;
extern int no_gps_reset_counter;

extern unsigned long posinfo_timer;      // we check periodically to send GPS
extern unsigned long heyinfo_timer;      // we check periodically to send HEY
extern unsigned long telemetry_timer;    // we check periodically to send TELEMETRY
extern unsigned long temphum_timer;      // we check periodically get TEMP/HUM
extern unsigned long druck_timer;        // we check periodically get AIRPRESURE
extern unsigned long hb_timer;           // GW Server-Heartbeat-Timer
extern unsigned long web_timer;          // Refreshtime WEbServer

// batt
extern float global_batt;
extern int global_proz;

extern unsigned char mheardBuffer[MAX_MHEARD][60]; //Ringbuffer for MHeard Lines
extern unsigned char mheardCalls[MAX_MHEARD][10]; //Ringbuffer for MHeard Key = Call
extern unsigned long mheardEpoch[MAX_MHEARD];  //Ringbuffer for MHeard EPoch Update Time

extern unsigned char mheardPathCalls[MAX_MHPATH][10]; //Ringbuffer for MHeard Key = Call
extern unsigned long mheardPathEpoch[MAX_MHPATH];  //Ringbuffer for MHeard EPoch Update Time
extern unsigned char mheardPathBuffer1[MAX_MHPATH][30]; //Ringbuffer for MHeard Sourcepath
extern uint8_t mheardPathLen[MAX_MHPATH];

extern char cTimeSource[10];

extern char cBLEName[50];

// SOFTSER global variables
extern String strSOFTSER_BUF;
extern bool bSOFTSER_APP;

extern String strSOFTSERAPP_ID;
extern String strSOFTSERAPP_PEGEL;
extern String strSOFTSERAPP_TEMP;
extern String strSOFTSERAPP_BATT;

extern String strSOFTSERAPP_FIXPEGEL;
extern String strSOFTSERAPP_FIXTEMP;

extern byte c1;