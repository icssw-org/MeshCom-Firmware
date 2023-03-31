// (C) 2023 OE1KBC Kurt Baumann, OE1KFR Rainer 
// (C) 2016, 2017, 2018, 2018, 2019, 2020 OE1KBC Kurt Baumann
//
// This code is not for distribution and may not be published!
// Vervielfältigung und Veröffentlichung des Codes sind nicht gestattet!
// Lizenz: kein Open Source.
// zugelassen ausschließlich für OE1KBC, OE1KFR
// alle anderen Finger weg !
// 20230326: Version 4.00: START

extern bool bDEBUG;

extern s_meshcom_settings meshcom_settings;

// common variables
extern char msg_text[MAX_MSG_LEN_PHONE];

extern unsigned int _GW_ID; // ID of our Node

#ifdef BOARD_HELTEC
    extern U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2;
#else
    extern U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2;
#endif

extern unsigned int msg_counter;

extern uint8_t RcvBuffer[UDP_TX_BUF_SIZE];
extern uint8_t RcvBuffer_before[MAX_RING_UDP_OUT][4];

// RINGBUFFER for incoming UDP lora packets for lora TX
extern unsigned char ringBuffer[MAX_RING][UDP_TX_BUF_SIZE];
extern int iWrite;
extern int iRead;

extern bool hasMsgFromPhone;

// BLE Ringbuffer to phone
extern unsigned char BLEtoPhoneBuff[MAX_RING][UDP_TX_BUF_SIZE];
extern int toPhoneWrite;
extern int toPhoneRead;

extern uint8_t ringBufferLoraRX[MAX_RING_UDP_OUT][4]; //Ringbuffer for UDP TX from LoRa RX, first byte is length
extern uint8_t udpWrite;   // counter for ringbuffer
extern uint8_t udpRead;    // counter for ringbuffer

