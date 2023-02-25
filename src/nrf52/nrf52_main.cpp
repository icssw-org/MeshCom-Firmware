#include <Arduino.h>
#include <SPI.h>
#include <configuration.h>
#include <SX126x-RAK4630.h>
#include <debugconf.h>

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


// Ethernet Object
//NrfETH neth;

// LoRa Events and Buffers
static RadioEvents_t RadioEvents;
static uint8_t RcvBuffer[UDP_TX_BUF_SIZE];
static uint8_t RcvBuffer_before[UDP_TX_BUF_SIZE];
static uint8_t txBuffer[UDP_TX_BUF_SIZE]; // we need an extra buffer for udp tx, as we add other stuff (ID, RSSI, SNR, MODE)
uint8_t lora_tx_buffer[UDP_TX_BUF_SIZE];  // lora tx buffer
static uint8_t ringBufferUDPout[MAX_RING_UDP_OUT][UDP_TX_BUF_SIZE]; //Ringbuffer for UDP TX from LoRa RX, first byte is length
static uint8_t ringBufferLoraInUDPout[MAX_RING_UDP_OUT][UDP_TX_BUF_SIZE]; //Ringbuffer for UDP TX from LoRa RX, first byte is length
uint8_t udpWrite = 0;   // counter for ringbuffer
uint8_t udpRead = 0;    // counter for ringbuffer

//variables and helper functions
int sendlng = 0;              // lora tx message length
uint8_t cmd_counter = 2;      // ticker dependant on main cycle delay time
void print_radioStatus(void); // prints the current Radio Status
uint8_t preamble_cnt = 0;     // stores how often a preamble detect is thrown
bool tx_is_active = false;    // avoids calling doTX() on each main iteration when we are already in TX mode
bool is_receiving = false;  // flag to store we are receiving a lora packet. triggered by header detect not preamble
uint8_t err_cnt_udp_tx = 0;    // counter on errors sending message via UDP

// timers
unsigned long hb_time = 0;            // heartbeat timer
unsigned long dhcp_timer = 0;         // dhcp refresh timer
unsigned long ntp_timer = 0;          // dhcp refresh timer
unsigned long chk_udp_conn_timer = 0; // we check periodically if we have received a HB from server
//unsigned long till_header_time = 0;    // stores till a header is detected after preamble detect

// Prototypes
bool is_new_packet(uint16_t size);                         // switch if we have a packet received we never saw before RcvBuffer[12] changes, rest is same
void addNodeData(uint16_t size, int16_t rssi, int8_t snr); // add additional data we need and send the udp packet
void blinkLED();
void sendHeartbeat();                                // heartbeat to server
void doTX();                                         // LoraTX function
void addUdpOutBuffer(uint8_t *buffer, uint16_t len); // function adds outgoing udp messages in the udp_out_ringbuffer
void printBuffer(uint8_t *buffer, int len);

// Client basic variables
    uint32_t _GW_ID = 0x99999999; // ID of our Node

    String _longname = "CL_NAME"; // our longanme, can be up to 20 chars ending with 0x00

    String shortname = "XXXXX";     //our shortname

void nrf52setup()
{

     // LEDs
    pinMode(PIN_LED1, OUTPUT);
    pinMode(PIN_LED2, OUTPUT);

    // clear the buffers
    for (int i = 0; i < uint8_t(sizeof(RcvBuffer)); i++)
    {
        RcvBuffer[i] = RcvBuffer_before[i] = 0x00;
    }

    //clear ringbuffer
    for(int i=0; i<MAX_RING_UDP_OUT; i++)
        memset(ringBufferUDPout[i],0,UDP_TX_BUF_SIZE);

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

    // Ethernet INIT
    //neth.initethDHCP();
    delay(100);
    //sendHeartbeat();
}

void nrf52loop()
{
    //check if we got from the serial input
    while(Serial.available() > 0)
    {
        String str = Serial.readString();
        str.trim();                 // removes CR, NL, Whitespace at end
        if (str.equals("reset"))
            NVIC_SystemReset();     // resets the device
    }
    
    //  We are on FreeRTOS, give other tasks a chance to run
    delay(100);
    yield();
}

/**@brief Function to be executed on Radio Rx Done event
 */


void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    //unsigned long diff_rx = millis() - till_header_time;
    //DEBUG_MSG_VAL("RADIO", diff_rx, "Time Preamble to RxDone");

    memcpy(RcvBuffer, payload, size);

    if (is_new_packet(size) || (SEE_ALL_PACKETS == 1))
    {
        // print which message type we got
        uint8_t msg_type_b_lora = payload[17];

        switch (msg_type_b_lora)
        {

            case 0x01: DEBUG_MSG("RADIO", "Received Textmessage"); break;
            case 0x03: DEBUG_MSG("RADIO", "Received PosInfo"); break;
            case 0x04: DEBUG_MSG("RADIO", "Received NodeInfo"); break;
            case 0x05: DEBUG_MSG("RADIO", "Routing APP"); break;
            case 0x06: DEBUG_MSG("RADIO", "Admin APP"); break;
            case 0x20: DEBUG_MSG("RADIO", "Reply APP"); break;
            case 0x42: DEBUG_MSG("RADIO", "Rangetest APP"); break;
            case 0x43: DEBUG_MSG("RADIO", "Environmental APP"); break;
            default: DEBUG_MSG("RADIO", "Received unknown"); break;
        
        }

        // print hex of message
        //neth.printBuffer(RcvBuffer, size);

        // we add now Longname (up to 20), ID - 4, RSSI - 2, SNR - 1 and MODE BYTE - 1
        // MODE BYTE: LongSlow = 1, MediumSlow = 3
        // and send the UDP packet (done in the method)

        // we only send the packet via UDP if we have no collision with UDP rx
        // und wenn MSG nicht von einem anderen Gateway empfangen wurde welches es bereits vopm Server bekommen hat
        int msg_id = (RcvBuffer[11]<<24) | (RcvBuffer[10]<<16) | (RcvBuffer[9]<<8) | RcvBuffer[8];
        int msg_hop = 0;
        int msg_hop_pos = 0;

        // gültige Meldung
        if(RcvBuffer[16] == 0x08)
        {
            for(int ihop=18;ihop<size;ihop++)
            {
                if(RcvBuffer[ihop] == 0x1A && RcvBuffer[ihop+1] > 0 && RcvBuffer[ihop+1] <= 7)
                {
                    msg_hop_pos=ihop;
                    msg_hop=RcvBuffer[ihop+1];
                    break;
                }
            }
        }

        int lora_msg_len = size;
        if (lora_msg_len > UDP_TX_BUF_SIZE)
        lora_msg_len = UDP_TX_BUF_SIZE; // zur Sicherheit

        printf("msg_id: %04X msg_len: %i payload[%i]=%i\n", msg_id, lora_msg_len, msg_hop_pos, msg_hop);

        if(RcvBuffer[11] != 0x77)
        {
            // Wiederaussendung via LORA
            // Ringbuffer filling

            bool bMsg=false;

            for(int iop=0;iop<MAX_RING_UDP_OUT;iop++)
            {
                for(int irp=0;irp<UDP_TX_BUF_SIZE;irp++)
                {
                    if(ringBufferUDPout[iop][irp] == 0x00)
                    {
                        int ring_msg_id = (ringBufferUDPout[iop][irp+20]<<24) | (ringBufferUDPout[iop][irp+19]<<16) | (ringBufferUDPout[iop][irp+18]<<8) | ringBufferUDPout[iop][irp+17];

                        if(ring_msg_id != 0)
                            printf("ring_msg_id:%08X msg_id:%08X\n", ring_msg_id, msg_id);

                        if(ring_msg_id == msg_id)
                            bMsg=true;

                        break;
                    }
                }
            }

            if(!bMsg)
            {
                if(msg_hop > 0 && msg_hop_pos > 0)
                    payload[msg_hop_pos]=msg_hop-1;
            


                if ((msg_type_b_lora == 0x01 || msg_type_b_lora == 0x03 || msg_type_b_lora == 0x04))
                {
                    addNodeData(size, rssi, snr);
                    DEBUG_MSG("RADIO", "Packet sent to -->");
                }
                else
                    DEBUG_MSG("RADIO", "Packet not sent to -->");
            }    
        }

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
    digitalWrite(PIN_LED2, LOW);
}

/**@brief Function to be executed on Radio Tx Timeout event
 */
void OnTxTimeout(void)
{
    // DEBUG_MSG("RADIO", "OnTxTimeout");
    tx_is_active = false;
    digitalWrite(PIN_LED2, LOW);
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

/**@brief fires when a correct syncword is detected
 */
/* NICHT BENÖTIGT
void OnHeaderDetect(void)
{
    is_receiving = true;
    DEBUG_MSG("RADIO", "Header detected");
    //unsigned long diff = millis() - till_header_time;
    //DEBUG_MSG_VAL("RADIO", diff, "Time Preamble to Header");
}
*/


/**@brief our Lora TX sequence
 */
/*
void doTX()
{
    tx_is_active = true;

    if (neth.iWrite != neth.iRead && neth.iWrite < MAX_RING)
    {

        sendlng = neth.ringBuffer[neth.iRead][0];
        memcpy(lora_tx_buffer, neth.ringBuffer[neth.iRead] + 1, sendlng);

        // we can now tx the message
        if (TX_ENABLE == 1)
        {
            int irs = neth.iRead;

            digitalWrite(PIN_LED2, HIGH);

            // print tx buffer
            // neth.printBuffer(neth.lora_tx_buffer_eth, neth.lora_tx_msg_len);

            neth.iRead++;
            if (neth.iRead >= MAX_RING)
                neth.iRead = 0;

            Radio.Send(lora_tx_buffer, sendlng);

            cmd_counter = WAIT_TX;

            if (neth.iWrite == neth.iRead)
                DEBUG_MSG_VAL("RADIO", irs, "TX (LAST) :");
            else
                DEBUG_MSG_VAL("RADIO", irs, "TX :");

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

    uint8_t longname_len = _longname.length();
    char longname_c[longname_len + 1];

    // copying the contents of the
    // string to char array
    strcpy(longname_c, _longname.c_str());

    uint8_t offset = 8 + longname_len + 1; // offset for the payload written into tx udp buffer. We add 0x00 after Longanme

    if (longname_len <= LONGNAME_MAXLEN)
    {
        memcpy(&txBuffer, &longname_c, longname_len);
        txBuffer[longname_len] = 0x00; // we add a trailing 0x00 to mark the end of longname
    }
    else
    {
        DEBUG_MSG("ERROR", "LongName is too long!");
        longname_len = LONGNAME_MAXLEN;
    }

    uint8_t offset_params = longname_len + 1;
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
        addUdpOutBuffer(txBuffer, (size + offset));
    }
    else
    {
        DEBUG_MSG("ERROR", "Exceeding Buffer length!");
    }

}

/**@brief Function adding messages into outgoing UDP ringbuffer
 * 
 */
void addUdpOutBuffer(uint8_t *buffer, uint16_t len)
{
    if (len > UDP_TX_BUF_SIZE)
        len = UDP_TX_BUF_SIZE; // just for safety

    //first byte is always the message length
    ringBufferUDPout[udpWrite][0] = len;
    memcpy(ringBufferUDPout[udpWrite] + 1, buffer, len + 1);

    Serial.printf("Out Ringbuffer added element: %u\n", udpWrite);
    //DEBUG_MSG_VAL("UDP", udpWrite, "UDP Ringbuf added El.:");
    printBuffer(ringBufferUDPout[udpWrite], len + 1);

    udpWrite++;
    if (udpWrite >= MAX_RING_UDP_OUT) // if the buffer is full we start at index 0 -> take care of overwriting!
        udpWrite = 0;
}

/**@brief Method to print our buffers
 */
void printBuffer(uint8_t *buffer, int len)
{
  for (int i = 0; i < len; i++)
  {
    Serial.printf("%02X ", buffer[i]);
  }
  Serial.println("");
}

/**@brief Function to send our heartbeat
 * longanme0x000xAABBCCDDKEEPGW0100x00
 *               GW_ID
 */
/*
void sendHeartbeat()
{
    String keep = "KEEP";
    String firmware ="GW010";
    uint8_t longname_len = neth._longname.length();
    uint16_t hb_buffer_size = longname_len + 1 + sizeof(neth._GW_ID) + keep.length() + firmware.length();;
    uint8_t hb_buffer[hb_buffer_size];

    // Serial.print("\nHB buffer size: ");
    // Serial.println(hb_buffer_size);

    char longname_c[longname_len + 1];
    strcpy(longname_c, neth._longname.c_str());
    longname_c[longname_len] = 0x00;

    char keep_c[keep.length()];
    strcpy(keep_c, keep.c_str());

    char firmware_c[firmware.length()];
    strcpy(firmware_c, firmware.c_str());

    // copying all together
    memcpy(&hb_buffer, &longname_c, longname_len + 1);
    memcpy(&hb_buffer[longname_len + 1], &neth._GW_ID, sizeof(neth._GW_ID));
    memcpy(&hb_buffer[longname_len + 1 + sizeof(neth._GW_ID)], &keep_c, sizeof(keep_c));
    memcpy(&hb_buffer[longname_len + 1 + sizeof(neth._GW_ID) + sizeof(keep_c)], &firmware_c, sizeof(firmware_c));

    // if sending fails via UDP.endpacket() for a maximum of counts reset UDP stack
    //also avoid UDP tx when UDP is getting a packet
    // add HB message to the ringbuffer
    //DEBUG_MSG("UDP", "HB Buffer");
    //neth.printBuffer(hb_buffer, hb_buffer_size);
    addUdpOutBuffer(hb_buffer, hb_buffer_size);

}

/**@brief Function to check if the modem detected a preamble
 */
void blinkLED()
{
    digitalWrite(PIN_LED1, HIGH);
    delay(2);
    digitalWrite(PIN_LED1, LOW);
}

void blinkLED2()
{
    digitalWrite(PIN_LED2, HIGH);
    delay(2);
    digitalWrite(PIN_LED2, LOW);
}


/**@brief Function to get the current Radio Status
 * Radio status.[RF_IDLE, RF_RX_RUNNING, RF_TX_RUNNING]
 */

void print_radioStatus()
{
    if(Radio.GetStatus() == RF_IDLE) Serial.println("RF_IDLE");
    if(Radio.GetStatus() == RF_RX_RUNNING) Serial.println("RF_RX_RUNNING");
    if(Radio.GetStatus() == RF_TX_RUNNING) Serial.println("RF_TX_RUNNING");
    if(Radio.GetStatus() == RF_CAD) Serial.println("RF_CAD");

} 
