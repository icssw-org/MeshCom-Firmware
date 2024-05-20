//#ifndef BOARD_TLORA_OLV216

#include "loop_functions.h"
#include "loop_functions_extern.h"

#include "lora_setchip.h"

#ifdef SX127X
    #include <RadioLib.h>
    extern SX1278 radio;
    extern int transmissionState;
#endif

#ifdef SX126X
    #include <RadioLib.h>
    extern SX1268 radio;
    extern int transmissionState;
#endif

#ifdef SX126X_V3
    #include <RadioLib.h>
    extern SX1262 radio;
    extern int transmissionState;
#endif

#ifdef BOARD_T_ECHO
    #include <RadioLib.h>
    extern SX1262 radio;
    extern int transmissionState;
#endif

bool rf_crc = true;
uint16_t rf_preamble_length = LORA_PREAMBLE_LENGTH;

//0...EU  1...UK, 3...EA, 10...US, ..... 18...868, 19...915, 20...MAN
String strCountry[21] = {"EU", "UK", "none", "EA", "none", "none", "none", "none", "none", "none", "US", "VR2", "none", "none", "none", "none", "none", "none", "868", "915", "MAN"};

String getCountry(int iCtry)
{
    if(iCtry < 0 || iCtry > 19)
    {
        return "none";
    }

    return strCountry[iCtry];
}

int getCountryID(String strCtry)
{
    for(int ic=0;ic<22;ic++)
    {
        if(strCountry[ic] == strCtry)
            return ic;
    }

    return -1;
}

float getFreq()
{
    float freq = meshcom_settings.node_freq;
    if(freq <= 0)
        freq = RF_FREQUENCY;

    #ifdef BOARD_RAK4630
        freq=freq/1000000;
    #endif

    return freq;
}

float getBW()
{
    float bw = meshcom_settings.node_bw;

    #if defined BOARD_RAK4630
        if(bw == 0)
            bw = 125.0;
        else
        if(bw == 1)
            bw = 250.0;
        else
        if(bw == 2)
            bw = 500.0;
    #else
        if(bw == 0)
            bw = LORA_BANDWIDTH;
    #endif

    return bw;
}

int getSF()
{
    int sf = meshcom_settings.node_sf;
    if(sf == 0)
        sf = LORA_SF;

    return sf;
}

int getCR()
{
    int rf_cr_info = meshcom_settings.node_cr;
    if(rf_cr_info == 0)
        rf_cr_info = LORA_CR;

    #ifdef BOARD_RAK4630
        if(meshcom_settings.node_cr == 1)
            rf_cr_info=5;
        else
        if(meshcom_settings.node_cr == 2)
            rf_cr_info=6;
        else
        if(meshcom_settings.node_cr == 3)
            rf_cr_info=7;
        else
        if(meshcom_settings.node_cr == 4)
            rf_cr_info=8;
    #endif
    
    return rf_cr_info;
}

int8_t getPower()
{
    int power = meshcom_settings.node_power;
    if(power <= 0)
        power = TX_OUTPUT_POWER;

    return power;
}

void lora_setcountry(int iCtry)
{
    float dec_bandwith = 0;

    switch (iCtry)
    {
        case 1:  // UK ... 
            
            #if defined BOARD_RAK4630
                meshcom_settings.node_freq = 439912500;
                meshcom_settings.node_bw = 0;
                meshcom_settings.node_cr = 1;
            #else
                meshcom_settings.node_freq = 439.9125;
                meshcom_settings.node_bw = 125.0;
                meshcom_settings.node_cr = 5;
            #endif

            meshcom_settings.node_sf = 10;

            meshcom_settings.node_track_freq = LORA_APRS_FREQUENCY;
            
            break;

        case 10:  // US ... 
            #if defined BOARD_RAK4630
                meshcom_settings.node_freq = 433175000;
                meshcom_settings.node_bw = 1;
                meshcom_settings.node_cr = 2;
            #else
                meshcom_settings.node_freq = 433.175;
                meshcom_settings.node_bw = 250.0;
                meshcom_settings.node_cr = 6;
            #endif

            meshcom_settings.node_sf = 11;

            meshcom_settings.node_track_freq = LORA_APRS_FREQUENCY;
            
            break;

        case 11:  // VR2 ... 
            #if defined BOARD_RAK4630
                meshcom_settings.node_freq = 435775000;
                meshcom_settings.node_bw = 1;
                meshcom_settings.node_cr = 2;
            #else
                meshcom_settings.node_freq = 435.775;
                meshcom_settings.node_bw = 250.0;
                meshcom_settings.node_cr = 6;
            #endif

            meshcom_settings.node_sf = 11;

            meshcom_settings.node_track_freq = LORA_APRS_FREQUENCY;
            
            break;

        case 18:  // 868 ... 

            #if defined BOARD_RAK4630
                meshcom_settings.node_freq = 869525000;
                meshcom_settings.node_bw = 1;
                meshcom_settings.node_cr = 2;
            #else
                meshcom_settings.node_freq = 869.525;
                meshcom_settings.node_bw = 250.0;
                meshcom_settings.node_cr = 6;
            #endif

            meshcom_settings.node_sf = LORA_SF;

            meshcom_settings.node_track_freq = 999;

            break;

        case 19:  // 906 ... 

            #if defined BOARD_RAK4630
                meshcom_settings.node_freq = 906875000;
                meshcom_settings.node_bw = 1;
                meshcom_settings.node_cr = 2;
            #else
                meshcom_settings.node_freq = 906.875;
                meshcom_settings.node_bw = 250.0;
                meshcom_settings.node_cr = 6;
            #endif

            meshcom_settings.node_sf = LORA_SF;

            meshcom_settings.node_track_freq = LORA_APRS_FREQUENCY;
            
            break;

        case 20:  // MAN ... manual

            // bandwith        
            if(meshcom_settings.node_bw <= 0)
                meshcom_settings.node_bw = LORA_BANDWIDTH;

            if(meshcom_settings.node_bw != 125 && meshcom_settings.node_bw != 250)
                meshcom_settings.node_bw = LORA_BANDWIDTH;

            // frequency
            if(meshcom_settings.node_freq <= 0)
                meshcom_settings.node_freq = RF_FREQUENCY;

            dec_bandwith = (meshcom_settings.node_bw/2.0)/100.0;

            if(!((meshcom_settings.node_freq >= (430.0 + dec_bandwith) && meshcom_settings.node_freq <= (439.000 - dec_bandwith)) || (meshcom_settings.node_freq >= (869.4 + dec_bandwith) && meshcom_settings.node_freq <= (869.65 - dec_bandwith))))
                meshcom_settings.node_freq = RF_FREQUENCY;

            // set spreading factor 
            if(meshcom_settings.node_sf <= 0)
                meshcom_settings.node_sf = LORA_SF;

            if(meshcom_settings.node_sf < 6 ||  meshcom_settings.node_sf > 12)
                meshcom_settings.node_sf = LORA_SF;

            // set coding rate 
            if(meshcom_settings.node_cr <= 0)
                meshcom_settings.node_cr = LORA_CR;

            if(meshcom_settings.node_cr < 5 ||  meshcom_settings.node_cr > 8)
                meshcom_settings.node_cr = LORA_CR;

            meshcom_settings.node_track_freq = LORA_APRS_FREQUENCY;

            break;

        default:    // EU
            meshcom_settings.node_freq = RF_FREQUENCY;

            #if defined BOARD_RAK4630
                meshcom_settings.node_bw = 1;
                meshcom_settings.node_cr = 2;
            #else
                meshcom_settings.node_bw = 250.0;
                meshcom_settings.node_cr = 6;
            #endif

            meshcom_settings.node_sf = LORA_SF;

            meshcom_settings.node_track_freq = LORA_APRS_FREQUENCY;
            
            break;
    }

    save_settings();
}

bool lora_setchip_meshcom()
{
#if defined BOARD_RAK4630
void RadioInit();

    //  Initialize the LoRa Transceiver
    RadioInit();

    // Sets the Syncword new that we can set the MESHCOM SWORD
    Radio.SetPublicNetwork(true);

    if(bLORADEBUG)
        Serial.printf("[LoRa]...RF_FREQUENCY: %.4f kHz\n", getFreq());

    uint32_t ifreq=meshcom_settings.node_freq/10;
    ifreq=ifreq*10;

    //  Set the LoRa Frequency
    Radio.SetChannel(ifreq);

    //  Configure the LoRa Transceiver for receiving messages
    Radio.SetRxConfig(
        MODEM_LORA,
        (uint32_t)meshcom_settings.node_bw,
        (uint32_t)meshcom_settings.node_sf,
        (uint8_t)meshcom_settings.node_cr,
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
    if(bLORADEBUG)
        Serial.printf("[LoRa]...RF_POWER: %d dBm\n", getPower());

    Radio.SetTxConfig(
        MODEM_LORA,
        getPower(),
        0, // fsk only
        (uint32_t)meshcom_settings.node_bw,
        (uint32_t)meshcom_settings.node_sf,
        (uint8_t)meshcom_settings.node_cr,
        LORA_PREAMBLE_LENGTH,
        LORA_FIX_LENGTH_PAYLOAD_ON,
        true, // CRC ON
        0,    // fsk only frequ hop
        0,    // fsk only frequ hop period
        LORA_IQ_INVERSION_ON,
        TX_TIMEOUT_VALUE);

    //  Start receiving LoRa packets
    Radio.Rx(RX_TIMEOUT_VALUE);
    
    delay(500);

#else
    // Set MeshCom parameter
    float rf_freq = meshcom_settings.node_freq;
    float rf_bw = meshcom_settings.node_bw;
    int rf_sf = meshcom_settings.node_sf;
    int rf_cr = meshcom_settings.node_cr;
    uint16_t rf_preamble_length = LORA_PREAMBLE_LENGTH;
    bool rf_crc = true;
    return lora_setchip_new(rf_freq, rf_bw, rf_sf, rf_cr, SYNC_WORD_SX127x, rf_preamble_length, rf_crc);
#endif

    return true;

}

bool lora_setchip_aprs()
{
#if defined BOARD_RAK4630
void RadioInit();

    //  Initialize the LoRa Transceiver
    RadioInit();

    // Sets the Syncword new that we can set the MESHCOM SWORD
    Radio.SetPublicNetwork(false);

    if(bLORADEBUG)
        Serial.printf("[LoRa]...RF_FREQUENCY: %.4f kHz\n", LORA_APRS_FREQUENCY/1000000.);

    //  Set the LoRa Frequency
    Radio.SetChannel((uint32_t)LORA_APRS_FREQUENCY);

    //  Configure the LoRa Transceiver for receiving messages
    Radio.SetRxConfig(
        MODEM_LORA,
        LORA_APRS_BANDWIDTH,
        LORA_APRS_SPREADING_FACTOR,
        LORA_APRS_CODINGRATE,
        0, //  AFC bandwidth: Unused with LoRa
        LORA_APRS_PREAMBLE_LENGTH,
        LORA_SYMBOL_TIMEOUT,
        LORA_FIX_LENGTH_PAYLOAD_ON,
        0,    //  Fixed payload length: N/A
        false, //  CRC enabled
        0,    //  Frequency hopping disabled
        0,    //  Hop period: N/A
        LORA_IQ_INVERSION_ON,
        true //  Continuous receive mode
    );

    // Set Radio TX configuration
    if(bLORADEBUG)
        Serial.printf("[LoRa]...RF_POWER: %d dBm\n", getPower());

    Radio.SetTxConfig(
        MODEM_LORA,
        getPower(),
        0, // fsk only
        LORA_APRS_BANDWIDTH,
        LORA_APRS_SPREADING_FACTOR,
        LORA_APRS_CODINGRATE,
        LORA_APRS_PREAMBLE_LENGTH,
        LORA_FIX_LENGTH_PAYLOAD_ON,
        true, // CRC ON
        0,    // fsk only frequ hop
        0,    // fsk only frequ hop period
        LORA_IQ_INVERSION_ON,
        TX_TIMEOUT_VALUE);

    delay(500);

#else
    // Set LoRaAPRS parameter
    float rf_freq = LORA_APRS_FREQUENCY;
    float rf_bw = 125.0;
    int rf_sf = 12;
    int rf_cr = 5;
    uint16_t rf_preamble_length = 8;
    bool rf_crc = true;

    return lora_setchip_new(rf_freq, rf_bw, rf_sf, rf_cr, 0x12, rf_preamble_length, rf_crc);
#endif

    return true;
}

bool lora_setchip_new(float rf_freq, float rf_bw, int rf_sf, int rf_cr, int rf_syncword, uint16_t rf_preamble_length, bool rf_crc)
{

#ifdef ESP32

    if(bLORADEBUG)
        Serial.printf("LoRa RF_FREQUENCY: %.3f MHz\n", rf_freq);

    if (radio.setFrequency(rf_freq) == RADIOLIB_ERR_INVALID_FREQUENCY)
    {
        Serial.println(F("Selected frequency is invalid for this module!"));
        lora_setchip_meshcom();
        return false;
    }

    if(bLORADEBUG)
        Serial.printf("LoRa RF_BANDWIDTH: %.0f kHz\n", rf_bw);

    if (radio.setBandwidth(rf_bw) == RADIOLIB_ERR_INVALID_BANDWIDTH)
    {
        Serial.println(F("Selected bandwidth is invalid for this module!"));
        lora_setchip_meshcom();
        return false;
    }

    if(bLORADEBUG)
        Serial.printf("LoRa RF_SF: %i\n", rf_sf);

    if (radio.setSpreadingFactor(rf_sf) == RADIOLIB_ERR_INVALID_SPREADING_FACTOR)
    {
        Serial.println(F("Selected spreading factor is invalid for this module!"));
        lora_setchip_meshcom();
        return false;
    }

    if(bLORADEBUG)
        Serial.printf("LoRa RF_CR: 4/%i\n", rf_cr);

    if (radio.setCodingRate(rf_cr) == RADIOLIB_ERR_INVALID_CODING_RATE)
    {
        Serial.println(F("Selected coding rate is invalid for this module!"));
        lora_setchip_meshcom();
        return false;
    }

    if (radio.setSyncWord(rf_syncword) != RADIOLIB_ERR_NONE)
    {
        Serial.println(F("Unable to set sync word!"));
        lora_setchip_meshcom();
        return false;
    }

    // set LoRa preamble length to 15 symbols (accepted range is 6 - 65535)
    if (radio.setPreambleLength(rf_preamble_length) == RADIOLIB_ERR_INVALID_PREAMBLE_LENGTH)
    {
        Serial.println(F("Selected preamble length is invalid for this module!"));
        return false;
    }

    // CRC
    if (radio.setCRC(rf_crc) == RADIOLIB_ERR_INVALID_CRC_CONFIGURATION)
    {
        Serial.println(F("Selected CRC is invalid for this module!"));
        return false;
    }

    delay(500);

#endif

    return true;
}

//#endif