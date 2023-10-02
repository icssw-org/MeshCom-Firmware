#ifndef BOARD_TLORA_OLV216

#include "loop_functions.h"
#include "loop_functions_extern.h"

#include "lora_setchip.h"

#ifdef SX127X
    #include <RadioLib.h>
    extern SX1278 radio;
    extern int transmissionState;
#endif

#ifdef BOARD_E220
    #include <RadioLib.h>
    // RadioModule derived from SX1262 
    extern LLCC68 radio;
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

bool lora_setchip_meshcom()
{
#if defined BOARD_RAK4630
    // Set MeshCom parameter
    float rf_freq = RF_FREQUENCY;
    rf_freq = rf_freq / 1000000.0;
    float rf_bw = 250;
    int rf_sf = LORA_SPREADING_FACTOR;
    int rf_cr = 6;
    uint16_t rf_preamble_length = LORA_PREAMBLE_LENGTH;
    bool rf_crc = true;
#else
    // Set MeshCom parameter
    float rf_freq = meshcom_settings.node_freq;
    float rf_bw = meshcom_settings.node_bw;
    int rf_sf = meshcom_settings.node_sf;
    int rf_cr = meshcom_settings.node_cr;
    uint16_t rf_preamble_length = LORA_PREAMBLE_LENGTH;
    bool rf_crc = true;
#endif

    return lora_setchip_new(rf_freq, rf_bw, rf_sf, rf_cr, SYNC_WORD_SX127x, rf_preamble_length, rf_crc);
}

bool lora_setchip_aprs()
{
    // Set LoRaAPRS parameter
    float rf_freq = 433.775;
    float rf_bw = 125.0;
    int rf_sf = 12;
    int rf_cr = 5;
    uint16_t rf_preamble_length = 8;
    bool rf_crc = true;

    return lora_setchip_new(rf_freq, rf_bw, rf_sf, rf_cr, 0x12, rf_preamble_length, rf_crc);
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

#if defined BOARD_RAK4630

    /*
    bool lora_setchip_new(float rf_freq, float rf_bw, int rf_sf, int rf_cr, int rf_syncword, uint16_t rf_preamble_length, bool rf_crc)
    // Set LoRaAPRS parameter
    float rf_freq = 433.775;
    float rf_bw = 125.0;
    int rf_sf = 12;
    int rf_cr = 5;
    uint16_t rf_preamble_length = 8;
    bool rf_crc = true;
    */

    uint32_t rak_freq = rf_freq * 1000000.0;

    if(rf_syncword == 0x12)
        rak_freq = 433775000;
    else
        rak_freq = 433175000;

    uint32_t rak_bw = 0;    // 125
    if(rf_bw == 250.0)
        rak_bw = 1;
    if(rf_bw == 500.0)
        rak_bw = 2;

    uint32_t rak_sf = rf_sf;

    uint8_t rak_cr = 1;     // 4/5
    if(rf_cr == 6)
        rak_cr = 2;
    if(rf_cr == 7)
        rak_cr = 3;
    if(rf_cr == 8)
        rak_cr = 4;

    // Set Radio TX configuration
    int8_t tx_power = TX_OUTPUT_POWER;
    
    if(meshcom_settings.node_power > 0)
        tx_power=meshcom_settings.node_power;   //set by command

    if(tx_power > TX_POWER_MAX)
        tx_power= TX_POWER_MAX;

    if(tx_power < TX_POWER_MIN)
        tx_power= TX_POWER_MIN;

    Serial.printf("LoRa FREQ:%i SYNC:%02X RF_POWER: %d dBm BW:%i SF:%i CR:%i PRE:%i\n", rak_freq, rf_syncword, tx_power, rak_bw, rak_sf, rak_cr, rf_preamble_length);

    //  Initialize the LoRa Transceiver
    //Radio.Init(&RadioEvents);

    // Sets the Syncword new that we can set the MESHCOM SWORD
    if(rf_syncword == 0x12)
        Radio.SetPublicNetwork(false);   //LoRaAPRS
    else
        Radio.SetPublicNetwork(true);   // MeshCom

    //  Set the LoRa Frequency
    Radio.SetChannel(rak_freq);

    //  Configure the LoRa Transceiver for receiving messages
    Radio.SetRxConfig(
        MODEM_LORA,
        rak_bw,
        rak_sf,
        rak_cr,
        0, //  AFC bandwidth: Unused with LoRa
        rf_preamble_length,
        LORA_SYMBOL_TIMEOUT,
        LORA_FIX_LENGTH_PAYLOAD_ON,
        0,    //  Fixed payload length: N/A
        rf_crc, //  CRC enabled
        0,    //  Frequency hopping disabled
        0,    //  Hop period: N/A
        LORA_IQ_INVERSION_ON,
        true //  Continuous receive mode
    );

    Radio.SetTxConfig(
        MODEM_LORA,
        tx_power,
        0, // fsk only
        rak_bw,
        rak_sf,
        rak_cr,
        rf_preamble_length,
        LORA_FIX_LENGTH_PAYLOAD_ON,
        rf_crc, // CRC ON
        0,    // fsk only frequ hop
        0,    // fsk only frequ hop period
        LORA_IQ_INVERSION_ON,
        TX_TIMEOUT_VALUE
    );

    delay(1500);

#endif

    return true;
}

#endif