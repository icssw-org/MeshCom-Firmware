#ifndef BOARD_TLORA_OLV216

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

bool lora_setchip_meshcom()
{
#if defined BOARD_RAK4630
void RadioInit();

    //  Initialize the LoRa Transceiver
    RadioInit();

    // Sets the Syncword new that we can set the MESHCOM SWORD
    Radio.SetPublicNetwork(true);

    if(bLORADEBUG)
        Serial.printf("[LoRa]...RF_FREQUENCY: %i kHz\n", RF_FREQUENCY);

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
    int8_t tx_power = TX_OUTPUT_POWER;
    
    if(meshcom_settings.node_power <= 0)
        meshcom_settings.node_power = TX_OUTPUT_POWER;
    else
        tx_power=meshcom_settings.node_power;   //set by command

    if(tx_power > TX_POWER_MAX)
        tx_power= TX_POWER_MAX;

    if(tx_power < TX_POWER_MIN)
        tx_power= TX_POWER_MIN;

    if(bLORADEBUG)
        Serial.printf("[LoRa]...RF_POWER: %d dBm\n", tx_power);

    Radio.SetTxConfig(
        MODEM_LORA,
        tx_power,
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
        Serial.printf("[LoRa]...RF_FREQUENCY: %i kHz\n", LORA_APRS_FREQUENCY);

    //  Set the LoRa Frequency
    Radio.SetChannel(LORA_APRS_FREQUENCY);

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
        true, //  CRC enabled
        0,    //  Frequency hopping disabled
        0,    //  Hop period: N/A
        LORA_IQ_INVERSION_ON,
        true //  Continuous receive mode
    );

    // Set Radio TX configuration
    int8_t tx_power = TX_OUTPUT_POWER;
    
    if(meshcom_settings.node_power <= 0)
        meshcom_settings.node_power = TX_OUTPUT_POWER;
    else
        tx_power=meshcom_settings.node_power;   //set by command

    if(tx_power > TX_POWER_MAX)
        tx_power= TX_POWER_MAX;

    if(tx_power < TX_POWER_MIN)
        tx_power= TX_POWER_MIN;

    if(bLORADEBUG)
        Serial.printf("[LoRa]...RF_POWER: %d dBm\n", tx_power);

    Radio.SetTxConfig(
        MODEM_LORA,
        tx_power,
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

#endif