/**
 * @file ble.cpp
 * @author Bernd Giesecke (bernd.giesecke@rakwireless.com)
 * @brief BLE initialization & device configuration
 * @version 0.1
 * @date 2021-01-10
 *
 * @copyright Copyright (c) 2021
 *
 */
#ifdef NRF52_SERIES

#include "WisBlock-API.h"
#include <loop_functions.h>
#include <debugconf.h>
#include <configuration.h>

extern uint8_t isPhoneReady;
extern bool ble_busy_flag;
extern uint16_t swap2bytes(uint16_t value);
extern void commandAction(char *msg_text, int len, bool ble);
extern void sendMessage(char *buffer, int len);
extern bool hasMsgFromPhone;
extern char textbuff_phone [MAX_MSG_LEN_PHONE];
extern uint8_t txt_msg_len_phone;
extern bool bInitDisplay;
extern uint8_t dmac[6];


void sendConfigToPhone ();

// Create device name
char helper_string[256] = {0};

/** OTA DFU service */
BLEDfu ble_dfu;
/** BLE UART service */
BLEUart g_ble_uart;
/** Device information service */
BLEDis ble_dis;

/** LoRa service 0xF0A0 */
BLEService lora_service = BLEService(0xF0A0);
/** LoRa settings  characteristic 0xF0A1 */
BLECharacteristic g_lora_data = BLECharacteristic(0xF0A1);

// Settings callback
void settings_rx_callback(uint16_t conn_hdl, BLECharacteristic *chr, uint8_t *data, uint16_t len);

// Connect callback
void connect_callback(uint16_t conn_handle);
// Disconnect callback
void disconnect_callback(uint16_t conn_handle, uint8_t reason);
// Uart RX callback
void bleuart_rx_callback(uint16_t conn_handle);

/** Flag if BLE UART is connected */
bool g_ble_uart_is_connected = false;

/**
 * @brief Initialize BLE and start advertising
 *
 */
void init_ble(void)
{
	// Config the peripheral connection with maximum bandwidth
	// more SRAM required by SoftDevice
	// Note: All config***() function must be called before begin()
	Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
#ifdef ISP4520
	Bluefruit.configPrphConn(250, BLE_GAP_EVENT_LENGTH_MIN, 16, 16);
#else
	Bluefruit.configPrphConn(250, BLE_GAP_EVENT_LENGTH_MIN, 16, 16);
#endif
	// Start BLE
	Bluefruit.begin(1, 0);
	// BLE Pin 
	Bluefruit.Security.setPIN(PAIRING_PIN);

	// Set max power. Accepted values are: (min) -40, -20, -16, -12, -8, -4, 0, 2, 3, 4, 5, 6, 7, 8 (max)
	Bluefruit.setTxPower(0);

#if NO_BLE_LED > 0
	Bluefruit.autoConnLed(false);
	digitalWrite(LED_BLUE, LOW);
#endif

#ifdef _VARIANT_ISP4520_
	/** Device name for ISP4520 */
	sprintf(helper_string, "%s-%02X%02X%02X%02X%02X%02X", g_ble_dev_name,
			(uint8_t)(meshcom_settings.node_device_eui[2]), (uint8_t)(meshcom_settings.node_device_eui[3]),
			(uint8_t)(meshcom_settings.node_device_eui[4]), (uint8_t)(meshcom_settings.node_device_eui[5]), (uint8_t)(meshcom_settings.node_device_eui[6]), (uint8_t)(meshcom_settings.node_device_eui[7]));
#else
	/** Device name for RAK4631 */
	sprintf(helper_string, "%s-%02x%02x-%s", g_ble_dev_name, dmac[4], dmac[5], meshcom_settings.node_call);	// Anzeige mit callsign
#endif

	Bluefruit.setName(helper_string);

	// Set connection/disconnect callbacks
	Bluefruit.Periph.setConnectCallback(connect_callback);
	Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

	// Configure and Start Device Information Service
#ifdef _VARIANT_ISP4520_
	ble_dis.setManufacturer("Insight_SIP");

	ble_dis.setModel("ISP4520");
#else
	ble_dis.setManufacturer("RAKwireless");

	ble_dis.setModel("RAK4631");
#endif

	sprintf(helper_string, "%d.%d.%d", g_sw_ver_1, g_sw_ver_2, g_sw_ver_3);
	ble_dis.setSoftwareRev(helper_string);

	ble_dis.setHardwareRev("52840");

	ble_dis.begin();

	// Start the DFU service
	ble_dfu.begin();

	// Permission / Pairing
	g_ble_uart.setPermission(SECMODE_ENC_WITH_MITM, SECMODE_ENC_WITH_MITM);
	// Start the UART service
	g_ble_uart.begin();
	g_ble_uart.setRxCallback(bleuart_rx_callback);

	// Initialize the LoRa setting service
	BLEService sett_service = init_settings_characteristic();

	// Advertising packet
	Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE); //
	Bluefruit.Advertising.addService(sett_service);
	Bluefruit.Advertising.addName();
	Bluefruit.Advertising.addTxPower();

	/* Start Advertising
	 * - Enable auto advertising if disconnected
	 * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
	 * - Timeout for fast mode is 30 seconds
	 * - Start(timeout) with timeout = 0 will advertise forever (until connected)
	 *
	 * For recommended advertising interval
	 * https://developer.apple.com/library/content/qa/qa1931/_index.html
	 */
	Bluefruit.Advertising.restartOnDisconnect(true);
	Bluefruit.Advertising.setInterval(32, 244); // in unit of 0.625 ms
	Bluefruit.Advertising.setFastTimeout(15);	// number of seconds in fast mode
	// Bluefruit.Advertising.start(60);			// 0 = Don't stop advertising
	if (meshcom_settings.auto_join)
	{
		restart_advertising(60);
	}
	else
	{
		restart_advertising(0);
	}
}

/**
 * @brief Restart advertising for a certain time
 *
 * @param timeout timeout in seconds
 */
void restart_advertising(uint16_t timeout)
{
	Bluefruit.Advertising.start(timeout);
}

/**
 * @brief  Callback when client connects
 * @param  conn_handle: Connection handle id
 */
void connect_callback(uint16_t conn_handle)
{
	(void)conn_handle;
	Bluefruit.setTxPower(8);
	DEBUG_MSG("BLE", "Connected");
	g_ble_uart_is_connected = true;

}

/**
 * @brief  Callback invoked when a connection is dropped
 * @param  conn_handle: connection handle id
 * @param  reason: disconnect reason
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
	(void)conn_handle;
	(void)reason;
	g_ble_uart_is_connected = false;
	isPhoneReady = 0;
	Bluefruit.setTxPower(0);
	DEBUG_MSG("BLE", "Disconnected");
}

/**
 * Callback if data has been sent from the connected client
 * @param conn_handle
 * 		The connection handle
 */
void bleuart_rx_callback(uint16_t conn_handle)
{
	(void)conn_handle;

	g_task_event_type |= BLE_DATA;
	xSemaphoreGiveFromISR(g_task_sem, pdFALSE);

	// Forward data from Mobile to our peripheral
	uint8_t conf_data[MAX_MSG_LEN_PHONE] = {0};
	g_ble_uart.read(conf_data, MAX_MSG_LEN_PHONE);

	
	/**
	 * Config Messages
     * length 1B - Msg ID 1B - Data
	 * Msg ID:
	 * 0x10 - Hello Message (followed by 0x20, 0x30)
	 * 0x50 - Callsign
	 * 0x70 - Latitude
	 * 0x80 - Longitude
	 * 0x90 - Altitude
	 * 0xA0 - Textmessage
     * Data:
     * Callsign: length Callsign 1B - Callsign
     * Latitude: 8B Double
     * Longitude: 8B Double
     * Altitude: 4B Integer
	 * 
     * 
     *  */ 

	uint8_t msg_len = conf_data[0];
	uint8_t msg_type = conf_data[1];
	bool restart_ADV = false;

	DEBUG_MSG_VAL("BLE", msg_len, "Msg from Device Length");

	Serial.printBuffer(conf_data, msg_len);
	Serial.println();

	switch (msg_type){

		case 0x10: {

			if(conf_data[2] == 0x20 && conf_data[3] == 0x30){
				DEBUG_MSG("BLE", "Hello Msg from phone");
				sendConfigToPhone();
				isPhoneReady = 1;
			}

			break;
		}

		case 0x50: {

			DEBUG_MSG("BLE", "Callsing Setting from phone");

			char call_arr[(uint8_t)conf_data[2] + 1];
			call_arr[(uint8_t)conf_data[2]] = '\0';

			for(int i=0; i<(uint8_t)conf_data[2]; i++)
				call_arr[i] = conf_data[i+3];
			
			String sVar = call_arr;
			sVar.toUpperCase();
			

			sprintf(meshcom_settings.node_call, "%s", sVar.c_str());

            sprintf(meshcom_settings.node_short, "%s", convertCallToShort(meshcom_settings.node_call).c_str());

			save_settings();

			// send config back to phone
			sendConfigToPhone();

			sendDisplayHead(0);			

			sprintf(helper_string, "%s-%02x%02x-%s", g_ble_dev_name, dmac[4], dmac[5], meshcom_settings.node_call);	// Anzeige mit callsign
			Bluefruit.setName(helper_string);

			bInitDisplay = false;

			//restart_ADV = true;	//nicht notwendig

			break;
		}

		case 0x70: {

			DEBUG_MSG("BLE", "Latitude Setting from phone");
			float latitude;
			memcpy(&latitude, conf_data + 2, sizeof(latitude));
			Serial.println(latitude);

			meshcom_settings.node_lat=latitude;

        	save_settings();
			// send config back to phone
			sendConfigToPhone();

			break;
		}

		case 0x80: {

			DEBUG_MSG("BLE", "Longitude Setting from phone");
			float longitude;
			memcpy(&longitude, conf_data + 2, sizeof(longitude));
			Serial.println(longitude);

			meshcom_settings.node_lon=longitude;

        	save_settings();
			// send config back to phone
			sendConfigToPhone();

			break;
		}

		case 0x90: {

			int altitude;
			memcpy(&altitude, conf_data + 2, sizeof(altitude));
			DEBUG_MSG_VAL("BLE", altitude, "Altitude from phone:");

			meshcom_settings.node_alt=altitude;

        	save_settings();
			// send config back to phone
			sendConfigToPhone();

			break;
		}

		case 0xA0: {
			// length 1B - Msg ID 1B - Text

			uint8_t txt_len = msg_len - 1; // includes zero escape
			txt_msg_len_phone = txt_len - 1;	// now zero escape for lora TX

			char textbuff [txt_len] = {0};
			textbuff [txt_len] = '\0';
			memcpy(textbuff, conf_data + 2, txt_len);
			// kopieren der message in buffer fuer main
			memcpy(textbuff_phone,conf_data + 2, txt_msg_len_phone);

			DEBUG_MSG_TXT("BLE", textbuff, "Message from phone:");
			//Serial.printBuffer(textbuff_phone, txt_msg_len_phone);
			//Serial.println();
			// flag für main neue msg von phone
			hasMsgFromPhone = true;

		}

	}

	// BLE mit neuem Call resetten
	if(restart_ADV)
	{
		delay(1000);
		NVIC_SystemReset(); //resets the device 
		// restart_advertising(0);
	}

/*	
	else
	{
		#if BLE_TEST > 0
			if(str[0] == ':')
			{
				if(str[strlen(str)-1] == 0x0a)
					str[strlen(str)-1]=0x00;

				sendMessage(str, strlen(str));
			}
			else
			if(str[0] == '-')
			{
				commandAction(str, strlen(str), true);
			}
		#endif
	}
*/

}

/**
 * @brief Method to send configuration to phone 
 * Config Format:
 * LENGTH 2B - FLAG 1B - LENCALL 1B - Callsign - LAT 8B(Double) - LON 8B(Double) - ALT 4B(INT)
*/
void sendConfigToPhone () {

    ble_busy_flag = true;

	#if BLE_TEST
		char bleBuff [100] = {0};
		sprintf(bleBuff, "Connected to %s\n", meshcom_settings.node_call);
		// send to phone
		g_ble_uart.write(bleBuff, strlen(bleBuff));
	#else
		// assemble conf message
		uint8_t call_len = sizeof(meshcom_settings.node_call);
		uint8_t conf_len = call_len + 22;	// currently fixed length - adapt if needed
		uint8_t confBuff [conf_len] = {0};
		uint8_t call_offset = 2;

		confBuff [0] = 0x80;
		confBuff [1] = call_len;
		memcpy(confBuff + call_offset, meshcom_settings.node_call, call_len);
		uint8_t latOffset = call_offset + call_len;
		memcpy(confBuff + latOffset, &meshcom_settings.node_lat, 8);
		memcpy(confBuff + latOffset + 8, &meshcom_settings.node_lon, 8);
		memcpy(confBuff + latOffset + 16, &meshcom_settings.node_alt, 4);

		// send to phone
		g_ble_uart.write(confBuff, conf_len);
	#endif

	ble_busy_flag = false;
}

/**
 * @brief Initialize the settings characteristic
 *
 */
BLEService init_settings_characteristic(void)
{
	// Initialize the LoRa setting service
	lora_service.begin();
	g_lora_data.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ | CHR_PROPS_WRITE);
	g_lora_data.setPermission(SECMODE_OPEN, SECMODE_OPEN);
	g_lora_data.setFixedLen(sizeof(s_meshcom_settings) + 1);
	g_lora_data.setWriteCallback(settings_rx_callback);

	g_lora_data.begin();

	g_lora_data.write((void *)&meshcom_settings, sizeof(s_meshcom_settings));

	return lora_service;
}

/**
 * Callback if data has been sent from the connected client
 * @param conn_hdl
 * 		The connection handle
 * @param chr
 *      The called characteristic
 * @param data
 *      Pointer to received data
 * @param len
 *      Length of the received data
 */
void settings_rx_callback(uint16_t conn_hdl, BLECharacteristic *chr, uint8_t *data, uint16_t len)
{
	API_LOG("SETT", "Settings received");

	delay(1000);

	// Check the characteristic
	if (chr->uuid == g_lora_data.uuid)
	{
		if (len != sizeof(s_meshcom_settings))
		{
			API_LOG("SETT", "Received settings have wrong size %d", len);
			return;
		}

		s_meshcom_settings *rcvdSettings = (s_meshcom_settings *)data;
		if ((rcvdSettings->valid_mark_1 != 0xAA) || (rcvdSettings->valid_mark_2 != MESHCOM_DATA_MARKER))
		{
			API_LOG("SETT", "Received settings data do not have required markers");
			return;
		}

		// Save new LoRa settings
		memcpy((void *)&meshcom_settings, data, sizeof(s_meshcom_settings));

		// Save new settings
		save_settings();

		// Update settings
		g_lora_data.write((void *)&meshcom_settings, sizeof(s_meshcom_settings));

		// Inform connected device about new settings
		g_lora_data.notify((void *)&meshcom_settings, sizeof(s_meshcom_settings));

        /*KBC
        if (meshcom_settings.resetRequest)
		{
			API_LOG("SETT", "Initiate reset");
			delay(1000);
			sd_nvic_SystemReset();
		}
        */

		// Notify task about the event
		if (g_task_sem != NULL)
		{
			g_task_event_type |= BLE_CONFIG;
			API_LOG("SETT", "Waking up loop task");
			xSemaphoreGive(g_task_sem);
		}
	}
}

#endif