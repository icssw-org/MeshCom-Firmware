#include <loop_functions.h>
#include <loop_functions_extern.h>
#include <phone_commands.h>
#include <debugconf.h>
#include <configuration.h>
#include <batt_functions.h>

// Create device name
extern char helper_string[256];

char textbuff_phone [MAX_MSG_LEN_PHONE] = {0};
uint8_t txt_msg_len_phone = 0;

extern int iInitDisplay;

// Client basic variables
extern uint8_t dmac[6];

// lat / lon from phone. Phone currently sends float and we cast to double. We should fix that
double d_lat = 0.0;
double d_lon = 0.0;

bool ble_busy_flag = false;

void esp32_write_ble(uint8_t confBuff[300], uint8_t conf_len);

extern bool g_ble_uart_is_connected;

/**
 * @brief Method to send configuration to phone 
 * Config Format:
 * LENGTH 2B - FLAG 1B - LENCALL 1B - Callsign - LAT 8B(Double) - LON 8B(Double) - ALT 4B(INT) - 1B SSID_Length - Wifi_SSID - 1B Wifi_PWD - Wifi_PWD - 1B APRS_PRIM_SEC - 1B APRS_SYMBOL
 * SSID and PWD Buffers have as always a fixed length with trailing zeros. 
*/
void sendConfigToPhone ()
{

    ble_busy_flag = true;

	// assemble conf message
	uint8_t call_len = sizeof(meshcom_settings.node_call);
	uint8_t ssid_len = 0;
	uint8_t pwd_len = 0;

	// remove trailing zeros of the arrays
	for(int i =0; i<(int)sizeof(meshcom_settings.node_ssid); i++){
		if(meshcom_settings.node_ssid[i] == 0x00){
			ssid_len = i;
			break;
		} 
	}

	for(int i =0; i<(int)sizeof(meshcom_settings.node_pwd); i++){
		if(meshcom_settings.node_pwd[i] == 0x00){
			pwd_len = i;
			break;
		} 
	}

	
	uint8_t conf_len = call_len + 22 + ssid_len + pwd_len + 4;	// +4 because of APRS Symbols
	uint8_t confBuff [conf_len] = {0};
	uint8_t call_offset = 2;
	

	confBuff [0] = 0x80;
	confBuff [1] = call_len;
	memcpy(confBuff + call_offset, meshcom_settings.node_call, call_len);

	uint8_t latOffset = call_offset + call_len;
	uint8_t ssid_offset = latOffset + 20;				// first byte is ssid_length
	uint8_t pwd_offset = ssid_offset + ssid_len + 1;	// first byte is pwd_length
	uint8_t aprs_symbols_offset = 0; 					// offset for aprs map symbols

	//DEBUG_MSG_VAL("Wifi", ssid_len, "SSID Len");
	//DEBUG_MSG_VAL("Wifi", pwd_len, "PWD Len");
	//DEBUG_MSG_VAL("Wifi", conf_len, "Conf Len");
	//DEBUG_MSG_TXT("Wifi", meshcom_settings.node_ssid, "Wifi SSID to phone");
	//DEBUG_MSG_TXT("Wifi", meshcom_settings.node_pwd, "Wifi PWD");

	memcpy(confBuff + latOffset, &meshcom_settings.node_lat, 8);
	memcpy(confBuff + latOffset + 8, &meshcom_settings.node_lon, 8);
	memcpy(confBuff + latOffset + 16, &meshcom_settings.node_alt, 4);

	// WiFissid
	confBuff[ssid_offset] = ssid_len;
	meshcom_settings.node_ssid[39]=0x00;
	if(strlen(meshcom_settings.node_ssid) < 1 || strlen(meshcom_settings.node_ssid) > 40)
	{
		strcpy(meshcom_settings.node_ssid, "none");
        save_settings();
	}
	memcpy(confBuff + ssid_offset + 1, &meshcom_settings.node_ssid, ssid_len);
	
	// WiFipasswword
	confBuff[pwd_offset] = pwd_len;
	meshcom_settings.node_pwd[39]=0x00;
	if(strlen(meshcom_settings.node_pwd) < 1 || strlen(meshcom_settings.node_pwd) > 40)
	{
		strcpy(meshcom_settings.node_pwd, "none");
        save_settings();
	}
	memcpy(confBuff + pwd_offset + 1, &meshcom_settings.node_pwd, pwd_len);

	// APRS SYMBOLS
	aprs_symbols_offset = pwd_offset + pwd_len;
	memcpy(confBuff + aprs_symbols_offset + 1, &meshcom_settings.node_symid, 1);
	memcpy(confBuff + aprs_symbols_offset + 2, &meshcom_settings.node_symcd, 1);

	//printBuffer(confBuff, conf_len);

	// send to phone
	#if defined(ESP8266) || defined(ESP32)
		esp32_write_ble(confBuff, conf_len);
	#else
		g_ble_uart.write(confBuff, conf_len);
	#endif

	ble_busy_flag = false;
}

/**
 * @brief Method to send incoming LoRa messages to BLE connected device
 * 
*/
void sendToPhone()
{
    if(ble_busy_flag)
        return;

    ble_busy_flag = true;

    if(g_ble_uart_is_connected && isPhoneReady == 1)
    {
		// we need to insert the first byte text msg flag
		uint8_t toPhoneBuff [MAX_MSG_LEN_PHONE] = {0};

		uint16_t blelen = BLEtoPhoneBuff[toPhoneRead][0];   //len ist um ein byte zu kurz

		toPhoneBuff[0] = 0x40;

		memcpy(toPhoneBuff+1, BLEtoPhoneBuff[toPhoneRead]+1, blelen);

		// send to phone
		#if defined(ESP8266) || defined(ESP32)
			blelen=blelen+2;
			esp32_write_ble(toPhoneBuff, blelen);
		#else
			g_ble_uart.write(toPhoneBuff, blelen + 2);
		#endif

		toPhoneRead++;
		if (toPhoneRead >= MAX_RING)
			toPhoneRead = 0;

		if(bDEBUG)
			Serial.printf("toPhoneWrite:%i toPhoneRead:%i \n", toPhoneWrite, toPhoneRead);
    }
    
    ble_busy_flag = false;
}

void readPhoneCommand(uint8_t conf_data[MAX_MSG_LEN_PHONE])
{
	/**
	 * Config Messages
     * length 1B - Msg ID 1B - Data
	 * Msg ID:
	 * 0x10 - Hello Message (followed by 0x20, 0x30)
	 * 0x50 - Callsign
	 * 0x55 - Wifi SSID and PW
	 * 0x70 - Latitude
	 * 0x80 - Longitude
	 * 0x90 - Altitude
	 * 0x95 - APRS Symbols
	 * 0xA0 - Textmessage
     * Data:
     * Callsign: length Callsign 1B - Callsign
     * Latitude: 4B Float
     * Longitude: 4B Float
     * Altitude: 4B Integer
	 * 
	 * WiFi SSID and PWD:
	 * 1B - SSID Length - SSID - 1B PWD Length - PWD
	 * 
     * Position Settings from phone are: length 1B | Msg ID 1B | 4B lat/lon/alt | 1B save_settings_flag
	 * Save_flag is 0x0A for save and 0x0B for don't save
	 * If phone send periodicaly position, we don't save them.
	 * 
	 * currently we save the settings when the last config arrives which is APRS SYMBOLS - adapt is needed!
     *  */ 


	uint8_t msg_len = conf_data[0];
	uint8_t msg_type = conf_data[1];
	bool restart_ADV = false;
	bool save_setting = false;		//flag to save when positions from phone. config or periodic positions
	float lat_phone = 0.0;
	float long_phone = 0.0;

	DEBUG_MSG_VAL("BLE", msg_len, "Msg from Device Length");

	if(bDEBUG)
	{
		printBuffer(conf_data, msg_len);
		Serial.println();
	}

	// get save settings flag if position setting
	if(msg_type == 0x70 || msg_type == 0x80 || msg_type == 0x90)
	{

		if(conf_data[6] == 0x0A)  save_setting = true;
		if(conf_data[6] == 0x0B)  save_setting = false;
	}

	switch (msg_type){

		case 0x10: {

			if(conf_data[2] == 0x20 && conf_data[3] == 0x30){
				DEBUG_MSG("BLE", "Hello Msg from phone");
				sendConfigToPhone();
				isPhoneReady = 1;
			}

			break;
		}

		case 0x50:
		{

			DEBUG_MSG("BLE", "Callsing Setting from phone");

			char call_arr[(uint8_t)conf_data[2] + 1];
			call_arr[(uint8_t)conf_data[2]] = '\0';

			for (int i = 0; i < (uint8_t)conf_data[2]; i++)
				call_arr[i] = conf_data[i + 3];

			String sVar = call_arr;
			sVar.toUpperCase();

			sprintf(meshcom_settings.node_call, "%s", sVar.c_str());

			sprintf(meshcom_settings.node_short, "%s", convertCallToShort(meshcom_settings.node_call).c_str());


			sendDisplayHead();

			#if defined NRF52_SERIES
				sprintf(helper_string, "%s-%02x%02x-%s", g_ble_dev_name, dmac[4], dmac[5], meshcom_settings.node_call); // Anzeige mit callsign
				
				Serial.print("helper_string:");
				Serial.println(helper_string);

				Bluefruit.setName(helper_string);
			#endif

			iInitDisplay = 99;

			// restart_ADV = true;	//nicht notwendig

			break;
		}

		case 0x70:
		{

			DEBUG_MSG("BLE", "Latitude Setting from phone");
			memcpy(&lat_phone, conf_data + 2, sizeof(lat_phone));
			d_lat = (double)lat_phone;
			
			if(bDEBUG)
			{
				Serial.println(lat_phone);
			}

			if (save_setting)
			{
				meshcom_settings.node_lat = lat_phone;

			}

			break;
		}

		case 0x80:
		{

			DEBUG_MSG("BLE", "Longitude Setting from phone");

			memcpy(&long_phone, conf_data + 2, sizeof(long_phone));
			d_lon = (double)long_phone;
		
			if(bDEBUG)
			{
				Serial.println(long_phone);
			}

			if (save_setting)
			{
				meshcom_settings.node_lon = long_phone;

			}

			break;
		}

		case 0x90:
		{
			int altitude = 0;
			memcpy(&altitude, conf_data + 2, sizeof(altitude));
			DEBUG_MSG_VAL("BLE", altitude, "Altitude from phone:");

			if (save_setting)
			{
				meshcom_settings.node_alt = altitude;

			}
			else
			{
				// send to mesh - phone sends pos perdiocaly
				/// TODO N/S und E/W Char muss man noch korrekt setzen
				DEBUG_MSG("RADIO", "Sending Pos from Phone to Mesh");

				sendPosition(d_lat, meshcom_settings.node_lat_c, d_lon, meshcom_settings.node_lon_c, altitude);

				posinfo_timer = millis();	// damit die loop-Schleife nicht asynchron läuft
			}

			break;
		}

		case 0x95: {

			char aprs_pri_sec = conf_data[2];
			char aprs_symbol = conf_data[3];

			DEBUG_MSG_VAL("BLE", aprs_pri_sec, "APRS PRI_SEC Symbol");
			DEBUG_MSG_VAL("BLE", aprs_symbol, "APRS Symbol");

			if(aprs_pri_sec == 0x2f || aprs_pri_sec == 0x5c){

				// Variablen entsprechend setzen beim APRS Encode
				meshcom_settings.node_symid = aprs_pri_sec;
				meshcom_settings.node_symcd = aprs_symbol;

				// currently the last config coming from phone, so we save settings here
				save_settings();
				delay(1000);
				
				// send config back to phone
				sendConfigToPhone();

				// reset node
				delay(2000);
				#if defined NRF52_SERIES
					NVIC_SystemReset();
				#else
					ESP.restart();
				#endif

			}
		}

		case 0xA0: {
			// length 1B - Msg ID 1B - Text

			DEBUG_MSG("BLE", "Text from phone");

			txt_msg_len_phone = msg_len - 2;	// now zero escape for lora TX

			// kopieren der message in buffer fuer main
			memcpy(textbuff_phone, conf_data + 2, txt_msg_len_phone);
			textbuff_phone[txt_msg_len_phone]=0x00;

			//Serial.printBuffer(textbuff_phone, txt_msg_len_phone);
			//Serial.println();
			// flag für main neue msg von phone
			hasMsgFromPhone = true;

		}

		case 0x55: {
			// 1B - SSID Length - SSID - 1B PWD Length - PWD

			DEBUG_MSG("BLE", "Wifi Setting from phone");
			
			uint8_t ssid_len = conf_data[2];
			uint8_t pwd_len = conf_data[ssid_len + 3];

			if(ssid_len > 0 && pwd_len > 0)
			{
				char ssid_arr [ssid_len +1] = {0};
				char pwd_arr [pwd_len +1] = {0};

				ssid_arr[ssid_len +1] = '\0';
				pwd_arr[ssid_len +1] = '\0';
				
				memcpy(ssid_arr, conf_data + 3, ssid_len);
				memcpy(pwd_arr, conf_data + (4 + ssid_len), pwd_len);

				String s_SSID = ssid_arr;
				String s_PWD = pwd_arr;

				sprintf(meshcom_settings.node_ssid, "%s", s_SSID.c_str());
				sprintf(meshcom_settings.node_pwd, "%s", s_PWD.c_str());

				Serial.println("Wifi Setting from phone set");
			}
			
		}

	}

	// BLE mit neuem Call resetten
	if(restart_ADV)
	{
		#if defined NRF52_SERIES
			delay(1000);
			NVIC_SystemReset(); //resets the device 
			// restart_advertising(0);
		#endif
	}
}
