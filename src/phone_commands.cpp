#include <loop_functions.h>
#include <loop_functions_extern.h>
#include <phone_commands.h>
#include <debugconf.h>
#include <configuration.h>
#include <batt_functions.h>
#include <time.h>
#include <clock.h>
#include <rtc_functions.h>

//#include <command_functions.h>

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

extern uint8_t shortVERSION();

/**
 * DEPRECATED - NOT USED ANYMORE
 * @brief Method to send configuration to phone 
 * Config Format:
 * LENGTH 2B - FLAG 1B - LENCALL 1B - Callsign - LAT 8B(Double) - LON 8B(Double) - ALT 4B(INT) - 1B SSID_Length - Wifi_SSID - 1B Wifi_PWD - Wifi_PWD 
 * - 1B APRS_PRIM_SEC - 1B APRS_SYMBOL - 4B SettingsMask - 1B HW-ID - 1B MOD-ID - 1B FW-Vers - 1B TX Pwr - 4B Frequency - 1B Comment Length - Comment 
 * - 4B Settingsbyte2 - 0x00
 * 
 * SSID and PWD Buffers have as always a fixed length with trailing zeros. 
*/
void sendConfigToPhone ()
{

    ble_busy_flag = true;

	// assemble conf message
	uint8_t call_len = sizeof(meshcom_settings.node_call);
	uint8_t ssid_len = 0;
	uint8_t pwd_len = 0;
	uint8_t comment_len = 0;

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

	for(int i =0; i<(int)sizeof(meshcom_settings.node_atxt); i++){
		if(meshcom_settings.node_atxt[i] == 0x00){
			comment_len = i;
			break;
		} 
	}

	if(bBLEDEBUG)
		Serial.println("Comment Len: " + String(comment_len) + " Comment: " + String(meshcom_settings.node_atxt) + " Comment StrLen: " + String(strlen(meshcom_settings.node_atxt)));

	
	uint8_t conf_len = call_len + ssid_len + pwd_len + comment_len + 44;	// +9 because of APRS Symbols 2B, Settings 4B, till FRQ and 0x00 end
	uint8_t confBuff [conf_len] = {0};
	uint8_t call_offset = 2;
	

	confBuff [0] = 0x80;
	confBuff [1] = call_len;
	memcpy(confBuff + call_offset, meshcom_settings.node_call, call_len);

	uint8_t latOffset = call_offset + call_len;
	uint8_t ssid_offset = latOffset + 20;				// first byte is ssid_length
	uint8_t pwd_offset = ssid_offset + ssid_len + 1;	// first byte is pwd_length
	uint8_t aprs_symbols_offset = 0; 					// offset for aprs map symbols
	uint8_t gw_cl_offset = 0;							// settings masked
	uint8_t endIndex = 0;								// last byte is 0x00

	//DEBUG_MSG_VAL("Wifi", ssid_len, "SSID Len");
	//DEBUG_MSG_VAL("Wifi", pwd_len, "PWD Len");
	//DEBUG_MSG_VAL("Wifi", conf_len, "Conf Len");
	//DEBUG_MSG_TXT("Wifi", meshcom_settings.node_ssid, "Wifi SSID to phone");
	//DEBUG_MSG_TXT("Wifi", meshcom_settings.node_pwd, "Wifi PWD");

	d_lat=meshcom_settings.node_lat;
	if(meshcom_settings.node_lat_c == 'S')
		d_lat=meshcom_settings.node_lat * -1.0;

	memcpy(confBuff + latOffset, &d_lat, 8);

	d_lon=meshcom_settings.node_lon;
	if(meshcom_settings.node_lon_c == 'W')
		d_lon=meshcom_settings.node_lon * -1.0;

	memcpy(confBuff + latOffset + 8, &d_lon, 8);
	
	memcpy(confBuff + latOffset + 16, &meshcom_settings.node_alt, 4);

	// WiFissid
	confBuff[ssid_offset] = ssid_len;
	meshcom_settings.node_ssid[32]=0x00;
	if(strlen(meshcom_settings.node_ssid) < 1 || strlen(meshcom_settings.node_ssid) > 32)
	{
		strcpy(meshcom_settings.node_ssid, "none");
        save_settings();
	}
	memcpy(confBuff + ssid_offset + 1, &meshcom_settings.node_ssid, ssid_len);
	
	// WiFipasswword
	confBuff[pwd_offset] = pwd_len;
	meshcom_settings.node_pwd[63]=0x00;
	if(strlen(meshcom_settings.node_pwd) < 1 || strlen(meshcom_settings.node_pwd) > 63)
	{
		strcpy(meshcom_settings.node_pwd, "none");
        save_settings();
	}
	memcpy(confBuff + pwd_offset + 1, &meshcom_settings.node_pwd, pwd_len);

	// APRS SYMBOLS
	aprs_symbols_offset = pwd_offset + pwd_len;
	memcpy(confBuff + aprs_symbols_offset + 1, &meshcom_settings.node_symid, 1);
	memcpy(confBuff + aprs_symbols_offset + 2, &meshcom_settings.node_symcd, 1);

	if(bBLEDEBUG)
		Serial.printf("Sym ID: %c Sym CD: %c\n", meshcom_settings.node_symid, meshcom_settings.node_symcd);

	// Settings
	gw_cl_offset = aprs_symbols_offset + 3;
	memcpy(confBuff + gw_cl_offset, &meshcom_settings.node_sset, sizeof(meshcom_settings.node_sset));

	// HW-ID, MOD-ID, FW-Version (FW Vers. deprecated!)
	uint8_t hw_offset = gw_cl_offset + sizeof(meshcom_settings.node_sset);
	uint8_t hw_id = BOARD_HARDWARE;
	memcpy(confBuff + hw_offset, &hw_id, sizeof(hw_id));

	// Mod-ID is currently fixed to 3 -> SF11CR46BW250
	uint8_t mod_id = 3;
	uint8_t mod_offset = hw_offset + sizeof(hw_id);
	memcpy(confBuff + mod_offset, &mod_id, sizeof(mod_id));

	// FW-Version (short version only 1B 4.x)
	uint8_t fw_vers = shortVERSION();
	uint8_t fw_vers_offset = mod_offset + sizeof(mod_id);
	memcpy(confBuff + fw_vers_offset, &fw_vers, sizeof(fw_vers));

	// TX Power
	char tx_pwr = (char) meshcom_settings.node_power;	// setting is INT values are from -5 to +30
	uint8_t tx_pwr_offset = fw_vers_offset + sizeof(tx_pwr);
	memcpy(confBuff + tx_pwr_offset, &tx_pwr, sizeof(tx_pwr));

	// Frequency 
	uint8_t frq_offset = tx_pwr_offset + sizeof(tx_pwr);
	// if the frequency is set to 0 it wasn't saved/set to flash so far -> default QRG
	float frq = 0.0;

	if(meshcom_settings.node_freq == 0){
		#ifdef BOARD_RAK4630
			frq = (float) RF_FREQUENCY;
			frq = frq / 1000000.0;
		#else
			frq = RF_FREQUENCY;
		#endif
		
	} else {
		frq = meshcom_settings.node_freq;
	}
	memcpy(confBuff + frq_offset, &frq, sizeof(frq));


	// add meshcom_settings.node_atxt to confBuff
	uint8_t comment_offset = frq_offset + sizeof(frq);
	confBuff[comment_offset] = comment_len;

	if(comment_len != 0){
		// copy comment
		memcpy(confBuff + comment_offset + 1, &meshcom_settings.node_atxt, comment_len);
	} else {
		// no comment
		confBuff[comment_offset] = 0x00;
	}
	
	
	// second settings byte
	gw_cl_offset = 0;
	gw_cl_offset = comment_offset + comment_len + 1;
	memcpy(confBuff + gw_cl_offset, &meshcom_settings.node_sset2, sizeof(meshcom_settings.node_sset2));


	// add 0x00 at end
	endIndex = gw_cl_offset + sizeof(meshcom_settings.node_sset2);
	confBuff[endIndex] = 0x00;

	//printBuffer(confBuff, conf_len);

	// send to phone
	#if defined(ESP8266) || defined(ESP32)
		esp32_write_ble(confBuff, conf_len);
	#else
		g_ble_uart.write(confBuff, conf_len);
	#endif

	ble_busy_flag = false;

	// one shot GPS
	posinfo_shot = true;

	// is done in main loop now
	//pos_shot = true;

	//wx_shot = true;

	//BattTimeAPP=0;	// Batt Status zum Phone senden

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
		// MAXIMUM PACKET Length over BLE is 245 (MTU=247 bytes), two get lost, otherwise we need to split it up!
		uint8_t blelen = BLEtoPhoneBuff[toPhoneRead][0];

		//Mheard
		if(BLEtoPhoneBuff[toPhoneRead][1] == 0x91)
		{
			memcpy(toPhoneBuff, BLEtoPhoneBuff[toPhoneRead]+1, blelen-1);
		}
		else 
		// Data Message (JSON)
		if(BLEtoPhoneBuff[toPhoneRead][1] == 0x44)
		{		
			memcpy(toPhoneBuff, BLEtoPhoneBuff[toPhoneRead]+1, blelen);	
		} 
		else
		// Text Message and Position
		{
			toPhoneBuff[0] = 0x40;
			memcpy(toPhoneBuff+1, BLEtoPhoneBuff[toPhoneRead]+1, blelen);
		}

		// send to phone
		// why do we need to add 2 bytes??
		#if defined(ESP8266) || defined(ESP32)
			blelen=blelen + 2;
			esp32_write_ble(toPhoneBuff, blelen);
		#else
			g_ble_uart.write(toPhoneBuff, blelen + 2);
		#endif

		toPhoneRead++;
		if (toPhoneRead >= MAX_RING)
			toPhoneRead = 0;

		if(bBLEDEBUG)
		{
			if(toPhoneBuff[0] == ':' || toPhoneBuff[0] == '!' || toPhoneBuff[0] == '@')
				Serial.printf("toPhoneWrite:%i toPhoneRead:%i buff:%s\n", toPhoneWrite, toPhoneRead, toPhoneBuff+7);
			else
				Serial.printf("toPhoneWrite:%i toPhoneRead:%i buff:%s\n", toPhoneWrite, toPhoneRead, toPhoneBuff);
		}
    }
    
    ble_busy_flag = false;
}

/**
 * @brief Method to send incoming LoRa messages to BLE connected device
 * 
*/
void sendComToPhone()
{
    if(ble_busy_flag)
        return;

    ble_busy_flag = true;

    if(g_ble_uart_is_connected && isPhoneReady == 1)
    {
		// we need to insert the first byte text msg flag
		uint8_t ComToPhoneBuff [MAX_MSG_LEN_PHONE] = {0};
		// MAXIMUM PACKET Length over BLE is 245 (MTU=247 bytes), two get lost, otherwise we need to split it up!
		uint8_t blelen = BLEComToPhoneBuff[ComToPhoneRead][0];

		//Mheard
		if(BLEComToPhoneBuff[ComToPhoneRead][1] == 0x91)
		{
			memcpy(ComToPhoneBuff, BLEComToPhoneBuff[ComToPhoneRead]+1, blelen-1);
		} else 
		// Data Message (JSON)
		if(BLEComToPhoneBuff[ComToPhoneRead][1] == 0x44)
		{
			
			memcpy(ComToPhoneBuff, BLEComToPhoneBuff[ComToPhoneRead]+1, blelen);

			
		} 
		else
		// Text Message
		{
			ComToPhoneBuff[0] = 0x40;
			memcpy(ComToPhoneBuff+1, BLEComToPhoneBuff[ComToPhoneRead]+1, blelen-1);

		}

		// send to phone
		// why do we need to add 2 bytes??
		#if defined(ESP8266) || defined(ESP32)
			blelen=blelen + 2;
			esp32_write_ble(ComToPhoneBuff, blelen);
		#else
			g_ble_uart.write(ComToPhoneBuff, blelen + 2);
		#endif

		ComToPhoneRead++;
		if (ComToPhoneRead >= MAX_RING)
			ComToPhoneRead = 0;

		if(bBLEDEBUG)
		{
			if(ComToPhoneBuff[0] == ':' || ComToPhoneBuff[0] == '!' || ComToPhoneBuff[0] == '@')
				Serial.printf("ComToPhoneWrite:%i ComToPhoneRead:%i buff:%s\n", ComToPhoneWrite, ComToPhoneRead, ComToPhoneBuff+7);
			else
				Serial.printf("ComToPhoneWrite:%i ComToPhoneRead:%i buff:%s\n", ComToPhoneWrite, ComToPhoneRead, ComToPhoneBuff);
		}
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
	 * 0x20 - Timestamp from phone 
	 * 0x50 - Callsign
	 * 0x55 - Wifi SSID and PW
	 * 0x70 - Latitude
	 * 0x80 - Longitude
	 * 0x90 - Altitude
	 * 0x95 - APRS Symbols
	 * 0xA0 - Textmessage
	 * 0xF0 - Save Settings to Flash
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
	uint8_t msg_payload_len = conf_data[2];

	bool save_setting = false;		//flag to save when positions from phone. config or periodic positions
	float lat_phone = 0.0;
	float long_phone = 0.0;

	DEBUG_MSG_VAL("BLE", msg_len, "Msg from Device Length");

	if(bBLEDEBUG)
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

	//Serial.printf("msg_type:%02x\n", msg_type);

	switch (msg_type)
	{
		case 0x10: {

			if(conf_data[2] == 0x20 && conf_data[3] == 0x30){

				//sendConfigToPhone(); // config data comes now via JSONs from main loop and commandfunctions

				if(bBLEDEBUG)
					Serial.println("BLE Hello Msg from phone");
				
				isPhoneReady = 1;
			}

			break;
		}

		case 0x20: {

			if(bBLEDEBUG)
				Serial.print("[BLE] Timestamp from phone received ");

			// 4B Timestamp
			uint32_t timestamp = 0;
			memcpy(&timestamp, conf_data + 2, sizeof(timestamp));
			// set the meshcom settings variables for the timestamp
			struct tm timeinfo = {0};
			gmtime_r((time_t*)&timestamp, &timeinfo);

			if(bBLEDEBUG)
				Serial.printf("%i.%i.%i %i:%i:%i\n", timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

			// set the clock
			if(bRTCON)
			{
				setRTCNow(timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
				
				DateTime utc = getRTCNow();

				DateTime now (utc + TimeSpan(meshcom_settings.node_utcoff * 60 * 60));

				meshcom_settings.node_date_year = now.year();
				meshcom_settings.node_date_month = now.month();
				meshcom_settings.node_date_day = now.day();

				meshcom_settings.node_date_hour = now.hour();
				meshcom_settings.node_date_minute = now.minute();
				meshcom_settings.node_date_second = now.second();
			}
			else
			{
				MyClock.setCurrentTime(meshcom_settings.node_utcoff, timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

				MyClock.CheckEvent();
				
				meshcom_settings.node_date_year = MyClock.Year();
				meshcom_settings.node_date_month = MyClock.Month();
				meshcom_settings.node_date_day = MyClock.Day();

				meshcom_settings.node_date_hour = MyClock.Hour();
				meshcom_settings.node_date_minute = MyClock.Minute();
				meshcom_settings.node_date_second = MyClock.Second();
			}

			if (bBLEDEBUG)
			{
				Serial.printf("[BLE] Timestamp from phone <UTC>: %u\n", timestamp);
				Serial.printf("[BLE] Date <LT>: %02d.%02d.%04d %02d:%02d:%02d\n", meshcom_settings.node_date_day, meshcom_settings.node_date_month, meshcom_settings.node_date_year, meshcom_settings.node_date_hour, meshcom_settings.node_date_minute, meshcom_settings.node_date_second);
			}

			break;
		}

		case 0x50:
		{

			DEBUG_MSG("BLE", "Callsing Setting from phone");

			char call_arr[msg_payload_len + 1];

			for (int i = 0; i < msg_payload_len; i++)
			{
				call_arr[i] = conf_data[i + 3];
				call_arr[i+1] = 0x00;
			}


			String sVar = call_arr;
			sVar.toUpperCase();
			sVar.trim();

			snprintf(meshcom_settings.node_call, sizeof(meshcom_settings.node_call), "%s", sVar.c_str());

			snprintf(meshcom_settings.node_short, sizeof(meshcom_settings.node_short), "%s", convertCallToShort(meshcom_settings.node_call).c_str());

			//Führt zu Reconnect sendDisplayHead(false);

			#if defined NRF52_SERIES
				snprintf(helper_string, sizeof(helper_string),"%s-%02x%02x-%s", g_ble_dev_name, dmac[4], dmac[5], meshcom_settings.node_call); // Anzeige mit callsign
				
				if(bBLEDEBUG)
				{
					Serial.print("helper_string:");
					Serial.println(helper_string);
				}

				Bluefruit.setName(helper_string);
			#endif

			iInitDisplay = 99;

			break;
		}

		case 0x70:
		{

			DEBUG_MSG("BLE", "Latitude Setting from phone");
			memcpy(&lat_phone, conf_data + 2, sizeof(lat_phone));
			
			d_lat = (double)lat_phone;
			
			meshcom_settings.node_lat_c='N';
			meshcom_settings.node_lat=d_lat;

			if(d_lat < 0)
			{
				meshcom_settings.node_lat_c='S';
				meshcom_settings.node_lat=fabs(d_lat);
			}

			break;
		}

		case 0x80:
		{

			DEBUG_MSG("BLE", "Longitude Setting from phone");

			memcpy(&long_phone, conf_data + 2, sizeof(long_phone));
			d_lon = (double)long_phone;
		
			meshcom_settings.node_lon_c='E';
			meshcom_settings.node_lon=d_lon;

			if(d_lon < 0)
			{
				meshcom_settings.node_lon_c='W';
				meshcom_settings.node_lon=fabs(d_lon);
			}

			break;
		}

		case 0x90:
		{
			int altitude = 0;
			memcpy(&altitude, conf_data + 2, sizeof(altitude));
			DEBUG_MSG_VAL("BLE", altitude, "Altitude from phone:");

			meshcom_settings.node_alt = altitude;

			if (!save_setting)
			{
				// send to mesh - phone sends pos perdiocaly
				DEBUG_MSG("RADIO", "Sending Pos from Phone to Mesh");
				
				posinfo_shot = true;

				pos_shot = true;
				
				wx_shot = true;
			} else {
				// save settings
				save_settings();
			}

			break;
		}

		case 0x95: {

			char aprs_pri_sec = conf_data[2];
			char aprs_symbol = conf_data[3];

			if(bBLEDEBUG)
				Serial.printf("aprs_pri_sec:%c aprs_symbol:%c\n", aprs_pri_sec, aprs_symbol);

			if(aprs_pri_sec == 0x2f || aprs_pri_sec == 0x5c)
			{

				// Variablen entsprechend setzen beim APRS Encode
				meshcom_settings.node_symid = aprs_pri_sec;
				meshcom_settings.node_symcd = aprs_symbol;

			}
			break;
		}

		case 0xA0: {
			// length 1B - Msg ID 1B - Text

			txt_msg_len_phone = msg_len - 2;	// now zero escape for lora TX

			if(ble_busy_flag && bBLEDEBUG)
				Serial.println("BLE is busy. Waiting...");

			while(ble_busy_flag)
			{
				delay(5);
			}

			if (bBLEDEBUG)
				Serial.printf("Text from phone: %s\n", conf_data + 2);

			// kopieren der message in buffer fuer main
			int iposn=0;
			if(memcmp(conf_data + 2, "--", 2) != 0)
			{
				textbuff_phone[0] = ':';
				iposn=1;
			}
			memcpy(textbuff_phone+iposn, conf_data + 2, txt_msg_len_phone);
			textbuff_phone[txt_msg_len_phone]=0x00;

			// flag für main neue msg von phone
			hasMsgFromPhone = true;
			break;
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

				snprintf(meshcom_settings.node_ssid, sizeof(meshcom_settings.node_ssid),"%s", s_SSID.c_str());
				snprintf(meshcom_settings.node_pwd, sizeof(meshcom_settings.node_pwd),"%s", s_PWD.c_str());

				if(bBLEDEBUG)
					Serial.println("Wifi Setting from phone set");
				
				// Node will reset after saving settings. Settings back are coming on ble reconnect.

			}
			break;
		}

		case 0xF0: {
			
			if(bBLEDEBUG)
				Serial.println("Save Settings");
			
			//Save Settings

			save_settings();
			//delay(1000);
			
			// send config back to phone
			//sendConfigToPhone();	// config data comes now via JSONs from main loop and commandfunctions

			// reset node
			delay(2000);
			#if defined NRF52_SERIES
				NVIC_SystemReset();
			#else
				ESP.restart();
			#endif
		}
	}
}
