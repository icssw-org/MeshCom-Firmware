#include <loop_functions.h>
#include <loop_functions_extern.h>
#include <phone_commands.h>
#include <debugconf.h>
#include <configuration.h>
#include <batt_functions.h>

extern void sendConfigToPhone ();

// Create device name
extern char helper_string[256];

extern char textbuff_phone [MAX_MSG_LEN_PHONE];
extern uint8_t txt_msg_len_phone;
extern bool bInitDisplay;

// Client basic variables
extern uint8_t dmac[6];

// lat / lon from phone. Phone currently sends float and we cast to double. We should fix that
double d_lat = 0.0;
double d_lon = 0.0;

void readPhoneCommand(uint8_t conf_data[MAX_MSG_LEN_PHONE])
{
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
     * Latitude: 4B Float
     * Longitude: 4B Float
     * Altitude: 4B Integer
	 * 
     * Position Settings from phone are: length 1B | Msg ID 1B | 4B lat/lon/alt | 1B save_settings_flag
	 * Save_flag is 0x0A for save and 0x0B for don't save
	 * If phone send periodicaly position, we don't save them.
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

			save_settings();

			// send config back to phone
			sendConfigToPhone();

			sendDisplayHead((int)mv_to_percent(read_batt()));

			#if defined NRF52_SERIES
				sprintf(helper_string, "%s-%02x%02x-%s", g_ble_dev_name, dmac[4], dmac[5], meshcom_settings.node_call); // Anzeige mit callsign
				
				Serial.print("helper_string:");
				Serial.println(helper_string);

				Bluefruit.setName(helper_string);
			#endif

			bInitDisplay = false;

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

				// send config back to phone
				sendConfigToPhone();
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

				// send config back to phone
				sendConfigToPhone();
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

				// if we have all three pos settings, save to flash
				save_settings();
				// send config back to phone
				sendConfigToPhone();
			}
			else
			{
				// send to mesh - phone sends pos perdiocaly
				/// TODO N/S und E/W Char muss man noch korrekt setzen
				DEBUG_MSG("RADIO", "Sending Pos from Phone to Mesh");

				sendPosition(d_lat, meshcom_settings.node_lat_c, d_lon, meshcom_settings.node_lon_c, altitude, (int)mv_to_percent(read_batt()));

				posinfo_timer = millis();	// damit die loop-Schleife nicht asynchron läuft
			}

			break;
		}

		case 0xA0: {
			// length 1B - Msg ID 1B - Text

			txt_msg_len_phone = msg_len - 2;	// now zero escape for lora TX

			// kopieren der message in buffer fuer main
			memcpy(textbuff_phone, conf_data + 2, txt_msg_len_phone);
			textbuff_phone[txt_msg_len_phone]=0x00;

			//Serial.printBuffer(textbuff_phone, txt_msg_len_phone);
			//Serial.println();
			// flag für main neue msg von phone
			hasMsgFromPhone = true;

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