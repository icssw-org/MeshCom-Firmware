/**
 * @file flash-nrf52.cpp
 * @author Bernd Giesecke (bernd.giesecke@rakwireless.com)
 * @brief Initialize, read and write parameters from/to internal flash memory
 * @version 0.1
 * @date 2021-01-10
 *
 * @copyright Copyright (c) 2021
 *
 */
#ifdef NRF52_SERIES

#include <debugconf.h>

#include "WisBlock-API.h"

s_meshcom_settings meshcom_settings;

s_meshcom_settings g_flash_content;
s_meshcomcompat_settings g_flash_content_compat;

#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
using namespace Adafruit_LittleFS_Namespace;

const char settings_name[] = "MeshCom-RAK";

File lora_file(InternalFS);

void flash_int_reset(void);

/**
 * @brief Initialize access to nRF52 internal file system
 *
 */
void init_flash(void)
{
	if (init_flash_done)
	{
		return;
	}

	// Initialize Internal File System
	InternalFS.begin();

	// Check if file exists
	lora_file.open(settings_name, FILE_O_READ);
	if (!lora_file)
	{
		DEBUG_MSG("FLASH", "File doesn't exist, force format");
		delay(1000);
		flash_reset();
		lora_file.open(settings_name, FILE_O_READ);
	}

	uint8_t markers[2] = {0};
	lora_file.read(markers, 2);
	if ((markers[0] == 0xAA) && (markers[1] == MESHCOM_COMPAT_MARKER))
	{
		DEBUG_MSG("FLASH", "File has old structure, merge into new structure");
		// Found old structure
		lora_file.close();
		// Read data into old structure
		s_meshcomcompat_settings old_struct;
		lora_file.open(settings_name, FILE_O_READ);
		lora_file.read((uint8_t *)&old_struct, sizeof(s_meshcomcompat_settings));
		lora_file.close();
		// Merge old structure into new structure

        memcpy(meshcom_settings.node_call, old_struct.node_call, 10);
        memcpy(meshcom_settings.node_short, old_struct.node_short, 5);

		meshcom_settings.node_lat = old_struct.node_lat;
		meshcom_settings.node_lat_c = old_struct.node_lat_c;
		meshcom_settings.node_lon = old_struct.node_lon;
		meshcom_settings.node_lon_c = old_struct.node_lon_c;
		meshcom_settings.node_alt = old_struct.node_alt;
		meshcom_settings.node_symid = old_struct.node_symid;
		meshcom_settings.node_symcd = old_struct.node_symcd;

		meshcom_settings.auto_join = old_struct.auto_join;
		memcpy(meshcom_settings.node_device_eui, old_struct.node_device_eui, 8);

		meshcom_settings.node_date_year = old_struct.node_date_year;
		meshcom_settings.node_date_month = old_struct.node_date_month;
		meshcom_settings.node_date_day = old_struct.node_date_day;

		meshcom_settings.node_date_hour = old_struct.node_date_hour;
		meshcom_settings.node_date_minute = old_struct.node_date_minute;
		meshcom_settings.node_date_second = old_struct.node_date_second;
		meshcom_settings.node_date_hundredths = old_struct.node_date_hundredths;

		meshcom_settings.node_age = old_struct.node_age;

		meshcom_settings.node_temp = old_struct.node_temp;
		meshcom_settings.node_hum = old_struct.node_hum;
		meshcom_settings.node_press = old_struct.node_press;

		memcpy(meshcom_settings.node_ossid, old_struct.node_ossid, 40);
		memcpy(meshcom_settings.node_opwd, old_struct.node_opwd, 40);

		meshcom_settings.node_hamnet_only = old_struct.node_hamnet_only;

		meshcom_settings.node_sset = old_struct.node_sset;

		meshcom_settings.node_maxv = old_struct.node_maxv;

		memcpy(meshcom_settings.node_extern, old_struct.node_extern, 40);

		meshcom_settings.node_msgid = old_struct.node_msgid;
		meshcom_settings.node_ackid = old_struct.node_ackid;

		meshcom_settings.node_power = old_struct.node_power;
		meshcom_settings.node_freq = old_struct.node_freq;
		meshcom_settings.node_bw = old_struct.node_bw;
		meshcom_settings.node_sf = old_struct.node_sf;
		meshcom_settings.node_cr = old_struct.node_cr;

		memcpy(meshcom_settings.node_atxt, old_struct.node_atxt, 40);

		meshcom_settings.node_sset2 = old_struct.node_sset2;
		meshcom_settings.node_owgpio = old_struct.node_owgpio;
		meshcom_settings.node_temp2 = old_struct.node_temp2;

    	meshcom_settings.node_utcoff = old_struct.node_utcoff;

		// BME680
    	meshcom_settings.node_gas_res = old_struct.node_gas_res;

		// MCU-811
    	meshcom_settings.node_co2 = old_struct.node_co2;

		// MCP23017
    	meshcom_settings.node_mcp17io = old_struct.node_mcp17io;
    	meshcom_settings.node_mcp17out = old_struct.node_mcp17out;
    	meshcom_settings.node_mcp17in = old_struct.node_mcp17in;

		memcpy(meshcom_settings.node_mcp17t[0], old_struct.node_mcp17t[0], 16);
		memcpy(meshcom_settings.node_mcp17t[1], old_struct.node_mcp17t[1], 16);
		memcpy(meshcom_settings.node_mcp17t[2], old_struct.node_mcp17t[2], 16);
		memcpy(meshcom_settings.node_mcp17t[3], old_struct.node_mcp17t[3], 16);
		memcpy(meshcom_settings.node_mcp17t[4], old_struct.node_mcp17t[4], 16);
		memcpy(meshcom_settings.node_mcp17t[5], old_struct.node_mcp17t[5], 16);
		memcpy(meshcom_settings.node_mcp17t[6], old_struct.node_mcp17t[6], 16);
		memcpy(meshcom_settings.node_mcp17t[7], old_struct.node_mcp17t[7], 16);

		memcpy(meshcom_settings.node_mcp17t[8], old_struct.node_mcp17t[8], 16);
		memcpy(meshcom_settings.node_mcp17t[9], old_struct.node_mcp17t[9], 16);
		memcpy(meshcom_settings.node_mcp17t[10], old_struct.node_mcp17t[10], 16);
		memcpy(meshcom_settings.node_mcp17t[11], old_struct.node_mcp17t[11], 16);
		memcpy(meshcom_settings.node_mcp17t[12], old_struct.node_mcp17t[12], 16);
		memcpy(meshcom_settings.node_mcp17t[13], old_struct.node_mcp17t[13], 16);
		memcpy(meshcom_settings.node_mcp17t[14], old_struct.node_mcp17t[14], 16);
		memcpy(meshcom_settings.node_mcp17t[15], old_struct.node_mcp17t[15], 16);

		// GC fields
    	meshcom_settings.node_gcb[0] = old_struct.node_gcb[0];
    	meshcom_settings.node_gcb[1] = old_struct.node_gcb[1];
    	meshcom_settings.node_gcb[2] = old_struct.node_gcb[2];
    	meshcom_settings.node_gcb[3] = old_struct.node_gcb[3];
    	meshcom_settings.node_gcb[4] = old_struct.node_gcb[4];
    	meshcom_settings.node_gcb[5] = old_struct.node_gcb[5];

    	meshcom_settings.node_country = old_struct.node_country;

		meshcom_settings.node_track_freq = old_struct.node_track_freq;
		meshcom_settings.node_preamplebits = old_struct.node_preamplebits;

		meshcom_settings.node_ss_rx_pin = old_struct.node_ss_rx_pin;
		meshcom_settings.node_ss_tx_pin = old_struct.node_ss_tx_pin;
		meshcom_settings.node_ss_baud = old_struct.node_ss_baud;

		meshcom_settings.node_postime = old_struct.node_postime;

		memcpy(meshcom_settings.node_passwd, old_struct.node_passwd, 15);

		meshcom_settings.node_sset3 = old_struct.node_sset3;

		meshcom_settings.bt_code = old_struct.bt_code;

		meshcom_settings.node_button_pin = old_struct.node_button_pin;

		memcpy(meshcom_settings.node_ownip, old_struct.node_ownip, 20);
		memcpy(meshcom_settings.node_owngw, old_struct.node_owngw, 20);
		memcpy(meshcom_settings.node_ownms, old_struct.node_ownms, 20);

		memcpy(meshcom_settings.node_name, old_struct.node_name, 20);

		memcpy(meshcom_settings.node_webpwd, old_struct.node_webpwd, 20);

		memcpy(meshcom_settings.node_ssid, old_struct.node_ssid, 33);
		memcpy(meshcom_settings.node_pwd, old_struct.node_pwd, 64);

		meshcom_settings.node_analog_pin = old_struct.node_analog_pin;
		meshcom_settings.node_analog_faktor = old_struct.node_analog_faktor;

		memcpy(meshcom_settings.node_parm, old_struct.node_parm, 50);
		memcpy(meshcom_settings.node_unit, old_struct.node_unit, 50);
		memcpy(meshcom_settings.node_format, old_struct.node_format, 50);
		memcpy(meshcom_settings.node_eqns, old_struct.node_eqns, 50);
		memcpy(meshcom_settings.node_values, old_struct.node_values, 50);

		meshcom_settings.node_parm_time = old_struct.node_parm_time;

		meshcom_settings.node_wifi_power = old_struct.node_wifi_power;
		memcpy(meshcom_settings.node_lora_call, old_struct.node_lora_call, 10);

		meshcom_settings.node_analog_alpha = old_struct.node_analog_alpha;

		save_settings();
		// delay(1000);
		// sd_nvic_SystemReset();
	}
	else
	{
		// Found new structure
		lora_file.close();
		lora_file.open(settings_name, FILE_O_READ);
		lora_file.read((uint8_t *)&meshcom_settings, sizeof(s_meshcom_settings));
		lora_file.close();

		//printf("meshcom_settings%s\n", meshcom_settings.node_call);

		// Check if it is LPWAN settings^
		if ((meshcom_settings.valid_mark_1 != 0xAA) || (meshcom_settings.valid_mark_2 != MESHCOM_DATA_MARKER))
		{
			// Data is not valid, reset to defaults
			DEBUG_MSG("FLASH", "Invalid data set, deleting and restart node");
			InternalFS.format();
			delay(1000);
			sd_nvic_SystemReset();
		}
		log_settings();
		init_flash_done = true;
	}
}

/**
 * @brief Save changed settings if required
 *
 * @return boolean
 * 			result of saving
 */
boolean save_settings(void)
{
	bool result = true;
	// Read saved content
	// printf("settings_name:%s\n", settings_name);

	lora_file.open(settings_name, FILE_O_READ);
	if (!lora_file)
	{
		DEBUG_MSG("FLASH", "File doesn't exist, force format");
		delay(100);
		flash_reset();
		lora_file.open(settings_name, FILE_O_READ);
	}
	lora_file.read((uint8_t *)&g_flash_content, sizeof(s_meshcom_settings));
	lora_file.close();
	
	DEBUG_MSG("FLASH", "close");
	
	//printf("g_flash_content%s\n", g_flash_content.node_call);

	if (memcmp((void *)&g_flash_content, (void *)&meshcom_settings, sizeof(s_meshcom_settings)) != 0)
	{
		DEBUG_MSG("FLASH", "Flash content changed, writing new data");
		delay(100);

		InternalFS.remove(settings_name);

		if (lora_file.open(settings_name, FILE_O_WRITE))
		{
			lora_file.write((uint8_t *)&meshcom_settings, sizeof(s_meshcom_settings));
			lora_file.flush();
		}
		else
		{
			result = false;
		}
		lora_file.close();
	}
	
	log_settings();

	//Test only Serial.println("\nflash save...");

	return result;
}

/**
 * @brief Reset content of the filesystem
 *
 */
void flash_reset(void)
{
	InternalFS.format();
	if (lora_file.open(settings_name, FILE_O_WRITE))
	{
		s_meshcom_settings default_settings;
		lora_file.write((uint8_t *)&default_settings, sizeof(s_meshcom_settings));
		lora_file.flush();
		lora_file.close();
	}
}

/**
 * @brief Printout of all settings
 *
 */
void ble_log_settings(void)
{
	g_ble_uart.printf("Saved settings:");
	delay(50);
	g_ble_uart.printf("Marks: %02X %02X", meshcom_settings.valid_mark_1, meshcom_settings.valid_mark_2);
	delay(50);
}

#endif
