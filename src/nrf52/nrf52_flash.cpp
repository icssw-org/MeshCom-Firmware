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

		meshcom_settings.node_ssid = old_struct.node_ssid;
		meshcom_settings.node_pwd = old_struct.node_pwd;

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
