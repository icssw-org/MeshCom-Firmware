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

        memcpy(g_meshcom_settings.node_call, old_struct.node_call, 10);
        memcpy(g_meshcom_settings.node_short, old_struct.node_short, 5);

		g_meshcom_settings.node_lat = old_struct.node_lat;
		g_meshcom_settings.node_lat_c = old_struct.node_lat_c;
		g_meshcom_settings.node_lon = old_struct.node_lon;
		g_meshcom_settings.node_lon_c = old_struct.node_lon_c;
		g_meshcom_settings.node_alt = old_struct.node_alt;
		g_meshcom_settings.node_symid = old_struct.node_symid;
		g_meshcom_settings.node_symcd = old_struct.node_symcd;

		g_meshcom_settings.auto_join = old_struct.auto_join;
		memcpy(g_meshcom_settings.node_device_eui, old_struct.node_device_eui, 8);

        /*KBC
		g_meshcom_settings.adr_enabled = old_struct.adr_enabled;
		g_meshcom_settings.app_port = old_struct.app_port;
		g_meshcom_settings.auto_join = old_struct.auto_join;
		g_meshcom_settings.confirmed_msg_enabled = old_struct.confirmed_msg_enabled;
		g_meshcom_settings.data_rate = old_struct.data_rate;
		g_meshcom_settings.duty_cycle_enabled = old_struct.duty_cycle_enabled;
		g_meshcom_settings.join_trials = old_struct.join_trials;
		g_meshcom_settings.lora_class = old_struct.lora_class;
		g_meshcom_settings.lora_region = old_struct.lora_region;
		memcpy(g_meshcom_settings.node_app_eui, old_struct.node_app_eui, 8);
		memcpy(g_meshcom_settings.node_app_key, old_struct.node_app_key, 16);
		memcpy(g_meshcom_settings.node_apps_key, old_struct.node_apps_key, 16);
		g_meshcom_settings.node_dev_addr = old_struct.node_dev_addr;
		memcpy(g_meshcom_settings.node_device_eui, old_struct.node_device_eui, 8);
		memcpy(g_meshcom_settings.node_nws_key, old_struct.node_nws_key, 16);
		g_meshcom_settings.otaa_enabled = old_struct.otaa_enabled;
		g_meshcom_settings.public_network = old_struct.public_network;
		g_meshcom_settings.send_repeat_time = old_struct.send_repeat_time;
		g_meshcom_settings.subband_channels = old_struct.subband_channels;
		g_meshcom_settings.tx_power = old_struct.tx_power;
        */
		save_settings();
		// delay(1000);
		// sd_nvic_SystemReset();
	}
	else
	{
		// Found new structure
		lora_file.close();
		lora_file.open(settings_name, FILE_O_READ);
		lora_file.read((uint8_t *)&g_meshcom_settings, sizeof(s_meshcom_settings));
		lora_file.close();

		//printf("g_meshcom_settings%s\n", g_meshcom_settings.node_call);

		// Check if it is LPWAN settings^
		if ((g_meshcom_settings.valid_mark_1 != 0xAA) || (g_meshcom_settings.valid_mark_2 != MESHCOM_DATA_MARKER))
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

	if (memcmp((void *)&g_flash_content, (void *)&g_meshcom_settings, sizeof(s_meshcom_settings)) != 0)
	{
		DEBUG_MSG("FLASH", "Flash content changed, writing new data");
		delay(100);

		InternalFS.remove(settings_name);

		if (lora_file.open(settings_name, FILE_O_WRITE))
		{
			lora_file.write((uint8_t *)&g_meshcom_settings, sizeof(s_meshcom_settings));
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
	g_ble_uart.printf("Marks: %02X %02X", g_meshcom_settings.valid_mark_1, g_meshcom_settings.valid_mark_2);
	delay(50);

    /*KBC
    g_ble_uart.printf("Dev EUI %02X%02X%02X%02X%02X%02X%02X%02X", g_meshcom_settings.node_device_eui[0], g_meshcom_settings.node_device_eui[1],
					  g_meshcom_settings.node_device_eui[2], g_meshcom_settings.node_device_eui[3],
					  g_meshcom_settings.node_device_eui[4], g_meshcom_settings.node_device_eui[5],
					  g_meshcom_settings.node_device_eui[6], g_meshcom_settings.node_device_eui[7]);
	delay(50);
	g_ble_uart.printf("App EUI %02X%02X%02X%02X%02X%02X%02X%02X", g_meshcom_settings.node_app_eui[0], g_meshcom_settings.node_app_eui[1],
					  g_meshcom_settings.node_app_eui[2], g_meshcom_settings.node_app_eui[3],
					  g_meshcom_settings.node_app_eui[4], g_meshcom_settings.node_app_eui[5],
					  g_meshcom_settings.node_app_eui[6], g_meshcom_settings.node_app_eui[7]);
	delay(50);
	g_ble_uart.printf("App Key %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",
					  g_meshcom_settings.node_app_key[0], g_meshcom_settings.node_app_key[1],
					  g_meshcom_settings.node_app_key[2], g_meshcom_settings.node_app_key[3],
					  g_meshcom_settings.node_app_key[4], g_meshcom_settings.node_app_key[5],
					  g_meshcom_settings.node_app_key[6], g_meshcom_settings.node_app_key[7],
					  g_meshcom_settings.node_app_key[8], g_meshcom_settings.node_app_key[9],
					  g_meshcom_settings.node_app_key[10], g_meshcom_settings.node_app_key[11],
					  g_meshcom_settings.node_app_key[12], g_meshcom_settings.node_app_key[13],
					  g_meshcom_settings.node_app_key[14], g_meshcom_settings.node_app_key[15]);
	delay(50);
	g_ble_uart.printf("Dev Addr %08lX", g_meshcom_settings.node_dev_addr);
	delay(50);
	g_ble_uart.printf("NWS Key %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",
					  g_meshcom_settings.node_nws_key[0], g_meshcom_settings.node_nws_key[1],
					  g_meshcom_settings.node_nws_key[2], g_meshcom_settings.node_nws_key[3],
					  g_meshcom_settings.node_nws_key[4], g_meshcom_settings.node_nws_key[5],
					  g_meshcom_settings.node_nws_key[6], g_meshcom_settings.node_nws_key[7],
					  g_meshcom_settings.node_nws_key[8], g_meshcom_settings.node_nws_key[9],
					  g_meshcom_settings.node_nws_key[10], g_meshcom_settings.node_nws_key[11],
					  g_meshcom_settings.node_nws_key[12], g_meshcom_settings.node_nws_key[13],
					  g_meshcom_settings.node_nws_key[14], g_meshcom_settings.node_nws_key[15]);
	delay(50);
	g_ble_uart.printf("Apps Key %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",
					  g_meshcom_settings.node_apps_key[0], g_meshcom_settings.node_apps_key[1],
					  g_meshcom_settings.node_apps_key[2], g_meshcom_settings.node_apps_key[3],
					  g_meshcom_settings.node_apps_key[4], g_meshcom_settings.node_apps_key[5],
					  g_meshcom_settings.node_apps_key[6], g_meshcom_settings.node_apps_key[7],
					  g_meshcom_settings.node_apps_key[8], g_meshcom_settings.node_apps_key[9],
					  g_meshcom_settings.node_apps_key[10], g_meshcom_settings.node_apps_key[11],
					  g_meshcom_settings.node_apps_key[12], g_meshcom_settings.node_apps_key[13],
					  g_meshcom_settings.node_apps_key[14], g_meshcom_settings.node_apps_key[15]);
	delay(50);
	g_ble_uart.printf("OTAA %s", g_meshcom_settings.otaa_enabled ? "enabled" : "disabled");
	delay(50);
	g_ble_uart.printf("ADR %s", g_meshcom_settings.adr_enabled ? "enabled" : "disabled");
	delay(50);
	g_ble_uart.printf("%s Network", g_meshcom_settings.public_network ? "Public" : "Private");
	delay(50);
	g_ble_uart.printf("Dutycycle %s", g_meshcom_settings.duty_cycle_enabled ? "enabled" : "disabled");
	delay(50);
	g_ble_uart.printf("Repeat time %ld", g_meshcom_settings.send_repeat_time);
	delay(50);
	g_ble_uart.printf("Join trials %d", g_meshcom_settings.join_trials);
	delay(50);
	g_ble_uart.printf("TX Power %d", g_meshcom_settings.tx_power);
	delay(50);
	g_ble_uart.printf("DR %d", g_meshcom_settings.data_rate);
	delay(50);
	g_ble_uart.printf("Class %d", g_meshcom_settings.lora_class);
	delay(50);
	g_ble_uart.printf("Subband %d", g_meshcom_settings.subband_channels);
	delay(50);
	g_ble_uart.printf("Auto join %s", g_meshcom_settings.auto_join ? "enabled" : "disabled");
	delay(50);
	g_ble_uart.printf("Fport %d", g_meshcom_settings.app_port);
	delay(50);
	g_ble_uart.printf("%s Message", g_meshcom_settings.confirmed_msg_enabled ? "Confirmed" : "Unconfirmed");
	delay(50);
	g_ble_uart.printf("Region %s", region_names[g_meshcom_settings.lora_region]);
	delay(50);
	g_ble_uart.printf("Mode %s", g_meshcom_settings.lorawan_enable ? "LPWAN" : "P2P");
	delay(50);
	g_ble_uart.printf("P2P frequency %ld", g_meshcom_settings.p2p_frequency);
	delay(50);
	g_ble_uart.printf("P2P TX Power %d", g_meshcom_settings.p2p_tx_power);
	delay(50);
	g_ble_uart.printf("P2P BW %d", g_meshcom_settings.p2p_bandwidth);
	delay(50);
	g_ble_uart.printf("P2P SF %d", g_meshcom_settings.p2p_sf);
	delay(50);
	g_ble_uart.printf("P2P CR %d", g_meshcom_settings.p2p_cr);
	delay(50);
	g_ble_uart.printf("P2P Preamble length %d", g_meshcom_settings.p2p_preamble_len);
	delay(50);
	g_ble_uart.printf("P2P Symbol Timeout %d", g_meshcom_settings.p2p_symbol_timeout);
	delay(50);
    */
}

#endif
