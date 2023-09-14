#ifndef _ESP32_FLASH_H_
#define _ESP32_FLASH_H_

#include <Arduino.h>

#define MESHCOM_DATA_MARKER 0x55

struct s_meshcom_settings
{
	uint8_t valid_mark_1 = 0xAA;				// Just a marker for the Flash
	uint8_t valid_mark_2 = MESHCOM_DATA_MARKER; // Just a marker for the Flash
												// OTAA Device EUI MSB

	// OTAA Device EUI MSB
	uint8_t node_device_eui[8] = {0x00, 0x0D, 0x75, 0xE6, 0x56, 0x4D, 0xC1, 0xF3};

	char node_call[10] = {0x58, 0x58, 0x30, 0x58, 0x58, 0x58, 0x2D, 0x30, 0x30, 0x00};
	char node_short[6] = {0x58, 0x58, 0x58, 0x34, 0x30, 0x00};

	double node_lat = 0.0;
	char node_lat_c = {' '};
	double node_lon = 0.0;
	char node_lon_c = {' '};
	int	  node_alt = 0;
	char  node_symid = '/';
	char  node_symcd = '#';

	char node_ssid[40] = {0};
	char node_pwd[40] = {0};

	int node_date_year = 0;
	int node_date_month = 0;
	int node_date_day = 0;

	int node_date_hour = 0;
	int node_date_minute = 0;
	int node_date_second = 0;
	int node_date_hundredths = 0;

	unsigned long node_age = 0;

	float node_temp = 0;
	float node_hum = 0;
	float node_press = 0;

	int node_hamnet_only = 0;

	int node_sset = 0x0004;	// bDisplayPos = true

	float node_maxv = 4.24;

	char node_extern[40] = {0};

	int node_msgid = 0;
	int node_ackid = 0;

	int node_power = 0;
	float node_freq = 0;
	float node_bw = 0;
	int node_sf = 0;
	int node_cr = 0;

	char node_atxt[40] = {0};

	int node_sset2 = 0x0000;
	int node_owgpio = 36;

	float node_temp2 = 0;

	// nicht im Flash
	int node_press_alt = 0;
	float node_press_asl = 0;

};

extern s_meshcom_settings meshcom_settings;

void save_settings(void);
// Get LoRa parameter
void init_flash(void);

#endif