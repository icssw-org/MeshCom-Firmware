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

	char node_ossid[40] = {0};
	char node_opwd[40] = {0};

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

	float node_utcoff = 1;

	// BME680
	float node_gas_res = 0;

	// CMCU-811
	float node_co2 = 0;

	// MCP23017
	int node_mcp17io = 0;
	int node_mcp17out = 0;
	int node_mcp17in = 0;
	char node_mcp17t[16][16] = {0};

	// GC Fields
	int node_gcb[6] = {0};

	// internatioal fields
	int node_country = 0;	// 0...EU  1...UK, 2...ON, 3...EA, 4...OM, 8...EU8, 10...US, ..... 18...868, 19...915, 20...MAN

	float node_track_freq = 0;
	int node_preamplebits = 0;

	int node_ss_rx_pin = 0;
	int node_ss_tx_pin = 0;
	int node_ss_baud = 0;

	int node_postime = 0;

	char node_passwd[15] = {0};

	int node_sset3 = 0x0000;

	int bt_code = 0;

	int node_button_pin = 0;

	char node_ownip[20] = {0};
	char node_owngw[20] = {0};
	char node_ownms[20] = {0};

	char node_name[20] = {0};

	char node_webpwd[20] = {0};

	char node_ssid[33] = {0};
	char node_pwd[64] = {0};

	int node_analog_pin = 99;
	float node_analog_faktor = 1.0;

	char node_parm[50] = {0};
	char node_unit[50] = {0};
	char node_format[50] = {0};
	char node_eqns[50] = {0};
	char node_values[50] = {0};
	int node_parm_time = 15;

	float node_specstart = 432.0;
	float node_specend = 434.0;
	float node_specstep = 0.025;
	int node_specsamples = 2048;

	float node_analog_batt_faktor = 0.0;
	
	int node_wifi_power = 60;

	char node_lora_call[10] = {0x00};

	// nicht im Flash
	int node_press_alt = 0;
	float node_press_asl = 0;
	float node_vbus = 0;
	float node_vshunt = 0;
	float node_vcurrent = 0;
	float node_vpower = 0;

	char node_ip[40] = {0};
	char node_dns[40] = {0};
	char node_gw[40] = {0};
	char node_subnet[40] = {0};
	bool node_hasIPaddress = false;
	unsigned long node_last_upd_timer = 0;

	int max_hop_text = 0;
	int max_hop_pos = 0;

	char node_update[20] = {0};
};

extern s_meshcom_settings meshcom_settings;

void save_settings(void);
// Get LoRa parameter
void init_flash(void);

#endif