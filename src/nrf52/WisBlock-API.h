/**
 * @file SX126x-API.h
 * @author Bernd Giesecke (bernd.giesecke@rakwireless.com)
 * @brief Includes and global declarations
 * @version 0.1
 * @date 2021-01-10
 *
 * @copyright Copyright (c) 2021
 *
 */
#ifndef SX126x_API_H
#define SX126x_API_H

#ifndef NO_BLE_LED
// Set usage of BLE connection LED (blue). Comment the line to enable LED
//#define NO_BLE_LED 1
#endif

// Debug output set to 0 to disable app debug output
#ifndef API_DEBUG
#define API_DEBUG 0
#endif

#if API_DEBUG > 0
#if defined NRF52_SERIES
#define API_LOG(tag, ...)         \
	do                            \
	{                             \
		if (tag)                  \
			PRINTF("[%s] ", tag); \
		PRINTF(__VA_ARGS__);      \
		PRINTF("\n");             \
	} while (0)
#endif
#if defined ARDUINO_ARCH_RP2040 || defined ESP32
#define API_LOG(tag, ...)                \
	do                                   \
	{                                    \
		if (tag)                         \
			Serial.printf("[%s] ", tag); \
		Serial.printf(__VA_ARGS__);      \
		Serial.printf("\n");             \
	} while (0)
#endif
#else
#define API_LOG(...)
#endif

#include <Arduino.h>
#include <LoRaWan-Arduino.h>
//#include "wisblock_cayenne.h"

#ifdef NRF52_SERIES
#include <nrf_nvic.h>
#endif
#ifdef ARDUINO_ARCH_RP2040
#include <mbed.h>
#include <rtos.h>
#include <multicore.h>
#include <time.h>

using namespace rtos;
using namespace mbed;
using namespace std::chrono_literals;
using namespace std::chrono;

#endif

#ifdef ESP32
#include <Preferences.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <esp_system.h>
#include <nvs.h>
#include <nvs_flash.h>
#endif

// Main loop stuff
#if defined NRF52_SERIES
void periodic_wakeup(TimerHandle_t unused);
extern SemaphoreHandle_t g_task_sem;
// extern SoftwareTimer g_task_wakeup_timer;
extern TimerHandle_t g_task_wakeup_timer;
#endif
#ifdef ARDUINO_ARCH_RP2040
void periodic_wakeup(void);
extern osThreadId loop_thread;
extern TimerEvent_t g_task_wakeup_timer;
#endif
#if defined ESP32
void periodic_wakeup(void);
extern SemaphoreHandle_t g_task_sem;
// extern SoftwareTimer g_task_wakeup_timer;
extern Ticker g_task_wakeup_timer;
#endif
extern volatile uint16_t g_task_event_type;

/** Wake up events, more events can be defined in app.h */
#define NO_EVENT 0
#define STATUS 0b0000000000000001
#define N_STATUS 0b1111111111111110
#define BLE_CONFIG 0b0000000000000010
#define N_BLE_CONFIG 0b1111111111111101
#define BLE_DATA 0b0000000000000100
#define N_BLE_DATA 0b1111111111111011
#define LORA_DATA 0b0000000000001000
#define N_LORA_DATA 0b1111111111110111
#define LORA_TX_FIN 0b0000000000010000
#define N_LORA_TX_FIN 0b1111111111101111
#define AT_CMD 0b0000000000100000
#define N_AT_CMD 0b1111111111011111
#define LORA_JOIN_FIN 0b0000000001000000
#define N_LORA_JOIN_FIN 0b1111111110111111

/** Wake signal for RAK11310 */
#define SIGNAL_WAKE 0x001

#if defined NRF52_SERIES
// BLE
#include <bluefruit.h>
void init_ble(void);
void init_ble_name(void);
BLEService init_settings_characteristic(void);
void restart_advertising(uint16_t timeout);
extern BLECharacteristic g_lora_data;
extern BLEUart g_ble_uart;
extern bool g_ble_uart_is_connected;
extern bool g_enable_ble;
#endif
extern char g_ble_dev_name[];

#if defined ESP32
// BLE
#include <NimBLEUtils.h>
#include <NimBLEServer.h>
#include <NimBLEDevice.h>
#include <NimBLEAdvertising.h>
#include <ArduinoJson.h>

void init_ble(void);
BLEService init_settings_characteristic(void);
void restart_advertising(uint16_t timeout);
extern BLECharacteristic *lora_characteristic;
extern BLECharacteristic *uart_tx_characteristic;
extern bool g_ble_uart_is_connected;
extern bool g_enable_ble;

// WiFi
void init_wifi(void);
void get_wifi_prefs(void);
extern bool g_has_credentials;
extern bool g_conn_status_changed;
extern volatile bool g_wifi_connected;
extern char g_ap_name[];
extern String g_ssid_prim;
extern String g_ssid_sec;
extern String g_pw_prim;
extern String g_pw_sec;

#endif

// LoRa
int8_t init_lora(void);
int8_t init_meshcom(void);
bool send_p2p_packet(uint8_t *data, uint8_t size);
//lmh_error_status send_lora_packet(uint8_t *data, uint8_t size, uint8_t fport = 0);
extern bool g_lpwan_has_joined;
extern bool g_rx_fin_result;
extern bool g_join_result;
extern uint32_t otaaDevAddr;

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

	char node_ssid[40] = {0};
	char node_pwd[40] = {0};

	// Default is off
	uint32_t send_repeat_time = 0;
	
	bool auto_join = false;

	int node_hamnet_only = 0;

	int node_sset = 0;

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

	int node_sset2 = 0;
	int node_owgpio = 16;

	float node_temp2 = 0;

	float node_utcoff = 0;

	// BME680
	float node_gas_res = 0;

	// CMCU-811
	float node_co2 = 0;

	// MCP23017
	int node_mcp17io = 0;
	char node_mcp17t[16][16] = {0};
	int node_mcp17out = 0;
	int node_mcp17in = 0;

	// GC Fields
	int node_gcb[6] = {0};

	// internatioal fields
	int node_country = 0;	// 0...EU  1...UK, 2...IT, 3...US, ..... 18...868, 19...915

	float node_track_freq = 0;
	int node_preamplebits = 0;

	int node_ss_rx_pin = 0;
	int node_ss_tx_pin = 0;
	int node_ss_baud = 0;

	int node_postime = 0;

	char node_passwd[15] = {0};

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
};

extern s_meshcom_settings meshcom_settings;

extern uint8_t g_rx_lora_data[];
extern uint8_t g_rx_data_len;
extern uint8_t g_tx_lora_data[];
extern uint8_t g_tx_data_len;
extern bool g_meshcom_initialized;
extern int16_t g_last_rssi;
extern int8_t g_last_snr;
enum P2P_RX_MODE
{
	RX_MODE_NONE = 0,
	RX_MODE_RX = 1,
	RX_MODE_RX_TIMED = 2,
	RX_MODE_RX_WAIT = 3
};
extern uint8_t g_lora_p2p_rx_mode;
extern uint32_t g_lora_p2p_rx_time;

#define MESHCOM_COMPAT_MARKER 0x57
struct s_meshcomcompat_settings
{
	uint8_t valid_mark_1 = 0xAA;				  // Just a marker for the Flash
	uint8_t valid_mark_2 = MESHCOM_COMPAT_MARKER; // Just a marker for the Flash

	// OTAA Device EUI MSB
	uint8_t node_device_eui[8] = {0x00, 0x0D, 0x75, 0xE6, 0x56, 0x4D, 0xC1, 0xF3};

	// Flag if node joins automatically after reboot
	bool auto_join = false;

    // CALLsign
	char node_call[10] = {0x58, 0x58, 0x30, 0x58, 0x58, 0x58, 0x2D, 0x30, 0x30, 0x00};
	char node_short[6] = {0x58, 0x58, 0x58, 0x34, 0x30, 0x00};

	double node_lat = 0.0;
	char node_lat_c = {' '};
	double node_lon = 0.0;
	char node_lon_c = {' '};
	int	  node_alt = 0;
	char  node_symid = '/';
	char  node_symcd = '#';

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

	char node_ssid[40] = {0};
	char node_pwd[40] = {0};

	// Send repeat time in milliseconds: 2 * 60 * 1000 => 2 minutes
	uint32_t send_repeat_time = 0;

	int node_hamnet_only = 0;

	int node_sset = 0;

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
	int node_owgpio = 16;

	float node_temp2 = 0;

	float node_utcoff = 0;

	// BME680
	float node_gas_res = 0;

	// CMCU-811
	float node_co2 = 0;

	// MCP23017
	int node_mcp17io = 0;
	char node_mcp17t[16][16] = {0};
	int node_mcp17out = 0;
	int node_mcp17in = 0;

	// GC Fields
	int node_gcb[6] = {0};

	// internatioal fields
	int node_country = 0;	// 0...EU  1...UK, 2...IT, 3...US, ..... 18...868, 19...915

	float node_track_freq = 0;
	int node_preamplebits = 0;

	int node_ss_rx_pin = 0;
	int node_ss_tx_pin = 0;
	int node_ss_baud = 0;

	int node_postime = 0;

	char node_passwd[15] = {0};

	// nicht im Flash
	int node_press_alt = 0;
	float node_press_asl = 0;
};

// Flash
void init_flash(void);
bool save_settings(void);
void log_settings(void);
void flash_reset(void);
extern bool init_flash_done;

// Battery
void init_batt(void);
float read_batt(void);
uint8_t get_lora_batt(void);
uint8_t mv_to_percent(float mvolts);

// AT command parser
#include "at_cmd.h"
typedef struct atcmd_s
{
	const char *cmd_name;		   // CMD NAME
	const char *cmd_desc;		   // AT+CMD?
	int (*query_cmd)(void);		   // AT+CMD=?
	int (*exec_cmd)(char *str);	   // AT+CMD=value
	int (*exec_cmd_no_para)(void); // AT+CMD
} atcmd_t;
void at_serial_input(uint8_t cmd);
extern char *region_names[];
extern char g_at_query_buf[];
bool user_at_handler(char *user_cmd, uint8_t cmd_size) __attribute__((weak));
// extern atcmd_t g_user_at_cmd_list[] __attribute__((weak));
extern atcmd_t *g_user_at_cmd_list __attribute__((weak));
extern uint8_t g_user_at_cmd_num __attribute__((weak));
extern bool has_custom_at;

void at_settings(void);
#ifdef ARDUINO_ARCH_RP2040
bool init_serial_task(void);
#endif
#ifdef ESP32
#include "USB.h"
void usb_rx_cb(void);
void stop_ble_adv(void);
#endif

// API stuff
void setup_app(void);
bool init_app(void);
void app_event_handler(void);
void ble_data_handler(void) __attribute__((weak));
void lora_data_handler(void);

void api_set_version(uint16_t sw_1 = 1, uint16_t sw_2 = 0, uint16_t sw_3 = 0);
void api_read_credentials(void);
void api_set_credentials(void);
void api_reset(void);
void api_wait_wake(void);
void api_wake_loop(uint16_t reason);
uint32_t api_init_lora(void);
void api_timer_init(void);
void api_timer_start(void);
void api_timer_stop(void);
void api_timer_restart(uint32_t new_time);
void api_log_settings(void);

bool api_fs_init(void);
void api_fs_format(const char *filename);
bool api_file_open_read(const char *filename);
bool api_file_open_write(const char *filename);
void api_file_read(uint8_t *destination, uint16_t size);
void api_file_write(uint8_t *source, uint32_t size);
void api_file_remove(const char *filename);
void api_file_close(const char *filename);
extern const char settings_name[];

#ifdef NRF52_SERIES
#define api_ble_printf(...)             \
	if (g_ble_uart_is_connected)        \
	{                                   \
		g_ble_uart.printf(__VA_ARGS__); \
	}
#endif
#ifdef ESP32
#define api_ble_printf(...)                                             \
	if (g_ble_uart_is_connected)                                        \
	{                                                                   \
		char buff[255];                                                 \
		int len = sprintf(buff, __VA_ARGS__);                           \
		uart_tx_characteristic->setValue((uint8_t *)buff, (size_t)len); \
		uart_tx_characteristic->notify(true);                           \
	}
#endif
#ifdef ARDUINO_ARCH_RP2040
#define api_ble_printf(...) \
	\\ RP2040 does not have BLE
#endif

// Read/Write for WisBlock-API, module independent
enum
{
	WB_FILE_READ = 0,
	WB_FILE_WRITE = 1,
};

extern uint16_t g_sw_ver_1; // major version increase on API change / not backwards compatible
extern uint16_t g_sw_ver_2; // minor version increase on API change / backward compatible
extern uint16_t g_sw_ver_3; // patch version increase on bugfix, no affect on API

#endif // SX126x-API_H