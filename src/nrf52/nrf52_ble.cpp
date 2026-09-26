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
#include "ble_settings_v1.h"
#include <loop_functions.h>
#include <loop_functions_extern.h>
#include <phone_commands.h>
#include <debugconf.h>
#include <configuration.h>
#include <command_functions.h>

extern int isPhoneReady;
extern bool bAckInfo;
extern bool ble_busy_flag;
extern uint16_t swap2bytes(uint16_t value);
extern void commandAction(char *msg_text, int len, bool ble);
extern bool hasMsgFromPhone;
extern char textbuff_phone [MAX_MSG_LEN_PHONE];
extern uint8_t txt_msg_len_phone;
// `extern bool bInitDisplay;` stand hier und band an NICHTS: den Namen gibt
// es nirgendwo sonst im Baum, weder als Definition noch als zweite
// Verwendung. Uebersetzt und linkt nur, weil ihn niemand liest -- ein
// Symbol, das aussieht, als gaebe es irgendwo einen Display-Init-Zustand.
// Gefunden 2026-09-16, als carve_extern_lint.py auf diese Datei erweitert
// wurde.
extern uint8_t dmac[6];
extern bool config_to_phone_prepare;
extern bool conffin_sent;

// FreeRTOS queue holding raw BLE payloads handed from the BLE stack's
// RX callback (adafruit_ble_task context) to the Main Loop task, which
// drains it in nrf52loop() (src/nrf52/nrf52_main.cpp). readPhoneCommand()/
// commandAction()/save_settings() must not run inline in the BLE callback
// concurrently with the Main Loop; this mirrors the ESP32 bleQueue design
// (CONC-14). Non-static: nrf52_main.cpp references it via extern.
struct BleQueueItem {
	uint8_t data[MAX_MSG_LEN_PHONE];
	size_t length;
};

QueueHandle_t bleQueue = NULL;

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
	// Create the BLE RX queue before starting BLE so the RX callback can
	// enqueue into it as soon as connections are possible (CONC-14).
	bleQueue = xQueueCreate(5, sizeof(BleQueueItem));

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
	// RAK4631 - nRF52840 max 4dBm
	Bluefruit.setTxPower(0);

#if NO_BLE_LED > 0
	Bluefruit.autoConnLed(false);
	digitalWrite(LED_BLUE, LOW);
#endif

#ifdef _VARIANT_ISP4520_
	/** Device name for ISP4520 */
	snprintf(helper_string, sizeof(helper_string), "%s-%02X%02X%02X%02X%02X%02X", g_ble_dev_name,
			(uint8_t)(meshcom_settings.node_device_eui[2]), (uint8_t)(meshcom_settings.node_device_eui[3]),
			(uint8_t)(meshcom_settings.node_device_eui[4]), (uint8_t)(meshcom_settings.node_device_eui[5]), (uint8_t)(meshcom_settings.node_device_eui[6]), (uint8_t)(meshcom_settings.node_device_eui[7]));
#else
	/** Device name for RAK4631 */
	snprintf(helper_string, sizeof(helper_string), "%s-%02x%02x-%s", g_ble_dev_name, dmac[1], dmac[0], meshcom_settings.node_call);	// Anzeige mit callsign

	Serial.print("[INIT]...init helper_string:");
	Serial.println(helper_string);

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

	snprintf(helper_string, sizeof(helper_string), "%d.%d.%d", g_sw_ver_1, g_sw_ver_2, g_sw_ver_3);
	ble_dis.setSoftwareRev(helper_string);

	ble_dis.setHardwareRev("52840");

	// Start the DFU service first - needed for the OTA update
	ble_dfu.begin();

	ble_dis.begin();

	// Permission / Pairing
	// KBC 28.04.2025 g_ble_uart.setPermission(SECMODE_ENC_WITH_MITM, SECMODE_ENC_WITH_MITM);
	// Start the UART service
	g_ble_uart.begin();
	g_ble_uart.setRxCallback(bleuart_rx_callback);

	// Initialize the LoRa setting service
	BLEService sett_service = init_settings_characteristic();

	// Secondary Scan Response packet (optional)
  	// Since there is no room for 'Name' in Advertising packet
	Bluefruit.ScanResponse.addName();

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
 * @brief Stop advertising for a certain time
 *
 * @param timeout timeout in seconds
 */
void stop_advertising()
{
	Bluefruit.Advertising.stop();

	// disconnect
	g_ble_uart_is_connected = false;
	isPhoneReady = 0;
	bAckInfo = false;
	Bluefruit.setTxPower(0);
	DEBUG_MSG("BLE", "Disconnected");
}

/**
 * @brief  Callback when client connects
 * @param  conn_handle: Connection handle id
 */
void connect_callback(uint16_t conn_handle)
{
	(void)conn_handle;
	Bluefruit.setTxPower(4);
	DEBUG_MSG("BLE", "Connected");

	// BLE link established — app auth not yet done.
	// isPhoneReady and config_to_phone_prepare are set only after
	// successful app-layer PIN authentication via the hello message.
	isPhoneReady = 0;
	bAckInfo = false;
	config_to_phone_prepare = false;
	conffin_sent = false;
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
	bAckInfo = false;
	config_to_phone_prepare = false;
	conffin_sent = false;
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

	// Forward data from Mobile to our peripheral: enqueue for processing
	// in the Main Loop task instead of calling readPhoneCommand() here,
	// inline in the BLE stack's callback context (CONC-14).
	BleQueueItem item = {};
	item.length = g_ble_uart.read(item.data, MAX_MSG_LEN_PHONE);
	xQueueSend(bleQueue, &item, 0);  // non-blocking, drop on full queue

	// Disconnect if app-layer auth failed
	if(ble_disconnect_requested)
	{
		ble_disconnect_requested = false;
		Bluefruit.disconnect(conn_handle);
	}

}

// CONC-17: settings_rx_callback() runs in the BLE stack's task context, which
// can be preempted mid-memcpy by the FreeRTOS timer-service task that drives
// OnRxDone (priority 2, see C-01/09-concurrency-map.md) — a torn copy of
// meshcom_settings could put a beacon on the air with a spliced callsign or
// frequency. The callback stages the incoming wire-format bytes into this
// private buffer (no shared state touched) and only sets a flag;
// applyPendingBleSettings(), called once per Main Loop iteration, converts
// the staged v1 image into a local s_meshcom_settings and only THEN copies
// it into the live meshcom_settings under a short critical section.
static s_ble_settings_v1 s_pendingBleSettingsV1;
static volatile bool s_bBleSettingsPending = false;

// Scratch buffers, file-scope static rather than on-stack: both types are
// ~2 KB, too large to put on the Main Loop task's stack repeatedly.
// s_convertedBleSettings holds the member-by-member conversion result of a
// pending write, built OUTSIDE the critical section (see
// applyPendingBleSettings()); s_bleSettingsOutBuf holds the v1 image handed
// to write()/notify() for the read direction.
// CONC-17: the BLE task may stage a NEW image into s_pendingBleSettingsV1 at
// any moment, including while applyPendingBleSettings() is converting. So the
// staging buffer is snapshotted under the lock and the conversion reads the
// snapshot, never the shared buffer -- otherwise a second settings write
// arriving mid-conversion splices two images into one and that spliced result
// gets applied AND saved. Before the v1 freeze this could not happen: the only
// read of the staging buffer was itself inside the critical section.
static s_ble_settings_v1 s_bleSettingsSnapshot;
static s_meshcom_settings s_convertedBleSettings;
static s_ble_settings_v1 s_bleSettingsOutBuf;

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
	g_lora_data.setFixedLen(sizeof(s_ble_settings_v1) + 1);
	g_lora_data.setWriteCallback(settings_rx_callback);

	g_lora_data.begin();

	// The characteristic ships the frozen v1 wire image, never
	// s_meshcom_settings directly (see ble_settings_v1.h).
	bleSettingsToV1(meshcom_settings, s_bleSettingsOutBuf);
	g_lora_data.write((void *)&s_bleSettingsOutBuf, sizeof(s_bleSettingsOutBuf));

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
	(void)conn_hdl;
	API_LOG("SETT", "Settings received");

	delay(1000);

	// Check the characteristic
	if (chr->uuid == g_lora_data.uuid)
	{
		if (!bleSettingsV1LengthOk(len))
		{
			API_LOG("SETT", "Received settings have wrong size %d", len);
			return;
		}

		const s_ble_settings_v1 *rcvdSettings = (const s_ble_settings_v1 *)data;
		if (!bleSettingsV1MarkersOk(*rcvdSettings))
		{
			API_LOG("SETT", "Received settings data do not have required markers");
			return;
		}

		// CONC-17: stage only, apply from the Main Loop (see comment above
		// s_pendingBleSettingsV1). Still just a memcpy of the raw wire
		// bytes -- the member-by-member conversion happens later, outside
		// any critical section, in applyPendingBleSettings().
		memcpy((void *)&s_pendingBleSettingsV1, data, sizeof(s_ble_settings_v1));
		s_bBleSettingsPending = true;

		// Notify task about the event
		if (g_task_sem != NULL)
		{
			g_task_event_type |= BLE_CONFIG;
			API_LOG("SETT", "Waking up loop task");
			xSemaphoreGive(g_task_sem);
		}
	}
}

/**
 * @brief Apply a settings write staged by settings_rx_callback(), if any.
 * Must be called from the Main Loop task only (CONC-17).
 */
void applyPendingBleSettings(void)
{
	if (!s_bBleSettingsPending)
		return;
	s_bBleSettingsPending = false;

	// Take a consistent snapshot of the staged image before reading it: the
	// BLE task writes s_pendingBleSettingsV1 from settings_rx_callback() and
	// can preempt this function. One bounded memcpy, the same shape and cost
	// as the apply below.
	taskENTER_CRITICAL();
	memcpy((void *)&s_bleSettingsSnapshot, (const void *)&s_pendingBleSettingsV1,
	       sizeof(s_bleSettingsSnapshot));
	taskEXIT_CRITICAL();

	// Convert OUTSIDE the critical section: bleSettingsFromV1() walks every
	// member one at a time and only ever touches these two scratch buffers,
	// never meshcom_settings and never the shared staging buffer, so it can
	// take as long as it needs without holding interrupts off.
	bleSettingsFromV1(s_bleSettingsSnapshot, s_convertedBleSettings);

	// Short, non-blocking copy — safe to run with interrupts masked, unlike
	// the delay()-based patterns fixed under N-16.
	taskENTER_CRITICAL();
	memcpy((void *)&meshcom_settings, &s_convertedBleSettings, sizeof(meshcom_settings));
	taskEXIT_CRITICAL();

	// Save new settings
	save_settings();

	// Update settings
	bleSettingsToV1(meshcom_settings, s_bleSettingsOutBuf);
	g_lora_data.write((void *)&s_bleSettingsOutBuf, sizeof(s_bleSettingsOutBuf));

	// Inform connected device about new settings
	g_lora_data.notify((void *)&s_bleSettingsOutBuf, sizeof(s_bleSettingsOutBuf));

	/*KBC
	if (meshcom_settings.resetRequest)
	{
		API_LOG("SETT", "Initiate reset");
		delay(1000);
		sd_nvic_SystemReset();
	}
	*/
}

#endif
