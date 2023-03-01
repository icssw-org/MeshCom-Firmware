/**
 * @file api_settings.cpp
 * @author Bernd Giesecke (bernd.giesecke@rakwireless.com)
 * @brief Application settings
 * @version 0.1
 * @date 2021-09-09
 *
 * @copyright Copyright (c) 2021
 *
 */
#include "WisBlock-API.h"

#include <debugconf.h>

// Define alternate pdMS_TO_TICKS that casts uint64_t for long intervals due to limitation in nrf52840 BSP
#define mypdMS_TO_TICKS(xTimeInMs) ((TickType_t)(((uint64_t)(xTimeInMs)*configTICK_RATE_HZ) / 1000))

#ifdef SW_VERSION_1
uint16_t g_sw_ver_1 = SW_VERSION_1; // major version increase on API change / not backwards compatible
#else
uint16_t g_sw_ver_1 = 4; // major version increase on API change / not backwards compatible
#endif
#ifdef SW_VERSION_2
uint16_t g_sw_ver_2 = SW_VERSION_2; // minor version increase on API change / backward compatible
#else
uint16_t g_sw_ver_2 = 0; // minor version increase on API change / backward compatible
#endif
#ifdef SW_VERSION_3
uint16_t g_sw_ver_3 = SW_VERSION_3; // patch version increase on bugfix, no affect on API
#else
uint16_t g_sw_ver_3 = 1; // patch version increase on bugfix, no affect on API
#endif

/**
 * @brief Set application version
 *
 * @param sw_1 SW version major number
 * @param sw_2 SW version minor number
 * @param sw_3 SW version patch number
 */
void api_set_version(uint16_t sw_1, uint16_t sw_2, uint16_t sw_3)
{
	g_sw_ver_1 = sw_1;
	g_sw_ver_2 = sw_2;
	g_sw_ver_3 = sw_3;
}

/**
 * @brief Inform API that hard coded LoRaWAN credentials are used
 *
 */
void api_set_credentials(void)
{
	save_settings();
}

/**
 * @brief Force reading the LoRaWAN credentials
 *
 */
void api_read_credentials(void)
{
	init_flash();
}

/**
 * @brief System reset
 *
 */
void api_reset(void)
{
#ifdef NRF52_SERIES
	sd_nvic_SystemReset();
#endif
#ifdef ARDUINO_ARCH_RP2040
	NVIC_SystemReset();
#endif
#ifdef ESP32
	esp_restart();
#endif
}

/**
 * @brief Waits for a trigger to wake up
 *    On FreeRTOS the trigger is release of a semaphore
 *    On mbed the trigger is an OS signal
 *
 */
void api_wait_wake(void)
{
#if defined NRF52_SERIES || defined ESP32
	// Wait until semaphore is released (FreeRTOS)
	xSemaphoreTake(g_task_sem, portMAX_DELAY);
#endif
#ifdef ARDUINO_ARCH_RP2040
	bool got_signal = false;
	while (!got_signal)
	{
		osEvent event = osSignalWait(0, osWaitForever);

		if (event.status == osEventSignal)
		{
			got_signal = true;
		}
	}
#endif
}

/**
 * @brief Wake up loop task
 *
 * @param reason for wakeup
 */
void api_wake_loop(uint16_t reason)
{
	g_task_event_type |= reason;
	API_LOG("API", "Waking up loop task");

#if defined NRF52_SERIES || defined ESP32
	if (g_task_sem != NULL)
	{
		// Wake up task to send initial packet
		xSemaphoreGive(g_task_sem);
	}
#endif
#ifdef ARDUINO_ARCH_RP2040
	if (loop_thread != NULL)
	{
		osSignalSet(loop_thread, SIGNAL_WAKE);
	}
#endif
}

/**
 * @brief Initialize LoRa transceiver
 *
 * @return uint32_t result of initialization, 0 = success
 */
uint32_t api_init_lora(void)
{
#ifdef NRF52_SERIES
#ifdef _VARIANT_ISP4520_
	return lora_isp4520_init(SX1262);
#else
	return lora_rak4630_init();
#endif
#endif
#ifdef ARDUINO_ARCH_RP2040
	return lora_rak11300_init();
#endif
#ifdef ESP32
	return lora_rak13300_init();
#endif
}

/**
 * @brief Initialize the timer for frequent sending
 *
 */
void api_timer_init(void)
{
#if defined NRF52_SERIES
	g_task_wakeup_timer = xTimerCreate(NULL, mypdMS_TO_TICKS(g_meshcom_settings.send_repeat_time), true, NULL, periodic_wakeup);
#endif
#ifdef ARDUINO_ARCH_RP2040
	g_task_wakeup_timer.oneShot = false;
	g_task_wakeup_timer.ReloadValue = g_meshcom_settings.send_repeat_time;
	TimerInit(&g_task_wakeup_timer, periodic_wakeup);
	TimerSetValue(&g_task_wakeup_timer, g_meshcom_settings.send_repeat_time);
#endif
#if defined ESP32
	// Nothing to do for ESP32
#endif
}

/**
 * @brief Start the timer for frequent sending
 *
 */
void api_timer_start(void)
{
#if defined NRF52_SERIES
	// g_task_wakeup_timer.start();
	if (isInISR())
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xTimerChangePeriodFromISR(g_task_wakeup_timer, mypdMS_TO_TICKS(g_meshcom_settings.send_repeat_time), &xHigherPriorityTaskWoken);
		xTimerStartFromISR(g_task_wakeup_timer, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
	else
	{
		xTimerChangePeriod(g_task_wakeup_timer, mypdMS_TO_TICKS(g_meshcom_settings.send_repeat_time), 0);
		xTimerStart(g_task_wakeup_timer, 0);
	}
#endif
#if defined ARDUINO_ARCH_RP2040
	TimerStart(&g_task_wakeup_timer);
#endif
#if defined ESP32
	g_task_wakeup_timer.attach_ms(g_meshcom_settings.send_repeat_time, periodic_wakeup);
#endif
}

/**
 * @brief Stop the timer for frequent sending
 *
 */
void api_timer_stop(void)
{
#if defined NRF52_SERIES
	// g_task_wakeup_timer.stop();
	if (isInISR())
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xTimerStopFromISR(g_task_wakeup_timer, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
	else
	{
		xTimerStop(g_task_wakeup_timer, 0);
	}

#endif
#if defined ARDUINO_ARCH_RP2040
	TimerStop(&g_task_wakeup_timer);
#endif
#if defined ESP32
	g_task_wakeup_timer.detach();
#endif
}

/**
 * @brief Restart the time with a new time
 *
 * @param new_time
 */
void api_timer_restart(uint32_t new_time)
{
#if defined NRF52_SERIES || defined ESP32
	// g_task_wakeup_timer.stop();
	api_timer_stop();
#endif
#ifdef ARDUINO_ARCH_RP2040
	TimerStop(&g_task_wakeup_timer);
#endif
#if defined ESP32
	g_task_wakeup_timer.detach();
#endif

	if ((g_meshcom_settings.send_repeat_time != 0) && (g_meshcom_settings.auto_join))
	{
#if defined NRF52_SERIES
		// g_task_wakeup_timer.setPeriod(new_time);
		// g_task_wakeup_timer.start();

		if (isInISR())
		{
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			xTimerChangePeriodFromISR(g_task_wakeup_timer, mypdMS_TO_TICKS(new_time), &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
		else
		{
			xTimerChangePeriod(g_task_wakeup_timer, mypdMS_TO_TICKS(new_time), 0);
		}
#endif
#if defined ARDUINO_ARCH_RP2040
		TimerSetValue(&g_task_wakeup_timer, new_time);
		TimerStart(&g_task_wakeup_timer);
#endif
#if defined ESP32
		g_task_wakeup_timer.attach_ms(g_meshcom_settings.send_repeat_time, periodic_wakeup);
#endif
	}
}

/**
 * @brief Print current device status over USB
 *
 */
void api_log_settings(void)
{
	at_settings();
}

/**
 * @brief Printout of all settings
 *
 */
void log_settings(void)
{
	DEBUG_MSG("FLASH", "Saved settings:");
/*KBC
	API_LOG("FLASH", "000 Marks: %02X %02X", g_meshcom_settings.valid_mark_1, g_meshcom_settings.valid_mark_2);
	API_LOG("FLASH", "002 Dev EUI %02X%02X%02X%02X%02X%02X%02X%02X", g_meshcom_settings.node_device_eui[0], g_meshcom_settings.node_device_eui[1],
			g_meshcom_settings.node_device_eui[2], g_meshcom_settings.node_device_eui[3],
			g_meshcom_settings.node_device_eui[4], g_meshcom_settings.node_device_eui[5],
			g_meshcom_settings.node_device_eui[6], g_meshcom_settings.node_device_eui[7]);
	API_LOG("FLASH", "010 App EUI %02X%02X%02X%02X%02X%02X%02X%02X", g_meshcom_settings.node_app_eui[0], g_meshcom_settings.node_app_eui[1],
			g_meshcom_settings.node_app_eui[2], g_meshcom_settings.node_app_eui[3],
			g_meshcom_settings.node_app_eui[4], g_meshcom_settings.node_app_eui[5],
			g_meshcom_settings.node_app_eui[6], g_meshcom_settings.node_app_eui[7]);
	API_LOG("FLASH", "018 App Key %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",
			g_meshcom_settings.node_app_key[0], g_meshcom_settings.node_app_key[1],
			g_meshcom_settings.node_app_key[2], g_meshcom_settings.node_app_key[3],
			g_meshcom_settings.node_app_key[4], g_meshcom_settings.node_app_key[5],
			g_meshcom_settings.node_app_key[6], g_meshcom_settings.node_app_key[7],
			g_meshcom_settings.node_app_key[8], g_meshcom_settings.node_app_key[9],
			g_meshcom_settings.node_app_key[10], g_meshcom_settings.node_app_key[11],
			g_meshcom_settings.node_app_key[12], g_meshcom_settings.node_app_key[13],
			g_meshcom_settings.node_app_key[14], g_meshcom_settings.node_app_key[15]);
	API_LOG("FLASH", "034 Dev Addr %08lX", g_meshcom_settings.node_dev_addr);
	API_LOG("FLASH", "038 NWS Key %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",
			g_meshcom_settings.node_nws_key[0], g_meshcom_settings.node_nws_key[1],
			g_meshcom_settings.node_nws_key[2], g_meshcom_settings.node_nws_key[3],
			g_meshcom_settings.node_nws_key[4], g_meshcom_settings.node_nws_key[5],
			g_meshcom_settings.node_nws_key[6], g_meshcom_settings.node_nws_key[7],
			g_meshcom_settings.node_nws_key[8], g_meshcom_settings.node_nws_key[9],
			g_meshcom_settings.node_nws_key[10], g_meshcom_settings.node_nws_key[11],
			g_meshcom_settings.node_nws_key[12], g_meshcom_settings.node_nws_key[13],
			g_meshcom_settings.node_nws_key[14], g_meshcom_settings.node_nws_key[15]);
	API_LOG("FLASH", "054 Apps Key %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",
			g_meshcom_settings.node_apps_key[0], g_meshcom_settings.node_apps_key[1],
			g_meshcom_settings.node_apps_key[2], g_meshcom_settings.node_apps_key[3],
			g_meshcom_settings.node_apps_key[4], g_meshcom_settings.node_apps_key[5],
			g_meshcom_settings.node_apps_key[6], g_meshcom_settings.node_apps_key[7],
			g_meshcom_settings.node_apps_key[8], g_meshcom_settings.node_apps_key[9],
			g_meshcom_settings.node_apps_key[10], g_meshcom_settings.node_apps_key[11],
			g_meshcom_settings.node_apps_key[12], g_meshcom_settings.node_apps_key[13],
			g_meshcom_settings.node_apps_key[14], g_meshcom_settings.node_apps_key[15]);
	API_LOG("FLASH", "070 OTAA %s", g_meshcom_settings.otaa_enabled ? "enabled" : "disabled");
	API_LOG("FLASH", "071 ADR %s", g_meshcom_settings.adr_enabled ? "enabled" : "disabled");
	API_LOG("FLASH", "072 %s Network", g_meshcom_settings.public_network ? "Public" : "Private");
	API_LOG("FLASH", "073 Dutycycle %s", g_meshcom_settings.duty_cycle_enabled ? "enabled" : "disabled");
	API_LOG("FLASH", "074 Repeat time %ld", g_meshcom_settings.send_repeat_time);
	API_LOG("FLASH", "078 Join trials %d", g_meshcom_settings.join_trials);
	API_LOG("FLASH", "079 TX Power %d", g_meshcom_settings.tx_power);
	API_LOG("FLASH", "080 DR %d", g_meshcom_settings.data_rate);
	API_LOG("FLASH", "081 Class %d", g_meshcom_settings.lora_class);
	API_LOG("FLASH", "082 Subband %d", g_meshcom_settings.subband_channels);
	API_LOG("FLASH", "083 Auto join %s", g_meshcom_settings.auto_join ? "enabled" : "disabled");
	API_LOG("FLASH", "084 Fport %d", g_meshcom_settings.app_port);
	API_LOG("FLASH", "085 %s Message", g_meshcom_settings.confirmed_msg_enabled ? "Confirmed" : "Unconfirmed");
	API_LOG("FLASH", "086 Region %s", region_names[g_meshcom_settings.lora_region]);
	API_LOG("FLASH", "087 Mode %s", g_meshcom_settings.lorawan_enable ? "LPWAN" : "P2P");
	API_LOG("FLASH", "088 P2P frequency %ld", g_meshcom_settings.p2p_frequency);
	API_LOG("FLASH", "092 P2P TX Power %d", g_meshcom_settings.p2p_tx_power);
	API_LOG("FLASH", "093 P2P BW %d", g_meshcom_settings.p2p_bandwidth);
	API_LOG("FLASH", "094 P2P SF %d", g_meshcom_settings.p2p_sf);
	API_LOG("FLASH", "095 P2P CR %d", g_meshcom_settings.p2p_cr);
	API_LOG("FLASH", "096 P2P Preamble length %d", g_meshcom_settings.p2p_preamble_len);
	API_LOG("FLASH", "097 P2P Symbol Timeout %d", g_meshcom_settings.p2p_symbol_timeout);
    */
}
