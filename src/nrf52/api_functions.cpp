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
#include "configuration.h"

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
uint16_t g_sw_ver_2 = 29 ; // minor version increase on API change / backward compatible
#endif
#ifdef SW_VERSION_3
uint16_t g_sw_ver_3 = SW_VERSION_3; // patch version increase on bugfix, no affect on API
#else
uint16_t g_sw_ver_3 = 0; // patch version increase on bugfix, no affect on API
#endif

#ifdef USE_HELTEC_T114
uint32_t lora_t114_init(void)
{
	_hwConfig.CHIP_TYPE = SX1262;		   // Chip type, SX1261 or SX1262
	_hwConfig.PIN_LORA_RESET = 25;		   // LORA RESET
	_hwConfig.PIN_LORA_NSS = 24;		   // LORA SPI CS
	_hwConfig.PIN_LORA_SCLK = 19;		   // LORA SPI CLK
	_hwConfig.PIN_LORA_MISO = 23;		   // LORA SPI MISO
	_hwConfig.PIN_LORA_DIO_1 = 20;		   // LORA DIO_1
	_hwConfig.PIN_LORA_BUSY = 17;		   // LORA SPI BUSY
	_hwConfig.PIN_LORA_MOSI = 22;		   // LORA SPI MOSI
	_hwConfig.RADIO_TXEN = -1;			   // LORA ANTENNA TX ENABLE
	_hwConfig.RADIO_RXEN = -1;			   // LORA ANTENNA RX ENABLE (power for antenna switch)
	_hwConfig.USE_DIO2_ANT_SWITCH = true;  // LORA DIO2 controls antenna
	_hwConfig.USE_DIO3_TCXO = true;		   // LORA DIO3 controls oscillator voltage
	_hwConfig.USE_DIO3_ANT_SWITCH = false; // LORA DIO3 controls antenna
	_hwConfig.USE_RXEN_ANT_PWR = false;	   // RXEN is used as power for antenna switch
	_hwConfig.USE_LDO = false;			   // LORA usage of LDO or DCDC power regulator (defaults to DCDC)

	TimerConfig();

	SX126xIoInit();

	SPI_LORA.setPins(_hwConfig.PIN_LORA_MISO, _hwConfig.PIN_LORA_SCLK, _hwConfig.PIN_LORA_MOSI);
	delay(300);
	SPI_LORA.end();
	delay(300);
	SPI_LORA.begin();
	delay(300);
	
	// After power on the sync word should be 2414. 4434 could be possible on a restart
	// If we got something else, something is wrong.
	uint16_t readSyncWord = 0;
	SX126xReadRegisters(REG_LR_SYNCWORD, (uint8_t *)&readSyncWord, 2);

	Serial.printf("[BRD]...SyncWord = %04X\n", readSyncWord);

	// There could be a custom syncword, better test for 0xFFFF
	// if ((readSyncWord == 0x2414) || (readSyncWord == 0x4434))
	if (readSyncWord != 0xFFFF)
	{
#if defined NRF52_SERIES || defined ESP32 || defined ARDUINO_ARCH_RP2040
		if (start_lora_task())
		{
			return 0;
		}
		else
		{
			return 1;
		}
#else
		return 0;
#endif
	}
	return 1;
}
#endif

#ifdef USE_T_ECHO
uint32_t lora_t_echo_init(void)
{
	_hwConfig.CHIP_TYPE = SX1262;		   // Chip type, SX1261 or SX1262
	_hwConfig.PIN_LORA_RESET = 25;		   // LORA RESET
	_hwConfig.PIN_LORA_NSS = 24;		   // LORA SPI CS
	_hwConfig.PIN_LORA_SCLK = 19;		   // LORA SPI CLK
	_hwConfig.PIN_LORA_MISO = 23;		   // LORA SPI MISO
	_hwConfig.PIN_LORA_DIO_1 = 20;		   // LORA DIO_1
	_hwConfig.PIN_LORA_BUSY = 17;		   // LORA SPI BUSY
	_hwConfig.PIN_LORA_MOSI = 22;		   // LORA SPI MOSI
	_hwConfig.RADIO_TXEN = -1;			   // LORA ANTENNA TX ENABLE
	_hwConfig.RADIO_RXEN = -1;			   // LORA ANTENNA RX ENABLE (power for antenna switch)
	_hwConfig.USE_DIO2_ANT_SWITCH = true;  // LORA DIO2 controls antenna
	_hwConfig.USE_DIO3_TCXO = true;		   // LORA DIO3 controls oscillator voltage
	_hwConfig.USE_DIO3_ANT_SWITCH = false; // LORA DIO3 controls antenna
	_hwConfig.USE_RXEN_ANT_PWR = false;	   // RXEN is used as power for antenna switch
	_hwConfig.USE_LDO = false;			   // LORA usage of LDO or DCDC power regulator (defaults to DCDC)

	TimerConfig();

	SX126xIoInit();

	SPI_LORA.setPins(_hwConfig.PIN_LORA_MISO, _hwConfig.PIN_LORA_SCLK, _hwConfig.PIN_LORA_MOSI);
	delay(300);
	SPI_LORA.end();
	delay(300);
	SPI_LORA.begin();
	delay(300);
	
	// After power on the sync word should be 2414. 4434 could be possible on a restart
	// If we got something else, something is wrong.
	uint16_t readSyncWord = 0;
	SX126xReadRegisters(REG_LR_SYNCWORD, (uint8_t *)&readSyncWord, 2);

	Serial.printf("[BRD]...SyncWord = %04X\n", readSyncWord);

	// There could be a custom syncword, better test for 0xFFFF
	// if ((readSyncWord == 0x2414) || (readSyncWord == 0x4434))
	if (readSyncWord != 0xFFFF)
	{
#if defined NRF52_SERIES || defined ESP32 || defined ARDUINO_ARCH_RP2040
		if (start_lora_task())
		{
			return 0;
		}
		else
		{
			return 1;
		}
#else
		return 0;
#endif
	}
	return 1;
}
#endif

#if not defined(USE_HELTEC_T114) and not defined (USE_T_ECHO)
uint32_t lora_rak4630_meshcom_init(void)
{
	_hwConfig.CHIP_TYPE = SX1262;		   // Chip type, SX1261 or SX1262
	_hwConfig.PIN_LORA_RESET = 38;		   // LORA RESET
	_hwConfig.PIN_LORA_NSS = 42;		   // LORA SPI CS
	_hwConfig.PIN_LORA_SCLK = 43;		   // LORA SPI CLK
	_hwConfig.PIN_LORA_MISO = 45;		   // LORA SPI MISO
	_hwConfig.PIN_LORA_DIO_1 = 47;		   // LORA DIO_1
	_hwConfig.PIN_LORA_BUSY = 46;		   // LORA SPI BUSY
	_hwConfig.PIN_LORA_MOSI = 44;		   // LORA SPI MOSI
	_hwConfig.RADIO_TXEN = -1;			   // LORA ANTENNA TX ENABLE
	_hwConfig.RADIO_RXEN = 37;			   // LORA ANTENNA RX ENABLE (power for antenna switch)
	_hwConfig.USE_DIO2_ANT_SWITCH = true;  // LORA DIO2 controls antenna
	_hwConfig.USE_DIO3_TCXO = true;		   // LORA DIO3 controls oscillator voltage
	_hwConfig.USE_DIO3_ANT_SWITCH = false; // LORA DIO3 controls antenna
	_hwConfig.USE_RXEN_ANT_PWR = true;	   // RXEN is used as power for antenna switch
	_hwConfig.USE_LDO = false;			   // LORA usage of LDO or DCDC power regulator (defaults to DCDC)

	TimerConfig();

	SX126xIoInit();

	// After power on the sync word should be 2414. 4434 could be possible on a restart
	// If we got something else, something is wrong.
	uint16_t readSyncWord = 0;
	SX126xReadRegisters(REG_LR_SYNCWORD, (uint8_t *)&readSyncWord, 2);

	LOG_LIB("BRD", "SyncWord = %04X", readSyncWord);

	// There could be a custom syncword, better test for 0xFFFF
	// if ((readSyncWord == 0x2414) || (readSyncWord == 0x4434))
	if (readSyncWord != 0xFFFF)
	{
#if defined NRF52_SERIES || defined ESP32 || defined ARDUINO_ARCH_RP2040
		if (start_lora_task())
		{
			return 0;
		}
		else
		{
			return 1;
		}
#else
		return 0;
#endif
	}
	return 1;
}
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
#ifdef USE_HELTEC_T114
	return lora_t114_init();
#else
#ifdef BOARD_T_ECHO
	return lora_t_echo_init();
#else
	return lora_rak4630_meshcom_init();
#endif
#endif

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
	g_task_wakeup_timer = xTimerCreate(NULL, mypdMS_TO_TICKS(meshcom_settings.send_repeat_time), true, NULL, periodic_wakeup);
#endif
#ifdef ARDUINO_ARCH_RP2040
	g_task_wakeup_timer.oneShot = false;
	g_task_wakeup_timer.ReloadValue = meshcom_settings.send_repeat_time;
	TimerInit(&g_task_wakeup_timer, periodic_wakeup);
	TimerSetValue(&g_task_wakeup_timer, meshcom_settings.send_repeat_time);
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
		xTimerChangePeriodFromISR(g_task_wakeup_timer, mypdMS_TO_TICKS(meshcom_settings.send_repeat_time), &xHigherPriorityTaskWoken);
		xTimerStartFromISR(g_task_wakeup_timer, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
	else
	{
		xTimerChangePeriod(g_task_wakeup_timer, mypdMS_TO_TICKS(meshcom_settings.send_repeat_time), 0);
		xTimerStart(g_task_wakeup_timer, 0);
	}
#endif
#if defined ARDUINO_ARCH_RP2040
	TimerStart(&g_task_wakeup_timer);
#endif
#if defined ESP32
	g_task_wakeup_timer.attach_ms(meshcom_settings.send_repeat_time, periodic_wakeup);
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

	if ((meshcom_settings.send_repeat_time != 0) && (meshcom_settings.auto_join))
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
		g_task_wakeup_timer.attach_ms(meshcom_settings.send_repeat_time, periodic_wakeup);
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
}