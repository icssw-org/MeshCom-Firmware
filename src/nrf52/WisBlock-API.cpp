/**
 * @file main.cpp
 * @author Bernd Giesecke (bernd.giesecke@rakwireless.com)
 * @brief LoRa configuration over BLE
 * @version 0.1
 * @date 2022-01-23
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "WisBlock-API.h"

/** Flag if data flash was initialized */
//bool init_flash_done;

#if defined NRF52_SERIES
/** Semaphore used by events to wake up loop task */
SemaphoreHandle_t g_task_sem = NULL;

/** Timer to wakeup task frequently and send message */
TimerHandle_t g_task_wakeup_timer;

/** Flag for the event type */
volatile uint16_t g_task_event_type = NO_EVENT;

/** Flag if BLE should be enabled */
bool g_enable_ble = false;

/**
 * @brief Timer event that wakes up the loop task frequently
 *
 * @param unused
 */
void periodic_wakeup(TimerHandle_t unused)
{
	// Switch on LED to show we are awake
	digitalWrite(LED_GREEN, HIGH);
	api_wake_loop(STATUS);
}
#endif

#if defined ARDUINO_ARCH_RP2040
/** Loop thread ID */
osThreadId loop_thread = NULL;

/** Timer for periodic sending */
TimerEvent_t g_task_wakeup_timer;

/** Flag for the event type */
volatile uint16_t g_task_event_type = NO_EVENT;

/**
 * @brief Timer event that wakes up the loop task frequently
 *
 * @param unused
 */
void periodic_wakeup(void)
{
	// Switch on LED to show we are awake
	digitalWrite(LED_GREEN, HIGH);
	api_wake_loop(STATUS);
}
#endif

#ifdef ESP32
/** Semaphore used by events to wake up loop task */
SemaphoreHandle_t g_task_sem = NULL;

/** Timer to wakeup task frequently and send message */
Ticker g_task_wakeup_timer;

/** Flag for the event type */
volatile uint16_t g_task_event_type = NO_EVENT;

/** Flag if BLE should be enabled */
bool g_enable_ble = false;

/**
 * @brief Timer event that wakes up the loop task frequently
 *
 * @param unused
 */
void periodic_wakeup(void)
{
	// Switch on LED to show we are awake
	digitalWrite(LED_GREEN, HIGH);
	api_wake_loop(STATUS);
}
#endif