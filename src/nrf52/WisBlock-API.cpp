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

/** Flag for the event type */
volatile uint16_t g_task_event_type = NO_EVENT;

/** Flag if BLE should be enabled */
bool g_enable_ble = false;

#endif

#if defined ARDUINO_ARCH_RP2040
/** Loop thread ID */
osThreadId loop_thread = NULL;

/** Flag for the event type */
volatile uint16_t g_task_event_type = NO_EVENT;

#endif

#ifdef ESP32
/** Semaphore used by events to wake up loop task */
SemaphoreHandle_t g_task_sem = NULL;

/** Flag for the event type */
volatile uint16_t g_task_event_type = NO_EVENT;

/** Flag if BLE should be enabled */
bool g_enable_ble = false;

#endif