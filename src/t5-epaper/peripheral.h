#ifndef __PERIPHERAL_H__
#define __PERIPHERAL_H__

#include <Arduino.h>

#define GPS_PRIORITY     (configMAX_PRIORITIES - 1)
#define LORA_PRIORITY    (configMAX_PRIORITIES - 2)
#define WS2812_PRIORITY  (configMAX_PRIORITIES - 3)
#define BATTERY_PRIORITY (configMAX_PRIORITIES - 4)
#define INFARED_PRIORITY (configMAX_PRIORITIES - 5)

// lora sx1262
#define LORA_FREQUENNCY     433.175
#define LORA_BANDWIDTH      250.0
#define LORA_OUTPUT_POWER   22 // -17 - 22 dBm
#define LORA_SPREAD_FACTOR  11
#define LORA_CODING_RATE    6

// LORA_PREAMBLE_LENGTH stand hier als LilyGo-Beispielwert 8 und ist entfernt.
// Er war nicht nur doppelt, sondern WIDERSPRUECHLICH: MeshCom fordert
// DEFAULT_PREAMPLE_LENGTH (32), definiert in variants/t5_epaper/configuration.h.
// Welcher der beiden Werte auf Sendung ging, haette allein an der
// Include-Reihenfolge gehangen -- und ein Knoten mit Praeambel 8 spricht nicht
// mit dem Mesh. src/country_profile.cpp:109 sagt dasselbe ausdruecklich:
// "LORA_PREAMBLE_LENGTH, not the literal 8".
//
// Der einzige Verwender in diesem Verzeichnis, peri_lora.cpp:123, sieht
// configuration.h ueber lora_functions.h, bekommt also den MeshCom-Wert.
//
// LORA_BANDWIDTH bleibt hier stehen, obwohl configuration.h denselben Wert
// (250.0) fuehrt: ui_port.cpp:170 liest es, und diese Datei erreicht
// configuration.h NICHT (ui.h -> t5epaper_main.h zieht es nicht herein).
// Gleicher Wert, also kein Konflikt -- nur eine Dopplung, die erst mit dem
// Include-Geflecht dieses Verzeichnisses aufloesbar ist.
#define SYNC_WORD_SX127x 0x2b  // MeshCom Sync Word!!
#define SYNC_WORD_SX127x_2BYTE 0x2b24 // MeshCom Sync Word!!

#define LORA_MODE_SEND 0
#define LORA_MODE_RECV 1

bool lora_sx1262_init(void);
void lora_set_mode(int mode);
int lora_get_mode(void);
void lora_receive_loop(void);
void lora_transmit(const char *str);
bool lora_get_recv(const char **str, int *rssi);
void lora_set_recv_flag(void);
void lora_sleep(void);
void lora_recv_suspend(void);
void lora_recv_resume(void);

// gps u-blox m10q
bool gps_init(void);
void gps_task_create(void);
uint32_t gps_get_charsProcessed(void);
void gps_task_suspend(void);
void gps_task_resume(void);
void gps_get_coord(double *lat, double *lng);
void gps_get_data(uint16_t *year, uint8_t *month, uint8_t *day);
void gps_get_time(uint8_t *hour, uint8_t *minute, uint8_t *second);
void gps_get_satellites(uint32_t *vsat);
// DRY-Kampagne D4-01/02: additiv aus dem gepflegten Zwilling
// src/t-deck-pro/peripheral.h:72,74 nachgezogen (reine TinyGPS++-
// Buchfuehrung, kein Hardwarebezug) -- siehe src/t5-epaper/peri_gps.cpp.
void gps_get_satellites(uint32_t *vsat, int *hdop);
void gps_get_fix(uint8_t *fix);
void gps_get_speed(double *speed);

#endif