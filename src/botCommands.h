/*
made by

DA6SRM

function: in-firmware Bot-commands

*/
#pragma once

#include <Arduino.h>
//#include <configuration.h>
#include <aprs_structures.h>

//#ifdef BOARD_TLORA_OLV216
//#define MAX_STATIONS_FOR_AVG_SNR 30
//#define MAX_STORED_SNR_PER_STATION 40
//#else
#define MAX_STATIONS_FOR_AVG_SNR 30
#define MAX_STORED_SNR_PER_STATION 40
#define CALLSIGN_MAX_LEN 11
//#endif

bool HandleBotCommands(String &msg_payload, String source_callsign, String dest_callsign);

void AvgSNR_Update(String callsign, float snr, float rssi);
float getAverageSNR(String callsign);
float getAverageRSSI(String callsign);
uint8_t collectedSamples(String callsign);
void SendSNRMessage(String dest_call, bool requestPerGroup, String requestingCall);
void SendHeard(String dest_call);
void SendRepeat(String message);
void SendPingAnswer();

void WifiWatchdogSetup();
static void WifiWatchdogTimer_callback(void* arg);

void ReportBadPing();