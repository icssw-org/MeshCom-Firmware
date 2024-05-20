#ifndef _APRS_FUNCTIONS_H_
#define _APRS_FUNCTIONS_H_

#include <Arduino.h>
#include <configuration.h>
#include <aprs_structures.h>

void initAPRS(struct aprsMessage &aprsMessage);
uint16_t decodeAPRS(uint8_t RcvBuffer[UDP_TX_BUF_SIZE], uint16_t size, struct aprsMessage &aprsMessage);
uint16_t encodeAPRS(uint8_t RcvBuffer[UDP_TX_BUF_SIZE], struct aprsMessage &aprsMessage);
uint16_t encodeStartAPRS(uint8_t msg_buffer[MAX_MSG_LEN_PHONE], struct aprsMessage &aprsmsg);
uint16_t encodePayloadAPRS(uint8_t msg_buffer[MAX_MSG_LEN_PHONE], struct aprsMessage &aprsmsg);
void initAPRSPOS(struct aprsPosition &aprsPosition);
uint16_t decodeAPRSPOS(String PayloadBuffer, struct aprsPosition &aprsPosition);

uint16_t encodeLoRaAPRS(uint8_t msg_buffer[UDP_TX_BUF_SIZE], char strSourceCall[10], double lat, char lat_c, double lon, char lon_c, int alt);
uint16_t encodeLoRaAPRScompressed(uint8_t msg_buffer[UDP_TX_BUF_SIZE], char cSourceCall[10], double lat, char lat_c, double lon, char lon_c, int alt);

double conv_coord_to_dec(double coord);

int CheckGroup(String callsign);
bool CheckOwnGroup(String callsign);

#endif