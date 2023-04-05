#ifndef _APRS_FUNCTIONS_H_
#define _APRS_FUNCTIONS_H_

#include <Arduino.h>
#include <configuration.h>
#include <structures.h>

void initAPRS(struct aprsMessage &aprsMessage);
uint8_t decodeAPRS(uint8_t RcvBuffer[UDP_TX_BUF_SIZE], uint8_t size, struct aprsMessage &aprsMessage);
uint8_t encodeAPRS(uint8_t RcvBuffer[UDP_TX_BUF_SIZE], struct aprsMessage &aprsMessage, unsigned int GW_ID);
int encodeStartAPRS(uint8_t msg_buffer[MAX_MSG_LEN_PHONE], struct aprsMessage &aprsmsg);
int encodePayloadAPRS(uint8_t msg_buffer[MAX_MSG_LEN_PHONE], struct aprsMessage &aprsmsg);

#endif