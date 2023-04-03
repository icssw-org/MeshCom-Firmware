#ifndef _APRS_FUNCTIONS_H_
#define _APRS_FUNCTIONS_H_

#include <Arduino.h>
#include <configuration.h>
#include <structures.h>

void initAPRS(struct aprsMessage &aprsMessage);
uint8_t decodeAPRS(uint8_t RcvBuffer[UDP_TX_BUF_SIZE], uint8_t size, struct aprsMessage &aprsMessage);

#endif