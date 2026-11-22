#ifndef _GW_PHONE_ROUTE_H_
#define _GW_PHONE_ROUTE_H_

// GW-GROUP-01 / GW-DM-01 (docs/plan-gateway-routing-20261121.md): what a server
// text frame may show and hand to the phone link (BLE, AppLink). Mirrors the LoRa
// rule in lora_functions.cpp: only "*" (unless --nomsgall), the own call or an own
// group. The relay to LoRa and the gateway ack are not decided here.

#include <stdint.h>
#include <string.h>
#include <Arduino.h>
#include "aprs_functions.h"

// true when a server text frame is for this node: "*" (unless --nomsgall), the own
// call, or an own group (CheckOwnGroup: a node with no group set takes every group)
static inline bool gwUdpTextForNode(const char *dest, const char *ownCall, bool noMsgToAll)
{
    if(strcmp(dest, "*") == 0)
        return !noMsgToAll;

    if(strcmp(dest, ownCall) == 0)
        return true;

    return CheckOwnGroup(String(dest));
}

// false only for a text frame (0x3A, not {SET}/{CET}) that is not for this node;
// position, HEY and the server control texts keep reaching the phone
static inline bool gwUdpPhoneWants(uint8_t msgType, const char *payload, bool forNode)
{
    if(msgType != 0x3A)
        return true;

    if(memcmp(payload, "{SET}", 5) == 0 || memcmp(payload, "{CET}", 5) == 0)
        return true;

    return forNode;
}

#endif
