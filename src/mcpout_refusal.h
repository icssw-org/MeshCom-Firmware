#ifndef MCPOUT_REFUSAL_H
#define MCPOUT_REFUSAL_H

// WEB-PIN-01: port check of the web GUI's mcpout<P> setparam. Pure C++:
// test/test_extudp_target/ runs it on the host.
//
// The web handler used to run "--setout" first and answer OKAY whatever came
// back, but --setout refuses a pin that is configured as input (its io bit is
// clear) without a word. The handler now validates the port, refuses an input
// pin with WS_RETURNCODE_FAIL and only then runs the command.
//
// port: two characters, upper case, A0..A7 / B0..B7. io_mask is
// meshcom_settings.node_mcp17io (bit set == OUTPUT).

#include <stdint.h>
#include <string.h>

enum McpoutPortCheck
{
    MCPOUT_PORT_OK = 0,    // valid, configured as output; bitmask set
    MCPOUT_PORT_INVALID,   // not A0..A7 / B0..B7
    MCPOUT_PORT_IS_INPUT   // valid, but the pin is an input
};

static inline McpoutPortCheck mcpout_check_port(const char *port, uint16_t io_mask, uint16_t &bitmask)
{
    if(port == NULL || strlen(port) != 2)
        return MCPOUT_PORT_INVALID;
    if((port[0] != 'A' && port[0] != 'B') || port[1] < '0' || port[1] > '7')
        return MCPOUT_PORT_INVALID;

    unsigned bit = (unsigned)(port[1] - '0') + (port[0] == 'B' ? 8u : 0u);
    bitmask = (uint16_t)(1u << bit);

    return (io_mask & bitmask) ? MCPOUT_PORT_OK : MCPOUT_PORT_IS_INPUT;
}

#endif
