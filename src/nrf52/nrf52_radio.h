#pragma once

/**
 * Sets the radio to reception mode for the duration defined by RX_TIMEOUT_VALUE.
 * If the global variable bBOOSTEDGAIN is set, the maximum LNA gain is used.
 */
void startReceive();
