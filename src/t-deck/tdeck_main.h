/**
 * 
 */

#ifndef _TDECK_MAIN_H_
#define _TDECK_MAIN_H_

#include <AceButton.h>
using namespace ace_button;

extern AceButton button;

void initTDeck();
void startAudio();
void addMessage(const char*);

void tdeck_update_batt_label(float batt, int proz);
void tdeck_update_time_label();
void tdeck_update_header_standby();
void tdeck_addMessage(bool bSuccess);
void tdeck_clear_text_ta();

#endif // _TDECK_MAIN_H_