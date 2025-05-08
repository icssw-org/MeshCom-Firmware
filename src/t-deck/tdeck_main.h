/**
 * 
 */

#ifndef _TDECK_MAIN_H_
#define _TDECK_MAIN_H_

#include <AceButton.h>
using namespace ace_button;

extern AceButton button;

void initTDeck();

void tdeck_update_batt_label(float batt, float proz);
void tdeck_update_time_label();

#endif // _TDECK_MAIN_H_