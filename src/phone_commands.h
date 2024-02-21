#ifndef _PHONE_COMMANDS_H_
#define _PHONE_COMMANDS_H_

void readPhoneCommand(uint8_t conf_data[MAX_MSG_LEN_PHONE]);
void sendToPhone();
void sendComToPhone();
void sendConfigToPhone ();

#endif