#ifndef _TIME_FUNCTIONS_H_
#define _TIME_FUNCTIONS_H_

void setupAceTime();
void loopAceTime();

void getCurrentTime();
void setCurrentTime(char strDateTime[20]);
void setCurrentTime(int16_t Year, int16_t Month, int16_t Day, int16_t Hour, int16_t Minute, int16_t Second);

#endif
