#include <configuration.h>

void esp32setup();

void esp32loop();

int checkRX(void);

unsigned long analog_oversample_timer = 0;
float ADCslope = 1.0;
float ADCoffset = 0;
// ADC-filtering variables
float ADCalpha = 0.1;
float ADCexp1 = 0.0;
float ADCexp1pre = 0.0;
float ADCexp12 = 0.0;
float ADCexp12pre = 0.0;
float ADCexp2 = 0.0;

// same set of variables for BATT
float BATTalpha = 0.1;
float BATTexp1 = 0.0;
float BATTexp1pre = 0.0;
float BATTexp12 = 0.0;
float BATexp12pre = 0.0;
float BATexp2 = 0.0;