#ifndef __HELTEC_EINK_MODULES_H__
#define __HELTEC_EINK_MODULES_H__

#include <Arduino.h>
#include "GFX_Root/GFX.h"

#include "Platforms/platforms.h"
#include "Platforms/warnings.h"

#include "Displays/class-aliases.h"                     // Rename displays, for backwards compatibility

// SPI display modules
//#include "Displays/DEPG0150BNS810/DEPG0150BNS810.h"     // Heltec 1.54" BW V2   - Red Tab
//#include "Displays/DEPG0154BNS800/DEPG0154BNS800.h"     // Heltec 1.54" BW V2   - Red Tab
//#include "Displays/GDEP015OC1/GDEP015OC1.h"             // Heltec 1.54" BW V2   - Blue Tab
//#include "Displays/QYEG0213RWS800/QYEG0213RWS800.h"     // Heltec 2.13" RED V2  - Red Tab
//#include "Displays/DEPG0290BNS75A/DEPG0290BNS75A.h"     // Heltec 2.9" BW V2    - Red Tab
#include "Displays/DEPG0290BNS800/DEPG0290BNS800.h"     // Heltec 2.9" BW V2    - Red Tab
//#include "Displays/GDE029A1/GDE029A1.h"                 // Heltec 2.9" BW V2    - Blue tab

// All-in-one "Wireless Paper" boards
#include "Displays/DEPG0213BNS800/DEPG0213BNS800.h"         
#include "Displays/LCMEN2R13EFC1/LCMEN2R13EFC1.h"

#endif