// rm_au_none.cpp -- builds without the firmware auto update (everything but ESP32, and the FWNET_STUB
// measurement builds): the RM1 command
// "upd" answers "err unsupported". ESP32 uses src/esp32/rm_au_esp32.cpp. Contract: src/rm_au.h
#if !defined(ESP32) || defined(FWNET_STUB)

#include <string.h>

#include "rm_au.h"

void rmAuGetState(RmAuState &out)
{
    memset(&out, 0, sizeof(out));
    out.supported = false;
    out.ckMin = -1;
}

RmAuResult rmAuCheck(void)
{
    return RM_AU_UNSUPPORTED;
}

RmAuResult rmAuGo(const char *)
{
    return RM_AU_UNSUPPORTED;
}

#endif
