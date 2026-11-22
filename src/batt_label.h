// TDECK-VOLT-01 (#1312): one battery-label formatter shared by the OLED header lines
// (loop_functions.cpp, byte-identical to the former inline snprintf calls) and the T-Deck
// header (lv_obj_functions.cpp). Pure: no Arduino includes, host-testable.
#pragma once
#include <stdio.h>
#include <stddef.h>

// Formats into buf (snprintf semantics, returns its result).
//   usb      -> usbText verbatim (caller owns its USB rule and the padding)
//   showVolt -> "%*.*fV" (volts, width, decimals)
//   else     -> "%*d%%" (percent, width); clampPercent limits it to 0..100 first
// width 0 means no padding.
static inline int batt_label_format(char *buf, size_t n, double volts, int percent, bool showVolt,
                                    bool usb, int width, int decimals, const char *usbText,
                                    bool clampPercent)
{
    if (usb)
        return snprintf(buf, n, "%s", usbText);
    if (showVolt)
        return snprintf(buf, n, "%*.*fV", width, decimals, volts);
    if (clampPercent)
        percent = percent < 0 ? 0 : (percent > 100 ? 100 : percent);
    return snprintf(buf, n, "%*d%%", width, percent);
}
