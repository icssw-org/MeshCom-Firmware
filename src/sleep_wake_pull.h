#pragma once

// Deep sleep wakes on the user button (ext1, active low). The internal pull-up on that pin used to
// exist only with --button on (init_onebutton() sets INPUT_PULLUP). With --button off and no external
// pull-up the pin floats once asleep and the node wakes at once (upstream issue #1341). The wake pin
// now gets the same pull-up before ext1 is armed, with two exceptions:
//  - boards whose button has an external pull-up (init_onebutton() uses plain INPUT there);
//  - GPIO12 on classic ESP32: MTDI is a boot strapping pin, a pull-up held through deep sleep can
//    select 1.8 V flash I/O at the wake reset and the node fails to boot.
static inline bool sleepWakeNeedsPullup(int pin, bool boardHasExternalPullup, bool classicEsp32)
{
    if (pin < 0 || pin == 99)
        return false;
    if (boardHasExternalPullup)
        return false;
    if (classicEsp32 && pin == 12)
        return false;
    return true;
}
