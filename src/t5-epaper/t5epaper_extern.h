#include <RadioLib.h>

extern SX1262 radio;
// Die EINZIGE Definition steht in src/esp32/esp32_main.cpp:465 und lautet
// `volatile int`. Diese Deklaration liess das volatile weg, was ein
// "conflicting declaration" ist, sobald beide in einer Uebersetzungseinheit
// sichtbar werden -- bisher nie, weil env:t5_epaper noch nie uebersetzt hat.
extern volatile int transmissionState;
