This folder contains fonts sources from GFX_Root 2.0.0, from https://github.com/ZinggJM/GFX_Root
Please find attached license files relevant to content in this folder

___

To use a font, include its header file, then pass the address of its GFXFont object (from the header file) to `setFont()`.

```cpp
#include <heltec-eink-modules.h>
#include <Fonts/FreeMono9pt7b.h>

DEPG0150BNS810 display(2, 4, 5);    // Display instance, pins for DC, CS, and BUSY set.

void setup() {
    display.setFont( & FreeMono9pt7b );
}

void loop() {

}
```