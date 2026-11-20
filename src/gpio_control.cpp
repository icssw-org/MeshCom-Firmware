// GPIO-01 Part B: firmware glue for src/gpio_control.h. ESP32 only; on nRF52 the whole file compiles to nothing.
// Hardware is touched only on an explicit console command (or a release); never at boot.
#if defined(ESP32)
#include <Arduino.h>
#include <configuration.h>
#include <meshcom_settings.h>
#include "pin_board.h"
#include "gpio_control.h"

static GpioCtl s_gpio; // 32 bytes of RAM (sizeof GpioCtl, static_assert in the header); levels are never persisted (concept decision 1)

static void gpioSyncMaster() { s_gpio.master = meshcom_settings.node_gpioctl != 0; }

bool gpioCtlMaster()
{
    gpioSyncMaster();
    return s_gpio.master;
}

GpioRes gpioCtlCommand(int pin, GpioOp op, PinClass *why, int *readLevel)
{
    gpioSyncMaster();
    // every enabled function's pin (and the board LED) counts as claimed
    const GpioRes r = gpioDo(s_gpio, pinBoardContext(PIN_FN_COUNT), pin, op, why);
    if (!gpioNeedsHardware(r)) return r; // refused, or release of a pin we do not hold: hardware untouched
    switch (op)
    {
    case GPIO_OP_ON:
        digitalWrite(pin, HIGH);
        pinMode(pin, OUTPUT);
        break;
    case GPIO_OP_OFF:
        digitalWrite(pin, LOW);
        pinMode(pin, OUTPUT);
        break;
    case GPIO_OP_IN:
        pinMode(pin, INPUT);
        if (readLevel) *readLevel = digitalRead(pin);
        break;
    default:
        pinMode(pin, INPUT);
        break;
    }
    return GPIO_OK;
}

size_t gpioCtlSnapshot(GpioPinRow *rows, size_t max)
{
    gpioSyncMaster();
    uint64_t in = 0;
    for (int p = 0; p < 64; p++)
        if (gpioDirOf(s_gpio, p) == GPIO_DIR_IN && digitalRead(p)) in |= pinBit(p);
    return gpioSnapshot(s_gpio, pinBoardContext(PIN_FN_COUNT), in, rows, max);
}

void gpioCtlSetMaster(bool on)
{
    uint64_t was = gpioSetMaster(s_gpio, on);
    for (int p = 0; p < 64 && was; p++)
        if (was & pinBit(p))
        {
            pinMode(p, INPUT);
            was &= ~pinBit(p);
        }
}

void gpioCtlReleasePin(int pin)
{
    if (gpioReleaseForFunction(s_gpio, pin)) pinMode(pin, INPUT);
}
#endif
