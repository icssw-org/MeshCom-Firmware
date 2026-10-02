// Issue 962 (docs/issue-962-deepsleep-verdict.md, Option A). See esp32_sleep.h.

#include "esp32_sleep.h"

#include <Arduino.h>
#include <esp_sleep.h>
#include <driver/gpio.h>

#include "configuration.h"
#include "printfdeb_functions.h"
#include "lora_functions.h"
#include "esp32_pmu.h"
#include "batt_functions.h"
#include "loop_breadcrumb.h"   // INS-05: loopCrumbClear() right before the sleep

// Runtime wake-button GPIO (loop_functions.cpp:186). Seeded from the
// compile-time BUTTON_PIN at boot but overridable via `--button <pin>` /
// meshcom_settings.node_button_pin (esp32_main.cpp:889-896); 99 means "no
// button configured". Arming the wake source on the compile-time macro
// instead of this variable would ignore a user's remapped pin.
extern uint8_t iButtonPin;

// `--button on|off` (node_sset bit 0x0010, loop_functions.cpp). Only when it
// is on does init_onebutton() configure iButtonPin (INPUT_PULLUP) and attach
// PressLong(); with it off the pin is unconfigured and may float LOW
// (RAK4631 WB_IO6 does), so a release wait there would only burn its 10 s
// bound on every --deepsleep -- and no long press can have brought us here.
extern bool bButtonCheck;

#if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
#include <t-deck/lv_obj_functions.h>
#endif

// Same negative guard command_functions.cpp and loop_functions.cpp use to
// declare/define the U8G2 display object: boards on a different display
// technology (e-ink WP_DISP, TFT/LVGL T-Deck family, T5-epaper, T-Connect
// Pro, BOARD_TRACKER) or with no display driven from this TU (nRF52
// BOARD_HELTEC_T114/BOARD_T_ECHO) never compile the `u8g2` symbol at all --
// keep this file's reference to it under the identical guard or those
// boards fail to link.
#if !defined(BOARD_E290) && !defined(BOARD_WIRELESS_PAPER) && !defined(BOARD_E213) && !defined(BOARD_T_DECK) && !defined(BOARD_T_DECK_PLUS) && !defined(BOARD_TRACKER) && !defined(BOARD_HELTEC_T114) && !defined(BOARD_T_ECHO) && !defined(BOARD_T5_EPAPER) && !defined(BOARD_T_DECK_PRO) && !defined(BOARD_T_CONNECT_PRO)
#include <U8g2lib.h>
extern U8G2 *u8g2;
#endif

void esp32WaitButtonRelease()
{
    // PressLong() (onebutton_functions.cpp) is attached with
    // attachLongPressStart(), so it fires 800 ms into the press and the
    // operator's finger is still on the button when --deepsleep gets here.
    // The ext1 wake armed below (ANY_LOW / ALL_LOW on that very pin) is
    // then already true at esp_deep_sleep_start(): the chip wakes at once
    // and reboots (RESET_REASON=5 DEEPSLEEP), and from the outside the node
    // "cannot be switched off" -- field report, Heltec V3 on v4.35t. On
    // v4.35s no wake source was armed at all, which is why the same press
    // used to leave the node dark until reset. Wait for the release first.
    // Bounded so that a pin stuck LOW, or a `--button` remapped to a LOW
    // GPIO, still lets a serial/BLE --deepsleep go through; a pin that is
    // HIGH on entry costs nothing. Only with `--button on` has OneButton
    // configured the pin INPUT_PULLUP (init_onebutton()), so digitalRead()
    // is meaningful there and only there.
    if (!bButtonCheck || iButtonPin == 99)
        return;
    if (digitalRead(iButtonPin) != LOW)
        return;

    uint32_t t0 = millis();
    while (digitalRead(iButtonPin) == LOW && millis() - t0 < 10000)
        delay(10);
    delay(100);   // contact bounce after release
}

void esp32EnterDeepSleep()
{
    // (a) Radio to sleep first, so it stops burning RX current while the
    // rest of this sequence runs. Guarded exactly like the loraDeepSleep()
    // declaration in lora_functions.h: only compiled where lora_functions.cpp
    // has a real `radio` object of a matching RadioLib type in scope, and
    // never for WP_DISP boards, which keep their own Platform::loraToSleep()
    // call at the --deepsleep call site in command_functions.cpp.
    #if (defined(SX127X) || defined(BOARD_E220) || defined(SX1262X) || defined(SX126X) || \
         defined(SX1262_E22) || defined(USING_SX1262) || defined(SX1268_E22) || \
         defined(SX1262_V3) || defined(SX1262_E290) || defined(SX1262_V4) || \
         defined(BOARD_T5_EPAPER)) && !defined(WP_DISP)
    loraDeepSleep();
    #endif

    // T-Beam-1W: RADIO_LDO_EN (GPIO40) feeds the SX1262 + LNA and is driven
    // HIGH once at boot (esp32_main.cpp:1294-1297) and never lowered
    // elsewhere -- radio.sleep() above puts the chip in low-power mode but
    // this LDO keeps supplying it. gpio_hold_en() survives the wake reset
    // (esp_idf gpio.h: "retain the pin state through ... system reset
    // triggered by ... Deep-sleep events"), so boot's own digitalWrite(HIGH)
    // would silently be ignored without the matching gpio_hold_dis() added
    // on the wake side in esp32_main.cpp's radio init.
    #ifdef RADIO_LDO_EN
    digitalWrite(RADIO_LDO_EN, LOW);
    gpio_hold_en((gpio_num_t) RADIO_LDO_EN);
    #endif

    // (b) Display off. u8g2 boards: real power-down via the U8g2 API.
    // Heltec V2/V3/V4 additionally gate the OLED off its Vext rail (moved
    // here from the old --deepsleep body, now covering V4 too -- V3 and V4
    // share the same Vext-gated OLED design, see batt_functions.cpp's own
    // "#if defined(BOARD_HELTEC_V3) || defined(BOARD_HELTEC_V4)" pattern for
    // the same pin).
    #if !defined(BOARD_E290) && !defined(BOARD_WIRELESS_PAPER) && !defined(BOARD_E213) && !defined(BOARD_T_DECK) && !defined(BOARD_T_DECK_PLUS) && !defined(BOARD_TRACKER) && !defined(BOARD_HELTEC_T114) && !defined(BOARD_T_ECHO) && !defined(BOARD_T5_EPAPER) && !defined(BOARD_T_DECK_PRO) && !defined(BOARD_T_CONNECT_PRO)
    if (u8g2 != NULL)
    {
        u8g2->setPowerSave(1);
    }
    #endif

    #if defined(BOARD_HELTEC) || defined(BOARD_HELTEC_V3) || defined(BOARD_HELTEC_V4) || defined(BOARD_STICK_V3)
    printlndeb(F("[INIT]...Disbling Vext for OLED power"));
    pinMode(Vext, OUTPUT);
    digitalWrite(Vext, HIGH);   // Vext OFF (active high)
    delay(50);
    #endif

    // T-Deck / T-Deck Plus: no u8g2 (LVGL/TFT), no AXP PMU. tft_off() blanks
    // the panel and already drives TDECK_TFT_BACKLIGHT (GPIO42) LOW as part
    // of its own dimming sequence; TDECK_POWERON (GPIO10) is the shared
    // LoRa/GPS/keyboard rail, driven HIGH once at boot
    // (tdeck_main.cpp:141-142) and otherwise never lowered. Both held LOW
    // like the WP e-ink path holds its NSS pin, for the same reason: an
    // unheld digital output floats once the pin's domain powers down in
    // deep sleep -- confirmed on the bench as a dim, flickering backlight
    // with the panel blanked (floating GPIO42 picking up noise on the
    // pulse-dimming driver's EN line). GPIO10 is RTC-capable (S3 RTC/LP
    // range 0-21) so gpio_hold_en() alone survives actual sleep; GPIO42 is
    // not, so it additionally needs the gpio_deep_sleep_hold_en() call
    // below (esp_idf gpio.h: "the state of the digital gpio cannot be held
    // during Deep-sleep ... unless gpio_deep_sleep_hold_en is also
    // called"). Released on the wake side in tdeck_main.cpp's initTDeck().
    #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
    tft_off();
    digitalWrite(TDECK_POWERON, LOW);
    gpio_hold_en((gpio_num_t) TDECK_POWERON);
    digitalWrite(TDECK_TFT_BACKLIGHT, LOW);
    gpio_hold_en((gpio_num_t) TDECK_TFT_BACKLIGHT);
    #endif

    // esp_idf gpio.h: gpio_hold_en() on a non-RTC ("digital") pad retains
    // its state through a reset, but NOT through the actual Deep-sleep
    // power-down, unless gpio_deep_sleep_hold_en() is also armed -- without
    // it RADIO_LDO_EN (T-Beam-1W, GPIO40) and TDECK_TFT_BACKLIGHT (GPIO42)
    // would float for the whole sleep duration, not just survive the wake
    // reset. Only takes effect during actual Deep-sleep ("When the chip is
    // in active mode, the digital gpio state can be changed freely even you
    // have called this function") so it is harmless to call unconditionally
    // on the boards that need it; released on the wake side alongside each
    // pin's own gpio_hold_dis().
    #if defined(RADIO_LDO_EN) || defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
    gpio_deep_sleep_hold_en();
    #endif

    // (c) PMU: LoRa + GPS rails off. No-op on boards without an AXP192/
    // AXP2101 PMU (T-Beam family only today).
    pmuSleepRails();

    // (d) heltec_wireless_tracker ("HWT"): VEXT_CTRL/ADC_CTRL gate its
    // TFT+GPS rail, driven HIGH at boot (esp32_main.cpp:995-999) and never
    // lowered elsewhere. The block that used to live in command_functions.cpp
    // guarded on the lowercase `vEXT_CTRL`, which no variant defines (every
    // variant spells it `VEXT_CTRL`), so it never actually fired for any
    // board including the tracker -- fixed here rather than carried forward,
    // since the tracker calls --deepsleep from its long-press
    // (onebutton_functions.cpp) and would otherwise leave its TFT/GPS
    // powered through every sleep. Scoped to BOARD_TRACKER: heltec_t114 and
    // t_echo also define VEXT_CTRL but are nRF52 and never compile this file.
    #if defined(BOARD_TRACKER) && defined(VEXT_CTRL)
    digitalWrite(VEXT_CTRL, LOW);
    #ifdef ADC_CTRL
    digitalWrite(ADC_CTRL, LOW);
    #endif
    #endif

    // (e) Battery ADC divider off. ADC_BATT_OFF() itself is self-guarded on
    // ADC_CTRL_PIN, but its declaration only exists under USE_NEW_BATT
    // (batt_functions.h) -- boards without that macro (e.g. Heltec V3) don't
    // compile the symbol at all, so the call needs the same guard here.
    #if defined(USE_NEW_BATT)
    ADC_BATT_OFF();
    #endif

    // (f) Arm the button (runtime iButtonPin, not the compile-time
    // BUTTON_PIN -- a node with `--button <pin>` / node_button_pin set would
    // otherwise wake on the wrong, unconfigured GPIO) as an ext1 wake
    // source. 99 means no button is configured on this board; arm nothing
    // rather than a bogus GPIO 99.
    //
    // esp_sleep.h: "internal pullups and pulldowns don't work when RTC
    // peripherals are shut down ... may be kept enabled using
    // esp_sleep_pd_config" -- OneButton already configures this pin
    // INPUT_PULLUP (onebutton_functions.cpp). Several boards (E22-DevKitC/
    // E22_XML/ttgo-lora32-v21 on GPIO12, LilyGo_T-Beam-1W on GPIO17) have no
    // external pull-up, so without this the pin floats once asleep and
    // either wakes immediately (ALL_LOW sees a drifting low) or never wakes
    // at all. Keeping the whole RTC_PERIPH domain powered is the
    // ESP-IDF-documented alternative to re-enabling pulls pin-by-pin, which
    // this deliberately avoids: a manual rtc_gpio_pullup_en() on GPIO12
    // (MTDI, a boot-strapping pin on modules without the VDD_SDIO efuse)
    // would risk selecting 1.8V flash I/O and failing to boot.
    //
    // esp_sleep_ext1_wakeup_mode_t is a different enum per target
    // (esp_sleep.h): classic ESP32 only has ESP_EXT1_WAKEUP_ALL_LOW /
    // _ANY_HIGH (no per-pin "any low"), S2/S3/C3 only have _ANY_LOW /
    // _ANY_HIGH (ALL_LOW there is a deprecated alias for ANY_LOW). With a
    // single-GPIO mask "all" and "any" are the same wakeup, so branching on
    // CONFIG_IDF_TARGET_ESP32 (as esp_sleep.h itself does) picks the name
    // that exists on each target without a deprecation warning on either.
    // (A second bit for GPIO0 was deliberately not added: on classic ESP32
    // ALL_LOW requires every masked pin low simultaneously, so a two-pin
    // mask would make a single button press unable to wake the node.)
    esp32WaitButtonRelease();

    if (iButtonPin != 99)
    {
        esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

        #if CONFIG_IDF_TARGET_ESP32
        esp_sleep_enable_ext1_wakeup((1ULL << iButtonPin), ESP_EXT1_WAKEUP_ALL_LOW);
        #else
        esp_sleep_enable_ext1_wakeup((1ULL << iButtonPin), ESP_EXT1_WAKEUP_ANY_LOW);
        #endif
    }

    // (g) Sleep. BOARD_RAK4630 (nRF52) never reaches this function -- see
    // the call site in command_functions.cpp.
    loopCrumbClear();   // INS-05: deliberate sleep, RTC memory survives it -- cleared last, after the
                        // bounded button wait above that could still trip the task WDT
    esp_deep_sleep_start();
}
