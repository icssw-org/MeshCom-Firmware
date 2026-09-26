#include <Arduino.h>

#if defined(BOARD_T5_EPAPER) or defined(BOARD_T_ECHO)
#else
  #include <SPI.h>
#endif

#include <configuration.h>
#include <capture_functions.h>

#ifdef ESP32
  #include <esp32/esp32_main.h>
#endif

#if defined(BOARD_RAK4630)
  #include <nrf52/nrf52_main.h>
  // Stack of the task that runs nrf52loop(), in 32-bit words: 8 kB, twice the
  // core's LOOP_STACK_SZ. Heap-allocated by xTaskCreate() (malloc-backed).
  #define NRF52_LOOP_STACK_WORDS (2048)

  // What the core's loop() would run for this board; see ETH-03 in setup().
  static void nrf52LoopBody()
  {
    nrf52loop();
    captureDrain();
  }
#endif

#if defined (BOARD_T_DECK_PRO)
  #include <t-deck-pro/tdeck_pro.h>
#endif

void setup()
{
  #if defined(BOARD_T5_EPAPER)
  if (psramInit()) {
      Serial.println("\nThe PSRAM is correctly initialized");
  } else {
      Serial.println("\nPSRAM does not work");
  }
  #endif

  #if defined(BOARD_T_ETH_ELITE)
    SPI.begin(RADIO_SCK, RADIO_MISO, RADIO_MOSI);
    pinMode(RADIO_CS_PIN, OUTPUT);
    digitalWrite(RADIO_CS_PIN, HIGH);
    
  #elif !defined(BOARD_HELTEC_T114) && !defined(BOARD_T_ECHO) && !defined(BOARD_T_DECK) && !defined(BOARD_T_DECK_PLUS) && !defined(BOARD_T5_EPAPER) && !defined(BOARD_T_DECK_PRO) && !defined(BOARD_TBEAM_1W) && !defined(BOARD_T3S3_V13)
    SPI.begin();
  #endif
    #if defined(BOARD_RAK4630)
      nrf52setup();

      // ETH-03 (docs/BACKLOG.md 3.8at): the Adafruit core runs setup()/loop()
      // in a task with a fixed 4 kB stack (LOOP_STACK_SZ, cores/nRF5/main.cpp,
      // not overridable by build flag). R2-04 made struct aprsMessage ~590 B of
      // inline text, and the EXTUDP inbound path getExtern() -> sendMessage()
      // now carries two of them plus a 381 B frame buffer on that stack: one
      // inbound datagram overran it and reset the node (RESETREAS=0x4, the
      // same signature as N-22). Every earlier fix of this class moved single
      // buffers into BSS (N-22, TM-43); this moves the loop itself into a task
      // with twice the stack, via the core's own Scheduler API, and parks the
      // core's loop task for good. The body is the same as loop() below:
      // nrf52loop() plus captureDrain() (the raw-frame capture ring is only
      // drained from loop context). If the task cannot be created (heap), the
      // core's 4 kB loop keeps running and the boot log says so.
      if(Scheduler.startLoop(nrf52LoopBody, NRF52_LOOP_STACK_WORDS, TASK_PRIO_LOW, "mcloop"))
      {
        Serial.printf("[BOOT];loopstack;words;%u\n", (unsigned)NRF52_LOOP_STACK_WORDS);
        suspendLoop();   // suspends the calling task: setup() never returns
      }
      else
      {
        Serial.println("[BOOT];loopstack;FAILED;core 4 kB loop task stays");
      }
    #endif

    //#if defined (BOARD_T_DECK_PRO)
    //  initTDeck_pro();
    //#elif ESP32
    #if defined ESP32
      esp32setup();
    #endif
  }

void loop()
{

  #if defined(BOARD_T5_EPAPER)
    esp32loop();
  #endif  

  #if defined(BOARD_RAK4630)
    nrf52loop();
  #endif

  //#if defined (BOARD_T_DECK_PRO)
  //  loopTDeck_pro();
  //#elif ESP32
  #if defined ESP32
    esp32loop();
  #endif

  // Rohframe-Mitschnitt ausgeben. Die Erfassung sitzt im Radio-Callback bzw.
  // zwischen CAD und startTransmit() und darf dort nicht drucken; hier ist
  // Loop-Kontext. Ein Frame je Durchlauf, siehe capture_functions.h.
  captureDrain();
}