/**
 * @file        esp32_audio.cpp
 * @brief       functions for handling audio playback on ESP32
 * @author      Ing. Jakob Gurnhofer (OE3GJC)
 * @author      Ralph Weich (DD5RW)
 * @license     MIT
 * @copyright   Copyright (c) 2025 ICSSW.org
 * @date        2025-12-03
 */

#if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)

#include "esp32_audio.h"
#include <configuration.h>
#include <Audio.h>
#include <SD.h>
#include <driver/i2s.h>
#include <esp32/esp32_flash.h>
#include <t-deck/tdeck_extern.h>
#include <cstring>

#include <loop_functions_extern.h>

#include "printfdeb_functions.h"

Audio audio;

// I2S-Handle
i2s_port_t i2s_num = I2S_NUM;

// Der Audio-Task ist der einzige Ort, an dem I2S geschrieben, die SD fuer
// Audio gelesen oder der I2S-Treiber ein-/ausgebaut wird. loopTask (LVGL,
// LoRa, Serial) stellt nur Auftraege in die Queue und blockiert nie auf Audio.
// Vorher lief play_cw() synchron auf loopTask (1.1 s Stillstand pro
// Nachricht) und der Dateiplayer als Task mit Prioritaet 50 (auf 24 geklemmt,
// ueber WiFi/BT). Details: docs/lvgl-research/06-audio-blocking.md.
enum audio_req_kind : uint8_t
{
    AUDIO_REQ_FILE = 1,     // Datei von SD, optional CW-Ersatz wenn sie fehlt
    AUDIO_REQ_CW,           // ein Morsezeichen
    AUDIO_REQ_CW_START,     // Startkennung
    AUDIO_REQ_MUTE          // I2S-Treiber ausbauen (flag=1) / einbauen (flag=0)
};

struct audio_request
{
    uint8_t kind;
    char    path[64];       // AUDIO_REQ_FILE
    char    cw;             // AUDIO_REQ_FILE: Ersatzzeichen (0 = keiner), AUDIO_REQ_CW: Zeichen
    int8_t  volume;
    bool    flag;           // AUDIO_REQ_MUTE: Zielzustand; sonst: Stummschaltung ignorieren (Testkommando)
};

#define AUDIO_QUEUE_DEPTH   4
#define AUDIO_TASK_PRIO     3       // ueber loopTask (1), unter jedem IDF-Systemtask
#define AUDIO_TASK_STACK    (16 * 1024)
#define AUDIO_BUS_WAIT_MS   200     // Wartezeit auf den SPI-Bus pro Zugriff

static QueueHandle_t audioQueue = NULL;
TaskHandle_t xHandle = NULL;

static void play_function(void *parameter);
static void play_cw_now(char character, int volume);
static void play_cw_start_now();

/**
 * library callback: informational / status text from the Audio library
 * (this is also where the library reports its internal errors, e.g.
 * failed connections or decode problems -- there is no separate
 * error-only callback in this library version)
 */
void audio_info(const char *info)
{
    printfdeb("[AUDIO];info;%s\n", info);
}

/**
 * library callback: end of mp3 playback
 */
void audio_eof_mp3(const char *info)
{
    printfdeb("[AUDIO];eof;%s\n", info);
}

/**
 * initializes audio: I2S pinout, queue and the one audio task
 */
void init_audio()
{
    printlndeb("[AUDIO]..initializing");

    // Remove manual I2S driver installation as the Audio library handles this internally.
    // Double initialization causes "register I2S object to platform failed" errors.

    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
    audio.setVolume(12); // Set a default volume

    if (meshcom_settings.node_mute) {
        if (bDEBUG) printlndeb("[AUDIO]..initially muted, disabling hardware");
        i2s_driver_uninstall(i2s_num);
    }

    audioQueue = xQueueCreate(AUDIO_QUEUE_DEPTH, sizeof(struct audio_request));
    if (audioQueue == NULL)
    {
        printlndeb("[AUDIO];err;queue");
        return;
    }

    if (xTaskCreatePinnedToCore(play_function, "audio", AUDIO_TASK_STACK, NULL,
                                AUDIO_TASK_PRIO, &xHandle, 1) != pdPASS)
    {
        xHandle = NULL;
        printlndeb("[AUDIO];err;task");
    }
}

// EXPERIMENT: 0 = normal, 1 = skip the SD lookup (act as if the file is missing), 2 = skip the tone.
int audio_dbg_mode = 0;

/**
 * put a request into the audio queue; never blocks the caller for long
 */
static bool audio_enqueue(const struct audio_request &req)
{
    if (audioQueue == NULL || xHandle == NULL)
        return false;
    if (xQueueSend(audioQueue, &req, pdMS_TO_TICKS(20)) != pdTRUE)
    {
        printfdeb("[AUDIO];err;queue_full;%u\n", (unsigned)req.kind);
        return false;
    }
    return true;
}

/**
 * play a file from SD in the background; if it is missing (or SD absent) and
 * cw_fallback != 0, the audio task plays that CW character instead.
 * The SD lookup happens in the audio task, not on the caller's task.
 */
bool audio_play_file_or_cw(const char *filename, int volume, char cw_fallback)
{
    if (meshcom_settings.node_mute)
    {
        if (bDEBUG)
            printlndeb("[AUDIO]..muted");
        return true;
    }

    struct audio_request req = {};
    req.kind = AUDIO_REQ_FILE;
    req.volume = (int8_t)volume;
    req.cw = cw_fallback;
    if (filename != NULL)
        snprintf(req.path, sizeof(req.path), "%s", filename);
    return audio_enqueue(req);
}

/**
 * play a supported file from SD in the background
 */
bool play_file_from_sd(const char *filename, int volume)
{
    return audio_play_file_or_cw(filename, volume, 0);
}

bool play_file_from_sd(const char *filename)
{
    return play_file_from_sd(filename, 20);
}

/**
 * create tone and play via I2S -- audio task only
 */
void playTone(int duration_ms, int volume_percent) {
    int samples_per_period = SAMPLE_RATE / TONE_FREQ;
    int half_period = samples_per_period / 2;

    // buffer for 1 ms Ton (16bit Samples)
    int16_t buffer[BUFFER_SIZE];

    // fill buffer
    for (int i = 0; i < BUFFER_SIZE; i++) {
        int pos_in_period = i % samples_per_period;

        if (pos_in_period < half_period) {
            buffer[i] = (3000 * volume_percent) / 100;  // positiv Value (Amplitude)
        } else {
            buffer[i] = (-3000 * volume_percent) / 100; // negativ Value
        }
    }

    int total_ms = duration_ms;
    size_t bytes_written;

    // send buffer until duration is et
    while (total_ms > 0) {
        // Use a timeout instead of portMAX_DELAY to prevent freezing if I2S is not consuming data
        esp_err_t err = i2s_write(i2s_num, buffer, sizeof(buffer), &bytes_written, 100 / portTICK_PERIOD_MS);
        if (err != ESP_OK) {
            if (bDEBUG) 
                printfdeb("[AUDIO]..i2s_write failed: %d\n", err);
            break; // Exit loop on error to prevent freeze
        }
        total_ms--;
    }
}

/**
 * play a CW character -- audio task only
 */
static void play_cw_now(char character, int volume)
{
    String morseCode;

    switch (character)
    {
        case 'a': case 'A':
            morseCode = ".-";
            break;
        case 'b': case 'B':
            morseCode = "-...";
            break;
        case 'c': case 'C':
            morseCode = "-.-.";
            break;
        case 'd': case 'D':
            morseCode = "-..";
            break;
        case 'e': case 'E':
            morseCode = ".";
            break;
        case 'f': case 'F':
            morseCode = "..-.";
            break;
        case 'g': case 'G':
            morseCode = "--.";
            break;
        case 'h': case 'H':
            morseCode = "....";
            break;
        case 'i': case 'I':
            morseCode = "..";
            break;
        case 'j': case 'J':
            morseCode = ".---";
            break;
        case 'k': case 'K':
            morseCode = "-.-";
            break;
        case 'l': case 'L':
            morseCode = ".-..";
            break;
        case 'm': case 'M':
            morseCode = "--";
            break;
        case 'n': case 'N':
            morseCode = "-.";
            break;
        case 'o': case 'O':
            morseCode = "---";
            break;
        case 'p': case 'P':
            morseCode = ".--.";
            break;
        case 'q': case 'Q':
            morseCode = "--.-";
            break;
        case 'r': case 'R':
            morseCode = ".-.";
            break;
        case 's': case 'S':
            morseCode = "...";
            break;
        case 't': case 'T':
            morseCode = "-";
            break;
        case 'u': case 'U':
            morseCode = "..-";
            break;
        case 'v': case 'V':
            morseCode = "...-";
            break;
        case 'w': case 'W':
            morseCode = ".--";
            break;
        case 'x': case 'X':
            morseCode = "-..-";
            break;
        case 'y': case 'Y':
            morseCode = "-.--";
            break;
        case 'z': case 'Z':
            morseCode = "--..";
            break;
        case '0':
            morseCode = "-----";
            break;
        case '1':
            morseCode = ".----";
            break;
        case '2':
            morseCode = "..---";
            break;
        case '3':
            morseCode = "...--";
            break;
        case '4':
            morseCode = "....-";
            break;
        case '5':
            morseCode = ".....";
            break;
        case '6':
            morseCode = "-....";
            break;
        case '7':
            morseCode = "--...";
            break;
        case '8':
            morseCode = "---..";
            break;
        case '9':
            morseCode = "----.";
            break;
        case '.':
            morseCode = ".-.-.-";
            break;
        case ',':
            morseCode = "--..--";
            break;
        case '?':
            morseCode = "..--..";
            break;
        case '!':
            morseCode = "-.-.--";
            break;
        case ':':
            morseCode = "---...";
            break;
        case ';':
            morseCode = "-.-.-.";
            break;
        case '-':
            morseCode = "-....-";
            break;
        case '_':
            morseCode = "..--.-";
            break;
        case '(':
            morseCode = "-.--.";
            break;
        case ')':
            morseCode = "-.--.-";
            break;
        case '\'':
            morseCode = ".----.";
            break;
        case '=':
            morseCode = "-...-";
            break;
        case '+':
            morseCode = ".-.-.";
            break;
        case '/':
            morseCode = "-..-.";
            break;
        case '@':
            morseCode = ".--.-.";
            break;
        case '"':
            morseCode = ".-..-.";
        default:
            return;
    } 

    for (int i = 0; morseCode[i] != '\0'; i++) {
        if (morseCode[i] == '.')
        {
            playTone(DOT_DURATION, (volume * 100) / 22);
        }
        else if (morseCode[i] == '-')
        {
            playTone(DASH_DURATION, (volume * 100) / 22);
        }
        playTone(SYMBOL_PAUSE, 0); // pause (silence) between symbols
    }
    playTone(LETTER_PAUSE, 0); // pause (silence) between letters
}

/**
 * play a CW character (queued, returns immediately)
 */
void play_cw(const char character, int volume)
{
    if (meshcom_settings.node_mute)
    {
        if (bDEBUG)
            printlndeb("[AUDIO]..muted");
        return;
    }
    struct audio_request req = {};
    req.kind = AUDIO_REQ_CW;
    req.cw = character;
    req.volume = (int8_t)volume;
    audio_enqueue(req);
}

void play_cw(const char character)
{
    play_cw(character, 20);
}

/**
 * play CW start -- audio task only
 */
static void play_cw_start_now()
{
    if (bDEBUG)
        printlndeb("[AUDIO]..playing CW start");

    const char *morseCode = "-.-.-";

    for (int i = 0; morseCode[i] != '\0'; i++) {
        if (morseCode[i] == '.')
        {
            playTone(DOT_DURATION, (15 * 100) / 22);
        }
        else if (morseCode[i] == '-')
        {
            playTone(DASH_DURATION, (15 * 100) / 22);
        }
        playTone(SYMBOL_PAUSE, 0); // pause (silence) between symbols
    }
    playTone(LETTER_PAUSE, 0); // pause (silence) between letters
}

/**
 * play CW start (queued, returns immediately)
 */
void play_cw_start()
{
    if (meshcom_settings.node_mute)
    {
        if (bDEBUG)
            printlndeb("[AUDIO]..muted");
        return;
    }
    struct audio_request req = {};
    req.kind = AUDIO_REQ_CW_START;
    audio_enqueue(req);
}

/**
 * (re)install the I2S driver -- audio task only
 */
static void audio_i2s_install()
{
    // Re-install I2S driver with default settings (same as Audio lib default)
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = 16000,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        #if ESP_ARDUINO_VERSION_MAJOR >= 2
            .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_STAND_I2S),
        #else
            .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
        #endif
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 1024,
        .use_apll = false,
        .tx_desc_auto_clear = true,
        .fixed_mclk = I2S_PIN_NO_CHANGE
    };

    i2s_driver_install(i2s_num, &i2s_config, 0, NULL);

    // Re-apply pinout to Audio lib
    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
    audio.setVolume(12);
}

/**
 * file playback -- audio task only. SD and TFT share SPI2: every SD touch
 * (exists, open, each audio.loop() read) holds the bus mutex disp_flush()
 * uses, so the two never interleave on the wire. Returns false if the file
 * could not be started (missing, SD absent, connect failed).
 */
static bool audio_task_play_file(const char *filename, int volume)
{
    if (!bSDDected)
        return false;

    if (audio_dbg_mode == 1)
        return false;                       // EXPERIMENT: no SD access at all

    String strAudioWithType = filename;
    if (!strAudioWithType.startsWith("/"))
    {
        strAudioWithType = "/";
        strAudioWithType.concat(filename);
    }

    if (strAudioWithType.indexOf('.') <= 0)
        strAudioWithType.concat(".mp3");

    // init_audio() laeuft vor initTDeck(), das den Bus-Mutex anlegt; ein
    // Auftrag kann den Task also theoretisch vor dem Mutex erreichen.
    if (xSemaphore == NULL)
    {
        printlndeb("[AUDIO];err;no_bus_mutex");
        return false;
    }

    bool started = false;
    if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(AUDIO_BUS_WAIT_MS)) != pdTRUE)
    {
        printlndeb("[AUDIO];err;bus_busy");
        return false;
    }
    if (SD.exists(strAudioWithType.c_str()))
    {
        audio.setVolume(volume);
        started = audio.connecttoFS(SD, strAudioWithType.c_str());
        if (!started)
            printfdeb("[AUDIO];err;connect;%s\n", strAudioWithType.c_str());
    }
    else
    {
        printfdeb("[AUDIO]..file %s not found on SD\n", filename);
        printfdeb("[AUDIO];err;missing;%s\n", filename);
    }
    xSemaphoreGive(xSemaphore);

    if (!started)
        return false;

    printfdeb("[AUDIO];play;file;%s;vol;%d\n", strAudioWithType.c_str(), volume);

    while (audio.isRunning())
    {
        if (meshcom_settings.node_mute)
            break;
        if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(AUDIO_BUS_WAIT_MS)) == pdTRUE)
        {
            audio.loop();
            xSemaphoreGive(xSemaphore);
        }
        // vTaskDelay(10) is too long and causes buffer underruns/stuttering
        vTaskDelay(1);
    }
    audio.stopSong();
    return true;
}

/**
 * the audio task: waits for requests and plays them one after another
 */
static void play_function(void *parameter)
{
    if (bDEBUG)
    {
        printdeb("[AUDIO]..play_function running on core ");
        printlndeb(xPortGetCoreID());
    }

    struct audio_request req;
    for (;;)
    {
        if (xQueueReceive(audioQueue, &req, portMAX_DELAY) != pdTRUE)
            continue;

        if (req.kind == AUDIO_REQ_MUTE)
        {
            if (req.flag)
            {
                if (audio.isRunning())
                    audio.stopSong();
                i2s_driver_uninstall(i2s_num);
            }
            else
            {
                audio_i2s_install();
            }
            continue;
        }

        // Stummschaltung gilt fuer alles ausser dem Testkommando (flag)
        bool muted = meshcom_settings.node_mute;
        if (muted && !req.flag)
            continue;
        if (muted)
            audio_i2s_install();            // Testkommando: Hardware kurz einschalten

        switch (req.kind)
        {
            case AUDIO_REQ_FILE:
                if (!audio_task_play_file(req.path, req.volume) && req.cw != 0)
                {
                    if (audio_dbg_mode == 2)
                        printlndeb("[AUDIO];dbg;tone skipped");   // EXPERIMENT: no tone
                    else if (req.cw == AUDIO_CW_START)
                    {
                        printlndeb("[AUDIO];play;cw;start");
                        play_cw_start_now();
                    }
                    else
                    {
                        printfdeb("[AUDIO];play;cw;%c\n", req.cw);
                        play_cw_now(req.cw, 20);
                    }
                }
                break;
            case AUDIO_REQ_CW:
                if (audio_dbg_mode == 2)
                    printlndeb("[AUDIO];dbg;tone skipped");
                else
                    play_cw_now(req.cw, req.volume);
                break;
            case AUDIO_REQ_CW_START:
                play_cw_start_now();
                break;
            default:
                break;
        }

        if (muted)
            i2s_driver_uninstall(i2s_num);  // Testkommando: Zustand wiederherstellen
    }
}

/**
 * Set mute state. The flag takes effect at once (a running playback stops at
 * its next loop pass); the I2S driver teardown/reinstall is queued behind it
 * so it never runs while the audio task is inside audio.loop().
 */
void audio_set_mute(bool mute) {
    meshcom_settings.node_mute = mute;

    if (bDEBUG)
        printlndeb(mute ? "[AUDIO]..muting and disabling hardware"
                        : "[AUDIO]..unmuting and enabling hardware");

    struct audio_request req = {};
    req.kind = AUDIO_REQ_MUTE;
    req.flag = mute;
    audio_enqueue(req);
}

/**
 * single entry point for serial "play tone" commands
 * "start" -> CW start tone, "msg" -> CW message-received tone,
 * anything else -> play that file from SD
 * Test command: deliberately ignores node_mute so the tone path can be exercised in isolation.
 */
bool audio_play_tone(const char *what)
{
    struct audio_request req = {};
    req.flag = true;                        // Stummschaltung ignorieren
    req.volume = 20;
    if (strcmp(what, "start") == 0)
    {
        printlndeb("[AUDIO];play;start");
        req.kind = AUDIO_REQ_CW_START;
    }
    else if (strcmp(what, "msg") == 0)
    {
        printlndeb("[AUDIO];play;msg");
        req.kind = AUDIO_REQ_CW;
        req.cw = 'r';
    }
    else
    {
        req.kind = AUDIO_REQ_FILE;
        snprintf(req.path, sizeof(req.path), "%s", what);
    }
    return audio_enqueue(req);
}

#endif
