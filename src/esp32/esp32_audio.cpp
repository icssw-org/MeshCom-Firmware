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

#include <loop_functions_extern.h>
 
Audio audio;
SemaphoreHandle_t audioSemaphore;

// I2S-Handle
i2s_port_t i2s_num = I2S_NUM;

/**
 * initializes audio
 */
void init_audio()
{
    Serial.println("[audio]...initializing");

    audioSemaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(audioSemaphore);

    // Remove manual I2S driver installation as the Audio library handles this internally.
    // Double initialization causes "register I2S object to platform failed" errors.
    
    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
    audio.setVolume(12); // Set a default volume

    if (meshcom_settings.node_mute) {
        if (bDEBUG) Serial.println("[audio]...initially muted, disabling hardware");
        i2s_driver_uninstall(i2s_num);
    }
}

/**
 * play a supported file from SD in the background
 */
bool play_file_from_sd(const char *filename, int volume)
{
    if (meshcom_settings.node_mute)
    {
        if (bDEBUG)
        {
            Serial.println("[audio]...muted");
        }
        return true;
    }

    if (xSemaphoreTake(audioSemaphore, 0) == pdTRUE)
    {
        if(!bSDDected)
        {
            xSemaphoreGive(audioSemaphore);
            return false;
        }

        String strAudioWithType = filename;
        if(!strAudioWithType.startsWith("/"))
        {
            strAudioWithType = "/";
            strAudioWithType.concat(filename);
        }

        if(strAudioWithType.indexOf('.') <= 0)
            strAudioWithType.concat(".mp3");

        if (SD.exists(strAudioWithType.c_str()))
        {
            audio.setVolume(volume);
            audio.connecttoFS(SD, strAudioWithType.c_str());

            if (bDEBUG)
                Serial.printf("[audio]...playing %s in background\n", strAudioWithType.c_str());

            xTaskCreate(
                play_function,
                "audio play task",
                16 * 1024,
                NULL,
                50,
                NULL
            );
            return true;
        }
        else
        {
            Serial.printf("[audio]...file %s not found on SD\n", filename);
            xSemaphoreGive(audioSemaphore);
            return false;
        }
    }
    else
    {
        Serial.println("[audio]...currently playing another file");
        return true;
    }
}

/**
 * play a supported file from SD in the background
 */
bool play_file_from_sd(const char *filename)
{
    return play_file_from_sd(filename, 20);
}

/**
 * play a supported file from SD
 */
bool play_file_from_sd_blocking(const char *filename, int volume)
{
    if (meshcom_settings.node_mute)
    {
        if (bDEBUG)
            Serial.println("[audio]...muted");
        return true;
    }
    
    if(!bSDDected) return false;

    String strAudioWithType = filename;
    if(!strAudioWithType.startsWith("/"))
    {
        strAudioWithType = "/";
        strAudioWithType.concat(filename);
    }

    if(strAudioWithType.indexOf('.') <= 0)
        strAudioWithType.concat(".mp3");

    if (SD.exists(strAudioWithType.c_str()))
    {
        audio.setVolume(volume);
        audio.connecttoFS(SD, strAudioWithType.c_str());
        if (bDEBUG)
            Serial.printf("[audio]...playing %s\n", strAudioWithType.c_str());

        while (audio.isRunning())
        {
            if (meshcom_settings.node_mute) {
                break;
            }
            audio.loop();
        }
        audio.stopSong();

        return true;
    }
    else
    {
        Serial.printf("[audio]...file %s not found on SD\n", filename);
        return false;
    }
}

/**
 * play a supported file from SD
 */
bool play_file_from_sd_blocking(const char *filename)
{
    return play_file_from_sd_blocking(filename, 20);
}

/**
 * create tone and play via I2S
 */
void playTone(int duration_ms, int volume_percent) {
    if (meshcom_settings.node_mute) return;

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
                Serial.printf("[audio]...i2s_write failed: %d\n", err);
            break; // Exit loop on error to prevent freeze
        }
        total_ms--;
    }
}

/**
 * play a CW character
 */
void play_cw(const char character, int volume)
{
    if (meshcom_settings.node_mute)
    {
        if (bDEBUG)
            Serial.println("[audio]...muted");
        return;
    }

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
 * play a CW character
 */
void play_cw(const char character)
{
    return play_cw(character, 20);
}

/**
 * play CW start 
 */
void play_cw_start()
{
    if (meshcom_settings.node_mute)
    {
        if (bDEBUG)
            Serial.println("[audio]...muted");
        return;
    }

    if (bDEBUG)
        Serial.println("[audio]...playing CW start");

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
 * function for task to play audio in background
 */
void play_function(void *parameter)
{
    while (audio.isRunning()) {
        if (meshcom_settings.node_mute) {
            break;
        }
        audio.loop();
        // Reduce delay to minimum to keep audio buffer full
        // vTaskDelay(10) is too long and causes buffer underruns/stuttering
        vTaskDelay(1); 
    }
    audio.stopSong();

    xSemaphoreGive(audioSemaphore);

    vTaskDelete(NULL); // Use vTaskDelete instead of vTaskSuspend to properly clean up
}

/**
 * Set mute state and enable/disable audio hardware to save power
 */
void audio_set_mute(bool mute) {
    meshcom_settings.node_mute = mute;
    
    if (mute) {
        if (bDEBUG)
            Serial.println("[audio]...muting and disabling hardware");
        // Stop any playing audio
        if (audio.isRunning()) {
            audio.stopSong();
        }
        // Uninstall I2S driver to save power
        i2s_driver_uninstall(i2s_num);
    } else {
        if (bDEBUG)
            Serial.println("[audio]...unmuting and enabling hardware");
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
}

#endif