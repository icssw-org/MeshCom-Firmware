/**
 * @file        esp32_audio.h
 * @brief       functions for handling audio playback on ESP32
 * @author      Ing. Jakob Gurnhofer (OE3GJC)
 * @author      Ralph Weich (DD5RW)
 * @license     MIT
 * @copyright   Copyright (c) 2025 ICSSW.org
 * @date        2025-12-03
 */

#ifndef _ESP32_AUDIO_H_
#define _ESP32_AUDIO_H_

#include <Audio.h>

// Morse output parameters
#define TONE_FREQ       700     // 700 Hz Ton
#define SAMPLE_RATE     44100   // Audio-Sample-Rate
// duration Morsecode (in ms)
#define DOT_DURATION    100
#define DASH_DURATION   (3 * DOT_DURATION)
#define SYMBOL_PAUSE    DOT_DURATION
#define LETTER_PAUSE    (3 * DOT_DURATION)
#define WORD_PAUSE      (7 * DOT_DURATION)
// Buffergröße für einen Millisekunden-Ton
#define BUFFER_SIZE     (SAMPLE_RATE / 1000)

#define I2S_NUM         I2S_NUM_0

extern Audio audio;

void init_audio();
// Alle Aufrufe stellen nur einen Auftrag in die Audio-Queue und kehren sofort
// zurueck; abgespielt wird ausschliesslich im Audio-Task.
bool play_file_from_sd(const char *filename);
bool play_file_from_sd(const char *filename, int volume);
// cw_fallback: Morsezeichen, das der Audio-Task spielt, wenn die Datei fehlt;
// AUDIO_CW_START fuer die Startkennung, 0 fuer keinen Ersatz.
#define AUDIO_CW_START  '\x01'
bool audio_play_file_or_cw(const char *filename, int volume, char cw_fallback);
void play_cw(const char character);
void play_cw(const char character, int volume);
void play_cw_start();
void audio_set_mute(bool mute);
bool audio_play_tone(const char *what);
extern int audio_dbg_mode;

#endif