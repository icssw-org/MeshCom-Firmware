/**
 * meshcom_settings.h -- THE settings struct, defined once for both platforms
 * (D1-04 step 2, docs/BACKLOG.md "D1-04 target architecture").
 *
 * Until this header existed, `struct s_meshcom_settings` was written twice --
 * src/esp32/esp32_flash.h and src/nrf52/WisBlock-API.h -- and the two copies
 * drifted (23 named rows in docs/testplan/drift-matrix.csv, DR-12/DR-13).
 * They could only be merged once NOTHING depended on the struct's byte layout
 * any more, and as of W3 nothing does:
 *
 *   - ESP32 persists by NVS key (esp32_flash.cpp walks settings_schema).
 *   - nRF52 persists by key too (settings_store_nrf52.cpp, keyed file).
 *   - the nRF52 BLE settings characteristic ships the frozen s_ble_settings_v1
 *     image (src/nrf52/ble_settings_v1.h) and translates member by member.
 *   - the legacy nRF52 raw blob is read through that same frozen v1 type.
 *
 * So the member order below is free to change; only the NAMES are contracts
 * (they are the NVS keys / keyed-store keys via settings_schema).
 *
 * WHY X-MACROS
 * The struct body is generated from three member lists so that a native test
 * (test/test_settings_members) can ENUMERATE every member with offsetof()/
 * sizeof() and prove that each one is either covered by a settings_schema
 * descriptor or named in the explicit runtime lists of
 * src/meshcom_settings_runtime.h. That is the member-level, fail-closed gate
 * W3 step 3 asked for: a member added here without a schema row and without
 * a runtime entry fails the test instead of silently never persisting.
 *
 * Row shapes:
 *   M(type, name, default)                scalar member, `type name = default;`
 *   A(type, name, [bounds], init...)      array member, `type name[bounds] = init...;`
 *                                         (variadic so a braced initialiser
 *                                         with commas survives the macro)
 *
 * Platform-specific DEFAULTS are kept exactly as the two old structs had
 * them (MC_PLATFORM_DEFAULT) -- on nRF52 the struct default IS the first-boot
 * value (settings_store::decode() only overwrites keys it finds), on ESP32
 * init_flash() seeds most of these again before the walk. Do not "unify"
 * them without reading esp32_flash.cpp's pre-load block first.
 *
 * Platform-specific MEMBERS: the 13 T-Deck UI fields stay T-Deck-only and
 * node_disp_rot / node_ntp stay ESP32-only (operator decision 2026-09-13,
 * BACKLOG W3 step 4: platform-scoped via CFG_FIELD_LIST_PLATFORM, not inert
 * members on the other platform). Removed for good in the merge:
 * send_repeat_time / auto_join (LoRaWAN OTAA leftovers, timer scaffolding
 * deleted with them) and node_ackid (loaded, saved, never read).
 * node_msgid stays a member but is no longer a settings_schema row: it is a
 * counter, persisted through src/counters_store.h so a settings restore or
 * rewrite can never rewind it.
 */
#ifndef MESHCOM_SETTINGS_H
#define MESHCOM_SETTINGS_H

#include <stdint.h>

#define MESHCOM_DATA_MARKER 0x55

#ifdef ESP32
    #define MC_PLATFORM_DEFAULT(esp32_value, nrf52_value) esp32_value
#else
    #define MC_PLATFORM_DEFAULT(esp32_value, nrf52_value) nrf52_value
#endif

/* Members every platform has. Order: the old ESP32 struct's order, persisted
 * block first, then the block the old headers labelled "nicht im Flash". */
#define MESHCOM_SETTINGS_MEMBERS_COMMON(M, A)                                                           \
    M(uint8_t, valid_mark_1, 0xAA)                /* flash marker, legacy blob only */                  \
    M(uint8_t, valid_mark_2, MESHCOM_DATA_MARKER) /* flash marker, legacy blob only */                  \
    A(uint8_t, node_device_eui, [8], {0x00, 0x0D, 0x75, 0xE6, 0x56, 0x4D, 0xC1, 0xF3})                  \
    A(char, node_call, [10], {0x58, 0x58, 0x30, 0x58, 0x58, 0x58, 0x2D, 0x30, 0x30, 0x00})               \
    A(char, node_short, [6], {0x58, 0x58, 0x58, 0x34, 0x30, 0x00})                                       \
    M(double, node_lat, 0.0)                                                                             \
    M(char, node_lat_c, ' ')                                                                             \
    M(double, node_lon, 0.0)                                                                             \
    M(char, node_lon_c, ' ')                                                                             \
    M(int, node_alt, 0)                                                                                  \
    M(char, node_symid, '/')                                                                             \
    M(char, node_symcd, '#')                                                                             \
    A(char, node_ossid, [40], {0})                                                                       \
    A(char, node_opwd, [40], {0})                                                                        \
    M(int, node_date_year, 0)                                                                            \
    M(int, node_date_month, 0)                                                                           \
    M(int, node_date_day, 0)                                                                             \
    M(int, node_date_hour, 0)                                                                            \
    M(int, node_date_minute, 0)                                                                          \
    M(int, node_date_second, 0)                                                                          \
    M(int, node_date_hundredths, 0)                                                                      \
    M(unsigned long, node_age, 0)                                                                        \
    M(float, node_temp, 0)                                                                               \
    M(float, node_hum, 0)                                                                                \
    M(float, node_press, 0)                                                                              \
    M(int, node_hamnet_only, 0)                                                                          \
    M(int, node_sset, MC_PLATFORM_DEFAULT(0x0004, 0)) /* ESP32: bDisplayPos = true */                    \
    M(float, node_maxv, MC_PLATFORM_DEFAULT(4.24, 4.200))                                                \
    A(char, node_extern, [40], {0})                                                                      \
    M(int, node_msgid, 0) /* counter, see counters_store.h -- NOT a schema row */                        \
    M(int, node_power, MC_PLATFORM_DEFAULT(0, -20))                                                      \
    M(float, node_freq, 0)                                                                               \
    M(float, node_bw, 0)                                                                                 \
    M(int, node_sf, 0)                                                                                   \
    M(int, node_cr, 0)                                                                                   \
    A(char, node_atxt, [40], {0})                                                                        \
    M(int, node_sset2, 0x0000)                                                                           \
    M(int, node_owgpio, MC_PLATFORM_DEFAULT(36, 16))                                                     \
    M(float, node_temp2, 0)                                                                              \
    M(float, node_utcoff, MC_PLATFORM_DEFAULT(2, 0))                                                     \
    M(float, node_gas_res, 0) /* BME680 */                                                               \
    M(float, node_co2, 0)     /* CCS811 */                                                               \
    M(int, node_mcp17io, 0)   /* MCP23017 */                                                             \
    M(int, node_mcp17out, 0)                                                                             \
    M(int, node_mcp17in, 0)                                                                              \
    A(char, node_mcp17t, [16][16], {0})                                                                  \
    A(int, node_gcb, [6], {0}) /* GC fields */                                                           \
    M(int, node_country, 0)    /* 0 EU, 1 UK, 2 ON, 3 EA, 4 OM, 8 EU8, 10 US, ... 18 868, 19 915, 20 MAN */ \
    M(float, node_track_freq, 0)                                                                         \
    M(int, node_preamplebits, 0)                                                                         \
    M(int, node_ss_rx_pin, 0)                                                                            \
    M(int, node_ss_tx_pin, 0)                                                                            \
    M(int, node_ss_baud, 0)                                                                              \
    M(int, node_postime, 0)                                                                              \
    A(char, node_passwd, [15], {0})                                                                      \
    M(int, node_sset3, 0x0000)                                                                           \
    M(int, bt_code, 0)                                                                                   \
    M(int, node_button_pin, 0)                                                                           \
    A(char, node_ownip, [20], {0})                                                                       \
    A(char, node_owngw, [20], {0})                                                                       \
    A(char, node_ownms, [20], {0})                                                                       \
    A(char, node_name, [20], {0})                                                                        \
    A(char, node_webpwd, [20], {0})                                                                      \
    A(char, node_ssid, [33], {0})                                                                        \
    A(char, node_pwd, [64], {0})                                                                         \
    M(int, node_analog_pin, 99)                                                                          \
    M(float, node_analog_faktor, 1.0)                                                                    \
    A(char, node_parm, [50], {0})                                                                        \
    A(char, node_unit, [50], {0})                                                                        \
    A(char, node_format, [50], {0})                                                                      \
    A(char, node_eqns, [50], {0})                                                                        \
    A(char, node_values, [50], {0})                                                                      \
    M(int, node_parm_time, 15)                                                                           \
    M(float, node_specstart, 432.0) /* persisted on ESP32 only (CFG_FIELD_LIST_PLATFORM) */              \
    M(float, node_specend, 434.0)                                                                        \
    M(float, node_specstep, 0.025)                                                                       \
    M(int, node_specsamples, 2048)                                                                       \
    M(float, node_analog_batt_faktor, 0.0) /* persisted on ESP32 only */                                 \
    M(int, node_wifi_power, 60)                                                                          \
    A(char, node_lora_call, [10], {0x00})                                                                \
    M(float, node_analog_alpha, 0.0)                                                                     \
    M(float, node_analog_slope, MC_PLATFORM_DEFAULT(1.0, 0.0))                                           \
    M(float, node_analog_offset, 0.0)                                                                    \
    M(float, node_analog_atten, 0.0)                                                                     \
    A(char, node_gwsrv, [3], {0x00})                                                                     \
    M(float, node_tempi_off, 0.0)                                                                        \
    M(float, node_tempo_off, 0.0)                                                                        \
    M(float, node_shunt, 0.002)                                                                          \
    M(float, node_imax, 20.0)                                                                            \
    M(int, node_isamp, 7)                                                                                \
    A(char, node_owndns, [20], {0})                                                                      \
    M(int, node_contrast, 255)                                                                           \
    M(int, node_fversion, 1)                                                                             \
    A(char, node_ownntp, [40], {0})                                                                      \
    M(int, node_mversion, 0)                                                                             \
    A(char, node_fwversion, [8], {0})                                                                    \
    M(uint32_t, node_gpsbaud, 38400) /* CFG_U32 range 1200..921600 needs 32 bit */                       \
    M(int, node_cleanflash, 0)                                                                           \
    M(int, node_netmode, 0)  /* 0 WiFi, 1 Ethernet */                                                    \
    M(int, node_gpsdebug, 0) /* 0 none, 1 only valid, 2 all */                                           \
    M(int, node_relay, 0x0000)                                                                           \
    A(char, node_via, [40], {0})                                                                         \
    M(int, node_sset4, 0x0002) /* DEBUGEN = true */                                                      \
    A(char, node_aprsmc, [10], {0})                                                                      \
    M(int, node_pingtime, 0)                                                                             \
    A(char, node_pingcall, [10], {0})                                                                    \
    M(int, node_pingmax, 0)                                                                              \
    /* ---- runtime state, never persisted ("nicht im Flash" in the old headers) ---- */                 \
    M(int, node_press_alt, 0)                                                                            \
    M(float, node_press_asl, 0)                                                                          \
    M(float, node_vbus, 0)                                                                               \
    M(float, node_vshunt, 0)                                                                             \
    M(float, node_vcurrent, 0)                                                                           \
    M(float, node_vpower, 0)                                                                             \
    A(char, node_ip, [40], {0})                                                                          \
    A(char, node_dns, [40], {0})                                                                         \
    A(char, node_gw, [40], {0})                                                                          \
    A(char, node_subnet, [40], {0})                                                                      \
    M(bool, node_hasIPaddress, false)                                                                    \
    M(unsigned long, node_last_upd_timer, 0)                                                             \
    M(int, max_hop_text, 0)                                                                              \
    M(int, max_hop_pos, 0)                                                                               \
    A(char, node_update, [20], {0}) /* 19-char timestamp + NUL */                                        \
    A(char, node_parm_1, [100], {0})                                                                     \
    A(char, node_parm_t, [150], {0})                                                                     \
    A(char, node_parm_id, [100], {0})                                                                    \
    M(float, node_ntctemp, 0.0)                                                                          \
    M(bool, node_fanon, false)                                                                           \
    M(int, node_pingcount, 0)                                                                            \
    M(unsigned long, node_pingduration, 0)

/* ESP32-only members (every ESP32 board). */
#ifdef ESP32
    #define MESHCOM_SETTINGS_MEMBERS_ESP32(M, A)                                                        \
        /* --rotate 0/90/180/270, additive to the board's base rotation (applyDisplayRotation()); \
         * NVS key "node_disrot" */                                                                  \
        M(int, node_disp_rot, 0)                                                                     \
        /* runtime: the NTP host in use, recopied from node_ownntp on every connect */              \
        A(char, node_ntp, [40], {0})
#else
    #define MESHCOM_SETTINGS_MEMBERS_ESP32(M, A)
#endif

/* T-Deck device UI state. Persisted (SETTINGS_PERSIST_ONLY_LIST_PLATFORM),
 * deliberately not exported, and deliberately absent on every other board. */
#if defined(ESP32) && (defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS) || defined(BOARD_T_DECK_PRO))
    #define MESHCOM_SETTINGS_MEMBERS_TDECK(M, A)                                                        \
        M(int, node_map, 0)                                                                          \
        /* fixed char[] on purpose: a descriptor persists offsetof()/sizeof() bytes, so an       \
         * Arduino String here would persist its control block, not the path. 128 covers the     \
         * setup screen's lv_textarea_set_max_length(100) with margin. */                          \
        A(char, node_audio_start, [128], "/")                                                        \
        A(char, node_audio_msg, [128], "/")                                                          \
        M(bool, node_keyboardlock, false)                                                            \
        M(bool, node_backlightlock, false)                                                           \
        M(bool, node_kbllightlock, false)                                                            \
        M(int, node_modus, 0) /* 0 OFF, 1 KB LOCK, 2 LIGHT ON, 3 KBL&LIGHT */                        \
        M(bool, node_mute, false)                                                                    \
        /* true: persist every incoming non-system message immediately (test aid, flash wear) */   \
        M(bool, node_persist_to_flash, false)                                                        \
        M(bool, node_persist_to_sd, false)                                                           \
        M(bool, node_immediate_save, false)                                                          \
        M(bool, node_kbl_sync, true)                                                                 \
        M(bool, node_wifion, true)
#else
    #define MESHCOM_SETTINGS_MEMBERS_TDECK(M, A)
#endif

#define MESHCOM_SETTINGS_MEMBERS(M, A) \
    MESHCOM_SETTINGS_MEMBERS_COMMON(M, A) \
    MESHCOM_SETTINGS_MEMBERS_ESP32(M, A) \
    MESHCOM_SETTINGS_MEMBERS_TDECK(M, A)

struct s_meshcom_settings
{
#define MC_SETTINGS_M(type, name, dflt) type name = dflt;
#define MC_SETTINGS_A(type, name, bounds, ...) type name bounds = __VA_ARGS__;
    MESHCOM_SETTINGS_MEMBERS(MC_SETTINGS_M, MC_SETTINGS_A)
#undef MC_SETTINGS_M
#undef MC_SETTINGS_A
};

extern s_meshcom_settings meshcom_settings;

#endif // MESHCOM_SETTINGS_H
