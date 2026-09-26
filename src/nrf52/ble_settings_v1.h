/**
 * @file ble_settings_v1.h
 * @brief Frozen v1 wire format for the nRF52 BLE settings characteristic.
 *
 * WHY THIS EXISTS
 * ----------------------------------------------------------------------
 * The MeshCom phone app talks to the nRF52 BLE settings characteristic by
 * exchanging the settings struct AS RAW BYTES -- there is no version field,
 * length is the only negotiation (src/nrf52/nrf52_ble.cpp). Today that raw
 * struct is `s_meshcom_settings` (src/nrf52/WisBlock-API.h) itself, so any
 * future reorder, removal, or resize of that struct silently breaks every
 * phone app already in the field: either the write is rejected on length,
 * or -- worse, if the size coincidentally still matches -- it is accepted
 * and memcpy'd into misaligned fields, scrambling a user's configuration.
 *
 * `s_ble_settings_v1` is a byte-for-byte SNAPSHOT of `s_meshcom_settings` as
 * it stood at commit 64ba5774 (the commit this struct was frozen against).
 * It is a completely independent type: nothing here changes when
 * s_meshcom_settings changes later. src/nrf52/nrf52_ble.cpp translates
 * between the two with bleSettingsToV1()/bleSettingsFromV1() below, so the
 * wire format the app already understands stays fixed even as the live
 * struct is reorganised in a later wave.
 *
 * THE STATIC_ASSERTS ARE THE WHOLE SAFETY MECHANISM. If anyone edits a
 * member below -- reorders it, changes its type, resizes an array -- the
 * offsetof() of every member after it shifts, and the corresponding
 * static_assert fails the build immediately. The numbers are DERIVED BY
 * COMPILING (a throwaway probe built against the real nRF52 (ARM EABI)
 * toolchain -- see the wave report for the exact command), not computed by
 * hand, and must NEVER be "fixed" to match a struct that changed: a change
 * here means the wire format has changed, which is precisely the accident
 * this file exists to prevent silently. If sizeof/offsetof legitimately
 * needs to change, that is a new v2 format with its own type, not an edit
 * to this one.
 *
 * The two marker fields (valid_mark_1 == 0xAA, valid_mark_2 == 0x55) are
 * validated by settings_rx_callback() (nrf52_ble.cpp) before anything is
 * staged; they stay part of the v1 image and are frozen the same way as
 * every other member -- see BLE_SETTINGS_V1_MARK_1/2 below, which are
 * deliberately NOT the same macro as the live MESHCOM_DATA_MARKER so that a
 * future change to that macro cannot silently change what v1 validates.
 */
#ifndef BLE_SETTINGS_V1_H
#define BLE_SETTINGS_V1_H

// Quoted, same-directory include: this always resolves to the real
// src/nrf52/WisBlock-API.h (it lives right next to this file), exactly the
// way src/nrf52/nrf52_ble.cpp already includes it. Needed here only for the
// s_meshcom_settings type used by the conversion function signatures below.
#include "WisBlock-API.h"

#include <cstddef> // offsetof
#include <cstdint>

// Frozen wire-format marker values, exactly as validated at commit 64ba5774
// (src/nrf52/WisBlock-API.h:180-181, src/nrf52/nrf52_ble.cpp:346-348). These
// must NEVER be redefined to track a future change to MESHCOM_DATA_MARKER --
// v1 documents what the app already understands on the wire today.
static constexpr uint8_t BLE_SETTINGS_V1_MARK_1 = 0xAA;
static constexpr uint8_t BLE_SETTINGS_V1_MARK_2 = 0x55; // == MESHCOM_DATA_MARKER at freeze time

/**
 * s_ble_settings_v1 -- frozen, byte-for-byte copy of s_meshcom_settings'
 * layout as it stood at commit 64ba5774. Same member order, same types,
 * same array bounds as the real struct at that commit. Do not add, remove,
 * reorder or resize a member here for any reason -- see the file banner.
 */
struct s_ble_settings_v1
{
	uint8_t valid_mark_1 = BLE_SETTINGS_V1_MARK_1; // Just a marker for the Flash
	uint8_t valid_mark_2 = BLE_SETTINGS_V1_MARK_2; // Just a marker for the Flash
												// OTAA Device EUI MSB

	// OTAA Device EUI MSB
	uint8_t node_device_eui[8] = {0x00, 0x0D, 0x75, 0xE6, 0x56, 0x4D, 0xC1, 0xF3};

	char node_call[10] = {0x58, 0x58, 0x30, 0x58, 0x58, 0x58, 0x2D, 0x30, 0x30, 0x00};
	char node_short[6] = {0x58, 0x58, 0x58, 0x34, 0x30, 0x00};

	double node_lat = 0.0;
	char node_lat_c = {' '};
	double node_lon = 0.0;
	char node_lon_c = {' '};
	int	  node_alt = 0;
	char  node_symid = '/';
	char  node_symcd = '#';

	int node_date_year = 0;
	int node_date_month = 0;
	int node_date_day = 0;

	int node_date_hour = 0;
	int node_date_minute = 0;
	int node_date_second = 0;
	int node_date_hundredths = 0;

	unsigned long node_age = 0;

	float node_temp = 0;
	float node_hum = 0;
	float node_press = 0;

	char node_ossid[40] = {0};
	char node_opwd[40] = {0};

	// Default is off
	uint32_t send_repeat_time = 0;

	bool auto_join = false;

	int node_hamnet_only = 0;

	int node_sset = 0;

	float node_maxv = 4.200;

	char node_extern[40] = {0};

	int node_msgid = 0;
	int node_ackid = 0;

	int node_power = -20;
	float node_freq = 0;
	float node_bw = 0;
	int node_sf = 0;
	int node_cr = 0;

	char node_atxt[40] = {0};

	int node_sset2 = 0;
	int node_owgpio = 16;

	float node_temp2 = 0;

	float node_utcoff = 0;

	// BME680
	float node_gas_res = 0;

	// CMCU-811
	float node_co2 = 0;

	// MCP23017
	int node_mcp17io = 0;
	char node_mcp17t[16][16] = {0};
	int node_mcp17out = 0;
	int node_mcp17in = 0;

	// GC Fields
	int node_gcb[6] = {0};

	// internatioal fields
	int node_country = 0;

	float node_track_freq = 0;
	int node_preamplebits = 0;

	int node_ss_rx_pin = 0;
	int node_ss_tx_pin = 0;
	int node_ss_baud = 0;

	int node_postime = 0;

	char node_passwd[15] = {0};

	int node_sset3 = 0;

	int bt_code = 0;

	int node_button_pin = 0;

	char node_ownip[20] = {0};
	char node_owngw[20] = {0};
	char node_ownms[20] = {0};

	char node_name[20] = {0};

	char node_webpwd[20] = {0};

	char node_ssid[33] = {0};
	char node_pwd[64] = {0};

	int node_analog_pin = 99;
	float node_analog_faktor = 1.0;

	char node_parm[50] = {0};
	char node_unit[50] = {0};
	char node_format[50] = {0};
	char node_eqns[50] = {0};
	char node_values[50] = {0};

	int node_parm_time = 15;

	int node_wifi_power = 60;
	char node_lora_call[10] = {0x00};

	float node_analog_alpha = 0.0;
	float node_analog_slope = 0.0;
	float node_analog_offset = 0.0;
	float node_analog_atten = 0.0;

	char node_gwsrv[3] = {0x00};

	float node_tempi_off = 0.0;
	float node_tempo_off = 0.0;

	float node_shunt = 0.002;
	float node_imax = 20.0;
	int node_isamp = 7;

	char node_owndns[20] = {0};

	int node_contrast = 255;
	int node_fversion = 1;

	char node_ownntp[40] = {0};

	int node_mversion = 0;
	char node_fwversion[8] = {0};

	uint32_t node_gpsbaud = 38400;   // D1-04: one spelling on both platforms; CFG_U32 range 1200..921600 needs 32 bit

	int node_cleanflash = 0;

	int node_netmode = 0;

	int node_gpsdebug = 0;

	int node_relay = 0x0000;

	char node_via[40] = {0};

	int node_sset4 = 0x0002;

	char node_aprsmc[10] = {0};

	int node_pingtime = 0;
	char node_pingcall[10] = {0};
	int node_pingmax = 0;

	//////////////////////////////////////////////////////////////////////////////////////////////
	// nicht im Flash
	float node_specstart = 432.0;
	float node_specend = 434.0;
	float node_specstep = 0.025;
	int node_specsamples = 2048;

	float node_analog_batt_faktor = 0.0;


	int node_press_alt = 0;
	float node_press_asl = 0;
	float node_vbus = 0;
	float node_vshunt = 0;
	float node_vcurrent = 0;
	float node_vpower = 0;

	char node_ip[40] = {0};
	char node_dns[40] = {0};
	char node_gw[40] = {0};
	char node_subnet[40] = {0};
	bool node_hasIPaddress = false;
	unsigned long node_last_upd_timer = 0;

	int max_hop_text = 0;
	int max_hop_pos = 0;

	// 9999-99-99 00:00:00
	char node_update[20] = {0};

	char node_parm_1[100] = {0};
	char node_parm_t[150] = {0};
	char node_parm_id[100] = {0};

	float node_ntctemp = 0.0;
	bool node_fanon = false;

	int node_pingcount = 0;
	unsigned long node_pingduration = 0;
};

// ---------------------------------------------------------------------------
// FROZEN CONTRACT -- v1 wire layout, ARM EABI (nRF52 target: wiscore_rak4631
// / t_echo, arm-none-eabi-g++, -std=gnu++17). Every literal below was read
// off a throwaway offsetof()/sizeof() probe compiled with
// `pio run -e wiscore_rak4631` against this exact struct (a
// template<unsigned long V> struct ProbeVal; ProbeVal<offsetof(...)> x;
// trick -- the "incomplete type" compiler error prints the numeric value in
// its own text, so the numbers below were read out of a compiler diagnostic,
// never computed by hand). Do not "fix" a literal here to match an edited
// struct above -- that is exactly the silent drift this file exists to
// catch. A future v2 format gets its own type instead.
//
// NATIVE_BUILD (the host test toolchain, x86_64/arm64 macOS clang or Linux
// gcc) genuinely produces a DIFFERENT layout for this exact struct -- double
// alignment and padding differ from the ARM EABI target, empirically
// verified while writing this file (sizeof came back 2008, not 2000, with
// several offsets shifted by 4). These asserts pin the ONE layout that
// actually ships to the phone app -- the ARM one -- so they are compiled
// only for real firmware builds; test/test_ble_settings_v1 gets its
// regression coverage instead from a host-layout golden byte fixture (which
// pins the host's own, different, but equally deterministic layout) plus
// the round-trip and field-list tests, none of which depend on a specific
// byte offset. See the wave report for the full reasoning.
// ---------------------------------------------------------------------------
#ifndef NATIVE_BUILD
static_assert(sizeof(s_ble_settings_v1) == 2000, "s_ble_settings_v1 total size drifted from the frozen v1 wire contract");

static_assert(offsetof(s_ble_settings_v1, valid_mark_1) == 0, "v1 offset drift: valid_mark_1");
static_assert(offsetof(s_ble_settings_v1, valid_mark_2) == 1, "v1 offset drift: valid_mark_2");
static_assert(offsetof(s_ble_settings_v1, node_device_eui) == 2, "v1 offset drift: node_device_eui");
static_assert(offsetof(s_ble_settings_v1, node_call) == 10, "v1 offset drift: node_call");
static_assert(offsetof(s_ble_settings_v1, node_short) == 20, "v1 offset drift: node_short");
static_assert(offsetof(s_ble_settings_v1, node_lat) == 32, "v1 offset drift: node_lat");
static_assert(offsetof(s_ble_settings_v1, node_lat_c) == 40, "v1 offset drift: node_lat_c");
static_assert(offsetof(s_ble_settings_v1, node_lon) == 48, "v1 offset drift: node_lon");
static_assert(offsetof(s_ble_settings_v1, node_lon_c) == 56, "v1 offset drift: node_lon_c");
static_assert(offsetof(s_ble_settings_v1, node_alt) == 60, "v1 offset drift: node_alt");
static_assert(offsetof(s_ble_settings_v1, node_symid) == 64, "v1 offset drift: node_symid");
static_assert(offsetof(s_ble_settings_v1, node_symcd) == 65, "v1 offset drift: node_symcd");
static_assert(offsetof(s_ble_settings_v1, node_date_year) == 68, "v1 offset drift: node_date_year");
static_assert(offsetof(s_ble_settings_v1, node_date_month) == 72, "v1 offset drift: node_date_month");
static_assert(offsetof(s_ble_settings_v1, node_date_day) == 76, "v1 offset drift: node_date_day");
static_assert(offsetof(s_ble_settings_v1, node_date_hour) == 80, "v1 offset drift: node_date_hour");
static_assert(offsetof(s_ble_settings_v1, node_date_minute) == 84, "v1 offset drift: node_date_minute");
static_assert(offsetof(s_ble_settings_v1, node_date_second) == 88, "v1 offset drift: node_date_second");
static_assert(offsetof(s_ble_settings_v1, node_date_hundredths) == 92, "v1 offset drift: node_date_hundredths");
static_assert(offsetof(s_ble_settings_v1, node_age) == 96, "v1 offset drift: node_age");
static_assert(offsetof(s_ble_settings_v1, node_temp) == 100, "v1 offset drift: node_temp");
static_assert(offsetof(s_ble_settings_v1, node_hum) == 104, "v1 offset drift: node_hum");
static_assert(offsetof(s_ble_settings_v1, node_press) == 108, "v1 offset drift: node_press");
static_assert(offsetof(s_ble_settings_v1, node_ossid) == 112, "v1 offset drift: node_ossid");
static_assert(offsetof(s_ble_settings_v1, node_opwd) == 152, "v1 offset drift: node_opwd");
static_assert(offsetof(s_ble_settings_v1, send_repeat_time) == 192, "v1 offset drift: send_repeat_time");
static_assert(offsetof(s_ble_settings_v1, auto_join) == 196, "v1 offset drift: auto_join");
static_assert(offsetof(s_ble_settings_v1, node_hamnet_only) == 200, "v1 offset drift: node_hamnet_only");
static_assert(offsetof(s_ble_settings_v1, node_sset) == 204, "v1 offset drift: node_sset");
static_assert(offsetof(s_ble_settings_v1, node_maxv) == 208, "v1 offset drift: node_maxv");
static_assert(offsetof(s_ble_settings_v1, node_extern) == 212, "v1 offset drift: node_extern");
static_assert(offsetof(s_ble_settings_v1, node_msgid) == 252, "v1 offset drift: node_msgid");
static_assert(offsetof(s_ble_settings_v1, node_ackid) == 256, "v1 offset drift: node_ackid");
static_assert(offsetof(s_ble_settings_v1, node_power) == 260, "v1 offset drift: node_power");
static_assert(offsetof(s_ble_settings_v1, node_freq) == 264, "v1 offset drift: node_freq");
static_assert(offsetof(s_ble_settings_v1, node_bw) == 268, "v1 offset drift: node_bw");
static_assert(offsetof(s_ble_settings_v1, node_sf) == 272, "v1 offset drift: node_sf");
static_assert(offsetof(s_ble_settings_v1, node_cr) == 276, "v1 offset drift: node_cr");
static_assert(offsetof(s_ble_settings_v1, node_atxt) == 280, "v1 offset drift: node_atxt");
static_assert(offsetof(s_ble_settings_v1, node_sset2) == 320, "v1 offset drift: node_sset2");
static_assert(offsetof(s_ble_settings_v1, node_owgpio) == 324, "v1 offset drift: node_owgpio");
static_assert(offsetof(s_ble_settings_v1, node_temp2) == 328, "v1 offset drift: node_temp2");
static_assert(offsetof(s_ble_settings_v1, node_utcoff) == 332, "v1 offset drift: node_utcoff");
static_assert(offsetof(s_ble_settings_v1, node_gas_res) == 336, "v1 offset drift: node_gas_res");
static_assert(offsetof(s_ble_settings_v1, node_co2) == 340, "v1 offset drift: node_co2");
static_assert(offsetof(s_ble_settings_v1, node_mcp17io) == 344, "v1 offset drift: node_mcp17io");
static_assert(offsetof(s_ble_settings_v1, node_mcp17t) == 348, "v1 offset drift: node_mcp17t");
static_assert(offsetof(s_ble_settings_v1, node_mcp17out) == 604, "v1 offset drift: node_mcp17out");
static_assert(offsetof(s_ble_settings_v1, node_mcp17in) == 608, "v1 offset drift: node_mcp17in");
static_assert(offsetof(s_ble_settings_v1, node_gcb) == 612, "v1 offset drift: node_gcb");
static_assert(offsetof(s_ble_settings_v1, node_country) == 636, "v1 offset drift: node_country");
static_assert(offsetof(s_ble_settings_v1, node_track_freq) == 640, "v1 offset drift: node_track_freq");
static_assert(offsetof(s_ble_settings_v1, node_preamplebits) == 644, "v1 offset drift: node_preamplebits");
static_assert(offsetof(s_ble_settings_v1, node_ss_rx_pin) == 648, "v1 offset drift: node_ss_rx_pin");
static_assert(offsetof(s_ble_settings_v1, node_ss_tx_pin) == 652, "v1 offset drift: node_ss_tx_pin");
static_assert(offsetof(s_ble_settings_v1, node_ss_baud) == 656, "v1 offset drift: node_ss_baud");
static_assert(offsetof(s_ble_settings_v1, node_postime) == 660, "v1 offset drift: node_postime");
static_assert(offsetof(s_ble_settings_v1, node_passwd) == 664, "v1 offset drift: node_passwd");
static_assert(offsetof(s_ble_settings_v1, node_sset3) == 680, "v1 offset drift: node_sset3");
static_assert(offsetof(s_ble_settings_v1, bt_code) == 684, "v1 offset drift: bt_code");
static_assert(offsetof(s_ble_settings_v1, node_button_pin) == 688, "v1 offset drift: node_button_pin");
static_assert(offsetof(s_ble_settings_v1, node_ownip) == 692, "v1 offset drift: node_ownip");
static_assert(offsetof(s_ble_settings_v1, node_owngw) == 712, "v1 offset drift: node_owngw");
static_assert(offsetof(s_ble_settings_v1, node_ownms) == 732, "v1 offset drift: node_ownms");
static_assert(offsetof(s_ble_settings_v1, node_name) == 752, "v1 offset drift: node_name");
static_assert(offsetof(s_ble_settings_v1, node_webpwd) == 772, "v1 offset drift: node_webpwd");
static_assert(offsetof(s_ble_settings_v1, node_ssid) == 792, "v1 offset drift: node_ssid");
static_assert(offsetof(s_ble_settings_v1, node_pwd) == 825, "v1 offset drift: node_pwd");
static_assert(offsetof(s_ble_settings_v1, node_analog_pin) == 892, "v1 offset drift: node_analog_pin");
static_assert(offsetof(s_ble_settings_v1, node_analog_faktor) == 896, "v1 offset drift: node_analog_faktor");
static_assert(offsetof(s_ble_settings_v1, node_parm) == 900, "v1 offset drift: node_parm");
static_assert(offsetof(s_ble_settings_v1, node_unit) == 950, "v1 offset drift: node_unit");
static_assert(offsetof(s_ble_settings_v1, node_format) == 1000, "v1 offset drift: node_format");
static_assert(offsetof(s_ble_settings_v1, node_eqns) == 1050, "v1 offset drift: node_eqns");
static_assert(offsetof(s_ble_settings_v1, node_values) == 1100, "v1 offset drift: node_values");
static_assert(offsetof(s_ble_settings_v1, node_parm_time) == 1152, "v1 offset drift: node_parm_time");
static_assert(offsetof(s_ble_settings_v1, node_wifi_power) == 1156, "v1 offset drift: node_wifi_power");
static_assert(offsetof(s_ble_settings_v1, node_lora_call) == 1160, "v1 offset drift: node_lora_call");
static_assert(offsetof(s_ble_settings_v1, node_analog_alpha) == 1172, "v1 offset drift: node_analog_alpha");
static_assert(offsetof(s_ble_settings_v1, node_analog_slope) == 1176, "v1 offset drift: node_analog_slope");
static_assert(offsetof(s_ble_settings_v1, node_analog_offset) == 1180, "v1 offset drift: node_analog_offset");
static_assert(offsetof(s_ble_settings_v1, node_analog_atten) == 1184, "v1 offset drift: node_analog_atten");
static_assert(offsetof(s_ble_settings_v1, node_gwsrv) == 1188, "v1 offset drift: node_gwsrv");
static_assert(offsetof(s_ble_settings_v1, node_tempi_off) == 1192, "v1 offset drift: node_tempi_off");
static_assert(offsetof(s_ble_settings_v1, node_tempo_off) == 1196, "v1 offset drift: node_tempo_off");
static_assert(offsetof(s_ble_settings_v1, node_shunt) == 1200, "v1 offset drift: node_shunt");
static_assert(offsetof(s_ble_settings_v1, node_imax) == 1204, "v1 offset drift: node_imax");
static_assert(offsetof(s_ble_settings_v1, node_isamp) == 1208, "v1 offset drift: node_isamp");
static_assert(offsetof(s_ble_settings_v1, node_owndns) == 1212, "v1 offset drift: node_owndns");
static_assert(offsetof(s_ble_settings_v1, node_contrast) == 1232, "v1 offset drift: node_contrast");
static_assert(offsetof(s_ble_settings_v1, node_fversion) == 1236, "v1 offset drift: node_fversion");
static_assert(offsetof(s_ble_settings_v1, node_ownntp) == 1240, "v1 offset drift: node_ownntp");
static_assert(offsetof(s_ble_settings_v1, node_mversion) == 1280, "v1 offset drift: node_mversion");
static_assert(offsetof(s_ble_settings_v1, node_fwversion) == 1284, "v1 offset drift: node_fwversion");
static_assert(offsetof(s_ble_settings_v1, node_gpsbaud) == 1292, "v1 offset drift: node_gpsbaud");
static_assert(offsetof(s_ble_settings_v1, node_cleanflash) == 1296, "v1 offset drift: node_cleanflash");
static_assert(offsetof(s_ble_settings_v1, node_netmode) == 1300, "v1 offset drift: node_netmode");
static_assert(offsetof(s_ble_settings_v1, node_gpsdebug) == 1304, "v1 offset drift: node_gpsdebug");
static_assert(offsetof(s_ble_settings_v1, node_relay) == 1308, "v1 offset drift: node_relay");
static_assert(offsetof(s_ble_settings_v1, node_via) == 1312, "v1 offset drift: node_via");
static_assert(offsetof(s_ble_settings_v1, node_sset4) == 1352, "v1 offset drift: node_sset4");
static_assert(offsetof(s_ble_settings_v1, node_aprsmc) == 1356, "v1 offset drift: node_aprsmc");
static_assert(offsetof(s_ble_settings_v1, node_pingtime) == 1368, "v1 offset drift: node_pingtime");
static_assert(offsetof(s_ble_settings_v1, node_pingcall) == 1372, "v1 offset drift: node_pingcall");
static_assert(offsetof(s_ble_settings_v1, node_pingmax) == 1384, "v1 offset drift: node_pingmax");
static_assert(offsetof(s_ble_settings_v1, node_specstart) == 1388, "v1 offset drift: node_specstart");
static_assert(offsetof(s_ble_settings_v1, node_specend) == 1392, "v1 offset drift: node_specend");
static_assert(offsetof(s_ble_settings_v1, node_specstep) == 1396, "v1 offset drift: node_specstep");
static_assert(offsetof(s_ble_settings_v1, node_specsamples) == 1400, "v1 offset drift: node_specsamples");
static_assert(offsetof(s_ble_settings_v1, node_analog_batt_faktor) == 1404, "v1 offset drift: node_analog_batt_faktor");
static_assert(offsetof(s_ble_settings_v1, node_press_alt) == 1408, "v1 offset drift: node_press_alt");
static_assert(offsetof(s_ble_settings_v1, node_press_asl) == 1412, "v1 offset drift: node_press_asl");
static_assert(offsetof(s_ble_settings_v1, node_vbus) == 1416, "v1 offset drift: node_vbus");
static_assert(offsetof(s_ble_settings_v1, node_vshunt) == 1420, "v1 offset drift: node_vshunt");
static_assert(offsetof(s_ble_settings_v1, node_vcurrent) == 1424, "v1 offset drift: node_vcurrent");
static_assert(offsetof(s_ble_settings_v1, node_vpower) == 1428, "v1 offset drift: node_vpower");
static_assert(offsetof(s_ble_settings_v1, node_ip) == 1432, "v1 offset drift: node_ip");
static_assert(offsetof(s_ble_settings_v1, node_dns) == 1472, "v1 offset drift: node_dns");
static_assert(offsetof(s_ble_settings_v1, node_gw) == 1512, "v1 offset drift: node_gw");
static_assert(offsetof(s_ble_settings_v1, node_subnet) == 1552, "v1 offset drift: node_subnet");
static_assert(offsetof(s_ble_settings_v1, node_hasIPaddress) == 1592, "v1 offset drift: node_hasIPaddress");
static_assert(offsetof(s_ble_settings_v1, node_last_upd_timer) == 1596, "v1 offset drift: node_last_upd_timer");
static_assert(offsetof(s_ble_settings_v1, max_hop_text) == 1600, "v1 offset drift: max_hop_text");
static_assert(offsetof(s_ble_settings_v1, max_hop_pos) == 1604, "v1 offset drift: max_hop_pos");
static_assert(offsetof(s_ble_settings_v1, node_update) == 1608, "v1 offset drift: node_update");
static_assert(offsetof(s_ble_settings_v1, node_parm_1) == 1628, "v1 offset drift: node_parm_1");
static_assert(offsetof(s_ble_settings_v1, node_parm_t) == 1728, "v1 offset drift: node_parm_t");
static_assert(offsetof(s_ble_settings_v1, node_parm_id) == 1878, "v1 offset drift: node_parm_id");
static_assert(offsetof(s_ble_settings_v1, node_ntctemp) == 1980, "v1 offset drift: node_ntctemp");
static_assert(offsetof(s_ble_settings_v1, node_fanon) == 1984, "v1 offset drift: node_fanon");
static_assert(offsetof(s_ble_settings_v1, node_pingcount) == 1988, "v1 offset drift: node_pingcount");
static_assert(offsetof(s_ble_settings_v1, node_pingduration) == 1992, "v1 offset drift: node_pingduration");
#endif // NATIVE_BUILD

// ---------------------------------------------------------------------------
// Conversion (Task 2, src/nrf52/nrf52_ble.cpp)
// ---------------------------------------------------------------------------

/**
 * Populates `out` (the frozen wire image) from the live settings struct.
 * Used for every BLE read/write()/notify() of the settings characteristic.
 */
void bleSettingsToV1(const s_meshcom_settings &src, s_ble_settings_v1 &out);

/**
 * Populates `out` (the live settings struct) from a wire image already
 * validated (length + markers) by the caller. Does NOT validate -- that is
 * settings_rx_callback()'s job, and must happen on the raw bytes BEFORE
 * this is ever called.
 */
void bleSettingsFromV1(const s_ble_settings_v1 &in, s_meshcom_settings &out);

// ---------------------------------------------------------------------------
// Wire-image validation -- pulled out of settings_rx_callback() (nrf52_ble.cpp)
// so the length (":339") and marker (":344-348") contracts are host-testable
// without the BLE stack. settings_rx_callback() must check both, IN THIS
// ORDER, on the raw bytes BEFORE anything is staged.
// ---------------------------------------------------------------------------

/** True iff `len` matches the frozen v1 wire image's length. */
bool bleSettingsV1LengthOk(size_t len);

/** True iff both frozen marker bytes are present in `img`. */
bool bleSettingsV1MarkersOk(const s_ble_settings_v1 &img);

#endif // BLE_SETTINGS_V1_H
