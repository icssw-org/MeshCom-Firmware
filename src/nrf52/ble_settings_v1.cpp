/**
 * @file ble_settings_v1.cpp
 * @brief Member-by-member conversion between the live settings struct and
 *        the frozen v1 BLE wire format. See ble_settings_v1.h for why this
 *        exists.
 *
 * Every member is copied explicitly, in both directions, rather than a
 * single memcpy/blit of the whole struct -- that is the whole point: once
 * s_meshcom_settings' layout changes (a later wave), only these two
 * functions need to change, never the wire format s_ble_settings_v1
 * describes.
 *
 * send_repeat_time and auto_join (in s_meshcom_settings, no NVS key of
 * their own -- see src/config_json.h's CFG_FIELD_LIST_PLATFORM comment)
 * are the two members the next wave plans to drop from s_meshcom_settings.
 * They still map straight through here for now (search "v1-only carryover"
 * below): when that wave lands, the fix is to delete exactly those two
 * lines from each function -- the v1 struct itself must NOT change.
 */
#include "ble_settings_v1.h"

#include <cstring>

void bleSettingsToV1(const s_meshcom_settings &src, s_ble_settings_v1 &out)
{
	out.valid_mark_1 = src.valid_mark_1;
	out.valid_mark_2 = src.valid_mark_2;
	memcpy(out.node_device_eui, src.node_device_eui, sizeof(out.node_device_eui));

	memcpy(out.node_call, src.node_call, sizeof(out.node_call));
	memcpy(out.node_short, src.node_short, sizeof(out.node_short));

	out.node_lat = src.node_lat;
	out.node_lat_c = src.node_lat_c;
	out.node_lon = src.node_lon;
	out.node_lon_c = src.node_lon_c;
	out.node_alt = src.node_alt;
	out.node_symid = src.node_symid;
	out.node_symcd = src.node_symcd;

	out.node_date_year = src.node_date_year;
	out.node_date_month = src.node_date_month;
	out.node_date_day = src.node_date_day;

	out.node_date_hour = src.node_date_hour;
	out.node_date_minute = src.node_date_minute;
	out.node_date_second = src.node_date_second;
	out.node_date_hundredths = src.node_date_hundredths;

	out.node_age = src.node_age;

	out.node_temp = src.node_temp;
	out.node_hum = src.node_hum;
	out.node_press = src.node_press;

	memcpy(out.node_ossid, src.node_ossid, sizeof(out.node_ossid));
	memcpy(out.node_opwd, src.node_opwd, sizeof(out.node_opwd));

	// v1-only members: send_repeat_time / auto_join (dead LoRaWAN OTAA state)
	// and node_ackid (never read) were removed from s_meshcom_settings in the
	// D1-04 struct merge. The wire image keeps their slots at their frozen
	// defaults; s_ble_settings_v1 itself is never touched.
	out.send_repeat_time = 0;
	out.auto_join = false;

	out.node_hamnet_only = src.node_hamnet_only;

	out.node_sset = src.node_sset;

	out.node_maxv = src.node_maxv;

	memcpy(out.node_extern, src.node_extern, sizeof(out.node_extern));

	out.node_msgid = src.node_msgid;
	out.node_ackid = 0;

	out.node_power = src.node_power;
	out.node_freq = src.node_freq;
	out.node_bw = src.node_bw;
	out.node_sf = src.node_sf;
	out.node_cr = src.node_cr;

	memcpy(out.node_atxt, src.node_atxt, sizeof(out.node_atxt));

	out.node_sset2 = src.node_sset2;
	out.node_owgpio = src.node_owgpio;

	out.node_temp2 = src.node_temp2;

	out.node_utcoff = src.node_utcoff;

	out.node_gas_res = src.node_gas_res;

	out.node_co2 = src.node_co2;

	out.node_mcp17io = src.node_mcp17io;
	memcpy(out.node_mcp17t, src.node_mcp17t, sizeof(out.node_mcp17t));
	out.node_mcp17out = src.node_mcp17out;
	out.node_mcp17in = src.node_mcp17in;

	memcpy(out.node_gcb, src.node_gcb, sizeof(out.node_gcb));

	out.node_country = src.node_country;

	out.node_track_freq = src.node_track_freq;
	out.node_preamplebits = src.node_preamplebits;

	out.node_ss_rx_pin = src.node_ss_rx_pin;
	out.node_ss_tx_pin = src.node_ss_tx_pin;
	out.node_ss_baud = src.node_ss_baud;

	out.node_postime = src.node_postime;

	memcpy(out.node_passwd, src.node_passwd, sizeof(out.node_passwd));

	out.node_sset3 = src.node_sset3;

	out.bt_code = src.bt_code;

	out.node_button_pin = src.node_button_pin;

	memcpy(out.node_ownip, src.node_ownip, sizeof(out.node_ownip));
	memcpy(out.node_owngw, src.node_owngw, sizeof(out.node_owngw));
	memcpy(out.node_ownms, src.node_ownms, sizeof(out.node_ownms));

	memcpy(out.node_name, src.node_name, sizeof(out.node_name));

	memcpy(out.node_webpwd, src.node_webpwd, sizeof(out.node_webpwd));

	memcpy(out.node_ssid, src.node_ssid, sizeof(out.node_ssid));
	memcpy(out.node_pwd, src.node_pwd, sizeof(out.node_pwd));

	out.node_analog_pin = src.node_analog_pin;
	out.node_analog_faktor = src.node_analog_faktor;

	memcpy(out.node_parm, src.node_parm, sizeof(out.node_parm));
	memcpy(out.node_unit, src.node_unit, sizeof(out.node_unit));
	memcpy(out.node_format, src.node_format, sizeof(out.node_format));
	memcpy(out.node_eqns, src.node_eqns, sizeof(out.node_eqns));
	memcpy(out.node_values, src.node_values, sizeof(out.node_values));

	out.node_parm_time = src.node_parm_time;

	out.node_wifi_power = src.node_wifi_power;
	memcpy(out.node_lora_call, src.node_lora_call, sizeof(out.node_lora_call));

	out.node_analog_alpha = src.node_analog_alpha;
	out.node_analog_slope = src.node_analog_slope;
	out.node_analog_offset = src.node_analog_offset;
	out.node_analog_atten = src.node_analog_atten;

	memcpy(out.node_gwsrv, src.node_gwsrv, sizeof(out.node_gwsrv));

	out.node_tempi_off = src.node_tempi_off;
	out.node_tempo_off = src.node_tempo_off;

	out.node_shunt = src.node_shunt;
	out.node_imax = src.node_imax;
	out.node_isamp = src.node_isamp;

	memcpy(out.node_owndns, src.node_owndns, sizeof(out.node_owndns));

	out.node_contrast = src.node_contrast;
	out.node_fversion = src.node_fversion;

	memcpy(out.node_ownntp, src.node_ownntp, sizeof(out.node_ownntp));

	out.node_mversion = src.node_mversion;
	memcpy(out.node_fwversion, src.node_fwversion, sizeof(out.node_fwversion));

	out.node_gpsbaud = src.node_gpsbaud;

	out.node_cleanflash = src.node_cleanflash;

	out.node_netmode = src.node_netmode;

	out.node_gpsdebug = src.node_gpsdebug;

	out.node_relay = src.node_relay;

	memcpy(out.node_via, src.node_via, sizeof(out.node_via));

	out.node_sset4 = src.node_sset4;

	memcpy(out.node_aprsmc, src.node_aprsmc, sizeof(out.node_aprsmc));

	out.node_pingtime = src.node_pingtime;
	memcpy(out.node_pingcall, src.node_pingcall, sizeof(out.node_pingcall));
	out.node_pingmax = src.node_pingmax;

	out.node_specstart = src.node_specstart;
	out.node_specend = src.node_specend;
	out.node_specstep = src.node_specstep;
	out.node_specsamples = src.node_specsamples;

	out.node_analog_batt_faktor = src.node_analog_batt_faktor;

	out.node_press_alt = src.node_press_alt;
	out.node_press_asl = src.node_press_asl;
	out.node_vbus = src.node_vbus;
	out.node_vshunt = src.node_vshunt;
	out.node_vcurrent = src.node_vcurrent;
	out.node_vpower = src.node_vpower;

	memcpy(out.node_ip, src.node_ip, sizeof(out.node_ip));
	memcpy(out.node_dns, src.node_dns, sizeof(out.node_dns));
	memcpy(out.node_gw, src.node_gw, sizeof(out.node_gw));
	memcpy(out.node_subnet, src.node_subnet, sizeof(out.node_subnet));
	out.node_hasIPaddress = src.node_hasIPaddress;
	out.node_last_upd_timer = src.node_last_upd_timer;

	out.max_hop_text = src.max_hop_text;
	out.max_hop_pos = src.max_hop_pos;

	memcpy(out.node_update, src.node_update, sizeof(out.node_update));

	memcpy(out.node_parm_1, src.node_parm_1, sizeof(out.node_parm_1));
	memcpy(out.node_parm_t, src.node_parm_t, sizeof(out.node_parm_t));
	memcpy(out.node_parm_id, src.node_parm_id, sizeof(out.node_parm_id));

	out.node_ntctemp = src.node_ntctemp;
	out.node_fanon = src.node_fanon;

	out.node_pingcount = src.node_pingcount;
	out.node_pingduration = src.node_pingduration;
}

void bleSettingsFromV1(const s_ble_settings_v1 &in, s_meshcom_settings &out)
{
	out.valid_mark_1 = in.valid_mark_1;
	out.valid_mark_2 = in.valid_mark_2;
	memcpy(out.node_device_eui, in.node_device_eui, sizeof(out.node_device_eui));

	memcpy(out.node_call, in.node_call, sizeof(out.node_call));
	memcpy(out.node_short, in.node_short, sizeof(out.node_short));

	out.node_lat = in.node_lat;
	out.node_lat_c = in.node_lat_c;
	out.node_lon = in.node_lon;
	out.node_lon_c = in.node_lon_c;
	out.node_alt = in.node_alt;
	out.node_symid = in.node_symid;
	out.node_symcd = in.node_symcd;

	out.node_date_year = in.node_date_year;
	out.node_date_month = in.node_date_month;
	out.node_date_day = in.node_date_day;

	out.node_date_hour = in.node_date_hour;
	out.node_date_minute = in.node_date_minute;
	out.node_date_second = in.node_date_second;
	out.node_date_hundredths = in.node_date_hundredths;

	out.node_age = in.node_age;

	out.node_temp = in.node_temp;
	out.node_hum = in.node_hum;
	out.node_press = in.node_press;

	memcpy(out.node_ossid, in.node_ossid, sizeof(out.node_ossid));
	memcpy(out.node_opwd, in.node_opwd, sizeof(out.node_opwd));

	// v1-only members (send_repeat_time, auto_join, node_ackid): no longer in
	// s_meshcom_settings, so an inbound image's values for them are ignored.

	out.node_hamnet_only = in.node_hamnet_only;

	out.node_sset = in.node_sset;

	out.node_maxv = in.node_maxv;

	memcpy(out.node_extern, in.node_extern, sizeof(out.node_extern));

	// node_msgid is deliberately NOT copied inbound (advisor finding 1, W3c):
	// the app writes back the image it read earlier, and applying its stale
	// counter would hand out message ids a second time. The counter is state
	// behind counters_store.h, never configuration. Outbound (bleSettingsToV1)
	// still exports it, so the app sees the live value. The legacy-blob path in
	// nrf52_flash.cpp copies it explicitly, because there the blob IS the only
	// source of the counter.

	out.node_power = in.node_power;
	out.node_freq = in.node_freq;
	out.node_bw = in.node_bw;
	out.node_sf = in.node_sf;
	out.node_cr = in.node_cr;

	memcpy(out.node_atxt, in.node_atxt, sizeof(out.node_atxt));

	out.node_sset2 = in.node_sset2;
	out.node_owgpio = in.node_owgpio;

	out.node_temp2 = in.node_temp2;

	out.node_utcoff = in.node_utcoff;

	out.node_gas_res = in.node_gas_res;

	out.node_co2 = in.node_co2;

	out.node_mcp17io = in.node_mcp17io;
	memcpy(out.node_mcp17t, in.node_mcp17t, sizeof(out.node_mcp17t));
	out.node_mcp17out = in.node_mcp17out;
	out.node_mcp17in = in.node_mcp17in;

	memcpy(out.node_gcb, in.node_gcb, sizeof(out.node_gcb));

	out.node_country = in.node_country;

	out.node_track_freq = in.node_track_freq;
	out.node_preamplebits = in.node_preamplebits;

	out.node_ss_rx_pin = in.node_ss_rx_pin;
	out.node_ss_tx_pin = in.node_ss_tx_pin;
	out.node_ss_baud = in.node_ss_baud;

	out.node_postime = in.node_postime;

	memcpy(out.node_passwd, in.node_passwd, sizeof(out.node_passwd));

	out.node_sset3 = in.node_sset3;

	out.bt_code = in.bt_code;

	out.node_button_pin = in.node_button_pin;

	memcpy(out.node_ownip, in.node_ownip, sizeof(out.node_ownip));
	memcpy(out.node_owngw, in.node_owngw, sizeof(out.node_owngw));
	memcpy(out.node_ownms, in.node_ownms, sizeof(out.node_ownms));

	memcpy(out.node_name, in.node_name, sizeof(out.node_name));

	memcpy(out.node_webpwd, in.node_webpwd, sizeof(out.node_webpwd));

	memcpy(out.node_ssid, in.node_ssid, sizeof(out.node_ssid));
	memcpy(out.node_pwd, in.node_pwd, sizeof(out.node_pwd));

	out.node_analog_pin = in.node_analog_pin;
	out.node_analog_faktor = in.node_analog_faktor;

	memcpy(out.node_parm, in.node_parm, sizeof(out.node_parm));
	memcpy(out.node_unit, in.node_unit, sizeof(out.node_unit));
	memcpy(out.node_format, in.node_format, sizeof(out.node_format));
	memcpy(out.node_eqns, in.node_eqns, sizeof(out.node_eqns));
	memcpy(out.node_values, in.node_values, sizeof(out.node_values));

	out.node_parm_time = in.node_parm_time;

	out.node_wifi_power = in.node_wifi_power;
	memcpy(out.node_lora_call, in.node_lora_call, sizeof(out.node_lora_call));

	out.node_analog_alpha = in.node_analog_alpha;
	out.node_analog_slope = in.node_analog_slope;
	out.node_analog_offset = in.node_analog_offset;
	out.node_analog_atten = in.node_analog_atten;

	memcpy(out.node_gwsrv, in.node_gwsrv, sizeof(out.node_gwsrv));

	out.node_tempi_off = in.node_tempi_off;
	out.node_tempo_off = in.node_tempo_off;

	out.node_shunt = in.node_shunt;
	out.node_imax = in.node_imax;
	out.node_isamp = in.node_isamp;

	memcpy(out.node_owndns, in.node_owndns, sizeof(out.node_owndns));

	out.node_contrast = in.node_contrast;
	out.node_fversion = in.node_fversion;

	memcpy(out.node_ownntp, in.node_ownntp, sizeof(out.node_ownntp));

	out.node_mversion = in.node_mversion;
	memcpy(out.node_fwversion, in.node_fwversion, sizeof(out.node_fwversion));

	out.node_gpsbaud = in.node_gpsbaud;

	out.node_cleanflash = in.node_cleanflash;

	out.node_netmode = in.node_netmode;

	out.node_gpsdebug = in.node_gpsdebug;

	out.node_relay = in.node_relay;

	memcpy(out.node_via, in.node_via, sizeof(out.node_via));

	out.node_sset4 = in.node_sset4;

	memcpy(out.node_aprsmc, in.node_aprsmc, sizeof(out.node_aprsmc));

	out.node_pingtime = in.node_pingtime;
	memcpy(out.node_pingcall, in.node_pingcall, sizeof(out.node_pingcall));
	out.node_pingmax = in.node_pingmax;

	out.node_specstart = in.node_specstart;
	out.node_specend = in.node_specend;
	out.node_specstep = in.node_specstep;
	out.node_specsamples = in.node_specsamples;

	out.node_analog_batt_faktor = in.node_analog_batt_faktor;

	out.node_press_alt = in.node_press_alt;
	out.node_press_asl = in.node_press_asl;
	out.node_vbus = in.node_vbus;
	out.node_vshunt = in.node_vshunt;
	out.node_vcurrent = in.node_vcurrent;
	out.node_vpower = in.node_vpower;

	memcpy(out.node_ip, in.node_ip, sizeof(out.node_ip));
	memcpy(out.node_dns, in.node_dns, sizeof(out.node_dns));
	memcpy(out.node_gw, in.node_gw, sizeof(out.node_gw));
	memcpy(out.node_subnet, in.node_subnet, sizeof(out.node_subnet));
	out.node_hasIPaddress = in.node_hasIPaddress;
	out.node_last_upd_timer = in.node_last_upd_timer;

	out.max_hop_text = in.max_hop_text;
	out.max_hop_pos = in.max_hop_pos;

	memcpy(out.node_update, in.node_update, sizeof(out.node_update));

	memcpy(out.node_parm_1, in.node_parm_1, sizeof(out.node_parm_1));
	memcpy(out.node_parm_t, in.node_parm_t, sizeof(out.node_parm_t));
	memcpy(out.node_parm_id, in.node_parm_id, sizeof(out.node_parm_id));

	out.node_ntctemp = in.node_ntctemp;
	out.node_fanon = in.node_fanon;

	out.node_pingcount = in.node_pingcount;
	out.node_pingduration = in.node_pingduration;
}

bool bleSettingsV1LengthOk(size_t len)
{
	return len == sizeof(s_ble_settings_v1);
}

bool bleSettingsV1MarkersOk(const s_ble_settings_v1 &img)
{
	return (img.valid_mark_1 == BLE_SETTINGS_V1_MARK_1) && (img.valid_mark_2 == BLE_SETTINGS_V1_MARK_2);
}
