/**
 * meshcom_settings_runtime.h -- the members of s_meshcom_settings that are
 * deliberately NOT persisted, named one by one.
 *
 * This list is the other half of the member-level gate
 * (test/test_settings_members, D1-04 W3 step 3): every member the X-macro
 * lists in meshcom_settings.h produce must either be covered by a
 * settings_schema descriptor or appear here. Neither list is derived from
 * the other, so adding a member to the struct without deciding which of the
 * two it belongs to fails the test. A member listed here that ALSO has a
 * schema row fails too -- the two sets are disjoint by construction.
 *
 * R(name) -- one member name per row.
 */
#ifndef MESHCOM_SETTINGS_RUNTIME_H
#define MESHCOM_SETTINGS_RUNTIME_H

#define MESHCOM_SETTINGS_RUNTIME_COMMON(R)                                                  \
    R(valid_mark_1)   /* legacy blob marker */                                              \
    R(valid_mark_2)                                                                         \
    R(node_device_eui) /* rebuilt from the MAC on every boot (nrf52_main.cpp) */            \
    R(node_date_year)  /* wall clock, set from GPS/NTP */                                   \
    R(node_date_month)                                                                      \
    R(node_date_day)                                                                        \
    R(node_date_hour)                                                                       \
    R(node_date_minute)                                                                     \
    R(node_date_second)                                                                     \
    R(node_date_hundredths)                                                                 \
    R(node_age)                                                                             \
    R(node_temp)    /* last sensor readings -- operator decision 2026-09-13: not persisted */ \
    R(node_hum)                                                                             \
    R(node_press)                                                                           \
    R(node_temp2)                                                                           \
    R(node_gas_res)                                                                         \
    R(node_co2)                                                                             \
    R(node_msgid)   /* a counter, persisted by counters_store.h, never by the settings store */ \
    R(node_press_alt)                                                                       \
    R(node_press_asl)                                                                       \
    R(node_vbus)                                                                            \
    R(node_vshunt)                                                                          \
    R(node_vcurrent)                                                                        \
    R(node_vpower)                                                                          \
    R(node_ip)                                                                              \
    R(node_dns)                                                                             \
    R(node_gw)                                                                              \
    R(node_subnet)                                                                          \
    R(node_hasIPaddress)                                                                    \
    R(node_last_upd_timer)                                                                  \
    R(max_hop_pos)                                                                          \
    R(node_update)                                                                          \
    R(node_parm_1)                                                                          \
    R(node_parm_t)                                                                          \
    R(node_parm_id)                                                                         \
    R(node_ntctemp)                                                                         \
    R(node_fanon)                                                                           \
    R(node_pingcount)                                                                       \
    R(node_pingduration)

#ifdef ESP32
    #define MESHCOM_SETTINGS_RUNTIME_PLATFORM(R) \
        R(node_ntp) /* recopied from node_ownntp on every connect (udp_functions.cpp) */
#else
    /* On nRF52 the spectrum-scan window and the battery factor have no key
     * (CFG_FIELD_LIST_PLATFORM is ESP32-only for them); they sat below the
     * old header's "nicht im Flash" line. */
    #define MESHCOM_SETTINGS_RUNTIME_PLATFORM(R) \
        R(node_specstart)                        \
        R(node_specend)                          \
        R(node_specstep)                         \
        R(node_specsamples)                      \
        R(node_analog_batt_faktor)
#endif

#define MESHCOM_SETTINGS_RUNTIME(R)      \
    MESHCOM_SETTINGS_RUNTIME_COMMON(R)   \
    MESHCOM_SETTINGS_RUNTIME_PLATFORM(R)

#endif // MESHCOM_SETTINGS_RUNTIME_H
