# Offene Buffer & Race Condition Fixes

**Bezug:** [audit-buffer-race-2026-03-17.md](audit-buffer-race-2026-03-17.md)
**Stand:** 2026-03-17

---

## Uebersicht

| Block | Findings | Aufwand |
|-------|----------|---------|
| 1. MHeard System | BUF-01 | Klein |
| 2. Webserver | BUF-04 | Mittel |
| 3. Command Functions | BUF-02, BUF-03 | Mittel |
| 4. sprintf-Haertung | BUF-08 | Klein |
| 5. T-Deck Hardware | PTR-01, PTR-02, PTR-03, RACE-09, RACE-10 | Mittel |
| 6. T5 / Display | BUF-05, BUF-06, BUF-07 | Mittel |
| 7. Race Condition | RACE-06 | Mittel |

---

## Block 1: MHeard System

### BUF-01 — mheard_functions.cpp:471 — memcpy ohne Laengenbegrenzung

**Aktuell (fehlerhaft):**
```cpp
memcpy(mheardPathCalls[ipos], mheardLine.mh_sourcecallsign.c_str(),
       mheardLine.mh_sourcecallsign.length());
mheardPathCalls[ipos][9] = 0x00;
```

**Fix:**
```cpp
size_t len = min(mheardLine.mh_sourcecallsign.length(),
                 sizeof(mheardPathCalls[ipos]) - 1);
memcpy(mheardPathCalls[ipos], mheardLine.mh_sourcecallsign.c_str(), len);
mheardPathCalls[ipos][len] = 0x00;
```

**Begruendung:** Identisches Pattern wie Zeile 268-269 (`mheardCalls`), das bereits korrekt
implementiert ist. Gleiche Absicherung fuer `mheardPathCalls` uebernehmen.

---

## Block 2: Webserver

### BUF-04 — web_commonServer.h:23-24, 40-41 — Use-after-free

**Aktuell (fehlerhaft):**
```cpp
CommonWebClient available() {
    WiFiClient c = WiFiServer::available();
    CommonWebClient* cwc = static_cast<CommonWebClient*>(&c);
    return *cwc;
}
```

**Fix — Variante A (einfachster Fix):**
```cpp
CommonWebClient available() {
    return WiFiServer::available();
}
```

Dies funktioniert wenn `CommonWebClient` von `WiFiClient` erbt und einen impliziten
Konvertierungskonstruktor oder Slicing-Copy unterstuetzt. Muss geprueft werden ob
`CommonWebClient` zusaetzliche Member hat.

**Fix — Variante B (explizite Konstruktion):**
```cpp
CommonWebClient available() {
    WiFiClient c = WiFiServer::available();
    return CommonWebClient(c);
}
```

**Analoger Fix fuer den Ethernet-Branch (Zeile 38-42).**

**Pruefpunkt vor Implementierung:** Klassendeklaration von `CommonWebClient` pruefen —
erbt sie direkt von `WiFiClient`/`EthernetClient`? Hat sie eigene Member? Davon haengt ab
welche Variante korrekt ist.

---

## Block 3: Command Functions

### BUF-02 — command_functions.cpp:4006, 4053, 4156, 4242 — JSON-Overflow

**Aktuell (fehlerhaft, 4 Stellen):**
```cpp
memset(print_buff, 0, sizeof(print_buff));
serializeJson(tmdoc, print_buff, measureJson(tmdoc));

memset(msg_buffer, 0, sizeof(msg_buffer));
msg_buffer[0] = 0x44;
memcpy(msg_buffer + 1, print_buff, strlen(print_buff));
```

**Fix:**
```cpp
memset(print_buff, 0, sizeof(print_buff));
serializeJson(tmdoc, print_buff, measureJson(tmdoc));

size_t json_len = strlen(print_buff);
if (json_len > MAX_MSG_LEN_PHONE - 2) {
    json_len = MAX_MSG_LEN_PHONE - 2;  // 1 Byte Header + Null-Terminator
}

memset(msg_buffer, 0, sizeof(msg_buffer));
msg_buffer[0] = 0x44;
memcpy(msg_buffer + 1, print_buff, json_len);
```

**Betroffene Stellen:**
- Zeile 4006: Telemetrie-JSON (`0x44`)
- Zeile 4053: Wetter-JSON (`0x44`)
- Zeile 4156: IO-JSON (`0x44`)
- Zeile 4242: Status-JSON (`0x44`)

**Hinweis:** Alternativ koennte `serializeJson()` direkt mit `MAX_MSG_LEN_PHONE - 2` als
Groesse begrenzt werden. Das waere sauberer, da die Abschneidung dann im JSON-Serializer
passiert und kein ungueltiges JSON entsteht:

```cpp
size_t json_len = serializeJson(tmdoc, print_buff, min(measureJson(tmdoc), (size_t)(MAX_MSG_LEN_PHONE - 2)));
```

---

### BUF-03 — command_functions.cpp:155 — memcpy msg_detail unbegrenzt

**Aktuell (fehlerhaft):**
```cpp
memcpy(msg_detail, msg_text + inext, strlen(msg_text) - inext);
```

**Fix:**
```cpp
size_t detail_len = strlen(msg_text) - inext;
if (detail_len > sizeof(msg_detail) - 1)
    detail_len = sizeof(msg_detail) - 1;
memcpy(msg_detail, msg_text + inext, detail_len);
msg_detail[detail_len] = '\0';
```

---

## Block 4: sprintf-Haertung

### BUF-08 — command_functions.cpp:1907 — sprintf in node_gwsrv[3]

**Aktuell (fragil):**
```cpp
sprintf(meshcom_settings.node_gwsrv, "%s", strCtry.c_str());
```

**Fix:**
```cpp
snprintf(meshcom_settings.node_gwsrv, sizeof(meshcom_settings.node_gwsrv), "%s", strCtry.c_str());
```

**Begruendung:** Funktional identisch, aber zukunftssicher falls sich die Validierungslogik
aendert oder neue Laendercodes hinzukommen.

---

## Block 5: T-Deck Hardware

### PTR-01 — tdeck_pro.cpp:195-198 — ps_calloc ohne NULL-Check

**Fix:**
```cpp
lv_color_t *buf_1 = (lv_color_t *)ps_calloc(sizeof(lv_color_t), DISP_BUF_SIZE);
lv_color_t *buf_2 = (lv_color_t *)ps_calloc(sizeof(lv_color_t), DISP_BUF_SIZE);
decodebuffer = (uint8_t *)ps_calloc(sizeof(uint8_t), DISP_BUF_SIZE);

if (!buf_1 || !buf_2 || !decodebuffer) {
    Serial.println("[INIT] FATAL: Display buffer allocation failed!");
    delay(3000);
    ESP.restart();
}

lv_disp_draw_buf_init(&draw_buf_dsc_1, buf_1, buf_2, LCD_HOR_SIZE * LCD_VER_SIZE);
```

### PTR-02 — tdeck_main.cpp:318 — ps_malloc ohne NULL-Check

**Fix:**
```cpp
static lv_color_t *buf = (lv_color_t *)ps_malloc(LVGL_BUFFER_SIZE);
if (!buf) {
    Serial.println("[INIT] FATAL: LVGL buffer allocation failed!");
    delay(3000);
    ESP.restart();
}
```

### PTR-03 — lv_obj_functions.cpp:2881, 2903 — new ohne NULL-Check

**Fix:**
```cpp
HeaderEventData *hed = new HeaderEventData();
if (!hed) return;  // Graceful bail-out
```

```cpp
DeleteEventData *ded = new DeleteEventData();
if (!ded) return;  // Graceful bail-out
```

### RACE-09 — SPI-Bus ohne gemeinsamen Mutex

**Analyse noetig:** Pruefen ob das Arduino-SPI-Framework auf ESP32-S3 bereits intern
einen Mutex verwendet (`SPI.beginTransaction()`). Falls nicht:

**Konzept:**
```cpp
extern SemaphoreHandle_t spiMutex;

// Vor jedem SPI-Zugriff (Display, SD, Radio):
if (xSemaphoreTake(spiMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    // SPI-Operation
    xSemaphoreGive(spiMutex);
}
```

**Vorsicht:** Hoher Aufwand, da SPI-Zugriffe ueber mehrere Abstraktionsschichten laufen
(TFT_eSPI, SD-Library, RadioLib). Moeglicherweise besser ueber CS-Pin-Management loesbar.

### RACE-10 — Audio-Task umgeht Display-Semaphore

**Fix-Konzept:** Audio-Task muss vor Hardware-Zugriff das Display-Semaphore holen:

```cpp
// In esp32_audio.cpp, vor Audio-Playback:
if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(500)) == pdTRUE) {
    audio.connecttoFS(SD, filename);
    xSemaphoreGive(xSemaphore);
}
```

---

## Block 6: T5 / Display

### BUF-05 — ui.cpp:1242-1244 — Out-of-Bounds Write

**Aktuell (fehlerhaft):**
```cpp
char buf[16] = {0};
strncpy(buf, text, 16);
buf[text_len - 4] = '\0';
```

**Fix:**
```cpp
char buf[16] = {0};
int cut = (text_len > 4) ? min(text_len - 4, (int)sizeof(buf) - 1) : 0;
strncpy(buf, text, sizeof(buf) - 1);
buf[cut] = '\0';
```

### BUF-06 — ui.cpp:74-80, ui_deckpro.cpp:79-88 — line_full_format

**Aktuell (fehlerhaft):**
```cpp
strncpy(global_buf, str1, len1);
// ... Padding ...
strncpy(global_buf + j, str2, len2);
```

**Fix (t5-epaper/ui.cpp):**
```cpp
int buf_size = sizeof(global_buf);  // 256
if (len1 >= buf_size) len1 = buf_size - 1;
strncpy(global_buf, str1, len1);
// ... Padding mit max_c Begrenzung ...
int remaining = buf_size - j - 1;
if (len2 > remaining) len2 = remaining;
strncpy(global_buf + j, str2, len2);
global_buf[buf_size - 1] = '\0';
```

**Fix (t-deck-pro/ui_deckpro.cpp) — gleiches Pattern mit GLOBAL_BUF_LEN (30).**

### BUF-07 — ui_deckpro_port.cpp:223 — WiFi SSID ohne Null-Terminator

**Fix:**
```cpp
snprintf(list[i].name, sizeof(list[i].name), "%s", WiFi.SSID(i).c_str());
```

---

## Block 7: Race Condition (offen)

### RACE-06 — meshcom_settings ohne Mutex

**Einschaetzung:** Niedrigere Prioritaet, da die meisten Felder nur selten geschrieben werden
(Sensor-Updates alle paar Sekunden). Trotzdem sollten Float-Writes (temp, hum, press) mit
einem leichtgewichtigen Mutex geschuetzt werden:

```cpp
extern SemaphoreHandle_t settingsMutex;

// Beim Schreiben (bme680.cpp etc.):
if (xSemaphoreTake(settingsMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    meshcom_settings.node_temp = bme.temperature + meshcom_settings.node_tempi_off;
    meshcom_settings.node_hum = bme.humidity;
    meshcom_settings.node_press = bme.pressure / 100.0;
    xSemaphoreGive(settingsMutex);
}
```

---

## Priorisierung

### Prioritaet 1 — Sofort (Absturzrisiko)
1. **BUF-01** mheard memcpy — 5 Min, 1 Zeile
2. **BUF-02** JSON-Overflow in command_functions — 30 Min, 4 Stellen
3. **BUF-03** msg_detail Overflow — 10 Min, 1 Stelle
4. **BUF-05** ui.cpp Out-of-Bounds Write — 10 Min, 1 Stelle

### Prioritaet 2 — Kurzfristig (Datenkorruption)
5. **BUF-04** web_commonServer Use-after-free — 30 Min, Klasse pruefen

### Prioritaet 3 — Mittelfristig (Haertung)
6. **BUF-06** line_full_format Bounds — 30 Min, 2 Dateien
7. **BUF-07** WiFi SSID Null-Terminator — 5 Min
8. **PTR-01/02** ps_calloc/ps_malloc NULL-Checks — 15 Min

### Prioritaet 4 — Langfristig (Defense in Depth)
9. **BUF-08** sprintf → snprintf Haertung — 5 Min
10. **PTR-03** new NULL-Checks — 10 Min
11. **RACE-06** meshcom_settings Mutex — 1 Std
12. **RACE-09/10** SPI-Mutex, Audio-Semaphore — 2 Std, Analyse noetig
