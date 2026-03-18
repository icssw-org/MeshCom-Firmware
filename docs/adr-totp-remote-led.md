# ADR: TOTP-basierte Remote-LED-Steuerung via MeshCom-Textnachricht

**Status:** Proposed
**Datum:** 2026-03-15
**Autor:** Martin DK5EN

---

## Kontext

Ein Heltec V3 Node soll per MeshCom-Textnachricht aus der Ferne einen GPIO schalten können (Demonstration: weiße LED auf GPIO 35). Die Authentifizierung erfolgt über TOTP (Time-based One-Time Password, RFC 6238), damit nicht jeder beliebige Node den Pin steuern kann.

## Entscheidung

### 1. Authentifizierung: TOTP (RFC 6238)

- **Shared Secret**: 16-Byte-Schlüssel, Base32-kodiert (160 Bit), wird im Webserver konfiguriert
- **Algorithmus**: HMAC-SHA1, 6-stelliger Code, 30-Sekunden-Zeitfenster
- **Toleranz**: ±1 Zeitschritt (aktueller + vorheriger + nächster Code werden akzeptiert → effektiv ±30s)
- **Zeitquelle**: Vorhandene NTP-Synchronisation der Firmware
- **Kompatibilität**: Standard-Authenticator-Apps (Google Authenticator, Authy, etc.)

**Library**: [TOTP-Arduino](https://github.com/lucadentella/TOTP-Arduino) von Luca Dentella — kompakt (~2 Dateien), RFC 6238-konform, bewährt auf ESP32.

### 2. Nachrichtenformat

Eingehende MeshCom-Textnachrichten werden auf folgendes Muster geprüft:

```
TOTP:<6-stelliger-code>:<befehl>
```

**Befehle (Phase 1):**

| Befehl | Wirkung |
|--------|---------|
| `ON`   | GPIO auf HIGH setzen |
| `OFF`  | GPIO auf LOW setzen |

**Beispiele:**
```
TOTP:482913:ON
TOTP:482913:OFF
```

### 3. Antwort-Nachricht

Der Node antwortet dem Absender mit einer **persönlichen Nachricht** (DM):

| Antwort | Bedeutung |
|---------|-----------|
| `TOTP ACK ON`  | Befehl erkannt, GPIO eingeschaltet |
| `TOTP ACK OFF` | Befehl erkannt, GPIO ausgeschaltet |
| `TOTP NACK`    | Authentifizierung fehlgeschlagen oder ungültiger Befehl |

### 4. Nachrichtenempfang

**Phase 1**: Node reagiert auf alle Textnachrichten (Broadcast `*`, Gruppe `999`, persönliche Nachricht) — einfachster Einstieg.

**Phase 2 (zukünftig)**: Nur persönliche Nachrichten werden verarbeitet.

### 5. Konfiguration via Webserver

Auf der Setup-Seite des Webservers werden zwei neue Felder ergänzt:

| Feld | Typ | Beschreibung | Default |
|------|-----|-------------|---------|
| TOTP Secret | Text (Base32, 16 Zeichen) | Shared Secret für TOTP | leer (Feature deaktiviert) |
| TOTP GPIO Pin | Zahl | GPIO-Pin der geschaltet wird | 35 |

- **QR-Code-Pairing**: Die Webseite generiert clientseitig (JavaScript, ~3KB inline) einen QR-Code im `otpauth://totp/MeshCom:<callsign>?secret=<base32>&algorithm=SHA1&digits=6&period=30` Format. Kein Firmware-Overhead — der ESP32 liefert nur die URI, der Browser rendert den QR-Code.
- **Wenn Secret leer**: Feature ist deaktiviert, keine TOTP-Auswertung
- **"Generate"-Button**: Erzeugt clientseitig ein zufälliges 16-Byte-Secret (Base32-kodiert), zeigt es an und generiert den QR-Code

### 6. Persistenz

Neue Felder in `meshcom_settings` (Struct in `esp32_flash.h`):

```cpp
char node_totp_secret[33] = "";   // Base32-kodiertes Secret (bis 32 Zeichen + \0)
int  node_totp_gpio = 35;         // GPIO-Pin für TOTP-Schaltung
```

NVS-Keys: `"node_totps"` (Secret), `"node_totpg"` (GPIO)

Gespeichert/geladen in `save_settings()` / `init_flash()` wie alle anderen Settings.

### 7. Firmware-Architektur

```
Textnachricht empfangen (lora_functions.cpp / loop_functions.cpp)
  │
  ├─ Beginnt mit "TOTP:" ?
  │    ├─ Nein → normale Nachrichtenverarbeitung
  │    └─ Ja  → totp_handle_command()
  │              │
  │              ├─ Secret konfiguriert?
  │              │    └─ Nein → ignorieren
  │              │
  │              ├─ Parse: code + befehl
  │              │
  │              ├─ TOTP validieren (±1 Zeitschritt)
  │              │    └─ Ungültig → DM "TOTP NACK" an Absender
  │              │
  │              ├─ GPIO schalten (ON/OFF)
  │              │
  │              └─ DM "TOTP ACK ON/OFF" an Absender
```

**Neue Dateien:**
- `src/totp_functions.cpp` — TOTP-Validierung, Command-Handling, GPIO-Steuerung
- `src/totp_functions.h` — Header

**Bestehende Dateien (minimale Änderungen):**
- `esp32_flash.h` — 2 Felder in Struct
- `esp32_flash.cpp` — 2× get/put in init_flash()/save_settings()
- `web_functions.cpp` — UI-Elemente auf Setup-Seite (TOTP-Sektion)
- `web_setup.cpp` — Setter/Getter für die 2 neuen Parameter
- `loop_functions.cpp` oder `lora_functions.cpp` — Abfangen der TOTP-Nachricht im Empfangspfad
- `configuration.h` (Heltec V3) — `#define BOARD_LED 35`

### 8. Sicherheitsüberlegungen

| Risiko | Mitigation |
|--------|-----------|
| Secret im Klartext auf Webseite | Web-Passwort schützt Zugang; Secret nur bei Konfiguration sichtbar |
| Replay-Angriff (gleicher Code nochmal) | TOTP-Code ist nur 30s gültig; ±1 Fenster ist akzeptabler Kompromiss |
| Brute-Force (6 Stellen = 1M Kombinationen) | Rate-Limiting: max. 3 Fehlversuche pro 90s, danach 5 Min Sperre |
| NTP-Drift | ±1 Zeitschritt fängt moderate Drift ab; NTP wird regelmäßig gesynct |
| MeshCom-Nachrichten sind unverschlüsselt | TOTP-Code ist Einmal-Code — Mitlesen bringt nichts, da er nach 30s verfällt |

### 9. Abhängigkeiten

- **TOTP-Arduino Library** (oder Minimal-Implementierung von HMAC-SHA1 + TOTP)
- **NTP-Zeit muss synchronisiert sein** — Feature sollte erst aktiv werden, wenn Uhrzeit vorhanden
- **QR-Code JS-Library**: z.B. [qrcode.js](https://github.com/davidshimjs/qrcodejs) (~3KB), inline im HTML

### 10. Aufwandsschätzung (Dateien/Änderungen)

| Bereich | Dateien | Umfang |
|---------|---------|--------|
| TOTP-Library einbinden | lib/ | ~3 Dateien (extern) |
| TOTP Command Handler | src/totp_functions.cpp/h | ~150 Zeilen (neu) |
| Settings + Flash | esp32_flash.h/cpp | ~10 Zeilen (Erweiterung) |
| Webserver UI + QR | web_functions.cpp, web_setup.cpp | ~80 Zeilen (Erweiterung) |
| Nachrichtenempfang Hook | loop_functions.cpp oder lora_functions.cpp | ~10 Zeilen (Erweiterung) |
| Heltec V3 Board-LED | configuration.h | 1 Zeile |

---

## Alternativen (verworfen)

1. **One-Time Pad**: Erfordert vorgenerierte Schlüssellisten gleicher Länge wie die Nachrichten — unpraktisch für Embedded
2. **Statisches Passwort**: Replay-Angriff trivial, da MeshCom unverschlüsselt
3. **HOTP (Counter-based)**: Counter-Sync-Probleme wenn Nachrichten verloren gehen — TOTP mit Zeitsync ist robuster
4. **QR-Code auf ESP32 rendern**: Unnötiger Firmware-Overhead — clientseitiges JS ist kostenlos

## Offene Punkte

- [ ] TOTP-Arduino Library evaluieren: direkt einbinden oder nur HMAC-SHA1 Core extrahieren?
- [ ] Prüfen wo genau im Empfangspfad die TOTP-Nachricht am besten abgefangen wird
- [ ] Klären ob `commandAction()`-Pattern für TOTP-Befehle verwendet werden soll
- [ ] Rate-Limiting Implementierungsstrategie (einfacher Counter + Timestamp reicht)
