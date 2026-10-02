#ifndef _APRS_STRUCTURES_H_
#define _APRS_STRUCTURES_H_

#include <Arduino.h>

// APRS protokol

// R2-04: die Textfelder von aprsMessage sind feste char[] statt Arduino-String.
//
// WARUM. decodeAPRS() baut jedes dieser Felder ohnehin schon in einem lokalen
// `char cConcat1..3[UDP_TX_BUF_SIZE]` auf dem Stack zusammen und kopiert es
// danach in einen String -- und rund 110 Aufrufstellen holen sich mit
// `.c_str()` wieder einen `char*` heraus. Das ist eine Serialisierung mit
// anschliessender Deserialisierung fuer Daten, die die Struktur nie verlassen:
// dieselbe Form, die R2-01 aus dem alten MHeard-Puffer entfernt hat. Der Preis war
// rund sieben malloc/free je empfangenem Frame, auf einer Plattform, auf der
// Heap-Verkehr pro Zeile der dokumentierte Grund fuer scheiternden
// BLE-Verbindungsaufbau ist.
//
// DIE BREITEN sind aus dem Decoder abgeleitet, nicht geschaetzt -- eine zu
// knappe Breite wuerde echten Verkehr abschneiden, statt Bytes zu sparen:
//
//   MC_PATH_LEN    Die beiden Pfad-Schleifen in decodeAPRS() nehmen je genau
//                  120 Zeichen an (`(ib - 6) < 120` bzw.
//                  `(ib - inextstart) < 120`). 121 bildet das exakt ab, also
//                  bleibt jeder heute angenommene Frame angenommen.
//
//   MC_CALL_LEN_Z  MAX_CALL_LEN (20) ist die im Projekt erklaerte Obergrenze
//                  fuer ein Rufzeichen, +1 fuer die NUL. Das ist die EINZIGE
//                  Stelle, an der sich das Verhalten aendert: ein "Rufzeichen"
//                  laenger als 20 Zeichen wird jetzt beim Zerlegen verworfen
//                  statt erst danach von checkRegexCall(). Beide Wege
//                  verwerfen den Frame; nur der Zeitpunkt ist frueher.
//
//   MC_PAYLOAD_LEN Die Nutzlast-Schleife hat KEINE eigene Schranke; sie laeuft
//                  bis `rsize`. Der 340-Byte-Test darueber
//                  (MAX_APRS_FRAME_SIZE) kann nicht ausloesen: jeder Aufrufer
//                  ist auf UDP_TX_BUF_SIZE (255) begrenzt -- LoRa durch
//                  rxPayloadCopy[2][UDP_TX_BUF_SIZE], BLE durch ein uint8_t
//                  als Laenge, UDP durch die Ringpuffer-Slots (R1-06 hat das
//                  bereits nachgewiesen). 256 bildet diese tatsaechliche
//                  Schranke ab, und die Schleife prueft sie jetzt SELBST,
//                  statt sich auf ihre Aufrufer zu verlassen.
//
// DIESE DATEI ZIEHT ABSICHTLICH configuration_global.h NICHT HEREIN, obwohl
// die Breiten von MAX_CALL_LEN und UDP_TX_BUF_SIZE abgeleitet sind. Der
// Versuch hat damals die MHeard-Tests umgeworfen: in configuration_global.h
// haengen die Ringgroessen am Board (je Zweig verschieden), und
// aprs_structures.h wird vielerorts FRUEH eingebunden -- frueher, als das
// Board-Makro gesetzt ist. Der Header waehlte dann den falschen Zweig, und
// die Tabellen hatten ploetzlich andere Groessen. Die Werte stehen
// deshalb als Literale hier, und die Kopplung an die Konstanten wird dort
// GEPRUEFT, wo beide Header ohnehin sichtbar sind (aprs_functions.cpp).
#define MC_PATH_LEN     121
#define MC_CALL_LEN_Z   21     // MAX_CALL_LEN + 1
#define MC_PAYLOAD_LEN  256    // UDP_TX_BUF_SIZE + 1

struct aprsMessage
{
    uint16_t msg_len;
    char payload_type;
    unsigned int msg_id;

    uint8_t max_hop;
    bool  msg_server;
    bool  msg_track;
    bool  msg_app_offline;
    bool  msg_mesh;

    char msg_source_path[MC_PATH_LEN];
    char msg_source_call[MC_CALL_LEN_Z];
    char msg_source_last[MC_CALL_LEN_Z];
    char msg_destination_path[MC_PATH_LEN];
    char msg_destination_call[MC_CALL_LEN_Z];
    char msg_payload[MC_PAYLOAD_LEN];
    char msg_gateway_call[MC_CALL_LEN_Z];
    unsigned int msg_fcs;
    uint8_t msg_source_hw;
    uint8_t msg_source_mod;
    uint8_t msg_source_fw_version;
    char msg_source_fw_sub_version;
    uint8_t msg_last_hw;
    uint8_t msg_last_path_cnt;
};

struct aprsPosition
{
    String pos_atxt;
    String pos_name;

    double lat;
    char lat_c;
    double lon;
    char lon_c;
    int alt;
    int bat;
    double lat_d;
    double lon_d;
    char aprs_group;
    char aprs_symbol;

    // wx
    float press;
    float hum;
    float temp;
    float temp2;
    int qfe;
    float qnh;
    float gasres;
    float co2;
    int ncnt;

    // more
    int version;
    int telemetry;
    char din[9]; // /D= MCP23017 port A bits, GPA0 first; "" when absent or malformed

    // power
    float vbus;      // /U= INA226 bus voltage
    float vcurrent;  // /I= INA226 current
    int grc[6];      // /R= Group-Call list, up to 6 groups (1..99999 each)
    int grccnt;      // number of valid entries in grc[]
};

#endif // _APRS_STRUCTURES_H_
