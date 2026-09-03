#ifndef _LOOP_FUNCTIONS_H_
#define _LOOP_FUNCTIONS_H_

#include <Arduino.h>
#include <configuration.h>
#include <debugconf.h>
#include <aprs_functions.h>

#ifdef ESP32
    #include <esp32/esp32_flash.h>
#else
    #include <nrf52/WisBlock-API.h>
#endif

unsigned long getUnixClock();

void sendDisplay1306(bool bClear, bool bTransfer, int x, int y, char *text);
void sendDisplayHead(bool bInit);
#if defined(WP_DISP)
void wpShowStoredMessage(int slot, int idx);   // idx = Index oben rechts (neueste=N..aelteste=1, 0=keiner)
void wpShowDeepSleep();                         // E-Ink nur loeschen (kein Text) vor dem Deepsleep -> Sleep sichtbar
#endif
void sendDisplayTrack();
void sendDisplayWX();
void sendDisplayMainline();
void sendDisplayTime();
void mainStartTimeLoop();

void E290DisplayUpdate();


void init_loop_function();

void initAnalogPin();

void sendDisplayText(struct aprsMessage &aprsmsg, int16_t rssi, int8_t snr);
void sendDisplayPosition(struct aprsMessage &aprsmsg, int16_t rssi, int8_t snr);

String getDateString();
String getTimeString();

void printBuffer(uint8_t *buffer, int len);
void printAsciiBuffer(uint8_t *buffer, int len);
// SL-01: `tail` wird VOR dem `\n` angehaengt (Pegel-/Dedup-Anhang der
// RX-Zeile, siehe setlogFormatRxTail()); leer heisst Zeile wie bisher.
void printBuffer_aprs(char *msg_source, struct aprsMessage &aprsMessage, const char *tail = "");
void charBuffer_aprs(struct aprsMessage &aprsMessage);
void printBuffer_ack(char *msgSource, uint8_t payload[UDP_TX_BUF_SIZE+10], int16_t size, const char *tail = "");

void addBLEOutBuffer(uint8_t *buffer, uint16_t len);
void addBLEComToOutBuffer(uint8_t *buffer, uint16_t len);
void addBLECommandBack(char *text);
// addLoraRxBuffer() wird jetzt in dedup_functions.h deklariert.
// N-14: kompletter TX-Ring-Enqueue (Slot-Wahl, Payload-Kopie, Prio/Overflow,
// iWrite/iRead) in einer Funktion unter einem Lock (nRF52) -- siehe
// lora_functions.cpp fuer Details/Locking-Begruendung. Rueckgabe: Slot-Index
// oder -1 wenn die Overflow-Logik den Eintrag verworfen hat.
// retryCountIn: -1 (Default) laesst retryCount[Slot] unangetastet.
int addTxRingEntry(const uint8_t* frame, uint16_t len, uint8_t ring_status,
                    const char* source, int retryCountIn = -1, bool clearSlotFirst = false);

// checkOwnRx()/checkServerRx() werden jetzt in dedup_functions.h deklariert.
int checkOwnTx(unsigned int msg_id);
void insertOwnTx(unsigned int id);

int esp32_isSSD1306(int address);

void DisplayPong(char line1[20], char line2[20], char line3[20], char line4[20]);
void sendPing(char msg_call[10]);
void SendPong(String msg_source_call, unsigned int msg_id);
void PongFail(String msg_source_call);

// BP-09: return value is a BpSendResult (see backpressure.h) -- lets a caller
// keep the operator's typed text instead of clearing an input field for a
// message that never went out.
// src_override: NULL = send under the node call (normal case). A non-empty
// string sends the message under that callsign instead -- used by the KISS
// interface to preserve the client's callsign (its base must match the node
// call, checked by the caller).
// out_msg_id: when non-NULL, receives the assigned msg_id on BP_SEND_OK (the
// KISS interface needs it for the TX-result frame and the ack map).
int sendMessage(char *msg_text, int len, const char *src_override = nullptr,
                unsigned int *out_msg_id = nullptr);

// Inject an APRS position received over the KISS interface, under srcCall.
// posData is the APRS position payload without the leading data-type char
// (and without any timestamp). The caller has already verified srcCall.
// Returns the assigned msg_id, or 0 on a bad argument.
unsigned int sendInjectedPosition(const char *srcCall, const char *posData);
String PositionToAPRS(bool bConvPos, bool bWeather, bool bFuss, double lat, char lat_c, double lon, char lon_c, int alt, float press, float hum, float temp, float temp2, float gasres, int qfe, float qnh);
void sendPosition(unsigned long intervall, double lat, char lat_c, double lon, char lon_c, int alt, float press, float hum, float temp, float temp2, float gasres, float co2, int qfe, float qnh);
void sendAPPPosition(double lat, char lat_c, double lon, char lon_c, float temp2);
// Send a MeshCom message ACK to dest_call. src_override (when set) puts the ACK
// on air under a foreign source callsign — used by the KISS interface to relay a
// directly-connected client's APRS ack. Returns the assigned msg_id.
unsigned int SendAckMessage(String dest_call, unsigned int iAckId, const char *src_override = nullptr);
void sendHey();
bool sendHeyShot();
void sendTelemetry(int ID);

unsigned int setSMartBeaconing(double flat, double flon);

String convertCallToShort(char callsign[10]);

uint8_t shortVERSION();
char shortSUBVERSION();

double cround4(double dvar);
double cround4abs(double dvar);

int conv_fuss(int alt_meter);
int conv_meter(int alt_fuss);

#if defined(BOARD_E290) || defined(BOARD_WIRELESS_PAPER) || defined(BOARD_E213)
void DrawDirection(float angle, int cx, int cy, int radius);
void DrawRssi(int cx, int cy, int16_t rssi);
double degreesToRadians(double degrees);
#endif

byte utf8ascii(byte ascii);
String utf8ascii(String s);
void utf8ascii(char* s);

String getTimeZone();

int count_char(String s, char c);

void addRingPointer(volatile int &toWrite, volatile int &toRead, int iMAX, const char* bufName = "?");

void oledStat();
void oledInvalidate();
extern bool bOledLog;
bool is_equ(const char* buf1, const char* buf2);
int is_pos(const char* buf, const char* comp_buf);

#endif // _LOOP_FUNCTIONS_H_