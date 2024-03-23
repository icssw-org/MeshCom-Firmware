# MeshCom 4.0 Firmware
MeshCom is a project to exchange text messages via LORA radio modules. The primary goal is to realize networked off-grid messaging with low power and low cost hardware.

The technical approach is based on the use of LORA radio modules which transmit messages, positions, measured values, telecontrol and much more with low transmission power over long distances. MeshCom modules can be combined to form a mesh network, but can also be connected to a message network via MeshCom gateways, which are ideally connected via HAMNET. This enables MeshCom radio networks, which are not connected to each other via radio, to communicate with each other.

## MeshCom Protocol
MeshCom 4.0 uses the APRS PROTOCOL REFERENCE for the source, destination, Digipeater and payload data as defined for APRS. (aprs101.pdf APRS PROTOCOL REFERENCE Version 1.0 2000, Page 12)
MeshCom 4.0 verwendet für die Payload-Daten das AX.25 Protokoll, wie es für APRS definiert ist. (aprs101.pdf APRS PROTOCOL REFERENCE Version 1.0 2000, Seite 12)

### Terms:
- Identifier — APRS data type identifier
- Message ID – 32-bit LSB->MSB unique value
- MAX-HOP – max. 7 (mask 0x07) default 5 is used which allows another 4 transfers.
  - 0x80 – ID as to whether this message has already been sent via the MQTT server
  - 0x40 – Identification that this message should be supplemented for each MeshClient with the call sign of the transmitting station. For measurement and control purposes.
- Source Address — This field contains the callsign and SSID of the transmitting station
- Destination Address — This field can contain an APRS destination call sign or
  - “*” for transmissions to ALL.
- Digipeater — There can be 0 to 8 digipeater callsigns in this field. Note: These digipeater addresses can be overwritten by a generic APRS digipeater path (specified by the SSID of the destination address).
- Payload – This field contains transport data. The first character of this field is the APRS data type identifier, which indicates what type of payload data follows.
- Hardware ID - see table below
- LoRa-Modulation INDEX - see table below
- Frame Check Sequence – The FCS is a sequence of 16 bits used to check the integrity of a received frame.

### Messages:
- Text messages:
   - :|!MMMMMMMM|!HH|OE0XXX-99|>*|:|Text message|!00|!HW|!MOD|FCS#
- Text messages with Path from Mesh:
    - :|!MMMMMMMM|!HH|OE0XXX-99,OE3XXX-12,OE3YYY-12|>*|:|Text message|!00|!HW|!MOD|FCS#
- Position reports:
    - !|!MMMMMMMM|!HH|OE0XXX-99|>*|!|4800.00|N|/|01600.00|E|#| BBB /A=HHHH|!00|!HW|!MOD|FCS#
- Legend:
   - | ... only serves to show the separations here in the text

Message elements
- Medlution ID: ! @ ... text, position, weather message
- MMMMMMMM Message ID 32-bit LSB->MSB
- HH MAX-HOP 8-bit bit mask 0x07
- Message via MQTT server bit mask 0x80
- Insert path into mesh (with comma as separation) bit mask 0x40
- 4800.00 latitude degrees/decimal x 100
- 01600.00 Longidude degrees/decimal x 100
- N north / south char
- / APRS SYMBOL group (/ or \) char
- E E ast / West char
- \# APRS SYMBOL char
- BBB battery status in % int 0 - 100
- /A=HHHH GPS sea level (m) int 0 - 9999
- Message completion closes the APRS message range from 0x00
- HW ... Hardware Type ID
- MOD ... LoRa modulation ID
- FCS# checksum including identifier and 0x00 from message completion unsigned int 16-bit
- Extra information to form MHEARD
  - GGGGGGGG Gateway ID (only for MeshCom 2.0 compatibility) 32-bit LSB->MSB
  - HW hardware ID 8-bit (see table)

### MeshCom hardware ID

- Hardware ID HW type MCU type LoRa type HW short name HW version
- 1 TTGO ESP32 Paxcounter ESP32 SX1278 TLORA V2
- 2 TTGO ESP32 Paxcounter ESP32 SX1278 TLORA V1
- 3 TTGO ESP32 Paxcounter ESP32 SX1278 TLORA V2 1.6
- 4 TTGO T-Beam ESP32 SX1278 T-BEAM 1.1
- 5 TTGO T-Beam ESP32 SX1268 T-BEAM-1268 1.1 1268
- 6 TTGO T-Beam ESP32 SX1262 T-BEAM-0.7 0.7
- 7 T-Echo LoRa SX1262 nRF SX1262 T-ECHO
- 8 T-Deck ESP32-S3 SX1262 T-DECK
- 9 Wisblock RAK4631 nRF Wisblock nRF RAK4631
- 10 WiFi LoRa 32 v2 ESP32 SX1262 HELTEC-V2-1 V2
- 11 WiFi LoRa 32 v1 ESP32 SX1276 HELTEC-V1 V1
- 12 TTGO T-Beam ESP32 SX1278 TBEAM-AXP2101
- 39 Ebyte Lora E22 ESP32 SX1278 EBYTE-E22
- 43 WiFi LoRa 32 v3 ESP32-S3 SX1262 HELTEC-V3 V3

### MeshCom LoRa modulations index

- 0 Extended range 10-20 fast
- 1 Extended range 10-20 slow (robust) 0.275 kbps
- 2 Additional extended range 20-40 slow (robust) 0.183 kbps
- 3 Normal range 5-10 slow (robust)
- 4 Normal range 5-10 fast 5,469 kbps
- 5 Local range 0-5 slow (robust)
- 6 Local range 0-5 fast 21,875 kbps
