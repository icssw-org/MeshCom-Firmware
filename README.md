# MeshCom
MeshCom is indeed an exciting project of the Institute of Citizen Science for Space & Wireless communication (www.icssw.org)  aimed at creating a resilient, text-based communication tool for amateur radio operators. It utilizes LORA™ modulation technology and the APRS protocol to establish a mesh network in the 70cm band. The main objectives of MeshCom are to realize a connected off-grid messaging system with low energy consumption and cost-effective hardware. The technical implementation is based on LORA™  radio modules, which can transmit messages, positions, measurements, and more over long distances with low transmit power. MeshCom modules can be connected to form a mesh network or establish a messaging network via MeshCom gateways, ideally connected through HAMNET.

## Basic functions:
-	Each Node is identified by a Amateur Radio Callsign (with optional SSID)
-	Short test messages can be sent to ALL (everybody), with ACK from Server/Gateway
-	Short text messages can be sent DIRECTLY to other Callsign, with End-toEnd Aknowledgement
-	Some Nodes can als be to configured to act as GATEWAY to HAMNET or INTERNET (wifi)
-	Each Node should act as a repeater for all other MESHCOM messages on air
-	Servers and Gateways might have some functionalities to avoid the transmission of redundant trafic
-	Nodes will automatically send STATUS and POSITION messages
-	NODES with optional Sensors will send WX-Data or TELEMETRY Data periodically
-	Messages will be diplayed on small OLED Display or via BT connected smartphone or tablet device or via USB connected serial console

The main goal is to have a selfbuilding and selfhealing Mesh-Network, that can be enhanced by other components of the Amateur Radio Service, like HAMNET (IP-Network), centralised or distributed Meshcom servers. This will increase coverage to all continents and enable interconnection to other modes and services (APRS, WINLINK, DMR, TETRA-SDS, SOTA-WATCH, POCSAG,VARA-AC, …) building an unified communication plattform.
Particulary useful is Meshcom for Emergency Communication (EMCOM) in case of disaster or Blackout.
In all usecases terms  and rules of Amateur Radio Service (strictly non commercial, experimental) should be respected.
This is an open Citizen Science project that should help to promote Amateur Radio Service within academic and society.

## Frequency in Region: 
EU: 433.175
UK: tbd
Nordic: tbd
USA: 433.175
Afrika: 433.175
Asia/Pacific: tbd

## Lora parameter:
SF: 11
Bandwith: 250kHz
CR: 4/6

## APRS-Protokoll: 
Document: http://www.aprs.org/doc/APRS101.PDF
Address: Call-SSID, Source, Target, DIGI1-5
Telemetry: data, formula, units,…
Weather: Temp, pressure, rain,…
Aim is to be fully compatible to aprs.fi

## Hardware:
ESP32/LoRa-Modul, RAK-WISBLOCK, ESP32-DEV4/E22-LoRa, ...

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

#### Message elements
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

## Preparations for platform.io VSCode plugin
- Install the needed frameworks under Platforms: 
  + Espressif 32
  + Nordic nRF52 Version 9.6.0 (for now)
- For adding the correct Board defintions for RAK Wireless RAK4631 follow these instructions:
    https://github.com/RAKWireless/WisBlock/blob/master/PlatformIO/README.md

## Flashing Firmware
Usually it is done via the upload button in VSCode directly. 
### ESP32 Via Command Line:
- For this task the esptool is needed. You can either use the one from platform.io which is located at the `.platformio/tool-esptoolpy/esptool.py` in addition with the python venv, which is at: `.platformio/penv/bin/python`. The hidden `.platformio` directory is located in your User-Directory.<br/>
Otherwise if not already installed, install a recent python version. Then you need to get the esptool via Pip: `pip install esptool` <br/>
- The firmware.bin, bootloader.bin and partition.bin file is written after compiling to the hidden `.pio/build` directory of the MeshCom-Firmware repo directory.<br/>

If you only update the firmware, you only want the corresponding file to flash.<br> Adresses where to flash each one of the files:<br/>

| Address | File |
| --- | ----------- |
| 0x1000 | bootloader.bin |
| 0x8000 | partitions.bin |
| 0x10000 | firmware.bin |

Mac: `python esptool.py -p /dev/tty.usbserial-<NUMBER> write_flash 0x10000 <PATH-TO-BIN-FILE>/firmware.bin`<br/>
Linux: same but serial device under `/dev` can be `ttyUSB0` or similar.<br/>
Windows: serial device is usually some COM<br/>
Ready build firmware can also be flashed via the online tool (Chrome, Edge, Opera):<br/>
https://oe1kfr.com/esptool/<br/>

#### Erasing the NVS: 
If you want to wipe the settings stored on the node:<br/>
`python esptool.py --port <SERIAL-PORT> erase_region 0x009000 0x005000`
### RAK4631 via CLI:
To do so, you need the Adafruit nrfutil. Installation and Usage:<br/>
https://github.com/adafruit/Adafruit_nRF52_nrfutil<br/>

### RAK4631 via UF2 File:
When you double click the button on the module it mounts a USB Device where you can copy an .uf2 file onto the module. To generate that file you need the following Python script:<br/>
https://github.com/microsoft/uf2/blob/master/utils/uf2conv.py<br/>

`./uf2conv.py <PATH_TO-HEX-FILE> -c -o firmware.uf2 -f 0xADA52840`
