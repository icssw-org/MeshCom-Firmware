# Meshcom_Client 4.0

MeshCom 4.0 Protokoll
MeshCom 4.0 verwendet für die Payload-Daten das AX.25 Protokoll, wie es für APRS definiert ist. (aprs101.pdf APRS PROTOCOL REFERENCE Version 1.0 2000, Seite 12)

Verwendung finden:
- Kennung — APRS-Datentyp-Identifikator
- Meldungs-ID – 32-Bit LSB->MSB eindeutiger Wert
- MAX-HOP – max. 7  (Maske 0x07) default wird 5 verwendet was weitere 4 Weitergaben ermöglicht.
- 0x80 – Kennung ob diese Meldung bereits via MQTT-Server gelaufen ist
- 0x40 – Kennung das diese Meldung pro MeshClient mit dem Rufzeichen der weitergebenden Station ergänzt werden soll. Für Mess- und Kontrollzwecke.
- Quelladresse — Dieses Feld enthält das Rufzeichen und die SSID der Sendestation
- Zieladresse — Dieses Feld kann ein APRS-Ziel enthalten Rufzeichen oder „*“ für Übertragungen an ALLE.
- Digipeater — Es können 0 bis 8 Digipeater-Rufzeichen in diesem Feld enthalten sein. Hinweis: Diese Digipeater-Adressen können durch einen generischen APRS-Digipeater-Pfad überschrieben werden (angegeben durch die SSID der  Zieladresse).
- Information – Dieses Feld enthält Transportdaten. Das erste Zeichen dieses Feldes ist der APRS-Datentyp-Identifikator, der angibt welche Art von Daten folgen.
- Frame Check Sequence – Der FCS ist eine Sequenz von 16 Bits, die verwendet wird um die Integrität eines empfangenen Rahmens zu überprüfen.
- Gateway-ID – Das Gateway welches die nachricht an den MQTT-Server übergibt hängt die 32-Bit MAC-Adresse an damit eine Kompatibilität zu MeshCom 2.0 gegeben ist.

Meldungen:
- Textmeldungen:
:|!MMMMMMMM|!HH|OE0XXX-99|>*|:|Text-Meldung|!00|FCS#|!GGGGGGGG|!HW
- Textmeldungen mit Path aus Mesh:
:|!MMMMMMMM|!HH|OE0XXX-99,OE3XXX-12,OE3YYY-12|>*|:|Text-Meldung|!00|FCS#|!GGGGGGGG|!HW
- Positionsmeldungen:
!|!MMMMMMMM|!HH|OE0XXX-99|>*|!|4800.00|N|/|01600.00|E|#| BBB /A=HHHH|!00|FCS#|!GGGGGGGG|!HW

Legende:

| … dient nur zur Darstellung der Trennungen hier im Text

Meldungselemente
- : !	Meldungs-Kennung	Version 4.0
- MMMMMMMM	Meldungs-ID	32-Bit LSB->MSB
- HH	MAX-HOP	8-Bit Bit-Maske 0x07
- Message via MQTT-Server	Bit-Maske 0x80
- Path im Mesh einfügen (mit Beistrich als Trennung)	Bit-Maske 0x40
- 4800.00	Latitude	Grad/dezimal x 100
- 01600.00	Longidude	Grad/dezimal x 100
- N	N ord / S üd	char
- /	APRS SYMBOL Gruppe (/ oder \)	char
- E	E ast / W est	char
- \#	APRS SYMBOL	char
- BBB	Akkuzustand in %	int 0 - 100
- /A=HHHH	GPS Meereshöhe (m)	int 0 - 9999
- Meldungsabschluss	schließt den APRS-Meldungsbereich ab	0x00
- FCS#	Prüfsumme inkl. Kennung und 0x00 vom Meldungsabschluss	unsigned int 16-Bit
- GGGGGGGG	Gateway-ID (nur für MeshCom 2.0 Kompatibilität)	32-Bit LSB->MSB
- HW	Hardware-ID	8-Bit (siehe Tabelle)

MeshCom Hardware
- 1	TLORA_V2
- 2	TLORA_V1
- 3	TLORA_V2_1_1p6
- 4 TBEAM
- 6	TBEAM_0p7
- 7	T_ECHO
- 9	RAK4631
- 10	HELTEC_V2_1
- 11	HELTEC_V1
- 39	DIY_v1
