# MeshCom 4.0 Firmware
MeshCom is a project to exchange text messages via LORA radio modules. The primary goal is to realize networked off-grid messaging with low power and low cost hardware.

The technical approach is based on the use of LORA radio modules which transmit messages, positions, measured values, telecontrol and much more with low transmission power over long distances. MeshCom modules can be combined to form a mesh network, but can also be connected to a message network via MeshCom gateways, which are ideally connected via HAMNET. This enables MeshCom radio networks, which are not connected to each other via radio, to communicate with each other.

# MeshCom Protocol
MeshCom 4.0 uses the APRS PROTOCOL REFERENCE for the source, destination, Digipeater and payload data as defined for APRS. (aprs101.pdf APRS PROTOCOL REFERENCE Version 1.0 2000, Page 12)
MeshCom 4.0 verwendet für die Payload-Daten das AX.25 Protokoll, wie es für APRS definiert ist. (aprs101.pdf APRS PROTOCOL REFERENCE Version 1.0 2000, Seite 12)

# Verwendung finden:
- Kennung — APRS-Datentyp-Identifikator
- Meldungs-ID – 32-Bit LSB->MSB eindeutiger Wert
- MAX-HOP – max. 7  (Maske 0x07) default wird 5 verwendet was weitere 4 Weitergaben ermöglicht.
  - 0x80 – Kennung ob diese Meldung bereits via MeshCom-Server gelaufen ist
  - 0x40 – Kennung das diese Meldung pro MeshClient mit dem Rufzeichen der weitergebenden Station ergänzt werden soll. Für Mess- und Kontrollzwecke.
- Quelladresse — Dieses Feld enthält das Rufzeichen und die SSID der Sendestation
- Zieladresse — Dieses Feld kann ein APRS-Ziel enthalten Rufzeichen oder „*“ für Übertragungen an ALLE.
- Digipeater — Es können 0 bis 8 Digipeater-Rufzeichen in diesem Feld enthalten sein. Hinweis: Diese Digipeater-Adressen können durch einen generischen APRS-Digipeater-Pfad überschrieben werden (angegeben durch die SSID der  Zieladresse).
- Information – Dieses Feld enthält Transportdaten. Das erste Zeichen dieses Feldes ist der APRS-Datentyp-Identifikator, der angibt welche Art von Daten folgen.
- Frame Check Sequence – Der FCS ist eine Sequenz von 16 Bits, die verwendet wird um die Integrität eines empfangenen Rahmens zu überprüfen.

# Meldungen:
- Textmeldungen:
:|!MMMMMMMM|!HH|OE0XXX-99|>*|:|Text-Meldung|!00|!HW|!MOD|FCS#
- Textmeldungen mit Path aus Mesh:
:|!MMMMMMMM|!HH|OE0XXX-99,OE3XXX-12,OE3YYY-12|>*|:|Text-Meldung|!00|!HW|!MOD|FCS#
- Positionsmeldungen:
!|!MMMMMMMM|!HH|OE0XXX-99|>*|!|4800.00|N|/|01600.00|E|#| BBB /A=HHHHH|!00|!HW|!MOD|FCS#
Prüfsumme: FCS wir als unsigned 16-Bit-Summe der Protokoll-Bytes 0 bis inkl. MOD-Byte gebildet. und Little-Endian nach dem MOD-Byte angehängt.
