#!/usr/bin/env python3
"""
Robuster serieller Monitor fuer Heltec Vision Master E213 (ESP32-S3 USB-Serial/JTAG / HWCDC).

Warum dieses Skript? Der USB-Serial/JTAG-Port des ESP32-S3 verschwindet auf macOS im Betrieb
immer wieder kurz aus dem System (USB-Re-Enumeration OHNE Board-Reset). 'screen' stirbt beim
ersten Aussetzer; dieses Skript verbindet automatisch neu und ueberbrueckt die Luecke.

Nutzung:
    python3 tools/e213_monitor.py            # Port automatisch suchen (usbmodem*)
    python3 tools/e213_monitor.py /dev/cu.usbmodem1101 115200

Eingabe: Tippe eine Zeile + ENTER -> wird mit \\n an das Board gesendet (z.B. "--info").
Beenden: Ctrl-C.
"""
import sys, time, glob, threading

try:
    import serial
except ImportError:
    sys.exit("pyserial fehlt. Installiere: ~/.platformio/penv/bin/python -m pip install pyserial\n"
             "oder starte dieses Skript mit ~/.platformio/penv/bin/python")

BAUD = 115200
PORT_ARG = None
for a in sys.argv[1:]:
    if a.isdigit():
        BAUD = int(a)
    else:
        PORT_ARG = a


def find_port(timeout=10.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if PORT_ARG:
            import os
            if os.path.exists(PORT_ARG):
                return PORT_ARG
        else:
            ports = sorted(glob.glob("/dev/cu.usbmodem*"))
            if ports:
                return ports[0]
        time.sleep(0.2)
    return None


_ser = None
_stop = False


def reader():
    """Liest fortlaufend und gibt aus; bei Verbindungsverlust wird in main() neu verbunden."""
    global _ser
    while not _stop:
        s = _ser
        if s is None:
            time.sleep(0.1)
            continue
        try:
            data = s.readline()
            if data:
                sys.stdout.write(data.decode("utf-8", "replace"))
                sys.stdout.flush()
        except Exception:
            time.sleep(0.1)  # Verbindung weg -> main() reconnectet


def main():
    global _ser, _stop
    print("E213-Monitor (Auto-Reconnect). Ctrl-C zum Beenden.\n")
    th = threading.Thread(target=reader, daemon=True)
    th.start()
    cur_port = None
    try:
        # Eingabe-Thread: Zeilen von stdin an das Board
        def writer():
            for line in sys.stdin:
                s = _ser
                if s is not None:
                    try:
                        s.write((line.rstrip("\n") + "\n").encode("utf-8", "replace"))
                    except Exception:
                        pass
        wt = threading.Thread(target=writer, daemon=True)
        wt.start()

        while True:
            if _ser is None:
                port = find_port(15)
                if not port:
                    print("\n[monitor] kein usbmodem-Port gefunden, warte ...", flush=True)
                    continue
                try:
                    _ser = serial.Serial(port, BAUD, timeout=0.3)
                    if port != cur_port:
                        print(f"\n[monitor] verbunden mit {port} @ {BAUD}\n", flush=True)
                        cur_port = port
                except Exception:
                    _ser = None
                    time.sleep(0.3)
                    continue
            # Verbindung pruefen (Port noch da?)
            import os
            if cur_port and not os.path.exists(cur_port):
                try:
                    _ser.close()
                except Exception:
                    pass
                _ser = None  # -> reconnect oben
            time.sleep(0.4)
    except KeyboardInterrupt:
        _stop = True
        print("\n[monitor] beendet.")


if __name__ == "__main__":
    main()
