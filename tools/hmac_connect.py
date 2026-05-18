#!/usr/bin/env python3
"""
hmac_connect.py — Connect to the MeshCom HMAC console (port 2323).

Usage:
    python3 hmac_connect.py <ip>                   # no password
    python3 hmac_connect.py <ip> <password>         # with HMAC auth
    python3 hmac_connect.py <ip> <password> <port>  # custom port

Protocol:
    <- "NONCE: <32 hex chars>\\r\\n"
    -> "<64 hex HMAC-SHA256(password, nonce)>\\r\\n"
    <- "OK\\r\\n<banner>"  or  "FAIL\\r\\n"
    -- bidirectional plaintext afterwards --

Requires: Python 3.6+, no external dependencies.
"""

import sys
import socket
import hmac
import hashlib
import threading
import os

def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <ip> [password] [port]", file=sys.stderr)
        sys.exit(1)

    host     = sys.argv[1]
    password = sys.argv[2].encode() if len(sys.argv) > 2 else b""
    port     = int(sys.argv[3]) if len(sys.argv) > 3 else 2323

    print(f"Connecting to {host}:{port} ...", flush=True)
    s = socket.create_connection((host, port), timeout=10)
    s.settimeout(35)

    f = s.makefile("r", newline="\n")

    # Read challenge line
    line = f.readline()
    if not line:
        print("Connection closed before challenge.", file=sys.stderr)
        sys.exit(1)

    line = line.strip()

    if password:
        # Expect "NONCE: <hex>"
        if not line.upper().startswith("NONCE:"):
            print(f"Unexpected server response: {line!r}", file=sys.stderr)
            sys.exit(1)
        nonce_hex = line.split()[1].strip()
        nonce     = bytes.fromhex(nonce_hex)
        response  = hmac.new(password, nonce, hashlib.sha256).hexdigest()
        s.sendall((response + "\n").encode())

        # Read OK / FAIL
        result = f.readline().strip()
        if result != "OK":
            print(f"Authentication failed: {result!r}", file=sys.stderr)
            sys.exit(1)
        print("Authentication OK.", flush=True)
    else:
        # No password — server sends banner directly
        print(line, flush=True)

    # Read banner lines until empty line or prompt
    s.settimeout(1.0)
    try:
        while True:
            banner_line = f.readline()
            if not banner_line:
                break
            print(banner_line, end="", flush=True)
    except socket.timeout:
        pass
    s.settimeout(None)

    # Interactive bidirectional I/O
    def recv_loop():
        try:
            while True:
                data = s.recv(256)
                if not data:
                    break
                sys.stdout.write(data.decode(errors="replace"))
                sys.stdout.flush()
        except Exception:
            pass
        print("\n[Disconnected]", flush=True)
        os._exit(0)

    t = threading.Thread(target=recv_loop, daemon=True)
    t.start()

    try:
        while True:
            line = input()
            s.sendall((line + "\r\n").encode())
    except (EOFError, KeyboardInterrupt):
        pass

    s.close()

if __name__ == "__main__":
    main()
