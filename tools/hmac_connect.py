#!/usr/bin/env python3
"""
hmac_connect.py — Connect to the MeshCom HMAC console (port 2323).

Usage:
    python3 hmac_connect.py <host>                       # no password
    python3 hmac_connect.py <host> -p 2323               # custom port, no password
    python3 hmac_connect.py <host>:2323                  # same, host:port form
    python3 hmac_connect.py <host> <password>            # with HMAC auth
    python3 hmac_connect.py <host> <password> <port>     # legacy positional form
    python3 hmac_connect.py <host> --ask                 # prompt for password
    python3 hmac_connect.py <host> --log                 # write session log next to this script
    python3 hmac_connect.py <host> --log /some/dir       # write session log into a directory
    python3 hmac_connect.py <host> --log my.log          # write session log to a file

Log file default name: hmac_<host>_<YYYYmmdd-HHMMSS>.log
Each received line is prefixed with a local timestamp; sent lines are
marked with ">>". Use --no-timestamps to log the raw stream only.

Protocol:
    password set:
        <- "NONCE: <32 hex chars>\\r\\n"
        -> "<64 hex HMAC-SHA256(password, nonce)>\\r\\n"
        <- "OK\\r\\n<banner>"  or  "FAIL\\r\\n"
    no password set:
        <- "OK\\r\\n<banner>"
    -- bidirectional plaintext afterwards --

Requires: Python 3.6+, no external dependencies.
"""

import argparse
import codecs
import datetime
import getpass
import hashlib
import hmac
import os
import re
import signal
import socket
import sys
import threading
import _thread

DEFAULT_PORT = 2323
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


class SessionLog:
    """Thread-safe line-oriented session logger."""

    def __init__(self, path, timestamps=True):
        self.path = path
        self.timestamps = timestamps
        self.fh = open(path, "a", encoding="utf-8", buffering=1)
        self.lock = threading.Lock()
        self.partial = ""
        self.closed = False

    def _stamp(self):
        return datetime.datetime.now().strftime("[%Y-%m-%d %H:%M:%S.%f")[:-3] + "] "

    def rx(self, text):
        with self.lock:
            if self.closed:
                return
            if not self.timestamps:
                self.fh.write(text)
                return
            self.partial += text.replace("\r\n", "\n").replace("\r", "\n")
            *lines, self.partial = self.partial.split("\n")
            for ln in lines:
                self.fh.write(self._stamp() + ln + "\n")

    def tx(self, text):
        with self.lock:
            if self.closed:
                return
            self._flush_partial()
            prefix = self._stamp() if self.timestamps else ""
            self.fh.write(prefix + ">> " + text + "\n")

    def note(self, text):
        with self.lock:
            if self.closed:
                return
            self._flush_partial()
            prefix = self._stamp() if self.timestamps else ""
            self.fh.write(prefix + "## " + text + "\n")

    def _flush_partial(self):
        if self.timestamps and self.partial:
            self.fh.write(self._stamp() + self.partial + "\n")
            self.partial = ""

    def close(self):
        with self.lock:
            if self.closed:
                return
            self._flush_partial()
            self.fh.close()
            self.closed = True


def log_path_for(host, log_arg):
    safe_host = re.sub(r"[^A-Za-z0-9._-]", "_", host)
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    name = f"hmac_{safe_host}_{stamp}.log"
    if log_arg is None or log_arg == "":
        return os.path.join(SCRIPT_DIR, name)
    log_arg = os.path.expanduser(log_arg)
    if os.path.isdir(log_arg) or log_arg.endswith(os.sep):
        os.makedirs(log_arg, exist_ok=True)
        return os.path.join(log_arg, name)
    return log_arg


def parse_args():
    ap = argparse.ArgumentParser(
        description="Connect to the MeshCom net console (HMAC auth, port 2323).")
    ap.add_argument("host", help="IP or hostname, optionally host:port")
    ap.add_argument("password", nargs="?", default=None,
                    help="console password (omit if none is set)")
    ap.add_argument("legacy_port", nargs="?", type=int, default=None,
                    metavar="port", help="port (legacy positional form)")
    ap.add_argument("-p", "--port", type=int, default=None,
                    help=f"TCP port (default {DEFAULT_PORT})")
    ap.add_argument("-a", "--ask", action="store_true",
                    help="prompt for the password (not visible in shell history)")
    ap.add_argument("-l", "--log", nargs="?", const="", default=None, metavar="PATH",
                    help="write a session log; PATH may be a file or directory "
                         "(default: script directory)")
    ap.add_argument("--no-timestamps", action="store_true",
                    help="log the raw stream without per-line timestamps")
    args = ap.parse_args()

    host = args.host
    port = args.port or args.legacy_port
    # host:port (but not a bare IPv6 address)
    if host.count(":") == 1:
        h, p = host.rsplit(":", 1)
        if p.isdigit():
            host, port = h, port or int(p)
    args.host = host
    args.port = port or DEFAULT_PORT
    return args


class Session:
    """Holds the resources that must be released on exit."""

    def __init__(self):
        self.sock = None
        self.log = None
        self.rx_thread = None
        self.stop = threading.Event()
        self.close_reason = "session closed by user"


def _raise_keyboard_interrupt(signum, frame):
    raise KeyboardInterrupt


# Signals main() turns into KeyboardInterrupt; cleanup() ignores all of them.
EXIT_SIGNALS = tuple(s for s in (signal.SIGINT, signal.SIGTERM, getattr(signal, "SIGHUP", None))
                     if s is not None)


def wake_main_thread(main_tid):
    """Interrupt the main thread's blocking input() after the remote closed the session."""
    if hasattr(signal, "pthread_kill"):
        signal.pthread_kill(main_tid, signal.SIGINT)
        return
    # Windows has no pthread_kill. A console Ctrl-C event aborts the console read the same way the
    # user's Ctrl-C does; without a console (stdin redirected) interrupt_main is enough.
    try:
        import ctypes
        if ctypes.windll.kernel32.GenerateConsoleCtrlEvent(0, 0):  # CTRL_C_EVENT, own process group
            return
    except (ImportError, AttributeError, OSError):
        pass
    _thread.interrupt_main()


def run(args, sess):
    host, port = args.host, args.port

    password = args.password
    if args.ask:
        password = getpass.getpass("Console password: ")
    password_b = password.encode() if password else b""

    if args.log is not None:
        path = log_path_for(host, args.log)
        sess.log = SessionLog(path, timestamps=not args.no_timestamps)
        sess.log.note(f"session start {host}:{port}")
        print(f"Logging to {path}", flush=True)
    log = sess.log

    print(f"Connecting to {host}:{port} ...", flush=True)
    s = sess.sock = socket.create_connection((host, port), timeout=10)
    s.settimeout(35)

    rxbuf = bytearray()

    def readline():
        """Read one line from the socket; leftover bytes stay in rxbuf."""
        while b"\n" not in rxbuf:
            chunk = s.recv(1024)
            if not chunk:
                break
            rxbuf.extend(chunk)
        idx = rxbuf.find(b"\n")
        if idx < 0:
            out = bytes(rxbuf)
            rxbuf.clear()
        else:
            out = bytes(rxbuf[:idx + 1])
            del rxbuf[:idx + 1]
        return out.decode("utf-8", errors="replace")

    def fail(msg):
        print(msg, file=sys.stderr)
        sess.close_reason = msg
        return 1

    line = readline()
    if not line:
        return fail("Connection closed before challenge.")
    line = line.strip()

    if line.upper().startswith("NONCE:"):
        if not password_b:
            if not sys.stdin.isatty():
                return fail("Server requires a password.")
            password_b = getpass.getpass("Server requires a password: ").encode()
        nonce = bytes.fromhex(line.split()[1].strip())
        response = hmac.new(password_b, nonce, hashlib.sha256).hexdigest()
        s.sendall((response + "\n").encode())

        result = readline().strip()
        if result != "OK":
            return fail(f"Authentication failed: {result!r}")
        print("Authentication OK.", flush=True)
        if log:
            log.note("authentication OK")
    elif line == "OK":
        if password_b:
            msg = "Note: server has no password set; the given password was ignored."
            if password and password.isdigit():
                msg += f" (Did you mean the port? Use -p {password})"
            print(msg, file=sys.stderr, flush=True)
        print("Connected (no password).", flush=True)
        if log:
            log.note("connected, no password required")
    else:
        return fail(f"Unexpected server response: {line!r}")

    # Interactive bidirectional I/O (banner is printed by the receive loop)
    s.settimeout(None)
    decoder = codecs.getincrementaldecoder("utf-8")(errors="replace")
    main_tid = threading.main_thread().ident

    def emit(text):
        if not text:
            return
        try:
            sys.stdout.write(text)
        except UnicodeEncodeError:
            # console code page without these characters (e.g. cp1252 on Windows)
            enc = sys.stdout.encoding or "ascii"
            sys.stdout.write(text.encode(enc, errors="replace").decode(enc))
        sys.stdout.flush()
        if log:
            log.rx(text)

    def recv_loop():
        try:
            if rxbuf:
                emit(decoder.decode(bytes(rxbuf)))
                rxbuf.clear()
            while not sess.stop.is_set():
                data = s.recv(1024)
                if not data:
                    break
                emit(decoder.decode(data))
        except OSError:
            pass
        except Exception as e:  # noqa: BLE001 -- the thread must not die without waking the main thread
            if not sess.stop.is_set():
                sess.close_reason = f"receive error: {e!r}"
                print(f"\n[Receive error: {e!r}]", file=sys.stderr, flush=True)
                wake_main_thread(main_tid)
            return
        if not sess.stop.is_set():
            # Remote side closed: wake the main thread out of input()
            sess.close_reason = "disconnected by remote"
            print("\n[Disconnected by remote]", flush=True)
            wake_main_thread(main_tid)

    sess.rx_thread = threading.Thread(target=recv_loop, name="rx")
    sess.rx_thread.start()

    while True:
        try:
            cmd = input()
        except EOFError:
            return 0
        if log:
            log.tx(cmd)
        try:
            s.sendall((cmd + "\r\n").encode())
        except OSError as e:
            sess.close_reason = f"send failed: {e}"
            return 1


def cleanup(sess):
    # From here on, ignore further Ctrl-C, SIGTERM and SIGHUP so cleanup cannot be interrupted
    for sig in EXIT_SIGNALS:
        signal.signal(sig, signal.SIG_IGN)
    sess.stop.set()
    if sess.sock is not None:
        try:
            # Graceful TCP close (FIN) so the node frees its console slot;
            # also unblocks recv() in the receive thread.
            sess.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
    if sess.rx_thread is not None:
        sess.rx_thread.join(timeout=3)
    if sess.sock is not None:
        sess.sock.close()
    if sess.log is not None:
        sess.log.note(sess.close_reason)
        sess.log.close()


def main():
    args = parse_args()
    for sig in EXIT_SIGNALS:
        signal.signal(sig, _raise_keyboard_interrupt)

    sess = Session()
    rc = 0
    try:
        rc = run(args, sess)
    except KeyboardInterrupt:
        if sess.close_reason == "session closed by user":
            print("\n[Closed]", flush=True)
            sess.close_reason = "session closed by user (interrupt)"
    except (OSError, ValueError) as e:
        print(f"Error: {e}", file=sys.stderr)
        sess.close_reason = f"error: {e}"
        rc = 1
    finally:
        cleanup(sess)
    sys.exit(rc)


if __name__ == "__main__":
    main()
