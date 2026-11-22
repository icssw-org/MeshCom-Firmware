"""PlatformIO pre-script (ETH-06): patch RAK13800-W5100S socketSendUDP() in place.

What: the wait for SEND_OK / TIMEOUT after Sock_SEND gets a 3 s deadline. When neither
flag comes, the interrupt bits are cleared and the send fails (returns false), exactly
like the library's own TIMEOUT branch.
Why: the loop had no bound of its own. After a W5100S hardware reset the chip no longer
knows the socket, so a SEND on it raises neither flag and the loop never ends. On
DB0AIS-12 (RAK4631, 2026-11-14 05:43) this froze the main loop for 52 h: no LoRa TX, no
gateway keepalive, RX still running. The
firmware side (extudp_functions.cpp externUdpChipReset()) removes that trigger; this
bound turns any other stuck socket into a 3 s stall instead of a dead node. The chip's
own UDP timeout (ARP retries) measured 1.6 s on DB0AIS, so 3 s never cuts a live send.
Remove when RAK13800-W5100S bounds socketSendUDP() itself.

Idempotent (marker, no write when patched); keeps the file's CRLF line endings; fails
the build if the library text changed; no-op where the library is absent (ESP32,
native). The text transform is importable without SCons.
"""

import sys
from pathlib import Path

MARKER = "/* MESHCOM-PATCH ETH-06 */"
REL = "RAK13800-W5100S/src/socket.cpp"
DEADLINE_MS = 3000

ORIGINAL = """\t/* +2008.01 bj */
\twhile ( (W5100.readSnIR(s) & SnIR::SEND_OK) != SnIR::SEND_OK ) {
\t\tif (W5100.readSnIR(s) & SnIR::TIMEOUT) {"""

PATCHED = (
    """\t/* +2008.01 bj */
\tuint32_t meshcom_send_t0 = millis(); """
    + MARKER
    + """
\twhile ( (W5100.readSnIR(s) & SnIR::SEND_OK) != SnIR::SEND_OK ) {
\t\tif ((uint32_t)(millis() - meshcom_send_t0) >= """
    + str(DEADLINE_MS)
    + """) { """
    + MARKER
    + """
\t\t\tW5100.writeSnIR(s, (SnIR::SEND_OK|SnIR::TIMEOUT));
\t\t\tSPI.endTransaction();
\t\t\treturn false;
\t\t}
\t\tif (W5100.readSnIR(s) & SnIR::TIMEOUT) {"""
)


def patch_text(text: str) -> str:
    """Return patched text; unchanged if already patched; RuntimeError if unknown."""
    if MARKER in text:
        return text
    eol = "\r\n" if "\r\n" in text else "\n"
    orig = ORIGINAL.replace("\n", eol)
    if text.count(orig) != 1:
        raise RuntimeError(
            "w5100s_patch ETH-06: socketSendUDP() wait loop not found in the expected form; "
            "RAK13800-W5100S changed, review the patch"
        )
    return text.replace(orig, PATCHED.replace("\n", eol))


def patch_file(path: Path) -> bool:
    """Patch file in place; True if written. Missing file is a no-op."""
    if not path.is_file():
        return False
    text = path.read_bytes().decode("utf-8")
    out = patch_text(text)
    if out == text:
        return False
    path.write_bytes(out.encode("utf-8"))
    return True


def wants_w5100s(lib_deps) -> bool:
    """True if a lib_deps list (strings or one newline-joined string) names RAK13800-W5100S."""
    if isinstance(lib_deps, str):
        lib_deps = lib_deps.splitlines()
    return any("rak13800-w5100s" in str(d).lower() for d in lib_deps)


try:
    Import("env")  # noqa: F821  (SCons builtin)
except NameError:
    env = None


if env is not None:
    _p = Path(env.subst("$PROJECT_LIBDEPS_DIR")) / env["PIOENV"] / REL
    # Dependencies are installed before pre-scripts run, so an env that pulls the
    # library and has no patch target here means the layout moved: fail, never
    # build an unpatched image silently.
    if not _p.is_file() and wants_w5100s(env.GetProjectOption("lib_deps", [])):
        raise RuntimeError(f"w5100s_patch ETH-06: RAK13800-W5100S in lib_deps but {_p} is missing")
    if patch_file(_p):
        print("w5100s_patch: ETH-06 applied to", _p, file=sys.stderr)
