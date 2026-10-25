"""PlatformIO pre-script (AU-19, AU-20): patch NimBLE-Arduino in place.

AU-19, ble_hs.c:

What: in ble_hs_timer_reset(), the "host not enabled" branch no longer calls
ble_npl_callout_deinit(&ble_hs_timer); it only stops the timer. A timer event
that is already queued keeps its callback and runs harmlessly (it reschedules,
which now just stops). No ble_npl_eventq_remove(): its drain-and-requeue holds a
function-local portMUX, which does not lock against the host task on the other core.
Why: while a stop begins, the deinit zeroes the callout (including the embedded
event's callback) although the esp_timer may have just queued that event on the
host queue; the host task then calls NULL (classic ESP32 panic InstrFetchProhibited,
PC 0x0). ble_hs_deinit() deinits the callout anyway, so the early deinit is redundant.
Remove when NimBLE-Arduino fixes ble_hs_timer_reset().

AU-20, ble_hs_flow.c:
What: ble_hs_flow_startup() calls ble_npl_callout_deinit(&ble_hs_flow_timer) right
before its ble_npl_callout_init(&ble_hs_flow_timer, ...).
Why: ble_hs_flow_init() and ble_hs_flow_startup() both init the same callout, and
the live npl_freertos_callout_init() always esp_timer_create()s; the second init
orphans the first esp_timer (~50 B per BLE pause/resume); ble_hs_flow_deinit() deletes
only the second. The deinit runs in the host task at sync; on a pause/resume the timer
was just created and is not armed. After a host reset it may be armed: the deinit
stops and deletes it, and a timer event already queued is harmless (the init restores
its callback in the same host task before the event can be dequeued).
Remove when NimBLE-Arduino fixes the double init.

Idempotent (marker, no write when patched); fails the build if the library text
changed; no-op where the library is absent (nRF52, native). The text transform
is importable without SCons.
"""

import sys
from pathlib import Path

MARKER = "/* MESHCOM-PATCH AU-19 */"
REL = "NimBLE-Arduino/src/nimble/nimble/host/src/ble_hs.c"

ORIGINAL = """    if (!ble_hs_is_enabled()) {
        ble_npl_callout_stop(&ble_hs_timer);
        ble_npl_callout_deinit(&ble_hs_timer);
    } else {"""

PATCHED = (
    """    if (!ble_hs_is_enabled()) {
        """
    + MARKER
    + """
        ble_npl_callout_stop(&ble_hs_timer);
    } else {"""
)


def patch_text(text: str) -> str:
    """Return patched text; unchanged if already patched; RuntimeError if unknown."""
    if MARKER in text:
        return text
    if text.count(ORIGINAL) != 1:
        raise RuntimeError(
            "nimble_patch AU-19: ble_hs_timer_reset() not found in the expected form; NimBLE-Arduino changed, review the patch"
        )
    return text.replace(ORIGINAL, PATCHED)


FLOW_MARKER = "/* MESHCOM-PATCH AU-20 */"
FLOW_REL = "NimBLE-Arduino/src/nimble/nimble/host/src/ble_hs_flow.c"

FLOW_ORIGINAL = """#endif
    ble_npl_callout_init(&ble_hs_flow_timer, ble_hs_evq_get(),
                         ble_hs_flow_event_cb, NULL);
#endif

    return 0;"""

FLOW_PATCHED = (
    """#endif
    ble_npl_callout_deinit(&ble_hs_flow_timer); """
    + FLOW_MARKER
    + """
    ble_npl_callout_init(&ble_hs_flow_timer, ble_hs_evq_get(),
                         ble_hs_flow_event_cb, NULL);
#endif

    return 0;"""
)


def patch_flow_text(text: str) -> str:
    """AU-20: return patched text; unchanged if already patched; RuntimeError if unknown."""
    if FLOW_MARKER in text:
        return text
    if text.count(FLOW_ORIGINAL) != 1:
        raise RuntimeError(
            "nimble_patch AU-20: ble_hs_flow_startup() not found in the expected form; NimBLE-Arduino changed, review the patch"
        )
    return text.replace(FLOW_ORIGINAL, FLOW_PATCHED)


def _patch_path(path: Path, transform) -> bool:
    if not path.is_file():
        return False
    text = path.read_text()
    out = transform(text)
    if out == text:
        return False
    path.write_text(out)
    return True


def patch_flow_file(path: Path) -> bool:
    """AU-20: patch file in place; True if written. Missing file is a no-op."""
    return _patch_path(path, patch_flow_text)


def patch_file(path: Path) -> bool:
    """Patch file in place; True if written. Missing file is a no-op."""
    return _patch_path(path, patch_text)


try:
    Import("env")  # noqa: F821  (SCons builtin)
except NameError:
    env = None


def wants_nimble(lib_deps) -> bool:
    """True if a lib_deps list (strings or one newline-joined string) names NimBLE-Arduino."""
    if isinstance(lib_deps, str):
        lib_deps = lib_deps.splitlines()
    return any("nimble-arduino" in str(d).lower() for d in lib_deps)


if env is not None:
    _libdeps = Path(env.subst("$PROJECT_LIBDEPS_DIR")) / env["PIOENV"]
    # Dependencies are installed before pre-scripts run, so an env that pulls
    # NimBLE-Arduino and has no patch target here means the layout moved: fail,
    # never build an unpatched image silently.
    for _id, _rel, _patch in (("AU-19", REL, patch_file), ("AU-20", FLOW_REL, patch_flow_file)):
        _p = _libdeps / _rel
        if not _p.is_file() and wants_nimble(env.GetProjectOption("lib_deps", [])):
            raise RuntimeError(
                f"nimble_patch {_id}: NimBLE-Arduino in lib_deps but {_p} is missing"
            )
        if _patch(_p):
            print(f"nimble_patch: {_id} applied to", _p, file=sys.stderr)
