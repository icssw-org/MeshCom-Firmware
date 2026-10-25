"""PlatformIO pre-script (AU-D10, #1187): every env gets MC_ENV_NAME (the env
name, used to pick the release asset) and MC_BUILD_TAG (empty for local
builds; the release build exports MC_BUILD_TAG=vX.YYz[.MM.DD] before
`pio run`). Wired once in the top-level [env] section of platformio.ini; an
env with its own extra_scripts does not inherit it, so code must default both
macros with #ifndef."""

import os

Import("env")  # noqa: F821  (SCons builtin)

env.Append(  # noqa: F821
    CPPDEFINES=[
        ("MC_ENV_NAME", env.StringifyMacro(env["PIOENV"])),  # noqa: F821
        ("MC_BUILD_TAG", env.StringifyMacro(os.environ.get("MC_BUILD_TAG", ""))),  # noqa: F821
    ]
)

# Fork builds only: MC_AU_DEV_CHANNEL=1 (fork CI, local fork builds) compiles the
# Auto Update dev channel (DK5EN releases, src/fw_update.h). Unset = prod only.
if os.environ.get("MC_AU_DEV_CHANNEL") == "1":
    env.Append(CPPDEFINES=["MC_AU_DEV_CHANNEL"])  # noqa: F821
