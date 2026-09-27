# SPDX-License-Identifier: MIT
"""Ensure the Tasmota Arduino framework is installed before the builder runs.

The safeboot environments use the Tasmota platform-espressif32 fork, whose
framework-arduinoespressif32 package shares its installation directory with
the incompatible framework package of the mainline espressif32 platform
(used by all application environments). After any mainline environment has
built, the shared directory holds the mainline framework (3.20017.x, owner
"platformio") and the Tasmota builder crashes before compiling anything:
get_package_dir() returns None and builder/frameworks/arduino.py raises
"TypeError: expected str, bytes or os.PathLike object, not NoneType".

The mainline platform handles the reverse swap gracefully (it reinstalls
its framework whenever a safeboot build has replaced it). This pre-script
makes the Tasmota side equally graceful: if the framework the platform
expects is not resolvable, the stale directory is removed and the correct
package is installed before the builder script needs it.
"""

Import("env")

import shutil
from pathlib import Path

PKG = "framework-arduinoespressif32"

platform = env.PioPlatform()

if platform.get_package_dir(PKG) is None:
    stale = Path(platform.packages_dir) / PKG
    if stale.exists():
        print(f"ensure_tasmota_framework: removing mismatched {stale}")
        shutil.rmtree(stale)
    spec = platform.packages[PKG]["version"]
    print(f"ensure_tasmota_framework: installing {PKG} from {spec}")
    platform.pm.install(spec)
