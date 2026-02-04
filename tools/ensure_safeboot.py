# SPDX-License-Identifier: MIT
"""Ensure the safeboot binary exists before flashing composite images."""

Import("env")
import subprocess
import sys
from pathlib import Path

SAFEBOOT_ENV = "esp32-S3-safeboot"
SAFEBOOT_FILENAME = "safeboot.bin"


def _build_safeboot(target, source, env):
    project_build_dir = Path(env.subst("$PROJECT_BUILD_DIR"))
    safeboot_path = project_build_dir / SAFEBOOT_ENV / SAFEBOOT_FILENAME

    if safeboot_path.exists():
        return

    python_exe = env.get("PYTHONEXE", sys.executable)
    cmd = [python_exe, "-m", "platformio", "run", "-e", SAFEBOOT_ENV]
    print(f"Building {SAFEBOOT_ENV} to create {SAFEBOOT_FILENAME}...")
    result = subprocess.run(cmd, cwd=env["PROJECT_DIR"], check=False)
    if result.returncode != 0:
        raise SystemExit(f"Failed to build {SAFEBOOT_ENV} (exit code {result.returncode}).")

    if not safeboot_path.exists():
        raise SystemExit(f"{SAFEBOOT_FILENAME} not found at {safeboot_path} after building {SAFEBOOT_ENV}.")

    print(f"Found {SAFEBOOT_FILENAME} at {safeboot_path}.")


# Run before building the firmware so the upload command always has the binary ready.
env.AddPreAction("$BUILD_DIR/${PROGNAME}.bin", _build_safeboot)
