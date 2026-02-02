# SPDX-License-Identifier: MIT
#
# Copyright (C) 2023-2024 Mathieu Carbou
#
Import("env")
import os
import sys
import shutil

quiet = False


def status(msg):
    """Print status message to stderr"""
    if not quiet:
        critical(msg)


def critical(msg):
    """Print critical message to stderr"""
    sys.stderr.write("safeboot.py: ")
    sys.stderr.write(msg)
    sys.stderr.write("\n")


def safeboot(source, target, env):
    # platformio estimates the amount of flash used to store the firmware. this
    # estimate is not accurate. we perform a final check on the firmware bin
    # size by comparing it against the respective partition size.
    max_size = 704 * 1024  # 704KB as set in the partition table, update accordingly
    fw_size = os.path.getsize(env.subst("$BUILD_DIR/${PROGNAME}.bin"))
    if fw_size > max_size:
        raise Exception("firmware binary too large: %d > %d" % (fw_size, max_size))

    status("Firmware size valid: %d <= %d" % (fw_size, max_size))

    status("SafeBoot firmware created: %s" % env.subst("$BUILD_DIR/${PROGNAME}.bin"))


def copy_safeboot_binary(source, target, env):
    # copy the safeboot binary to the project source folder
    print("Copying safeboot binary to project folder...")
    safeboot_bin_source = os.path.join(env.subst("$BUILD_DIR"), "%s.bin" % env.subst("${PROGNAME}"))
    print("Source: %s" % safeboot_bin_source)
    try:
        shutil.copy2(safeboot_bin_source, env.subst("$PROJECT_DIR"))
    except Exception as e:
        critical("Failed to copy safeboot binary: %s" % str(e))
        sys.exit(1)
    print("Copied to: %s" % env.subst("$PROJECT_DIR"))


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", safeboot)
env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", copy_safeboot_binary)



