# SPDX-License-Identifier: MIT
#
# Copyright (C) 2023-2024 Mathieu Carbou
#
Import("env")
import os
import sys

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


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", safeboot)
