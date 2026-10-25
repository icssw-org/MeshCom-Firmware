#!/usr/bin/env python3
"""Compress ESP32 app images into `<name>.autoupdate` release assets (AU-D11, #1187).

Each output is one standard zlib stream (RFC 1950): header 0x78 0xDA (deflate,
32 KB window, best compression), deflate data, Adler-32. That is exactly what
the ESP32 ROM `tinfl` inflates with TINFL_FLAG_PARSE_ZLIB_HEADER; the 32 KB
window (wbits 15) equals TINFL_LZ_DICT_SIZE, so Safeboot needs no larger
dictionary than the ROM provides.

    make_autoupdate.py firmware.bin [more.bin ...] [--out-dir DIR] [--check]
               [--manifest out.json] [--force]

Prints one line per input: name, size, zsize, ratio, sha256 of the .autoupdate.
--check re-inflates every output and compares it byte-exact with the input.
Inputs whose first byte is not 0xE9 (ESP32 app image magic) are refused
unless --force. Exit status: 0 ok, 1 an input failed, 2 usage error.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
import zlib
from pathlib import Path
from typing import Any

ESP_IMAGE_MAGIC = 0xE9
ZLIB_WBITS = 15  # 32 KB window == TINFL_LZ_DICT_SIZE
ZLIB_LEVEL = 9
ZLIB_HEADER = b"\x78\xda"  # deflate, 32 KB window, level 9 ("best")


class MakeAutoupdateError(Exception):
    """An input was refused or failed verification."""


def sha256_hex(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def is_esp_image(data: bytes) -> bool:
    return len(data) > 0 and data[0] == ESP_IMAGE_MAGIC


def compress(data: bytes) -> bytes:
    """Standard zlib stream, level 9, 32 KB window."""
    co = zlib.compressobj(ZLIB_LEVEL, zlib.DEFLATED, ZLIB_WBITS)
    return co.compress(data) + co.flush()


def inflate(zdata: bytes) -> bytes:
    """Inflate with a 32 KB window and a zlib header (what tinfl accepts)."""
    do = zlib.decompressobj(ZLIB_WBITS)
    out = do.decompress(zdata) + do.flush()
    if not do.eof or do.unused_data:
        raise MakeAutoupdateError("zlib stream truncated or has trailing bytes")
    return out


def verify(zdata: bytes, original: bytes) -> None:
    if zdata[:2] != ZLIB_HEADER:
        raise MakeAutoupdateError(f"unexpected zlib header {zdata[:2].hex()}")
    try:
        back = inflate(zdata)
    except zlib.error as exc:
        raise MakeAutoupdateError(f"inflate failed: {exc}") from exc
    if back != original:
        raise MakeAutoupdateError("round trip differs from input")


def asset_name(src: Path) -> str:
    """heltec.bin -> heltec.autoupdate (no ".bin", so nobody flashes the zlib file)."""
    stem = src.name[:-4] if src.name.endswith(".bin") else src.name
    return stem + ".autoupdate"


def process(src: Path, out_dir: Path, *, check: bool, force: bool) -> dict[str, Any]:
    """Compress one input, write the asset, return its manifest entry."""
    try:
        data = src.read_bytes()
    except OSError as exc:
        raise MakeAutoupdateError(f"cannot read: {exc.strerror}") from exc
    if not data:
        raise MakeAutoupdateError("empty input")
    if not force and not is_esp_image(data):
        raise MakeAutoupdateError(
            f"not an ESP32 app image (first byte 0x{data[0]:02x}, expected 0xe9); use --force"
        )
    zdata = compress(data)
    if check:
        verify(zdata, data)
    out = out_dir / asset_name(src)
    out.write_bytes(zdata)
    if check:
        # Verify what actually landed on disk, not just the in-memory buffer.
        verify(out.read_bytes(), data)
    return {
        "asset": out.name,
        "size": len(data),
        "zsize": len(zdata),
        "sha256_bin": sha256_hex(data),
        "sha256_zz": sha256_hex(zdata),
    }


def parse_args(argv: list[str] | None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="Write <name>.autoupdate (zlib, level 9, 32 KB window) for ESP32 app images."
    )
    ap.add_argument("inputs", nargs="+", type=Path, metavar="in.bin")
    ap.add_argument("--out-dir", type=Path, default=None, help="default: next to each input")
    ap.add_argument("--check", action="store_true", help="re-inflate and compare byte-exact")
    ap.add_argument("--manifest", type=Path, default=None, help="write a JSON listing here")
    ap.add_argument("--force", action="store_true", help="accept inputs without the 0xE9 magic")
    return ap.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.out_dir is not None:
        args.out_dir.mkdir(parents=True, exist_ok=True)
    entries: list[dict[str, Any]] = []
    failed = 0
    for src in args.inputs:
        out_dir = args.out_dir if args.out_dir is not None else src.parent
        try:
            e = process(src, out_dir, check=args.check, force=args.force)
        except MakeAutoupdateError as exc:
            print(f"FAIL {src.name}: {exc}", file=sys.stderr)
            failed += 1
            continue
        entries.append(e)
        size, zsize = e["size"], e["zsize"]
        tail = "  check=ok" if args.check else ""
        print(f"{e['asset']} {size} {zsize} {zsize / size:.3f} {e['sha256_zz']}{tail}")
    if args.manifest is not None and entries:
        args.manifest.write_text(json.dumps(entries, indent=2) + "\n")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
