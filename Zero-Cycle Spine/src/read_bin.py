#!/usr/bin/env python3
"""
Reference reader for NimBrace .BIN recordings.

Implements the format documented in FORMAT.md:
    86-byte header, then N packets of 92 bytes each.
    Packet = uint32 timestamp (ms since boot) + 22 float32, little-endian.

This module is deliberately minimal and depends only on numpy. It is the
canonical definition of how to get numbers out of the published recordings;
if this file and FORMAT.md ever disagree, FORMAT.md is wrong.

Usage
-----
    python read_bin.py FILE.BIN --summary
    python read_bin.py FILE.BIN --csv out.csv

    from read_bin import read_bin
    rec = read_bin("DATA_20260106_181927_18.BIN")
    rec["imu1"]["quat"]      # (N, 4) float64, [w, x, y, z]
    rec["t_s"]               # (N,)   seconds from first sample

SpineRebel Technology.
"""

from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

import numpy as np

# --- format constants (see FORMAT.md) --------------------------------------

HEADER_BYTES = 86
PACKET_BYTES = 92
N_FLOATS = 22

_PACKET_STRUCT = struct.Struct("<I22f")
assert _PACKET_STRUCT.size == PACKET_BYTES, "struct/packet size mismatch"

# float payload slices
_SLICES = {
    "imu1": {"quat": slice(0, 4), "accel": slice(4, 7), "gyro": slice(7, 10)},
    "imu2": {"quat": slice(10, 14), "accel": slice(14, 17), "gyro": slice(17, 20)},
}
_RESERVED = slice(20, 22)  # undocumented; carried through, not interpreted


class BinFormatError(ValueError):
    """Raised when a file cannot be interpreted as a NimBrace recording."""


def read_bin(path, strict: bool = False, warn=None) -> dict:
    """
    Read a .BIN recording into numpy arrays.

    Parameters
    ----------
    path : str or Path
    strict : bool
        If True, raise on a truncated trailing packet instead of discarding it.
    warn : callable or None
        Receives human-readable warning strings. Defaults to stderr.

    Returns
    -------
    dict with keys:
        header    : raw header bytes (86,)
        t_ms      : (N,) uint32,  milliseconds since firmware boot
        t_s       : (N,) float64, seconds relative to first sample
        fs_hz     : float, median sampling rate derived from timestamps
        imu1      : {"quat": (N,4), "accel": (N,3), "gyro": (N,3)}
        imu2      : same
        reserved  : (N,2) float32, payload indices 20-21 (uninterpreted)
        n_packets : int
        gaps      : list of (index, dt_ms) where dt exceeded 3 nominal intervals
    """
    if warn is None:
        def warn(msg):
            print(f"warning: {msg}", file=sys.stderr)

    path = Path(path)
    raw = path.read_bytes()

    if len(raw) < HEADER_BYTES + PACKET_BYTES:
        raise BinFormatError(
            f"{path.name}: {len(raw)} bytes is too short to contain a header "
            f"({HEADER_BYTES}) plus at least one packet ({PACKET_BYTES})."
        )

    header = raw[:HEADER_BYTES]
    body = raw[HEADER_BYTES:]

    n_packets, remainder = divmod(len(body), PACKET_BYTES)
    if remainder:
        msg = (f"{path.name}: trailing {remainder} bytes do not form a whole "
               f"packet (truncated recording, likely power loss)")
        if strict:
            raise BinFormatError(msg)
        warn(msg + " — discarded")
        body = body[: n_packets * PACKET_BYTES]

    # Vectorised decode. dtype mirrors the struct exactly; '<' forces
    # little-endian regardless of host byte order.
    dt = np.dtype([("t_ms", "<u4"), ("f", "<f4", (N_FLOATS,))])
    arr = np.frombuffer(body, dtype=dt, count=n_packets)

    t_ms = arr["t_ms"].astype(np.uint32)
    floats = arr["f"]

    out = {
        "path": str(path),
        "header": header,
        "t_ms": t_ms,
        "n_packets": int(n_packets),
        "reserved": np.array(floats[:, _RESERVED], dtype=np.float32),
    }

    for imu, fields in _SLICES.items():
        out[imu] = {
            name: np.array(floats[:, sl], dtype=np.float64)
            for name, sl in fields.items()
        }

    # --- timing ----------------------------------------------------------
    d_ms = np.diff(t_ms.astype(np.int64))

    if n_packets > 1 and np.any(d_ms <= 0):
        n_bad = int(np.sum(d_ms <= 0))
        warn(f"{path.name}: {n_bad} non-monotonic timestamp step(s) — "
             f"possible firmware counter wrap or interleaved write")

    median_dt = float(np.median(d_ms)) if n_packets > 1 else float("nan")
    out["fs_hz"] = 1000.0 / median_dt if median_dt > 0 else float("nan")
    out["t_s"] = (t_ms.astype(np.float64) - float(t_ms[0])) / 1000.0

    gaps = []
    if n_packets > 1 and median_dt > 0:
        thresh = 3.0 * median_dt
        for i in np.flatnonzero(d_ms > thresh):
            gaps.append((int(i), int(d_ms[i])))
    out["gaps"] = gaps

    # --- sanity: quaternion norms ---------------------------------------
    for imu in ("imu1", "imu2"):
        q = out[imu]["quat"]
        norms = np.linalg.norm(q, axis=1)
        finite = np.isfinite(norms) & (norms > 0)
        out[imu]["valid"] = bool(finite.mean() > 0.5)
        if finite.any():
            off = np.abs(norms[finite] - 1.0)
            if np.median(off) > 0.02:
                warn(f"{path.name}: {imu} quaternion norms deviate from unity "
                     f"(median |‖q‖-1| = {np.median(off):.3f}) — check "
                     f"byte order and payload offsets")

    return out


def summarise(rec: dict) -> str:
    lines = []
    name = Path(rec["path"]).name
    dur = rec["t_s"][-1] if rec["n_packets"] else 0.0
    lines.append(f"{name}")
    lines.append(f"  packets      {rec['n_packets']}")
    lines.append(f"  duration     {dur:.1f} s  ({dur / 60.0:.1f} min)")
    lines.append(f"  sample rate  {rec['fs_hz']:.1f} Hz (median-derived)")
    for imu in ("imu1", "imu2"):
        state = "ok" if rec[imu]["valid"] else "INVALID"
        q = rec[imu]["quat"]
        lines.append(f"  {imu}         {state}   "
                     f"‖q‖ median {np.median(np.linalg.norm(q, axis=1)):.4f}")
    if rec["gaps"]:
        total = sum(g[1] for g in rec["gaps"])
        lines.append(f"  gaps         {len(rec['gaps'])} "
                     f"(total {total / 1000.0:.1f} s missing)")
        for i, dt_ms in rec["gaps"][:5]:
            lines.append(f"                 at packet {i}: {dt_ms} ms")
        if len(rec["gaps"]) > 5:
            lines.append(f"                 ... and {len(rec['gaps']) - 5} more")
    else:
        lines.append(f"  gaps         none")
    return "\n".join(lines)


def to_csv(rec: dict, out_path) -> None:
    """Flatten to CSV. Verbose — intended for spot-checking, not analysis."""
    cols = ["t_s"]
    data = [rec["t_s"]]
    for imu in ("imu1", "imu2"):
        for field, names in (("quat", "wxyz"), ("accel", "xyz"), ("gyro", "xyz")):
            block = rec[imu][field]
            for j, ax in enumerate(names):
                cols.append(f"{imu}_{field}_{ax}")
                data.append(block[:, j])
    mat = np.column_stack(data)
    np.savetxt(out_path, mat, delimiter=",", header=",".join(cols),
               comments="", fmt="%.6g")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("files", nargs="+", type=Path, help=".BIN file(s)")
    ap.add_argument("--summary", action="store_true", help="print a summary")
    ap.add_argument("--csv", type=Path, default=None,
                    help="write flattened CSV (single input file only)")
    ap.add_argument("--strict", action="store_true",
                    help="fail on truncated trailing packet")
    args = ap.parse_args(argv)

    if args.csv and len(args.files) > 1:
        ap.error("--csv accepts a single input file")

    rc = 0
    for f in args.files:
        try:
            rec = read_bin(f, strict=args.strict)
        except (BinFormatError, OSError) as e:
            print(f"error: {e}", file=sys.stderr)
            rc = 1
            continue
        if args.summary or not args.csv:
            print(summarise(rec))
            print()
        if args.csv:
            to_csv(rec, args.csv)
            print(f"wrote {args.csv}")
    return rc


if __name__ == "__main__":
    sys.exit(main())
