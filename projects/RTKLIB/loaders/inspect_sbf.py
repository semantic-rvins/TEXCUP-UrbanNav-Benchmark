#!/usr/bin/env python3
"""Check local Septentrio SBF framing, CRCs, timestamps and MeasEpoch layout.

Every byte is scanned without loading the entire file into a Python bytes
object. Strict validation rejects skipped bytes, damaged/truncated blocks,
scrambled or malformed MeasEpoch records, and files without observation
blocks decoded by the pinned RTKLIB. Passing this check does not establish
measurement accuracy, antenna physical identity, or RINEX reproducibility.

Format authority: RTKLIB v2.5.1 commit
62d4677ed8425a4e2748c6d390b500d1afb493fc, src/rcv/septentrio.c:
sbf_checksum(), decode_sbf(), decode_measepoch(), sig_tbl, and block IDs.
"""

import argparse
import binascii
from collections import Counter, defaultdict
from datetime import datetime, timedelta
import json
import mmap
from pathlib import Path
import struct
import sys


SYNC = b"$@"
GPS_ORIGIN = datetime(1980, 1, 6)
WEEK_MS = 604800000
MAX_EXAMPLES = 50
BLOCK_NAMES = {
    4000: "MeasEpochExtra", 4027: "MeasEpoch", 4098: "MeasFullRange",
    4109: "Meas3Ranges", 4110: "Meas3CN0", 4111: "Meas3Doppler",
    4112: "Meas3PP", 4113: "Meas3MP", 5922: "EndOfMeas",
    5944: "GenMeasEpoch",
}
# Only these main observation blocks are decoded by this RTKLIB revision.
OBSERVATION_BLOCKS = {4027, 4109}
# Exact mapping from the pinned septentrio.c sig_tbl (not receiver marketing
# names). Type 31 expands using the documented Info field before this lookup.
SIGNALS = {
    0: "G:1C", 1: "G:1W", 2: "G:2W", 3: "G:2L", 4: "G:5Q",
    5: "G:1L", 6: "J:1C", 7: "J:2L", 8: "R:1C", 9: "R:1P",
    10: "R:2P", 11: "R:2C", 12: "R:3Q", 13: "C:1P",
    14: "C:5P", 15: "I:5A", 17: "E:1C", 19: "E:6C",
    20: "E:5Q", 21: "E:7Q", 22: "E:8Q", 24: "S:1C",
    25: "S:5I", 26: "J:5Q", 27: "J:6L", 28: "C:2I",
    29: "C:7I", 30: "C:6I", 32: "J:1L", 33: "J:1Z",
    34: "C:7D", 36: "I:9A", 38: "J:1E", 39: "J:5P",
}


def crc16(data):
    """SBF CRC-16/XMODEM: polynomial 0x1021, initial 0, no final XOR."""
    return binascii.crc_hqx(data, 0)


def time_extent(bounds):
    if bounds is None:
        return None
    return {
        "scale": "GPST",
        "first": (GPS_ORIGIN + timedelta(milliseconds=bounds[0])).isoformat(
            timespec="milliseconds"),
        "last": (GPS_ORIGIN + timedelta(milliseconds=bounds[1])).isoformat(
            timespec="milliseconds"),
        "duration_seconds": (bounds[1] - bounds[0]) / 1000,
    }


def extend(bounds, value):
    return [value, value] if bounds is None else [min(bounds[0], value), max(bounds[1], value)]


def inspect_sbf(path):
    result = {
        "valid": False,
        "path": str(path.resolve()),
        "validation_scope": "Local SBF integrity and supported observation-block presence only",
        "crc_algorithm": "CRC-16/XMODEM, initial 0, bytes 4 through block end; CRC field little-endian",
        "format_authority": "RTKLIB v2.5.1 62d4677ed8425a4e2748c6d390b500d1afb493fc src/rcv/septentrio.c",
    }
    errors = []
    counts = Counter()
    block_counts = Counter()
    revisions = Counter()
    antenna_primary = Counter()
    antenna_secondary = Counter()
    antenna_signals = defaultdict(Counter)
    meas3_antennas = Counter()
    skipped = []
    truncated_details = []
    preceding_valid_frame = None
    bounds = observation_bounds = None
    valid_bytes = 0

    def fail(message):
        counts["error_count"] += 1
        if len(errors) < MAX_EXAMPLES:
            errors.append(message)

    def skip(first, last):
        if first == last:
            return
        counts["skipped_bytes"] += last - first
        counts["skipped_range_count"] += 1
        if len(skipped) < MAX_EXAMPLES:
            skipped.append({"offset": first, "bytes": last - first})

    def signal_record(ant, signal):
        antenna_signals[str(ant)][SIGNALS.get(signal, f"unmapped_sbf_signal_{signal}")] += 1

    def inspect_measepoch(data, first, length):
        """Count all antenna records; offsets and sizes follow decode_measepoch."""
        end = first + length
        if length < 20:
            raise ValueError("MeasEpoch shorter than its 20-byte header")
        n1, len1, len2, common_flags = struct.unpack_from("<BBBB", data, first + 14)
        if common_flags & 0x80:
            counts["scrambled_measepoch_blocks"] += 1
            raise ValueError("MeasEpoch has scrambled measurements")
        if n1 and len1 < 20:
            raise ValueError("MeasEpoch Type1 subblock length below 20")
        cursor = first + 20
        primary = Counter()
        secondary = Counter()
        signals = []
        for _ in range(n1):
            if cursor + len1 > end:
                raise ValueError("MeasEpoch Type1 subblock exceeds enclosing block")
            ant = data[cursor + 1] >> 5
            sig = data[cursor + 1] & 31
            if sig == 31:
                sig = (data[cursor + 18] >> 3) + 32
            n2 = data[cursor + 19]
            if n2 and len2 < 12:
                raise ValueError("MeasEpoch Type2 subblock length below 12")
            if cursor + len1 + n2 * len2 > end:
                raise ValueError("MeasEpoch Type2 subblocks exceed enclosing block")
            primary[str(ant)] += 1
            signals.append((ant, sig))
            cursor += len1
            for _ in range(n2):
                ant = data[cursor] >> 5
                sig = data[cursor] & 31
                if sig == 31:
                    sig = (data[cursor + 5] >> 3) + 32
                secondary[str(ant)] += 1
                signals.append((ant, sig))
                cursor += len2
        # SBF blocks are aligned to four bytes. No undisclosed records should
        # follow the declared subblocks, except the final 0-3 padding bytes.
        if end - cursor > 3:
            raise ValueError("MeasEpoch has excess bytes after declared subblocks")
        antenna_primary.update(primary)
        antenna_secondary.update(secondary)
        for ant, sig in signals:
            signal_record(ant, sig)
        counts["parsed_measepoch_blocks"] += 1
        counts["measepoch_type1_records"] += sum(primary.values())
        counts["measepoch_type2_records"] += sum(secondary.values())

    with path.open("rb") as stream:
        size = stream.seek(0, 2)
        result["bytes"] = size
        if size == 0:
            fail("Empty input")
        else:
            with mmap.mmap(stream.fileno(), 0, access=mmap.ACCESS_READ) as data:
                cursor = 0
                while cursor < size:
                    first = data.find(SYNC, cursor)
                    if first < 0:
                        skip(cursor, size)
                        break
                    skip(cursor, first)
                    if size - first < 8:
                        counts["truncated_headers"] += 1
                        fail(f"Offset {first}: truncated SBF header ({size - first}/8 bytes)")
                        skip(first, size)
                        break
                    expected_crc, identifier, length = struct.unpack_from("<HHH", data, first + 2)
                    block_id, revision = identifier & 0x1fff, identifier >> 13
                    if length < 8 or length % 4:
                        counts["invalid_length_candidates"] += 1
                        fail(f"Offset {first}: invalid block length {length}; require >=8 and divisible by 4")
                        skip(first, first + 1)
                        cursor = first + 1
                        continue
                    if first + length > size:
                        counts["truncated_blocks"] += 1
                        fail(f"Offset {first}: truncated block {block_id} ({size - first}/{length} bytes)")
                        detail = {"offset": first, "block_id": block_id,
                                  "revision": revision, "remaining_bytes": size - first,
                                  "declared_bytes": length,
                                  "preceding_crc_valid_frame": preceding_valid_frame}
                        if size - first >= 14:
                            tow, week = struct.unpack_from("<IH", data, first + 8)
                            if tow < WEEK_MS and week != 0xffff:
                                detail["unverified_header_timestamp"] = {
                                    "scale": "GPST", "wnc": week, "tow_ms": tow,
                                    "calendar": (GPS_ORIGIN + timedelta(weeks=week, milliseconds=tow)).isoformat(timespec="milliseconds"),
                                    "crc_validated": False,
                                }
                        truncated_details.append(detail)
                        skip(first, size)
                        break
                    # Each copy is at most the uint16 block length (65,532
                    # aligned bytes), keeping memory bounded for large files.
                    actual_crc = crc16(data[first + 4:first + length])
                    if actual_crc != expected_crc:
                        counts["crc_error_candidates"] += 1
                        fail(f"Offset {first}: CRC mismatch for block {block_id}; stored={expected_crc:04x}, computed={actual_crc:04x}")
                        # An invalid frame length cannot be trusted. Search
                        # from the following byte to recover later valid frames.
                        skip(first, first + 1)
                        cursor = first + 1
                        continue
                    cursor = first + length
                    valid_bytes += length
                    counts["valid_blocks"] += 1
                    block_counts[str(block_id)] += 1
                    revisions[f"{block_id}:{revision}"] += 1
                    preceding_valid_frame = {"offset": first, "block_id": block_id,
                                             "bytes": length, "timestamp": None}
                    if block_id in OBSERVATION_BLOCKS:
                        counts["supported_observation_blocks"] += 1
                    if length >= 14:
                        tow, week = struct.unpack_from("<IH", data, first + 8)
                        if tow == 0xffffffff or week == 0xffff:
                            counts["unavailable_timestamps"] += 1
                            if block_id in OBSERVATION_BLOCKS:
                                fail(f"Offset {first}: observation block lacks a valid TOW/WNc timestamp")
                        elif tow >= WEEK_MS:
                            counts["invalid_timestamps"] += 1
                            fail(f"Offset {first}: TOW {tow} ms lies outside a GPS week")
                        else:
                            timestamp = week * WEEK_MS + tow
                            preceding_valid_frame["timestamp"] = {
                                "scale": "GPST", "wnc": week, "tow_ms": tow,
                                "calendar": (GPS_ORIGIN + timedelta(milliseconds=timestamp)).isoformat(timespec="milliseconds"),
                                "crc_validated": True,
                            }
                            bounds = extend(bounds, timestamp)
                            counts["valid_timestamps"] += 1
                            if block_id in OBSERVATION_BLOCKS:
                                observation_bounds = extend(observation_bounds, timestamp)
                    elif block_id in OBSERVATION_BLOCKS:
                        fail(f"Offset {first}: observation block is shorter than the common time header")
                    if block_id == 4027:
                        try:
                            inspect_measepoch(data, first, length)
                        except ValueError as exc:
                            counts["malformed_measepoch_blocks"] += 1
                            fail(f"Offset {first}: {exc}")
                    elif block_id == 4109:
                        # decode_meas3ranges uses p=block+8; Misc at p+10
                        # carries the antenna index in bits 0..2. Full compressed
                        # Meas3 payload decoding remains RTKLIB's responsibility.
                        if length < 20:
                            fail(f"Offset {first}: Meas3Ranges is shorter than its common fields")
                        else:
                            meas3_antennas[str(data[first + 18] & 7)] += 1
                if counts["skipped_bytes"]:
                    fail(f"{counts['skipped_bytes']} bytes do not belong to CRC-valid SBF frames")
    if not counts["supported_observation_blocks"]:
        fail("No RTKLIB-supported main observation blocks (4027 MeasEpoch or 4109 Meas3Ranges); epoch markers alone are insufficient")
    if block_counts.get("4027") and not counts["measepoch_type1_records"] and not block_counts.get("4109"):
        fail("MeasEpoch blocks contain no Type1 measurement records")
    result.update({
        "valid": counts["error_count"] == 0,
        "errors": errors,
        "error_count": counts["error_count"],
        "errors_truncated": counts["error_count"] > len(errors),
        "valid_frame_bytes": valid_bytes,
        "statistics": {name: counts[name] for name in (
            "valid_blocks", "supported_observation_blocks", "parsed_measepoch_blocks",
            "measepoch_type1_records", "measepoch_type2_records", "valid_timestamps",
            "unavailable_timestamps", "invalid_timestamps", "invalid_length_candidates",
            "crc_error_candidates", "truncated_headers", "truncated_blocks",
            "scrambled_measepoch_blocks", "malformed_measepoch_blocks", "skipped_bytes",
            "skipped_range_count")},
        "skipped_ranges": skipped,
        "skipped_ranges_truncated": counts["skipped_range_count"] > len(skipped),
        "truncated_block_details": truncated_details,
        "valid_block_counts": dict(sorted(block_counts.items(), key=lambda item: int(item[0]))),
        "measurement_and_epoch_marker_counts": {
            f"{block_id}:{name}": block_counts[str(block_id)]
            for block_id, name in BLOCK_NAMES.items()
        },
        "block_id_revision_counts": dict(sorted(revisions.items())),
        "all_valid_timestamp_extent": time_extent(bounds),
        "observation_timestamp_extent": time_extent(observation_bounds),
        "measepoch_antennas": {
            "type1_records": dict(sorted(antenna_primary.items())),
            "type2_records": dict(sorted(antenna_secondary.items())),
            "signal_record_counts": {ant: dict(sorted(signals.items()))
                                     for ant, signals in sorted(antenna_signals.items())},
            "index_meanings": {"0": "main", "1": "AUX1", "2": "AUX2"},
        },
        "meas3ranges_antenna_block_counts": dict(sorted(meas3_antennas.items())),
        "limitations": [
            "GPST calendar timestamps are reported directly, without conversion to UTC.",
            "Antenna indices do not prove a receiver's physical antenna identity.",
            "Signal record counts describe encoded records, not finite usable pseudorange/phase availability.",
            "CRC/frame validation does not prove source authenticity, positioning accuracy, or publisher completeness.",
            "Compressed Meas3 payload layout and observables require RTKLIB conversion followed by RINEX validation.",
        ],
    })
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="original local SBF file")
    parser.add_argument("--output", type=Path, help="write JSON report here")
    args = parser.parse_args()
    try:
        result = inspect_sbf(args.input)
    except (OSError, ValueError, OverflowError) as exc:
        result = {"valid": False, "path": str(args.input), "error_count": 1, "errors": [str(exc)]}
    output = json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + "\n"
    if args.output:
        with args.output.open("x") as stream:
            stream.write(output)
        print(json.dumps({"valid": result["valid"], "report": str(args.output),
                          "error_count": result["error_count"]}))
    else:
        print(output, end="")
    return 0 if result["valid"] else 1


if __name__ == "__main__":
    sys.exit(main())
