#!/usr/bin/env python3
"""Validate RINEX 3 observation data without changing measurement fields.

Usage: validate_rinex.py INPUT [--profile PROFILE.json] [--allow-subset]
                          [--output RESULT.json]

Profiles specify time.scale=GPST, start/end_inclusive ISO datetimes,
tolerance_seconds, interval_seconds, expected_epochs, signals by system,
and observables (C/L/D/S). --allow-subset permits missing expected header
types/systems; it does not relax time, interval, or epoch-count requirements.
RINEX fields are inspected at their original 16-character boundaries.
"""

import argparse
from collections import Counter, defaultdict
from datetime import datetime, timedelta, timezone
import json
import math
from pathlib import Path
import re
import statistics
import sys


SATELLITE = re.compile(r"[GRECJSI]\d{2}$")
OBSERVABLE = re.compile(r"[CLDS][1-9][A-Z]$")
MAX_ERRORS = 100


def iso_time(value):
    parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    # UTC is used as a calendar arithmetic carrier, not a GPST->UTC conversion.
    if parsed.tzinfo is not None and parsed.utcoffset() != timedelta(0):
        raise ValueError("Profile GPST calendar times must have no timezone or Z")
    return parsed.replace(tzinfo=timezone.utc).timestamp()


def epoch_time(parts):
    year, month, day, hour, minute = map(int, parts[1:6])
    second = float(parts[6])
    if not math.isfinite(second) or not 0 <= second < 61:
        raise ValueError("Invalid epoch second")
    if not (0 <= hour < 24 and 0 <= minute < 60):
        raise ValueError("Invalid epoch hour/minute")
    value = datetime(year, month, day, hour, minute, tzinfo=timezone.utc).timestamp()
    return value + second


def display_time(timestamp):
    return datetime.fromtimestamp(timestamp, timezone.utc).isoformat(
        timespec="microseconds"
    ).replace("+00:00", "")


def validate(path, profile=None, allow_subset=False):
    errors = []
    error_count = 0

    def fail(message):
        nonlocal error_count
        error_count += 1
        if len(errors) < MAX_ERRORS:
            errors.append(message)

    result = {"valid": False, "path": str(path.resolve()), "errors": errors}
    raw = path.read_bytes()
    result["bytes"] = len(raw)
    try:
        lines = raw.decode("ascii").splitlines()
    except UnicodeDecodeError as exc:
        fail(f"RINEX must be ASCII text: {exc}")
        result["error_count"] = error_count
        return result
    if not lines:
        fail("Empty input")
        result["error_count"] = error_count
        return result
    if lines[0][60:80].strip() != "RINEX VERSION / TYPE":
        fail("Missing RINEX VERSION / TYPE at columns 61-80; LFS pointers are not data")
    try:
        version = float(lines[0][:9])
    except ValueError:
        version = 0
    result["rinex_version"] = version
    if not 3 <= version < 4:
        fail("Validator requires RINEX 3 observation data")
    if lines[0][20:21] != "O":
        fail("Input is not an observation file")

    obs_types = {}
    declared_counts = {}
    current_system = None
    header_end = None
    header_times = {}
    header_interval = None
    i = 0
    for i, line in enumerate(lines):
        label = line[60:80].strip()
        if label == "SYS / # / OBS TYPES":
            if line[:1] != " ":
                current_system = line[:1]
                if current_system not in "GRECJSI":
                    fail(f"Line {i+1}: unknown system {current_system!r}")
                if current_system in obs_types:
                    fail(f"Line {i+1}: repeated system observation header")
                try:
                    declared_counts[current_system] = int(line[3:6])
                except ValueError:
                    declared_counts[current_system] = 0
                    fail(f"Line {i+1}: invalid observation type count")
                obs_types[current_system] = []
            if current_system is None:
                fail(f"Line {i+1}: observation type continuation lacks a system")
            else:
                obs_types[current_system].extend(line[7:60].split())
        elif label in ("TIME OF FIRST OBS", "TIME OF LAST OBS"):
            header_times[label] = {"calendar": line[:43].split(), "scale": line[48:51].strip()}
        elif label == "INTERVAL":
            try:
                header_interval = float(line[:10])
                if not math.isfinite(header_interval) or header_interval <= 0:
                    raise ValueError()
            except ValueError:
                fail(f"Line {i+1}: invalid header interval")
        elif label == "END OF HEADER":
            header_end = i
            break
    if header_end is None:
        fail("Missing END OF HEADER")
        result["error_count"] = error_count
        return result
    if not obs_types:
        fail("Missing system observation type declarations")
    if "TIME OF FIRST OBS" not in header_times:
        fail("Missing TIME OF FIRST OBS header")
    for system, types in obs_types.items():
        if declared_counts[system] != len(types) or len(types) == 0:
            fail(f"{system}: declared observation count does not match header fields")
        if len(types) != len(set(types)):
            fail(f"{system}: duplicated observation types")
        for kind in types:
            if not OBSERVABLE.fullmatch(kind):
                fail(f"{system}: unsupported observation type {kind!r}")

    tolerance = 0.005
    expected_interval = None
    expected_start = expected_end = None
    expected_epochs = None
    expected_types = {}
    if profile is not None:
        time_options = profile.get("time", {})
        if time_options.get("scale", "GPST") != "GPST":
            fail("Only GPST profile time scale is supported")
        tolerance = float(time_options.get("tolerance_seconds", 0.005))
        if not math.isfinite(tolerance) or tolerance <= 0:
            raise ValueError("Profile time tolerance must be finite and positive")
        expected_interval = profile.get("interval_seconds")
        if expected_interval is not None:
            expected_interval = float(expected_interval)
            if not math.isfinite(expected_interval) or expected_interval <= 0:
                raise ValueError("Profile interval must be finite and positive")
        expected_epochs = profile.get("expected_epochs")
        if expected_epochs is not None and (not isinstance(expected_epochs, int) or expected_epochs <= 0):
            raise ValueError("Profile expected_epochs must be a positive integer")
        if time_options.get("start"):
            expected_start = iso_time(time_options["start"])
        if time_options.get("end_inclusive"):
            expected_end = iso_time(time_options["end_inclusive"])
        if expected_start is not None and expected_end is not None and expected_end < expected_start:
            raise ValueError("Profile end precedes start")
        observables = profile.get("observables", ["C", "L", "D", "S"])
        if not observables or any(t not in ("C", "L", "D", "S") for t in observables):
            raise ValueError("Profile observables must contain only C/L/D/S")
        for system, signals in profile.get("signals", {}).items():
            if not signals or system not in "GRECJSI":
                raise ValueError("Profile contains invalid/empty signal system")
            expected_types[system] = {t + s for s in signals for t in observables}
        if not expected_types:
            raise ValueError("Profile must specify nonempty signals")
        if not allow_subset and set(obs_types) != set(expected_types):
            fail("Header GNSS systems differ from profile: "
                 f"actual={sorted(obs_types)}, expected={sorted(expected_types)}")
        if allow_subset and not set(obs_types).issubset(expected_types):
            fail("Header contains GNSS systems outside profile")
        for system, types in obs_types.items():
            permitted = expected_types.get(system, set())
            actual = set(types)
            if (allow_subset and not actual.issubset(permitted)) or (not allow_subset and actual != permitted):
                fail(f"{system}: observation types differ from profile; "
                     f"missing={sorted(permitted-actual)}, extra={sorted(actual-permitted)}")
        for name, value in header_times.items():
            if value["scale"] not in ("GPS", "GPST"):
                fail(f"{name}: expected GPST/GPS header time scale")
        if header_interval is not None and expected_interval is not None and abs(header_interval-expected_interval) > tolerance:
            fail("Header interval differs from profile")

    measurement_counts = defaultdict(Counter)
    nonzero_counts = defaultdict(Counter)
    blank_counts = defaultdict(Counter)
    lli_counts = defaultdict(Counter)
    ssi_counts = defaultdict(Counter)
    satellite_records = Counter()
    epoch_flags = Counter()
    event_flags = Counter()
    epoch_satellite_counts = Counter()
    clock_offsets = []
    epochs = []
    observation_records = 0
    cycle_slip_records = 0
    i = header_end + 1
    while i < len(lines):
        line = lines[i]
        if not line.startswith(">"):
            fail(f"Line {i+1}: expected epoch record")
            i += 1
            continue
        parts = line.split()
        if len(parts) < 9:
            fail(f"Line {i+1}: malformed epoch record")
            i += 1
            continue
        try:
            timestamp = epoch_time(parts)
            flag, count = int(parts[7]), int(parts[8])
            if not 0 <= flag <= 6 or count < 0:
                raise ValueError("invalid epoch flag/count")
            if len(parts) > 9:
                clock = float(parts[9].replace("D", "E"))
                if not math.isfinite(clock):
                    raise ValueError("nonfinite receiver clock offset")
                clock_offsets.append(clock)
        except (ValueError, OverflowError) as exc:
            fail(f"Line {i+1}: invalid epoch: {exc}")
            i += 1
            continue
        i += 1
        if flag in (2, 3, 4, 5):
            event_flags[str(flag)] += 1
            if i + count > len(lines):
                fail("Event block is truncated")
                i = len(lines)
            else:
                # Header-changing events need a stateful parser; do not silently
                # claim validation using an obsolete observation type map.
                for event_line in lines[i:i+count]:
                    if event_line[60:80].strip() == "SYS / # / OBS TYPES":
                        fail("Header-changing event is unsupported; observation map needs review")
                i += count
            continue
        if flag in (0, 1):
            epochs.append(timestamp)
            epoch_flags[str(flag)] += 1
            epoch_satellite_counts[str(count)] += 1
            if count == 0:
                fail(f"Epoch {display_time(timestamp)} has zero satellites")
        else:
            event_flags[str(flag)] += 1
        seen_satellites = set()
        for _ in range(count):
            if i >= len(lines) or lines[i].startswith(">"):
                fail(f"Epoch {display_time(timestamp)}: fewer satellite records than declared")
                break
            record_line_number = i + 1
            record = lines[i]
            satellite = record[:3]
            if not SATELLITE.fullmatch(satellite):
                fail(f"Line {i+1}: invalid satellite identifier {satellite!r}")
                i += 1
                continue
            if satellite in seen_satellites:
                fail(f"Epoch {display_time(timestamp)}: duplicated satellite {satellite}")
            seen_satellites.add(satellite)
            system = satellite[0]
            types = obs_types.get(system, [])
            if not types:
                fail(f"Line {i+1}: {satellite} lacks a system header")
            fields = record[3:]
            i += 1
            # RTKLIB writes long lines; also accept standard space-prefixed
            # continuations, padding every physical field to 16 columns.
            while i < len(lines) and lines[i].startswith("   "):
                fields += " " * ((-len(fields)) % 16) + lines[i][3:]
                i += 1
            width = 16 * len(types)
            if len(fields) > width and fields[width:].strip():
                fail(f"Line {record_line_number}: fields exceed {system} header count")
            fields = fields[:width].ljust(width)
            if flag == 6:
                cycle_slip_records += 1
            else:
                observation_records += 1
                satellite_records[satellite] += 1
            for j, kind in enumerate(types):
                field = fields[16*j:16*(j+1)]
                value_text, lli, ssi = field[:14].strip(), field[14], field[15]
                if lli not in " 01234567":
                    fail(f"Line {record_line_number}: invalid LLI for {satellite}/{kind}")
                if ssi not in " 0123456789":
                    fail(f"Line {record_line_number}: invalid SSI for {satellite}/{kind}")
                if flag != 6:
                    if lli != " ":
                        lli_counts[system+":"+kind][lli] += 1
                    if ssi != " ":
                        ssi_counts[system+":"+kind][ssi] += 1
                if not value_text:
                    if flag != 6:
                        blank_counts[system][kind] += 1
                    continue
                try:
                    value = float(value_text.replace("D", "E"))
                    if not math.isfinite(value):
                        raise ValueError("nonfinite value")
                except ValueError:
                    fail(f"Line {record_line_number}: invalid numeric value for {satellite}/{kind}")
                    continue
                if flag != 6:
                    measurement_counts[system][kind] += 1
                    if value != 0:
                        nonzero_counts[system][kind] += 1

    if not epochs or not observation_records:
        fail("No observation epochs/satellite records; empty conversions do not pass")
    if not sum(sum(c.values()) for c in measurement_counts.values()):
        fail("No finite nonblank measurements")
    required_signals = (
        {s: sorted({t[1:] for t in ts}) for s, ts in obs_types.items()}
        if profile is None or allow_subset else profile["signals"]
    )
    for system, signals in required_signals.items():
        for signal in signals:
            for observable in ("C", "L"):
                kind = observable + signal
                if not nonzero_counts[system][kind]:
                    fail(f"{system}/{signal}: no finite nonzero {observable} measurements")

    differences = [b-a for a, b in zip(epochs, epochs[1:])]
    duplicate_count = sum(abs(d) <= 1e-7 for d in differences)
    non_increasing_count = sum(d <= 0 for d in differences)
    if non_increasing_count:
        fail(f"Epochs are not strictly increasing: {non_increasing_count} transitions")
    gap_count = 0
    interval_mismatch_count = 0
    grid_mismatch_count = 0
    if expected_interval is not None:
        gap_count = sum(d > expected_interval+tolerance for d in differences)
        interval_mismatch_count = sum(abs(d-expected_interval) > tolerance for d in differences)
        if interval_mismatch_count:
            fail(f"{interval_mismatch_count} epoch intervals differ from profile")
        if expected_start is not None:
            grid_mismatch_count = sum(abs(t-(expected_start+j*expected_interval)) > tolerance
                                      for j, t in enumerate(epochs))
            if grid_mismatch_count:
                fail(f"{grid_mismatch_count} epochs differ from the expected sampling grid")
    if expected_epochs is not None and len(epochs) != expected_epochs:
        fail(f"Expected {expected_epochs} observation epochs; found {len(epochs)}")
    if epochs:
        for name, actual in (("TIME OF FIRST OBS", epochs[0]), ("TIME OF LAST OBS", epochs[-1])):
            if name not in header_times:
                continue
            try:
                header_epoch = epoch_time([">"] + header_times[name]["calendar"])
                if abs(header_epoch-actual) > tolerance:
                    fail(f"{name} header differs from observation body")
            except (ValueError, OverflowError, IndexError) as exc:
                fail(f"Invalid {name} header: {exc}")
        if expected_start is not None and abs(epochs[0]-expected_start) > tolerance:
            fail("First observation epoch differs from profile start")
        if expected_end is not None and abs(epochs[-1]-expected_end) > tolerance:
            fail("Last observation epoch differs from inclusive profile end")
        if expected_start is not None and any(t < expected_start-tolerance for t in epochs):
            fail("Observation precedes profile start")
        if expected_end is not None and any(t > expected_end+tolerance for t in epochs):
            fail("Observation follows profile end")
    interval_reference = expected_interval or header_interval
    interval_reference_source = "profile" if expected_interval is not None else "header"
    if interval_reference is None:
        positive_intervals = [d for d in differences if d > 0]
        interval_reference = statistics.median(positive_intervals) if positive_intervals else None
        interval_reference_source = "median of positive epoch intervals"
    if interval_reference is not None:
        gap_count = sum(d > interval_reference+tolerance for d in differences)
    result.update({
        "valid": error_count == 0,
        "error_count": error_count,
        "errors_truncated": error_count > MAX_ERRORS,
        "profile_applied": profile is not None,
        "allow_subset": allow_subset,
        "time_scale": "GPST" if profile is not None else "as recorded; see header_time_scales",
        "header_time_scales": {k: v["scale"] for k, v in header_times.items()},
        "header_interval_seconds": header_interval,
        "observation_types": obs_types,
        "signals": {s: sorted({t[1:] for t in ts}) for s, ts in obs_types.items()},
        "epochs": {
            "count": len(epochs),
            "first": display_time(epochs[0]) if epochs else None,
            "last": display_time(epochs[-1]) if epochs else None,
            "minimum_interval_seconds": min(differences) if differences else None,
            "maximum_interval_seconds": max(differences) if differences else None,
            "duplicate_adjacent_count": duplicate_count,
            "duplicate_epoch_count": len(epochs)-len(set(epochs)),
            "non_increasing_transition_count": non_increasing_count,
            "gap_count": gap_count if interval_reference is not None else None,
            "gap_reference_interval_seconds": interval_reference,
            "gap_reference_source": interval_reference_source if interval_reference is not None else None,
            "interval_mismatch_count": interval_mismatch_count if expected_interval is not None else None,
            "grid_mismatch_count": grid_mismatch_count if expected_interval is not None and expected_start is not None else None,
            "flags": dict(epoch_flags),
            "satellite_count_distribution": dict(epoch_satellite_counts),
        },
        "event_flags": dict(event_flags),
        "observation_satellite_records": observation_records,
        "cycle_slip_satellite_records": cycle_slip_records,
        "unique_satellites": len(satellite_records),
        "records_per_satellite": dict(sorted(satellite_records.items())),
        "finite_nonblank_measurements": {s: dict(sorted(c.items())) for s, c in measurement_counts.items()},
        "finite_nonzero_measurements": {s: dict(sorted(c.items())) for s, c in nonzero_counts.items()},
        "blank_measurements": {s: dict(sorted(c.items())) for s, c in blank_counts.items()},
        "lli_field_distribution": {s: dict(sorted(c.items())) for s, c in lli_counts.items()},
        "ssi_field_distribution": {s: dict(sorted(c.items())) for s, c in ssi_counts.items()},
        "receiver_clock_offset_records": len(clock_offsets),
        "measurement_fields_modified": False,
        "limitations": [
            "Structural and numerical observation validation does not establish positioning accuracy or antenna identity.",
            "LLI/SSI fields are inspected and counted unchanged; no source-to-output equality is asserted.",
            "Header-changing event records are rejected because dynamic observation maps are unsupported.",
        ],
    })
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--profile", type=Path)
    parser.add_argument("--allow-subset", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    try:
        profile = json.loads(args.profile.read_text()) if args.profile else None
        result = validate(args.input, profile, args.allow_subset)
    except (OSError, ValueError, TypeError, KeyError, OverflowError) as exc:
        result = {"valid": False, "path": str(args.input), "errors": [str(exc)], "error_count": 1}
    output = json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + "\n"
    if args.output:
        args.output.write_text(output)
        print(json.dumps({"valid": result["valid"], "report": str(args.output),
                          "error_count": result.get("error_count", 0)}))
    else:
        print(output, end="")
    return 0 if result["valid"] else 1


if __name__ == "__main__":
    sys.exit(main())
