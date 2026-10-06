#!/usr/bin/env python3
"""Convert a local SBF file/pattern using a reviewable JSON convbin profile."""
import argparse
from datetime import datetime, timedelta
import glob
import json
from pathlib import Path
import subprocess
import sys

LOADERS = Path(__file__).resolve().parent
ROOT = LOADERS.parent


def epoch_time(line):
    fields = line.split()
    second = float(fields[6])
    return datetime(*map(int, fields[1:6])) + timedelta(seconds=second)


def crop_observations(source, destination, profile):
    """Copy whole RINEX3 epoch records with unchanged observation fields."""
    start = datetime.fromisoformat(profile["time"]["start"])
    end = datetime.fromisoformat(profile["time"]["end_inclusive"])
    tol = timedelta(seconds=profile["time"]["tolerance_seconds"])
    header, first, last = [], None, None
    body_path = destination.with_suffix(".body.tmp")
    with source.open() as fin, body_path.open("x") as body:
        for line in fin:
            header.append(line)
            if line[60:80].strip() == "END OF HEADER":
                break
        else:
            raise ValueError("Missing RINEX END OF HEADER")
        block = []

        def flush():
            nonlocal first, last
            if not block:
                return
            epoch = epoch_time(block[0])
            if start - tol <= epoch <= end + tol:
                body.writelines(block)
                flag = int(block[0].split()[7])
                if flag in (0, 1):
                    first = first or epoch
                    last = epoch

        for line in fin:
            if line.startswith(">"):
                flush()
                block = [line]
            else:
                block.append(line)
        flush()
    if first is None:
        raise ValueError("Conversion produced no observations in the requested GPST window")
    with destination.open("x") as fout:
        for line in header:
            label = line[60:80].strip()
            if label in ("TIME OF FIRST OBS", "TIME OF LAST OBS"):
                epoch = first if label == "TIME OF FIRST OBS" else last
                seconds = epoch.second + epoch.microsecond / 1e6
                timestamp = f"{epoch.year:6d}{epoch.month:6d}{epoch.day:6d}{epoch.hour:6d}{epoch.minute:6d}{seconds:13.7f}"
                line = timestamp + line[43:]
            fout.write(line)
        with body_path.open() as body:
            for line in body:
                fout.write(line)
    body_path.unlink()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", help="one SBF file, or a quoted pattern for chronological chunks from the same receiver/antenna")
    parser.add_argument("--profile", type=Path, default=ROOT / "config/texcup_rover_sbf.json")
    parser.add_argument("--convbin", type=Path, default=ROOT / "upstream/RTKLIB-2.5.1/app/consapp/convbin/gcc/convbin")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "output/rover")
    args = parser.parse_args()
    profile = json.loads(args.profile.read_text())
    if profile["time"]["scale"] != "GPST":
        parser.error("This conversion recipe requires GPST bounds.")
    sources = [Path(path).resolve() for path in sorted(glob.glob(args.input))]
    if not sources:
        parser.error("No input files matched; download original SBF from the publisher first.")
    if any(not source.is_file() for source in sources):
        parser.error("Every matched input must be a regular file.")
    source_manifest = []
    for source in sources:
        source_manifest.append({"path": str(source), "bytes": source.stat().st_size})
    expected_source = profile.get("source_file")
    if expected_source is not None:
        if len(source_manifest) != 1 or source_manifest[0]["bytes"] != expected_source["bytes"]:
            parser.error("Input size differs from the source recording for this profile. Investigate the publisher/input revision before using a different profile.")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    intermediate = args.output_dir / "read-ahead.obs"
    output_name = profile.get("output_filename", "asterx4_rover.obs")
    if Path(output_name).name != output_name:
        parser.error("output_filename must be a filename without directories.")
    final = args.output_dir / output_name
    if any(args.output_dir.iterdir()):
        parser.error("Output directory must be empty; choose a new directory to preserve earlier results.")
    start = datetime.fromisoformat(profile["time"]["start"])
    end = datetime.fromisoformat(profile["time"]["end_inclusive"]) + timedelta(seconds=profile["time"]["read_ahead_seconds"])
    mask = ",".join(system + "L" + signal for system, signals in profile["signals"].items() for signal in signals)
    command = [str(args.convbin.resolve()), "-r", "sbf", "-v", profile["rinex_version"], "-od", "-os",
               "-mask", mask, "-ts", start.strftime("%Y/%m/%d"), start.strftime("%H:%M:%S.%f"),
               "-te", end.strftime("%Y/%m/%d"), end.strftime("%H:%M:%S.%f"),
               "-ti", str(profile["interval_seconds"]), "-tt", str(profile["time"]["tolerance_seconds"])]
    for system in "GREJSCI":
        if system not in profile["signals"]:
            command += ["-y", system]
    if profile.get("receiver_options"):
        command += ["-ro", profile["receiver_options"]]
    if profile.get("trace_level"):
        command += ["-trace", str(profile["trace_level"])]
    command += ["-o", str(intermediate.resolve()), str(Path(args.input).absolute())]
    (args.output_dir / "conversion-command.json").write_text(json.dumps(command, indent=2) + "\n")
    with (args.output_dir / "convbin.log").open("x") as log:
        subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True, cwd=args.output_dir)
    crop_observations(intermediate, final, profile)
    validation_command = [sys.executable, str(LOADERS / "validate_rinex.py"), str(final), "--profile", str(args.profile.resolve()),
                          "--output", str(args.output_dir / "validation.json")]
    subprocess.run(validation_command, check=True)
    manifest = {"profile": profile, "input_files": source_manifest, "output_file": str(final.resolve()),
                "scope": "Observation conversion only; no positioning or benchmark rerun; original-data distribution not included."}
    (args.output_dir / "reproduction-manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"Validated local observation file: {final}")


if __name__ == "__main__":
    main()
