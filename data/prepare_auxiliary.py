#!/usr/bin/env python3
"""Download TEX-CUP auxiliary inputs privately; never replace existing files."""

import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import tempfile
from urllib.parse import urlsplit
from urllib.request import HTTPRedirectHandler, build_opener


ARCHIVE = "https://rnl-data.ae.utexas.edu/texcup/"
SOURCES = {
    "lord_imu.log": {
        "url": ARCHIVE + "2019May09-rover/lord/lord_imu.log",
    },
    "ground_truth.log": {
        "url": ARCHIVE + "2019May09-rover/atlans/ground_truth.log",
    },
}
CALIBRATIONS = {
    "texcup_mono_calibration.yaml":
        "camera-calibration-mono/texcup_port-camchain.yaml",
    "texcup_stereo_calibration.yaml":
        "camera-calibration-stereo/texcup_stereo-camchain_ovins.yaml",
}


class HTTPSOnlyRedirect(HTTPRedirectHandler):
    def redirect_request(self, request, file_pointer, code, message, headers, new_url):
        if urlsplit(new_url).scheme != "https":
            raise ValueError("Refusing a non-HTTPS download redirect")
        return super().redirect_request(
            request, file_pointer, code, message, headers, new_url
        )


def same_bytes(first, second):
    if first.stat().st_size != second.stat().st_size:
        return False
    with first.open("rb") as left, second.open("rb") as right:
        while block := left.read(1024 * 1024):
            if block != right.read(len(block)):
                return False
        return not right.read(1)


def verify_existing(path, staged):
    if not path.exists() and not path.is_symlink():
        return False
    if not path.is_file() or not same_bytes(path, staged):
        raise ValueError(
            f"Existing file differs from the expected input: {path}. "
            "Preserve it and choose a new --data-dir or move it aside yourself."
        )
    return True


def install_staged(path, write, validate=None):
    """Stage privately, compare existing bytes, then install without replacement."""
    if (path.exists() or path.is_symlink()) and not path.is_file():
        raise ValueError(f"Preserving existing non-file path: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, name = tempfile.mkstemp(prefix=".texcup-", dir=path.parent)
    temporary = Path(name)
    try:
        with os.fdopen(descriptor, "wb") as destination:
            write(destination)
        if validate is not None:
            validate(temporary)
        if verify_existing(path, temporary):
            return
        try:
            os.link(temporary, path)
        except FileExistsError:
            verify_existing(path, temporary)
    finally:
        temporary.unlink(missing_ok=True)


def install_bytes(path, content):
    install_staged(path, lambda destination: destination.write(content))


def download(path, source):
    def transfer(destination):
        print(f"Downloading {source['url']}", flush=True)
        opener = build_opener(HTTPSOnlyRedirect())
        with opener.open(source["url"], timeout=120) as response:
            expected_length = response.headers.get("Content-Length")
            length = 0
            for block in iter(lambda: response.read(1024 * 1024), b""):
                destination.write(block)
                length += len(block)
            if expected_length is not None and length != int(expected_length):
                raise ValueError(f"Incomplete HTTP download: {length}/{expected_length} bytes")

    install_staged(path, transfer, lambda staged: validate_log(staged, path.name))


def validate_log(path, name):
    count = 0
    with path.open("r", encoding="latin1") as stream:
        for line in stream:
            if not line.strip() or line.startswith("#"):
                continue
            fields = line.split()
            try:
                if name == "lord_imu.log":
                    if len(fields) != 9 or not all(math.isfinite(float(value)) for value in fields):
                        raise ValueError()
                elif name == "ground_truth.log":
                    if (len(fields) < 6 or fields[0] != "2019/05/09"
                            or not all(math.isfinite(float(value)) for value in fields[3:6])):
                        raise ValueError()
                    datetime.fromisoformat(fields[0].replace("/", "-") + "T" + fields[1])
                else:
                    raise ValueError()
            except (ValueError, TypeError):
                raise ValueError(f"Unexpected {name} data row {count + 1}") from None
            count += 1
    if not count:
        raise ValueError(f"No sample rows in {name}")
    return count


def prepare(data_dir):
    repository = Path(__file__).resolve().parents[1]
    data_dir = data_dir.expanduser().resolve()
    raw_dir = data_dir / "raw" / "auxiliary"
    records = []
    for name, source in SOURCES.items():
        raw = raw_dir / name
        download(raw, source)
        records.append({
            "path": str(raw.relative_to(data_dir)),
            "source_url": source["url"],
            "bytes": raw.stat().st_size,
        })

    raw_gt = (raw_dir / "ground_truth.log").read_bytes()
    numeric_rows = [line for line in raw_gt.splitlines()
                    if line.strip() and not line.startswith(b"#")]
    # Standardize Latin-1 comments to UTF-8 for local tools. The evaluator also
    # accepts the publisher encoding; samples, columns, timestamps and poses stay unchanged.
    prepared_gt = raw_gt.decode("latin1").encode("utf8")
    prepared_rows = [line for line in prepared_gt.splitlines()
                     if line.strip() and not line.startswith(b"#")]
    if prepared_rows != numeric_rows:
        raise ValueError("Ground-truth conversion changed sample rows")
    prepared = {
        "lord_imu.log": (raw_dir / "lord_imu.log").read_bytes(),
        "ground_truth.log": prepared_gt,
    }
    for source_name, destination_name in CALIBRATIONS.items():
        source_path = repository / "common" / "config" / source_name
        prepared[destination_name] = source_path.read_bytes()

    for name, content in prepared.items():
        path = data_dir / name
        install_bytes(path, content)
        record = {"path": name, "bytes": len(content)}
        if name == "ground_truth.log":
            record.update({
                "processing": "Latin-1 to UTF-8; numeric rows unchanged",
                "numeric_rows": len(numeric_rows),
            })
        elif name in CALIBRATIONS.values():
            config_name = next(source_name for source_name, destination_name
                               in CALIBRATIONS.items() if destination_name == name)
            record["source"] = "common/config/" + config_name
        else:
            record["processing"] = "Unchanged publisher bytes"
        records.append(record)
    manifest = {
        "schema_version": 1,
        "archive": ARCHIVE,
        "files": records,
        "ground_truth_reference": {
            "publisher_description": "ATLANS-C post-processed IMU frame origin",
            "benchmark_interpretation": "Antenna 2 / ALT1; see METHOD_SETUP_GUIDE.md",
            "pose_correction_applied": False,
        },
    }
    install_bytes(data_dir / "auxiliary-manifest.json",
                  (json.dumps(manifest, indent=2) + "\n").encode("utf8"))
    print(f"Verified auxiliary inputs in {data_dir}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-dir", type=Path,
                        default=Path(__file__).resolve().parent / "tex_cup",
                        help="Private local dataset root (default: data/tex_cup)")
    args = parser.parse_args()
    try:
        prepare(args.data_dir)
    except (OSError, ValueError) as error:
        parser.exit(1, f"prepare_auxiliary: {error}\n")


if __name__ == "__main__":
    main()
