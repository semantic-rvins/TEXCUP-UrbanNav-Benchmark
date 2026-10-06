#!/usr/bin/env python3
"""Download the validated original SBF recordings directly from TEX-CUP.

Without --download, list the publisher's SBF files. Downloaded data and
manifests are local artifacts; keep them out of repository commits.
"""

import argparse
from html.parser import HTMLParser
import json
import os
from pathlib import Path
from urllib.parse import unquote, urljoin, urlsplit
from urllib.request import Request, urlopen


ROOT = Path(__file__).resolve().parent.parent
USER_AGENT = "TEX-CUP-local-reproduction/1.0"


class Links(HTMLParser):
    def __init__(self):
        super().__init__()
        self.links = []

    def handle_starttag(self, tag, attrs):
        if tag == "a":
            self.links.extend(value for key, value in attrs if key == "href" and value)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--receiver", choices=("rover", "base"), default="rover")
    parser.add_argument("--download", action="store_true", help="download asterx4.sbf; otherwise only list sources")
    parser.add_argument("--destination", type=Path, help="default: data/tex_cup/raw/RECEIVER")
    args = parser.parse_args()
    profile_path = ROOT / "projects/RTKLIB/config" / f"texcup_{args.receiver}_sbf.json"
    profile = json.loads(profile_path.read_text())
    expected = profile["source_file"]
    base = profile["source_directory"]
    source = urlsplit(expected["url"])
    directory = urlsplit(base)
    if (source.scheme != "https" or directory.scheme != "https"
            or source.netloc != "rnl-data.ae.utexas.edu"
            or source.netloc != directory.netloc):
        parser.error("Profiles must point to the publisher's HTTPS archive.")
    html = urlopen(Request(base, headers={"User-Agent": USER_AGENT}), timeout=45).read().decode("utf-8")
    links = Links()
    links.feed(html)
    files = {}
    for href in links.links:
        url = urljoin(base, href)
        parsed = urlsplit(url)
        name = unquote(parsed.path.rsplit("/", 1)[-1])
        if (parsed.scheme == "https" and parsed.netloc == directory.netloc
                and parsed.path.startswith(directory.path)
                and "/" not in name and "\\" not in name and name.lower().endswith(".sbf")):
            files[name] = url
    print(json.dumps({"directory": base, "sbf_files": files}, indent=2))
    if not args.download:
        return
    if files.get("asterx4.sbf") != expected["url"]:
        parser.error("The validated source URL is absent from the publisher listing; investigate the archive revision.")
    destination = args.destination or ROOT / "data/tex_cup/raw" / args.receiver
    destination.mkdir(parents=True, exist_ok=True)
    target = destination / "asterx4.sbf"
    partial = destination / "asterx4.sbf.part"
    manifest = destination / "download-manifest.json"
    for path in (target, partial, manifest):
        if path.exists() or path.is_symlink():
            parser.error(f"Preserving existing {path}; choose another destination.")
    length = 0
    with urlopen(Request(expected["url"], headers={"User-Agent": USER_AGENT}), timeout=60) as response:
        expected_length = response.headers.get("Content-Length")
        with partial.open("xb") as out:
            while chunk := response.read(1024 * 1024):
                out.write(chunk)
                length += len(chunk)
    if expected_length is not None and length != int(expected_length):
        raise SystemExit(f"Incomplete HTTP download: {length}/{expected_length} bytes; partial file retained.")
    if length != expected["bytes"]:
        raise SystemExit("Downloaded size differs from the source recording; partial file retained. Investigate before changing the profile.")
    try:
        os.link(partial, target)
    except FileExistsError:
        raise SystemExit(f"Preserving newly existing {target}; partial file retained.") from None
    partial.unlink()
    record = {"url": expected["url"], "path": str(target.resolve()), "bytes": length}
    with manifest.open("x") as out:
        out.write(json.dumps({"transport": "HTTPS with certificate verification",
                              "files": [record]}, indent=2) + "\n")
    print(f"Verified source recording: {target}")


if __name__ == "__main__":
    main()
