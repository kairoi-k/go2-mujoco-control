"""Download the recorded ``model_54950`` checkpoint from the GitHub Release."""

from __future__ import annotations

import argparse
import hashlib
import sys
import urllib.request
from pathlib import Path

RELEASE_URL = (
    "https://github.com/kairoi-k/go2-mujoco-control/releases/download/v0.1.0/model_54950.pt"
)
EXPECTED_SHA256 = "c2009f890e5b575a8832021ab717dd2dcc23678a64f423d2f4e793d861ed4b42"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("-o", "--output", type=Path, default=Path("model_54950.pt"))
    args = parser.parse_args()
    dest = args.output.expanduser()
    dest.parent.mkdir(parents=True, exist_ok=True)
    print(f"downloading {RELEASE_URL}")
    urllib.request.urlretrieve(RELEASE_URL, dest)
    digest = hashlib.sha256(dest.read_bytes()).hexdigest()
    if digest != EXPECTED_SHA256:
        dest.unlink(missing_ok=True)
        raise SystemExit(f"sha256 mismatch: got {digest}, expected {EXPECTED_SHA256}")
    print(f"wrote {dest} ({digest[:16]}…)")


if __name__ == "__main__":
    try:
        main()
    except OSError as exc:
        sys.exit(f"download failed: {exc}")
