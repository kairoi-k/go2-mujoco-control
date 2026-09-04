#!/usr/bin/env python3
"""Read B0 DDS domains from the frozen holdout manifest."""

import argparse
import json
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("kind", choices=("profiles", "fixed_3mps"))
    parser.add_argument("set_name", choices=("development", "holdout"))
    parser.add_argument("repeat", type=int)
    args = parser.parse_args()

    repo = Path(__file__).resolve().parents[3]
    manifest = json.loads(
        (repo / "docs/research/PHASE2_HOLDOUT_MANIFEST.json").read_text()
    )
    b0 = manifest["b0"]
    if args.set_name == "development":
        domains = b0["development_domains"][args.kind]
    else:
        if args.repeat not in (1, 2, 3):
            parser.error("holdout repeat must be 1..3")
        key = "repeats" if args.kind == "profiles" else "fixed_3mps_repeats"
        domains = next(
            item for item in b0[key] if item["repeat"] == args.repeat
        )
    print(domains["baseline_domain"], domains["terrain_domain"])


if __name__ == "__main__":
    main()
