#!/usr/bin/env python3
"""Validate a Go2 simulation profile and expose its shell-facing fields."""

import argparse
import json
import pathlib
import sys


def load(path: str) -> dict:
    profile_path = pathlib.Path(path)
    with profile_path.open(encoding="utf-8") as stream:
        profile = json.load(stream)
    if not isinstance(profile, dict) or profile.get("schema_version") != 1:
        raise ValueError("profile schema_version must be 1")
    args = profile.get("controller_args", [])
    env = profile.get("semantic_env", {})
    if not isinstance(args, list) or not all(isinstance(value, str) for value in args):
        raise ValueError("controller_args must be an array of strings")
    if not isinstance(env, dict):
        raise ValueError("semantic_env must be an object")
    for key, value in env.items():
        if not isinstance(key, str) or not key or not key.replace("_", "").isalnum():
            raise ValueError(f"invalid semantic environment name: {key!r}")
        if isinstance(value, bool):
            continue
        if not isinstance(value, (str, int, float)):
            raise ValueError(f"semantic environment value must be scalar: {key}")
    runner = profile.get("runner", {})
    if not isinstance(runner, dict):
        raise ValueError("runner must be an object")
    return profile


def scalar(value):
    if isinstance(value, bool):
        return "1" if value else "0"
    return str(value)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("profile")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--args", action="store_true")
    group.add_argument("--env", action="store_true")
    group.add_argument("--runner", metavar="FIELD")
    options = parser.parse_args()
    try:
        profile = load(options.profile)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        print(f"profile error: {exc}", file=sys.stderr)
        return 2
    if options.args:
        for value in profile["controller_args"]:
            print(value)
    elif options.env:
        for key, value in sorted(profile["semantic_env"].items()):
            print(f"{key}\t{scalar(value)}")
    else:
        value = profile["runner"].get(options.runner, "")
        if isinstance(value, (str, int, float)):
            print(value)
        else:
            print("", end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
