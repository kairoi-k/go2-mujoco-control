#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "$0")" && pwd)"
repo_dir="$(cd "$script_dir/.." && pwd)"
vendor_link="$repo_dir/simulate/mujoco"
vendor_dir="$(readlink -f "$vendor_link" 2>/dev/null || true)"

if [[ -z "$vendor_dir" || ! -d "$vendor_dir/simulate" ]]; then
  echo "MuJoCo 3.3.6 symlink is missing: $vendor_link" >&2
  exit 2
fi
if [[ "$(basename "$vendor_dir")" != "mujoco-3.3.6" ]]; then
  echo "Expected MuJoCo 3.3.6, found: $vendor_dir" >&2
  exit 2
fi

source_file="$vendor_dir/simulate/simulate.cc"
if grep -q "void Simulate::PublishRenderSnapshot" "$source_file"; then
  echo "MuJoCo passive render patch already applied."
  exit 0
fi

cd "$repo_dir"
patch --forward --batch -p0 < "$repo_dir/patches/mujoco-passive-render-snapshot.patch"
echo "Applied MuJoCo passive render patch to $vendor_dir"
