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

# Apply against the real vendor tree rather than through the repository symlink.
# The stored patch paths include the repo-local simulate/mujoco prefix and the
# old-side temporary .upstream suffix; strip both before handing it to patch.
cd "$vendor_dir"
sed -e 's#simulate/mujoco/##g' -e 's/\.upstream$//' \
  "$repo_dir/patches/mujoco-passive-render-snapshot.patch" \
  | patch --forward --batch -p0

grep -q "void Simulate::PublishRenderSnapshot" "$source_file"
echo "Applied MuJoCo passive render patch to $vendor_dir"
