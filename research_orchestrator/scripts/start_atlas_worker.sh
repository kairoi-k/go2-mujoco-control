#!/usr/bin/env bash
set -euo pipefail

control_plane_root="$(cd "$(dirname "$0")/../.." && pwd)"
uv_bin="${UV_BIN:-$(command -v uv || true)}"
if [[ -z "$uv_bin" && -x /home/che/.local/bin/uv ]]; then
  uv_bin=/home/che/.local/bin/uv
fi
if [[ -z "$uv_bin" || ! -x "$uv_bin" ]]; then
  echo "uv is not available" >&2
  exit 1
fi

export RESEARCH_REPO_ROOT="$control_plane_root"
export ATLAS_WORKSPACE="${ATLAS_WORKSPACE:-/home/che/dev/go2-workspace/current}"
export ATLAS_ARTIFACT_ROOT="${ATLAS_ARTIFACT_ROOT:-/home/che/dev/go2-workspace/atlas-artifacts}"
export ATLAS_MUJOCO_ROOT="${ATLAS_MUJOCO_ROOT:-/home/che/.mujoco/mujoco-3.3.6}"
# WSL currently resolves the MagicDNS/Funnel name to the public Funnel
# ingress. Use the Base Tailscale IPv4 for raw Temporal gRPC; override this
# value when the Base tailnet address changes.
export TEMPORAL_ADDRESS="${TEMPORAL_ADDRESS:-100.90.49.95:7233}"
export ATLAS_ADAPTER_READY=1

cd "$control_plane_root"
exec "$uv_bin" run python -m research_orchestrator.workers.atlas_worker "$@"
