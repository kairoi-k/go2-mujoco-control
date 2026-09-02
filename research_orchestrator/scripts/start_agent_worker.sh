#!/bin/zsh
set -euo pipefail

repo_root="$(cd "$(dirname "$0")/../.." && pwd)"
uv_bin="${UV_BIN:-$(command -v uv || true)}"
tailscale_bin="${TAILSCALE_BIN:-$(command -v tailscale || true)}"
if [[ -z "$uv_bin" || ! -x "$uv_bin" ]]; then
  print -u2 "uv is not available"
  exit 1
fi
if [[ -z "$tailscale_bin" || ! -x "$tailscale_bin" ]]; then
  print -u2 "Tailscale CLI is not available"
  exit 1
fi

export RESEARCH_REPO_ROOT="$repo_root"
export TEMPORAL_ADDRESS="${TEMPORAL_ADDRESS:-$($tailscale_bin ip -4 | head -n 1):7233}"
export CODEX_BIN="${CODEX_BIN:-$(command -v codex || true)}"
if [[ -z "$CODEX_BIN" || ! -x "$CODEX_BIN" ]]; then
  print -u2 "Codex CLI is not available"
  exit 1
fi

cd "$repo_root"
exec "$uv_bin" run python -m research_orchestrator.workers.agent_worker
