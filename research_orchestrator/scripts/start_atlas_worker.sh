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

# The Atlas worker owns local inference. Keep the default model/runtime pinned
# so a restart cannot silently select a different file or quantization. Every
# value remains overridable for a controlled candidate benchmark.
export ATLAS_LLM_SERVER_EXE="${ATLAS_LLM_SERVER_EXE:-/mnt/c/Users/w1881/go2-local-llm/bin/llama-server.exe}"
export ATLAS_LLM_MODEL_PATH="${ATLAS_LLM_MODEL_PATH:-/mnt/c/Users/w1881/go2-local-llm/models/Qwen3-Coder-30B-A3B-Instruct-Q2_K_L.gguf}"
export ATLAS_LLM_PORT="${ATLAS_LLM_PORT:-8090}"
export ATLAS_LLM_MODEL_ID="${ATLAS_LLM_MODEL_ID:-Qwen3-Coder-30B-A3B-Instruct-Q2_K_L}"
export ATLAS_LLM_MODEL_REVISION="${ATLAS_LLM_MODEL_REVISION:-unsloth/Qwen3-Coder-30B-A3B-Instruct-GGUF@b17cb02dd882d5b6ab62fc777ad2995f19668350}"
export ATLAS_LLM_QUANTIZATION="${ATLAS_LLM_QUANTIZATION:-Q2_K_L}"
export ATLAS_LLM_RUNTIME_VERSION="${ATLAS_LLM_RUNTIME_VERSION:-llama.cpp-b10766-cuda13.3}"
export ATLAS_LLM_MODEL_SHA256="${ATLAS_LLM_MODEL_SHA256:-7add73b0607b498f79157a5f4e4ccddc14ad7afd61d76655e064e1e92476267e}"
export ATLAS_LLM_VERIFY_MODEL_HASH="${ATLAS_LLM_VERIFY_MODEL_HASH:-1}"
export ATLAS_LLM_CTX_SIZE="${ATLAS_LLM_CTX_SIZE:-16384}"
export ATLAS_LLM_REASONING="${ATLAS_LLM_REASONING:-off}"
export ATLAS_LLM_REASONING_FORMAT="${ATLAS_LLM_REASONING_FORMAT:-none}"

cd "$control_plane_root"
exec "$uv_bin" run python -m research_orchestrator.workers.atlas_worker "$@"
