#!/bin/zsh
set -euo pipefail

repo_root="$(cd "$(dirname "$0")/../.." && pwd)"
temporal_bin="${TEMPORAL_BIN:-$(command -v temporal || true)}"
tailscale_bin="${TAILSCALE_BIN:-$(command -v tailscale || true)}"
if [[ -z "$temporal_bin" || ! -x "$temporal_bin" ]]; then
  print -u2 "Temporal CLI is not available"
  exit 1
fi
if [[ -z "$tailscale_bin" || ! -x "$tailscale_bin" ]]; then
  print -u2 "Tailscale CLI is not available"
  exit 1
fi

bind_address="${TEMPORAL_BIND_ADDRESS:-0.0.0.0}"
if [[ ! "$bind_address" =~ '^[0-9]+([.][0-9]+){3}$' ]]; then
  print -u2 "Refusing to bind Temporal to an invalid IPv4 address: $bind_address"
  exit 1
fi

tailscale_address="$($tailscale_bin ip -4 | head -n 1)"
if [[ ! "$tailscale_address" =~ '^[0-9]+([.][0-9]+){3}$' ]]; then
  print -u2 "Tailscale does not currently provide a usable IPv4 address"
  exit 1
fi

if [[ "$bind_address" == "0.0.0.0" ]]; then
  print -u2 "Temporal dev server requires wildcard binding for cross-host SDK discovery; keep port 7233 on the private Base/Tailscale network only"
fi

mkdir -p "$repo_root/.temporal"
exec "$temporal_bin" server start-dev \
  --ip "$bind_address" \
  --port "${TEMPORAL_PORT:-7233}" \
  --ui-ip 127.0.0.1 \
  --ui-port "${TEMPORAL_UI_PORT:-8233}" \
  --db-filename "$repo_root/.temporal/dev.sqlite" \
  --ui-disable-news-fetch
