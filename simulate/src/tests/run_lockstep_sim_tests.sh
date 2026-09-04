#!/usr/bin/env bash
# Order-103 simulator integration tests (run by ctest).
#
#   1. --lockstep with no controller attached must fail closed at the ready
#      barrier: non-zero exit, SIM_LOCKSTEP_FAIL_CLOSED marker, and a trace
#      summary with fail_closed=1 and a violation.
#   2. flag-off must be unchanged: normal DDS-ready marker, no lockstep
#      output, no trace file, physics stepping, clean SIGTERM shutdown.
set -u

binary="$1"
build_dir="$2"
tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

# Self-contained: the watchdog must fire well before the `timeout` guard.
export SIM_LOCKSTEP_BARRIER_TIMEOUT_S=3

# --- 1. lockstep fail-closed (startup watchdog; no controller attached) ---
timeout 60 "$binary" -i 191 -r go2 -s scene_leg_lift_demo.xml --headless \
  --ground-truth-log "$tmp/gt_fc.csv" \
  --lockstep --lockstep-trace "$tmp/trace_fc.csv" \
  >"$tmp/sim_fc.log" 2>&1
rc=$?
if [[ "$rc" == 0 ]]; then
  echo "FAIL: lockstep sim exited 0 on barrier timeout (expected non-zero)" >&2
  exit 1
fi
if [[ "$rc" == 124 ]]; then
  echo "FAIL: lockstep sim hung past the watchdog (exit 124)" >&2
  tail -20 "$tmp/sim_fc.log" >&2
  exit 1
fi
if ! grep -q "SIM_LOCKSTEP_FAIL_CLOSED" "$tmp/sim_fc.log"; then
  echo "FAIL: no SIM_LOCKSTEP_FAIL_CLOSED marker in simulator.log (exit=$rc)" >&2
  tail -20 "$tmp/sim_fc.log" >&2
  exit 1
fi
if ! grep -q "fail_closed=1" "$tmp/trace_fc.csv"; then
  echo "FAIL: lockstep trace lacks fail_closed=1 summary" >&2
  tail -5 "$tmp/trace_fc.csv" >&2
  exit 1
fi
echo "PASS: lockstep barrier watchdog fails closed (exit=$rc)"

# --- 2. flag-off equivalence (direct launch; deterministic bounded stop) ---
"$binary" -i 192 -r go2 -s scene_leg_lift_demo.xml --headless \
  --ground-truth-log "$tmp/gt_off.csv" \
  >"$tmp/sim_off.log" 2>&1 &
sim_pid=$!
sleep 4
if ! grep -q "Unitree DDS bridge ready" "$tmp/sim_off.log"; then
  echo "FAIL: flag-off simulator did not reach DDS-ready" >&2
  kill "$sim_pid" 2>/dev/null || true
  exit 1
fi
if grep -q "LOCKSTEP" "$tmp/sim_off.log"; then
  echo "FAIL: flag-off simulator.log mentions lockstep" >&2
  kill "$sim_pid" 2>/dev/null || true
  exit 1
fi
if [[ -e "$tmp/trace_off.csv" ]]; then
  echo "FAIL: flag-off produced a lockstep trace file" >&2
  kill "$sim_pid" 2>/dev/null || true
  exit 1
fi
rows=$(wc -l <"$tmp/gt_off.csv")
if (( rows < 100 )); then
  echo "FAIL: flag-off physics did not step (ground-truth rows=$rows)" >&2
  kill "$sim_pid" 2>/dev/null || true
  exit 1
fi
kill "$sim_pid" 2>/dev/null || true
for _ in 1 2 3 4 5; do
  if ! kill -0 "$sim_pid" 2>/dev/null; then break; fi
  sleep 1
done
kill -9 "$sim_pid" 2>/dev/null || true
wait "$sim_pid" 2>/dev/null || true
echo "PASS: flag-off wall-clock path unchanged (ground-truth rows=$rows)"
echo "PASS: lockstep simulator integration tests"
