# Phase1 benchmark freeze

This is the frozen Phase1 reference for the terrain worktree. It is a
simulation-only result and does not claim hardware or sim-to-real performance.

- Source: `1b4d9b8fcf3dcfb63cee144c9871a235101713c9`
- Branch: `terrain/phase2-20260824T150934Z`
- Build: MuJoCo 3.3.6, simulator and C++ controller built in Ubuntu-22.04/WSL
- Tests: `ctest --test-dir example/cpp/build --output-on-failure` - 25/25
- Profile: `bash example/cpp/scripts/run_sustained_running.sh --headless`
- Analyzer: `python3 example/cpp/tools/analysis/analyze_sustained_running.py <run>`

| run | median m/s | good window s | roll/pitch P95 deg | aerial | pair sync min | stop P95 m/s |
|---|---:|---:|---:|---:|---:|---:|
| `phase1_freeze_running_r1_20260824` | 3.242725 | 61.402 | 2.821/2.288 | 0.293086 | 0.817057 | 0.004574 |
| `phase1_freeze_running_r2_20260824` | 3.227325 | 61.442 | 3.166/2.541 | 0.283184 | 0.799286 | 0.003563 |
| `phase1_freeze_running_r3_20260824` | 3.234687 | 61.422 | 2.778/2.308 | 0.291980 | 0.810059 | 0.003980 |

All three runs passed the unchanged strict analyzer. Terrain changes must
retain this reference as a separate flat-ground regression.
