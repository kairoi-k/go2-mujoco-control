# Reproduce this checkpoint

No command in this document was used to start a new closed-loop development experiment during
closeout. The exact recorded runtime source is
`eeb5d757620712759604d8c51b2b9075d05625dc`; the containing checkpoint changes only
documents and evidence/replay tooling. A clean documentation HEAD is not a
claim that the old physics runs used that later SHA.

## Verify and replay retained evidence

Run from the repository in native Linux. The three immutable raw directories
are under `example/cpp/experiments/_runs/` and are identified in results.json.
Raw CSVs/videos are intentionally not committed. A separate machine must
obtain those original directories and verify their hashes before using them.
The committed summaries alone do not substitute for raw evidence.

```
python3 docs/research/evidence/b1_checkpoint_20260907/verify_checkpoint.py --check-binaries
python3 docs/research/evidence/b1_checkpoint_20260907/reproduce_results.py --repo "$PWD" --output-dir /tmp/b1-checkpoint-replay-UNIQUE
```

The output directory must be new. The replay reads logs and invokes the
versioned analyzers; it does not modify raw files or simulate. Comparing
replayed results to the committed results is required. Without local raw data,
`verify_checkpoint.py --skip-raw` checks only source and packet integrity and
explicitly reports `raw_verified=false`. Retained binary hashes are optional:
a rebuild at another path/compiler may legitimately have a different hash.

The source-binding manifest covers 135 runtime/build/test files. Fresh closeout
checks passed 44 controller tests, 3 simulator tests and 48 analyzer tests;
verbatim logs and toolchain versions are committed. Dependencies include native
MuJoCo 3.3.6, Unitree SDK2, Eigen, Python 3.10.12 and NumPy 2.2.6. The retained
native build uses g++ 11.4 and CMake 3.22.1. `simulate/mujoco` points to the local
MuJoCo installation; that ignored dependency is not delivered by Git.

## Rebuild, independently of replay

```
cmake -S example/cpp -B example/cpp/build
cmake --build example/cpp/build -j2
ctest --test-dir example/cpp/build --output-on-failure
ctest --test-dir simulate/build --output-on-failure
```

Simulator CTest assumes its existing configured/built tree. On a new machine,
configure and build `simulate` with the matching MuJoCo dependency first.

## Re-run the original experiment only in a later authorized research loop

Use a separate clean worktree at the exact runtime SHA above, rebuild there,
and choose never-used raw names. These commands are documented provenance,
not a request to run them as part of closeout:

```
export TROT_RESEARCH_MAP_INTERVALS_V2=1
export TROT_RESEARCH_RUNNING_PERIOD_S=0.14
export TROT_RESEARCH_RUNNING_LIFT_FLOOR_M=0
unset TROT_RESEARCH_NOMINAL_COM_HEIGHT TROT_TERRAIN_EXECUTION_CONSISTENCY_SHADOW
unset TROT_TERRAIN_DEBUG_PLANNER TROT_TERRAIN_DEBUG_SWING
bash example/cpp/scripts/run_b1_research_probe.sh eeb5d757620712759604d8c51b2b9075d05625dc terrain-b1-execution UNIQUE_FLAT phase2_flat.xml 32 b1_v3_running_1mps.csv
bash example/cpp/scripts/run_b1_research_probe.sh eeb5d757620712759604d8c51b2b9075d05625dc terrain-b1-execution UNIQUE_STEP b1_v3_running_step_5cm.xml 32
```

The harness holds the shared experiment lock, records seed 11/domain 231 and
all effective environment/argv, and refuses dirty source or existing raw
names. Its wrapper can return nonzero for legacy analyzers; inspect each
runtime status and the target V3 result. The debug run additionally sets
`TROT_TERRAIN_DEBUG_PLANNER=1`; it is attribution-only. Wall-clock scheduling
means same seed/source is not a guarantee of identical physical trajectories.
No repeatability distribution or holdout success is claimed by this packet.

## Video

The delivered `recorded_traversal_v1.mp4` shows recorded state, not a new
simulation. Its original renderer, input hashes and alignment diagnostics
remain in the step raw directory. The video manifest records maximum join gap
2 ms and maximum reconstructed foot-site error 7.824 mm. It cannot distinguish
millimetre-scale contacts visually; use the force logs for that claim.
Normal speed is followed by 0.25x replay. Contact labels come from truth data.
