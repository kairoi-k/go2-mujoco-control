# B1 closed-loop research audit, 2026-09-07

## Verdict and stopping decision

**B1 is NOT solved and this is NOT a candidate or acceptance packet.** The user
reopened the research route from clean `5cbc547f225dbb60683d96e440beffb0b014a075`,
authorizing route changes and versioned acceptance work. This investigation
restored real MuJoCo execution, ran two centered 5 cm development probes and a
flat steps development pair attempt, and found a material acceptance defect.
The stopping condition is the user's explicitly permitted *major research /
acceptance issue*, not completion of a Stage C module or an environment blocker.

The frozen Phase-1 analyzer reports a quantitative PASS for the fresh flat
steps terrain member even though two measured transitions never settle within
its own tolerance. Nonfinite transition results are removed before taking the
maximum. A PASS for the remaining transitions therefore masks the failed ones.
The same implementation accepts a truncated final sample as a one-second hold.
The independent audit in this packet leaves the original contract, thresholds,
analyzer and historical verdict files untouched. It is a diagnostic check,
not a replacement full B0/B1 acceptance contract.

## Real experiments and provenance

All timed runs used clean source `5cbc547f225dbb60683d96e440beffb0b014a075`,
the repository harness, and `/tmp/go2_mujoco_experiment.lock`. No controller,
planner, MPC/WBC, gait or frozen analyzer code was changed. The Stage C core
remains offline. The two obstacle probes use the frozen centered scene,
initial x/y and gait phase zero, the Phase-1 steps profile, a 25-second bounded
controller argument, the historical B1 development domain 231, and metadata
seed label 11. The seed label does not by itself prove randomized plant input.
These shortened probes intentionally cannot establish full-profile acceptance.
No holdout was opened or run.

| Run below `example/cpp/experiments/_runs/` | Actual outcome |
| --- | --- |
| `b1_research_baseline_5cbc547_20260907_0001` | Terrain execution enabled; frozen B1 FAIL; no crossing; hard-posture stop |
| `b1_research_sensoronly_5cbc547_20260907_0003` | Terrain actuation off; no crossing; controlled WBC stop; B1 analyzer refuses missing execution-only fields |
| `phase2_b0_development_steps_r0_20260907_000505_baseline` | Simulator fails before DDS readiness; no locomotion evidence |
| `phase2_b0_development_steps_r0_20260907_000505_terrain` | Full flat steps terrain member: legacy Phase-1 quantitative PASS; independent settling defect witness |

The failed baseline makes this an **incomplete B0 pair**, regardless of the
terrain member's legacy PASS. It is not a fresh full B0 admission result.

`physical_results.json` embeds the two probe manifests and hashes of every
retained run artifact. `manifest.json` binds the runtime source, binaries,
scene/model, frozen contract/analyzers and flat-run inputs. Raw evidence stays
ignored and untouched; curated results here are independently reproducible.
The experiment script is retained in the raw recovery directory, alongside
configuration/build/test logs. `ctest_35.txt` records 35/35 PASS after all
MuJoCo-dependent targets were built; tests are not a locomotion verdict.

## What actually happened at the obstacle

`reproduce_physical.py` reads MuJoCo ground truth only for post-run scoring.
Neither controller nor planner received scene geometry from this analysis.
At the first obstacle-foot contact:

| Probe | time (s) | leg | foot-site x/z (m) | foot GRF x/z (N) | base x (m) |
| --- | ---: | --- | --- | --- | ---: |
| execution on | 17.160 | FL | 0.678787 / 0.052098 | -36.604 / 10.638 | 0.530695 |
| sensor only | 17.032 | FR | 0.679158 / 0.030313 | -19.101 / 10.020 | 0.465720 |

The obstacle starts at x=0.700 m and its top is z=0.050 m. The forward contact
location, low foot site and backward force are evidence of leading-edge
interaction rather than a successful top-surface touchdown. Contact GRF is the
foot's total force, not a decomposed per-geom force; no stronger attribution is
claimed from that column alone. First nonfoot obstacle contact follows at
18.606 s / base x=0.675915 m with execution, and 19.736 s / base x=1.023065 m
sensor-only. Maximum base x is only 0.987377 / 1.075247 m respectively. Neither
body gets past the rear edge at 1.2 m, so neither can satisfy all-feet crossing
or a stable exit.

The first required plan rejection is data.csv line 6607, state time 15.044 s,
base x=-0.006067 m, failure 5 and geometric margin 0.012095610 m. Execution
loses a usable plan at line 6656, 15.142 s, 98 ms later, while the body is still
at x=0.014628 m. The longest later no-plan interval is 17.548--19.820 s.
`baseline_first_anomalies.txt` retains the row-level timeline. Its WBC mask
columns are diagnostic fields reset when a terrain plan is unavailable; zero
there must not be read as zero actual measured contact force. The chronology
establishes rejection-to-expiry before collision, but not that expiry alone
caused the collision.

Frozen B1 reports 1,344 required-plan rejection rows, 41 execution rows without
a WBC plan, coherence 0.008996, and 422 nonfoot collision rows in its evaluated
window. The full retained ground-truth streams contain 1,921 / 549 nonfoot
collision rows; those counts include later stop/fall portions and must not be
substituted for the frozen analyzer's windowed number. Its reduced reporting
window also explains why posture extrema in the full Phase-1 result are larger
than the terrain-window extrema. This is not a same-input replay, so the two
runs' divergence is not a causal estimate of the planner alone.

The existing force-balance diagnostic for the execution probe reports p95
0.435483 N and maximum norm 12.943693 N under its 20 N tolerance. This confirms
only that diagnostic's force accounting, not dynamic feasibility or stability.

An independent pointwise check finds no in-tolerance sample at all among the
4,000 samples of the 2 m/s hold: minimum absolute error is 0.252282 m/s against
0.150 m/s tolerance. The 3 m/s hold has only 78 in-tolerance samples among
4,001. The audited windows are the original analyzer's windows; these facts do
not invent evidence beyond the recorded hold or assert an unobserved deadline.

## Execution and certification findings

The production execution path is still the legacy per-leg planner. The new
centroidal core is not used by the controller. The legacy plan has eight
20 ms samples (last knot +0.140 s), while the MPC uses eight samples whose
spacing can be 30 ms; absolute horizon coverage is not guaranteed. The
24-knot correction is currently shadow-only. In-flight gait targets are
latched by leg while MPC can consume a later plan. WBC's high-speed velocity
and attitude tasks subsequently replace selected SRBD acceleration components.
A feasible offline centroidal trajectory therefore would not certify the
current commanded trajectory merely by connecting its output to MPC.

A newly isolated lift-accounting defect uses `max(CLI lift, runtime lift)` as
the already-applied lift. Both obstacle probes request 0.200 m on the CLI, but
the active low-speed scheduler uses 0.035 m. The resulting subtraction can
suppress the planner's required extra clearance. More generally, the execution
endpoint blend differs from the swept trajectory checked by terrain feasibility.
The native C++ `swing_path_witness.cpp` uses the production interpolation
helpers. Its reachable-phase, controlled nominal-path fixture has checked
foot-height algebra margin 0 and executed-algebra minimum margin -0.043863 m
at phase 0.176. This is specifically **not** a call to full
`CheckSwingClearance`, IK, unknown-map validation or the complete gait kernel.
It establishes that the two algebraic paths differ, not that a fully feasible
robot trajectory was executed unsafely. An earlier draft with worst phase
before its latch was independently rejected and is retained as rejected in the
raw analysis directory. Neither witness claims a scalar lift change fixes B1. The lower-leg sweep also explicitly skips unknown geometry
outside the map footprint, which cannot be advertised as fail-closed unknown.

The previously recorded T13 aerial-versus-transfer-contact conflict and 15 mm
geometric diagnostic remain unchanged and separately labeled. No geometric
margin, synthetic reduced-dynamics certificate, or logging counter here is
claimed to prove full-body execution feasibility.

## Independent diagnostic and reproduction

`example/cpp/tools/analysis/audit_phase1_settling.py` is named
`phase1-settling-coverage-v1`. It retains the original transition selection,
velocity tolerance, one-second tail duration and scenario settling limits.
It requires the first sample bracketing the full one-second endpoint, finite
input, a complete tail, and an individually satisfied deadline. A separately
reported 0.05-second maximum sample-gap assumption guards diagnostic coverage;
this is **not** an edit to a frozen acceptance threshold. Invalid profiles,
missing required CSV columns and unknown CLI scenarios fail closed. It does
not evaluate every B0 criterion, controller stability, or a full B1 verdict.

Native verification: 10/10 synthetic tests PASS. The mixed pass/fail test calls
the actual frozen analyzer for the legacy comparison. Cases include complete
nonsettling data, truncated tails, missing/NaN input, malformed profiles,
irregular 2 ms sampling, gaps, and late settling. On the actual flat CSV, the
five transition statuses are observed / unsettled / unsettled / observed /
observed. The old finite maximum remains 0.248001496 s and PASS, while the
independent audit exits 1 (FAIL). `settling_audit.json` and
`independent_transition_pointwise.json` agree on the two missing witnesses.

From the native repository root:

```bash
python3 -m unittest discover -s example/cpp/tools/analysis -p test_audit_phase1_settling.py -v
python3 example/cpp/tools/analysis/audit_phase1_settling.py \
  example/cpp/experiments/_runs/phase2_b0_development_steps_r0_20260907_000505_terrain \
  --profile example/cpp/configs/phase1_velocity_steps.csv
# The audit above is expected to exit 1; it must not overwrite the old analyzer result.
python3 docs/research/evidence/b1_research_audit_20260907/reproduce_physical.py .
python3 docs/research/evidence/b1_research_audit_20260907/reproduce_first_anomalies.py \
  --run-dir example/cpp/experiments/_runs/b1_research_baseline_5cbc547_20260907_0001 \
  --output /tmp/b1_first_anomalies_fresh.txt
g++ -std=c++17 -O2 -Wall -Wextra \
  -Iexample/cpp/terrain -Iexample/cpp/kinematics \
  -I/opt/unitree_robotics/include -I/opt/unitree_robotics/include/ddscxx \
  docs/research/evidence/b1_research_audit_20260907/swing_path_witness.cpp \
  -o /tmp/b1_swing_path_witness_20260907
/tmp/b1_swing_path_witness_20260907
```

The raw CSVs remain local and hash-addressed; a checkout without them can run
the synthetic tests and algebra witness, but cannot reproduce the actual-run
metrics. Do not fabricate a run result from the curated summaries alone.

## Environment recovery

`simulate/mujoco` was restored as an ignored symlink to the installed MuJoCo
3.3.6 distribution. Controller, simulator and all 35 CTest targets build/run.
WSL's new-command channel subsequently hung while Linux and its filesystem
remained responsive. An idle process snapshot was retained before a terminate
attempt, which also hung. Work continued through the already-running native
WSL SSH daemon using a temporary localhost-only, no-forwarding task key, with
the server public key independently verified through the local filesystem.
The key is removed at closeout. No administrative service reset was performed.

The B0 domain-220 failure is separately explained by Windows UDP exclusions
covering 62355--62654, including its default CycloneDDS participant ports;
domain 221 starts at an available participant port 62660 and ran. No domain,
port-base override, network setting or frozen manifest was changed to conceal
the failed member. The earlier missing-MuJoCo blocker is now resolved.

## Research judgment and next decision

Do not tune toward the old aggregate PASS or declare a B1 candidate. First
agree a versioned, fully observed per-transition settling verdict and a
running-trot-compatible terrain execution certificate, while retaining all old
baseline outcomes. Then unify the actual executed swing/contact/body/force
trajectory and its validity/commitment semantics before using a planner's
certificate as an execution guarantee. The next physical experiment should
isolate a concrete certified-versus-executed path correction; widening 15 mm,
raising a CLI lift, or patching planner score order alone is not supported as a
solution by this evidence.
