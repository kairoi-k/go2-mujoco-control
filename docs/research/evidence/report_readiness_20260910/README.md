# Teacher-report readiness audit, 2026-09-10
This is an evidence/report checkpoint, not a controller release, new physics run,
full Phase1 recertification, B1 acceptance, or solver selection decision.
REPORT_ZH.md is the teacher-facing narrative. Historical frozen files are unchanged.
## Findings
The archive phase1.tar.gz was recovered and SHA-256 verified against the retained
archive SHA256SUMS: d3cac74673bcf15f175c844f4f0c733aefa15bfa4841d6d33af210f559a369e4.
All 15 retained valid CSV runs record clean source
07cbc7efdd17b9960897e277f401654c46410e48. Five profile files are byte-identical to
that Git source. The archive also retains failed startup metadata; no omitted
startup becomes a successful trial. Individual file hashes are in phase1_reaudit.json.gz.
The recorded profile_sha256 field is absent; current profiles were instead
checked directly against the historical Git source, and hashed in the new audit.
This cannot retroactively manufacture an absent per-run profile hash.
Settling-only result: 9 PASS / 6 FAIL. All three steps and all three varying runs
fail. Across 42 selected transitions, 27 have a complete acceptable witness and
15 are unsettled. A separate NumPy window computation imports no audit code and
agrees on all 42 statuses. Ten existing diagnostic edge-case tests pass.
The original 15 legacy PASS verdicts remain unchanged. Nonfinite legacy values
are represented as explicit NONFINITE:NaN strings in the compact aggregate;
the original archived bytes are never edited. An initial JSON aggregation failed
on legacy NaN, after all 15 audits completed; aggregation was then repaired using
the saved audits. The final reproduction script creates a fresh output directory.
Commands are piecewise linear, including the profile named steps. 1->3 takes
10 seconds; 3->0 takes 12 seconds. The ramp settling diagnostic retains legacy
turning-point window semantics and does not certify continuous tracking of a
moving reference. No claim of random commands, rapid reversals, gait-phase
coverage, emergency stopping, or new full acceptance is justified.
First steps 0->1 legacy settling used only 0.220 seconds of tail; the 1->2
plateau also fails. Every varying run fails its first three selected transitions.
These are baseline defects worth resolving before adding stress trials.
The old B1 flat/step/debug raw analyses were replayed with reproduce_results.py;
parsed results exactly equal the historical results.json. 24 packet and 48 raw
hashes match. All 148 example/simulate/unitree_robots source entries match the
executed historical Git source eeb5d757620712759604d8c51b2b9075d05625dc.
Three documentation entries in the old source manifest belong to later closeout
content, so 152/155 full source entries match; see b1_verification.json. The old
verifier invoked against the current worktree failed CMakeLists first, correctly
showing that current source is not historical runtime source. No binary rebuild
or binary validation is claimed. The video SHA-256 is verified:
70d5afc8796399ef3c9b2dd0b365465770c1aaa733085a04f32fa017ce3c9edd.
One recorded crossing happened, but all-leg non-top contacts, 1/7 running cycles,
and 28.318ms clock drift retain NOT_CERTIFIED. Debug is attribution, not a repeat.
No fresh stress test or old-controller crossing repeat was started after the
reproducible velocity baseline failure was found. The user gets a report with
these explicit limits, not an unvalidated claim that tonight produced a finished robot.
## Reproduce
Canonical repo: /home/che/dev/go2-workspace/feat-stage-c-joint-planner.
Retained archive: /home/che/dev/go2-workspace/archive/evidence/phase1.tar.gz.
Use a NEW scratch directory. List the tar archive and select regular data.csv,
run_metadata.txt, run_manifest.json, environment.txt, phase1_analysis.json and
phase1_quantitative_analysis.json files below phase1/phase1-clean/
phase1_completion_20260825 and phase1_repeat_20260825. Extract without overwriting.
Original audit scratch is /tmp/go2-report-20260910/phase1-raw and is preserved.
```
python3 docs/research/evidence/report_readiness_20260910/recheck_phase1.py \
  --raw-root /tmp/go2-report-20260910/phase1-raw --out /tmp/NEW-phase1-audit
python3 -m unittest discover -s example/cpp/tools/analysis -p test_audit_phase1_settling.py -v
python3 docs/research/evidence/b1_checkpoint_20260907/reproduce_results.py \
  --repo "$PWD" --output-dir /tmp/NEW-b1-replay
```
Compare parsed reports, allowing only output/raw-root path changes. The archive
file bytes and source profiles must match the bound hashes. Full audit results
are preserved as deterministic gzip (mtime=0). independent_windows.py and
make_figures.py use the original /tmp/go2-report-20260910 layout; the former
produces independently checked full-rate telemetry NPZs, the latter plots those.
Before replaying them elsewhere, reproduce that scratch layout from the archive;
outputs are derived artifacts, not raw experiments. No controller/physics runs.
Figures select the first valid run of each scenario, not a best trial; plot-only
subsampling is 10x, while checks use full-rate data. velocity_trace.mp4 is a labeled
telemetry cursor animation, NOT a robot simulation. recorded_5cm_traversal.mp4 is
the preserved measured-state video. Videos are delivered separately, hash-bound,
and not committed to Git. The teacher report distinguishes them explicitly.
The report repeats historical fixed-speed 3m/s evidence with attribution only;
this checkpoint does not re-audit that separate protocol. Joint-planning status
comes from whole_body_mpc_oracle_20260909, without a new simulation or best-solver claim.
