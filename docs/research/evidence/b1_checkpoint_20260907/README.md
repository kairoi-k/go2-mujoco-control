# Bounded B1 recovery checkpoint, 2026-09-07

The interrupted work is closed as evidence, not as a B1 candidate. The latest
executed source remains `eeb5d757620712759604d8c51b2b9075d05625dc`. This checkpoint
changes documents and evidence/replay tooling only. No new closed-loop development run,
control/planning patch, threshold adjustment or acceptance modification was
performed during closeout. The old CURRENT is preserved verbatim and replaced
by a short live entrypoint to avoid executing obsolete next-step instructions.

## Actual capability and acceptance gap

The video shows a real completed 5 cm crossing from the interval-V2 actuation
run. Every foot achieves sustained top support; no nonfoot step contact or
runtime safety stop occurred. Interaction speed p05/median is
0.673269/0.850097 m/s. That is useful physical progress. It does not establish
reliable terrain-aware running traversal: all four legs record non-top contact
and only 1/7 complete interaction cycles satisfies the preserved V3 topology.
The state/profile clocks additionally drift 28.318033 ms, beyond V3's 20 ms
consistency gate. Keep this failure visible, not hidden behind wrapper status.

The same-SHA flat control completes with 46/49 good measured running cycles.
The debug third run is separately identified and is attribution-only; its
logging/scheduling differences prevent treating it as a repeatability trial.
Detailed raw-bound results are in `results.json`. Earlier 0.28 s period probes
remain in the previous iteration packet and were not rerun in this closeout.
No holdout or fresh full B0 acceptance campaign is claimed.

## Principal blocker: executable plans before impact

The latest timed step provides direct same-run evidence. In the 0.2 s preceding
and including FL's first step contact at 23.108 s, all 101 controller rows have
planner failure 4 (no-safe-foothold), no usable execution plan, no applied
terrain mask and no in-flight terrain target. FR's first contact at 23.146 s
has the same pattern over 100 rows. This is not evidence of a terrain target
being applied and then poorly tracked. Nominal locomotion can physically cross,
but the terrain planning/execution path is absent at the decisive entry.

The current source screens future touchdowns using a normalized swing from the
current measured FK foot. A currently force-supported foot need not be at the
future liftoff state, and a compliant ground contact is not a free-flight
initial condition. The recovered support audit and new debug witness identify
this as an important rejection mechanism. They do not prove it is the sole
cause of B1 impacts or that removing the gate would restore running topology.
Unknown coverage remains distinct. Contact force does not reveal penetration
depth; site minus radius is not an exact collision-distance measurement because
the collision sphere center is offset from the site.

Independent source review also keeps the following concerns explicit:
interior swing clearance and endpoint checks treat foot radius differently;
a late target latch restarts a position-only normalized curve; future-body and
contact-event timing remain incompletely represented. These are limitations or
hypotheses, not completed fixes. See `source_audit.md` for exact source locations
and the interruption inventory. No half-finished algorithm or ledger patch was
silently promoted into production.

The next bounded research step, proposed but not started, is one offline
same-state witness for current support versus future liftoff, with actual
collision-sphere geometry and an explicit unresolved/checked swing state.
Preserve landing unknown rejection and actual execution revalidation. Only
after that semantic test should one short flat/step controlled comparison be
considered. Do not start by broadly retuning WBC forces, relaxing all geometry
thresholds or replacing the solver: current evidence does not identify those
as the immediate bottleneck. Resolve the clock join separately for trustworthy
whole-profile evidence. The true-running topology and collision outcomes must
still be checked after any subsequent change.

## Verification and provenance

`verification.json` binds the retained 135 runtime/build/test sources and both
binaries to the executed runtime source. Fresh tests pass 44/44 controller,
3/3 simulator and 48/48 Python; their logs are included. The old Phase-2
acceptance, holdout and analyzer hashes are unchanged. The independent
force-plus-gravity impulse check in `step_impulse.json` has 20 ms residual
p50/p95/max 0.016171/0.043825/0.055254 Ns. It is an integral consistency
diagnostic, not a success threshold or a full-body feasibility certificate.

`manifest.json` binds every committed packet artifact, runtime source and
retained raw input. `verify_checkpoint.py` and `REPRODUCE.md` explain validation
and replay. Raw files stay immutable and ignored under `_runs`; restored raw
files are required to reproduce the measured results on another machine.
Seed/source equality does not imply bitwise wall-clock simulation repeatability.
The packet reports a development run, not a success-rate estimate.

The video is a recorded-state rendering with normal and quarter-speed segments.
`video_provenance.json` binds its inputs/output and records maximum 2 ms join
gap and 7.824 mm reconstructed foot-site discrepancy. It is unsuitable for
judging millimetre collision clearance by eye; its red labels use truth force
logs. It was copied to the user's OneDrive inbox and hash-checked.

## Environment and stopping point

MuJoCo 3.3.6 is available and the simulator tests pass. The previous Windows
WSL-launcher issue is separate from the working native runtime reached over
pinned localhost SSH. Agent quota interrupted the earlier auxiliary tasks;
this closeout resumed only evidence and source audits. Stashes, raw runs,
other worktrees and isolated temporary work remain preserved. No new long-run
autonomous research was started. The pushed containing commit is the recovery
checkpoint; B1 research remains open for a separately scoped next turn.
