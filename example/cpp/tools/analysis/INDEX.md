# Analysis-tool index

For Phase 2, start with [`CURRENT.md`](../../../../CURRENT.md). Only the frozen
protocol may select an analyzer or interpret its result. This index classifies
tools; it cannot create an acceptance route.

| Phase 2 tools | Owner | Use |
|---|---|---|
| `../analyze_phase2_b0.py` | `docs/research/PHASE2_ACCEPTANCE.md` | canonical B0 analyzer |
| `../analyze_phase2_terrain.py` | same frozen contract | B1/B2/B3 terrain analyzer; a pass is candidate evidence only |
| `../read_phase2_b0_domains.py` | holdout manifest | read frozen DDS allocations |
| `../compare_lockstep_canary.py` | determinism diagnostics | compare lockstep output; never substitutes for runtime acceptance |

`example/cpp/tools/analysis/` contains accepted Phase 1 analyzers and retained
diagnostics. “Safe for new work” means safe only inside its named protocol; no
generic analyzer can replace a target-specific acceptance analyzer.

Other tools one directory above are either operational helpers
(`profile_cli.py`, `write_run_manifest.py`) or retained Phase 1
reactive/automatic-environment tooling. They are not Phase 2 analyzers unless
`CURRENT.md` and the frozen acceptance contract explicitly name them.

| Family / scripts | Protocol owner | State | Safe for new work? |
|---|---|---|---|
| `analyze_sustained_running.py`, `analyze_sustained_sprint.py`, `analyze_running_gait.py`, `analyze_speed_acceptance.py`, `analyze_natural_gait.py`, `analyze_straightness.py` | Accepted Phase 1 gait protocols | Phase 1 | Only with the matching frozen Phase 1 document |
| `inspect_speed_profile.py`, `inspect_speed_transition.py`, `inspect_speed_window.py`, `inspect_stop_csv.py`, `summarize_high_speed_acceptance.py`, `summarize_wbc_speed_probe.py` | High-speed evidence review | Diagnostic | Supporting analysis only |
| `analyze_leg_lift_repeatability.py`, `analyze_leg_sequence.py`, `analyze_periodic_leg_lift.py`, `analyze_real_leg_lift.py`, `analyze_single_step.py`, `analyze_two_step.py`, `analyze_weight_shift.py`, `analyze_weight_shift_scan.py`, `analyze_locomotion_progress.py` | Historical low-level action experiments | Historical/supporting | No for Phase 2; only for an explicitly scoped historical study |
| `analyze_base_dynamics_closure.py`, `analyze_base_mass_matrix.py`, `analyze_bounded_dynamic_target.py`, `analyze_dynamic_target_feasibility.py`, `analyze_full_dynamics_closure.py`, `analyze_dynamics_task_replay.py`, `analyze_contact_dynamics.py`, `analyze_contact_ground_truth.py` | Dynamics and ground-truth checks | Diagnostic | Supporting analysis only |
| `analyze_constraint_wrench_truth.py`, `analyze_contact_conditioned_shadow.py`, `analyze_contact_moment_shadow.py`, `analyze_contact_torque_ground_truth_replay.py`, `analyze_task_wrench_gap.py`, `analyze_moment_slack_sweep.py` | Contact/wrench checks | Diagnostic | Supporting analysis only |
| `analyze_fullbody_wbc_replay.py`, `analyze_rate_aware_fallback.py`, `analyze_replay_torque_continuity.py`, `analyze_sequence_constrained_shadow.py`, `analyze_torque_rate_limit_shadow.py`, `analyze_torque_rate_limit_wrench_shadow.py`, `analyze_wbc_fallback_policy.py`, `analyze_wbc_task_features.py`, `analyze_wbc_torque_replay.py`, `analyze_wbc_wrench_components.py` | WBC/MPC replay checks | Diagnostic | Supporting analysis only |
| `perturb_ground_truth_orientation.py`, `perturb_replay_input.py`, `replay_rate_aware_state_machine.py`, `plot_lowcmd_lowstate_tracking.py` | Historical replay/perturbation tooling | Historical/diagnostic | No for new acceptance; preserve for provenance |
| `analyze_acceleration_consistent_task_target.py`, `analyze_hold_candidate.py`, `analyze_perturbation_robustness.py` | Exploratory task and robustness probes | Historical/supporting | No without a new protocol owner and review |

When adding an analyzer, update this index and the experiment catalog in the same PR. Never rename old analyzer paths merely for cleanup.
