# Analysis-tool index

`example/cpp/tools/analysis/` contains both current protocol analyzers and retained diagnostics. “Safe for new work” means the tool has an indexed protocol owner and its thresholds/metric semantics are suitable for a new experiment; it does not mean that it can replace a protocol-specific acceptance analyzer.

| Family / scripts | Protocol owner | State | Safe for new work? |
|---|---|---|---|
| `analyze_sustained_running.py`, `analyze_sustained_sprint.py`, `analyze_running_gait.py`, `analyze_speed_acceptance.py`, `analyze_natural_gait.py`, `analyze_straightness.py` | High-speed and natural/straight gait protocols | Current | Yes, with the matching acceptance document |
| `inspect_speed_profile.py`, `inspect_speed_transition.py`, `inspect_speed_window.py`, `inspect_stop_csv.py`, `summarize_high_speed_acceptance.py`, `summarize_wbc_speed_probe.py` | High-speed evidence review | Current diagnostic | Yes, as supporting analysis; not a standalone acceptance gate |
| `analyze_leg_lift_repeatability.py`, `analyze_leg_sequence.py`, `analyze_periodic_leg_lift.py`, `analyze_real_leg_lift.py`, `analyze_single_step.py`, `analyze_two_step.py`, `analyze_weight_shift.py`, `analyze_weight_shift_scan.py`, `analyze_locomotion_progress.py` | Low-level action and locomotion experiments | Current/supporting | Yes, with an experiment record |
| `analyze_base_dynamics_closure.py`, `analyze_base_mass_matrix.py`, `analyze_bounded_dynamic_target.py`, `analyze_dynamic_target_feasibility.py`, `analyze_full_dynamics_closure.py`, `analyze_dynamics_task_replay.py`, `analyze_contact_dynamics.py`, `analyze_contact_ground_truth.py` | Dynamics and ground-truth validation | Current/supporting | Yes, with explicit metric semantics |
| `analyze_constraint_wrench_truth.py`, `analyze_contact_conditioned_shadow.py`, `analyze_contact_moment_shadow.py`, `analyze_contact_torque_ground_truth_replay.py`, `analyze_task_wrench_gap.py`, `analyze_moment_slack_sweep.py` | Contact/wrench diagnostics | Current diagnostic | Yes, for diagnostics; no unreviewed claim promotion |
| `analyze_fullbody_wbc_replay.py`, `analyze_rate_aware_fallback.py`, `analyze_replay_torque_continuity.py`, `analyze_sequence_constrained_shadow.py`, `analyze_torque_rate_limit_shadow.py`, `analyze_torque_rate_limit_wrench_shadow.py`, `analyze_wbc_fallback_policy.py`, `analyze_wbc_task_features.py`, `analyze_wbc_torque_replay.py`, `analyze_wbc_wrench_components.py` | WBC/MPC replay and controller diagnostics | Current/supporting | Yes, only as declared replay/diagnostic evidence |
| `perturb_ground_truth_orientation.py`, `perturb_replay_input.py`, `replay_rate_aware_state_machine.py`, `plot_lowcmd_lowstate_tracking.py` | Historical replay/perturbation tooling | Historical/diagnostic | No for new acceptance; preserve for provenance |
| `analyze_acceleration_consistent_task_target.py`, `analyze_hold_candidate.py`, `analyze_perturbation_robustness.py` | Exploratory task and robustness probes | Historical/supporting | No without a new protocol owner and review |
| `analyze_actual_fk_crossing.py` | C-007 harness-only measured FK crossing evidence | Development-only | No; never replaces frozen acceptance analyzers |

When adding an analyzer, update this index and the experiment catalog in the same PR. Never rename old analyzer paths merely for cleanup.
