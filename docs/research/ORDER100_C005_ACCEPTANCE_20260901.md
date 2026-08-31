# Order-100 C-005 acceptance evidence

Source SHA: ccde8917acb8b263005938b88ff28c88b99d7757 (ccde891). No full B1 or C-006 was run.

## Deterministic fault harness

Command: example/cpp/build/test_contact_fusion_fault_harness > docs/research/evidence/ORDER100_C005_FAULT_HARNESS.csv

The tracked CSV emits every step as raw_mask,filtered_mask,planned_mask,fused_mask,reason,fallback_stage,guard,grace_remaining. It covers stable support, noisy-off, recovery, early/late touchdown, grace expiry through N/N+1/N+5/N+25, reset, planned-only input, and identity-mismatch fallback. The planned-only row is raw=8, filtered=0, planned=8, fused=0; fused is never promoted from planned. Adapter identity mismatch is also exercised by test_terrain_interfaces.

## B0 flag-off fixed pair

Command: TROT_CPU_AUTOPIN=1 SUSTAINED_SPRINT_DURATION_S=40 SUSTAINED_SPRINT_WALL_TIMEOUT_S=75 bash example/cpp/scripts/run_phase2_b0_fixed_pair.sh development 0

The wrapper observation timed out after both raw runs completed; both analyzers were rerun directly. analyze_phase2_b0.py returned 0 with acceptance_status=PASS, no_terrain_actuation=true, no_plan_consumer=true, no_plan_publish=true, planner_deadline_misses=0, planner_updates=2606, and both safety_status=0.

## Three serial normal full-approach probes

All used phase2_step_5cm.xml, Base=4000 DDS preload, domains 229/231/232, --terrain-planner --stage-c-execution, TROT_TERRAIN_C004_DIAGNOSTICS=1, TROT_TERRAIN_C005_DIAGNOSTICS=1, and 30 s controller duration. They were no-gate development probes, not a campaign.

| run | C005 records | FK valid/source | C004 consumed | gait/MPC identity | N/N+1/N+5/N+25 records | ID eq max | friction max | torque max Nm | promotions/demotions | safety |
|---|---:|---|---:|---|---|---:|---:|---:|---:|---|
| approach_4_long | 1671 | all / state_q+base_pose_fk | 1 | 222/222/9217295673002494915 = 222/222/9217295673002494915 | 22/48/125/1127 | 5.01e-5 | 1.018068 | 32.121881 | 23/26 | PASS |
| approach_5_long | 1533 | all / state_q+base_pose_fk | 1 | 226/226/18014457394627927319 = 226/226/18014457394627927319 | 28/56/144/1101 | 4.29e-5 | 1.011567 | 28.517222 | 22/25 | PASS |
| approach_6_long | 1520 | all / state_q+base_pose_fk | 1 | 229/229/651264811586172209 = 229/229/651264811586172209 | 19/31/88/212 | 4.31e-5 | 1.000482 | 21.211552 | 13/15 | PASS |

The C005 invariant fused & ~(filtered|robust_support) == 0 held for every record in all three runs. Planned contact remained a separate prediction field. All ID-WBC rows had wbc_full_id_ok=1; RNE residual matched the recorded equality residual. No first real C005 fusion caused an immediate unsafe stop.

Raw artifacts remain in ignored _runs/order100_c005_approach_{4,5,6}_long directories; the deterministic fault CSV is tracked above.
