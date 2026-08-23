# Terrain sensing progress log

Overnight executor. Stages go in order. Each entry: stage, runs, rate, evidence.

## P0 (baseline, already on HEAD ea0255b)

Lidar heightmap `rt/go2/lidar_heightmap` + `--sensor-map`. Commit `ce8de92`.

## P1 observe 2026-08-23 — PASS 3/3 scenes

- Change: observe plans in base_link xy; FR/FL +0.10 m lookahead; CSV plan/z; forward samples at 0.20/0.40/0.60/0.80 m. P0 fix: lidar world grid [-2,20]×[-2,2] after `p1_flat_n1` walked off x=3 (`known_cells=0`).
- Flat (`scene_leg_lift_demo.xml`) 3/3: walk look kValid z=0 after ~80 unknown settle samples (`p1_flat_n{1,2,3}_h2_2026-08-23`). max_look_z=0.
- 10 cm barrier 3/3: look first z=0.10 at base_x≈0.07–0.17, status kValid; riser is kNoSupportPatch not kStepTooHigh (10 cm < max_step_up 0.14). h3 map: 0.10 at 0.20–0.40 m then 0 behind the box (`p1_b10_n{1,2,3}_h2`, `p1_b10_n1_h3`).
- Stairs 3/3: fwd profile 0.10→0.30→0.50 (`p1_stair_n{1,2,3}_h3_2026-08-23`). Fourth tread often unk (occlusion / window). Observe-only still stalls on the first riser.
- Dual accept: CSV summaries in `_runs/*/observe_summary.txt`; frames `C:\Workspace\p1_p1_*_frames`; 720p30 `OneDrive/收件箱/go2_p1_observe_2026-08-23/`.
- ctest 27/27 after controller rebuild.
