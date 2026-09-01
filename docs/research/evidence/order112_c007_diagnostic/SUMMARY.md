# Order112 C-007 diagnostic bundle

- Probe: exactly one bounded C-007 diagnostic probe at source SHA `54561c0e021493a0c450415ff986c81a68fbd057`; no rerun or tuning.
- Contract: Stage-C v2 only; terrain crawl fallback-only; V3-C off; seed 11; simulator/domain lock and 60 s wall timeout; controller duration 30 s.
- Exact first planner failure: row 2170, state_tick_s 6.542000000.
- Exact first support-infeasible: row 3099, state_tick_s 8.400000000.
- Terminal RR failure: row None, state_tick_s insufficient.
- First causal subgate: insufficient; see `first_causal_subgate.json`.
- Patch/path/map epoch witness: insufficient; see `map_epoch_witness.json`; required-leg empty candidate gate is at `example/cpp/terrain/terrain_planner.h`, fed by `EvaluateFoothold`/`CheckSwingClearance` in `example/cpp/terrain/terrain_feasibility.h`.
- Order110/096 alignment is recorded in `order110_order096_alignment.json`; no historical run was rerun.

## Runner status

```text
started_at=2026-09-01T15:52:53+08:00
git_head=54561c0e021493a0c450415ff986c81a68fbd057
git_branch=phase2-b1-b3
git_dirty=false
simulator_sha256=52df364581d50aaf2324312922a15d676147b17058d8171bbc00a51d3a5d8f25
controller_sha256=0f4e88422af4fd6584cd95c9d0b7ad2494706e1ed95f01e43dc96162f142d6df
scene_sha256=ab2106afe65c86827aae02158e3b35b5ebb61e2b14d3665ddbd7c971e7573b0e
scene_file=/home/che/dev/go2-workspace/current/unitree_robots/go2/phase2_step_5cm.xml
phase2_milestone=B1
display=:0
runtime_dir=/run/user/1000
headless=true
camera_follow=false
terrain_lidar=true
lockstep=false
lockstep_trace=
sim_cpu_affinity=2,5
controller_cpu_affinity=4
controller_writer_cpu_affinity=3
terrain_worker_cpu_affinity=6
sim_lidar_cpu_affinity=5
sim_physics_cpu_affinity=2
sim_bridge_cpu_affinity=2
argv=--headless --wall-clock-motion --wbc-full --gait-pattern running-trot --kernel raibert-trot --period 0.50 --duty 0.75 --step-length 0.15 --foot-lift 0.08 --tau-limit 45 --velocity-max-accel 0.80 --velocity-max-decel 1.20 --velocity-max-jerk 4.0 --velocity-command-script example/cpp/configs/phase2_b1_velocity_0p3.csv --terrain-planner --stage-c-execution --phase2-milestone B1 --domain-id 232 --scene-file unitree_robots/go2/phase2_step_5cm.xml --controller-duration 30
controller_argv_shell=--wall-clock-motion --wbc-full --gait-pattern running-trot --kernel raibert-trot --period 0.50 --duty 0.75 --step-length 0.15 --foot-lift 0.08 --tau-limit 45 --velocity-max-accel 0.80 --velocity-max-decel 1.20 --velocity-max-jerk 4.0 --velocity-command-script example/cpp/configs/phase2_b1_velocity_0p3.csv --terrain-planner --stage-c-execution --domain-id 232 
profile_path=/home/che/dev/go2-workspace/current/example/cpp/configs/phase2_b1_velocity_0p3.csv
profile_sha256=61c260defafa21ef5c171380a4b93e972c59af34071e9e8f5dcf0b9f2b26190e
seed=11
event_script_hash=
controller_duration_s=30
max_cycles_requested=
domain_id=232
run_mode=bounded
task=
wall_timeout_s=60
stop_file=/home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/order112_c007_diagnostic/stop.request
contact_ground_truth_file=/home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/order112_c007_diagnostic/contact_ground_truth.csv
contact_ground_truth_dynamics_analysis_file=/home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/order112_c007_diagnostic/contact_ground_truth_dynamics_analysis.txt
environment_file=/home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/order112_c007_diagnostic/environment.txt
controller_status=0
safety_status=0
quality_status=0
analysis_status=0
ground_truth_status=0
contact_ground_truth_analysis_file=/home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/order112_c007_diagnostic/contact_ground_truth_analysis.txt
dynamics_status=0
contact_ground_truth_dynamics_analysis_file=/home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/order112_c007_diagnostic/contact_ground_truth_dynamics_analysis.txt
dynamics_tolerance_n=10
completion_status=0
phase1_quantitative_status=1
terrain_analysis_status=1
finished_at=2026-09-01T15:53:06+08:00

```
