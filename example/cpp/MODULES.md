# example/cpp 模块索引

全局跳转也见 [`docs/CODE_GUIDE.md`](../../docs/CODE_GUIDE.md)。

## 可执行文件

| 目标 | 入口 | 实现 |
|---|---|---|
| `real_trot_go2` | `trot/real_trot_go2.cpp` | `trot_cli.*` + `trot_task.*` + `trot_experiment_*` |
| `real_leg_lift_go2` | `real_leg_lift_go2.cpp` | `leg_lift_cli.*` + `leg_lift_*` |
| `stand_go2` / `hold_pose_go2` / `track_*` | 同名 cpp | 单体小工具 |
| `test_*` | 同名 cpp | 单测 |
| `analyze_contact_torque_replay` | `tools/analysis/` | 离线分析 |

## trot 阅读顺序

1. `trot/trot_cli.cpp` — 参数从哪来
2. `trot/trot_task.*` — 站立/行走/趴下
3. `trot/trot_experiment.h` — 剩余运行时状态
4. `trot/trot_experiment_control.cpp` 搜 `SECTION:` — 主环
5. `trot/trot_experiment_gait.cpp` / `gait/raibert_trot_kernel.h` — 脚点
6. `trot/trot_experiment_wbc.cpp` — 力/力矩
7. `trot/trot_experiment_diagnostics.cpp` — 门禁与 CSV

## leg-lift 阅读顺序

1. `leg_lift_cli.cpp`
2. `leg_lift_types.h` — StepConfig / 常量
3. `leg_lift_control.cpp` 搜 `SECTION:`
4. `leg_lift_world.cpp` — 世界系反馈
5. `leg_lift_diagnostics.cpp`

## 约定

- Named `go2_*` directories stay under `experiments/`
- Other runs → `experiments/_runs/`
- Batch scripts → `scripts/batch/`

## 本地 include 关系（生成）

- `leg_lift_cli.cpp` → `leg_lift_cli.h`
- `leg_lift_control.cpp` → `leg_lift_experiment.h`, `go2_forward_kinematics.h`, `go2_inverse_kinematics.h`
- `leg_lift_diagnostics.cpp` → `leg_lift_experiment.h`, `go2_forward_kinematics.h`, `go2_inverse_kinematics.h`
- `leg_lift_lifecycle.cpp` → `leg_lift_experiment.h`, `go2_forward_kinematics.h`, `go2_inverse_kinematics.h`
- `leg_lift_world.cpp` → `leg_lift_experiment.h`, `go2_forward_kinematics.h`, `go2_inverse_kinematics.h`
- `real_leg_lift_go2.cpp` → `leg_lift_cli.h`, `leg_lift_experiment.h`
- `real_trot_go2.cpp` → `trot_cli.h`, `trot_experiment.h`
- `trot_cli.cpp` → `trot_cli.h`
- `trot_experiment_control.cpp` → `trot_experiment.h`, `contact_wrench_projected_allocator.h`, `contact_state_filter.h`, `go2_contact_torque_mapping.h`, `go2_inverse_kinematics.h`, `motion_frame_utils.h`
- `trot_experiment_diagnostics.cpp` → `trot_experiment.h`, `contact_wrench_projected_allocator.h`, `contact_state_filter.h`, `go2_contact_torque_mapping.h`, `go2_inverse_kinematics.h`, `motion_frame_utils.h`
- `trot_experiment_gait.cpp` → `trot_experiment.h`, `contact_wrench_projected_allocator.h`, `contact_state_filter.h`, `go2_contact_torque_mapping.h`, `go2_inverse_kinematics.h`, `motion_frame_utils.h`
- `trot_experiment_lifecycle.cpp` → `trot_experiment.h`, `contact_wrench_projected_allocator.h`, `contact_state_filter.h`, `go2_contact_torque_mapping.h`, `go2_inverse_kinematics.h`, `motion_frame_utils.h`
- `trot_experiment_wbc.cpp` → `trot_experiment.h`, `trot_true_dynamics.h`, `contact_wrench_projected_allocator.h`, `contact_state_filter.h`, `go2_contact_torque_mapping.h`, `go2_inverse_kinematics.h`, `motion_frame_utils.h`


## leg_lift 相位方法（持续拆分中）

已抽出（`leg_lift_control.cpp`）：

- `PhaseStandUp` / `PhaseStandSettle` / `PhaseWeightShift`
- `PhaseFootLift` / `PhaseFootSwing` / `PhaseLandingHold` / `PhaseBodyReturn` / `PhaseFootLowerActive`
- 上下文：`LowCmdScratch`（`cycle_time` 等嵌套相位共享量）

主链剩余：仅 publish/log 尾段（`SECTION:` 跳转）。

## 清晰化进度快照

- 布局：`apps/` `trot/` `gait/` `kinematics/` `contact/` `wbc/` `util/` `leg_lift/` `tests/`
- trot：`TrotTask` 拥有站立/行走/趴下；`TrotExperiment` 保留 DDS/gait/WBC/诊断
- leg_lift：main/CLI/模块拆分完成；相位已抽 StandUp/Settle/WeightShift/FootLift/FootSwing/LandingHold/BodyReturn
- 单测：example/cpp/build/test_* 应全绿

## trot 相位方法

- `TrotTask::PhaseLieDown` / `PhaseStandUp` / `PhaseStandSettle` / `PhaseStopToStand` / `BeginGait`（`trot/trot_task.cpp`）
- 主链其余段：快照/时钟/WBC 判定/gait 执行/命令写入/诊断

## 体量（约，自动更新）

| 项 | 值 |
|---|---|
| leg `LowCmdWrite` | 218 行（含 AUTO-TOC） |
| trot `LowCmdWrite` | 107 行（含 AUTO-TOC） |
| leg Phase* | 11 |
| trot Phase* | 6 |
| mains | ~50 行级 |

## trot 主环已抽方法（`trot_experiment_control.cpp`）

- `SnapshotState` / `MotionClockStep` / `ComputeWbcPrimaryActive`
- `PhaseRunGait` / `WriteMotorCommands`（wbc 主控 vs 位置混合）
- `UpdateWbcShadowAndTorqueFf` / `UpdateJointVelocityFeedforward` / `UpdateGaitWorldDiagnostics`
- 相位：`PhaseLieDown` / `PhaseStandUp` / `PhaseStandSettle` / `PhaseStartGait` / `PhaseStopToStand`

## leg_lift 主环已抽方法（`leg_lift_control.cpp`）

- `MotionClockStep` / `TempoGovernorScale` / `UpdateAttitudeFeedback`
- `ApplyTaskSpaceIk` / `WriteMotorCommands` / `PublishLowCmdWithCrc`
- 相位：StandUp / StandSettle / WeightShift / FootLift / FootSwing / FootLowerActive / LandingHold / BodyReturn / BetweenCycles / NeutralSettle / TerminalCorrection
