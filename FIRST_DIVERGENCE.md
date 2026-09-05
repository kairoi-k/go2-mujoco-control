# FIRST_DIVERGENCE

## 结论

在这组四路实验中，最早的可观测分叉是 control_index=0 的 state_tick：A=2268，B=1910，C=1910，D=1910。对应 lockstep ready barrier：A=2274，B/C/D=1914。它发生在 terrain worker 能消费第一份控制快照之前，属于 terrain lidar/bridge 与启动阶段的线程调度差异；不是 planner side effect、gait、contact 或 WBC 参数分叉。

B 的 worker no-op 已复现与 C/D 相同的 1910 起始 tick，因此“worker 运行/建图/planner”不是第一因果分叉。D 的完整 worker 只产生诊断更新；allow_actuation=false，无 plan consumed、无 gait target override、无 MPC plan consumed。

## 实验范围与可复现性

用户指定的 /home/che/dev/go2-mujoco-control 在当前 Ubuntu-22.04 中对应的实际 worktree 是：

- /home/che/dev/go2-workspace/current
- branch: fix/phase2-b0-runtime-integrity
- HEAD: 918bef1e2ae95362d51edd02dbdcb6bab8dd2101

四组使用同一 running-trot、period=0.14、duty=0.44、同一 phase1 varying velocity profile、同一 WBC/velocity 参数；均设置 SIM_LOCKSTEP=1、ack handshake、2 ms lockstep step、相同显式 CPU affinity，并串行持有 /tmp/go2_mujoco_experiment.lock。临时插桩只增加观测与 B/C worker 模式，实验后已删除；没有改 B0 阈值、gait/WBC 参数或功能路径。

原始证据目录：

- example/cpp/experiments/_runs/first_divergence_A_20260905
- example/cpp/experiments/_runs/first_divergence_B_20260905
- example/cpp/experiments/_runs/first_divergence_C_20260905
- example/cpp/experiments/_runs/first_divergence_D_20260905

四组 lockstep trace 均为固定 2 ms、0 violation、无 fail-closed：

| 组 | terrain | worker | barrier tick | trace rows |
|---|---|---|---:|---:|
| A | off | disabled | 2274 | 9135 |
| B | on | no-op | 1914 | 9274 |
| C | on | copy-only | 1914 | 9128 |
| D | on | full | 1914 | 9124 |

trace 的 control row 是控制器快照，不等于 simulator interval row；lockstep bridge 会重复发布同一冻结状态。

## 第一不同变量

以下比较按相同 control_index，字段均来自同一控制计算的前后快照。

| 字段 | 最早不同 index | A / B / C / D | 判断 |
|---|---:|---|---|
| state_tick | 0 | 2268 / 1910 / 1910 / 1910 | 首个不同变量 |
| measured body velocity x | 0 | 0.000019421 / 0.000020478 / 0.000020478 / 0.000020478 | state snapshot 下游 |
| motion_dt | 1 | 0.002379528 / 0.002315094 / 0.002092097 / 0.002073128 s | barrier 前仍用 wall clock |
| contact mask | 64 | 3 / 15 / 3 / 3 | 后续状态差异 |
| scheduler raw | 无 | 四组逐行相同 | 非首因 |
| gait_time / phase / cycle_index | 无 | gait start 后逐行相同 | 非首因 |
| stance_hold | 无 | 四组逐行相同 | 非首因 |
| kernel period / duty | 无 | 四组逐行相同 | 非首因 |
| WBC velocity target | 无 | 四组逐行相同 | 非首因 |
| scheduler shaped/applied | post index 2150、pre index 2151 | 由启动阶段 wall-clock 状态继承 | 下游时钟差 |

gait start 位于 control_index=2151。在 handoff 后按各 run 的起始 tick 做归一化，gait、scheduler raw、kernel、stance hold 和 WBC 目标没有 terrain worker 导致的控制分叉。

## 最小代码路径

1. example/cpp/scripts/run_trot.sh:64-66,392-394：--terrain-sensor-only 同时打开 simulator 的 --terrain-lidar 与 controller terrain；B 即使 worker no-op 也保留该 simulator sensor。
2. simulate/src/unitree_sdk2_bridge.h:303-312：terrain lidar thread 在 bridge 启动时创建；:323-358 中它以 SCHED_IDLE 运行，但仍复制 simulator state、获取 simulator mutex，并每 50 ms 做 mj_fwdPosition/raycast/HeightMap publish。
3. simulate/src/unitree_sdk2_bridge.h:680-697：lockstep ready barrier 完成前明确走 RunWallClock()；因此 barrier 之前 simulation tick 由 bridge/physics/lidar/DDS 的实际调度决定。
4. simulate/src/lockstep.h:296-334：ready barrier 直到收到 controller 第一条 command 才把当前 simulator tick 定为 handoff tick。也就是说 lockstep 只约束 handoff 之后，不会消除启动前的绝对 tick 差异。
5. example/cpp/trot/trot_experiment_lifecycle.cpp:42-56：DDS LowState callback 在 state_mutex_ 下复制最新 snapshot；:103-158 的 natural-settle 也读取同一 snapshot。
6. example/cpp/trot/trot_experiment_control.cpp:366-377：LowCmdWrite 先 SnapshotState，再 MotionClockStep。:1103-1116 的 wall_clock_motion 在 lockstep writer gate engaged 前覆盖 motion_dt；之后才由 lockstep_motion_clock 使用 state tick。
7. example/cpp/trot/trot_experiment_lifecycle.cpp:200-203,297-309 与 example/cpp/trot/trot_experiment_control.cpp:103-231,233-330：terrain worker 在 settle 后才启动；它通过 snapshot/map/planner 诊断链工作，terrain planner 配置固定 allow_actuation=false，没有写入 gait/WBC 目标。

## 因果分类

主分类：线程/调度。

下游表现：state snapshot 与启动时钟。terrain-enabled simulator 多出的 lidar thread 早于 ready barrier 存在，改变 bridge/physics/DDS 的竞争和 controller 第一次有效快照的到达时刻。measured body velocity 与 contact 的差异是不同 snapshot/物理状态的后果；motion_dt 是 barrier 前 wall-clock 采样的后果。

排除项：

- gait state：gait_time、phase、cycle_index、stance_hold 没有首因分叉。
- contact：contact 在 state snapshot 之后才分叉，不能解释 index 0 的 state_tick。
- planner side effect：B no-op 已有同样的首个 tick；D 的 planner 仅更新 terrain diagnostics。
- WBC：WBC velocity target 四组未分叉。

## 唯一最小验证

做了 A/D 一次固定预热验证。临时设置 simulator 在 settled 2.500 s 后等待 barrier，controller 等待 state_tick=2500，随后恢复正常 lockstep；验证目录：

- first_divergence_validation2_A_20260905
- first_divergence_validation2_D_20260905

两组第一控制快照均为 state_tick=2500，第一 lockstep interval 均从 2502 开始；gait_time、phase、cycle_index、stance_hold、scheduler raw、kernel period/duty、WBC velocity target 仍没有 terrain worker 分叉。D 确实有 planner updates，但没有 actuator/plan-consumed 结果。

验证同时暴露了边界：A 在下一控制 row 仍停留 2500，而 D 已到 2502；第一 row 的 filtered body velocity 也有约 3e-9 m/s 差异。这说明“固定同一个 tick”仍不足以消除 DDS snapshot release 的微差，但确认了原始 358 tick 偏移来自 barrier 前启动调度，而不是 gait/WBC/planner。该微差不应被宣称为 terrain functional effect。

## 置信度与建议

对“第一大分叉属于启动阶段线程/调度，经 state snapshot 与 wall-clock motion_dt 传播”的置信度：高。对独立进程之间 sub-tick snapshot 微差的精确归因：中；当前证据不足以把它归给 full planner。

建议的最小后续修复方向是 debug-only handoff contract：在自然 settle 后由 simulator/controller 共同确认一个 settled state tick，并冻结/发布同一份 immutable LowState snapshot；需要更强复现时使用 snapshot record/replay，而不是继续调 gait/WBC 或 B0 阈值。生产 B0 验收仍应与 lockstep 显微镜分开。

本次未提交功能性修复；最终源码已恢复干净，仓库只保留本报告。


## Lockstep handoff 收尾（2026-09-05）

加入 opt-in debug-only handoff：baseline 在 tick=4000 ms 记录 MuJoCo state，terrain 回放该 state；两侧 controller 都记录了 LOCKSTEP_HANDOFF_CONTROLLER_READY tick_ms=4000，terrain simulator 记录了 LOCKSTEP_HANDOFF_READY mode=replay tick_ms=4000。有效 F14 证据目录：

- example/cpp/experiments/_runs/phase2_b0_lockstep_development_fixed_3mps_r0_20260905_084957_baseline
- example/cpp/experiments/_runs/phase2_b0_lockstep_development_fixed_3mps_r0_20260905_084957_terrain
- example/cpp/experiments/_runs/f14_shared_handoff_20260905_valid.bin

第一次启动因共享路径未传入而在 bridge ready 前退出，没有纳入比较；随后只完成这一组有效 paired F14，没有继续重跑 lockstep。

有效 F14 的首个控制快照 nominal state_tick 都是 4.000 s，但它们不是同一份字节级 LowState：raw body velocity x 为 0.000022385 / 0.000022404 m/s，z 为 -0.000042902 / -0.000042922 m/s。按 control row 对齐，row=1 同为 tick=4000 ms 时 motion_dt 为 0.002072320 / 0.002016233 s；row=4 的 state_tick 已为 baseline=4004 ms、terrain=4002 ms。由此新的第一可观测分叉仍是 handoff 后的 state snapshot/clock scheduling，而不是 planner。

| 指标 | 结果 |
|---|---|
| scheduler requested/shaped/applied/accel/jerk | 仍相同；paired max diff=0 |
| gait period/duty | 仍分叉；首个 aligned row=2451，period=0.198180907 / 0.199536597，duty=0.497726134 / 0.499420746 |
| WBC velocity target | 仍分叉；首个非零 target aligned row=3119，1.210318214 / 0.000000000 m/s；paired max diff=3.0 m/s |
| lockstep trace | baseline 28144 rows、terrain 26957 rows，均 dt=2 ms、0 violation |

因果判断：原始 barrier 前的绝对 tick 偏移已被 debug handoff 隐去，但旧 divergence 不能据此宣布“完全属于 handoff artifact”。当前 debug 实现记录/回放的是 MuJoCo state，再由各 bridge 重建 LowState；它没有跨进程持久化并回放同一份 DDS LowState message。同时 controller 的 wall_clock_motion 仍在 gait_started 前覆盖 motion_dt，lockstep writer gate 只在之后 Engage。因此当前最小证据指向“state snapshot + controller/bridge scheduling”，planner side effect 仍被排除。置信度：对直接分叉高，对 sub-tick 数值差异的精确线程归因中。

按用户边界，本次到此停止 lockstep 修改和实验；下一步切回正常 wall-clock B0，单独调查 terrain lidar/DDS/thread scheduling 对 state snapshot 与启动时钟的影响。本次没有执行 wall-clock B0。

本次未提交功能性修复；debug-only handoff 源码改动保留在工作树供后续评审，报告本补充单独提交。

