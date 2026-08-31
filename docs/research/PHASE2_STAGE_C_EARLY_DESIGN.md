# Phase 2 Stage C 提前设计：接触时序、落点与机身轨迹联合规划

状态：DESIGN ONLY（Order-091，2026-08-31）；本文件只定义后续实现，不改变行为代码，不声称任何 B1-B3 门通过，也不包含仿真结果。基线 HEAD：`4ac6a82`。

## 0. 决策、边界与失败证据

脚本化 crawl 路线在 Order-089/090 后停止：截至 `4ac6a82`，已有 48 次全流程尝试，测得支撑见证和完整序列均为 0；低姿态 pivot 的单次 staged probe 也为 0/1，按 Order-090 stop rule 不再开跑。其失败不是“trot 永远不能跨 5 cm”的证明，而是冻结的 0.30 m/s、冻结接触时序、单刚体预览和落点/机身分离的组合没有形成可执行的动态计划。Order-089 还将 260 个 edge-slip 事件归因为落点/机身转移几何，而不是继续调权重可解决的问题。

本设计因此把地形策略从 `TerrainCrawlSequencer` 移到 `TerrainPlanner`：planner 输出一个不可变、带时间轴的完整计划；gait scheduler 只负责把计划适配为连续的执行轨迹；SRBD-MPC 消费同一计划的逐 knot 接触和落点；ID-WBC 仍是高频执行器和安全边界。`terrain_crawl_sequencer.h` 保留作归档/故障回退，不能删除，但不再是正常 terrain policy。

明确不做：不读 XML、geom、world-x、阶次或 oracle map；不在本单修改 C++、合约阈值、analyzer、canary；不把 FSM 写成“前腿再后腿”；不把计划接触当成实测接触；不因困难直接上 whole-body NMPC。

## 1. 最小改动集与模块归属（现状审计及 file:line seam）

### 1.1 数据和 planner：由 planner 产生唯一时序计划

现有 `TerrainPlannerInput` 在 `example/cpp/terrain/terrain_planner.h:42-126` 已有状态、当前/名义足端、`TerrainContactSchedule`、next-touchdown 时间；`TerrainPlanner::Build` 从 `:448-1067` 做 map 检查、候选生成、组合搜索和原子计划生成。当前 `CandidateTouchdownKnot`（`:1619-1661`）只能从外部给定的固定 schedule 反推出 knot，`SupportFeasibleSelection`/`SupportFeasible`（`:1211-1617`）只验证该 schedule，故这里是“枚举时序并和落点/机身共同评分”的主 seam。

最小未来改动：

* 在 `TerrainPlannerInput` 增加 `TerrainTimingBounds`（当前 period/duty、每腿下一次 touchdown、允许的最早/最晚时间、窗口边界），而不是让 planner 调 gait setter。
* 在 `TerrainMotionPlan` 增加 `TerrainContactTiming`：`period_s`、`duty_factor`、每腿 phase offset 或（推荐）绝对 `touchdown_time_s`，每 knot 的 planned-contact，和 timing source/validity。不要同时把 phase offset 与 touchdown time 当两个独立权威；绝对 touchdown time 是执行、MPC、日志的唯一时间变量，phase 仅为派生遥测。
* 把当前 `gait_phase/gait_period_s/duty_factor`（`example/cpp/terrain/terrain_motion_plan.h:306-376`）从“生成时的描述”升级为计划时序的快照；`predicted_foothold[k][leg]`、`body_reference[k]`（`:336-340`）继续作为同一快照内的落点和机身轨迹。
* 在 `TerrainMotionPlan::valid()`（`:378-432`）增加时序单调性、窗口覆盖、至少三接触（本设计选择的 v2 模式）、每个 touchdown 对应有效 foothold、period/duty 边界检查。`BuildTerrainPlanHorizonIndices`（`:434-479`）和 `TerrainPlanStore`（`:481-508`）继续提供时间重采样与 atomic latest-valid snapshot，不做局部字段更新。

`TerrainFeasibility` 仍只负责 lidar 派生的安全区域、edge/slope/roughness/IK/swept-clearance；planner 复用现有候选和 `CheckSwingClearance` 语义，不重新引入脚本目标。现有 `terrain_planner.h` 包含的 `terrain_crawl_script.h` 中测量 edge、contact-patch/foot-site endpoint 和 standoff 的纯几何 helper 可以复用；`MeasureTerrainScriptTarget` 只能作为传感器候选/回退几何，不得恢复“按腿序取 target”的 policy。

### 1.2 gait scheduler/执行适配器：消费计划，不拥有 terrain policy

目前 gait 相位与接触规则在 `example/cpp/gait/locomotion_kernel.h:101-155`（`GaitLegPhase`、`GaitLegScheduledStance`），参数和接口在 `:157-224`；`HandCodedTrotKernel::Compute` 在 `:277-355` 按相位生成连续足端及 touchdown endpoint。它没有逐腿时间表输入，正是需要新增“计划执行请求”而非继续扩展环境分支的 seam。

最小接口为 `GaitExecutionRequest`：基准 pattern、连续 `phase_origin`、period/duty、逐腿 touchdown time/stance interval、plan_id/epoch、有效期；`Compute` 在一个执行边界一次接受完整请求并保持 phase continuity。若请求无效，保留上一份请求直到 valid-until，然后回 Phase-1 schedule；绝不能在 swing 中途重写腿的起点或 endpoint。`GaitKernelRequest/Result`（`:171-205`）增加 provenance 与实际采用时序，便于日志对账。

现有 `TrotExperiment::UpdateRuntimeVelocityCommand` 在 `example/cpp/trot/trot_experiment_gait.cpp:115-533` 同时整合 command shaper、`ScheduleTerrainCrawl`、`SetGaitPattern/Period/Duty/StepLength/FootLift`（`:508-520`）。这里改为单一 `TerrainPlanExecutionAdapter` 的未来 seam：从 `TerrainMotionPlan` 读取 timing/body/foothold，向 gait kernel 提交完整请求；planner 不得直接调用这些 setter。`BuildGaitTargets` 在 `:534-620` 目前先造 neutral feet 再按 crawl 输出覆盖，未来只在适配器内用计划的有效 swing path 覆盖，并保留普通 Phase-1 路径。

`TerrainCrawlSequencer` 的状态与顺序（`example/cpp/terrain/terrain_crawl_sequencer.h:331-629`，尤其 `:472-581`）不再驱动正常地形策略。它的 `TerrainCrawlSequencerWbcContactOverride`（`:17-39`）、`TerrainStanceForceHandoffReference`（`:91-121`）、endpoint/leading-edge、测量 force/contact/COM 字段（`:59-187`）可作为安全/诊断 helper；`target_`、`kLegacyFrontFirstLegOrder`、STAGE/SHIFT/SWING/COMMIT 状态和固定 `ShouldAdvanceBodyAfterCommit`（`:651-667`）只能在显式 fallback 或故障注入路径使用。已有遥测字段应迁移到通用 plan/timing diagnostics，保持字段含义，不以 sequencer state 推断成功。

### 1.3 MPC：消费同一个逐 knot 接触/落点/机身参考

`example/cpp/wbc/srbd_mpc.h:46-74` 已有 `contact[k][leg]`、`foot_from_com_world_horizon[k][leg]`、逐 knot reference、`plan_id/epoch`；`ValidateSrbdFootHorizon`、`SrbdFootAt`（`:126-160`）已经禁止不完整的计划混搭。`SolveSrbdMpc`（`:199-375`）仍是 contact-force QP，所有 knot 的 `B` 使用相应脚位置（`:231-253`），因此 Stage C early 不重写 solver：把 planner 的 retimed contact/foothold/body reference 完整填入这些现有槽位。

`trot_experiment_wbc.cpp` 的当前接缝是 contact 与计划选择（`:279-355`），terrain plan 的时间对齐（`:873-901`），计划 horizon 填充（`:1128-1300`），以及 `SolveSrbdMpc` 调用和 first acceleration 输出（`:1353-1370`）。这些代码目前还在 sequencer active 时强制覆盖整个 horizon（`:1302-1328`）；未来仅允许在“plan 无效/安全回退”时走该分支。正常路径必须保证：同一个 `plan_id/map_epoch` 的 contact mask、foothold、body reference 和 timing 一起进入 MPC；计划在新 touchdown 后只能以新 snapshot 替换，不能在 solve 中途变更。

### 1.4 ID-WBC：执行和测量安全，不重新做 terrain policy

`example/cpp/wbc/inverse_dynamics_wbc.h:49-73` 的 `IdWbcInput` 已分开 `measured_contact`、`planned_contact`、实际 `contact`，带 terrain identity 和 support normal；`SolveInverseDynamicsWbc`（`:127-444`）以浮基动力学、摩擦锥、swing/stance acceleration、torque inequality 为执行约束。Stage C 只增加 plan timing/epoch 的 provenance 和 contact-fusion diagnostics，保留 `last_srbd_.first_linear_acc/first_angular_acc` 到 ID-WBC 的连接（`trot_experiment_wbc.cpp:1371-1415`）。ID-WBC 不选择腿序、落点或速度，不因 planner 请求而放宽 torque/posture/contact gate。

### 1.5 snapshot/worker/诊断

`PublishTerrainControlSnapshot` 在 `example/cpp/trot/trot_experiment_control.cpp:135-297` 每 20 ms 收集 state/map/contact/COM；`UpdateTerrainRuntime` 与 worker 在 `:299-548`、`:799-873` 负责 Build、统计和 publish。未来只需把 timing bounds、计划 adopted/rejected reason、planner latency、actual-vs-planned touchdown 和 fallback reason 贯穿这里；仍用 `TerrainPlanStore::Publish`，不改变控制线程阻塞约束。diagnostics 的已有 plan 字段在 `trot_experiment_diagnostics.cpp:47-74, 823-858, 905-1166` 可扩展为 timing/contact/body 摘要。

## 2. 在线接触时序、落点、机身轨迹的具体形式

### 2.1 初始模式：v2-B quasi-static planner schedule

本设计首先采用合约 V2-B 的准静态模式：transfer window 内任何时刻保持至少 3 个**实测/确认**接触，planner 可以选择和延后 touchdown，但不允许 2-contact 或 aerial interval。这样不需要 v3 合约才能开始，也不把“planned contact=loaded force”混为一谈。V2-A 的速度仍由现有 Phase-1 shaper 独占：planner 只发布 `TerrainVelocityRequest`，窗口内可请求 `[0.05, ...] m/s` 的 cap/target，不能完全刹停超过 1.0 s；窗口外恢复原 profile。

“至少 3 个”是硬 feasibility 条件，不是把 crawl 的四步顺序搬进 planner。每次更新从当前测量支持和传感器安全区域生成候选接触事件，按可行性选择下一事件；不得按 `kLegacyFrontFirstLegOrder` 或 scene step index 排序。初始可使用 rotary-crawl 的接触族作为 seed，但事件次序由候选 score 和 support/clearance 约束决定，且必须被记录为 plan data。

### 2.2 时序变量：逐腿 touchdown offset 为主、period/duty 为低维变量

不要把全局 phase shift 当作唯一优化变量：它只能整体搬移四腿，不能处理一条腿需要更晚落地、另一条腿需要提前确认的情况。也不要同时独立搜索 `phase_offset` 和 `touchdown_time`，否则同一自由度有两个权威。建议的第一版变量为：

```
q = { T, D, delta_t[FR], delta_t[FL], delta_t[RR], delta_t[RL],
      touchdown_time[i] for the next bounded events }
t_touch[i] = t_nominal[i] + delta_t[leg]
phase_offset[leg] = wrap((t_touch[i] - t_now) / T)  # derived telemetry only
```

其中 `T` 是本窗口共享 period、`D` 是 duty；每腿只优化视野内最近 1 个、最多 2 个 touchdown。起始搜索是确定性有限格点，而不是引入新在线黑箱求解器：`delta_t` 取以 planner knot dt 为步长的有限小集合，`T/D` 取当前 Phase-1 schedule 周围的有限集合；具体上下界在 C-000/B0 前冻结，未冻结前不能调到观测结果。作为工程初始候选，可评估不超过约 ±2--3 个 20 ms knot 的 per-leg offset、`D` 的高于三接触下限的窄范围和 `T` 的小幅范围；这些是实现搜索预算，不是验收阈值。

每个候选 schedule 都要通过以下硬检查：

1. 时间单调、每个 swing 有正的且 clearance 足够的持续时间；同一腿不能同时 stance/swing，不能跨越 `valid_until`。
2. transfer window 内每个时间 knot 的 planned contact count >=3，并把测量支持保留到确认的 touchdown；不得用一个 noisy off 样本删除最后 robust support。
3. 每一待落点来自 `SafeFootholdRegion`，满足 known/fresh、edge inset、slope/roughness、IK/reachability、swept-volume clearance、collision 和 uncertainty hard gate。
4. 以实测支持锚点与候选未来落点计算每个 transfer 的 support polygon/margin、法向力/摩擦/torque 可行性 proxy；整个 transition 的最小 uncertainty-inflated margin 必须为正。
5. 时序相对 nominal 的改动、period/duty 的变化和 v_cmd 偏差受预冻结 slew/bound 约束，不能通过瞬时变更破坏相位连续性。

可选候选按硬约束后排序：最小 support/clearance/edge uncertainty、body/CoM deviation、touchdown timing deviation、forward progress、foothold displacement、velocity-request deviation、solver cost。安全 margin 采用字典序优先于进度；无可行组合则发布 rejected/degraded 和已有 latest-valid/fallback，不发明一个替代 target。

### 2.3 机身/CoM 共规划

对每个“时序组合 + 每腿安全落点组合”，建立同一 horizon 的 body reference `b[k]`。初版不需要全身非线性求解：用当前 state/velocity 生成 base prediction，再对 `x,y,z,roll,pitch` 做小型约束投影或 bounded convex QP；已有 `SupportMargin2D`、`PredictBasePosition` 和 `body_reference[k]` 是直接 seam（`terrain_planner.h:246-319, 1401-1418, 1586-1612`）。投影变量允许 CoM 横向/纵向小移动、base height 和 terrain plane 的姿态参考，限制 base rate、roll/pitch、angular rate、height、jerk；不得用当前四脚均值代替 support polygon。

在每个 knot：

* 已确认接触使用 measured support anchor；尚未 touchdown 的腿不能贡献力，只保留 swing clearance 约束。
* 已计划 touchdown 的腿在 touchdown knot 后才成为预测力作用点；其后续 `foot_from_com_world_horizon` 使用该预测落点。
* body/CoM 必须在缩减后的 predicted support polygon 内，并留出 uncertainty buffer；若 transfer 中任何 knot 不能正 margin，组合被拒绝，优先 slow/hold/stop，而不是把机身 reference 推到可达但不稳定的位置。
* body reference 与 timing/foothold 一起写入同一 `TerrainMotionPlan`，由 `plan_id`、`map_epoch`、`valid_until` 绑定。

这把 Order-089 的“脚放在 lip/被机身拖向 lip”问题变成联合约束：落点不只满足 IK，机身轨迹也不能把已加载脚拖过 edge keep-away；未来可把 support edge distance 作为显式 signed constraint，但阈值要从 map/foot patch/uncertainty 冻结流程得到，不能采用观测拟合值。

### 2.4 执行、融合和复位

执行适配器在 touchdown boundary 采纳 plan timing；已开始的 swing/endpoint 是 immutable，更新只能影响尚未开始的事件。planned schedule 只给 gait/MPC 预测；`MeasuredContactState` 由 force/kinematic filter 给 ID-WBC 和安全逻辑。可在预冻结的传感器 latency grace 内保持最后 robust support或承认 early touchdown，但每次 promotion/demotion 必须写 plan_id、measured mask、reason。

窗口结束时，适配器撤销 terrain request，清除 terrain timing override，恢复 Phase-1 period/duty/pattern/velocity profile；不能调用 crawl `RESUME` 来掩盖计划状态。只有 stable passage 和 V2-A 的恢复时间条件都满足，才释放 window。planner stale/timeout 时保留 bounded latest-valid plan；超过 grace 进入 Phase-1 shaper 的 braking/support-safe fallback。`terrain_crawl_sequencer.h` 可作为显式 fallback backend，但 fallback 不是正常成功路径，也不能将其固定 leg order 写进 Stage C acceptance。

### 2.4a 实测支撑运行时门禁

这是执行期的硬门禁，不是 analyzer 软指标。每个控制 tick 重新计算 `measured_contact`：沿用现有 `HystereticContactParams` 和 `UpdateHystereticContact`（`example/cpp/contact/contact_state_filter.h:8-38`），当前运行参数为 engage `>=5 N`、release `<=3 N`（`example/cpp/trot/trot_types.h:55-56`，调用于 `trot_experiment_wbc.cpp:264-278`）。力值非有限或参数非法即判定观测无效并走回退。kinematic 判据不新造阈值：touchdown promotion 只能复用 `TerrainTouchdownTolerance`（`terrain_motion_plan.h:15-29`，窗口内下限 45 mm）及现有 endpoint error 检查（`trot_experiment_wbc.cpp:445-459`）；仅有位置接近、没有 filter contact bit，永远不能算支撑。

定义 `M(t)=count(measured_contact[t])`。当任一控制 tick `N` 检出 `M(N)<3`，在同一 tick 锁存 `support_guard`：禁止新的 swing、禁止 touchdown commit、禁止扩展/推进 contact schedule；已在途 swing 的目标与起点不再漂移，只允许执行适配器在下一 tick 以内（`N+1`，当前 500 Hz 环境即不超过约 2 ms）冻结其计划进度并进入 recovery。适配器职责是计划生命周期：按以下固定链执行，而不把计划接触伪装成测量接触：

1. `N`：保留当前 measured mask，停止新事件；向现有 v_cmd shaper 提交带 `plan_id` 的减速至零/安全 cap 请求。
2. `N+1`：尝试加载 `latest-valid plan` 的 bounded recovery segment；只允许保持当前已测支撑并把已有在途腿导向该计划中已验证的 touchdown，不能发明落点。期间最多保持一个预冻结的 filter latency grace；WBC 的实际 contact mask 仍是 measured/fused mask。
3. 到 `N+5`（不超过约 10 ms）仍 `M<3`，或 recovery segment 不存在/过期/不满足当前 measured mask：适配器放弃 terrain timing，切换 Phase-1 safe-stop request，保持 gait phase 不再推进。
4. 到 `N+25`（不超过约 50 ms）仍未恢复至少 3 个实测接触，或 posture/torque guard 先触发：调用既有 posture stop/emergency safety path；不以“计划上有三脚”解除门禁。

执行 `N+5` 的 Phase-1 safe-stop 前先将 terrain transfer 标记为 `transfer_abort`；因此 V2-A 的“完成或 transfer abort”边界已发生，后续完全停车是安全回退而非新的 terrain-crossing 行为。若 safety path 必须优先于窗口时限，则 safety envelope 胜过速度包络；该 episode 只能记为 abort/failure，不能声称满足 V2-A 的成功速度剖面。

上述 tick/时间是 proposed runtime contract，必须在 C-000 根据实测控制周期冻结；不得用更宽 grace 掩盖支撑丢失。若 `M` 在 N+5 前恢复到 >=3，仍需 endpoint/force filter 原有 promotion 条件，才可恢复最新计划；一次恢复不追溯地把缺失 tick 记为计划接触。`latest-valid` 是恢复来源，不是 WBC 支撑事实。

ID-WBC 职责相反且明确：它接收适配器给出的实际 `contact`，并以 `measured_contact` 做安全约束；执行已有摩擦锥、support normal、stance/swing acceleration、torque limit 和 solver validity（`example/cpp/wbc/inverse_dynamics_wbc.h:49-73, 127-161, 260-357`），在 guard 期间不得因 `planned_contact` 增位或 promotion。ID-WBC 不选择 planner fallback、腿序或 touchdown target；若输入无效/扭矩或姿态保护失败，返回 invalid 让适配器沿 Phase-1 safe-stop → posture stop 链处理。强制条款：**planned contact 永远不得替代 measured contact 进入 WBC 安全决策**；它只能作为 SRBD 预测和执行适配器的候选时间表。

### 2.5 合约合法性及未生效 v3 草案

本设计首次实现只声称当前 v2 的 V2-B 准静态模式；窗口内速度仍按 V2-A 通过既有 shaper 仲裁。下表是行为主张到合约的逐条对账，不把设计提议误写成已通过的 B1-B3 证据。

| 本设计行为主张 | v2 合约条款 | 原文摘要/执行解释 |
|---|---|---|
| 窗口内速度由 planner 请求受控 cap，最低 0.05 m/s，结束后恢复 | `修订条款 V2-A`（第 24--31 行） | “接近段仍 0.30m/s；转移窗口内允许受控减速至爬行速度（不低于 0.05m/s，不允许完全刹停超过 1.0s）；穿越完成后 1.0s 内恢复脚本速度”；窗口外保留 v1 的 ±0.020 m/s、禁刹车、单速度权威。planner 不能直接写 gait setter 或 shaper 输出。 |
| planner-owned quasi-static schedule 在任意时刻至少 3 个实测接触 | `修订条款 V2-B`（第 33--36 行） | “转移窗口内允许从 running-trot 切换为准静态 crawl（任意时刻 ≥3 接触）”；本设计把“接触”定义为现有 filter 的 measured bit，不是 planned mask。 |
| planned/measured 分离、实测力才可确认 touchdown | `修订条款 V2-B`（第 33--36 行）及 `不变的底线`（第 38--43 行） | “窗口内的 contact 一致性、planned/measured 转移一致、零碰撞、每腿实测力支撑触地等全部 v1 条款不变”；lidar-only、零碰撞、完成性、0.45s stable passage 和 manifest 也原样继承。 |
| stale/timeout 用 latest-valid → Phase-1 safe-stop → posture stop | `V2-A` 的 “transfer abort” 与 `V2-B` 的窗口外恢复；`不变的底线` | 合约明确 abort 和恢复冻结 Phase1 profile；本设计把 `PHASE2_TERRAIN_PLAN.md` §8--9 的 latest-valid/fallback 具体化，且不把 fallback 计为穿越成功或放宽任何 v1 gate。 |
| 少于 3 接触的 dynamic timing adaptation | **不属于当前 v2；本文件 V3-C 草案** | 只能作为下方未生效提案，不能用于当前 B1-B3 验收。 |

对账基线：`docs/research/PHASE2_B123_ACCEPTANCE_CONTRACT_V2.md` 最后修改 commit 为 `d0d6252821381fce53060159570ed1a03b3d1ff0`（`git log -1 -- docs/research/PHASE2_B123_ACCEPTANCE_CONTRACT_V2.md`）；当前文件 blob hash 为 `34a7e74b136f3b0b100f2ca1f7405499da7b8e38`（`git hash-object`）。后续验收应记录该 commit，并在合约变更时重新生成对账 hash。

以下仅为若未来要接受少于 3 接触的 dynamic-with-timing-adaptation 而起草，**未生效、未应用、不能用于当前 B1-B3 验收**：

> V3-C（DRAFT—未生效）：在 V2-A transfer window 内，允许 planner 从冻结的候选 hybrid contact schedules 中选择短暂 2-contact interval；每个 interval 必须有预注册 duration 上限、正的 full-body force/torque/support feasibility margin、planned/measured contact fusion witness、zero collision 和 failure-safe braking。不得接受 aerial interval；窗口外恢复 v1 topology/profile。该条款只有在人类 owner 明确批准、形成 hash 后才可进入验收合约。

## 3. SRBD 不足到 whole-body NMPC 的升级判据

### 3.1 不因一次失败升级

升级决定必须在 B0 flat planner-enabled regression 通过、TerrainModel/Feasibility 传感器语义通过、Stage C 的 timing/foothold/body plan 可复现且 SRBD 已确实消费逐 knot foothold/contact 后做。先排除：map frame/age、safe-region 错误、planned/measured 混淆、contact filter latency、gait handoff、ID-WBC torque/stance task 和 solver deadline。否则“SRBD 不够”只是错误接口的归因。

建议验收模式预注册 holdout 样本量、seed、场景生成器和 analyzer hash；至少做一组固定样本的 SRBD 对照（旧 flat foothold、time-indexed foothold、timing+body plan），抖动门按 `PHASE2_WORKFLOW.md` 预注册 n 并给 Wilson 区间，不能由一次通过/失败下结论。每行证据至少包括：run/plan/map id、schedule/touchdown、每 knot contact mask、foothold/CoM、support margin、SRBD status/iterations/cost/force residual、ID-WBC status/eq/RNE residual、friction/torque activity、measured contact、failure stage、latency/deadline 和 fallback reason。
### 3.2 数值化的 B0 与 SRBD 升级分界（proposed）

下列数字是后续验收前的 proposed freeze，不是本单结果，也不能调到 holdout 结果。B0 的“绿”定义为：在 sensor-only/no-actuation、clean source 下，选定的 Phase-1 profile 与已接受 baseline 的完整量化合约逐门相同，并同时满足以下既有门（来源：`PHASE2_TERRAIN_PLAN.md §11.2` 及其引用的 Phase-1 analyzer/contract）：

* completion/lifecycle/status：完成、controller/safety/quality/dynamics status 合法，zero unplanned status failure；terrain 仅有 telemetry，不得改变 foothold、body reference、gait topology、event response 或 WBC task gate。
* posture：所选 arbitrary-velocity profile 的 roll/pitch P95 `<=4 deg`、全运行最大绝对 roll/pitch `<=15 deg`；若运行 fixed-3-m/s profile，仍使用其冻结的 `5 deg` P95 analyzer gate，但 Phase-2 common ceiling `4 deg` 更严格时取 4 deg；活动 execution mode 更低的 hard limit 优先。
* velocity/shaper：所选 profile 的 exact tracking/steady-state/overshoot/undershoot/settling row 不变；shaped-to-measured P95 `<=0.45 m/s`、shaper acceleration `<=1.25 m/s²`、jerk `<=4.20 m/s³`、acceleration-step change `<=0.02 m/s³`；需要停步的 profile stop-tail P95 `<=0.05 m/s`。terrain request 必须为 no-actuation/no-cap。
* support/foothold：contact-loss fraction `<=0.25`、single-contact fraction `<=0.45`、touchdown x `<=0.18 m`、y `<=0.07 m`、slip evidence `==0`；B0 不新增 terrain touchdown gate。计划/脚步 validity 与 baseline 一致。
* actuation/model/runtime：unchanged `--tau-limit` 下 torque-saturation fraction `<=0.003`；solver、SRBD、ID-WBC、footstep-plan validity `==1.0`；solver-budget fraction `>=0.80`；minimum base height `>=0.28 m`。fixed-3-m/s profile 另保留其冻结的 base-height percentile `[0.33,0.40] m` 和 foot-clearance `>=0.08 m` 条款。
* identity/separation：terrain flag、lidar source、event response、runtime v_cmd、kernel target、WBC target、MPC input、ID-WBC output 必须逐字段与 baseline 分离；manifest 写明 HEAD、contract/analyzer hash 和 source provenance。ctest 全绿只是必要条件，不替代这些门。

“SRBD 足够”与“允许升级”的统计分界建议固定为同一组 `n=20` holdout episode、双侧 95% Wilson 区间、全数纳入分母（无效运行不得删除）：

| 结论 | 数值门 | 含义 |
|---|---|---|
| SRBD sufficient | 20/20 episode 全部通过继承门、Stage-C 几何/时序/支撑门和完成性；pass-rate Wilson 95% 下限 `>=0.80` | `20/20` 的 Wilson 下限约 `0.839`，故可声称在 proposed 0.80 置信下 SRBD+Stage-C 足够；任一未分类失败都不能声称 sufficient。 |
| 证据不足、继续诊断 | 0--4 个可归因 SRBD failures，或有任意 map/fusion/执行器/unclassified failure | 例如 `4/20` failure 的 Wilson 下限约 `0.081<0.10`，不能升级；先修接口/归因。 |
| SRBD insufficient、可提交升级评审 | 至少 `5/20`（failure rate `>=0.25`），且每个均满足下述 SRBD 归因条件；failure-rate Wilson 95% 下限 `>=0.10` | `5/20` 的下限约 `0.112`；这只是“可提交 reviewer 决策”，不是自动批准 NMPC。 |

SRBD 归因必须可从日志独立判定，而不是从“穿越失败”倒推：

1. 输入与计划：`TerrainModel` 为 lidar-only、frame/age/epoch 有效；无 truth/XML/oracle；accepted `plan_id/map_epoch` 的 timing、foothold、body horizon 完整且每个硬几何/uncertainty/support margin 门通过；planned contact count 和 measured contact 明确分列。
2. 计划消费：`BuildTerrainPlanHorizonIndices` 成功，`mpc_in.has_time_indexed_footholds/reference==true`，每个 planned contact 有对应 valid foothold，`terrain_plan_consumed==true`；不得是 planner reject、stale fallback、缺 knot 或 sequencer override。
3. 求解/执行：每个失败 episode 的 SRBD QP `ok==true`、ID-WBC `ok==true`、plan/contact coherent==true，solver-budget `>=0.80`；无 timeout/deadline、NaN、摩擦锥/normal-force violation、torque saturation（继承 `<=0.003`）、slip 或 `<3` measured-contact runtime-gate failure。ID-WBC 的 eq/RNE residual、force、friction ratio、q/dq、torque 和 base error 必须带同一时间戳。
4. 现象归因：失败必须重复落在可观测的 SRBD 缺失项（例如逐 knot contact wrench/foothold 变化与 full-body joint-limit/velocity coupling、swing-foot reaction 或姿态动力学不一致），且 measured support、touchdown、collision、posture/velocity safety gate 本身没有先失败。使用明确的 `failure_class=srbd_model_gap`；其他类别为 planner/map/contact_fusion/id_wbc/actuator/unclassified。
5. 反事实对照：同一 `plan_id/map_epoch`、state、timing、foothold、seed 的旧 flat、time-indexed foothold、timed+body 三组结果及离线 full-body rollout 都入 manifest；只有在 timing/foothold/body 和 ID-WBC 修复对照仍不能消除该类失败，而约束相同的离线 full-body 解可行时，才算 SRBD model gap。harness 可读取 truth 做评分，controller 不可读取。

输出为 machine-readable `srbd_escalation_evidence.json`、CSV 和 manifest：记录每 episode 的 pass/failure、Wilson n/k/bounds、上述五类判据、残差、plan identity、配置和对照。未同时满足 `5/20 + Wilson 下限 + 全部归因判据`，只能继续 Stage C/SRBD，不启动 NMPC；满足也必须经过 reviewer 批准。

### 3.3 未来 whole-body NMPC 范围草案

这不是本单实现。触发后第一阶段仍保持现有计划接口、safe fallback 和 ID-WBC；NMPC 只在 bounded horizon 产生下一 control interval，不直接绕过 safety。

* horizon：初始 10--20 knot、20--40 ms dt（约 0.4--0.8 s），只覆盖下一个/两个 support exchange；以实际测得的控制周期冻结，不用论文频率假设。
* state：浮基 pose/velocity、12 个关节 q/dq，必要时接触状态/摩擦参数；control 为 joint torque 或 joint acceleration + contact wrench，保持与 `RigidBodyDynamics`、`IdWbcInput` 的可验证映射。
* constraints：全刚体动力学、joint/velocity/torque limits、contact unilateral/friction/wrench bounds、planned/measured contact consistency、safe foothold region/normal/edge/roughness、swing swept-volume、CoM/support margin、body posture/rate、plan validity/deadline。离散 contact sequence 先由 Stage C 的有限候选枚举，避免第一版把 hybrid integer solver 引进高频环。
* cost：tracking body/CoM/velocity、foothold and timing displacement、wrench regularization、contact transition、uncertainty/risk、torque/smoothness；hard constraints 优先，不以 cost 代替碰撞/支持硬门。
* solver：现有 `DenseQp` 只能做线性/二次子问题，不能冒充 NMPC。优先做离线/低频 SQP/RTI 原型，用代码生成的 full-body dynamics 和 HPIPM/acados 类 QP backend；若新增依赖未获批准，则先实现可审计的 Gauss-Newton/SQP + 现有 DenseQp local subproblem，且只在测得预算和 fallback 后考虑 runtime。任何 solver failure 都回 latest-valid SRBD/Phase-1 safe path。

## 4. 有序实施订单（每单含验收、探针、回滚）

以下是建议后续订单，不是本 Order-091 的执行清单。每单在验收模式声称 gate 前都必须先 commit，记录精确 SHA；探索模式可跑 ctest/离线 replay，但不得把结果写成 acceptance claim。

### C-000：冻结接口、时序合约和 provenance（最小可运行改动）

*改动*：只新增/扩展 value types、plan identity、timing bounds、manifest/CSV 字段和 validator；默认关闭 terrain actuation，不能改变平地输出。

*验收/探针*：编译与 ctest 全绿；构造 flat/unknown/stale/invalid-frame、非单调 touchdown、缺 foothold、少于 3 contact 的单元测试；运行一次 sensor-only B0 smoke，确认 command/event/WBC/MPC 旧字段不变。

*回滚*：删除新增字段/validator 或禁用 `has_stage_c_timing`，保留现有 `TerrainMotionPlan` 旧 fallback；不得回滚合约 hash 以外的 Phase-1 代码。

### C-001：原子 timed plan 与 schedule validator

*改动*：在 `TerrainMotionPlan` 中加入绝对 touchdown/period/duty、每 knot contact、timing provenance；在 `TerrainPlanStore` 保持 atomic whole-snapshot；禁止 planner partial mutable writes。

*验收/探针*：计划 round-trip、时间重采样、valid-until、plan/map epoch、旧 single-foot fallback 的单测；注入 planner timeout/rejection，证明旧 valid snapshot 在 grace 内保留且不会出现新 foot+旧 body 混合。

*回滚*：adapter 只读取 legacy contact/foothold path；新 timed fields ignored，store/legacy B0 输出保持不变。

### C-002：shadow contact-timing + body/CoM planner（不执行）

*改动*：在 `TerrainPlanner::Build` 外围加入有限格点 timing candidate 和 body projection；使用现有 safe-region、standoff、clearance、support helpers，输出 `shadow_plan`，不 publish 给 actuator。

*验收/探针*：确定性 replay/单元测试覆盖 per-leg offset、period/duty、>=3 contact、support margin、未知/陈旧 map、candidate tie-break；同一输入/seed 必须相同 plan hash；planner p95/硬 deadline 统计但不假定已达标。

*回滚*：shadow flag off；继续使用现有 Stage-B plan 或 latest-valid plan。任何 shadow candidate 不得改 command、gait pattern、motion reference。

### C-003：gait execution adapter（只开 v2 window）

*改动*：给 `LocomotionKernel` 增加一次性 `GaitExecutionRequest` 消费；在 `trot_experiment_gait.cpp` 以 plan timing 产生连续 swing/stance，保留 phase continuity；正常 terrain policy 移出 sequencer。window 外和 flat B0 完全走旧 scheduler。

*验收/探针*：kernel 单测验证 phase boundary、early/late touchdown、计划替换和 in-flight immutable；先做 staged harness 仅观察 handoff/contact mask，记录 no-gate probe，不声称跨越；确认 sequencer fallback 可显式打开。

*回滚*：`stage_c_execution=false`，恢复旧 `terrain_crawl_sequencer` backend；prepared/in-flight transaction 清理遵循旧逻辑，不删除 sequencer。
### C-004：SRBD horizon coherent consumption

*改动*：在 `trot_experiment_wbc.cpp` 填充 planner 的逐 knot contact、future foothold、body reference 和 timing identity；删除正常 Stage-C 路径对整段 horizon 的 sequencer 强制覆盖，但保留 invalid/fallback override；`SolveSrbdMpc` 算法和 first acceleration 输出不改。

*验收/探针*：SRBD 单测验证每个 touchdown knot 的 lever arm、contact 变更、旧 flat numerical equivalence、plan identity；注入缺 foot/expired plan，必须 reject whole snapshot，不混 legacy anchor；检查 MPC p95/hard deadline。

*回滚*：`has_time_indexed_footholds=false`，回到当前单 anchor/Phase-1 schedule；保留 diagnostics 以便定位，不切换 WBC solver。

### C-005：planned/measured contact fusion 与 ID-WBC provenance

*改动*：统一 contact-state filter 的 latency/hysteresis/grace；计划接触只进 prediction，measured contact 决定 WBC safety；把 fusion transition、wrench/torque residual、plan/timing id 写入 diagnostics。

*验收/探针*：force dropout、early touchdown、late touchdown、单样本 noisy-off、三接触丢失注入；验证不移除最后 robust support、不将 planned touchdown 计为 force、不超过 bounded grace；ID-WBC 原有 torque/posture gates 100% 保留。

*回滚*：回到现有 `MergeHighSpeedContact`/legacy measured schedule；timed plan 可继续 shadow，但不能驱动接触 mask。

### C-006：B0 flat planner-enabled regression gate

*改动*：无新策略，只冻结 C-000..005 的配置、contract/analyzer/scene/hash，运行 sensor-only/no-actuation flat profile。

*验收/探针*：按 `PHASE2_WORKFLOW.md` 验收模式，先提交 SHA，再按既有 Phase-1 全量 quantitative contract 检查 velocity/event/WBC/MPC/ID-WBC 分离、planner runtime、source provenance；ctest 绿不等于 B0 通过。

*回滚*：若任一旧门变化，默认关闭 Stage C 接口并修复接口；不调 analyzer 阈值，不保留“部分通过”作为启用理由。

### C-007：B1 5 cm timing/foothold/body development probe

*改动*：仅在 B0 通过后使 sensor-derived timed plan 在 v2 window 生效；禁用正常 crawl sequencer policy，sequencer 仅 fallback。

*验收/探针*：开发集先分 approach、first touchdown、support exchange、exit；serial harness、固定 config/seed、manifest 记录 plan replacement。检查 planned/measured contact、support margin、touchdown x/y/z 定义、clearance/collision、torque/SRBD/ID-WBC、planner deadline；不以 kinematic candidate 通过作结论。

*回滚*：任一 safety/model/contract gate 失败则关闭 Stage-C execution，保留 B0 和 shadow planner；不得退回低姿态 crawl 作为新主线。

### C-008：B1 holdout 与 SRBD sufficiency evidence

*改动*：无阈值调优；冻结 holdout manifest，运行预注册 n 和 Wilson 统计，形成 `srbd_escalation_evidence.json`。

*验收/探针*：同一 plan schema 下比较 legacy fixed schedule、timed foothold、timed+body plan；若失败，先分类 planner/map/fusion/SRBD/ID-WBC/actuator；只有满足 §3 全部条件才标记 SRBD insufficient。

*回滚*：保留最稳定的已验收配置和 latest-valid fallback；没有完整证据就不引入 NMPC 依赖。

### C-009：B2/B3 holdout、故障注入与升级决策

*改动*：混合/重复 steps、unknown/stale/contact lag/planner deadline injection；仅使用传感器 TerrainModel。

*验收/探针*：冻结 initial x/y、gait phase、step position/height/width/slope/roughness、seed、scene/analyzer/contract hash；验证跨 map epoch replacement、safe-stop 和窗口恢复。通过后才由 reviewer 决定保持 SRBD 或批准 whole-body NMPC prototype。

*回滚*：逐场景禁用新 planner execution、保留 B0；故障注入必须仍能进入 documented safe path。

### C-010：仅经批准的 NMPC prototype（非本单）

*改动*：离线或低频 shadow SQP/RTI，范围严格按 §3.3；不替换 ID-WBC，不新增隐式 terrain policy。

*验收/探针*：用 C-008/C-009 的同一 manifest 做 paired comparison，报告 full-body residual、solver budget、warm-start sensitivity、fallback；没有 runtime deadline/failure evidence 不上控制线程。

*回滚*：删除 NMPC backend/关闭 flag，保留 validated SRBD plan 和 ID-WBC。

## 5. 按 PHASE2_WORKFLOW 的验证计划

### 5.1 设计单本身

Order-091 只做文档；禁止仿真、禁止行为代码变更、禁止 gate claim。验证仅为：指定实现审计完成、文档 diff check、working tree 无行为代码 diff。`ctest` 在本单不运行仿真；若运行构建测试，仅能报告为未改变源码的基线检查，不能替代后续 B0。

### 5.2 探索模式阶段

C-000..005 可在探索模式做 ctest、纯函数单测和既有 CSV/log replay。允许批量分析既有数据，但 controller/planner 不得看到 harness truth。每个工作状态提交 checkpoint，`git status` 和 `git diff --check` 记录清楚。任何新的阈值先是 proposed，必须在 B0/B1 前冻结并 hash。

### 5.3 验收模式阶段

C-006 及以后凡声称修复 gate，按 `docs/research/PHASE2_WORKFLOW.md`：一次验证一个假设；抖动门预注册样本量并给 Wilson 区间，稳定门双采样；canary 前工作树必须 commit，证据引用精确 SHA；reviewer 通过后才能写 handoff/ESCALATION_LOG。所有 serial simulation 使用既有锁、domain/preload/clean checkout 纪律；本设计单不启动任何 simulation。

每个 B0-B3 run 必须有 manifest、data.csv、planner diagnostics、analyzer output、machine-readable status，至少包含 completion/lifecycle、state/posture、foothold geometry、support/contact、actuation/model、planner runtime 六组指标。继承 Phase-1 的 exact gates（P95 roll/pitch、max posture、velocity/shaper、contact loss/single contact、touchdown x/y、torque saturation、base height、solver/SRBD/ID-WBC validity、stop tail 等）不重调；terrain-only edge/slope/roughness/support/clearance margin 在 map resolution、patch geometry 和 uncertainty freeze 后注册。

### 5.4 B0、B1、B2、B3 证据顺序

1. B0：terrain sensor/planner plumbing 开启但 no-actuation；确认没有 event/velocity/WBC/task-gate coupling，且精确 Phase-1 contract 通过。
2. B1：5 cm 开发集拆分 approach、触地、support exchange、exit；再锁 holdout，检查 timed plan 的几何与动态 support，不把 sequencer 的序列完成当作新 policy 证据。
3. B2：10 cm 使用同一接口，要求每 knot 的 future foothold 和 support margin 证据；单边转移若无正 margin 必须 slow/hold/stop。
4. B3：mixed/repeated rises/falls、lateral offsets、unknown/stale patches、随机初始 x/phase/obstacle/seed；要求多 map epoch replacement、failure injection、no scene memorization。
5. 只有 B0-B3 完整 holdout 证据齐全，才可使用 §3 的 SRBD escalation format；whole-body NMPC、learning 和 sim-to-real 不是本阶段 DONE 前提。

## Reviewer-ready summary

Order-089/090 的证据已判死“固定 leg order + endpoint event + 低姿态 crawl”路线：48 次全流程和低姿态 staged stop 均无完整序列。Stage C early 不再让 sequencer 选择地形行为，而是在 `TerrainPlanner` 内对有限的逐腿 touchdown 时间、period/duty、safe foothold 组合和受约束 body/CoM trajectory 做一个原子短时域计划。现有 `TerrainMotionPlan`、`TerrainPlanStore`、逐 knot `srbd_mpc.h` 输入和 `IdWbcInput` 已足够承载最小迁移；gait kernel 只需新增 plan execution adapter，保留相位连续与 Phase-1 fallback。首次实现严格采用 V2-B 的 planner-owned quasi-static 模式（每时刻 >=3 实测确认接触），V2-A 速度仍由 command shaper 负责；少于三接触的 dynamic 方案只在附录 V3-C 草案中出现，明确“未生效”。

实施从 C-000 的 schema/validator/telemetry 和 shadow planner 开始，经过 atomic timed plan、gait adapter、SRBD horizon、contact fusion，再做 B0、B1、B2/B3；每单都有 unit/replay/serial probe 和 feature-off rollback。必须先证明 map/geometry/timing/fusion/ID-WBC 均正确且 SRBD 已消费逐 knot 计划，仍有可重复的 full-body coupling 失败，才允许 reviewer 决定 NMPC。NMPC 仅草拟为 0.4--0.8 s、10--20 knot 的 full-body SQP/RTI shadow，DenseQp 只能作局部 QP，不能冒充 NMPC；所有 solver failure 都回 latest-valid SRBD/Phase-1 safe path。本单无代码行为改动、无仿真、无验收门结论。
