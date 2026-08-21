# 地形适应路线决策（2026-08-21）

## 决策

继续沿用当前 `locomotion kernel → SRBD-MPC → 18-DoF ID-WBC` 执行层，在它前面增加一个感知与落脚点规划层：

```text
模拟深度/高度图
  → 局部地形表示（高度、法向、可通行性、未知区）
  → 结构识别（隔离带/楼梯）与短视距落脚点候选
  → IK、台阶边缘、步高、坡度、支撑稳定性约束下的候选评分
  → 地形化 MotionReference（速度、步长、抬脚高、机身高度、俯仰）
  → 现有步态 kernel → MPC/WBC → 力矩
```

这条路线既保留当前已经反复验证的力控植物，又把“每条腿落在哪里、机身怎么跟地形走”从平地规则提升为显式规划问题。感知只提供地形事实，规划层输出连续参考；不为“上楼”“跨带”“左绕”分别编写一套关节动作，也不把未知地形直接交给 WBC 猜。

## 为什么不是马上换成端到端 RL 或全身 NMPC

当前 Isaac Lab RL 与 MuJoCo 主线是分离仓库，已有 sim-to-sim 和部署接口成本；RL 适合后续作为残差/策略候选，暂不适合作为老师要求的首个可解释验收主线。全身非线性 MPC 能提高复杂地形上模型一致性，但实现、实时性和参数验证成本更高，应在落脚点与参考层验证后再评估，而不是先重写整套植物。

Perceptive Locomotion 的路线是高度图、几何落脚约束、SDF/碰撞约束与 MPC/WBC 的分层组合，已验证坡面、间隙和踏石；见 [Grandia et al., 2022](https://arxiv.org/abs/2208.08373)。面向楼梯的工作先识别台阶几何，再按步态周期选择可达台阶和落脚点，并把机身俯仰参考插入 MPC；见 [Qi et al., IROS 2021](https://www.researchgate.net/publication/357098806_Perceptive_Autonomous_Stair_Climbing_for_Quadrupedal_Robots)。这与当前架构最匹配。

## 当前代码事实

- `raibert_trot_kernel` 和 `preview_footstep_horizon` 目前只规划平地 x/y 触地点；没有地形 z、支撑面法向或边缘安全约束。
- `MotionReference` 目前只有速度、步幅、占空比和抬脚高；楼梯所需的地面高度和机身俯仰尚未进入参考接口。
- `--wbc-full` 可以跟踪连续的速度/姿态/高度参考，但不会仅凭力控自动推断台阶落脚点。
- 当前自动障碍层是事件/侧移响应，不等于“跨越障碍或爬楼梯”的落脚点规划。

## 已完成的可复现实验

独立 worktree：`/home/che/dev/go2-mujoco-control-terrain`，基线 HEAD `97b6b0a0`。

新增 `example/cpp/terrain/terrain_adaptation.h`，以及：

- `test_terrain_adaptation`：平地、实体隔离带台面、未知区拒绝、四级楼梯连续落脚、参考渐变、3×3 步高/踏深参数扫掠、超出垂向可达范围的 fail-closed；全部通过。
- `test_terrain_scene_geometry`：加载 MuJoCo 场景，确认 `terrain_barrier` 和 `terrain_step_1..4` 是 worldbody 中真实 `box` 碰撞 geom，而非 marker；通过。
- 完整 CTest：27/27 通过。

专用场景：

- `unitree_robots/go2/scene_barrier_acceptance.xml`：0.15 m 高、0.28 m 深、0.75 m 宽实体隔离带。
- `unitree_robots/go2/scene_stair_acceptance.xml`：四级实体楼梯，踏深 0.24 m、每级 0.10 m、宽 0.75 m。

主线基线也已在两场景运行并留存于 `example/cpp/experiments/_runs/terrain_baseline_*`。结果不能冒充成功：当前主线未启用地形模式，运行中没有地形落脚点或机身姿态规划；普通参数的近场基线最大相对前进量只有约 0.24 m（隔离带）和 0.10 m（楼梯），加速探针在地形前即触发约 -45° 俯仰硬安全停机。因此，下一阶段必须先把“真正遇到实体地形”作为验收前置条件，不能只看控制器正常退出。

## 下一步实现顺序

1. 先接入模拟深度/高度图接口，明确时间戳、延迟、未知区和审计用 ground truth；规划器不能直接读 MuJoCo 真值。
2. 把 `TerrainFootholdOutput` 接到摆腿 touchdown 目标：候选必须通过支撑面 patch、边缘/坡度、步高和 Go2 IK；无可行候选时减速、站稳或回退，不盲目迈步。
3. 扩展 `MotionReference`：`ground_height`、`pitch`、`terrain_mode`；由统一 slew 层限速后同时供 gait kernel 与 MPC/WBC，保持连续过渡。
4. 先做单隔离带，再做四级楼梯，再做噪声/延迟/遮挡参数扫掠；每次同时记录落脚误差、碰撞次数、机身姿态、速度、支撑接触数、IK/求解器失败和恢复时间。
5. 只有经典路线稳定通过后，才比较 RL 残差或全身 NMPC；它们不能替代前述安全约束与验收指标。

## 结论

最值得继续的是“感知高度图 + 结构化/局部落脚点规划 + 地形化连续参考 + 现有 WBC/MPC”。它能直接回答隔离带和楼梯问题，解释每次落脚为何可行，并把后续 RL 留在可替换的上层，而不是让训练结果决定底层安全边界。
