# Go2 地形路线 13h 停滞调查与破局方案（2026-08-22）

仓库：`/home/che/dev/go2-mujoco-control-terrain`，分支 `terrain/adaptation-2026-08-21`。
WIP 已封存：`git stash` 条目 `wip-13h-stair-thrashing-2026-08-22`（含未跟踪文件），工作区已回到 ce46ef4 干净状态。

## 一、13 小时实际留下了什么

- 3022 行未提交改动，散在 18 个文件；核心在 `trot_experiment_gait.cpp`（+749）、`trot_experiment_wbc.cpp`（+244）、`raibert_trot_kernel.h`（重写 +479 行新版本）、bridge/main.cc 各 +85。
- 13 个 barrier 场景 XML 变体（0.02/0.04/0.05/0.10、narrow/close/far/wide…）——典型的无假设调参扫射。
- 20 个实验 run（v277–v296），命名轨迹本身就讲了故事：terrain_v* → stair_rear_hold_v*（9 次）→ stair_plan_diag → stair_map_diag。一直在加 hold、加诊断，没有一次通过验收。
- 没有任何提交。实验窗口只有 4 秒，而 WIP 自己加的 mode-entry hold 就耗掉约 4 秒——最后一个 run（v296）里 gait 相位被冻结在 0.817/0.567/0.317/0.067，整场仿真一步没迈出去就结束了。

## 二、根因诊断（三层）

1. **规划器没问题**。`terrain_adaptation.h` 是已提交、27/27 测试通过的原型；v293 日志显示它在仿真里也实际输出了 4 条腿全部有效的楼梯目标（前足 z=-0.28 踏上首级踏面），并成功 plant 了 leg=0（世界 z=0.124）。失败不在感知也不在规划。
2. **失败在集成架构**。agent 试图在周期性 raibert trot kernel 内部长出一个准静态爬楼梯状态机：terrain_mode/pending、mode-entry hold、front_pair_planted 追踪、rear_hold、swing_hold_targets、bypass……每一层 hold 都冻结相位图来保护支撑，层数越加越多，最终整个相位图死锁。对角 trot 的相位机天然不适合"一次一腿、恒三足支撑"的楼梯动作，这是路线级错配，不是增益问题。
3. **顺序错了**。文档自己写的顺序是"先单隔离带，再四级楼梯"；agent 直接上楼梯（动态爬楼梯是四足经典控制的硬问题），而且把已提交场景的真楼梯（riser 0.2 m，注释却写 0.1）偷偷改成 0.1 riser 来迁就控制器——验收场景被改动，即使通过也不算数。

## 三、破局方案

**核心原则：不动 kernel 相位图，接缝选在它输出之后。**

已验证存在的最小接缝：kernel 产出 leg states → experiment 层算出 `commanded_world_feet_` → WBC swing PD（`trot_experiment_wbc.cpp` L447-505）。地形逻辑只需在这一步覆写摆动腿的 touchdown 目标（x/y/z），不碰 kernel 内部。

具体分四级，每级一个可判死活的验收门，不过不进下一级：

1. **平地回归**：地形管线接线但输出必须与主线一致，27/27 测试 + 平地 trot 回归全绿。防回归地基。
2. **单隔离带**（0.15 m 高台面，场景已有）：前足踏上台面 + 机身越过即成功。用准静态 crawl 模式——独立的一次一腿步态，恒三足支撑，不塞进 trot kernel，而是作为 experiment 层的独立 reference 源（检测到 terrain 事件后减速、站稳、切换）。这正是文档里写的 fallback"静态安全步态"，也是 Qi et al. IROS 2021 爬楼梯的标准做法。
3. **一级台阶**，再四级楼梯。crawl 模式下楼梯退化为重复 N 次"上一步"，没有新机制。
4. 噪声/延迟扫掠只在经典路线全绿之后。

工程纪律：每次 run 固定 ≥12 s 窗口；一次只改一件事；验收场景锁死（把 ce46ef4 的 scene_stair_acceptance.xml 哈希写进验收脚本，改动即失败）；每个 run 的 run_metadata 已存的 git_head/哈希继续沿用。

**降级备选**：如果 crawl 模式楼梯仍超预算，可交付目标降级为"隔离带跨越 + 楼梯识别后 fail-closed 停步/绕行"——这在文档结论里是合法的（"无可行候选时减速、站稳或回退"），是有解释力的成果，不算烂尾。

## 四、可立即执行的第一步

从 stash 里只挑两样东西回收：`unitree_sdk2_bridge.h` 的高度图 footprint 修正（box 用半边长而不是对角半径，这是真 bug 修复）和 5 cm 网格参数。其余 3000 行状态机代码全部不要。
