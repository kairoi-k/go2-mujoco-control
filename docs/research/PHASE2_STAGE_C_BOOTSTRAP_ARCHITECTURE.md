# Phase 2 Stage C Bootstrap 架构：双证书 C0/C1

状态：**DESIGN ONLY / NOT AUTHORIZED**（Order-115，2026-09-01）。本文件只定义架构、状态机、安全不变量与 proof obligations，不改变行为代码，不下达实现订单，不启动仿真，不声称任何 B1-B3 门通过，也不包含仿真结果。基线 HEAD：`3eacac1`（Order-114 E1 冻结后的 4ed0157 表面）。

## 0. 决策背景与 C006 事实

- **正式 C006 事实（order109b / `5b95e8265c885a81f8488e4930e682aa55f05674`）**：canary + 3/3 holdout 独立固定对全部 PASS；生命周期、fixed 3 m/s、lockstep 协议、authoritative B0、terrain sensor-only、planner deadline、no publish/consumer/actuation、frozen 非回归检查全部通过；`ctest` example/cpp 31/31、simulate 2/2 PASS；证据见 `docs/research/evidence/order109b_c006i/`（docs-only，运行数据在忽略的 experiment workspace）。这是 Stage-C 可引用的唯一正式认证基线。
- **早期 wall-clock 失败仅为残余风险**：`e65e155`/`3273bd5` 的 B0 失败归类为 controller 内部时钟与 sim 时间不同步（cmd_time ~1.4--2× sim time、wall_clock_motion override），属于 Phase-1 控制律/接触鲁棒性残留，不是对本架构的否决；不构成"能跨过已认证局部观察运动"的否定证据。
- **Order-112 INCONCLUSIVE**（diagnostic collector 未接生产日志，terminal RR/subgate 证据缺失）；**Order-113** 清理至 `e14263d`；**Order-114** E1 plan-before-motion 原型冻结（3 个 P1 findings、0 探针消耗），行为表面精确恢复 `4ed0157`；旧 Stage-C live 路线保持冻结为失败。
- **未证明项**（本架构的授权前提，属未来订单证明义务）：ROI 暖图因果、Dstop 预算、consumer exact-ack。

## 1. 选定架构：双证书 C0/C1 over BRH

- **双证书（dual-certificate）**：C0 与 C1 是两个独立、可分别失效的证书层，运行于 BRH（brake-recovery-hold）安全包络之上。
  - **C0**（观察运动证书）：解锁**已认证局部观察运动**——仅 known 平地 + 扫掠体（swept-volume）clearance + Dstop 包络内的连续 bounded approach / OBSERVE_HOLD 暖 ROI。C0 不含任何 timed contact transition。
  - **C1**（过渡执行证书）：**锁住 transition 执行**——仅当 full-ROI / Family-A ≥3-contact 完整 timed 计划 / exact-ack 齐备后才接管 crossing 的 timed contact transition。
- **BRH 基线**：任何证书失效时的安全归宿为 brake→recovery→hold 链；C0 失效优先进入 BRAKE_HOLD（见 §4）。
- **deadlock 解除**：C0 永不等待 C1；C0 失效先在 viability 边界内刹停；C1 只在 ROI 齐备后接管。不存在"必须先有 C1 完整计划才能动"的观测死锁。
- **预计划运动包络**：C0 仅 known 平地 + 扫掠体 + Dstop；C1 才引入 timed contact transition。两层的运动承诺严格分离，C0 不得提前执行 C1 的过渡语义。
- **主动感知策略**：连续 C0 bounded approach→OBSERVE_HOLD 暖 ROI；禁止离散 perception maneuver 脚本与 unknown 探测（unknown 区域视为不可入，走 BRAKE_HOLD / 回退）。

## 2. 状态机

```
HOLD → LOCAL_CERT_READY → APPROACH → OBSERVE_HOLD → TRANSITION_PLAN_BUILD
     → TRANSITION_PLAN_PUBLISH → TRANSITION_PLAN_ADOPT → CROSSING
```

- 任一状态中 C0 失效（viability / Dstop / swept-volume 破坏、证书过期、观测无效）优先进入 **BRAKE_HOLD**，不得推进 TRANSITION_*。
- C1 只在 ROI 齐备（full-ROI、Family-A 完整 timed plan、exact-ack）后在 OBSERVE_HOLD 处接管 BUILD/PUBLISH/ADOPT/CROSSING。
- APPROACH 期间的观察仅用于暖 ROI；不得将观察当作过渡计划或实测接触。

## 3. Warmup readiness 定义（分层，不复用单一就绪位）

- **C0 local readiness**：局部 known 平地覆盖、扫掠体 clear、Dstop 预算有效、C0 证书有效且未过期。
- **C1 full-ROI readiness**：完整 ROI 图、Family-A ≥3-contact 完整 timed 计划、identity/epoch 精确对齐的 consumer exact-ack 齐备。
- 两层就绪分别计算、分别发布；C1 就绪缺失不阻塞 C0，C0 就绪缺失直接禁止 APPROACH。

## 4. 安全不变量（Safety Invariants）

1. **C0 不得越过 viability boundary**：C0 运动承诺 ⊆ {known 平地} ∩ {扫掠体 clear} ∩ {Dstop 预算}；越界判定即失效→BRAKE_HOLD。
2. **C1 无完整 ROI / 独立 ack 不得 crossing**：TRANSITION_PLAN_ADOPT 与 CROSSING 以 C1 证书为硬前提；exact-ack 为消费者（gait/SRBD/ID-WBC）对同一 plan identity/epoch 的独立确认，非自证。
3. **planned/measured 分离**：planned contact 永不替代 measured contact 进入 WBC 安全决策；V3-C off；crawl 仅作显式 fallback backend，非正常策略。
4. **C0 失效优先 BRAKE_HOLD**：任何推进优先于 C0 失效后的安全刹停；episode 记 abort/failure，不声称成功剖面。

## 5. 拒绝的备选方案

- **strict E1 full-plan-before-approach**（拒绝）：要求完整计划后才能动造成观测死锁（Order-114 三个 P1 findings 亦证实该路线不成熟）。
- **裸 BRH**、**离散 perception maneuver**、**replay**、**whole-body NMPC**（均不足）：无证书分层 / 离散脚本违反连续感知策略 / replay 无 live 保证 / NMPC 证据与计算负担未满足 B 门。

## 6. 原型与探针（纸面/审查级；不实现、不仿真）

- **一个双证书纸面 trace**：HOLD→…→CROSSING 的逐状态 C0/C1 证书保持与失效注入分支。
- **P0 暖图因果**：OBSERVE_HOLD 的 map 因果链（谁写入、epoch 如何随观察更新、回退时如何失效）纸面审查。
- **P1 actual-FK 仅测量链**：actual-FK 只进测量链（不生成计划落点），纸面审查分离性。
- **P2 静态接口审查**：C0/C1 证书接口、readiness 接口、ack 接口的静态检查。
- 本订单不实现、不仿真、不消耗探针。

## 7. Proof Obligations（未来订单逐项闭合）

- **PO-C0-1**：C0 运动包络 ⊆ viability boundary（known 平地、扫掠体、Dstop）。
- **PO-C0-2**：C0 失效→BRAKE_HOLD 的时延与刹停距离 ≤ Dstop 预算。
- **PO-C1-1**：C1 启动条件 ⇔ full-ROI ∧ Family-A complete timed plan ∧ exact-ack。
- **PO-C1-2**：CROSSING 期间 C0 不在位即刹停（无证书滑脱窗口）。
- **PO-DL**：不存在 C0 等待 C1 的阻塞路径；C1 接管仅由 ROI 齐备触发。
- **PO-SEP**：planned/measured 分离成立；V3-C off；crawl 仅 fallback。
- **证据形式限制**：本架构阶段的 obligations 仅以纸面 trace + P0/P1/P2 支撑；任何实现/仿真证据需后续独立订单产生。

## 8. 证据边界与 Order-115 授权范围

- **可引用证据**：仅 order109b/`5b95e82` C006 canary+3/3 PASS（含 31/31 + 2/2 ctest）；早期 wall-clock 失败（cmd_time 1.4--2×）为残余风险，不否定本架构。
- **未证明（未来义务）**：ROI 暖图因果、Dstop 预算、consumer exact-ack。
- **本订单范围**：仅本设计文档 + `example/cpp/experiments/_runs/ESCALATION_LOG.md` append-only 记录。非 docs/_runs 的 diff 必须为零；无行为代码、无仿真、无新 _runs 运行、无阈值/合约/analyzer/crawl policy 变更。
- **NOT AUTHORIZED**：实现、仿真、探针消耗、任何 Stage-C 执行旗标开启。
