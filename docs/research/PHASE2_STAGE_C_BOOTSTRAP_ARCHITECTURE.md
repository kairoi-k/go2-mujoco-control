# Phase 2 Stage C Bootstrap 架构：双证书 C0/C1

状态：**DESIGN ONLY / NOT AUTHORIZED**（Order-115，2026-09-01）。本文件只定义架构、状态机、安全不变量与 proof obligations，不改变行为代码，不下达实现订单，不启动仿真，不声称任何 B1-B3 门通过，也不包含仿真结果。基线 HEAD：`3eacac1`（Order-114 E1 冻结后的 4ed0157 表面）。

**修正版说明（Order-115 P1 修正）**：本版修正安全句（C0 失效后的 BRAKE_HOLD 优先于任何推进）、状态机（OBSERVE_HOLD→C1_CANDIDATE_BUILD→C1_CERTIFY→PUBLISH→ADOPT_PENDING→C1_ARMED→CROSSING）、owner/token 与认证 tuple、可接受观测视点与完整延迟 Dstop 证明义务；ESCALATION_LOG 以 append-only correction 块同步，旧块未改动。

## 0. 决策背景与 C006 事实

- **正式 C006 事实（order109b / `5b95e8265c885a81f8488e4930e682aa55f05674`）**：canary + 3/3 holdout 独立固定对全部 PASS；生命周期、fixed 3 m/s、lockstep 协议、authoritative B0、terrain sensor-only、planner deadline、no publish/consumer/actuation、frozen 非回归检查全部通过；`ctest` example/cpp 31/31、simulate 2/2 PASS；证据见 `docs/research/evidence/order109b_c006i/`（docs-only，运行数据在忽略的 experiment workspace）。这是 Stage-C 可引用的唯一正式认证基线。
- **早期 wall-clock 失败仅为残余风险**：`e65e155`/`3273bd5` 的 B0 失败归类为 controller 内部时钟与 sim 时间不同步（cmd_time ~1.4--2× sim time、wall_clock_motion override），属于 Phase-1 控制律/接触鲁棒性残留，不是对本架构的否决；不构成"能跨过已认证局部观察运动"的否定证据。
- **Order-112 INCONCLUSIVE**（diagnostic collector 未接生产日志，terminal RR/subgate 证据缺失）；**Order-113** 清理至 `e14263d`；**Order-114** E1 plan-before-motion 原型冻结（3 个 P1 findings、0 探针消耗），行为表面精确恢复 `4ed0157`；旧 Stage-C live 路线保持冻结为失败。
- **未证明项**（本架构的授权前提，属未来订单证明义务）：ROI 暖图因果、完整延迟 Dstop 预算、consumer exact-ack（ack_deadline 语义）。

## 1. 选定架构：双证书 C0/C1 over BRH

- **双证书（dual-certificate）**：C0 与 C1 是两个独立、可分别失效的证书层，运行于 BRH（brake-recovery-hold）安全包络之上。
  - **C0**（观察运动证书）：解锁**已认证局部观察运动**——仅 known 平地 + 扫掠体（swept-volume）clearance + Dstop 包络内的连续 bounded approach / OBSERVE_HOLD 暖 ROI。C0 不含任何 timed contact transition。
  - **C1**（过渡执行证书）：**锁住 transition 执行**——仅当 complete candidate（full-ROI / Family-A ≥3-contact 完整 timed 计划）与独立 exact-ack 齐备后才接管 crossing 的 timed contact transition。
- **owner/token（运动所有权）**：C0 在 C1_ARMED 之前始终持有运动 owner（motion lease：观察运动与 BRAKE_HOLD 的排他权）；C1 仅是过渡执行候选证书，在 C1_CANDIDATE_BUILD 之前不得 ready。owner 移交只在 C1_ARMED 的**原子 owner lease** 中发生：独立 acks 齐备后一次性交接，不存在证书滑脱窗口。
- **认证 tuple**：所有候选/计划/ack 携带 `plan_id / plan_epoch / map_epoch / state_seq / input_hash / frame_id / valid_until / ack_deadline`。任一 **unknown**（陌生 id/frame）、**stale**（valid_until 过期）、**frame 失配**、**epoch 失配**或 **late ack**（超过 ack_deadline）即撤销 C1 证书；撤销后 C0 BRAKE_HOLD 抢占，不得推进任何 ADOPT/CROSSING。
- **BRH 基线**：任何证书失效时的安全归宿为 brake→recovery→hold 链；C0 失效后的 BRAKE_HOLD 优先于任何推进（见 §4）。
- **deadlock 解除**：C0 永不等待 C1；C0 失效先在 viability 边界内刹停；C1 只在 ROI 齐备后接管。不存在"必须先有 C1 完整计划才能动"的观测死锁。
- **预计划运动包络**：C0 仅 known 平地 + 扫掠体 + Dstop；C1 才引入 timed contact transition。两层的运动承诺严格分离，C0 不得提前执行 C1 的过渡语义。
- **主动感知策略**：连续 C0 bounded approach→OBSERVE_HOLD 暖 ROI；禁止离散 perception maneuver 脚本与 unknown 探测。**可接受观测视点（admissible observation viewpoint）**：观测仅允许从 C0 已认证包络内的视点进行——该视点自身须 known 平地、扫掠体 clear、Dstop 预算成立；若 C0 无法安全到达视点或 ROI 无法完整覆盖 → **HOLD**，绝不探 unknown（unknown 区域视为不可入）。

## 2. 状态机

```
HOLD → LOCAL_CERT_READY → APPROACH → OBSERVE_HOLD → C1_CANDIDATE_BUILD
     → C1_CERTIFY → PUBLISH → ADOPT_PENDING → C1_ARMED → CROSSING
```

- **C1_CANDIDATE_BUILD**：由 C0/中立 builder（neutral builder）在 C0 仍持运动 owner 的前提下构建过渡候选；C1 在此之前**不得 ready**。
- **C1_CERTIFY**：full ROI ∧ complete candidate（Family-A ≥3-contact 完整 timed 计划）齐备后，候选通过认证。
- **PUBLISH**：候选以认证 tuple（plan_id/plan_epoch/map_epoch/state_seq/input_hash/frame_id/valid_until/ack_deadline）整体发布。
- **ADOPT_PENDING**：等待消费者独立 exact-ack（对同一 plan identity/epoch 的确认，ack_deadline 内）。
- **C1_ARMED**：独立 acks 齐备后执行原子 owner lease 交接（C0→C1）；交接完成才允许 CROSSING。
- **CROSSING**：timed contact transition 执行；C0 仍作为失效哨兵，任一失效即 BRAKE_HOLD 抢占。
- 任一状态中 C0 失效（viability / Dstop / swept-volume 破坏、证书过期、观测无效）优先进入 **BRAKE_HOLD**，不得推进 PUBLISH/ADOPT/CROSSING。
- 认证 tuple 任一失配（unknown / stale / frame / epoch / late ack）即撤销 C1，回退 C0 BRAKE_HOLD 抢占。
- APPROACH/OBSERVE_HOLD 期间的观察仅用于暖 ROI；不得将观察当作过渡计划或实测接触。

## 3. Warmup readiness 定义（分层，不复用单一就绪位）

- **C0 local readiness**：局部 known 平地覆盖、扫掠体 clear、Dstop 预算有效、C0 证书有效且未过期。
- **C1 candidate readiness**：仅在 C1_CANDIDATE_BUILD 完成后可达——完整 ROI 图 + Family-A ≥3-contact 完整 timed 计划（complete candidate）齐备；**C1 不得在 BUILD 前 ready**。
- **C1_ARMED gate**：独立 exact-ack（身份/epoch 对齐、ack_deadline 内）齐备后原子 owner lease；ack 缺失/过期不得 arm。
- 两层就绪分别计算、分别发布；C1 就绪缺失不阻塞 C0，C0 就绪缺失直接禁止 APPROACH。

## 4. 安全不变量（Safety Invariants）

1. **C0 不得越过 viability boundary**：C0 运动承诺 ⊆ {known 平地} ∩ {扫掠体 clear} ∩ {Dstop 预算}；越界判定即失效→BRAKE_HOLD。
2. **C1 无完整 ROI / 独立 ack 不得 crossing**：C1_CERTIFY、ADOPT_PENDING、C1_ARMED 与 CROSSING 以 C1 证书与认证 tuple 为硬前提；exact-ack 为消费者（gait/SRBD/ID-WBC）对同一 plan identity/epoch 的独立确认，非自证。
3. **planned/measured 分离**：planned contact 永不替代 measured contact 进入 WBC 安全决策；V3-C off；crawl 仅作显式 fallback backend，非正常策略。
4. **C0 失效后的 BRAKE_HOLD 优先于任何推进**：C0 失效锁存后，刹停动作优先于任何推进动作（含 C1 的 ADOPT/CROSSING）；episode 记 abort/failure，不声称成功剖面。
5. **不可观测则不探**：C0 无法安全到达可接受观测视点或 ROI 无法完整覆盖时进入 HOLD，绝不探 unknown。

## 5. 拒绝的备选方案

- **strict E1 full-plan-before-approach**（拒绝）：要求完整计划后才能动造成观测死锁（Order-114 三个 P1 findings 亦证实该路线不成熟）。
- **裸 BRH**、**离散 perception maneuver**、**replay**、**whole-body NMPC**（均不足）：无证书分层 / 离散脚本违反连续感知策略 / replay 无 live 保证 / NMPC 证据与计算负担未满足 B 门。

## 6. 原型与探针（纸面/审查级；不实现、不仿真）

- **一个双证书纸面 trace**：HOLD→LOCAL_CERT_READY→APPROACH→OBSERVE_HOLD→C1_CANDIDATE_BUILD→C1_CERTIFY→PUBLISH→ADOPT_PENDING→C1_ARMED→CROSSING 的逐状态 C0/C1 证书保持、owner lease 与失效注入分支（含 tuple 失配撤销路径）。
- **P0 暖图因果**：OBSERVE_HOLD 的 map 因果链（谁写入、epoch 如何随观察更新、回退时如何失效）纸面审查。
- **P1 actual-FK 仅测量链**：actual-FK 只进测量链（不生成计划落点），纸面审查分离性。
- **P2 静态接口审查**：C0/C1 证书接口、readiness 接口、ack 接口、认证 tuple 字段的静态检查。
- 本订单不实现、不仿真、不消耗探针。

## 7. Proof Obligations（未来订单逐项闭合）

- **PO-C0-1**：C0 运动包络 ⊆ viability boundary（known 平地、扫掠体、Dstop）。
- **PO-C0-2（完整延迟 Dstop）**：C0 失效→BRAKE_HOLD 的刹停距离 ≤ Dstop 预算；延迟链须含 sensor→filter→planner→publish→adoption→actuation→halt 的**全延迟**，逐环节测量并加总，不得只证纯制动段。
- **PO-C1-1**：C1_CERTIFY 条件 ⇔ full-ROI ∧ complete candidate（Family-A complete timed plan）。
- **PO-C1-2**：C1_ARMED 条件 ⇔ 独立 exact-ack 齐备（ack_deadline 内、identity/epoch 对齐）∧ 原子 owner lease 完成；CROSSING 期间 C0 不在位即刹停（无证书滑脱窗口）。
- **PO-C1-3（tuple 失配撤销）**：任一 unknown/stale/frame/epoch/late-ack 失配 → 撤销 C1 且 C0 BRAKE_HOLD 抢占的时序 ≤ 安全时限。
- **PO-OBS（视点与不可观测）**：可接受观测视点的可达性（C0 包络内）与视点内 ROI 完整性；C0 无法安全到达 → HOLD，不探 unknown。
- **PO-OWN（owner 原子性）**：C1_ARMED 前 C0 持有运动 owner；owner 移交一次性原子完成，无证书滑脱窗口。
- **PO-DL**：不存在 C0 等待 C1 的阻塞路径；C1 接管仅由 ROI 齐备触发。
- **PO-SEP**：planned/measured 分离成立；V3-C off；crawl 仅 fallback。
- **证据形式限制**：本架构阶段的 obligations 仅以纸面 trace + P0/P1/P2 支撑；任何实现/仿真证据需后续独立订单产生。

## 8. 证据边界与 Order-115 授权范围

- **可引用证据**：仅 order109b/`5b95e82` C006 canary+3/3 PASS（含 31/31 + 2/2 ctest）；早期 wall-clock 失败（cmd_time 1.4--2×）为残余风险，不否定本架构。
- **未证明（未来义务）**：ROI 暖图因果、完整延迟 Dstop 预算、consumer exact-ack（ack_deadline 语义）。
- **本订单范围**：仅本设计文档（P1 修正）+ `example/cpp/experiments/_runs/ESCALATION_LOG.md` append-only correction 块。非 docs/_runs 的 diff 必须为零；无行为代码、无仿真、无新 _runs 运行、无阈值/合约/analyzer/crawl policy 变更。
- **NOT AUTHORIZED**：实现、仿真、探针消耗、任何 Stage-C 执行旗标开启。
