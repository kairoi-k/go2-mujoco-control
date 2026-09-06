# Terrain committed-prefix audit v1

历史行为源自 5cbc547f225dbb60683d96e440beffb0b014a075；本轮读取 aa14604 工作树（gait lift WIP，未见 commitment 修改）。本文只记录代码语义与可检验方案；未运行模拟、未启动控制器、未修改生产源码。

## 结论

当前已锁定的 terrain swing target 不是由 terrain_tick_plan_ 指针直接保存，而是分散在 TrotExperiment 的 per-leg 数组中。控制 tick 每次先清空 terrain_tick_plan_，再只加载 usable plan，因此计划过期会让 gait 停止应用仍处于 in-flight 的 target；该 target 的 valid/in_flight 标志本身不会被清空。新计划稍后恢复时，in_flight 仍为真，gait 会继续使用旧 target，但过期窗口的一个或多个输出 tick 已回到 kernel 足端轨迹。

WBC 过期时不把已锁 target 当作当前时刻的 foothold；当前时刻使用 dyn 实际足端构造杠杆臂是正确的，不能换成尚未达到的 target。真正缺口是未来 MPC 接触 knot 没有 committed event 的 target/normal。若只修 gait 让足端继续走旧 target，命令层可连续，但未来 WBC reference/force cone 仍可能与 committed prefix 不一致，不能据此宣称 dynamic B1 已闭环。

## Gait 生命周期证据

1. trot_experiment_control.cpp:460 每个控制 tick 执行 terrain_tick_plan_.reset()；:463-468 仅在 terrain enabled 且 actuation/shadow 打开时调用 TerrainPlanStore::LoadUsable。terrain_motion_plan.h:201-205 的 usable_at 同时要求 plan.valid() 且 now <= valid_until_s；:280-286 的 LoadUsable 对过期 plan 返回 nullptr。

2. trot_experiment_gait.cpp:1484-1491 用清空后的 terrain_tick_plan_ 建 lookup，并要求 plan 存在、usable_at 为真且 lookup.valid 才进入 terrain execution 整段。故过期 tick 不会执行 :1501-1522 的完成处理，也不会执行 :1525-1565 的新 target prepare，更不会执行 :1567-1596 的 target-to-body blend 和 lift。

3. target 数组定义在 trot_experiment.h:542-562。当前代码没有在控制 tick 过期时清空这些数组。成功 prepare 在 gait.cpp:1539-1552 写入 target_world、target_lift、touchdown time、swing start phase、source plan id/epoch、target_valid 和 in_flight。若新 plan 在旧 swing 仍 in-flight 时恢复，:1525 条件为假，因此会复用旧 target；这解释了“目标状态还在、输出却中间丢失”的断裂。

4. gait.cpp:1503-1522 的 !in_swing 完成分支也受当前 plan 外层 gate 保护。若 plan 从 swing 中途过期并持续到 stance，in_flight 可能一直保持真；只把 target apply 移出外层 gate、而不把 completion 移出，会把旧 in-flight 状态带入后续 swing。completion 必须每 tick 按 running-trot phase 运行，prepare 才能继续依赖 usable plan。

5. touchdown 后另有独立的竖直连续性缺口。gait.cpp:1503-1522 在 !in_swing 时直接 continue，不应用 target z；而 support_anchor_feedback 的 gait.cpp:1364-1404 只把 feet.x/y 混合到世界 support anchor，未混合 z。因此上台 foothold 着地后的下一 stance tick，terrain target z 会退回 nominal kernel z。这个问题独立于“过期中途丢失”，5 cm dynamic acceptance 必须单独决定是否要求 terrain support z hold。

## WBC 过期语义与不一致

WBC 的接触 mask 不是 terrain target 的第二份来源。trot_experiment_wbc.cpp:143-210 从力传感器、running-trot schedule、stop/merge 状态形成 qp_contact；:647 直接把它送入 IdWbcInput.contact。当前 plan 只提供 provenance、foot reference 和 normals，不替换 qp_contact。

当前 usable plan 在 wbc.cpp:212-235 被筛成 terrain_plan_active。MPC 输入先在 :523-525 以 dyn.foot_pos_world[leg] - dyn.com_world 初始化 nominal lever arm；只有 terrain_plan_active 时，:556-603 才按每个 horizon sample 的 TerrainPlanKnotAtTime 和 planned_contact，将 valid predicted_foothold.position_world 替换进去。srbd_mpc.h:129-139 的 SrbdFootAt 也明确：没有 valid time-indexed foothold 时回退到当前 dyn lever arm；:150-162 对接触 knot 要求 foothold 有效。

因此 plan 过期后，当前时刻 WBC 继续以真实 dyn 足端构造杠杆臂，这是正确的；不能把尚未达到的 target 提前用于当前力矩。缺口在未来 horizon：已锁 target_world 不会进入新的 MPC 接触 knot，输入继续以当前 dyn lever arm 回退；WBC 的 wbc_in.has_terrain_plan、planned_contact 和 surface normal 也只在 wbc.cpp:648-679 的 terrain_plan_active 分支写入。inverse_dynamics_wbc.h:319-327 和 :480-488 对无 valid normal 使用世界 Z 法向。因此未来预期 touchdown 的 target/normal 丢失，当前观测与未来预期未分开表达。

还有一个短暂的 reference lag：wbc.cpp:412-414 以 mpc_period_ticks 决定是否刷新 MPC；:608-616 仅成功刷新时更新 last_srbd_。plan 刚过期而尚未到下一次 MPC tick 时，:681-684 仍把上一解的 first_linear_acc 交给 WBC；在启用 high-speed force tracking 时，:744-750 也可能继续使用上一解的 first_force。这是刷新延迟，可能带有旧 plan 影响，但不能当作当前 WBC 已消费 committed foothold；测试必须单独记录当前 dyn、未来 event target 和 last_srbd 来源。

shadow 路径不能直接补这个缺口。wbc.cpp:280-282 明确 shadow 不反馈 gait、MPC、WBC 或 torque；:304-322 在无当前 plan 时只记录 readiness failure，terrain_shadow_commitments_ 不会成为执行权威。把它接入 gait/WBC 会形成第二套 commitment 状态，并且其 horizon/event coherence 条件（terrain_execution_consistency.h:258-433）与实际 per-leg gait 数组并非同一对象。

## 只让 gait 继续应用的后果

只把 gait.cpp:1567-1596 的 apply 逻辑移到 plan gate 外，并保持现有 qp_contact，能修复“过期窗口 foot command 回到 kernel”的一阶断裂，但不能修复未来 WBC reference 语义；无 commit/新 target 时 kernel fallback 只表示没有 terrain override，不是 terrain safety fail-closed 或 B1 认证：

- swing 期间 q target 可继续指向旧 terrain foothold；UpdateWbcFull 读取本 tick LowState 的 dyn.foot_pos_world，并以实际足端构造当前 nominal lever arm，这是正确的观测语义，不能把未到达 target 提前替换进当前力矩。
- 对未来 contact knot，当前 MPC 没有 committed event 的 target；过期后继续从 dyn 当前足端生成未来 lever arm。实际 touchdown 后当前观测最终会正确，但预期 foothold 与未来 reference 仍未被表达；MPC 刷新前还可能短暂复用上一次 SRBD 解。
- terrain plan 过期后，未来 event 的 surface normal 不再注入；WBC 对无 valid normal 回退 UnitZ。只补 target z 而不补同一 commitment 的法向，会让未来上台支撑的 force cone/reference 不完整。
- 若为了“让 WBC 继续”改成保留 expired terrain plan 指针，会把失效计划的 contact schedule、horizon 和 metadata 一起复活；若把 shadow commitments 接入 WBC，则建立第二接触/承诺权威。

## 最小修复候选

把现有 per-leg target 状态提升为唯一的 committed-prefix 记录，至少携带 target_world、target_lift、touchdown_time、swing start、surface normal 和 source plan id/epoch；terrain_tick_plan_ 只负责准备尚未承诺的新 touchdown。

Gait 侧每 tick 先按原 running-trot phase 更新 completion，再无条件应用仍 in_flight 的 committed target；只有 current plan usable 且 knot lookup 有效时才调用 TerrainPlanNextTouchdown 准备新 target。没有 commit 且没有 usable 新 target 时保持 kernel feet，不能猜测或沿用已完成 target；这是未认证 terrain fallback，不等于 terrain safety fail-closed，若要求安全闭环需另有明确 reject/stop gate。新 plan 到来时，in_flight commit 优先，直到 touchdown/completion；这不改变接触 schedule。

WBC 侧保留 qp_contact 作为唯一接触 mask；当前时刻仍用 dyn 实际足端，不能提前替换成未到达 target。若要闭合 future MPC contract，则由同一 committed-prefix 为 touchdown 之后的未来 contact knot 提供 target lever arm/normal；current plan 无效时不要设置 has_terrain_plan，不复活 expired plan。若暂时只做 gait patch，应把 future WBC prefix mismatch 记录为未解决的独立验收条件，不能称 dynamic B1 完成。touchdown 后 z hold 也必须明确是否纳入同一 support commitment；若纳入，则由同一记录提供 stance anchor z，并在 liftoff 时清除。

diagnostics 需同时输出 current_plan_usable/current plan id、per-leg committed source id/epoch、commitment in_flight、target_applied_this_tick、completion_this_tick，以及 WBC prefix lever-arm/normal source。现有 diagnostics.cpp:729-740 只反映当前 plan；:907-926 仍可显示旧 target valid/坐标；:878-899 的 transition mask 还不会在空 plan 清零，不能用现有字段证明“已应用”。

## 最小只读状态机测试矩阵

| 场景 | 输入状态 | 期望 gait | 期望 WBC | 关键断言 |
|---|---|---|---|---|
| A 当前 plan | P1 usable，腿 L 在 swing，L 无 commit，P1 有 touchdown T1 | prepare 并 apply T1 | terrain plan active；qp_contact 仍 kernel schedule | source=P1，target_applied=1 |
| B 过期中途 | P1 先锁 T1，随后 now>valid_until，仍在同一 swing，无 P2 | 每 tick 继续 apply T1；不 prepare 新 target | 不消费 expired plan；qp_contact 不变；prefix 若已实现则保留 T1 | feet 不回 kernel，source=P1 |
| C 新 plan 冲突 | B 后 P2 usable，但给 L 的 touchdown 与 T1 不同 | T1 直到 completion，不被替换 | contact mask 不因 P2 改变；prefix source 仍 P1 | no target replacement while in_flight |
| D 新目标未知 | 无 commit，P2 缺 target、invalid 或 lookup 不可用 | kernel fallback，不能猜测；仅未认证 terrain fallback | no terrain plan/prefix foothold injected；current dyn semantics unchanged | no prepare success，no contact authority change |
| E touchdown stance | T1 已到 touchdown，进入 !in_swing | completion 每 tick 运行；按选定合约保持或释放 target z | 若 support prefix 要求成立，lever arm/normal 继续来自 T1 | 当前代码预期暴露 z-drop，单独记录 |
| F 下一 swing | E 后 liftoff，无新 usable plan | 旧 commit 不得重新 apply；kernel fallback（非 terrain fail-closed） | qp_contact 仍 baseline | in_flight=false，no stale target |
| G WBC refresh lag | P1 解刚产生，plan 随后过期但尚未到 run_mpc tick | gait 继续 T1 | 旧 last_srbd 只能短暂复用并明确标 provenance，下一刷新不得带 expired foothold | no stale terrain plan metadata |
| H diagnostics | B/E/F 各阶段 | applied/completion 与 current plan 分开可见 | WBC terrain_plan_id/committed_prefix_id 可区分 | 不出现 valid=1 但 applied=0 的未标注状态 |

最小 fixture 不需要 MuJoCo：构造一个单腿 P1、T1、valid_until 和 running-trot phase 序列，依次喂入 usable、expired、无 P2、touchdown、下一 swing；同时给 WBC 一个固定 dyn foot 和 qpc schedule，检查 MPC lever arm/normal source。该 fixture 只验证状态机和输入接线，full dynamic acceptance 仍需真实闭环证据。
