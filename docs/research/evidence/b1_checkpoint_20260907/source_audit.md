# B1 checkpoint source audit (2026-09-07)

审计边界：本轮只读，未改仓库、未编译、未启动仿真。主线给出的当前源 provenance 为 WSL repo `eeb5d757620712759604d8c51b2b9075d05625dc`（eeb5d75），恢复开始时工作树 clean；收束过程中仅主 Agent 修改交接文档和证据。此前行为证据源自该 SHA。`/tmp/fb14417_initial_support_audit_v1.py/.json` 明确标为 `acceptance_claim=false`，head 是 fb14417，不能替代 eeb5d75 的闭环证据。

## 临时产物盘点

Windows 当前目录仍有未应用的草稿：`draft_terrain_swing_full_20260907.patch`（70914 B）、`draft_terrain_swing_production_integration_20260907.patch`（6866 B）、`draft_terrain_swing_contract_20260907.h`、`draft_terrain_swing_contract_prod_20260907.h`、`draft_test_terrain_swing_contract_20260907.cpp` 及 checker/production test 草稿。它们是隔离草稿/副本，不能当 eeb5d75 当前实现；integration 说明也以 764a21c 为基线并列出未解决的 MPC future translation/prefix。递归检索未发现名为 eventtiming 或 contactledger 的已完成补丁；`acceptance_v2_draft`、`acceptance_v2_draft_0001` 和 `/tmp/acceptance_v2` 只留下 analyzer、协议/审计文档和 stance 结果，没有当前生产 contact-ledger patch。`/tmp/b1_swing_path_witness_20260907` 是 ELF 二进制，未发现可复核的源码/原始结果包。

## 已由 eeb5d75 源码确认

1. `example/cpp/trot/trot_experiment_gait.cpp:212-219,300-314` 以 stand-up FK 足位填 `neutral_feet`，调用 kernel 后直接复制 `touchdown_target_feet_base`；`example/cpp/gait/locomotion_kernel.h:172-194,286-302,320-346` 说明该 target 是 body-frame nominal endpoint，算法是当前 neutral foot 加 `direction_sign * half_step`。它没有未来 base 平移项。

2. `example/cpp/terrain/terrain_planner.h:304-310` 对每个未来 touchdown 调 `EvaluateFoothold(..., &input.current_feet_base[leg], ...)`。`trot_experiment_control.cpp:263-268` 的 `current_feet_base` 来自当前 joint-position FK。因此“当前 stance FK 被当作未来 swing 起点”是确认的接线事实；该起点不等于事件对应的 liftoff/上一条已执行 commanded foot。

3. `terrain_planner.h:321-337` 用输入 target 与当前 `planner_frame` 做候选评分，`terrain_planner.h:512-550` 把同一当前 body pose/当前 state stamp 复制到 body knots，并以 `state_stamp + knot * knot_dt` 生成 touchdown 时间。`terrain_planner.h:535-550` 目标 world 点也用当前 frame 转换。故“future-body/base translation 缺失”在 planner 输入和变换语义上确认；未来平移造成多少实际落点误差，以及是否是本次撞击首因，仍未做对照实验。

4. `terrain_planner.h:472-482` 仅以 measured-contact false→planned-contact true 的首个 knot 识别 touchdown，`terrain_execution_consistency.h:227-255` 只按 leg、时间和 `foot.touchdown` 扫描，不携带 cycle/event ID。`terrain_swing_contract.h:74-97` 用当前 `state_time + (1-leg_phase)*period` 修正粗 knot TD，并以半周期容差接受旧 TD。事件身份可能错配是确认的语义缺口，但在 eeb5 run 中是否实际错配未记录/未证实。

5. `trot_experiment_gait.cpp:1551-1668` 在新 swing 用 `previous_commanded_world_feet_[leg]` 和其时间建立 contract，过期 plan 对已经 in-flight 的 contract 仍可继续；`terrain_commitment_lifecycle.h:44-74` 明确此生命周期。`trot_experiment_gait.cpp:1684-1713` 执行 contract 后做 live world→body。当前 WBC 用实际 dyn foot 杠杆臂计算；`trot_experiment_wbc.cpp:937-1021` 只把 gait 输出的 commanded-body-foot velocity 加入 swing reference。已锁定目标被单独取消的问题未在这些路径确认；真正的风险是候选未来事件仍缺 committed event target/normal，不能用当前观测冒充未来预期。

6. `terrain_planner.h:19-37,209-216` 默认 `horizon_knots=8,knot_dt=.020`；`trot_experiment_wbc.cpp:239-242` MPC 为 8 knots，dt=`clamp(period/8,.020,.05)`。所以 P=.14 时两者最后样本约 .14；P=.28 时 MPC span 约 .245 而 terrain plan 仍约 .14。`trot_experiment_lifecycle.cpp:327-346` 只有 `TROT_TERRAIN_EXECUTION_CONSISTENCY_SHADOW` 才把 terrain horizon 改到 24。eeb5 argv 为 `--period 0.14 --duty 0.44`，本次 run 不触发 P=.28 mismatch；P=.28 需单独一致性实验。

7. 足端几何常量由 `kinematics/go2_forward_kinematics.h:30-45` 定义：site→contact-patch z offset .022 m；`terrain_feasibility.h:63-100,130-143` 的 map patch radius .025 m，5 cm 栅格下可行 foothold half extent 被压到零。`terrain_feasibility.h:708-818` 内部 sweep 跳过端点并检查 patch 高度，端点另行检查。半径/采样造成误拒或漏碰是待证假设，不是当前证据已确认的首因；现有 site/foot contact 行不能证明它。

## eeb5 原始 run 对阻塞链的证据

`example/cpp/experiments/_runs/b1_intervals_v2_step_eeb5d75_20260907_0001/run_metadata.txt` 标明 clean eeb5d75、scene `b1_v3_running_step_5cm.xml`、period .14、duty .44。`data.csv` 的控制行在 22.902、22.934、22.984、23.038、23.140、23.238 s 均为 `terrain_plan_failure=4`、plan invalid、applied mask=0，四腿 execution valid=0；分析器交互窗口从 23.108 s 开始，所以记录的 first-impact 前段没有 target/applied foothold。23.758 s 才出现 FL/RR target（plan 353、mask 6），23.804 s FR 出现，23.810 s plan 354 已 invalid 但 in-flight 仍短暂执行，23.884 s 后清空。该 run 没有 planner per-candidate debug reason；只能确认“全局 plan 被拒/目标未产生”，不能从此非 debug run 单独区分 unknown 与 swing-clearance；源码先返回 no-safe-foothold 再进行 SupportFeasible，deadline 则有独立失败码，因此这些 failure=4 行不支持把 15 mm support 或 deadline 当作直接拒绝原因。

## 未证实项与最小下一步

未证实：future translation 是否足以解释实际撞击；event-time 容差是否在 scheduler 改 period 后错配；interior foot radius 是否改变 checker 结论；某一腿是否因 stance-pending 生命周期丢失 target。最小闭环是同一 frozen scene 做三组纯日志/fixture：当前 FK 起点与事件 liftoff 起点各跑 checker 并比较 clearance；P=.14 与 P=.28 断言 terrain plan 覆盖 MPC 最后采样；把 kernel body endpoint 按 `v*TD` 平移后对照当前 target，并记录每腿拒绝 reason。执行仍须保留 all-leg rejection，先补 per-leg reason/事件 ID，再做一次真实 canary；本 checkpoint 不启动。

结论：当前最硬的源码事实是“planner 用当前 FK 评估所有未来 touchdown”以及“target/frame 没有 future base displacement”；二者影响尚未被同场景最小对照实证。旧草稿未移植、无未完成生产补丁可直接恢复。
## 主 Agent 复核与范围限定

已核对 kernel 实际路径为 `example/cpp/gait/locomotion_kernel.h`，修正上述原草稿路径拼写。未来平移缺失是源码事实，尚无因果对照；本轮不采用未经验证的直接平移补丁。半径处理不一致也仅作为 certificate 限制，不能据此解释全部碰撞。

新 eeb5d75 debug 首碰前窗口 [22.25,23.05) 中，parsed 17 条 per-leg 日志包含 swing_clearance=249、unknown=332。三条起点 clearance 约 -2.35/-3.18/-3.42 mm 的日志共有 188 次候选拒绝；另 61 次对应 -52.905 mm，绝不能一并称为普通接触压入或统一放宽。unknown 既出现在起点 patch 不完整，也出现在起点已知但后续路径未知的日志。debug 轨迹不同于无 debug 的视频 run，不把其细分比例投射到视频 run。

本 checkpoint 的下一步以 README 的单个离线状态/几何 witness 为准；上文 horizon、future-target 等 fixture 是后续备选，不代表已安排或运行三组新实验。临时文件盘点不按名称声称全磁盘穷尽；已确认主源码和二进制与原始 135 文件 binding 一致，无中断补丁被偷偷纳入本次收束。
