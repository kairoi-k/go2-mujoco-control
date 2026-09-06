# 来源与读取范围

审计固定源码：`f3b452d56b2bedd5ea02249d4e5087b6ca151c47`。公开网页检索日期：2026-09-06。

本文中的 [Rxx]/[Pxx]/[Sxx] 指向这里；代码结论优先依据固定提交上的实现，运行结果优先标明“报告称”。没有取得 H8/H9 完整本地 CSV，没有在 Atlas 执行仿真。

## R01
[远端分支集合](https://api.github.com/repos/kairoi-k/go2-mujoco-control/branches)
GitHub branches GET；调查分支指向审计基线，fix 分支较旧；不是本地工作树检查。

## R02
[当前状态与 H9 记录](https://github.com/kairoi-k/go2-mujoco-control/blob/f3b452d56b2bedd5ea02249d4e5087b6ca151c47/CURRENT.md)
重点读取 H9、Active architecture、Ordered plan、Invariants；H9 是报告层证据。
Git blob SHA：`d61e468fcda66afca63dd30448bbbae4976e002d`。

## R03
[Agent 与证据规则](https://github.com/kairoi-k/go2-mujoco-control/blob/f3b452d56b2bedd5ea02249d4e5087b6ca151c47/AGENTS.md)
全文。
Git blob SHA：`461845180840de4bb90cac25b041ac5f9ca9878a`。

## R04
[冻结验收合同](https://github.com/kairoi-k/go2-mujoco-control/blob/f3b452d56b2bedd5ea02249d4e5087b6ca151c47/docs/research/PHASE2_ACCEPTANCE.md)
全文。
Git blob SHA：`1b065f4e7dadbcecc213d927dad248517a1e3b51`。

## R05
[冻结场景与域](https://github.com/kairoi-k/go2-mujoco-control/blob/f3b452d56b2bedd5ea02249d4e5087b6ca151c47/docs/research/PHASE2_HOLDOUT_MANIFEST.json)
全文；holdout 不用于参数开发。
Git blob SHA：`3dddeb7fbf57457dfc8bee4210b6b7d5635ea093`。

## R06
[逐腿规划与支撑检查](https://github.com/kairoi-k/go2-mujoco-control/blob/f3b452d56b2bedd5ea02249d4e5087b6ca151c47/example/cpp/terrain/terrain_planner.h)
全文分段读取；SupportMargin2D / Build / PopulatePlan / FirstTouchdownKnot。
Git blob SHA：`20d63d20f2677f6374dba8d61b6c796fc415ca3c`。

## R07
[控制快照与 planner 输入](https://github.com/kairoi-k/go2-mujoco-control/blob/f3b452d56b2bedd5ea02249d4e5087b6ca151c47/example/cpp/trot/trot_experiment_control.cpp)
103–285 行；支撑锚点的 actuation guard、COM 时间、地图与状态输入。
Git blob SHA：`da073de5e91c54e95ea9422da9d33b2a0c5793f3`。

## R08
[Shadow planner 配置](https://github.com/kairoi-k/go2-mujoco-control/blob/f3b452d56b2bedd5ea02249d4e5087b6ca151c47/example/cpp/trot/trot_experiment_lifecycle.cpp)
225–280 行；24 knots 与 0.46s validity。
Git blob SHA：`794aa5a0706edcf57c5bf985a8351f1ce62f894c`。

## R09
[执行一致性结构](https://github.com/kairoi-k/go2-mujoco-control/blob/f3b452d56b2bedd5ea02249d4e5087b6ca151c47/example/cpp/terrain/terrain_execution_consistency.h)
1–170 行及 e6253fd 的 readiness 变更；事件、身份与诊断结构。
Git blob SHA：`3a93e68aa018c8eff9cca50029fc7fd879c01c12`。

## R10
[WBC 与 shadow 调用](https://github.com/kairoi-k/go2-mujoco-control/blob/f3b452d56b2bedd5ea02249d4e5087b6ca151c47/example/cpp/trot/trot_experiment_wbc.cpp)
280–430 行，checked 设置位置、commitment 预筛选、MPC 更新条件；其他历史片段不当作完整现行文件复核。
Git blob SHA：`5d5e855db70b6c0494081d92aa1286f267ff4e41`。

## R11
[地图结构与坐标辅助](https://github.com/kairoi-k/go2-mujoco-control/blob/f3b452d56b2bedd5ea02249d4e5087b6ca151c47/example/cpp/terrain/terrain_model.h)
1–230、330–470 行；valid() 不检查 known coverage。
Git blob SHA：`f3abde5afc473699d3bf21dbb3fa3c973b8aeaac`。

## R12
[候选安全区域](https://github.com/kairoi-k/go2-mujoco-control/blob/f3b452d56b2bedd5ea02249d4e5087b6ca151c47/example/cpp/terrain/terrain_feasibility.h)
1090–文件结尾，另已读配置/辅助定义；BuildSafeFootholdRegions 的 half 公式。
Git blob SHA：`72d37b40a285adbf79e0ddbaed38a40c9aa9573c`。

## R13
[模拟传感器与有限地图](https://github.com/kairoi-k/go2-mujoco-control/blob/f3b452d56b2bedd5ea02249d4e5087b6ca151c47/simulate/src/unitree_sdk2_bridge.h)
605–1055 行；世界栅格边界、局部窗口、向下射线与传感器发布。
Git blob SHA：`c6421306ee2501d81ab02642b6fce2802d12ad0d`。

## R14
[当前 SRBD MPC](https://github.com/kairoi-k/go2-mujoco-control/blob/f3b452d56b2bedd5ea02249d4e5087b6ca151c47/example/cpp/wbc/srbd_mpc.h)
1–文件结尾；内部 nominal reference 积分、力 QP 与输出字段。
Git blob SHA：`33c579c8e04d3a9f6b0b3bf5aca261316698655e`。

## R15
[真正的 B1 分析器](https://github.com/kairoi-k/go2-mujoco-control/blob/f3b452d56b2bedd5ea02249d4e5087b6ca151c47/example/cpp/tools/analyze_phase2_terrain.py)
430–550 行；transfer_mpc_support 与 single_vcmd_authority 等检查。
Git blob SHA：`6d808965eaf19f9a49421ecfaabdbd534106cff3`。

## R16
[代码导航](https://github.com/kairoi-k/go2-mujoco-control/blob/f3b452d56b2bedd5ea02249d4e5087b6ca151c47/docs/CODE_GUIDE.md)
全文；仅用作代码定位。
Git blob SHA：`13aea830c35a0131f59a106534247adbd46d1076`。

## P01
[Jenelten et al., TAMOLS: Terrain-Aware Motion Optimization for Legged Systems (2022)](https://arxiv.org/abs/2206.14049)
摘要与 PDF 方法正文；不以未成功截图的实验表格作精确数据证据。

## P02
[Grandia et al., Perceptive Locomotion through Nonlinear Model Predictive Control (2022)](https://arxiv.org/abs/2208.08373)
摘要、方法/局限正文；PDF 第15页 Table IV 已截图核验。

## P03
[Kim et al., Highly Dynamic Quadruped Locomotion via Whole-Body Impulse Control and Model Predictive Control (2019)](https://arxiv.org/abs/1909.06586)
摘要与架构部分。

## P04
[Corbères et al., Perceptive Locomotion through Whole-Body MPC and Optimal Region Selection (2023)](https://arxiv.org/abs/2305.08926)
摘要；不援引未核实的硬件时间细节。

## P05
[Aceituno-Cabezas et al., Simultaneous Contact, Gait and Motion Planning ... Mixed-Integer Convex Optimization](https://arxiv.org/abs/1904.04595)
摘要；作为混合整数路线对照，不声称完成代码复现。

## P06
[Meduri et al., BiConMP: A Nonlinear Model Predictive Control Framework for Whole Body Motion Planning (2022)](https://arxiv.org/abs/2201.07601)
作者摘要；作为分解求解路线对照。

## P07
[Whole-Body MPPI, ICRA 2025, authors project page](https://whole-body-mppi.github.io/)
作者项目页；近期采样式全身控制路线扫描。

## P08
[Yang et al., Towards Terrain-Aware Safe Locomotion ... Proprioceptive Sensing (2026)](https://arxiv.org/abs/2603.09585)
作者摘要；感知/接触联合估计路线扫描，不直接移植安全保证。

## P09
[Impact-Aware Robust Convex Model Predictive Control ... (IEEE RA-L, 2026)](https://doi.org/10.1109/LRA.2026.3692086)
期刊摘要；Go2 上的冲击鲁棒性方向，不作为当前 joint planner 的现成解。

## S01
[OSQP official code generation](https://osqp.org/docs/codegen/index.html)
固定维度、固定稀疏模式的无动态分配 C 代码；不保证本项目时限。

## S02
[acados official documentation](https://docs.acados.org/)
SQP / real-time iteration 路线的工具备选；本轮未集成。

## 不可访问的证据

对固定提交下 `example/cpp/experiments/_runs/phase2_h8_shadow_fixed_e6253fd_flat_lockstep` 的 GitHub contents 请求返回 404。`docs/research/evidence/` 当前列出的已提交目录没有 H8/H9 原始包。这与 AGENTS.md 中 `_runs` 是忽略的本地证据相符，不证明本地文件不存在，也不授权删除它们。H9 的比例和生命周期时间只复核了提交报告与源码逻辑，不能冒充独立重算原始数据。

新 agent 应读取 Atlas 已有目录，以只读方式导出小型完整输入样本、相关时间段、分析命令和哈希到新的 `docs/research/evidence/stage_c_.../`；缺字段补采新样本并标明来源，不能反造旧 H5 坐标。
