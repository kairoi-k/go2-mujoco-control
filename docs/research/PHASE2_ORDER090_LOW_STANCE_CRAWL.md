# Phase 2 Order-090：v2 低身姿四足蠕动设计

状态：实现前设计，2026-08-31。本文只描述已授权的 v2 transfer-window pivot；不改变 analyzer、canary 或验收合约定义。

## 合法性与边界

PHASE2_B123_ACCEPTANCE_CONTRACT_V2.md 的 V2-A 明确允许 planner 宣布 terrain transfer active 后，在窗口内从 0.30 m/s 受控减速到不低于 0.05 m/s 的 crawl profile，并要求窗口完成后 1.0 s 内恢复脚本速度。V2-B 允许窗口内由 running-trot 切换为准静态 crawl，任意时刻保持至少 3 个接触；同一条款保留 planned/measured 一致、实测力支撑、零碰撞和完成性要求。因此本设计只在现有 v2 window gate 且 sequencer 已取得 authority 时生效；flat、v1 与窗口外路径不变。

## 低身姿参数

* **机身高度目标**：窗口内 MPC nominal base-height 目标为 0.34 m（相对冻结 Phase-1 0.42 m 降低 0.08 m）；只替换 terrain sequencer authority 的高度参考，RESUME 不再使用该覆盖。
* **COM 伺服**：窗口内测量支撑多边形 incenter 仍是唯一 XY 目标，但使用 w_pos_xy=420、w_vel_xy=24（相对当前 v2 shift 的 300/20），并使用低姿态高度 PD Kp=480、Kd=72。这提高 COM 对宽支撑域中心的跟随，同时不引入第二个 balance controller。
* **更宽的支撑**：crawl gait 的 lateral stance offset 以足端当前 body-frame y 的符号向外增加 0.025 m；已测量/冻结的 support anchor 不重写，故不会偷偷移动实测接触点。该偏置仅用于下一次非冻结 nominal crawl target。
* **更小步幅/抬腿**：v2 crawl 的 period=0.60 s、duty=0.85，步幅由速度与该 schedule 计算并封顶 0.035 m；抬腿 0.025 m，terrain sequencer 的 raised-foot apex 为 FL 0.035 m、其余 0.045 m。实际 terrain endpoint 仍由 lidar 测量目标和既有 commit witness 决定。
* **sequencer 内部余量**：低姿态 SHIFT 的内部 measured support-margin release 为 0.010 m，相对于原内部 0.000 m release 增加 10 mm；力阈值仍为 10 N/总力 50 N/最大最小比 4。该值只存在于 sequencer 的 v2 分支；合约和 analyzer 的阈值不动，且全程 measured contacts >=3 仍是硬不变式。

## RESUME 恢复

CLEAR 后 sequencer 进入 RESUME 时立即撤销低姿态覆盖：MPC 回到冻结 Phase-1 0.42 m 与原始 80/8 高度伺服，gait schedule 回到 Phase-1 profile；继续保留现有 0.45 s stable dwell 和窗口外 V2-A 的 1.0 s 恢复约束。若 transfer abort，则同样撤销低姿态覆盖并走既有安全停止，不把低姿态参数泄漏到 flat/v1。

## 验证顺序

先以 staged harness 连续取得至少 3 次完整序列，再按 epoch328--331、每 epoch 两次、串行锁 /tmp/go2_mujoco_experiment.lock 的 4 对探针验证 >=6/8 完整序列；达标后才执行 10 对穿越+确认战役。任何探针阶段若一次完整序列都没有，立即停止，不将结果解释为 B1 通过。
