# Phase1 residual velocity attribution（B semantic，离线）

本轮只读取 B semantic source f87cf929ddbb124a85464ce2863a4584b4270e2c 的既有 raw；没有启动 MuJoCo、修改 controller、参数或验收口径。重点为 varying 1.4→2.3 与 steps 1→2；0.6→1.4、2.3→2.8 仅作参考。

## 方法

t 从每个 run 第一个 active continuous-trot cmd_time 对齐。target_time 是 profile 到达新平台的时刻：varying 1.4→2.3 为 32 s，steps 1→2 为 32 s，参考 transition 为 16 s 与 48 s。kernel nominal 按 B runtime 的 step/period 重建，等于 applied。
realized_ax 使用 measured 的过去 100 ms 局部线性回归斜率，不使用逐帧差分。事件要求条件连续约 100 ms；WBC/SRBD 负加速度阈值为 −0.05 m/s²，ID-WBC 减速方向阈值为净 x 接触力 < −1 N，realized_ax 阈值为 < −0.05 m/s²。每 100 ms 的完整对齐 trace、事件与窗口幅值均在 attribution.csv。
Raibert source 确认 nominal touchdown = 0.5·step·duty，greedy correction = clamp(0.20·(measured−nominal), ±0.025 m)。但 runtime preview_horizon=4 会先经过 Footstep MPC；raw 未记录 greedy adjustment/preview QP adjustment，因此 CSV 的 foothold_adjustment 是实际 touchdown−nominal touchdown，不能无歧义拆成纯 Raibert correction，已标为观测缺口。
力符号按源码核实：SRBD/allocator 的 contact force 是施加于机体的 world force，+x 为前向、−x 为减速；ID-WBC 使用 Mqdd+h=Sᵀτ+Jᵀf，输出 tau=Mj·qdd+hj−Jjᵀf。

## high-speed mode

runtime BuildGaitTargets 关闭 effective-speed convention，并把 kernel nominal 覆盖为 applied；UpdateWbcFull 独立按 abs(kernel nominal)>1.25 激活 high-speed WBC。该 mode 将 MPC refresh 从 25 ticks 改为 5 ticks，并以 clamp(8·(nominal−measured), ±3) 覆盖 desired x acceleration。varying 1.4→2.3 从起点即在该 mode；steps 1→2 的切换是跨阈值混杂。源码锚点：raibert_trot_kernel.h、preview_footstep_horizon.h、trot_experiment_wbc.cpp、srbd_mpc.h、inverse_dynamics_wbc.h。

## 事件时间（active-relative t；相对 target_time 的 delta 也在 CSV）

|transition/run|mode switch|measured>target|>target+tolerance|governor|nominal<measured|WBC ax<0|SRBD ax<0|ID force<0|realized ax<0|
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
|varying_14to23/varying_20260913_154656|14.500|32.000|32.000|32.020|32.000|32.000|32.140|—|32.394|
|varying_14to23/varying_20260913_154848|14.500|32.000|32.000|32.032|32.000|32.000|32.000|—|33.824|
|varying_14to23/varying_20260913_155236|14.500|32.002|32.002|32.002|32.002|32.002|32.140|—|32.398|
|steps_1to2/steps_20260913_153832|26.002|32.001|32.001|32.161|32.001|32.001|32.039|—|35.365|
|steps_1to2/steps_20260913_154036|26.002|32.002|32.002|32.086|32.002|32.002|32.280|—|37.730|
|steps_1to2/steps_20260913_154450|26.000|32.002|32.002|32.220|32.002|32.002|32.560|—|32.838|
|varying_06to14/varying_20260913_154656|14.500|16.000|22.978|—|16.000|16.000|16.040|—|—|
|varying_06to14/varying_20260913_154848|14.500|16.000|—|—|16.000|16.000|16.078|—|—|
|varying_06to14/varying_20260913_155236|14.500|16.000|20.734|—|16.000|16.000|16.040|—|—|
|varying_23to28/varying_20260913_154656|14.500|48.000|48.046|48.258|48.000|48.000|48.240|—|48.658|
|varying_23to28/varying_20260913_154848|14.500|48.000|48.000|48.000|48.000|48.000|48.248|—|49.362|
|varying_23to28/varying_20260913_155236|14.500|48.002|48.002|48.002|48.002|48.002|48.380|—|48.622|

## 事件延迟汇总（相对 target_time；三次 run median）

|transition|measured>target|governor|WBC ax<0|SRBD ax<0|ID force<0|realized ax<0|
|---|---:|---:|---:|---:|---:|---:|
|varying_14to23|0.000|0.020|0.000|0.140|—|0.398|
|steps_1to2|0.002|0.161|0.002|0.280|—|3.365|

## 主要层级幅值（target 后 1 s；每 run）

|transition/run|measured error|foothold adj m/step|WBC ax median/min|SRBD ax median/min|ID force median/min|realized ax median/min|ID decel frac|realized neg frac|
|---|---:|---:|---:|---:|---:|---:|---:|---:|
|varying_14to23/varying_20260913_154656|0.225|0.016/0.052|-2.493/-2.976|-1.539/-5.092|1.880/-38.925|0.127/-0.677|0.368|0.298|
|varying_14to23/varying_20260913_154848|0.226|0.017/0.053|-2.512/-2.983|-1.685/-5.385|0.686/-42.863|0.040/-0.460|0.406|0.334|
|varying_14to23/varying_20260913_155236|0.224|0.017/0.053|-2.479/-3.195|-1.519/-5.167|1.284/-37.090|0.078/-0.446|0.387|0.299|
|steps_1to2/steps_20260913_153832|0.209|0.015/0.053|-2.188/-2.621|-1.445/-4.286|0.866/-19.894|0.033/-0.328|0.364|0.322|
|steps_1to2/steps_20260913_154036|0.210|0.015/0.054|-2.209/-2.640|-1.618/-4.605|0.823/-25.333|0.048/-0.368|0.373|0.291|
|steps_1to2/steps_20260913_154450|0.204|0.014/0.051|-2.072/-2.647|-1.499/-4.484|2.544/-27.054|0.040/-0.473|0.324|0.334|

## 归因结论

varying 1.4→2.3 的三次一致显示：measured/nominal 超 target 在 target_time 附近，governor 随后约 0.020 s 才介入；WBC requested ax 约 0.000 s、SRBD ax 约 0.140 s 内已为负，但 100 ms 持续的 ID-WBC 减速方向事件三次均未出现。target 后 1 s，ID force median 为 1.880, 0.686, 1.284 N，realized_ax median 为 0.127, 0.040, 0.078 m/s²；realized negative 只在更晚且不稳定地出现。第一处明显丢失的上游目标是 ID-WBC/contact-force realization 层，证据强度：中高；尚不能单凭该标量把问题进一步拆成 allocator、contact topology 或 plant。

steps 1→2 也有同方向顺序：mode switch median 为 -5.998 s（早于 target），WBC/SRBD 先给出负加速度，ID force 未形成持续负向，realized_ax median 延迟约 3.365 s。由于它跨越 1.25 threshold，归因置信度低于 varying 主证据，但不支持“仅 mode switch 即为首因”。

分层判断：A Raibert/foothold authority 不是首个明显失效层，但纯 preview/greedy 修正仍不可分离，证据中等；B SRBD authority 首因不支持；C ID-WBC/contact-force realization 为首个明显失效候选，证据中高；D plant/contact execution 仍与 C 耦合，尚不能单独定案；E 物理根因仍可能是分布式 coupling。

因此本轮结论为：主 transition 的最早可观测执行层失效候选是 ID-WBC/contact-force realization；plant-only 情形尚未被排除，属于后续分层验证项。Raibert 只能报告实际 touchdown−nominal 的有效修正，纯 preview/greedy authority 仍是观测缺口。

下一步只提出一个未执行的单变量 A/B：不改变 gait semantic、period/duty、governor、Raibert gain、WBC/MPC 参数和模型，只验证 SRBD desired x acceleration 到 ID-WBC 实际净 x contact force 的传递/接触 topology；本轮不执行。

Raw data.csv、run_manifest.json、run_metadata.txt 及 controller/simulator/scene SHA256 在 attribution.csv provenance 行；raw 未复制进 Git。
