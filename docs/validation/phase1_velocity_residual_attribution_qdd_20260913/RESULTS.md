# Phase1 residual velocity attribution: qdd/contact split (B semantic)

本轮只读取既有 B raw，source `f87cf929ddbb124a85464ce2863a4584b4270e2c`；没有启动 MuJoCo、修改 controller、参数或验收口径。raw 的 data.csv、run_manifest.json、run_metadata.txt SHA256 及逐 100 ms trace 均在 `attribution_qdd.csv`。

## 证据与方法

分析对象为 varying 1.4→2.3（3 runs，target endpoint t=32 s）和 steps 1→2（3 runs，t=32 s）。每个 run 取 endpoint 后 [0,1) s；realized_ax 沿用上一轮定义，用 measured speed 的过去 100 ms 局部线性回归斜率。负向持续事件要求连续约 100 ms；WBC/SRBD/ID-qdd 阈值为 <−0.05 m/s²，ID 净接触 Fx 阈值为 <−1 N。

源码确认：`go2_rigid_body.h` 将 MuJoCo free-joint qvel[0:3] 写入 world linear velocity；`inverse_dynamics_wbc.h` 的 `IdWbcOutput::qdd` 前 6 项是 floating-base generalized acceleration，`a_des` 先放 world linear acceleration，且脚 Jacobian 是 `foot_jac_world`。因此 qdd[0] 是 world-frame floating-base x acceleration；本批 forward run 的 measured 是 world x speed 的正方向标量，坐标语义可直接比较，不需要转换。它仍是 ID-WBC 解变量，不是 plant 实测加速度。

同一源码中的浮动基等式为 `M qdd + h = Sᵀτ + Jᵀf`，`wbc_out.force[3*leg]` 逐接触脚求和并记录为 `wbc_full_id_contact_force_x_n`；因此 +x 是前向接触作用，−x 是减速方向。该 Fx 与 qdd 不是无质量/无耦合时的等式替代，以下符号一致性只作 realization 诊断。另有源码路径允许在求解后改写 force 而不重算 qdd；本批 metadata 未启用 direct-force overlay，但这使 qdd/Fx 必须按两个输出分别审计。

## 1 s 汇总（跨三次 raw 的 pooled p05/median/p95；逐 run 数值在 CSV）

|transition|signal|p05|median|p95|
|---|---:|---:|---:|---:|
|steps_1to2|WBC desired ax|-2.501|-2.169|-1.897|
|steps_1to2|SRBD ax|-4.123|-1.471|0.000|
|steps_1to2|ID qdd_x|-2.931|-2.125|-0.914|
|steps_1to2|ID net Fx|-15.889|1.307|17.398|
|steps_1to2|realized ax|-0.315|0.044|0.266|
|steps_1to2|WBC−ID qdd|-1.507|-0.032|0.785|
|steps_1to2|SRBD−ID qdd|-2.366|0.706|2.514|
|steps_1to2|ID qdd−realized|-2.955|-2.173|-0.915|
|varying_14to23|WBC desired ax|-2.852|-2.497|-1.958|
|varying_14to23|SRBD ax|-4.493|-1.582|0.000|
|varying_14to23|ID qdd_x|-3.320|-2.223|-0.932|
|varying_14to23|ID net Fx|-18.443|1.282|19.115|
|varying_14to23|realized ax|-0.407|0.068|0.467|
|varying_14to23|WBC−ID qdd|-1.645|-0.245|0.853|
|varying_14to23|SRBD−ID qdd|-2.445|0.721|2.436|
|varying_14to23|ID qdd−realized|-3.449|-2.265|-0.884|

接触/状态：两组 1 s pooled 的 physical contact_count 中位数均为 2；逐样本 WBC/SRBD/ID/plan 状态与 contact mask、torque proxy 在 CSV。`torque_est_sat_fraction` 是 raw joint tau_est 达到 34.9 Nm 的诊断 proxy，不把它误称为硬饱和标志。

## 事件时序（每 run；dt 相对 target endpoint）

|transition/run|measured>target|WBC ax<0|SRBD ax<0|ID qdd_x<0|ID Fx<0|realized ax<0|
|---|---:|---:|---:|---:|---:|---:|
|steps_1to2 / steps_20260913_153832|30.625|32.001|32.039|32.209|—|—|
|steps_1to2 / steps_20260913_154036|30.746|32.002|32.280|32.002|—|—|
|steps_1to2 / steps_20260913_154450|30.744|32.002|32.560|32.002|—|32.838|
|varying_14to23 / varying_20260913_154656|30.196|32.000|32.140|32.070|—|32.394|
|varying_14to23 / varying_20260913_154848|30.272|32.000|32.000|32.068|—|—|
|varying_14to23 / varying_20260913_155236|30.414|32.002|32.140|32.002|—|32.398|

## 归因

varying 1.4→2.3 是主证据：三次中 ID qdd_x 在 endpoint 后约 100 ms 内持续为负，1 s pooled median 约 −2.22 m/s²；但 ID net Fx median 约 +1.28 N，p05/p95 约 −18.7/+19.1 N，三次均没有持续 100 ms 的负 Fx 事件，Fx 与 qdd 的符号一致性仅约 0.42–0.47。realized ax median 约 +0.08 m/s²，且负向事件延迟约 0.4–1.1 s、并非稳定跟随。故 SRBD→ID-WBC 的 qdd 优化输出本身已形成减速方向；第一处稳定偏离更具体地落在 ID-WBC qdd 与记录到的净接触力/接触 realization 不一致，支持 B，证据中高。

steps 1→2 的 qdd 同样快速持续为负（pooled median 约 −2.13 m/s²），Fx 中位数约 +1.39 N、无持续负 Fx，realized ax 更晚；但该 transition 在 target 前约 6 s 已进入 high-speed mode，因此作为次证据置信度中等，不能把 threshold switch 单独定为首因。

C（plant/contact execution）不能单独成立：ID qdd 已负但 Fx 未形成相应净减速，因此更早已存在 B；不过 Fx 的物理接触映射与 plant 实现仍耦合，现有标量 raw 不能拆分 allocator/contact topology 与真实接触执行。最终分类：主 transition = `ID solution/contact-force realization mismatch`，并带 `distributed coupling` 残余；不是“纯 SRBD authority”，也不是已证明的纯 plant-only bottleneck。

下一步只提出一个未执行的最小单变量验证：固定当前 semantic、period/duty、governor、Raibert、SRBD/WBC gains 和模型，增加/核对同一 tick 的 ID-WBC base-equation closure 与每接触脚 force/Jᵀf 映射证据，区分 qdd 保持而 Fx 被重写、接触约束投影失真，还是物理接触未实现；本轮不执行。

源码锚点：`example/cpp/kinematics/go2_rigid_body.h`、`example/cpp/wbc/inverse_dynamics_wbc.h`、`example/cpp/trot/trot_experiment_wbc.cpp`。
