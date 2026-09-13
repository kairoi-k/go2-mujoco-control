# Phase1 D duty=0.50 单变量结果

目的：在 B semantic、period、governor、接触模型与其余参数不变时，仅测试 runtime duty 0.44→0.50。B baseline 使用上一轮 varying 三次 raw；D 新跑 varying 三次；没有新增 3 m/s 或其他 duty/period 实验。

## Source 与精确 diff

B semantic source：f87cf929ddbb124a85464ce2863a4584b4270e2c；D 实验 source：b0c5748cd1dd07308ce8e0d1498be9e95c8e11a8。D 的唯一 controller diff 是 ScheduleContinuousVelocityGait() 中 schedule.duty_factor 0.44→0.50，见 DUTY050_SOURCE_DIFF.patch。wrapper argv 仍显示 --duty 0.44，但 raw 的 velocity_command_gait_duty 已逐样本验证为 0.50；period=0.14，step=applied*period。

## 运行命令

D 三次：TROT_CPU_AUTOPIN=1 bash example/cpp/scripts/run_phase1_velocity_benchmark.sh varying _runs/phase1_velocity_duty050_20260913/D 210、211、212（串行调用）；B 三次复用 varying_20260913_154656、154848、155236，不重跑。所有 run 的 data/manifest/metadata SHA256、source/controller/simulator/scene SHA256 与状态见 duty050_summary.csv provenance 行。

## 平台稳态（末 4 s；三次 repeat median，p05/p95 为 repeat envelope）

|target|B measured p05/med/p95|D measured p05/med/p95|B abs err|D abs err|B applied|D applied|B overspeed peak|D overspeed peak|B gov frac|D gov frac|B gov gap max|D gov gap max|
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
|0.6|0.489/0.525/0.550|0.515/0.567/0.661|0.075|0.042|0.600|0.600|-0.039|0.072|0.000|0.000|0.000|0.000|
|1.4|1.525/1.548/1.569|1.540/1.566/1.594|0.148|0.166|1.400|1.400|0.188|0.207|0.000|0.004|0.000|0.007|
|2.3|2.493/2.525/2.550|2.530/2.557/2.577|0.225|0.257|2.275|2.243|0.262|0.293|0.940|1.000|0.062|0.093|
|2.8|2.841/3.017/3.048|2.908/3.015/3.052|0.217|0.215|2.778|2.778|0.268|0.260|0.833|0.802|0.217|0.145|

roll/pitch/torque/contact/solver 平台 pooled：
|target|B roll p95/D|B pitch p95/D|B torque max/D|B contact-loss/D|B single-contact/D|B plan/SRBD/ID/solver|D plan/SRBD/ID/solver|
|---:|---:|---:|---:|---:|---:|---:|
|0.6|0.746/0.636|0.434/0.626|23.828/21.300|0.000/0.000|0.000/0.002|1.000/1.000/1.000/1.000|1.000/1.000/1.000/1.000|
|1.4|0.759/0.926|1.262/1.435|35.890/45.430|0.118/0.083|0.278/0.278|1.000/1.000/1.000/1.000|1.000/1.000/1.000/1.000|
|2.3|1.340/1.366|1.516/1.550|45.430/45.430|0.251/0.194|0.390/0.343|1.000/1.000/1.000/1.000|1.000/1.000/1.000/1.000|
|2.8|1.537/2.069|1.321/1.475|45.430/45.430|0.283/0.238|0.445/0.458|1.000/1.000/1.000/1.000|1.000/1.000/1.000/1.000|

## 上升 transition（完整 1 s settling audit）

|transition|B success/latency|D success/latency|B overspeed peak|D overspeed peak|
|---|---|---|---:|---:|
|0.6→1.4|0/3 (—,—,—)|0/3 (—,—,—)|0.188|0.219|
|1.4→2.3|0/3 (—,—,—)|0/3 (—,—,—)|0.272|0.298|
|2.3→2.8|1/3 (3.308,—,—)|1/3 (—,—,5.108)|0.286|0.305|

## 运行状态与结论

B varying_20260913_154656：runtime period=0.140 s, duty=0.440, roll/pitch max=3.645/3.507 deg, torque max=45.430 N·m, solver/SRBD/ID=1.000/1.000/1.000, status controller/safety/quality/analysis=0/0/0/0.
B varying_20260913_154848：runtime period=0.140 s, duty=0.440, roll/pitch max=3.380/3.425 deg, torque max=45.430 N·m, solver/SRBD/ID=1.000/1.000/1.000, status controller/safety/quality/analysis=0/0/0/0.
B varying_20260913_155236：runtime period=0.140 s, duty=0.440, roll/pitch max=2.834/3.502 deg, torque max=45.430 N·m, solver/SRBD/ID=1.000/1.000/1.000, status controller/safety/quality/analysis=0/0/0/0.
D varying_20260913_165410：runtime period=0.140 s, duty=0.500, roll/pitch max=3.035/3.509 deg, torque max=45.430 N·m, solver/SRBD/ID=1.000/1.000/1.000, status controller/safety/quality/analysis=0/0/0/0.
D varying_20260913_165602：runtime period=0.140 s, duty=0.500, roll/pitch max=3.425/3.500 deg, torque max=45.430 N·m, solver/SRBD/ID=1.000/1.000/1.000, status controller/safety/quality/analysis=0/0/0/0.
D varying_20260913_165754：runtime period=0.140 s, duty=0.500, roll/pitch max=2.934/3.512 deg, torque max=45.430 N·m, solver/SRBD/ID=1.000/1.000/1.000, status controller/safety/quality/analysis=0/0/0/0.

D 结论：中高速 pooled target absolute error 相对 B 的变化为 1.4:+0.017 m/s、2.3:+0.032 m/s、2.8:-0.002 m/s；1.4→2.3 仍为 0/3，2.3→2.8 未形成一致 settling 改善，overspeed peak 也未系统性下降。0.6 仅作记录，不以改善为要求。

跨三次重复看，duty=0.50 未显示中高速 residual overspeed 的系统性下降，因此本轮不支持把 duty/contact timing 作为已确认的主要解释。D 未出现新的 hard safety failure，姿态、torque、solver 状态未见明显恶化；但 contact-loss/single-contact 仍是原有 gait quality 风险，不能当作通过依据。

本轮到此停止：不继续扫描 duty，不再改 period/governor/WBC/Raibert/contact 参数。
