# Phase1 runtime gait-speed semantic A/B

离线汇总使用的 B source checkpoint：`f87cf929ddbb124a85464ce2863a4584b4270e2c`。基准 controller lineage：`a1d4e294092a39c8e649eda6f285ea2cfe01b05d`；A 为原行为，B 仅改变 step 映射并关闭 runtime effective-speed convention，未改其他 controller/参数/profile/阈值/模型。

## 实验定义

A：`step=applied*period/(2*duty)`，effective-speed convention=true。B：`step=applied*period`，effective-speed convention=false。两组均固定 period=0.14、duty=0.44、profile、shaper、governor、Raibert、preview、SRBD/WBC、接触、MuJoCo model、环境变量和验收口径。

B 的精确 source diff：
```diff
- schedule.step_length_m = speed * schedule.period_s / std::max(0.20, 2.0 * schedule.duty_factor);
+ schedule.step_length_m = speed * schedule.period_s;
- locomotion_kernel_->SetGaitEffectiveSpeedConvention(true);
+ locomotion_kernel_->SetGaitEffectiveSpeedConvention(false);
- params_.wbc_full && (high_speed_curriculum || runtime_velocity_command));
+ params_.wbc_full && high_speed_curriculum);
```
完整 patch 见 `PHASE1_VELOCITY_SEMANTIC_AB_DIFF.patch`；CSV 为本报告全部定量结论的可复核汇总。

## 运行与 raw SHA256

两组各执行 steps×3、varying×3；raw 由同一 benchmark wrapper 生成，完整 1 s settling audit 在本脚本离线重算。A raw 位于独立 detached worktree，B raw 位于当前 worktree。原始目录不入 Git，逐 run 的 data.csv、manifest、metadata SHA256 和 binary/model SHA256 已写入 `ab_summary.csv` 的 provenance 行。

|组|场景|run|controller SHA|data.csv SHA256|状态|
|---|---|---|---|---|---|
|A|steps|`steps_20260913_152621`|`dc3b12505a4f74b2c96c3f3f69ac5c5d75351ea6`|`df31240110aaafe848b8d02b9417129ce7b221b6bf7c1d4be681620ea2cdae4c`|controller=0, safety=0, quality=0, analysis=0|
|A|steps|`steps_20260913_152830`|`dc3b12505a4f74b2c96c3f3f69ac5c5d75351ea6`|`4119a270df4e3e1b956d65df391d8cf6e08eb9d0dbf21c965c70453516ccd943`|controller=0, safety=0, quality=0, analysis=0|
|A|steps|`steps_20260913_153038`|`dc3b12505a4f74b2c96c3f3f69ac5c5d75351ea6`|`b8f320f1384d12887b932ec30ea5c71f56d552b50c470a0b0e2a3dd820eea99b`|controller=0, safety=0, quality=0, analysis=0|
|A|varying|`varying_20260913_153246`|`dc3b12505a4f74b2c96c3f3f69ac5c5d75351ea6`|`2f7532fa2fa0c619c712597a42d8e66ac2264b2ca58a7ceb9acbf8f8158db42f`|controller=0, safety=0, quality=0, analysis=0|
|A|varying|`varying_20260913_153445`|`dc3b12505a4f74b2c96c3f3f69ac5c5d75351ea6`|`4fec06c391cc4f18cc990bb342f7f1f7a16b26edf312cb47a22c7d8e93f859c2`|controller=0, safety=0, quality=0, analysis=0|
|A|varying|`varying_20260913_153637`|`dc3b12505a4f74b2c96c3f3f69ac5c5d75351ea6`|`fb2f84605f7f79928d06ea1bf7205588f87f40f45e8273ea6038b635d2f73091`|controller=0, safety=0, quality=0, analysis=0|
|B|steps|`steps_20260913_153832`|`f87cf929ddbb124a85464ce2863a4584b4270e2c`|`20803709f4c610771b299e97485e061951ca44c1f15744558fddf06aa0957531`|controller=0, safety=0, quality=0, analysis=0|
|B|steps|`steps_20260913_154036`|`f87cf929ddbb124a85464ce2863a4584b4270e2c`|`27984673cf151ffc600b4360db16ed4b54aa272df32bae8cc5f82d57b318b5b8`|controller=0, safety=0, quality=0, analysis=0|
|B|steps|`steps_20260913_154450`|`f87cf929ddbb124a85464ce2863a4584b4270e2c`|`5973baed96fab03bcc6797a50c1fa74228375319474843a8115b95eb193862c0`|controller=0, safety=0, quality=0, analysis=0|
|B|varying|`varying_20260913_154656`|`f87cf929ddbb124a85464ce2863a4584b4270e2c`|`f668150f2e3fcb94e180327f3b5a1d6d98d4af33a6af2b039063680f1e1d453b`|controller=0, safety=0, quality=0, analysis=0|
|B|varying|`varying_20260913_154848`|`f87cf929ddbb124a85464ce2863a4584b4270e2c`|`bc81a46d60de10b22313285cd7fb506a2b27b462705b483a9d101a0f149a3d3d`|controller=0, safety=0, quality=0, analysis=0|
|B|varying|`varying_20260913_155236`|`f87cf929ddbb124a85464ce2863a4584b4270e2c`|`bfe530aa3d7890a80e2db7b520dc97bd194d70a3e1a72a89e41e388484365b71`|controller=0, safety=0, quality=0, analysis=0|

逐 run 命令（A/B 仅 binary/worktree 不同；其余参数由同一 wrapper 固定）：
```text
TROT_CPU_AUTOPIN=1 bash example/cpp/scripts/run_phase1_velocity_benchmark.sh steps _runs/phase1_velocity_semantic_ab_20260913/A 210
TROT_CPU_AUTOPIN=1 bash example/cpp/scripts/run_phase1_velocity_benchmark.sh steps _runs/phase1_velocity_semantic_ab_20260913/A 211
TROT_CPU_AUTOPIN=1 bash example/cpp/scripts/run_phase1_velocity_benchmark.sh steps _runs/phase1_velocity_semantic_ab_20260913/A 212
TROT_CPU_AUTOPIN=1 bash example/cpp/scripts/run_phase1_velocity_benchmark.sh varying _runs/phase1_velocity_semantic_ab_20260913/A 213
TROT_CPU_AUTOPIN=1 bash example/cpp/scripts/run_phase1_velocity_benchmark.sh varying _runs/phase1_velocity_semantic_ab_20260913/A 214
TROT_CPU_AUTOPIN=1 bash example/cpp/scripts/run_phase1_velocity_benchmark.sh varying _runs/phase1_velocity_semantic_ab_20260913/A 215
TROT_CPU_AUTOPIN=1 bash example/cpp/scripts/run_phase1_velocity_benchmark.sh steps _runs/phase1_velocity_semantic_ab_20260913/B 210
TROT_CPU_AUTOPIN=1 bash example/cpp/scripts/run_phase1_velocity_benchmark.sh steps _runs/phase1_velocity_semantic_ab_20260913/B 211
TROT_CPU_AUTOPIN=1 bash example/cpp/scripts/run_phase1_velocity_benchmark.sh steps _runs/phase1_velocity_semantic_ab_20260913/B 212
TROT_CPU_AUTOPIN=1 bash example/cpp/scripts/run_phase1_velocity_benchmark.sh varying _runs/phase1_velocity_semantic_ab_20260913/B 213
TROT_CPU_AUTOPIN=1 bash example/cpp/scripts/run_phase1_velocity_benchmark.sh varying _runs/phase1_velocity_semantic_ab_20260913/B 214
TROT_CPU_AUTOPIN=1 bash example/cpp/scripts/run_phase1_velocity_benchmark.sh varying _runs/phase1_velocity_semantic_ab_20260913/B 215
```

## 常值平台

窗口为每个平台末 4 s；表中 measured 为三次 repeat median，括号为三次 repeat 的 p05 最小值 / p95 最大值，target error 为 median absolute error。

|场景/target|A measured p05/med/p95|B measured p05/med/p95|A error|B error|A→B overspeed max|A→B governor gap max|
|---|---:|---:|---:|---:|---:|---:|
|steps 1|1.076/1.186/1.242|1.051/1.101/1.150|0.186|0.101|0.258→0.182|0.058→0.000|
|steps 2|2.252/2.300/2.329|2.186/2.211/2.226|0.300|0.211|0.342→0.237|0.142→0.037|
|steps 3|2.969/3.039/3.291|2.988/3.134/3.238|0.040|0.134|0.318→0.279|0.500→0.233|
|varying 0.6|0.534/0.569/0.613|0.489/0.525/0.550|0.031|0.075|0.050→-0.039|0.000→0.000|
|varying 1.4|1.610/1.633/1.666|1.525/1.548/1.569|0.233|0.148|0.291→0.188|0.091→0.000|
|varying 2.3|2.574/2.629/2.677|2.493/2.525/2.550|0.329|0.225|0.409→0.262|0.209→0.062|
|varying 2.8|2.976/3.106/3.201|2.841/3.017/3.048|0.306|0.217|0.423→0.268|0.300→0.217|

重点平台结论：
B 的 pooled repeat-median target error 在 steps 1 m/s, steps 2 m/s, varying 1.4 m/s, varying 2.3 m/s, varying 2.8 m/s 改善，在 steps 3 m/s, varying 0.6 m/s 变差；因此结果是速度相关的，不是全域单向获胜。

## Transition settling

完整 1 s audit 要求从 profile target endpoint 起，存在连续 1.0 s measured 误差不超过 `max(0.15, 0.05*max(|target|,1))` 的窗口；以下为每组成功数/3 与逐 run latency（秒）。

|场景 transition|A success / latency|B success / latency|
|---|---|---|
|steps 0→1|0/3; —,—,—|3/3; 0.554,0.000,0.000|
|steps 1→2|0/3; —,—,—|0/3; —,—,—|
|steps 2→3|2/3; 3.694,3.874,—|2/3; 1.201,—,0.002|
|steps 3→1|3/3; 0.546,0.554,0.112|3/3; 0.579,0.580,0.618|
|steps 1→0|3/3; 0.608,0.338,1.102|3/3; 0.443,0.634,1.186|
|varying 0.6→1.4|0/3; —,—,—|0/3; —,—,—|
|varying 1.4→2.3|0/3; —,—,—|0/3; —,—,—|
|varying 2.3→2.8|0/3; —,—,—|1/3; 3.308,—,—|
|varying 2.8→0.6|3/3; 0.698,0.702,0.702|3/3; 0.846,0.702,2.972|
|varying 0.6→0|3/3; 0.000,0.000,0.000|3/3; 0.000,0.000,0.000|

## 诊断性比较

|组|roll abs p95 median (deg)|pitch abs p95 median (deg)|torque max (N·m)|contact loss|single contact|plan/SRBD/ID/solver|
|---|---:|---:|---:|---:|---:|---:|
|A|1.617|1.638|45.430|0.224|0.413|1.000/1.000/1.000/1.000|
|B|1.340|1.262|45.430|0.238|0.373|1.000/1.000/1.000/1.000|

A/B 的 measured target error、overspeed、shaped→applied governor gap、roll/pitch、torque、contact/gait 质量以及 kernel/WBC/SRBD/solver fractions 全部在 `ab_summary.csv` 的 platform/transition 行中；所有 12 个成功 run 的 controller/safety/quality/analysis 状态均为 0，未见新的 safety failure。

`ab_platform_summary.svg` 展示七个平台两组 pooled measured median 与 p05–p95。该图和 CSV 是结果，不把 legacy `strict_pass` 当作 settling 全部成功。

## 结论边界

本实验只支持“B 语义统一对哪些速度平台的 settling 有何影响”的诊断结论；不自动调整 cadence、governor、WBC 或其他参数，也不把一次 A/B 结果提升为发布修复。若低中速改善而 3 m/s 退化，应保留为 speed-dependent gait scheduling 的后续研究信号。到此停止，等待人工批准。
