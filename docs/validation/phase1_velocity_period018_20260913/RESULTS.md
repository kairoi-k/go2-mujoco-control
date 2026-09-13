# Phase1 period 0.18 single-variable probe

基准为已确认的 B semantic：`step=applied*period`、effective-speed convention=false、duty=0.44，B source：`f87cf929ddbb124a85464ce2863a4584b4270e2c`。B baseline 直接复用上一轮 varying 三次；C 仅把 runtime schedule period 从 0.14 改为 0.18，其他控制参数、profile、governor、WBC、Raibert、模型和验收口径不变。C source commit：`982c893a88f756ff88629d195be7f1b1e20ea91e`。

## 精确 diff

```diff
-    schedule.period_s = 0.14;
+    schedule.period_s = 0.18;
```

## 运行与 raw SHA256

C 三次尝试使用同一 wrapper；C runtime data 的 period=0.18、duty=0.44。原始 data/manifest/metadata/controller/simulator/scene SHA 在 `period018_summary.csv` 的 provenance 行。

|组|run|period/duty|controller|safety|completion|data.csv SHA256|
|---|---|---|---:|---:|---:|---|
|B|`varying_20260913_154656`|0.140/0.440|0|0|0|`f668150f2e3fcb94e180327f3b5a1d6d98d4af33a6af2b039063680f1e1d453b`|
|B|`varying_20260913_154848`|0.140/0.440|0|0|0|`bc81a46d60de10b22313285cd7fb506a2b27b462705b483a9d101a0f149a3d3d`|
|B|`varying_20260913_155236`|0.140/0.440|0|0|0|`bfe530aa3d7890a80e2db7b520dc97bd194d70a3e1a72a89e41e388484365b71`|
|C|`varying_20260913_163158`|0.180/0.440|0|0|0|`4f3f632f85a4c22fe444b9933c35897025e10414b7e9b01f3aaeadfbf1f42825`|
|C|`varying_20260913_163351`|0.180/0.440|0|1|1|`e02abf27025124f77027903c7076649896096366606d9477bcc528bc9870212c`|
|C|`varying_20260913_163502`|0.180/0.440|0|1|1|`b30cd1609b7cbe5bc2684fb03dc931b49c6d8e49eb9d51f8c773e7e0a39d1458`|

实际执行命令：

```text
TROT_CPU_AUTOPIN=1 bash example/cpp/scripts/run_phase1_velocity_benchmark.sh varying _runs/phase1_velocity_semantic_ab_20260913/B 213
TROT_CPU_AUTOPIN=1 bash example/cpp/scripts/run_phase1_velocity_benchmark.sh varying _runs/phase1_velocity_semantic_ab_20260913/B 214
TROT_CPU_AUTOPIN=1 bash example/cpp/scripts/run_phase1_velocity_benchmark.sh varying _runs/phase1_velocity_semantic_ab_20260913/B 215
TROT_CPU_AUTOPIN=1 bash example/cpp/scripts/run_phase1_velocity_benchmark.sh varying _runs/phase1_velocity_period018_20260913/C 210
TROT_CPU_AUTOPIN=1 bash example/cpp/scripts/run_phase1_velocity_benchmark.sh varying _runs/phase1_velocity_period018_20260913/C 211
TROT_CPU_AUTOPIN=1 bash example/cpp/scripts/run_phase1_velocity_benchmark.sh varying _runs/phase1_velocity_period018_20260913/C 212
```

## 常值平台（完整窗口）

窗口为各 8s platform 的末 4s；C safety stop 的两次 run 在 2.8m/s 平台前结束，故 C 2.8 只报告 1/3 完整窗口，不能视为三次稳态重复。

|target|B n p05/median/p95|C n p05/median/p95|B target abs err|C target abs err|
|---:|---:|---:|---:|---:|
|0.6|3 0.489/0.525/0.550|3 0.446/0.492/0.562|0.075|0.108|
|1.4|3 1.525/1.548/1.569|3 1.499/1.517/1.552|0.148|0.117|
|2.3|3 2.493/2.525/2.550|3 2.276/2.535/2.594|0.225|0.235|
|2.8|3 2.841/3.017/3.048|1 2.321/2.741/2.860|0.217|0.068|

B→C target error：0.6m/s `0.075→0.108`；1.4m/s `0.148→0.117`；2.3m/s `0.225→0.235`；2.8m/s `0.217→0.068`（2.8 的 C 仅 n=1）。

## 上升 transition 1s settling

C 的 safety stop 使不可覆盖的 transition 标为 unavailable，而不是失败；B 三次均完整。

|transition|B success/available; latency|C success/available; latency|
|---|---|---|
|0.6→1.4|0/3; —,—,—|1/3; 5.112,—,—|
|1.4→2.3|0/3; —,—,—|0/3; —,—,—|
|2.3→2.8|1/3; 3.308,—,—|1/1; 1.866|

## Safety / execution check

|组|run roll max°|run pitch max°|run torque max N·m|solver ok|status|
|---|---:|---:|---:|---:|---|
|B|3.645|3.507|45.430|1.000|controller=0, safety=0, completion=0|
|B|3.380|3.425|45.430|1.000|controller=0, safety=0, completion=0|
|B|2.834|3.502|45.430|1.000|controller=0, safety=0, completion=0|
|C|16.946|9.186|126.525|1.000|controller=0, safety=0, completion=0|
|C|180.000|69.095|967.950|1.000|controller=0, safety=1, completion=1|
|C|180.000|79.146|343.881|1.000|controller=0, safety=1, completion=1|

C 后两次日志均命中 hard posture safety limit（roll 接近 ±180°）；其 post-failure 峰值不作为稳态 torque 估计，但说明存在新的明显安全/姿态退化。solver/plan/SRBD/ID 状态在已记录 rows 中仍为 1.0，未能抵消 safety failure。

## 结论

period 0.14→0.18 改变了部分完整窗口的 observed steady-state bias，但方向不一致，且 C 出现 2/3 safety/motion rejection；因此现有数据不足以证明 cadence 单独造成 residual bias 的系统性变化。它已经被证明是稳定性敏感变量：首轮即出现显著 roll/pitch/torque 峰值，后两次命中 hard posture limit，所以 period=0.18 不是当前配置的可接受单变量改进。停止继续追 period，下一步若获批转查 duty/contact/WBC interaction，本轮不执行。
