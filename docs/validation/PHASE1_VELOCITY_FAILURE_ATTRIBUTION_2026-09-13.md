# Phase1 velocity failure attribution

日期：2026-09-13。范围仅为 Phase1 velocity diagnosis；controller、控制参数、benchmark profile、验收阈值和物理模型均未修改。研究底座 controller SHA 为 `a1d4e294092a39c8e649eda6f285ea2cfe01b05d`，隔离工作树为 `research/phase1-velocity-failure-attribution-20260913`。运行使用基准树已有的 Phase1 `--wbc-full`（SRBD + ID-WBC）路径，没有 terrain、Phase2 crawl、Stage C、joint planner 或 whole-body MPC 执行参数。

复现命令（每条均原样调用既有 wrapper；仅 scenario、输出根目录和 DDS domain 不同）：

```text
bash example/cpp/scripts/run_phase1_velocity_benchmark.sh steps   _runs/phase1_velocity_failure_attribution_20260913 210
bash example/cpp/scripts/run_phase1_velocity_benchmark.sh steps   _runs/phase1_velocity_failure_attribution_20260913 211
bash example/cpp/scripts/run_phase1_velocity_benchmark.sh steps   _runs/phase1_velocity_failure_attribution_20260913 212
bash example/cpp/scripts/run_phase1_velocity_benchmark.sh varying _runs/phase1_velocity_failure_attribution_20260913 213
bash example/cpp/scripts/run_phase1_velocity_benchmark.sh varying _runs/phase1_velocity_failure_attribution_20260913 214
bash example/cpp/scripts/run_phase1_velocity_benchmark.sh varying _runs/phase1_velocity_failure_attribution_20260913 215
```

构建依赖是机器既有 MuJoCo 3.3.6 安装的只读链接。构建二进制 SHA-256：simulator `4b311183c0817aab82aee909d3f1a4d82fdec9ec8237eea5e8f3ba707475f137`，controller `17d2e4d8b4cbe5aee4b650a8c5cae2de2b2fac025208cfb0559aa6c2d4a9d88f`。模型 `scene_leg_lift_demo.xml` SHA-256 为 `12286418247d0e240ae131b5ae5c60f3a7a481d4754aefe4517476e937aa05b8`。steps profile SHA-256 为 `4c61c597990b806b1d85dbb9e7f9ce9745f825ca91915bec18aee87923b9a0ab`，varying profile SHA-256 为 `9efcc3b2d89fb349a12990ace1cf6ceb45e0d731deb470bdf2af084d82449d74`。26/26 CTest 通过。

## 运行结果

| 类型 | run | active s | 完整 1 s 窗口 | 未通过 transition | legacy status |
|---|---|---:|---:|---|---|
| steps | `steps_20260913_130149` | 94.804 | 3/5 | 0→1, 1→2 | 全 0 |
| steps | `steps_20260913_130415` | 94.802 | 3/5 | 0→1, 1→2 | 全 0 |
| steps | `steps_20260913_130640` | 94.804 | 3/5 | 0→1, 1→2 | 全 0 |
| varying | `varying_20260913_130859` | 82.802 | 2/5 | 0.6→1.4, 1.4→2.3, 2.3→2.8 | 全 0 |
| varying | `varying_20260913_131059` | 82.804 | 2/5 | 0.6→1.4, 1.4→2.3, 2.3→2.8 | 全 0 |
| varying | `varying_20260913_131301` | 82.804 | 2/5 | 0.6→1.4, 1.4→2.3, 2.3→2.8 | 全 0 |

这里的诊断 audit 要求每个变速点在下一变速点前，存在从某一候选时刻开始连续完整 1.0 s、测量速度落在 `max(0.15, 0.05*max(|target|,1)) m/s` 内的窗口。legacy analyzer 的 `strict_pass` 只检查 status 字段，因此六次均为 `true` 不能覆盖上述 settling 失败。steps 的 2→3 虽最终出现完整窗口，但 settling latency 为 1.298–3.234 s；若使用“latency≤1 s”语义，它也失败。

各 run 的 transition settling latency（秒；`nan` 表示直到该平台结束也没有完整窗口）为：steps r1 `[7.912,nan,2.588,0.114,0.362]`，r2 `[nan,nan,1.298,0.044,0.000]`，r3 `[7.866,nan,3.234,0.000,0.322]`；varying r1 `[nan,nan,nan,0.696,0.002]`，r2 `[nan,nan,nan,0.678,0.002]`，r3 `[nan,nan,nan,0.688,0.002]`。

## 失败 transition 的时间对齐证据

下表为三次重复在 transition 后 `+0.50 s` 的中位数；完整的 `0/0.25/0.50/1.00 s` 对齐值和每个 run 在 `analysis/transition_samples.csv` 中。`step/lift` 是该 SHA 原生 CSV 记录的 runtime gait schedule 值；kernel 内部 step 实际值另见 `kernel_step_events.csv`。`TD` 为 FR/FL/RR/RL touchdown target。

| 类型 transition | req / shaped / applied / measured m/s | period / duty | step / lift m | kernel err m/s | TD FR/FL/RR/RL m | WBC v / req ax / SRBD ax / ID Fx |
|---|---|---|---|---:|---|---|
| steps 0→1 | 1.000 / 1.000 / 0.976 / 1.224 | .140 / .440 | .155 / .050 | .224 | .037/.039/.039/.037 | 0 / -2.353 / -2.353 / -14.0 |
| steps 1→2 | 2.000 / 2.000 / 1.909 / 2.291 | .140 / .440 | .304 / .140 | .378 | .075/.075/.075/.075 | 1.909 / -3.819 / -4.784 / -21.0 |
| steps 2→3 | 3.000 / 3.000 / 2.767 / 3.285 | .140 / .440 | .440 / .197 | .555 | .110/.106/.106/.110 | 2.767 / -4.000 / -4.094 / +20.6 |
| varying 0.6→1.4 | 1.400 / 1.400 / 1.370 / 1.630 | .140 / .440 | .218 / .087 | .267 | .053/.053/.053/.053 | 1.370 / -2.599 / -2.439 / +4.5 |
| varying 1.4→2.3 | 2.300 / 2.300 / 2.169 / 2.631 | .140 / .440 | .345 / .162 | .455 | .087/.083/.083/.087 | 2.169 / -4.000 / -4.954 / -21.9 |
| varying 2.3→2.8 | 2.800 / 2.800 / 2.639 / 3.161 | .140 / .440 | .420 / .192 | .551 | .107/.104/.104/.107 | 2.639 / -4.000 / -2.924 / +23.9 |

所有对齐样本的 `wbc_full_srbd_ok=1`、`wbc_full_id_ok=1`、`wbc_shadow_solver_ok=1`。跨 active rows 的 full SRBD、ID-WBC 和 footstep-plan valid fraction 均为 1.0；shadow budget fraction 为 steps 0.899、varying 0.871–0.871。touchdown x 最大绝对误差为 0.061–0.079 m，y 最大绝对误差为 0.055–0.061 m。

## 假设判定

**A：强支持，但准确说是长期压低 applied，不是修改 shaped。** `request→shaped` 在失败的上升 transition 的首个 1 s 内最大误差为 0；然而 `shaped-applied` 在首个 1 s 已达到 steps 0.15–0.50 m/s、varying 0.20–0.30 m/s。常值平台末 2 s 的正 gap 占比在 steps 的 2/3 m/s 平台为 100%，varying 的 1.4/2.3/2.8 m/s 平台为 100%，均值约 0.032–0.500 m/s。代码中的 measured-speed lead governor 在 shaped>0.90 且 body 已领先时降低 applied；它没有回写 shaped。该偏差在 `shaped→applied` 层最先、最稳定出现。

**B：不支持为首因。** period/duty 在六次运行中均恒为 0.14 s / 0.44；解析 3,684 个 `STEPK` cycle 事件后，schedule step target 与 kernel step actual 的最大差仅 `4.99e-7 m`。因此没有看到 cycle-slew 跟不上 applied 的证据。foot-lift 的 CSV 是 schedule target，基准 SHA 没有单独输出 kernel 内部 lift actual，故该子项保留为观测缺口，不冒充已证伪。

**C：机制存在但因果证据弱。** 固定 period/duty 确实覆盖低中速，且低速通过更短 step/lift 实现；但原始记录没有 period/duty 异常、步长滞后或 planner invalid。由于没有做参数替换，不能把固定 cadence 单独归因；它不是第一处可观测偏差。

**D：不支持为首因。** kernel velocity error 在失败 transition 后上升至约 0.22–0.69 m/s，但它定义为 measured−nominal，时间上跟随 measured 偏离；footstep plan valid=1.0，touchdown 误差仍在 0.061–0.079 m（x）和 0.055–0.061 m（y）范围。Raibert 目标变化与落脚误差不足以解释最早的 shaped→applied gap。

**E：不支持为第一处偏差；不能宣称执行层完全无贡献。** WBC/SRBD/ID status 全部通过，且在 measured 高于 applied 时 WBC 请求的是负向 x 加速度（约 −2.4 至 −4.0 m/s²），方向与减速一致。它更像在响应上游已经存在的 applied/measurement mismatch；shadow budget 非 100% 是后续诊断风险，但没有 solver failure 或 SRBD/ID status failure 作为首因证据。

## 结论、证据强度和下一步

第一处明显偏离上游目标是 `shaped → applied`：measured-speed tracking-lead governor 在上升及高速度平台持续把 applied 压到 shaped 以下；WBC 随后忠实看到较低的 WBC velocity target，并请求负向加速度。证据强度：A 强，B 中高（lift actual 缺口），C 弱/未隔离，D 中高反证，E 中高反证但非完整排除。该结论解释了 steps 与 varying 的共同失败形状，但不等同于已证明最终动力学原因。

获准后的最小单变量实验应只改变 `--velocity-max-tracking-lead`，固定本 SHA、profile、period/duty、其余环境和重复协议，比较 applied-shape gap、完整 1 s coverage 及 measured response；本阶段不实施该实验、不修复、不调参，报告完成后停止等待人工批准。

关键曲线：[`steps_median_trace.svg`](../../example/cpp/experiments/_runs/phase1_velocity_failure_attribution_20260913/analysis/steps_median_trace.svg)、[`varying_median_trace.svg`](../../example/cpp/experiments/_runs/phase1_velocity_failure_attribution_20260913/analysis/varying_median_trace.svg)。原始证据和对齐表：`example/cpp/experiments/_runs/phase1_velocity_failure_attribution_20260913/`。
