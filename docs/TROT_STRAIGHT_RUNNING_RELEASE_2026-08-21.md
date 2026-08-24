# 直线小跑与高速跑态发布记录（2026-08-21）

## 结论

当前发布分成两个可复现实验入口：自然小跑作为稳定基线，低占空比跑态作为更高速配置。两者使用同一套 `--wbc-full`、SRBD-MPC、ID-WBC 和 Raibert 对角步态，只改变步态参考；不是拼接两套关节动作。

这次修复了两个实际问题：WSL 调度抖动会让仿真器与控制器争抢 CPU，造成偶发质量门拒绝；非转向时 MPC 原先没有持续跟踪固定世界航向，长期运行会出现左右摆偏。现在默认自动把 MuJoCo 与控制器分别固定到 CPU 2、3，并在直线参考下给 MPC 一个有界的世界航向/横向位置目标。低占空比模式的接触质量门按 duty=0.45 采用 0.25 的最低接触比例，但姿态、力矩、关节误差和硬安全限仍保持有效。

## 默认配置

自然小跑：`period=0.34`、`duty=0.50`、`step=0.340`、`foot_lift=0.120`、`max_cycles=30`。三次完整复现均无 safety/quality/posture 错误；直线横向 p95 为 3.8–8.5 cm，航向 p95 为 3.8–7.3°。

高速跑态：`period=0.26`、`duty=0.45`、`step=0.320`、`foot_lift=0.120`，实际速度中位数 1.12–1.18 m/s，三次完整复现通过速度、姿态、接触、腾空和急停尾段验收；直线横向 p95 为 4.1–7.9 cm，航向 p95 为 3.2–7.9°。该档是目前已重复验证的速度上限，不把单次更快但偏航明显的探针当作发布结果。

## 复现与验收

```bash
bash example/cpp/scripts/run_natural_trot.sh 100 natural_release_v1 --headless
python3 example/cpp/tools/analysis/analyze_natural_gait.py \
  example/cpp/experiments/_runs/natural_release_v1 \
  --min-cruise-speed 0.80 --min-speed-p05 0.60 --min-cycle-count 30
python3 example/cpp/tools/analysis/analyze_straightness.py \
  example/cpp/experiments/_runs/natural_release_v1

bash example/cpp/scripts/run_running_trot.sh 100 running_release_v1 --headless
python3 example/cpp/tools/analysis/analyze_running_gait.py \
  example/cpp/experiments/_runs/running_release_v1
python3 example/cpp/tools/analysis/analyze_straightness.py \
  example/cpp/experiments/_runs/running_release_v1
```

直线验收相对世界 +X：横向位置 p95≤0.10 m、末端≤0.12 m、航向 p95≤8°、横向速度 p95≤0.25 m/s。跑态还必须有真实腾空相、对角接触同步和急停后 1.5 s 速度 p95≤0.10 m/s。CPU 固定可用 `TROT_CPU_AFFINITY_SIM`、`TROT_CPU_AFFINITY_CTRL` 覆盖，设 `TROT_CPU_AUTOPIN=0` 可关闭。

## 边界记录

步长 0.324 m、周期 0.26 s、直行航向增益 2.2 的三次重复都通过了跑态和直线门，但速度中位数只有 1.119–1.153 m/s，没有形成高于发布档的可重复收益，因此保留为候选证据，不替换发布档。周期 0.255 s 的三次探针中，一次通过、一次接触同步差 0.0002、一次触发周期质量门；周期 0.25 s 在急停交接阶段触发硬姿态保护，均排除。当前可交付高速上限仍是周期 0.26 s、步长 0.320 m，三次中位速度 1.154–1.180 m/s。

最终 GUI 视频为 24 s、480 帧、camera-follow，已抽查起步、稳态和末段关键帧：
go2_natural_trot_straight_release.mp4、go2_running_trot_straight_release.mp4。完整数字表见 [TROT_RELEASE_METRICS_2026-08-21.txt](TROT_RELEASE_METRICS_2026-08-21.txt)。
