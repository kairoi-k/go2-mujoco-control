# Go2 1 m/s 跑态验收（2026-08-21）

## 结论

当前可交付的是低占空比对角跑态（running trot），不是 gallop/pace。它沿用同一套 ID-WBC + SRBD-MPC 植物，只改变步态参考：`period=0.26 s`、`duty=0.45`、`step=0.312 m`、`foot_lift=0.120 m`。MuJoCo 真值的三次重复都通过了严格验收，实际速度中位数为 1.057–1.144 m/s，并出现短暂腾空相（2.7–3.7%）。

## 固定入口

```bash
bash example/cpp/scripts/run_running_trot.sh 100 running_trot_rep1
python3 example/cpp/tools/analysis/analyze_running_gait.py \
  example/cpp/experiments/_runs/running_trot_rep1
```

入口末尾安排 `8.0 s` 的 `emergency_stop`，用于验收跑态减速以及回到四足 WBC 支撑。急停交接先保持原对角接触并刹车，0.45 s 后才让步态核进入 stance blend，0.95 s 后才切换 MPC/WBC 四足接触，避免跑态在腾空相直接跳到四足支撑。

## 验收口径

三次重复必须满足：元数据各状态为 0；至少 25 个 cycle health；无 hard posture、cycle-quality 或 safety rejection；稳态速度 median ≥ 1.00 m/s、p05 ≥ 0.85 m/s、p95 ≤ 1.35 m/s；机身高度 0.33–0.40 m；roll/pitch p95 < 8°；各脚摆动高度 p95 ≥ 0.055 m；四脚同时低位比例 ≤ 0.35；对角接触同步 ≥ 0.75、交叉对角反同步 ≥ 0.70；腾空比例 ≥ 0.02；急停后 1.5 s 的速度 p95 ≤ 0.10 m/s。

## 已通过证据

| Run | speed p05 / median / p95 (m/s) | body angle p95 (deg) | aerial | stop tail | status |
|---|---:|---:|---:|---:|---|
| `running_trot_release_v1` | 0.900 / 1.050 / 1.163 | 2.25 / 1.91 | 0.0305 | 115 | PASS |
| `running_trot_release_v2` | 0.871 / 1.072 / 1.207 | 3.04 / 2.79 | 0.0437 | 115 | PASS |
| `running_trot_release_v3` | 0.923 / 1.104 / 1.220 | 2.89 / 2.10 | 0.0314 | 117 | PASS |

数据在对应 `example/cpp/experiments/_runs/` 目录，分析器是 `example/cpp/tools/analysis/analyze_running_gait.py`。低占空比 `duty=0.40` 的探索记录保留在 `_runs`，但因偶发 cycle-quality/姿态失败，没有进入默认交付配置。

## 边界

这证明的是平地 MuJoCo 中约 1 m/s 的低占空比对角跑态与安全急停交接；尚未声称 gallop/pace、复杂地形或超过约 1.2 m/s 的稳定跑态。下一阶段应在此固定配置上比较 pace/bound/gallop，并分别做速度、落足冲击和扰动鲁棒性验收。
