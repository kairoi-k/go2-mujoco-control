# Go2 WBC/MPC >3 m/s bounded sprint acceptance

这是一套独立的高速验收 profile，不改变普通小跑默认参数，也不进入地形适应 worktree。跑态仍由同一套 locomotion kernel + SRBD-MPC + ID-WBC 产生；达到实测 `vx >= 3.0 m/s` 且姿态合格后，控制器在跑态内渐减速度，速度和姿态满足门限后才切入四足 WBC 保持，最后在保持态完成。

## 可复现命令

```bash
cd /home/che/dev/go2-mujoco-control
source example/cpp/configs/sprint_3mps_wbc_full_2026-08-21.env
bash example/cpp/scripts/run_trot.sh 24 sprint_3mps_acceptance_01 \
  --headless --forever --wbc-full --gait-pattern diagonal-trot \
  --tau-limit 45 --period 0.14 --duty 0.44 --step-length 0.48 \
  --foot-lift 0.18 --kernel raibert-trot \
  --raibert-velocity-gain 0.015 --raibert-max-adjustment 0.08 \
  --controller-duration 20 --domain-id 230
```

批量汇总：

```bash
python3 example/cpp/tools/analysis/summarize_high_speed_acceptance.py \
  example/cpp/experiments/_runs/hs27_height34_speedstop_03 \
  example/cpp/experiments/_runs/hs27_height34_speedstop_04 \
  example/cpp/experiments/_runs/hs27_height34_speedstop_06
```

## 已通过的三次独立复现

| run | 峰值速度 | `vx≥3` 且姿态≤10° | 高速窗口姿态 P95 | 收尾 |
|---|---:|---:|---:|---|
| `hs27_height34_speedstop_03` | 3.312 m/s | 0.964 s | 2.685° | 四足 WBC 保持 |
| `hs27_height34_speedstop_04` | 3.511 m/s | 0.960 s | 2.233° | 四足 WBC 保持 |
| `hs27_height34_speedstop_06` | 3.560 m/s | 1.016 s | 2.550° | 四足 WBC 保持 |

三次均无 hard-safety/posture 触发，最终速度约 0.002 m/s、姿态误差小于 0.32°，`dynamics_status=0`。验收脚本结果为 `passed=3/3`。

`docs/media/sprint_3mps_wbc_full_2026-08-21.mp4` 是便于汇报的 GUI 画面＋同步速度曲线，原始画面另存为 `sprint_3mps_wbc_full_raw_2026-08-21.mp4`；视频只作视觉演示，表中的定量结论来自上面三次独立 headless 复现。

## 判定与边界

通过标准是：实测速度峰值至少 3 m/s；连续高速窗口至少 0.60 s；高速窗口姿态 P95 不超过 10°；之后完成同一控制植物内的刹车、四足 WBC 保持，最终速度不超过 0.15 m/s、姿态不超过 10°；无硬安全触发，动力学闭合检查通过。

这是“可复现的有界高速冲刺＋受控收尾”，不是宣称无限时长的 3 m/s 稳态，也没有把地形适应能力混入本验收。下一阶段若要追求更长高速窗口，应单独优化跑态支撑裕量，不改变这套已通过的收尾链。
