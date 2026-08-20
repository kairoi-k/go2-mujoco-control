# Reactive WBC/MPC 实验索引（2026-08-19）

所有结果都使用 `real_trot_go2 --wbc-full`、Raibert trot、12 个周期、14 s 控制时长；`0/0/0` 依次表示 controller/safety/quality 状态。

| 实验目录 | 事件/扰动 | 结果 | 关键证据 |
|---|---|---|---|
| `go2_reactive_baseline_final_2026-08-19` | 无事件回归 | `0/0/0` | max roll 1.27°，pitch 9.27° |
| `go2_reactive_event_final_2026-08-19` | turn、急停、障碍、slip、impact | `0/0/0` | 事件在 CSV 中按脚本顺序出现，约 2 ms 级更新 |
| `go2_reactive_random_final_2026-08-19` | 确定性混合事件序列 | `0/0/0` | max roll 1.19°，pitch 9.25° |
| `go2_reactive_low_friction_final_2026-08-19` | μ=1→0.05→1 | `0/0/0` | `simulator.log` 有 8.002/9.002 s active/restored |
| `go2_reactive_low_friction_push08_2026-08-19` | 低摩擦 + 0.8 m/s 冲击 | `0/0/0` | 自动 impact，max roll 7.75° |
| `go2_reactive_auto_push_2026-08-19` | 1.5 m/s 冲击 | safety=`1` | 失败边界：roll 105.89°、pitch 77.13° |

原始 `data.csv`、`contact_ground_truth.csv`、`controller.log`、`simulator.log` 和 `run_metadata.txt` 保留在各目录；汇总命令：

```bash
python3 example/cpp/tools/analyze_reactive_events.py \
  example/cpp/experiments/go2_reactive_baseline_final_2026-08-19 \
  example/cpp/experiments/go2_reactive_event_final_2026-08-19 \
  example/cpp/experiments/go2_reactive_random_final_2026-08-19 \
  example/cpp/experiments/go2_reactive_low_friction_push08_2026-08-19
```

设计说明见 [`docs/REACTIVE_ENVIRONMENT_ADAPTATION.md`](../../docs/REACTIVE_ENVIRONMENT_ADAPTATION.md)。
