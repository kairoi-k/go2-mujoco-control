# EXP-20：世界位置 + yaw 闭环反馈

这是 2026-08-02 对 Go2 长序列航向反馈的正式摘要。完整的计划目标、问题、解决路径和历史对照见 `docs/experiment-ledger.md` 的 EXP-20。

## 结果

| 序列 | 反向差分 yaw | 终点平面误差 | 最大 yaw 误差 | 判定 |
|---:|---:|---:|---:|---|
| 8 步 | `1.209°` | `2.107 mm` | `2.271°` | 通过 |
| 24 步 | `2.183°` | `2.867 mm` | `2.997°` | 通过 |
| 32 步 | `2.321°` | `2.874 mm` | `3.084°` | 通过 |
| 32 步重复 | `2.320°` | `2.876 mm` | `3.083°` | 通过 |

32 步首次/重复运行均完成 `32/32` 步，终点位置和周期末指标几乎重合；最大支撑漂移分别为 `13.822/13.828 mm`，仍接近 `15 mm` 门槛。

## 实现

通过 `--world-feedback --yaw-feedback` 启用。控制器以自然稳定后的世界 yaw 为参考，计算归一化航向误差；x/y 目标先变换到参考世界坐标再反馈回机身坐标。yaw 修正由有界的横向机身补偿和左右脚相反的前后摆腿差分组成，差分落脚位置会写入足端记忆。

## 复现入口

```bash
bash example/cpp/scripts/run_leg_sequence.sh 380 \
  go2_32_step_yaw_swing_feedback_reverse_probe_2026-08-02 \
  example/cpp/configs/go2_32_step_continuation_060_rr_y0.txt \
  --world-feedback --yaw-feedback
```

原始运行目录（含本机保留的 `data.csv` 和 `simulator.log`）位于：
`example/cpp/experiments/_archive/2026-08-02_sequence-development/yaw-feedback-probes/`。

## 边界

这证明了 32 步闭环的可重复性，不证明 64/100 步、趴下状态机或真机安全性。第 24 步曾出现单个接触采样的 `min_support_contacts=1`，但周期达标比例为 `0.99977`，因此按 98% 采样门禁通过；该现象已在台账中保留。
