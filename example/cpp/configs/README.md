# 序列配置

供 `run_leg_sequence.sh` / `real_leg_lift_go2 --sequence-file` 使用。

典型正式配置：

- `go2_four_step_fr_rl_fl_rr.txt` — 对角四步正式候选

格式见加载函数 `LoadStepSequence`（`leg_lift_types.h`）：每步指定抬腿、重心偏移、抬脚高、摆动与机身前进等。

跑法示例：

```bash
bash example/cpp/scripts/run_leg_sequence.sh 60 \
  go2_four_step_diagonal_rrx10_2026-08-02 \
  example/cpp/configs/go2_four_step_fr_rl_fl_rr.txt
```
